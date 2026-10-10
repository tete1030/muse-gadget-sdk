/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "muse_lcd_bands.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "muse_mem.h"
#include "muse_rotation_pixels.h"

typedef struct {
    const uint8_t *data;   /* NULL: run s_call instead */
    int x1, y1, x2, y2;
    unsigned rotation;
} band_t;

static const char *TAG = "lcd_bands";

static lv_display_t *s_disp;
static esp_lcd_panel_handle_t s_panel;
static QueueHandle_t s_bands;            /* room for LVGL's band and a call */
static SemaphoreHandle_t s_call_lock;
static SemaphoreHandle_t s_call_done;
static void (*s_call)(void *);
static void *s_call_arg;
static SemaphoreHandle_t s_chunk_free;   /* internal buffers not on the wire */
static uint8_t *s_chunk[2];
static size_t s_chunk_bytes;
static int s_width, s_height;
static unsigned s_rotation;
static int s_chunks_out;                 /* of the band being sent */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/* A piece has gone, or failed to; true if it was the band's last. */
static bool IRAM_ATTR chunk_done(void)
{
    portENTER_CRITICAL_SAFE(&s_lock);
    bool last = s_chunks_out > 0 && --s_chunks_out == 0;
    portEXIT_CRITICAL_SAFE(&s_lock);
    return last;
}

static bool IRAM_ATTR on_chunk_sent(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *edata, void *ctx)
{
    (void)io;
    (void)edata;
    (void)ctx;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_chunk_free, &woken);
    bool yield = chunk_done() && esp_lv_adapter_display_notify_color_trans_done_from_isr(s_disp);
    return yield || woken == pdTRUE;
}

static esp_err_t queue_band(lv_display_t *disp, esp_lcd_panel_handle_t panel, int x1, int y1, int x2, int y2,
                            const void *data, void *ctx)
{
    (void)disp;
    (void)panel;
    (void)ctx;
    band_t b = { data, x1, y1, x2, y2, s_rotation };
    return xQueueSend(s_bands, &b, 0) == pdTRUE ? ESP_OK : ESP_FAIL;
}

/*
 * The panel's draw waits for the piece before to finish, so each piece is
 * copied while the one before is on the wire. Even row counts keep every
 * piece's window starting on an even row, which the CO5300 needs.
 */
static void send_bands(void *arg)
{
    (void)arg;
    int k = 0;
    band_t b;
    for (;;) {
        xQueueReceive(s_bands, &b, portMAX_DELAY);
        if (!b.data) {
            /* A queued band may have its final DMA still in flight. Do not
             * change coordinates or panel state until both buffers are free. */
            xSemaphoreTake(s_chunk_free, portMAX_DELAY);
            xSemaphoreTake(s_chunk_free, portMAX_DELAY);
            s_call(s_call_arg);
            xSemaphoreGive(s_chunk_free);
            xSemaphoreGive(s_chunk_free);
            xSemaphoreGive(s_call_done);
            continue;
        }
        int x1 = b.x1, y1 = b.y1, x2 = b.x2, y2 = b.y2;
        if (b.rotation && !muse_rotation_rect(s_width, b.rotation, b.x1, b.y1, b.x2, b.y2,
                                              &x1, &y1, &x2, &y2)) {
            lv_display_flush_ready(s_disp);
            continue;
        }
        size_t row = (size_t)(x2 - x1) * 2;
        int rows = s_chunk_bytes / row & ~1;
        /* LVGL sends no more until this band is done, so nothing is still
         * counting down. */
        s_chunks_out = (y2 - y1 + rows - 1) / rows;
        for (int y = y1; y < y2; y += rows) {
            int h = y2 - y < rows ? y2 - y : rows;
            xSemaphoreTake(s_chunk_free, portMAX_DELAY);
            bool copied = true;
            if (!b.rotation) {
                memcpy(s_chunk[k], b.data + (size_t)(y - b.y1) * row, h * row);
            } else {
                copied = muse_rotation_copy_rows(b.data, b.x1, b.y1, b.x2, b.y2,
                                                 s_width, b.rotation, y, h, s_chunk[k]);
            }
            esp_err_t err = copied ? esp_lcd_panel_draw_bitmap(s_panel, x1, y, x2, y + h, s_chunk[k]) : ESP_FAIL;
            if (err != ESP_OK) {
                xSemaphoreGive(s_chunk_free);
                if (chunk_done()) {
                    lv_display_flush_ready(s_disp);
                }
                /* A failed submission breaks the otherwise FIFO k0/k1
                 * alternation: the next index might still be in flight.
                 * Drain both tokens before reusing either buffer. */
                xSemaphoreTake(s_chunk_free, portMAX_DELAY);
                xSemaphoreTake(s_chunk_free, portMAX_DELAY);
                xSemaphoreGive(s_chunk_free);
                xSemaphoreGive(s_chunk_free);
            }
            k ^= 1;
        }
    }
}

lv_display_t *muse_lcd_bands_register(esp_lv_adapter_display_config_t cfg, int lines, size_t chunk_bytes)
{
    s_panel = cfg.panel;
    s_width = cfg.profile.hor_res;
    s_height = cfg.profile.ver_res;
    s_chunk_bytes = chunk_bytes;
    if (xTaskGetCoreID(xTaskGetCurrentTaskHandle()) != MUSE_UI_CORE) {
        ESP_LOGE(TAG, "not pinned to core %d: the panel's SPI interrupt can strand the send task", MUSE_UI_CORE);
    }
    s_bands = xQueueCreate(2, sizeof(band_t));
    s_call_lock = xSemaphoreCreateMutex();
    s_call_done = xSemaphoreCreateBinary();
    s_chunk_free = xSemaphoreCreateCounting(2, 2);
    s_chunk[0] = heap_caps_malloc(chunk_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    s_chunk[1] = heap_caps_malloc(chunk_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_bands || !s_call_lock || !s_call_done || !s_chunk_free || !s_chunk[0] || !s_chunk[1] ||
        xTaskCreatePinnedToCore(send_bands, "lcd_send", 2560, NULL, MUSE_UI_PRIORITY + 1, NULL, MUSE_UI_CORE) != pdPASS) {
        return NULL;
    }
    const esp_lcd_panel_io_callbacks_t io_cbs = { .on_color_trans_done = on_chunk_sent };
    esp_lcd_panel_io_register_event_callbacks(cfg.panel_io, &io_cbs, NULL);
    esp_lv_adapter_set_default_display_idf_callback_registration_enabled(false);

    cfg.profile.buffer_height = lines;
    cfg.profile.use_psram = true;
    cfg.profile.require_double_buffer = true;
    s_disp = esp_lv_adapter_register_display(&cfg);
    if (!s_disp) {
        return NULL;
    }
    lv_display_set_color_format(s_disp, LV_COLOR_FORMAT_RGB565_SWAPPED);
    const esp_lv_adapter_draw_bitmap_callbacks_t draw_cbs = { .custom_draw_bitmap = queue_band };
    esp_lv_adapter_set_draw_bitmap_callbacks(s_disp, &draw_cbs, NULL);
    return s_disp;
}

void muse_lcd_bands_run(void (*fn)(void *arg), void *arg)
{
    if (!s_bands) {
        fn(arg);
        return;
    }
    xSemaphoreTake(s_call_lock, portMAX_DELAY);
    s_call = fn;
    s_call_arg = arg;
    const band_t call = { 0 };
    xQueueSend(s_bands, &call, portMAX_DELAY);
    xSemaphoreTake(s_call_done, portMAX_DELAY);
    xSemaphoreGive(s_call_lock);
}

static void set_rotation(void *arg)
{
    s_rotation = *(unsigned *)arg;
}

bool muse_lcd_bands_set_rotation(unsigned quarter_turns)
{
    if (quarter_turns > 3 || s_width != s_height || !s_bands) return false;
    muse_lcd_bands_run(set_rotation, &quarter_turns);
    return true;
}

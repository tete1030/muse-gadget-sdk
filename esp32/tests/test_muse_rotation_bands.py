# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run production LCD bands with pthread-backed FreeRTOS and delayed DMA.

The complete bands implementation and real pixel-rotation implementation are
compiled. Only platform dependencies are mocked. FIFO color-done callbacks
are explicitly released by the test, and DMA buffers are checked until then.
This is host concurrency coverage, not a test of the panel or ESP-IDF driver.
"""

from __future__ import annotations

import os
import re
import shlex
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FAKE_ADAPTER = r'''
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
typedef struct { int unused; } lv_display_t;
typedef void *esp_lcd_panel_handle_t;
typedef void *esp_lcd_panel_io_handle_t;
typedef struct { int unused; } esp_lcd_panel_io_event_data_t;
typedef struct {
    bool (*on_color_trans_done)(esp_lcd_panel_io_handle_t,
                               esp_lcd_panel_io_event_data_t *, void *);
} esp_lcd_panel_io_callbacks_t;
typedef struct {
    esp_lcd_panel_handle_t panel;
    esp_lcd_panel_io_handle_t panel_io;
    struct {
        int hor_res, ver_res, buffer_height;
        bool use_psram, require_double_buffer;
    } profile;
} esp_lv_adapter_display_config_t;
typedef struct {
    esp_err_t (*custom_draw_bitmap)(lv_display_t *, esp_lcd_panel_handle_t,
                                   int, int, int, int, const void *, void *);
} esp_lv_adapter_draw_bitmap_callbacks_t;
#define LV_COLOR_FORMAT_RGB565_SWAPPED 1
'''
MOCKS = r'''
#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "muse_lcd_bands.h"
#include "muse_rotation_pixels.h"

#define IRAM_ATTR
#define MUSE_UI_CORE 1
#define MUSE_UI_PRIORITY 5
#define MALLOC_CAP_DMA 1
#define MALLOC_CAP_INTERNAL 2
#define ESP_LOGE(tag, ...) ((void)(tag))
#define pdFALSE 0
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY -1
typedef int BaseType_t;
typedef pthread_mutex_t portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED PTHREAD_MUTEX_INITIALIZER
#define portENTER_CRITICAL_SAFE(lock) assert(pthread_mutex_lock(lock) == 0)
#define portEXIT_CRITICAL_SAFE(lock) assert(pthread_mutex_unlock(lock) == 0)

typedef struct {
    pthread_mutex_t lock;
    pthread_cond_t changed;
    unsigned count, maximum, waiters;
} fake_semaphore_t;
typedef fake_semaphore_t *SemaphoreHandle_t;

static SemaphoreHandle_t xSemaphoreCreateCounting(unsigned maximum, unsigned count)
{
    SemaphoreHandle_t semaphore = calloc(1, sizeof(*semaphore));
    assert(semaphore && count <= maximum);
    assert(pthread_mutex_init(&semaphore->lock, NULL) == 0);
    assert(pthread_cond_init(&semaphore->changed, NULL) == 0);
    semaphore->count = count;
    semaphore->maximum = maximum;
    return semaphore;
}
static SemaphoreHandle_t xSemaphoreCreateMutex(void) { return xSemaphoreCreateCounting(1, 1); }
static SemaphoreHandle_t xSemaphoreCreateBinary(void) { return xSemaphoreCreateCounting(1, 0); }
static BaseType_t xSemaphoreTake(SemaphoreHandle_t semaphore, int wait)
{
    assert(pthread_mutex_lock(&semaphore->lock) == 0);
    while (!semaphore->count) {
        if (wait == 0) { pthread_mutex_unlock(&semaphore->lock); return pdFALSE; }
        ++semaphore->waiters;
        assert(pthread_cond_wait(&semaphore->changed, &semaphore->lock) == 0);
        --semaphore->waiters;
    }
    --semaphore->count;
    assert(pthread_mutex_unlock(&semaphore->lock) == 0);
    return pdTRUE;
}
static BaseType_t xSemaphoreGive(SemaphoreHandle_t semaphore)
{
    assert(pthread_mutex_lock(&semaphore->lock) == 0);
    assert(semaphore->count < semaphore->maximum);
    ++semaphore->count;
    assert(pthread_cond_signal(&semaphore->changed) == 0);
    assert(pthread_mutex_unlock(&semaphore->lock) == 0);
    return pdTRUE;
}
static BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t semaphore, BaseType_t *woken)
{
    *woken = pdFALSE;
    return xSemaphoreGive(semaphore);
}
static unsigned semaphore_count(SemaphoreHandle_t semaphore)
{
    assert(pthread_mutex_lock(&semaphore->lock) == 0);
    unsigned count = semaphore->count;
    assert(pthread_mutex_unlock(&semaphore->lock) == 0);
    return count;
}
static unsigned semaphore_waiters(SemaphoreHandle_t semaphore)
{
    assert(pthread_mutex_lock(&semaphore->lock) == 0);
    unsigned waiters = semaphore->waiters;
    assert(pthread_mutex_unlock(&semaphore->lock) == 0);
    return waiters;
}

typedef struct {
    pthread_mutex_t lock;
    pthread_cond_t readable, writable;
    size_t size;
    unsigned count, capacity, head;
    uint8_t *data;
} fake_queue_t;
typedef fake_queue_t *QueueHandle_t;
static QueueHandle_t xQueueCreate(unsigned capacity, unsigned size)
{
    QueueHandle_t queue = calloc(1, sizeof(*queue));
    assert(queue);
    queue->data = calloc(capacity, size);
    assert(queue->data);
    queue->size = size;
    queue->capacity = capacity;
    assert(pthread_mutex_init(&queue->lock, NULL) == 0);
    assert(pthread_cond_init(&queue->readable, NULL) == 0);
    assert(pthread_cond_init(&queue->writable, NULL) == 0);
    return queue;
}
static BaseType_t xQueueSend(QueueHandle_t queue, const void *item, int wait)
{
    assert(pthread_mutex_lock(&queue->lock) == 0);
    while (queue->count == queue->capacity) {
        if (wait == 0) { pthread_mutex_unlock(&queue->lock); return pdFALSE; }
        assert(pthread_cond_wait(&queue->writable, &queue->lock) == 0);
    }
    unsigned tail = (queue->head + queue->count) % queue->capacity;
    memcpy(queue->data + tail * queue->size, item, queue->size);
    ++queue->count;
    assert(pthread_cond_signal(&queue->readable) == 0);
    assert(pthread_mutex_unlock(&queue->lock) == 0);
    return pdTRUE;
}
static BaseType_t xQueueReceive(QueueHandle_t queue, void *item, int wait)
{
    (void)wait;
    assert(pthread_mutex_lock(&queue->lock) == 0);
    while (!queue->count) assert(pthread_cond_wait(&queue->readable, &queue->lock) == 0);
    memcpy(item, queue->data + queue->head * queue->size, queue->size);
    queue->head = (queue->head + 1) % queue->capacity;
    --queue->count;
    assert(pthread_cond_signal(&queue->writable) == 0);
    assert(pthread_mutex_unlock(&queue->lock) == 0);
    return pdTRUE;
}

typedef void *TaskHandle_t;
static TaskHandle_t xTaskGetCurrentTaskHandle(void) { return NULL; }
static int xTaskGetCoreID(TaskHandle_t task) { (void)task; return MUSE_UI_CORE; }
typedef struct { void (*fn)(void *); void *arg; } fake_task_t;
static void *task_entry(void *raw)
{
    fake_task_t task = *(fake_task_t *)raw;
    free(raw);
    task.fn(task.arg);
    return NULL;
}
static BaseType_t xTaskCreatePinnedToCore(void (*fn)(void *), const char *name,
                                        unsigned stack, void *arg, unsigned priority,
                                        TaskHandle_t *out, int core)
{
    (void)name; (void)stack; (void)priority; (void)out;
    assert(core == MUSE_UI_CORE);
    fake_task_t *task = malloc(sizeof(*task));
    assert(task);
    *task = (fake_task_t){ fn, arg };
    pthread_t thread;
    assert(pthread_create(&thread, NULL, task_entry, task) == 0);
    assert(pthread_detach(thread) == 0);
    return pdPASS;
}
static void *heap_caps_malloc(size_t size, unsigned caps)
{
    assert(caps == (MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    void *result = malloc(size);
    assert(result);
    memset(result, 0xCC, size);
    return result;
}

typedef struct {
    int x1, y1, x2, y2;
    size_t bytes;
    const uint8_t *buffer;
    uint8_t expected[256];
    bool pending, failed;
} draw_record_t;
static pthread_mutex_t monitor = PTHREAD_MUTEX_INITIALIZER;
static draw_record_t draws[32];
static unsigned draw_count, pending_count, flush_count, isr_flush_count, task_flush_count;
static unsigned fail_mask, control_done, callback_count;
static int callback_arguments[8];
static lv_display_t display;
static bool default_callbacks_disabled;
static esp_lcd_panel_io_callbacks_t color_callbacks;
static esp_lv_adapter_draw_bitmap_callbacks_t draw_callbacks;

static void assert_dma_intact_locked(void)
{
    for (unsigned index = 0; index < draw_count; ++index) {
        if (draws[index].pending && memcmp(draws[index].buffer, draws[index].expected, draws[index].bytes) != 0) {
            fprintf(stderr, "DMA buffer overwritten before color-done callback: draw %u\n", index);
            abort();
        }
    }
}
static esp_err_t esp_lcd_panel_draw_bitmap(esp_lcd_panel_handle_t panel,
                                          int x1, int y1, int x2, int y2, const void *data)
{
    assert(panel == (void *)1);
    assert(pthread_mutex_lock(&monitor) == 0);
    assert_dma_intact_locked();
    assert(draw_count < 32);
    draw_record_t *record = &draws[draw_count];
    record->x1 = x1; record->y1 = y1; record->x2 = x2; record->y2 = y2;
    record->bytes = (size_t)(x2 - x1) * (size_t)(y2 - y1) * 2;
    record->buffer = data;
    assert(record->bytes <= sizeof(record->expected));
    memcpy(record->expected, data, record->bytes);
    record->failed = (fail_mask & (1u << draw_count)) != 0;
    if (!record->failed) {
        for (unsigned index = 0; index < draw_count; ++index) {
            assert(!draws[index].pending || draws[index].buffer != data);
        }
        record->pending = true;
        ++pending_count;
        assert(pending_count <= 2);
    }
    ++draw_count;
    esp_err_t result = record->failed ? ESP_FAIL : ESP_OK;
    assert(pthread_mutex_unlock(&monitor) == 0);
    return result;
}
static esp_err_t esp_lcd_panel_io_register_event_callbacks(esp_lcd_panel_io_handle_t io,
                                                          const esp_lcd_panel_io_callbacks_t *callbacks,
                                                          void *context)
{
    assert(io == (void *)2 && context == NULL);
    color_callbacks = *callbacks;
    return ESP_OK;
}
static void esp_lv_adapter_set_default_display_idf_callback_registration_enabled(bool enabled)
{
    default_callbacks_disabled = !enabled;
}
static lv_display_t *esp_lv_adapter_register_display(const esp_lv_adapter_display_config_t *cfg)
{
    assert(cfg->profile.buffer_height == 4 && cfg->profile.use_psram && cfg->profile.require_double_buffer);
    return &display;
}
static void lv_display_set_color_format(lv_display_t *disp, int format)
{
    assert(disp == &display && format == LV_COLOR_FORMAT_RGB565_SWAPPED);
}
static esp_err_t esp_lv_adapter_set_draw_bitmap_callbacks(lv_display_t *disp,
                                                         const esp_lv_adapter_draw_bitmap_callbacks_t *callbacks,
                                                         void *context)
{
    assert(disp == &display && context == NULL);
    draw_callbacks = *callbacks;
    return ESP_OK;
}
static bool esp_lv_adapter_display_notify_color_trans_done_from_isr(lv_display_t *disp)
{
    assert(disp == &display);
    assert(pthread_mutex_lock(&monitor) == 0);
    ++flush_count; ++isr_flush_count;
    assert(pthread_mutex_unlock(&monitor) == 0);
    return true;
}
static void lv_display_flush_ready(lv_display_t *disp)
{
    assert(disp == &display);
    assert(pthread_mutex_lock(&monitor) == 0);
    ++flush_count; ++task_flush_count;
    assert(pthread_mutex_unlock(&monitor) == 0);
}
'''
HARNESS = r'''
static uint8_t source[128];

static double monotonic_seconds(void)
{
    struct timespec now;
    assert(clock_gettime(CLOCK_MONOTONIC, &now) == 0);
    return (double)now.tv_sec + (double)now.tv_nsec / 1e9;
}
static unsigned read_metric(unsigned metric)
{
    assert(pthread_mutex_lock(&monitor) == 0);
    unsigned value = metric == 0 ? draw_count : metric == 1 ? pending_count :
                     metric == 2 ? flush_count : control_done;
    assert(pthread_mutex_unlock(&monitor) == 0);
    return value;
}
static void wait_metric(unsigned metric, unsigned expected)
{
    double deadline = monotonic_seconds() + 2;
    while (read_metric(metric) != expected) {
        if (monotonic_seconds() >= deadline) {
            fprintf(stderr, "Timeout waiting for metric %u == %u (got %u)\n", metric, expected, read_metric(metric));
            abort();
        }
        struct timespec pause = { 0, 1000000 };
        nanosleep(&pause, NULL);
    }
}
static void wait_chunk_blocked(void)
{
    double deadline = monotonic_seconds() + 2;
    while (!semaphore_waiters(s_chunk_free) || semaphore_count(s_chunk_free) != 0) {
        assert(monotonic_seconds() < deadline);
        struct timespec pause = { 0, 1000000 };
        nanosleep(&pause, NULL);
    }
}
static int outstanding_chunks(void)
{
    portENTER_CRITICAL_SAFE(&s_lock);
    int count = s_chunks_out;
    portEXIT_CRITICAL_SAFE(&s_lock);
    return count;
}
static void complete_one(void)
{
    assert(pthread_mutex_lock(&monitor) == 0);
    assert_dma_intact_locked();
    unsigned index;
    for (index = 0; index < draw_count && !draws[index].pending; ++index) {}
    assert(index < draw_count);
    draws[index].pending = false;
    --pending_count;
    assert(pthread_mutex_unlock(&monitor) == 0);
    assert(color_callbacks.on_color_trans_done != NULL);
    color_callbacks.on_color_trans_done((void *)2, NULL, NULL);
}
static void register_display(size_t chunk_bytes, int height)
{
    esp_lv_adapter_display_config_t cfg = { 0 };
    cfg.panel = (void *)1;
    cfg.panel_io = (void *)2;
    cfg.profile.hor_res = 8;
    cfg.profile.ver_res = height;
    assert(muse_lcd_bands_register(cfg, 4, chunk_bytes) == &display);
    assert(default_callbacks_disabled);
    assert(draw_callbacks.custom_draw_bitmap != NULL);
}
static void queue_source(int x1, int y1, int x2, int y2)
{
    unsigned offset = 0;
    for (int y = y1; y < y2; ++y) {
        for (int x = x1; x < x2; ++x) {
            source[offset++] = (uint8_t)(x + 16 * y);
            source[offset++] = (uint8_t)(0xA5 ^ (x + 16 * y));
        }
    }
    assert(offset <= sizeof(source));
    assert(draw_callbacks.custom_draw_bitmap(&display, (void *)1,
                                            x1, y1, x2, y2, source, NULL) == ESP_OK);
}
static void assert_pixels(unsigned quarter)
{
    assert(pthread_mutex_lock(&monitor) == 0);
    for (unsigned index = 0; index < draw_count; ++index) {
        draw_record_t *record = &draws[index];
        size_t byte = 0;
        for (int py = record->y1; py < record->y2; ++py) {
            for (int px = record->x1; px < record->x2; ++px) {
                /* Independent inverse formula, rather than production helpers. */
                int x = quarter == 1 ? py : quarter == 2 ? 7 - px : quarter == 3 ? 7 - py : px;
                int y = quarter == 1 ? 7 - px : quarter == 2 ? 7 - py : quarter == 3 ? px : py;
                uint8_t pixel = (uint8_t)(x + 16 * y);
                assert(record->expected[byte++] == pixel);
                assert(record->expected[byte++] == (uint8_t)(0xA5 ^ pixel));
            }
        }
        assert(byte == record->bytes);
    }
    assert(pthread_mutex_unlock(&monitor) == 0);
}
static void assert_idle(unsigned ready)
{
    wait_metric(2, ready);
    assert(read_metric(1) == 0);
    assert(outstanding_chunks() == 0);
    /* A failure-recovery drain can still own tokens when the final successful
     * DMA callback notifies LVGL. Wait for recovery to return both tokens. */
    double deadline = monotonic_seconds() + 2;
    while (semaphore_count(s_chunk_free) != 2) {
        assert(monotonic_seconds() < deadline);
        struct timespec pause = { 0, 1000000 };
        nanosleep(&pause, NULL);
    }
}
typedef struct { int argument; bool rotation; } control_request_t;
static void control_callback(void *raw)
{
    assert(read_metric(1) == 0);
    if (s_bands) assert(semaphore_count(s_chunk_free) == 0);
    assert(pthread_mutex_lock(&monitor) == 0);
    assert(callback_count < 8);
    callback_arguments[callback_count++] = *(int *)raw;
    assert(pthread_mutex_unlock(&monitor) == 0);
}
static void *run_control(void *raw)
{
    control_request_t *request = raw;
    if (request->rotation) {
        assert(muse_lcd_bands_set_rotation((unsigned)request->argument));
    } else {
        muse_lcd_bands_run(control_callback, &request->argument);
    }
    assert(pthread_mutex_lock(&monitor) == 0);
    ++control_done;
    assert(pthread_mutex_unlock(&monitor) == 0);
    return NULL;
}
static pthread_t start_control(control_request_t *request)
{
    pthread_t thread;
    assert(pthread_create(&thread, NULL, run_control, request) == 0);
    return thread;
}
static void join_control(pthread_t thread, unsigned expected)
{
    wait_metric(3, expected);
    assert(pthread_join(thread, NULL) == 0);
}
static void test_original(void)
{
    register_display(16, 8);
    queue_source(1, 1, 5, 7);
    wait_metric(0, 2);
    wait_chunk_blocked();
    assert(read_metric(1) == 2 && read_metric(2) == 0);
    assert(outstanding_chunks() == 3);
    assert(draws[0].buffer != draws[1].buffer);
    complete_one();
    wait_metric(0, 3);
    assert(draws[0].buffer == draws[2].buffer);
    assert(read_metric(1) == 2 && read_metric(2) == 0);
    complete_one();
    assert(read_metric(2) == 0);
    complete_one();
    assert_idle(1);
    for (unsigned index = 0; index < 3; ++index) {
        assert(draws[index].x1 == 1 && draws[index].x2 == 5);
        assert(draws[index].y1 == 1 + (int)index * 2 && draws[index].y2 == 3 + (int)index * 2);
    }
    assert_pixels(0);
    assert(isr_flush_count == 1 && task_flush_count == 0);
}
static void test_rotated(unsigned quarter)
{
    register_display(24, 8);
    assert(muse_lcd_bands_set_rotation(quarter));
    queue_source(0, 1, 4, 7);
    wait_metric(0, 2);
    assert(read_metric(1) == 2 && read_metric(2) == 0);
    complete_one();
    assert(read_metric(2) == 0);
    complete_one();
    assert_idle(1);
    int y1 = quarter == 1 ? 0 : 4;
    for (unsigned index = 0; index < 2; ++index) {
        assert(draws[index].x1 == 1 && draws[index].x2 == 7);
        assert(draws[index].y1 == y1 + (int)index * 2 && draws[index].y2 == y1 + 2 + (int)index * 2);
    }
    assert_pixels(quarter);
}
static void test_barrier(bool rotation)
{
    register_display(24, 8);
    queue_source(2, 2, 6, 6);
    wait_metric(0, 2);
    assert(read_metric(1) == 2);
    control_request_t request = { rotation ? 1 : 7, rotation };
    pthread_t thread = start_control(&request);
    wait_chunk_blocked();
    assert(read_metric(3) == 0 && callback_count == 0 && s_rotation == 0);
    complete_one();
    wait_chunk_blocked();
    assert(read_metric(1) == 1 && read_metric(3) == 0 && callback_count == 0 && s_rotation == 0);
    complete_one();
    join_control(thread, 1);
    assert_idle(1);
    if (rotation) {
        assert(s_rotation == 1 && callback_count == 0);
    } else {
        assert(callback_count == 1 && callback_arguments[0] == 7 && s_rotation == 0);
    }
    /* Both barrier tokens are returned, so another call cannot deadlock. */
    int argument = 9;
    muse_lcd_bands_run(control_callback, &argument);
    assert(callback_arguments[callback_count - 1] == 9);
    assert(semaphore_count(s_chunk_free) == 2);
}
static void test_failures(unsigned mask)
{
    register_display(16, 8);
    fail_mask = mask;
    queue_source(1, 1, 5, 7);
    if (mask == 2 || mask == 4) {
        wait_metric(0, 2);
        wait_chunk_blocked();
        /* The middle failed submission must retain the first buffer until
         * its DMA callback. A final failed submission follows two successes. */
        assert(read_metric(1) == (mask == 2 ? 1u : 2u));
        assert(read_metric(2) == 0);
        complete_one();
    }
    wait_metric(0, 3);
    unsigned successful = 3 - ((mask & 1) != 0) - ((mask & 2) != 0) - ((mask & 4) != 0);
    unsigned pending = read_metric(1);
    assert(pending <= successful);
    while (pending--) complete_one();
    assert_idle(1);
    assert_pixels(0);
    assert(isr_flush_count + task_flush_count == 1);
    int argument = 5;
    muse_lcd_bands_run(control_callback, &argument);
    assert(callback_count == 1 && callback_arguments[0] == 5);
    assert(semaphore_count(s_chunk_free) == 2);
}
static void test_failure_then_next_band(void)
{
    register_display(16, 8);
    fail_mask = 4;  /* The third/last piece fails after two DMA submissions. */
    queue_source(1, 1, 5, 7);
    wait_metric(0, 2);
    wait_chunk_blocked();
    complete_one();
    wait_metric(0, 3);
    wait_chunk_blocked();
    assert(read_metric(1) == 1 && read_metric(2) == 0);
    complete_one();
    wait_metric(2, 1);
    /* LVGL may immediately queue a new band when it is notified, even while
     * the failed band's send task is finishing its buffer-recovery drain. */
    queue_source(0, 0, 4, 4);
    wait_metric(0, 5);
    assert(read_metric(1) == 2 && read_metric(2) == 1);
    complete_one();
    assert(read_metric(2) == 1);
    complete_one();
    assert_idle(2);
    assert_pixels(0);
    assert(isr_flush_count + task_flush_count == 2);
}
static void test_concurrent_controls(void)
{
    register_display(16, 8);
    queue_source(2, 2, 6, 6);
    wait_metric(0, 2);
    control_request_t first = { 7, false }, second = { 9, false };
    pthread_t first_thread = start_control(&first);
    wait_chunk_blocked();
    pthread_t second_thread = start_control(&second);
    complete_one();
    complete_one();
    join_control(first_thread, 2);
    assert(pthread_join(second_thread, NULL) == 0);
    assert_idle(1);
    assert(callback_count == 2 && callback_arguments[0] == 7 && callback_arguments[1] == 9);
}
static void test_invalid_geometry(void)
{
    register_display(24, 8);
    assert(!muse_lcd_bands_set_rotation(4));
    assert(s_rotation == 0);
    assert(muse_lcd_bands_set_rotation(1));
    queue_source(0, 1, 4, 9);
    assert_idle(1);
    assert(read_metric(0) == 0 && task_flush_count == 1 && isr_flush_count == 0);
    int argument = 3;
    muse_lcd_bands_run(control_callback, &argument);
    assert(callback_count == 1);
}
static void test_unregistered(void)
{
    assert(!muse_lcd_bands_set_rotation(1));
    int argument = 2;
    muse_lcd_bands_run(control_callback, &argument);
    assert(callback_count == 1 && callback_arguments[0] == 2);
}
static void test_nonsquare(void)
{
    register_display(16, 6);
    assert(!muse_lcd_bands_set_rotation(0));
    assert(!muse_lcd_bands_set_rotation(1));
    assert(s_rotation == 0 && semaphore_count(s_chunk_free) == 2);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    if (!strcmp(argv[1], "original")) test_original();
    else if (!strcmp(argv[1], "q1")) test_rotated(1);
    else if (!strcmp(argv[1], "q3")) test_rotated(3);
    else if (!strcmp(argv[1], "control")) test_barrier(false);
    else if (!strcmp(argv[1], "rotation")) test_barrier(true);
    else if (!strcmp(argv[1], "concurrent")) test_concurrent_controls();
    else if (!strcmp(argv[1], "first_failure")) test_failures(1);
    else if (!strcmp(argv[1], "middle_failure")) test_failures(2);
    else if (!strcmp(argv[1], "last_failure")) test_failures(4);
    else if (!strcmp(argv[1], "all_failures")) test_failures(7);
    else if (!strcmp(argv[1], "failure_then_next_band")) test_failure_then_next_band();
    else if (!strcmp(argv[1], "invalid")) test_invalid_geometry();
    else if (!strcmp(argv[1], "unregistered")) test_unregistered();
    else if (!strcmp(argv[1], "nonsquare")) test_nonsquare();
    else assert(false);
    printf("PASS %s\n", argv[1]);
    return 0;
}
'''


class MuseRotationBandsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cc = shlex.split(os.environ.get("CC", "cc"))
        if not cc or shutil.which(cc[0]) is None:
            raise unittest.SkipTest("C compiler not available")
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        temporary = Path(cls.temp.name)
        (temporary / "esp_lv_adapter.h").write_text(FAKE_ADAPTER)
        source = (ROOT / "components/muse/boards/muse_lcd_bands.c").read_text()
        source = re.sub(r"^#include .*$", "", source, flags=re.MULTILINE)
        harness = temporary / "rotation_bands_harness.c"
        harness.write_text(MOCKS + source + HARNESS)
        cls.binary = temporary / "rotation_bands_harness"
        compiled = subprocess.run(
            [*cc, "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-O2", "-pthread",
             "-I", str(temporary), "-I", str(ROOT / "components/muse/boards"),
             "-I", str(ROOT / "components/muse"), str(harness),
             str(ROOT / "components/muse/muse_rotation_pixels.c"), "-o", str(cls.binary)],
            capture_output=True, text=True, timeout=30,
        )
        if compiled.returncode:
            raise AssertionError(compiled.stdout + compiled.stderr)

    def run_case(self, case: str) -> None:
        try:
            ran = subprocess.run([str(self.binary), case], capture_output=True, text=True, timeout=5)
        except subprocess.TimeoutExpired as error:
            self.fail(f"LCD bands deadlocked in {case}: {error}")
        self.assertEqual(ran.returncode, 0, ran.stdout + ran.stderr)
        self.assertEqual(ran.stdout.strip(), f"PASS {case}")
        self.assertEqual(ran.stderr, "")

    def test_original_zero_degree_path_and_dma_buffer_lifetime(self) -> None:
        self.run_case("original")

    def test_clockwise_rotated_rectangles_and_pixel_chunks(self) -> None:
        self.run_case("q1")

    def test_counterclockwise_rotated_rectangles_and_pixel_chunks(self) -> None:
        self.run_case("q3")

    def test_control_callback_waits_for_both_pending_dma(self) -> None:
        self.run_case("control")

    def test_rotation_switch_waits_for_both_pending_dma(self) -> None:
        self.run_case("rotation")

    def test_concurrent_control_calls_keep_their_arguments(self) -> None:
        self.run_case("concurrent")

    def test_first_draw_failure_releases_buffer_and_flushes_once(self) -> None:
        self.run_case("first_failure")

    def test_middle_draw_failure_does_not_overwrite_pending_buffer(self) -> None:
        self.run_case("middle_failure")

    def test_last_draw_failure_drains_count_and_flushes_once(self) -> None:
        self.run_case("last_failure")

    def test_all_draw_failures_release_buffers_and_flush_once(self) -> None:
        self.run_case("all_failures")

    def test_new_band_after_failure_drain_keeps_pending_dma_intact(self) -> None:
        self.run_case("failure_then_next_band")

    def test_invalid_rotated_geometry_flushes_without_leaking_tokens(self) -> None:
        self.run_case("invalid")

    def test_control_call_before_registration_is_synchronous(self) -> None:
        self.run_case("unregistered")

    def test_nonsquare_display_rejects_rotation_without_state_change(self) -> None:
        self.run_case("nonsquare")


if __name__ == "__main__":
    unittest.main()

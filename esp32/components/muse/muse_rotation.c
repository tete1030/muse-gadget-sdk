/* Copyright (c) Meta Platforms, Inc. and affiliates. SPDX-License-Identifier: Apache-2.0 */
#include "muse_rotation.h"
#include "sdkconfig.h"
#if CONFIG_MUSE_BOARD_WAVESHARE_S3_175
#include "bsp/esp-bsp.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_lv_adapter.h"
#include "muse_settings.h"
#include "muse_orientation_math.h"
#include "muse_rotation_pixels.h"
#include "boards/muse_lcd_bands.h"

/* Register/scale reference: Waveshare's bundled SensorLib SensorQMI8658.hpp
 * and REG/QMI8658Constants.h. Only the accelerometer is enabled; no GPIO IRQ,
 * gyro, FIFO, recording, network or shared-bus reinitialization is used. */
#define QMI_ADDR 0x6B
#define QMI_WHOAMI 0x00
#define QMI_CTRL1 0x02
#define QMI_CTRL2 0x03
#define QMI_CTRL5 0x06
#define QMI_CTRL7 0x08
#define QMI_STATUS0 0x2E
#define QMI_AX_L 0x35
#define QMI_RESET_RESULT 0x4D
#define QMI_RESET 0x60
static i2c_master_dev_handle_t s_imu;
static lv_display_t *s_disp;
static lv_indev_t *s_indev;
static int s_rotation, s_manual = -1;
static bool s_auto, s_ready, s_reset_pending;
static unsigned s_failures;
static int64_t s_next_sample, s_reset_since;
static int64_t s_last_fresh = -1;
static muse_orientation_math_t s_math;

static esp_err_t rd(uint8_t reg, uint8_t *out, size_t n)
{
    return i2c_master_transmit_receive(s_imu, &reg, 1, out, n, 8);
}
static esp_err_t wr(uint8_t reg, uint8_t value)
{
    uint8_t data[] = { reg, value };
    return i2c_master_transmit(s_imu, data, sizeof(data), 8);
}

static esp_err_t touch_read(esp_lcd_touch_handle_t tp, esp_lcd_touch_point_data_t *points,
                            uint8_t *count, uint8_t max_count, void *ctx)
{
    (void)ctx;
    esp_err_t err = esp_lcd_touch_read_data(tp);
    if (err != ESP_OK) return err;
    err = esp_lcd_touch_get_data(tp, points, count, max_count);
    if (err != ESP_OK || !s_rotation) return err; /* exact existing 0-degree path */
    for (int i = 0; i < *count; i++) {
        /* The BSP's calibrated mirror transform can return its x/y_max (466).
         * Clamp only when applying the inverse pixel transform (0..465). */
        int x = points[i].x < BSP_LCD_H_RES ? points[i].x : BSP_LCD_H_RES - 1;
        int y = points[i].y < BSP_LCD_V_RES ? points[i].y : BSP_LCD_V_RES - 1;
        int lx, ly;
        if (!muse_rotation_touch_inverse(BSP_LCD_H_RES, s_rotation, x, y, &lx, &ly)) return ESP_FAIL;
        points[i].x = lx;
        points[i].y = ly;
    }
    return ESP_OK;
}

static void apply(int quadrant)
{
    if (quadrant == s_rotation || !s_disp) return;
    /* Never rotate under a held finger; release also permits a manual selection
     * to finish cleanly. We are on LVGL's task, so read/flush cannot interleave. */
    if (s_indev && lv_indev_get_state(s_indev) == LV_INDEV_STATE_PRESSED) return;
    if (!muse_lcd_bands_set_rotation(quadrant)) return;
    s_rotation = quadrant;
    if (s_indev) lv_indev_reset(s_indev, NULL);
    lv_obj_invalidate(lv_display_get_screen_active(s_disp));
}

void muse_rotation_attach(lv_display_t *disp, esp_lcd_touch_handle_t touch, lv_indev_t *indev)
{
    s_disp = disp;
    s_indev = indev;
    const esp_lv_adapter_touch_callbacks_t callbacks = { .custom_touch_read = touch_read };
    if (esp_lv_adapter_set_touch_callbacks(indev, &callbacks) != ESP_OK) {
        s_disp = NULL; /* do not rotate a display with an untransformed input */
        return;
    }
    (void)touch; /* The adapter supplies the registered handle to touch_read. */
    muse_orientation_math_init(&s_math, muse_settings_rotation());
}

/* Sensor reset is asynchronous: no delay() or long wait on the LVGL task. */
static bool imu_start(int64_t now)
{
    if (!s_imu) {
        const i2c_device_config_t cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = QMI_ADDR, .scl_speed_hz = 400000 };
        if (i2c_master_bus_add_device(bsp_i2c_get_handle(), &cfg, &s_imu) != ESP_OK) return false;
    }
    uint8_t id;
    if (rd(QMI_WHOAMI, &id, 1) != ESP_OK || id != 0x05 || wr(QMI_RESET, 0xB0) != ESP_OK) return false;
    s_reset_since = now;
    s_reset_pending = true;
    return true;
}

static bool imu_finish(void)
{
    uint8_t result;
    if (rd(QMI_RESET_RESULT, &result, 1) != ESP_OK || result != 0x80) return false;
    /* Little endian, address auto-increment; 4g, 31.25 Hz; LPF mode 0.
     * Gyro stays disabled. 4g/32768 converts signed output to g. */
    return wr(QMI_CTRL7, 0) == ESP_OK && wr(QMI_CTRL1, 0x40) == ESP_OK &&
           wr(QMI_CTRL2, 0x18) == ESP_OK && wr(QMI_CTRL5, 0x01) == ESP_OK &&
           wr(QMI_CTRL7, 0x01) == ESP_OK;
}

void muse_rotation_tick(void)
{
    if (!s_disp) return;
    int manual = muse_settings_rotation();
    bool automatic = muse_settings_rotation_auto();
    int64_t now = esp_timer_get_time() / 1000;
    if (manual != s_manual || automatic != s_auto) {
        s_manual = manual;
        s_auto = automatic;
        muse_orientation_math_init(&s_math, manual);
        s_last_fresh = -1;
        s_next_sample = 0;
        if (!automatic && s_imu) {
            wr(QMI_CTRL7, 0); /* manual mode needs no motion sampling */
            s_ready = false;
            s_reset_since = 0;
            s_reset_pending = false;
        }
    }
    if (!automatic) {
        apply(manual);
        return;
    }
    if (now < s_next_sample) {
        apply(s_ready ? s_math.current_quadrant : manual);
        return;
    }
    s_next_sample = now + 100;
    if (!s_ready) {
        apply(manual); /* unavailable IMU retains the saved manual direction */
        if (!s_reset_pending) {
            if (!imu_start(now)) s_next_sample = now + 1000;
            return;
        }
        if (now - s_reset_since < 20) return;
        if (imu_finish()) {
            s_ready = true;
            s_failures = 0;
            s_last_fresh = -1;
        } else if (now - s_reset_since < 500) {
            return;
        } else {
            s_next_sample = now + 1000;
        }
        s_reset_since = 0;
        s_reset_pending = false;
        return;
    }
    uint8_t ready, data[6];
    bool failed = rd(QMI_STATUS0, &ready, 1) != ESP_OK;
    if (!failed && !(ready & 1)) {
        muse_orientation_math_cancel_candidate(&s_math);
        s_last_fresh = -1;
        return; /* no fresh sample: never advance dwell using stale acceleration */
    }
    if (!failed) failed = rd(QMI_AX_L, data, sizeof(data)) != ESP_OK;
    if (failed) {
        muse_orientation_math_cancel_candidate(&s_math);
        s_last_fresh = -1;
        if (++s_failures >= 3) {
            s_ready = false;
            s_reset_since = 0;
            s_reset_pending = false;
            s_next_sample = now + 1000;
            muse_orientation_math_init(&s_math, manual);
            apply(manual);
        }
        return;
    }
    s_failures = 0;
    float raw_x = (int16_t)((uint16_t)data[0] | (uint16_t)data[1] << 8) / 8192.0f;
    float raw_y = (int16_t)((uint16_t)data[2] | (uint16_t)data[3] << 8) / 8192.0f;
    float raw_z = (int16_t)((uint16_t)data[4] | (uint16_t)data[5] << 8) / 8192.0f;
    /* A paused LVGL loop (including sleep) is not evidence that the pose stayed
     * stable. Restart dwell after more than two missed polling intervals. */
    if (s_last_fresh >= 0 && now - s_last_fresh > 250) muse_orientation_math_cancel_candidate(&s_math);
    s_last_fresh = now;
    /* Official PCB component-side drawing: +X up, +Y left. Viewed from the
     * screen (PWR upper right/BOOT lower right), +X up, +Y right. Positive raw
     * Y therefore asks for 90 degrees clockwise image correction. This mapping
     * is diagram-derived and must be checked in all four physical orientations. */
    apply(muse_orientation_math_update(&s_math, raw_y, raw_x, raw_z, now));
}
int muse_rotation_current(void) { return s_rotation; }
bool muse_rotation_auto_ready(void) { return s_ready; }
#else
void muse_rotation_attach(lv_display_t *d, esp_lcd_touch_handle_t t, lv_indev_t *i) { (void)d; (void)t; (void)i; }
void muse_rotation_tick(void) {}
int muse_rotation_current(void) { return 0; }
bool muse_rotation_auto_ready(void) { return false; }
#endif

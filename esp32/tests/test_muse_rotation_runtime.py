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

"""Production rotation runtime against clock, LVGL, touch and I2C mocks.

Only platform includes are removed from the production translation unit. Its
full functions and state remain intact, and the actual public rotation header,
orientation classifier and pixel/touch helpers are compiled. I2C transactions
are recorded and checked; no hardware, network, credentials or delays are used.
Diagram-derived axes are checked as a software contract, not a physical result.
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
COMPONENT = ROOT / "components" / "muse"
FAKE_LVGL = r'''
#pragma once
typedef struct { int unused; } lv_display_t;
typedef struct { int state; } lv_indev_t;
typedef struct { int unused; } lv_obj_t;
#define LV_INDEV_STATE_RELEASED 0
#define LV_INDEV_STATE_PRESSED 1
'''
FAKE_TOUCH = r'''
#pragma once
#include <stdint.h>
typedef void *esp_lcd_touch_handle_t;
typedef struct { uint8_t track_id; uint16_t x, y, strength; } esp_lcd_touch_point_data_t;
'''
MOCKS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "muse_rotation.h"
#include "muse_orientation_math.h"
#include "muse_rotation_pixels.h"

#define CONFIG_MUSE_BOARD_WAVESHARE_S3_175 1
#define BSP_LCD_H_RES 466
#define BSP_LCD_V_RES 466
#define ESP_OK 0
#define ESP_FAIL -1
#define I2C_ADDR_BIT_LEN_7 7
typedef int esp_err_t;
typedef void *i2c_master_dev_handle_t;
typedef struct { int dev_addr_length, device_address, scl_speed_hz; } i2c_device_config_t;
typedef esp_err_t (*touch_callback_t)(esp_lcd_touch_handle_t,
    esp_lcd_touch_point_data_t *, uint8_t *, uint8_t, void *);
typedef struct { touch_callback_t custom_touch_read; } esp_lv_adapter_touch_callbacks_t;

static lv_display_t display;
static lv_indev_t input;
static lv_obj_t screen;
static int setting_manual;
static bool setting_auto;
static int64_t clock_ms;
static unsigned band_calls, input_resets, invalidations;
static unsigned last_band;
static bool band_success = true;
static int callback_error;
static touch_callback_t installed_touch_read;
static uint8_t registers[256] = { [0x00] = 0x05, [0x4D] = 0x80, [0x2E] = 1 };
static int fail_read_reg = -1, fail_write_reg = -1;
static unsigned fail_reads, fail_writes;
static bool fail_add;
static esp_lcd_touch_point_data_t raw_points[4];
static uint8_t raw_count;
static int touch_read_error, touch_get_error;
static unsigned touch_reads, touch_gets;

typedef struct { char op; uint8_t reg, value; size_t len; int64_t ms; } transaction_t;
static transaction_t transactions[512];
static unsigned transaction_count;
static void record(char op, uint8_t reg, uint8_t value, size_t len)
{
    assert(transaction_count < sizeof(transactions) / sizeof(transactions[0]));
    transactions[transaction_count++] = (transaction_t){op, reg, value, len, clock_ms};
}
static unsigned count_transactions(char op, int reg)
{
    unsigned n = 0;
    for (unsigned i = 0; i < transaction_count; ++i)
        if (transactions[i].op == op && (reg < 0 || transactions[i].reg == reg)) ++n;
    return n;
}
static int muse_settings_rotation(void) { return setting_manual; }
static bool muse_settings_rotation_auto(void) { return setting_auto; }
static int64_t esp_timer_get_time(void) { return clock_ms * 1000; }
static void *bsp_i2c_get_handle(void) { return &registers; }
static esp_err_t i2c_master_bus_add_device(void *bus, const i2c_device_config_t *cfg,
                                          i2c_master_dev_handle_t *out)
{
    assert(bus == &registers && cfg->dev_addr_length == 7);
    assert(cfg->device_address == 0x6B && cfg->scl_speed_hz == 400000);
    record('A', (uint8_t)cfg->device_address, 0, 0);
    if (fail_add) return ESP_FAIL;
    *out = &registers;
    return ESP_OK;
}
static esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t dev,
    const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len, int timeout)
{
    assert(dev == &registers && tx_len == 1 && timeout == 8);
    assert(rx_len == 1 || (tx[0] == 0x35 && rx_len == 6));
    assert((size_t)tx[0] + rx_len <= sizeof(registers));
    record('R', tx[0], 0, rx_len);
    if (tx[0] == fail_read_reg && fail_reads) { --fail_reads; return ESP_FAIL; }
    memcpy(rx, registers + tx[0], rx_len);
    return ESP_OK;
}
static esp_err_t i2c_master_transmit(i2c_master_dev_handle_t dev,
    const uint8_t *tx, size_t tx_len, int timeout)
{
    assert(dev == &registers && tx_len == 2 && timeout == 8);
    record('W', tx[0], tx[1], tx_len);
    if (tx[0] == fail_write_reg && fail_writes) { --fail_writes; return ESP_FAIL; }
    registers[tx[0]] = tx[1];
    return ESP_OK;
}
static esp_err_t esp_lv_adapter_set_touch_callbacks(lv_indev_t *indev,
    const esp_lv_adapter_touch_callbacks_t *callbacks)
{
    assert(indev == &input && callbacks->custom_touch_read != NULL);
    if (callback_error) return callback_error;
    installed_touch_read = callbacks->custom_touch_read;
    return ESP_OK;
}
static int lv_indev_get_state(lv_indev_t *indev)
{
    assert(indev == &input);
    return indev->state;
}
static void lv_indev_reset(lv_indev_t *indev, void *obj)
{
    assert(indev == &input && obj == NULL);
    ++input_resets;
}
static lv_obj_t *lv_display_get_screen_active(lv_display_t *disp)
{
    assert(disp == &display);
    return &screen;
}
static void lv_obj_invalidate(lv_obj_t *obj) { assert(obj == &screen); ++invalidations; }
static bool muse_lcd_bands_set_rotation(unsigned quarter)
{
    assert(quarter < 4);
    ++band_calls;
    last_band = quarter;
    return band_success;
}
static esp_err_t esp_lcd_touch_read_data(esp_lcd_touch_handle_t touch)
{
    assert(touch == raw_points);
    ++touch_reads;
    return touch_read_error;
}
static esp_err_t esp_lcd_touch_get_data(esp_lcd_touch_handle_t touch,
    esp_lcd_touch_point_data_t *out, uint8_t *count, uint8_t max_count)
{
    assert(touch == raw_points);
    ++touch_gets;
    if (touch_get_error) return touch_get_error;
    *count = raw_count < max_count ? raw_count : max_count;
    for (unsigned i = 0; i < *count; ++i) {
        out[i] = raw_points[i];
        /* Existing BSP calibration: both axes mirrored, no axis swap.
         * esp_lcd_touch uses x/y_max - raw, so raw zero becomes 466. */
        out[i].x = 466 - raw_points[i].x;
        out[i].y = 466 - raw_points[i].y;
    }
    return ESP_OK;
}
'''
HARNESS = r'''
static void tick(int64_t ms) { clock_ms = ms; muse_rotation_tick(); }
static void attach(int manual, bool automatic)
{
    setting_manual = manual;
    setting_auto = automatic;
    muse_rotation_attach(&display, raw_points, &input);
    assert(callback_error || installed_touch_read != NULL);
}
static void raw_acceleration(int16_t x, int16_t y, int16_t z)
{
    int16_t values[] = {x, y, z};
    for (int i = 0; i < 3; ++i) {
        uint16_t bits = (uint16_t)values[i];
        registers[0x35 + i * 2] = (uint8_t)bits;
        registers[0x36 + i * 2] = (uint8_t)(bits >> 8);
    }
}
static void raw_direction(int quadrant)
{
    /* The production diagram mapping passes raw_y,raw_x,raw_z to the
     * classifier. Inputs below are signed little-endian 4g-range samples. */
    const int16_t xy[4][2] = {{8192, 0}, {0, 8192}, {-8192, 0}, {0, -8192}};
    raw_acceleration(xy[quadrant][0], xy[quadrant][1], 0);
}
static void expect_transaction(unsigned i, char op, int reg, int value, size_t len)
{
    assert(i < transaction_count);
    transaction_t t = transactions[i];
    assert(t.op == op && t.reg == reg && t.value == value && t.len == len);
}
static void start_auto(int manual)
{
    attach(manual, true);
    tick(1);
    assert(!muse_rotation_auto_ready());
    assert(transaction_count == 3);
    expect_transaction(0, 'A', 0x6B, 0, 0);
    expect_transaction(1, 'R', 0x00, 0, 1);
    expect_transaction(2, 'W', 0x60, 0xB0, 2);
    assert(muse_rotation_current() == manual);
    tick(20);
    tick(100);
    assert(transaction_count == 3 && !muse_rotation_auto_ready());
    tick(101);
    assert(muse_rotation_auto_ready());
    assert(transaction_count == 9);
    expect_transaction(3, 'R', 0x4D, 0, 1);
    expect_transaction(4, 'W', 0x08, 0, 2);
    expect_transaction(5, 'W', 0x02, 0x40, 2);
    expect_transaction(6, 'W', 0x03, 0x18, 2);
    expect_transaction(7, 'W', 0x06, 0x01, 2);
    expect_transaction(8, 'W', 0x08, 0x01, 2);
    assert(count_transactions('R', 0x35) == 0);
}
static void manual_mode(void)
{
    attach(2, false);
    tick(1);
    assert(muse_rotation_current() == 2 && last_band == 2);
    assert(band_calls == 1 && input_resets == 1 && invalidations == 1);
    tick(2);
    tick(10000);
    assert(band_calls == 1 && transaction_count == 0 && !muse_rotation_auto_ready());
    input.state = LV_INDEV_STATE_PRESSED;
    setting_manual = 3;
    tick(10001);
    tick(10100);
    assert(muse_rotation_current() == 2 && band_calls == 1);
    input.state = LV_INDEV_STATE_RELEASED;
    tick(10101);
    assert(muse_rotation_current() == 3 && band_calls == 2);
    assert(input_resets == 2 && invalidations == 2 && transaction_count == 0);
}
static void zero_clock_reset(void)
{
    attach(1, true);
    tick(0);
    assert(transaction_count == 3 && !muse_rotation_auto_ready());
    expect_transaction(2, 'W', 0x60, 0xB0, 2);
    tick(99);
    assert(transaction_count == 3);
    tick(100);
    assert(muse_rotation_auto_ready() && transaction_count == 9);
    assert(count_transactions('R', 0x00) == 1 && count_transactions('W', 0x60) == 1);
    expect_transaction(3, 'R', 0x4D, 0, 1);
    raw_direction(0);
    tick(200);
    assert(count_transactions('R', 0x35) == 1 && muse_rotation_current() == 1);
}
static void band_failure(void)
{
    attach(1, false);
    band_success = false;
    tick(1);
    assert(muse_rotation_current() == 0 && band_calls == 1);
    assert(input_resets == 0 && invalidations == 0);
    band_success = true;
    tick(2);
    assert(muse_rotation_current() == 1 && band_calls == 2);
    assert(input_resets == 1 && invalidations == 1);
}
static void attach_failure(void)
{
    callback_error = -9;
    attach(3, true);
    tick(1);
    tick(10000);
    assert(muse_rotation_current() == 0 && !muse_rotation_auto_ready());
    assert(band_calls == 0 && transaction_count == 0);
}
static void auto_off(void)
{
    start_auto(2);
    setting_auto = false;
    tick(150);
    assert(!muse_rotation_auto_ready());
    expect_transaction(9, 'W', 0x08, 0, 2);
    assert(registers[0x08] == 0 && transaction_count == 10);
    tick(201);
    tick(10000);
    assert(transaction_count == 10 && muse_rotation_current() == 2);
    setting_auto = true;
    tick(10001);
    assert(!muse_rotation_auto_ready() && transaction_count == 12);
    expect_transaction(10, 'R', 0x00, 0, 1);
    expect_transaction(11, 'W', 0x60, 0xB0, 2);
    tick(10101);
    assert(muse_rotation_auto_ready() && count_transactions('A', -1) == 1);
}
static void poll_rate(void)
{
    start_auto(0);
    raw_direction(1);
    tick(201);
    assert(transaction_count == 11);
    expect_transaction(9, 'R', 0x2E, 0, 1);
    expect_transaction(10, 'R', 0x35, 0, 6);
    tick(202); tick(250); tick(300);
    assert(transaction_count == 11);
    tick(301);
    assert(transaction_count == 13);
    registers[0x2E] = 2; /* gyro flag alone is not a new accelerometer sample */
    tick(401);
    assert(transaction_count == 14 && count_transactions('R', 0x35) == 2);
    tick(450);
    assert(transaction_count == 14);
    registers[0x2E] = 3;
    tick(501);
    assert(transaction_count == 16 && count_transactions('R', 0x35) == 3);
}
static void missing_ready_clears_dwell(void)
{
    start_auto(0);
    raw_direction(1);
    for (int ms = 201; ms <= 501; ms += 100) tick(ms);
    assert(muse_rotation_current() == 0);
    unsigned reads = count_transactions('R', 0x35);
    registers[0x2E] = 0;
    tick(601);
    assert(count_transactions('R', 0x35) == reads);
    registers[0x2E] = 1;
    for (int ms = 701; ms <= 1301; ms += 100) tick(ms);
    assert(muse_rotation_current() == 0);
    tick(1401);
    assert(muse_rotation_current() == 1);
}
static void loop_gap_restarts_dwell(int gap)
{
    start_auto(0);
    raw_direction(1);
    tick(201); tick(301); tick(401);
    assert(muse_rotation_current() == 0);
    int resume = 401 + gap;
    tick(resume); /* unchanged pose after a paused loop is a new candidate */
    assert(muse_rotation_current() == 0 && muse_rotation_auto_ready());
    for (int elapsed = 100; elapsed <= 600; elapsed += 100) {
        tick(resume + elapsed);
        assert(muse_rotation_current() == 0);
    }
    tick(resume + 700);
    assert(muse_rotation_current() == 1);
}
static void diagram_direction(int quadrant, bool held)
{
    int manual = (quadrant + 1) % 4;
    start_auto(manual);
    raw_direction(quadrant);
    for (int ms = 201; ms <= 801; ms += 100) tick(ms);
    assert(muse_rotation_current() == manual);
    if (held) input.state = LV_INDEV_STATE_PRESSED;
    tick(901); /* exactly 700ms of valid samples */
    if (held) {
        assert(muse_rotation_current() == manual);
        input.state = LV_INDEV_STATE_RELEASED;
        unsigned n = transaction_count;
        tick(950); /* apply the learned direction without another I2C sample */
        assert(transaction_count == n);
    }
    assert(muse_rotation_current() == quadrant);
    assert(last_band == (unsigned)quadrant && input_resets == invalidations);
}
static void three_failures(int reg)
{
    start_auto(0);
    raw_direction(1);
    for (int ms = 201; ms <= 901; ms += 100) tick(ms);
    assert(muse_rotation_current() == 1);
    fail_read_reg = reg;
    fail_reads = 3;
    tick(1001); tick(1101);
    assert(muse_rotation_current() == 1 && muse_rotation_auto_ready());
    tick(1201);
    assert(muse_rotation_current() == 0 && !muse_rotation_auto_ready());
    unsigned n = transaction_count;
    tick(1301); tick(2200);
    assert(transaction_count == n);
    tick(2201);
    assert(!muse_rotation_auto_ready() && transaction_count == n + 2);
    expect_transaction(n, 'R', 0x00, 0, 1);
    expect_transaction(n + 1, 'W', 0x60, 0xB0, 2);
    tick(2301);
    assert(muse_rotation_auto_ready());
    for (int ms = 2401; ms <= 3001; ms += 100) tick(ms);
    assert(muse_rotation_current() == 0);
    tick(3101);
    assert(muse_rotation_current() == 1 && count_transactions('A', -1) == 1);
}
static void success_clears_failure_count(void)
{
    start_auto(0);
    raw_direction(1);
    fail_read_reg = 0x2E;
    fail_reads = 2;
    tick(201); tick(301);
    assert(muse_rotation_auto_ready());
    tick(401); /* successful status and acceleration read */
    fail_reads = 2;
    tick(501); tick(601);
    assert(muse_rotation_auto_ready());
    fail_reads = 1;
    tick(701);
    assert(!muse_rotation_auto_ready());
}
static void whoami_failure(void)
{
    registers[0x00] = 0xFF;
    attach(2, true);
    tick(1);
    assert(!muse_rotation_auto_ready() && muse_rotation_current() == 2);
    assert(count_transactions('W', 0x60) == 0 && transaction_count == 2);
    registers[0x00] = 0x05;
    tick(1000);
    assert(transaction_count == 2);
    tick(1001);
    assert(transaction_count == 4 && count_transactions('W', 0x60) == 1);
    tick(1101);
    assert(muse_rotation_auto_ready());
}
static void add_failure(void)
{
    fail_add = true;
    attach(3, true);
    tick(1);
    assert(transaction_count == 1 && !muse_rotation_auto_ready());
    fail_add = false;
    tick(1000);
    assert(transaction_count == 1);
    tick(1001);
    assert(transaction_count == 4 && count_transactions('A', -1) == 2);
    tick(1101);
    assert(muse_rotation_auto_ready() && muse_rotation_current() == 3);
}
static void reset_timeout(void)
{
    registers[0x4D] = 0;
    attach(2, true);
    tick(1);
    for (int ms = 101; ms <= 501; ms += 100) tick(ms);
    assert(!muse_rotation_auto_ready() && count_transactions('W', 0x02) == 0);
    unsigned n = transaction_count;
    tick(601); tick(1500);
    assert(transaction_count == n && muse_rotation_current() == 2);
    registers[0x4D] = 0x80;
    tick(1501);
    assert(transaction_count == n + 2);
    tick(1601);
    assert(muse_rotation_auto_ready() && count_transactions('W', 0x60) == 2);
}
static void config_failure(void)
{
    fail_write_reg = 0x03;
    fail_writes = 1;
    attach(0, true);
    tick(1);
    tick(101);
    assert(!muse_rotation_auto_ready() && count_transactions('W', 0x08) == 1);
    tick(201);
    assert(muse_rotation_auto_ready() && registers[0x08] == 1);
    assert(count_transactions('W', 0x60) == 1 && count_transactions('R', 0x4D) == 2);
}
static void invalid_sample_clears_dwell(void)
{
    start_auto(0);
    raw_direction(1);
    tick(201); tick(301); tick(401);
    raw_acceleration(0, 0, 8192); /* flat */
    tick(501);
    raw_direction(1);
    tick(601); tick(701);
    raw_acceleration(5800, 5800, 0); /* diagonal */
    tick(801);
    raw_acceleration(20000, 0, 0); /* shaking, above 1.25g */
    tick(901);
    raw_direction(1);
    for (int ms = 1001; ms <= 1601; ms += 100) tick(ms);
    assert(muse_rotation_current() == 0);
    tick(1701);
    assert(muse_rotation_current() == 1);
}
static void touch_inverse(int quadrant)
{
    attach(quadrant, false);
    tick(1);
    const uint16_t physical[][2] = {{0,0}, {465,0}, {0,465}, {465,465},
                                  {233,233}, {117,349}, {466,466}};
    for (unsigned i = 0; i < sizeof(physical)/sizeof(physical[0]); ++i) {
        int px = physical[i][0], py = physical[i][1];
        raw_points[0] = (esp_lcd_touch_point_data_t){7, (uint16_t)(466-px),
                                                   (uint16_t)(466-py), 333};
        raw_points[1] = (esp_lcd_touch_point_data_t){8, 123, 321, 444};
        raw_count = 2;
        esp_lcd_touch_point_data_t points[3] = {{0}};
        uint8_t count = 99;
        assert(installed_touch_read(raw_points, points, &count, 3, NULL) == ESP_OK);
        assert(count == 2 && points[0].track_id == 7 && points[0].strength == 333);
        assert(points[1].track_id == 8 && points[1].strength == 444);
        for (int p = 0; p < 2; ++p) {
            int x = p ? 343 : px, y = p ? 145 : py;
            if (quadrant) {
                if (x > 465) x = 465;
                if (y > 465) y = 465;
                for (int q = 0; q < quadrant; ++q) {
                    int old_x = x;
                    x = y;
                    y = 465-old_x;
                }
            }
            assert(points[p].x == x && points[p].y == y);
        }
    }
    esp_lcd_touch_point_data_t point = {11, 22, 33, 44};
    uint8_t count = 1;
    touch_read_error = -8;
    unsigned gets = touch_gets;
    assert(installed_touch_read(raw_points, &point, &count, 1, NULL) == -8);
    assert(touch_gets == gets && point.x == 22 && point.y == 33);
    touch_read_error = 0;
    touch_get_error = -9;
    assert(installed_touch_read(raw_points, &point, &count, 1, NULL) == -9);
    assert(point.x == 22 && point.y == 33);
    touch_get_error = 0;
    raw_count = 0;
    assert(installed_touch_read(raw_points, &point, &count, 1, NULL) == ESP_OK);
    assert(count == 0 && point.x == 22 && point.y == 33);
    assert(touch_reads > 0 && touch_gets > 0 && transaction_count == 0);
}
int main(int argc, char **argv)
{
    assert(argc >= 2);
    const char *name = argv[1];
    int arg = argc > 2 ? atoi(argv[2]) : 0;
    if (!strcmp(name, "manual")) manual_mode();
    else if (!strcmp(name, "zero")) zero_clock_reset();
    else if (!strcmp(name, "bands")) band_failure();
    else if (!strcmp(name, "attach")) attach_failure();
    else if (!strcmp(name, "off")) auto_off();
    else if (!strcmp(name, "init")) start_auto(3);
    else if (!strcmp(name, "poll")) poll_rate();
    else if (!strcmp(name, "stale")) missing_ready_clears_dwell();
    else if (!strcmp(name, "gap")) loop_gap_restarts_dwell(arg);
    else if (!strcmp(name, "diagram")) diagram_direction(arg, false);
    else if (!strcmp(name, "held")) diagram_direction(arg, true);
    else if (!strcmp(name, "fail")) three_failures(arg);
    else if (!strcmp(name, "recover")) success_clears_failure_count();
    else if (!strcmp(name, "id")) whoami_failure();
    else if (!strcmp(name, "add")) add_failure();
    else if (!strcmp(name, "reset")) reset_timeout();
    else if (!strcmp(name, "config")) config_failure();
    else if (!strcmp(name, "invalid")) invalid_sample_clears_dwell();
    else if (!strcmp(name, "touch")) touch_inverse(arg);
    else assert(false);
    printf("%s passed: %u I2C transactions\n", name, transaction_count);
    return 0;
}
'''


class RotationRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cc = shlex.split(os.environ.get("CC", "cc"))
        if not cc or shutil.which(cc[0]) is None:
            raise unittest.SkipTest("C compiler not available")
        cls.tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.tmp.cleanup)
        temporary = Path(cls.tmp.name)
        (temporary / "lvgl.h").write_text(FAKE_LVGL)
        (temporary / "esp_lcd_touch.h").write_text(FAKE_TOUCH)
        production = (COMPONENT / "muse_rotation.c").read_text()
        production = re.sub(r'^\s*#include\s+[^\n]*$', '', production, flags=re.MULTILINE)
        wrapper = temporary / "rotation_runtime.c"
        wrapper.write_text(MOCKS + "\n" + production + "\n" + HARNESS)
        cls.binary = temporary / "rotation_runtime"
        compiled = subprocess.run(
            [*cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
             "-I", str(temporary), "-I", str(COMPONENT), str(wrapper),
             str(COMPONENT / "muse_orientation_math.c"),
             str(COMPONENT / "muse_rotation_pixels.c"), "-lm", "-o", str(cls.binary)],
            capture_output=True, text=True, timeout=30,
        )
        if compiled.returncode:
            raise AssertionError(compiled.stdout + compiled.stderr)

    def run_scenario(self, name: str, argument: int | None = None) -> None:
        argv = [str(self.binary), name]
        if argument is not None:
            argv.append(str(argument))
        ran = subprocess.run(argv, capture_output=True, text=True, timeout=5)
        self.assertEqual(ran.returncode, 0, ran.stdout + ran.stderr)
        self.assertEqual(ran.stderr, "")
        self.assertIn(name + " passed:", ran.stdout)

    def test_manual_rotation_defers_until_finger_release(self) -> None:
        self.run_scenario("manual")

    def test_failed_band_rotation_retries_without_false_current_state(self) -> None:
        self.run_scenario("bands")

    def test_touch_callback_failure_prevents_display_rotation(self) -> None:
        self.run_scenario("attach")

    def test_auto_off_disables_accelerometer_and_stops_polling(self) -> None:
        self.run_scenario("off")

    def test_async_sensor_reset_identity_and_exact_config_registers(self) -> None:
        self.run_scenario("init")

    def test_reset_started_at_zero_clock_finishes_without_restarting(self) -> None:
        self.run_scenario("zero")

    def test_fresh_sample_reads_are_limited_to_100_ms(self) -> None:
        self.run_scenario("poll")

    def test_missing_accelerometer_ready_bit_clears_dwell(self) -> None:
        self.run_scenario("stale")

    def test_paused_loop_requires_fresh_700_ms_dwell_on_resume(self) -> None:
        for gap in (1000, 600000):
            with self.subTest(gap=gap):
                self.run_scenario("gap", gap)

    def test_diagram_axis_mapping_and_signed_raw_conversion_all_quadrants(self) -> None:
        for quarter in range(4):
            with self.subTest(quarter=quarter):
                self.run_scenario("diagram", quarter)

    def test_auto_rotation_defers_until_finger_release_all_quadrants(self) -> None:
        for quarter in range(4):
            with self.subTest(quarter=quarter):
                self.run_scenario("held", quarter)

    def test_three_i2c_failures_fall_back_to_manual_then_retry(self) -> None:
        for register in (0x2E, 0x35):
            with self.subTest(register=register):
                self.run_scenario("fail", register)

    def test_successful_sample_restarts_failure_counter(self) -> None:
        self.run_scenario("recover")

    def test_wrong_sensor_identity_retries_after_one_second(self) -> None:
        self.run_scenario("id")

    def test_i2c_add_failure_retries_after_one_second(self) -> None:
        self.run_scenario("add")

    def test_reset_result_timeout_retries_without_blocking(self) -> None:
        self.run_scenario("reset")

    def test_partial_sensor_configuration_failure_can_recover(self) -> None:
        self.run_scenario("config")

    def test_flat_diagonal_and_shaking_samples_cancel_pending_rotation(self) -> None:
        self.run_scenario("invalid")

    def test_touch_inverse_after_original_mirror_calibration_all_quadrants(self) -> None:
        for quarter in range(4):
            with self.subTest(quarter=quarter):
                self.run_scenario("touch", quarter)


if __name__ == "__main__":
    unittest.main()

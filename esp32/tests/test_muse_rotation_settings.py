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

"""Exercise production rotation settings against fake NVS, including reboot.

Only the platform includes are substituted. All production declarations and
functions stay together, and the actual public settings header is compiled.
This tests successful NVS reads/writes, not power-loss or hardware behavior.
"""

from __future__ import annotations

import json
import os
import re
import shlex
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FAKE_ESP_ERR = r'''
#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_NVS_NOT_FOUND 1
#define ESP_ERR_NVS_NO_FREE_PAGES 2
#define ESP_ERR_NVS_NEW_VERSION_FOUND 3
#define ESP_ERR_INVALID_SIZE 4
'''
MOCKS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "host_compat.h"
#include "muse_settings.h"

#define CONFIG_MUSE_DEFAULT_VOLUME 50
#define ESP_LOGW(tag, ...) ((void)(tag))
#define ESP_LOGI(tag, ...) ((void)(tag))
#define ESP_ERROR_CHECK(call) assert((call) == ESP_OK)
#define portMAX_DELAY 0
#define NVS_READWRITE 0
typedef int SemaphoreHandle_t;
typedef int nvs_handle_t;
static int xSemaphoreCreateMutex(void) { return 1; }
static void xSemaphoreTake(int lock, int wait) { (void)lock; (void)wait; }
static void xSemaphoreGive(int lock) { (void)lock; }

static struct { char key[16]; uint8_t value; } entries[32];
static int entry_count, writes, commits, notifications;
static muse_setting_t last_notification;

static int stored(const char *key)
{
    for (int i = 0; i < entry_count; ++i) {
        if (strcmp(entries[i].key, key) == 0) return entries[i].value;
    }
    return -1;
}

static void seed(const char *key, int value)
{
    assert(value >= 0 && value <= 255);
    int index;
    for (index = 0; index < entry_count; ++index) {
        if (strcmp(entries[index].key, key) == 0) break;
    }
    if (index == entry_count) {
        assert(entry_count < (int)(sizeof(entries) / sizeof(entries[0])));
        assert(strlen(key) < sizeof(entries[index].key));
        strcpy(entries[index].key, key);
        ++entry_count;
    }
    entries[index].value = (uint8_t)value;
}

static esp_err_t nvs_flash_init(void) { return ESP_OK; }
static esp_err_t nvs_flash_erase(void) { entry_count = 0; return ESP_OK; }
static esp_err_t nvs_open(const char *name, int mode, nvs_handle_t *out)
{
    assert(strcmp(name, "muse") == 0 && mode == NVS_READWRITE);
    *out = 1;
    return ESP_OK;
}
static esp_err_t nvs_get_u8(nvs_handle_t handle, const char *key, uint8_t *out)
{
    assert(handle == 1);
    int value = stored(key);
    if (value < 0) return ESP_ERR_NVS_NOT_FOUND;
    *out = (uint8_t)value;
    return ESP_OK;
}
static esp_err_t nvs_set_u8(nvs_handle_t handle, const char *key, uint8_t value)
{
    assert(handle == 1);
    seed(key, value);
    ++writes;
    return ESP_OK;
}
static esp_err_t nvs_get_u16(nvs_handle_t handle, const char *key, uint16_t *out)
{
    (void)handle; (void)key; (void)out;
    return ESP_ERR_NVS_NOT_FOUND;
}
static esp_err_t nvs_set_u16(nvs_handle_t handle, const char *key, uint16_t value)
{
    (void)handle; (void)key; (void)value;
    return ESP_OK;
}
static esp_err_t nvs_get_str(nvs_handle_t handle, const char *key, char *out, size_t *len)
{
    (void)handle; (void)key; (void)out; (void)len;
    return ESP_ERR_NVS_NOT_FOUND;
}
static esp_err_t nvs_set_str(nvs_handle_t handle, const char *key, const char *value)
{
    (void)handle; (void)key; (void)value;
    return ESP_OK;
}
static esp_err_t nvs_commit(nvs_handle_t handle)
{
    assert(handle == 1);
    ++commits;
    return ESP_OK;
}
static bool muse_link_wifi_get(char *ssid, char *pass)
{
    (void)ssid; (void)pass;
    return false;
}
static bool muse_link_wifi_set(const char *ssid, const char *pass)
{
    (void)ssid; (void)pass;
    return false;
}
'''
HARNESS = r'''
static void listener(muse_setting_t setting)
{
    ++notifications;
    last_notification = setting;
}

static void output(void)
{
    printf("{\"manual\":%d,\"auto\":%d,\"stored_manual\":%d,\"stored_auto\":%d,"
           "\"writes\":%d,\"commits\":%d,\"notifications\":%d,\"rotation_notification\":%d,"
           "\"volume\":%d,\"brightness\":%d,\"wifi_on\":%d}\n",
           muse_settings_rotation(), muse_settings_rotation_auto(),
           stored("rotation"), stored("rot_auto"), writes, commits, notifications,
           notifications > 0 && last_notification == MUSE_SETTING_ROTATION,
           muse_settings_volume(), muse_settings_brightness(), muse_settings_wifi_on());
}

int main(void)
{
    /* Saving the static initializer lets a reboot restore every default while
     * preserving only fake NVS. Each test process also starts with fresh NVS. */
    const __typeof__(s) defaults = s;
    char line[256];
    while (fgets(line, sizeof(line), stdin) != NULL) {
        char key[16];
        int value;
        if (sscanf(line, "seed %15s %d", key, &value) == 2) {
            seed(key, value);
            continue;
        }
        if (strcmp(line, "boot\n") == 0) {
            s = defaults;
            s_lock = 0;
            s_nvs = 0;
            s_listener = NULL;
            assert(muse_settings_init() == ESP_OK);
            muse_settings_set_listener(listener);
        } else if (sscanf(line, "manual %d", &value) == 1) {
            muse_settings_set_rotation(value);
        } else if (sscanf(line, "auto %d", &value) == 1) {
            assert(value == 0 || value == 1);
            muse_settings_set_rotation_auto(value != 0);
        } else {
            assert(strcmp(line, "read\n") == 0);
        }
        output();
    }
    return 0;
}
'''


class MuseRotationSettingsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cc = shlex.split(os.environ.get("CC", "cc"))
        if not cc or shutil.which(cc[0]) is None:
            raise unittest.SkipTest("C compiler not available")
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        temporary = Path(cls.temp.name)
        (temporary / "esp_err.h").write_text(FAKE_ESP_ERR)
        source = (ROOT / "components/muse/muse_settings.c").read_text()
        source = re.sub(r"^#include .*$", "", source, flags=re.MULTILINE)
        harness = temporary / "rotation_settings_harness.c"
        harness.write_text(MOCKS + source + HARNESS)
        cls.binary = temporary / "rotation_settings_harness"
        compiled = subprocess.run(
            [*cc, "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-O2",
             "-I", str(temporary), "-I", str(ROOT / "tests"),
             "-I", str(ROOT / "components/muse"), str(harness), "-o", str(cls.binary)],
            capture_output=True, text=True,
        )
        if compiled.returncode:
            raise AssertionError(compiled.stdout + compiled.stderr)

    def run_commands(self, *commands: str) -> list[dict]:
        ran = subprocess.run(
            [str(self.binary)], input="\n".join(commands) + "\n",
            capture_output=True, text=True,
        )
        self.assertEqual(ran.returncode, 0, ran.stdout + ran.stderr)
        self.assertEqual(ran.stderr, "")
        return [json.loads(line) for line in ran.stdout.splitlines()]

    def assert_orientation(self, state: dict, manual: int, auto: bool) -> None:
        self.assertEqual((state["manual"], state["auto"]), (manual, int(auto)), state)

    def test_first_boot_defaults_to_manual_zero(self) -> None:
        state, = self.run_commands("boot")
        self.assert_orientation(state, 0, False)
        self.assertEqual((state["stored_manual"], state["stored_auto"]), (-1, -1))
        self.assertEqual((state["writes"], state["commits"], state["notifications"]), (0, 0, 0))

    def test_old_nvs_without_rotation_keeps_other_settings(self) -> None:
        state, = self.run_commands("seed volume 42", "seed bright 72", "seed wifi_on 0", "boot")
        self.assert_orientation(state, 0, False)
        self.assertEqual((state["volume"], state["brightness"], state["wifi_on"]), (42, 72, 0))

    def test_loads_each_valid_manual_rotation(self) -> None:
        for quadrant in range(4):
            for auto in (0, 1):
                with self.subTest(quadrant=quadrant, auto=auto):
                    state, = self.run_commands(f"seed rotation {quadrant}", f"seed rot_auto {auto}", "boot")
                    self.assert_orientation(state, quadrant, bool(auto))
                    self.assertEqual(state["notifications"], 0)

    def test_missing_auto_key_retains_manual_mode(self) -> None:
        state, = self.run_commands("seed rotation 3", "boot")
        self.assert_orientation(state, 3, False)

    def test_missing_manual_key_uses_zero_with_auto_enabled(self) -> None:
        state, = self.run_commands("seed rot_auto 1", "boot")
        self.assert_orientation(state, 0, True)

    def test_corrupt_manual_rotation_falls_back_to_zero(self) -> None:
        for value in (4, 42, 127, 255):
            with self.subTest(value=value):
                state, = self.run_commands(f"seed rotation {value}", "seed rot_auto 1", "boot")
                self.assert_orientation(state, 0, True)

    def test_corrupt_auto_flag_disables_auto(self) -> None:
        for value in (2, 42, 127, 255):
            with self.subTest(value=value):
                state, = self.run_commands("seed rotation 3", f"seed rot_auto {value}", "boot")
                self.assert_orientation(state, 3, False)

    def test_manual_selection_disables_auto_and_persists_both_keys(self) -> None:
        for quadrant in range(4):
            with self.subTest(quadrant=quadrant):
                initial, changed, rebooted = self.run_commands(
                    "seed rotation 2", "seed rot_auto 1", "boot", f"manual {quadrant}", "boot",
                )
                self.assert_orientation(initial, 2, True)
                for state in (changed, rebooted):
                    self.assert_orientation(state, quadrant, False)
                    self.assertEqual((state["stored_manual"], state["stored_auto"]), (quadrant, 0))
                self.assertEqual(changed["writes"], 2)
                self.assertGreaterEqual(changed["commits"], 1)
                self.assertEqual((changed["notifications"], changed["rotation_notification"]), (1, 1))
                self.assertEqual(rebooted["writes"], changed["writes"])

    def test_invalid_manual_selection_has_no_effect(self) -> None:
        for value in (-1, 4, 255, 2 ** 30):
            with self.subTest(value=value):
                initial, ignored, rebooted = self.run_commands(
                    "seed rotation 2", "seed rot_auto 1", "boot", f"manual {value}", "boot",
                )
                self.assertEqual(ignored, initial)
                self.assertEqual(rebooted, initial)

    def test_auto_toggles_preserve_saved_manual_orientation(self) -> None:
        for quadrant in range(4):
            with self.subTest(quadrant=quadrant):
                initial, enabled, auto_boot, disabled, manual_boot = self.run_commands(
                    f"seed rotation {quadrant}", "seed rot_auto 0", "boot",
                    "auto 1", "boot", "auto 0", "boot",
                )
                self.assert_orientation(initial, quadrant, False)
                for state in (enabled, auto_boot):
                    self.assert_orientation(state, quadrant, True)
                    self.assertEqual((state["stored_manual"], state["stored_auto"]), (quadrant, 1))
                    self.assertEqual(state["writes"], 1)
                for state in (disabled, manual_boot):
                    self.assert_orientation(state, quadrant, False)
                    self.assertEqual((state["stored_manual"], state["stored_auto"]), (quadrant, 0))
                    self.assertEqual(state["writes"], 2)
                self.assertEqual((enabled["notifications"], enabled["rotation_notification"]), (1, 1))
                self.assertEqual((disabled["notifications"], disabled["rotation_notification"]), (2, 1))
                self.assertGreaterEqual(enabled["commits"], 1)
                self.assertGreater(disabled["commits"], enabled["commits"])

    def test_latest_manual_choice_survives_reboot(self) -> None:
        states = self.run_commands("boot", "manual 1", "auto 1", "manual 2", "manual 3", "boot")
        self.assert_orientation(states[-1], 3, False)
        self.assertEqual((states[-1]["stored_manual"], states[-1]["stored_auto"]), (3, 0))
        self.assertEqual(states[-1]["notifications"], 4)


if __name__ == "__main__":
    unittest.main()

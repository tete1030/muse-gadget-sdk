/* Copyright (c) Meta Platforms, Inc. and affiliates. SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <stdbool.h>
#include "lvgl.h"
#include "esp_lcd_touch.h"
#ifdef __cplusplus
extern "C" {
#endif
/* All functions except the availability query run on the LVGL task/lock. */
void muse_rotation_attach(lv_display_t *disp, esp_lcd_touch_handle_t touch, lv_indev_t *indev);
void muse_rotation_tick(void);
int muse_rotation_current(void);
bool muse_rotation_auto_ready(void);
#ifdef __cplusplus
}
#endif

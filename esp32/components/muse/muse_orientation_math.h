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

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pure orientation classifier. The caller maps the board's accelerometer into
 * screen axes: +Y = 0 degrees, +X = 90, -Y = 180, -X = 270. Quadrants are
 * numbered 0..3, not degrees. No hardware, timing or display APIs are used. */
typedef struct {
    uint8_t current_quadrant;
    int8_t candidate_quadrant;       /* -1 when no rotation is pending */
    uint64_t candidate_since_ms;
} muse_orientation_math_t;

/* Start Auto mode from the user's selected manual orientation. An out-of-range
 * initial quadrant falls back to 0. */
void muse_orientation_math_init(muse_orientation_math_t *state,
                                uint8_t initial_quadrant);

/* Clear the dwell timer after a failed/missing sensor reading, retaining the
 * current orientation. Call this on read failure rather than reusing old data. */
void muse_orientation_math_cancel_candidate(muse_orientation_math_t *state);

/* Input is acceleration in g and monotonic milliseconds. A new direction must
 * remain valid for at least 700 ms. Valid readings have total magnitude
 * 0.75..1.25 g, in-plane magnitude >= 0.55 g, |Z| <= 0.8 g, and an in-plane
 * axis dominating the other by at least 0.2 g. Flat, diagonal, shaking and
 * non-finite readings retain the current quadrant and restart the dwell.
 * Returns the current quadrant (0 for a NULL state). */
uint8_t muse_orientation_math_update(muse_orientation_math_t *state,
                                    float ax_g, float ay_g, float az_g,
                                    uint64_t now_ms);

#ifdef __cplusplus
}
#endif

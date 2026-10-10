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

#include "muse_orientation_math.h"

#include <math.h>
#include <stddef.h>

#define ORIENTATION_HOLD_MS UINT64_C(700)

void muse_orientation_math_cancel_candidate(muse_orientation_math_t *state)
{
    if (state == NULL) return;
    state->candidate_quadrant = -1;
    state->candidate_since_ms = 0;
}

void muse_orientation_math_init(muse_orientation_math_t *state,
                                uint8_t initial_quadrant)
{
    if (state == NULL) return;
    state->current_quadrant = initial_quadrant < 4 ? initial_quadrant : 0;
    muse_orientation_math_cancel_candidate(state);
}

static int8_t sample_quadrant(float ax, float ay, float az)
{
    if (!isfinite(ax) || !isfinite(ay) || !isfinite(az)) return -1;

    /* Compare squared magnitudes to avoid square roots on every IMU sample. */
    float projected_squared = ax * ax + ay * ay;
    float norm_squared = projected_squared + az * az;
    if (norm_squared < 0.75f * 0.75f || norm_squared > 1.25f * 1.25f ||
        projected_squared < 0.55f * 0.55f || fabsf(az) > 0.8f) {
        return -1;
    }

    float abs_x = fabsf(ax), abs_y = fabsf(ay);
    if (abs_x - abs_y >= 0.2f) return ax > 0 ? 1 : 3;
    if (abs_y - abs_x >= 0.2f) return ay > 0 ? 0 : 2;
    return -1;
}

uint8_t muse_orientation_math_update(muse_orientation_math_t *state,
                                    float ax_g, float ay_g, float az_g,
                                    uint64_t now_ms)
{
    if (state == NULL) return 0;

    int8_t quadrant = sample_quadrant(ax_g, ay_g, az_g);
    if (quadrant < 0 || quadrant == state->current_quadrant) {
        muse_orientation_math_cancel_candidate(state);
        return state->current_quadrant;
    }

    if (quadrant != state->candidate_quadrant ||
        now_ms < state->candidate_since_ms) {
        state->candidate_quadrant = quadrant;
        state->candidate_since_ms = now_ms;
    } else if (now_ms - state->candidate_since_ms >= ORIENTATION_HOLD_MS) {
        state->current_quadrant = (uint8_t)quadrant;
        muse_orientation_math_cancel_candidate(state);
    }
    return state->current_quadrant;
}

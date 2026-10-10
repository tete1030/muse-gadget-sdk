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

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Rotate a nonempty, half-open logical rectangle within a size x size screen.
 * quarter is 0..3 clockwise quarter turns: (x,y), (size-1-y,x),
 * (size-1-x,size-1-y), (y,size-1-x). Outputs are the half-open physical bounds.
 * Invalid coordinates, quadrant or NULL output pointers return false without
 * modifying outputs. No clipping is performed. */
bool muse_rotation_rect(int size, int quarter, int x1, int y1, int x2, int y2,
                        int *px1, int *py1, int *px2, int *py2);

/* Copy a strip of a rotated logical rectangle, without allocating memory.
 * src holds the compact logical rectangle in row-major order. Each pixel is
 * an opaque pair of bytes: RGB565 byte order is preserved exactly.
 * dest_y is an absolute physical screen row; dest_rows must be positive and
 * the complete strip must fit inside the rectangle's rotated bounds.
 * dest receives every pixel across the rotated rectangle's width, row-major.
 * The caller provides at least (source_x2-source_x1)*(source_y2-source_y1)*2
 * bytes in src and rotated_width*dest_rows*2 bytes in dest. Buffers must not
 * overlap. Invalid arguments or byte-count overflow return false without
 * modifying dest. The unrotated path is a single contiguous memcpy. */
bool muse_rotation_copy_rows(const uint8_t *src,
                             int source_x1, int source_y1,
                             int source_x2, int source_y2,
                             int screen_size, int quarter,
                             int dest_y, int dest_rows, uint8_t *dest);

/* Convert one physical touch coordinate back to logical screen coordinates,
 * using the exact inverse of the displayed rotation. Both coordinates must
 * lie in [0,size), and quarter must be 0..3. Invalid arguments or NULL output
 * pointers return false without modifying outputs. */
bool muse_rotation_touch_inverse(int size, int quarter, int px, int py,
                                 int *lx, int *ly);

#ifdef __cplusplus
}
#endif

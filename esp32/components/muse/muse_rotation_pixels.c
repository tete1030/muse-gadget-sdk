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

#include "muse_rotation_pixels.h"

#include <stddef.h>
#include <string.h>

bool muse_rotation_rect(int size, int quarter, int x1, int y1, int x2, int y2,
                        int *px1, int *py1, int *px2, int *py2)
{
    if (size <= 0 || quarter < 0 || quarter > 3 ||
        x1 < 0 || y1 < 0 || x1 >= x2 || y1 >= y2 ||
        x2 > size || y2 > size ||
        px1 == NULL || py1 == NULL || px2 == NULL || py2 == NULL) {
        return false;
    }

    int a, b, c, d;
    switch (quarter) {
    case 1:
        a = size - y2;
        b = x1;
        c = size - y1;
        d = x2;
        break;
    case 2:
        a = size - x2;
        b = size - y2;
        c = size - x1;
        d = size - y1;
        break;
    case 3:
        a = y1;
        b = size - x2;
        c = y2;
        d = size - x1;
        break;
    default:
        a = x1;
        b = y1;
        c = x2;
        d = y2;
        break;
    }
    *px1 = a;
    *py1 = b;
    *px2 = c;
    *py2 = d;
    return true;
}

/* Check before multiplying, including on 32-bit targets. */
static bool pixel_bytes_fit(size_t width, size_t height)
{
    return width <= SIZE_MAX / 2 && height <= SIZE_MAX / (width * 2);
}

bool muse_rotation_copy_rows(const uint8_t *src,
                             int source_x1, int source_y1,
                             int source_x2, int source_y2,
                             int screen_size, int quarter,
                             int dest_y, int dest_rows, uint8_t *dest)
{
    int px1, py1, px2, py2;
    if (src == NULL || dest == NULL ||
        !muse_rotation_rect(screen_size, quarter,
                            source_x1, source_y1, source_x2, source_y2,
                            &px1, &py1, &px2, &py2) ||
        dest_y < py1 || dest_y >= py2 || dest_rows <= 0 ||
        dest_rows > py2 - dest_y) {
        return false;
    }

    size_t source_width = (size_t)(source_x2 - source_x1);
    size_t source_height = (size_t)(source_y2 - source_y1);
    size_t dest_width = (size_t)(px2 - px1);
    if (!pixel_bytes_fit(source_width, source_height) ||
        !pixel_bytes_fit(dest_width, (size_t)dest_rows)) {
        return false;
    }

    if (quarter == 0) {
        size_t first = (size_t)(dest_y - source_y1) * source_width * 2;
        memcpy(dest, src + first, dest_width * (size_t)dest_rows * 2);
        return true;
    }

    for (int row = 0; row < dest_rows; ++row) {
        int physical_y = dest_y + row;
        if (quarter == 1) {
            /* One source column, traversed from bottom to top. */
            size_t source_x = (size_t)(physical_y - source_x1);
            for (size_t col = source_height; col > 0; --col) {
                size_t i = ((col - 1) * source_width + source_x) * 2;
                *dest++ = src[i];
                *dest++ = src[i + 1];
            }
        } else if (quarter == 2) {
            /* One source row, traversed from right to left. */
            size_t source_y = (size_t)(screen_size - 1 - physical_y - source_y1);
            size_t start = source_y * source_width;
            for (size_t col = source_width; col > 0; --col) {
                size_t i = (start + col - 1) * 2;
                *dest++ = src[i];
                *dest++ = src[i + 1];
            }
        } else {
            /* One source column, traversed from top to bottom. */
            size_t source_x = (size_t)(screen_size - 1 - physical_y - source_x1);
            for (size_t col = 0; col < source_height; ++col) {
                size_t i = (col * source_width + source_x) * 2;
                *dest++ = src[i];
                *dest++ = src[i + 1];
            }
        }
    }
    return true;
}

bool muse_rotation_touch_inverse(int size, int quarter, int px, int py,
                                 int *lx, int *ly)
{
    if (size <= 0 || quarter < 0 || quarter > 3 ||
        px < 0 || py < 0 || px >= size || py >= size ||
        lx == NULL || ly == NULL) {
        return false;
    }

    int x, y;
    switch (quarter) {
    case 1:
        x = py;
        y = size - 1 - px;
        break;
    case 2:
        x = size - 1 - px;
        y = size - 1 - py;
        break;
    case 3:
        x = size - 1 - py;
        y = px;
        break;
    default:
        x = px;
        y = py;
        break;
    }
    *lx = x;
    *ly = y;
    return true;
}

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

"""Allocation-free square-screen pixel rotation and inverse touch mapping."""

from __future__ import annotations

import ctypes
import os
import random
import shlex
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT / "components" / "muse"
BYTE = ctypes.c_uint8
INT = ctypes.c_int
BYTE_PTR = ctypes.POINTER(BYTE)
INT_PTR = ctypes.POINTER(INT)
GUARD = bytes([0xA5]) * 16


def forward(size: int, quarter: int, x: int, y: int) -> tuple[int, int]:
    """Independent point-by-point reference; no rectangle formula."""
    for _ in range(quarter):
        x, y = size - 1 - y, x
    return x, y


def pixel_data(count: int) -> bytes:
    # Both bytes vary independently; catches unintended endian conversions.
    return bytes(value for i in range(count)
                 for value in ((i * 29 + 0x12) & 255, ((i >> 8) ^ i ^ 0xC7) & 255))


def reference(size: int, quarter: int, rect: tuple[int, int, int, int],
              source: bytes) -> tuple[tuple[int, int, int, int], bytes]:
    x1, y1, x2, y2 = rect
    width = x2 - x1
    pixels = {}
    for y in range(y1, y2):
        for x in range(x1, x2):
            i = ((y - y1) * width + x - x1) * 2
            pixels[forward(size, quarter, x, y)] = source[i:i + 2]
    xs, ys = zip(*pixels)
    bounds = min(xs), min(ys), max(xs) + 1, max(ys) + 1
    a, b, c, d = bounds
    result = b"".join(pixels[x, y] for y in range(b, d) for x in range(a, c))
    return bounds, result


class RotationPixelsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cc = shlex.split(os.environ.get("CC", "cc"))
        if not cc or shutil.which(cc[0]) is None:
            raise unittest.SkipTest("C compiler not available")
        cls.tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.tmp.cleanup)
        shared = Path(cls.tmp.name) / "rotation.so"
        subprocess.run(cc + ["-std=c99", "-Wall", "-Wextra", "-Werror", "-O2",
                             "-shared", "-fPIC", "-I", str(COMPONENT),
                             str(COMPONENT / "muse_rotation_pixels.c"),
                             "-o", str(shared)], check=True, capture_output=True, text=True)
        cls.lib = ctypes.CDLL(str(shared))
        cls.lib.muse_rotation_rect.argtypes = [INT] * 6 + [INT_PTR] * 4
        cls.lib.muse_rotation_rect.restype = ctypes.c_bool
        cls.lib.muse_rotation_copy_rows.argtypes = [BYTE_PTR] + [INT] * 8 + [BYTE_PTR]
        cls.lib.muse_rotation_copy_rows.restype = ctypes.c_bool
        cls.lib.muse_rotation_touch_inverse.argtypes = [INT] * 4 + [INT_PTR] * 2
        cls.lib.muse_rotation_touch_inverse.restype = ctypes.c_bool

    def bounds(self, size: int, quarter: int,
               rect: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
        outputs = [INT(-777) for _ in range(4)]
        self.assertTrue(self.lib.muse_rotation_rect(size, quarter, *rect,
                                                   *(ctypes.byref(v) for v in outputs)))
        return tuple(v.value for v in outputs)

    def strip(self, size: int, quarter: int, rect: tuple[int, int, int, int],
              source: bytes, dest_y: int, dest_rows: int) -> bytes:
        a, _, c, _ = self.bounds(size, quarter, rect)
        count = (c - a) * dest_rows * 2
        src = (BYTE * len(source)).from_buffer_copy(source)
        storage = (BYTE * (count + 2 * len(GUARD))).from_buffer_copy(
            GUARD + bytes([0xDE]) * count + GUARD)
        dest = ctypes.cast(ctypes.byref(storage, len(GUARD)), BYTE_PTR)
        self.assertTrue(self.lib.muse_rotation_copy_rows(
            src, *rect, size, quarter, dest_y, dest_rows, dest))
        self.assertEqual(bytes(src), source, "source must remain unchanged")
        self.assertEqual(bytes(storage[:len(GUARD)]), GUARD)
        self.assertEqual(bytes(storage[-len(GUARD):]), GUARD)
        return bytes(storage[len(GUARD):-len(GUARD)])

    def assert_rotation(self, size: int, quarter: int,
                        rect: tuple[int, int, int, int], chunks: tuple[int, ...]) -> None:
        x1, y1, x2, y2 = rect
        source = pixel_data((x2 - x1) * (y2 - y1))
        bounds, expected = reference(size, quarter, rect, source)
        self.assertEqual(self.bounds(size, quarter, rect), bounds)
        _, first, _, last = bounds
        self.assertEqual(self.strip(size, quarter, rect, source, first, last - first), expected)
        pieces = []
        y = first
        i = 0
        while y < last:
            rows = min(chunks[i % len(chunks)], last - y)
            pieces.append(self.strip(size, quarter, rect, source, y, rows))
            y += rows
            i += 1
        self.assertEqual(b"".join(pieces), expected)

    def test_known_four_by_four_rotations(self) -> None:
        source = bytes(value for i in range(16) for value in (i, 255 - i))
        expected = [
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
            [12, 8, 4, 0, 13, 9, 5, 1, 14, 10, 6, 2, 15, 11, 7, 3],
            [15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0],
            [3, 7, 11, 15, 2, 6, 10, 14, 1, 5, 9, 13, 0, 4, 8, 12],
        ]
        for quarter, values in enumerate(expected):
            with self.subTest(quarter=quarter):
                result = self.strip(4, quarter, (0, 0, 4, 4), source, 0, 4)
                self.assertEqual(result, bytes(v for i in values for v in (i, 255 - i)))

    def test_every_small_rectangle_odd_and_even_screens(self) -> None:
        for size in range(1, 6):
            for x1 in range(size):
                for x2 in range(x1 + 1, size + 1):
                    for y1 in range(size):
                        for y2 in range(y1 + 1, size + 1):
                            for quarter in range(4):
                                with self.subTest(size=size, quarter=quarter,
                                                  rect=(x1, y1, x2, y2)):
                                    self.assert_rotation(size, quarter, (x1, y1, x2, y2), (1, 2, 3))

    def test_random_dirty_rectangles_and_strips(self) -> None:
        rng = random.Random(0x466118)
        for size in (7, 8, 15, 16, 63, 64):
            for _ in range(20):
                x1, x2 = sorted(rng.sample(range(size + 1), 2))
                y1, y2 = sorted(rng.sample(range(size + 1), 2))
                for quarter in range(4):
                    with self.subTest(size=size, quarter=quarter, rect=(x1, y1, x2, y2)):
                        self.assert_rotation(size, quarter, (x1, y1, x2, y2), (7, 1, 13, 2))

    def test_real_466_screen_118_high_bands(self) -> None:
        # Three full bands, the final shorter band, and a partial dirty band.
        for rect in ((0, 0, 466, 118), (0, 118, 466, 236),
                     (0, 236, 466, 354), (0, 354, 466, 466),
                     (17, 121, 429, 239)):
            for quarter in range(4):
                with self.subTest(quarter=quarter, rect=rect):
                    self.assert_rotation(466, quarter, rect, (16, 31, 1, 118))

    def test_full_frame_reconstructed_from_real_bands(self) -> None:
        size = 466
        source = pixel_data(size * size)
        for quarter in range(4):
            _, expected = reference(size, quarter, (0, 0, size, size), source)
            actual = bytearray(len(source))
            coverage = bytearray(size * size)
            for logical_y in range(0, size, 118):
                logical_end = min(logical_y + 118, size)
                rect = (0, logical_y, size, logical_end)
                band = source[logical_y * size * 2:logical_end * size * 2]
                px1, py1, px2, py2 = self.bounds(size, quarter, rect)
                width = px2 - px1
                for physical_y in range(py1, py2, 16):
                    rows = min(16, py2 - physical_y)
                    strip = self.strip(size, quarter, rect, band, physical_y, rows)
                    for row in range(rows):
                        start = (physical_y + row) * size + px1
                        row_start = row * width * 2
                        actual[start * 2:(start + width) * 2] = strip[row_start:row_start + width * 2]
                        for pixel in range(start, start + width):
                            coverage[pixel] += 1
            with self.subTest(quarter=quarter):
                self.assertEqual(bytes(actual), expected)
                self.assertEqual(bytes(coverage), bytes([1]) * (size * size))

    def test_each_possible_strip_of_partial_rectangle(self) -> None:
        size, rect = 11, (2, 3, 9, 8)
        source = pixel_data(35)
        for quarter in range(4):
            bounds, expected = reference(size, quarter, rect, source)
            x1, y1, x2, y2 = bounds
            row_bytes = (x2 - x1) * 2
            for y in range(y1, y2):
                for rows in range(1, y2 - y + 1):
                    with self.subTest(quarter=quarter, y=y, rows=rows):
                        self.assertEqual(self.strip(size, quarter, rect, source, y, rows),
                                         expected[(y-y1)*row_bytes:(y-y1+rows)*row_bytes])

    def test_pixel_bytes_are_opaque(self) -> None:
        source = bytes([0x12, 0x34, 0xAB, 0xCD, 0xF0, 0x01, 0x07, 0xE0])
        result = self.strip(2, 1, (0, 0, 2, 2), source, 0, 2)
        self.assertEqual(result, bytes([0xF0, 0x01, 0x12, 0x34, 0x07, 0xE0, 0xAB, 0xCD]))

    def test_invalid_rectangles_leave_outputs_untouched(self) -> None:
        invalid = [
            (0, 0, (0, 0, 1, 1)), (-1, 0, (0, 0, 1, 1)),
            (4, -1, (0, 0, 4, 4)), (4, 4, (0, 0, 4, 4)),
            (4, 0, (-1, 0, 4, 4)), (4, 0, (0, -1, 4, 4)),
            (4, 0, (0, 0, 5, 4)), (4, 0, (0, 0, 4, 5)),
            (4, 0, (1, 0, 1, 4)), (4, 0, (0, 1, 4, 1)),
            (4, 0, (3, 0, 2, 4)), (4, 0, (0, 3, 4, 2)),
        ]
        for size, quarter, rect in invalid:
            with self.subTest(size=size, quarter=quarter, rect=rect):
                outputs = [INT(-777) for _ in range(4)]
                self.assertFalse(self.lib.muse_rotation_rect(
                    size, quarter, *rect, *(ctypes.byref(v) for v in outputs)))
                self.assertEqual([v.value for v in outputs], [-777] * 4)
        for missing in range(4):
            outputs = [INT(-777) for _ in range(4)]
            pointers = [ctypes.byref(v) for v in outputs]
            pointers[missing] = None
            self.assertFalse(self.lib.muse_rotation_rect(4, 0, 0, 0, 4, 4, *pointers))
            self.assertEqual([v.value for v in outputs], [-777] * 4)

    def test_large_coordinate_rectangles_do_not_overflow(self) -> None:
        size = 2**31 - 1
        for quarter in range(4):
            self.assertEqual(self.bounds(size, quarter, (0, 0, size, size)),
                             (0, 0, size, size))
        self.assertEqual(self.bounds(size, 1, (size-2, size-3, size, size)),
                         (0, size-2, 3, size))

    def test_invalid_copy_inputs_leave_destination_untouched(self) -> None:
        src = (BYTE * 32).from_buffer_copy(pixel_data(16))
        dest = (BYTE * 32).from_buffer_copy(GUARD * 2)
        invalid = [
            ((0, 0, 4, 4), 4, 0, -1, 1),
            ((0, 0, 4, 4), 4, 0, 4, 1),
            ((0, 0, 4, 4), 4, 0, 0, 0),
            ((0, 0, 4, 4), 4, 0, 0, -1),
            ((0, 0, 4, 4), 4, 0, 3, 2),
            ((0, 0, 4, 4), 4, 0, 1, 2**31-1),
            ((0, 0, 4, 4), 4, 4, 0, 1),
            ((0, 0, 4, 4), 0, 0, 0, 1),
            ((-1, 0, 4, 4), 4, 0, 0, 1),
            ((0, 0, 5, 4), 4, 0, 0, 1),
            ((0, 0, 0, 4), 4, 0, 0, 1),
        ]
        for rect, size, quarter, y, rows in invalid:
            with self.subTest(rect=rect, size=size, quarter=quarter, y=y, rows=rows):
                self.assertFalse(self.lib.muse_rotation_copy_rows(
                    src, *rect, size, quarter, y, rows, dest))
                self.assertEqual(bytes(dest), GUARD * 2)
        self.assertFalse(self.lib.muse_rotation_copy_rows(None, 0, 0, 4, 4, 4, 0, 0, 4, dest))
        self.assertEqual(bytes(dest), GUARD * 2)
        self.assertFalse(self.lib.muse_rotation_copy_rows(src, 0, 0, 4, 4, 4, 0, 0, 4, None))
        # Use nonzero rotated y origins, so accidental logical-y interpretation fails.
        for quarter in range(4):
            rect = (1, 2, 3, 4)
            _, first, _, last = self.bounds(7, quarter, rect)
            for y, rows in ((first - 1, 1), (last, 1), (last - 1, 2)):
                self.assertFalse(self.lib.muse_rotation_copy_rows(
                    src, *rect, 7, quarter, y, rows, dest))
                self.assertEqual(bytes(dest), GUARD * 2)

    def test_touch_inverse_round_trips_corners_center_and_random_points(self) -> None:
        rng = random.Random(0x175)
        for size in (1, 2, 3, 4, 7, 8, 465, 466):
            points = {(0, 0), (0, size-1), (size-1, 0), (size-1, size-1),
                      (size//2, size//2), ((size-1)//2, (size-1)//2)}
            points.update((rng.randrange(size), rng.randrange(size)) for _ in range(100))
            for quarter in range(4):
                for x, y in points:
                    px, py = forward(size, quarter, x, y)
                    lx, ly = INT(-777), INT(-777)
                    self.assertTrue(self.lib.muse_rotation_touch_inverse(
                        size, quarter, px, py, ctypes.byref(lx), ctypes.byref(ly)))
                    self.assertEqual((lx.value, ly.value), (x, y))

    def test_invalid_touch_inputs_leave_outputs_untouched(self) -> None:
        for size, quarter, x, y in ((0, 0, 0, 0), (-1, 0, 0, 0),
                                     (4, -1, 0, 0), (4, 4, 0, 0),
                                     (4, 0, -1, 0), (4, 0, 0, -1),
                                     (4, 0, 4, 0), (4, 0, 0, 4)):
            lx, ly = INT(-777), INT(-777)
            self.assertFalse(self.lib.muse_rotation_touch_inverse(
                size, quarter, x, y, ctypes.byref(lx), ctypes.byref(ly)))
            self.assertEqual((lx.value, ly.value), (-777, -777))
        for missing in range(2):
            outputs = [INT(-777), INT(-777)]
            pointers = [ctypes.byref(v) for v in outputs]
            pointers[missing] = None
            self.assertFalse(self.lib.muse_rotation_touch_inverse(4, 1, 0, 0, *pointers))
            self.assertEqual([v.value for v in outputs], [-777, -777])

    def test_header_is_cpp_compatible(self) -> None:
        cxx = shlex.split(os.environ.get("CXX", "c++"))
        if not cxx or shutil.which(cxx[0]) is None:
            self.skipTest("C++ compiler not available")
        subprocess.run(cxx + ["-std=c++11", "-Wall", "-Wextra", "-Werror", "-x", "c++",
                              "-fsyntax-only", "-I", str(COMPONENT), "-"],
                       input='#include "muse_rotation_pixels.h"\n',
                       check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()

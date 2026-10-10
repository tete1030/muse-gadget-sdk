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

"""Compile the pure Muse orientation classifier on the host, without ESP-IDF."""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DIRECTIONS = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0),
              (0.0, -1.0, 0.0), (-1.0, 0.0, 0.0))
HARNESS = r'''
#include "muse_orientation_math.h"

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void output(const muse_orientation_math_t *state)
{
    printf("%u %d %" PRIu64 "\n", (unsigned)state->current_quadrant,
           (int)state->candidate_quadrant, state->candidate_since_ms);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    /* Null handling is safe even when no sensor or Auto-mode state exists. */
    muse_orientation_math_init(NULL, 0);
    muse_orientation_math_cancel_candidate(NULL);
    assert(muse_orientation_math_update(NULL, 0, 1, 0, 0) == 0);

    muse_orientation_math_t state;
    muse_orientation_math_init(&state, (uint8_t)strtoul(argv[1], NULL, 10));
    output(&state);
    char line[256];
    while (fgets(line, sizeof(line), stdin) != NULL) {
        if (strcmp(line, "cancel\n") == 0) {
            muse_orientation_math_cancel_candidate(&state);
        } else {
            uint64_t now;
            float x, y, z;
            assert(sscanf(line, "%" SCNu64 " %f %f %f", &now, &x, &y, &z) == 4);
            uint8_t quadrant = muse_orientation_math_update(&state, x, y, z, now);
            assert(quadrant == state.current_quadrant);
            assert(quadrant < 4);
        }
        output(&state);
    }
    return 0;
}
'''


class MuseOrientationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cc = shlex.split(os.environ.get("CC", "cc"))
        if not cc or shutil.which(cc[0]) is None:
            raise unittest.SkipTest("C compiler not available")
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        harness = Path(cls.temp.name) / "orientation_harness.c"
        harness.write_text(HARNESS)
        cls.binary = Path(cls.temp.name) / "orientation_harness"
        compiled = subprocess.run(
            [*cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
             "-I", str(ROOT / "components/muse"), str(harness),
             str(ROOT / "components/muse/muse_orientation_math.c"),
             "-o", str(cls.binary)],
            capture_output=True, text=True,
        )
        if compiled.returncode:
            raise AssertionError(compiled.stdout + compiled.stderr)

    def run_samples(self, initial: int, samples: list) -> list[tuple[int, int, int]]:
        lines = []
        for sample in samples:
            lines.append(sample if isinstance(sample, str) else " ".join(map(str, sample)))
        ran = subprocess.run(
            [str(self.binary), str(initial)], input="\n".join(lines) + "\n" if lines else "",
            capture_output=True, text=True,
        )
        self.assertEqual(ran.returncode, 0, ran.stdout + ran.stderr)
        self.assertEqual(ran.stderr, "")
        states = [tuple(map(int, line.split())) for line in ran.stdout.splitlines()]
        self.assertEqual(states[0], (initial if initial < 4 else 0, -1, 0))
        self.assertEqual(len(states), len(samples) + 1)
        return states[1:]

    def test_initial_manual_quadrant_is_retained(self) -> None:
        for initial in range(4):
            with self.subTest(initial=initial):
                self.assertEqual(self.run_samples(initial, []), [])
                states = self.run_samples(initial, [(0, 0, 0, 1), (10000, 0, 0, 1)])
                self.assertEqual(states, [(initial, -1, 0), (initial, -1, 0)])
        self.assertEqual(self.run_samples(4, []), [])

    def test_all_four_canonical_directions(self) -> None:
        for quadrant, acceleration in enumerate(DIRECTIONS):
            with self.subTest(quadrant=quadrant):
                initial = (quadrant + 1) % 4
                states = self.run_samples(initial, [(100, *acceleration), (800, *acceleration)])
                self.assertEqual(states, [(initial, quadrant, 100), (quadrant, -1, 0)])

    def test_switch_at_exactly_700_ms_and_not_699(self) -> None:
        states = self.run_samples(0, [(0, 1, 0, 0), (699, 1, 0, 0), (700, 1, 0, 0)])
        self.assertEqual(states, [(0, 1, 0), (0, 1, 0), (1, -1, 0)])

    def test_new_candidate_restarts_dwell(self) -> None:
        states = self.run_samples(0, [
            (100, 1, 0, 0), (500, 0, -1, 0), (800, 0, -1, 0), (1200, 0, -1, 0),
        ])
        self.assertEqual(states, [(0, 1, 100), (0, 2, 500), (0, 2, 500), (2, -1, 0)])

    def test_return_to_current_direction_cancels_dwell(self) -> None:
        states = self.run_samples(0, [
            (0, 1, 0, 0), (300, 0, 1, 0), (600, 1, 0, 0), (1000, 1, 0, 0), (1300, 1, 0, 0),
        ])
        self.assertEqual([state[0] for state in states], [0, 0, 0, 0, 1])
        self.assertEqual(states[1], (0, -1, 0))
        self.assertEqual(states[2], (0, 1, 600))

    def test_diagonal_jitter_never_rotates(self) -> None:
        for initial in range(4):
            with self.subTest(initial=initial):
                samples = [(time, x, y, 0) for time, (x, y) in enumerate(
                    [(0.71, 0.70), (0.70, 0.71), (-0.71, 0.70), (-0.70, -0.71),
                     (0.71, -0.70), (0.70, -0.71)] * 5, start=1)]
                samples = [(time * 800, x, y, z) for time, x, y, z in samples]
                self.assertTrue(all(state == (initial, -1, 0)
                                    for state in self.run_samples(initial, samples)))

    def test_dominant_axis_needs_point_two_g_margin(self) -> None:
        rejected = self.run_samples(2, [(0, 0.80, 0.61, 0), (700, 0.80, 0.61, 0)])
        self.assertEqual(rejected, [(2, -1, 0), (2, -1, 0)])
        accepted = self.run_samples(2, [(0, 0.82, 0.60, 0), (700, 0.82, 0.60, 0)])
        self.assertEqual(accepted[-1], (1, -1, 0))

    def test_flat_readings_do_not_rotate(self) -> None:
        for sample in [(0, 0, 1), (0, 0, -1), (0.5, 0, 0.75), (0.6, 0, 0.81)]:
            with self.subTest(sample=sample):
                states = self.run_samples(3, [(0, *sample), (7000, *sample)])
                self.assertEqual(states, [(3, -1, 0), (3, -1, 0)])

    def test_in_plane_magnitude_and_z_limits(self) -> None:
        for sample, expected in [((0.549, 0, 0.7), 0), ((0.55, 0, 0.7), 1),
                                 ((0.6, 0, 0.8), 1), ((0.6, 0, -0.8), 1)]:
            with self.subTest(sample=sample):
                states = self.run_samples(0, [(0, *sample), (700, *sample)])
                self.assertEqual(states[-1], (expected, -1, 0))

    def test_invalid_norm_and_shake_keep_current(self) -> None:
        for sample in [(0, 0, 0), (0.749, 0, 0), (1.251, 0, 0), (3, 0, 0), (1, 0, 1)]:
            with self.subTest(sample=sample):
                states = self.run_samples(2, [(0, *sample), (7000, *sample)])
                self.assertEqual(states, [(2, -1, 0), (2, -1, 0)])

    def test_norm_limits_are_inclusive(self) -> None:
        for x in (0.75, 1.25):
            with self.subTest(x=x):
                self.assertEqual(self.run_samples(0, [(0, x, 0, 0), (700, x, 0, 0)])[-1],
                                 (1, -1, 0))

    def test_non_finite_acceleration_keeps_current(self) -> None:
        for invalid in (float("nan"), float("inf"), float("-inf"), 1e30):
            for axis in range(3):
                with self.subTest(invalid=invalid, axis=axis):
                    sample = [1.0, 0.0, 0.0]
                    sample[axis] = invalid
                    self.assertEqual(self.run_samples(3, [(0, *sample), (7000, *sample)]),
                                     [(3, -1, 0), (3, -1, 0)])

    def test_invalid_reading_interrupts_stability(self) -> None:
        for invalid in [(0, 0, 1), (2, 0, 0), (0.71, 0.70, 0), (float("nan"), 0, 0)]:
            with self.subTest(invalid=invalid):
                states = self.run_samples(0, [
                    (0, 1, 0, 0), (650, *invalid), (700, 1, 0, 0),
                    (1399, 1, 0, 0), (1400, 1, 0, 0),
                ])
                self.assertEqual([state[0] for state in states], [0, 0, 0, 0, 1])
                self.assertEqual(states[1], (0, -1, 0))
                self.assertEqual(states[2], (0, 1, 700))

    def test_sensor_failure_interrupts_stability(self) -> None:
        states = self.run_samples(0, [
            (0, 1, 0, 0), "cancel", (1000, 1, 0, 0), (1699, 1, 0, 0), (1700, 1, 0, 0),
        ])
        self.assertEqual(states, [(0, 1, 0), (0, -1, 0), (0, 1, 1000), (0, 1, 1000), (1, -1, 0)])

    def test_64_bit_monotonic_timestamps(self) -> None:
        start = 2 ** 32 + 123
        states = self.run_samples(0, [
            (start, -1, 0, 0), (start + 699, -1, 0, 0), (start + 700, -1, 0, 0),
        ])
        self.assertEqual(states, [(0, 3, start), (0, 3, start), (3, -1, 0)])

    def test_clock_before_candidate_start_restarts_dwell(self) -> None:
        states = self.run_samples(0, [
            (1000, 1, 0, 0), (500, 1, 0, 0), (1199, 1, 0, 0), (1200, 1, 0, 0),
        ])
        self.assertEqual(states, [(0, 1, 1000), (0, 1, 500), (0, 1, 500), (1, -1, 0)])


if __name__ == "__main__":
    unittest.main()

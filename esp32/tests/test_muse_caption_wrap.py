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

"""How reply captions wrap to the screen's page (muse_chat_text.c): words stay
whole, and CJK, which has no spaces, breaks between characters but never puts
closing punctuation at the start of a line. Letters the ASCII fonts lack show
as their plain ones (muse_text.c)."""

from __future__ import annotations

import os
import re
import shlex
import shutil
import subprocess
import tempfile
import unicodedata
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CLOSING = "、。，．：；！？）」』】》"


class CaptionWrapTest(unittest.TestCase):
    binary: Path

    @classmethod
    def setUpClass(cls) -> None:
        cc = shlex.split(os.environ.get("CC", "cc"))
        if not cc or shutil.which(cc[0]) is None:
            raise unittest.SkipTest("C compiler not available")
        cls.tmp = tempfile.TemporaryDirectory()
        cls.binary = Path(cls.tmp.name) / "muse_serial_chat_harness"
        proc = subprocess.run(
            [
                *cc,
                "-include",
                str(ROOT / "tests" / "host_compat.h"),
                "-std=gnu11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                str(ROOT / "components" / "muse"),
                str(ROOT / "tests" / "muse_serial_chat_harness.c"),
                str(ROOT / "components" / "muse" / "muse_chat_text.c"),
                str(ROOT / "components" / "muse" / "muse_text.c"),
                "-o",
                str(cls.binary),
            ],
            cwd=ROOT,
            text=True,
            capture_output=True,
        )
        if proc.returncode:
            raise AssertionError(proc.stdout + proc.stderr)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tmp.cleanup()

    def wrap(self, text: str, cols: int) -> list[str]:
        proc = subprocess.run(
            [str(self.binary), "caption", str(cols)],
            input=text.encode(),
            capture_output=True,
            check=True,
        )
        return proc.stdout.decode().split("\n")

    def test_words_stay_whole(self) -> None:
        self.assertEqual(self.wrap("the quick brown fox jumps", 10), ["the quick", "brown fox", "jumps"])

    def test_cjk_fills_each_line(self) -> None:
        text = "还伴有八级左右的雷雨大风和短时强降水出门记得带伞注意安全"
        lines = self.wrap(text, 13)
        self.assertEqual("".join(lines), text)
        self.assertEqual([len(line) for line in lines], [13, 13, 2])

    def test_ascii_runs_inside_cjk_stay_whole(self) -> None:
        text = "广州今天雷阵雨，30°/24°，还伴有8级左右的雷雨大风和短时强降水，出门小心。"
        lines = self.wrap(text, 13)
        self.assertEqual("".join(lines), text)
        self.assertTrue(any(line.startswith("30°/24°") for line in lines), lines)
        for line in lines:
            self.assertLessEqual(len(line), 13, line)

    def test_closing_punctuation_never_starts_a_line(self) -> None:
        self.assertEqual(self.wrap("一二三四五，六七", 5), ["一二三四", "五，六七"])
        for cols in range(4, 16):
            for line in self.wrap("好消息在后头：冷空气正在南下，明天雨就收了。气温一路往下走。", cols)[1:]:
                self.assertNotIn(line[:1], CLOSING, f"{cols} columns: {line!r}")

    def test_closing_punctuation_after_a_full_line_of_ascii(self) -> None:
        self.assertEqual(self.wrap("好abcdefghijklm，后", 13), ["好", "abcdefghijkl", "m，后"])
        text = "广州今天雷阵雨，30°/24°，还伴有8级左右的雷雨大风和短时强降水，出门小心。"
        for cols in range(4, 16):
            lines = self.wrap(text, cols)
            self.assertEqual("".join(lines), text)
            for line in lines[1:]:
                self.assertNotIn(line[:1], CLOSING, f"{cols} columns: {line!r}")

    def test_mixed_text_breaks_at_spaces_or_between_cjk(self) -> None:
        self.assertEqual(self.wrap("Muse 说 hello world", 8), ["Muse 说", "hello", "world"])

    def page(self, text: str, cols: int, lines: int, at: int) -> str:
        proc = subprocess.run(
            [str(self.binary), "page", str(cols), str(lines), str(at)],
            input=text.encode(), capture_output=True, check=True,
        )
        # Strict decoding also catches an offset splitting a UTF-8 character.
        return proc.stdout.decode()

    def test_reading_pages_at_every_utf8_byte_offset(self) -> None:
        for text in (
            "中文回复需要慢慢阅读，最后一页保留六秒。",
            "Muse 说 hello world，今天30°，请带伞。",
            "ASCII words remain whole while reading the reply.",
        ):
            for lines in (1, 2, 3, 5):
                pages = [self.page(text, 8, lines, at) for at in range(len(text.encode()))]
                self.assertTrue(all(pages))
                for page in pages:
                    self.assertLessEqual(len(page.splitlines()), lines)
                    for line in page.splitlines():
                        self.assertNotIn(line[:1], CLOSING)
                # No non-space character is lost, including the last character.
                shown = "".join(pages)
                for character in text.replace(" ", ""):
                    self.assertIn(character, shown)
                self.assertIn(text[-1], pages[-1])

    def test_silent_reading_defaults_and_frame_timing(self) -> None:
        source = (ROOT / "components/muse/muse_chat_session.cpp").read_text()
        constant = lambda name: int(re.search(r"^#define " + name + r" (\d+)", source, re.M)[1])
        rate, hold, sample_rate = map(constant, ("TEXT_CHARS_PER_S", "TEXT_HOLD_S", "MIC_RATE"))
        self.assertEqual((rate, hold), (6, 6))
        # Production keeps byte offsets; Chinese must take three times ASCII time.
        for text, seconds in (("abcdef", 1), ("中文回复", 2), ("Hi中文", 8 / 6)):
            frames = len(text.encode()) * sample_rate // rate
            self.assertAlmostEqual(frames / sample_rate, seconds, places=4)
            self.assertEqual(hold * sample_rate, 96000)
        self.assertIn("m.len * MIC_RATE / TEXT_CHARS_PER_S", source)
        self.assertIn("m.pcm_frames + TEXT_HOLD_S * MIC_RATE", source)

    def shown(self, text: str) -> str:
        proc = subprocess.run(
            [str(self.binary), "ascii"], input=text.encode(), capture_output=True, check=True
        )
        return proc.stdout.decode()

    def test_vietnamese_shows_its_plain_letters(self) -> None:
        self.assertEqual(
            self.shown("Thời tiết Hà Nội hôm nay đẹp, được không?"),
            "Thoi tiet Ha Noi hom nay dep, duoc khong?",
        )
        # Every Vietnamese letter, upper and lower case, decomposed or not.
        letters = "".join(map(chr, range(0x1EA0, 0x1EFA))) + "ƠơƯưĐđ"
        plain = "".join(unicodedata.normalize("NFD", c)[0] for c in letters).replace("Đ", "D").replace("đ", "d")
        self.assertEqual(self.shown(letters), plain)
        self.assertEqual(self.shown(unicodedata.normalize("NFD", letters)), plain)


if __name__ == "__main__":
    unittest.main()

# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
import math
import struct
import unittest

from tools.hfp_tone_quality import analyze


def capture(lost=(), length=30):
    packets = []
    for start in range(0, 2400, length):
        index = start // length
        status = 2 if index in lost else 0
        samples = [
            0 if status else round(1000 * math.sin(2 * math.pi * 440 * i / 8000) + 123)
            for i in range(start, start + length)
        ]
        packets.append(bytes([3]) + struct.pack("<HB", 1 | status << 12, length * 2))
        packets.append(struct.pack(f"<{length}h", *samples))
    return b"".join(packets)


class ToneQualityTest(unittest.TestCase):
    def test_known_tone_level_dc_and_packetization(self):
        for length in (30, 60):
            result = analyze(capture(length=length))
            self.assertAlmostEqual(result["tone_peak"], 1000, delta=1)
            self.assertAlmostEqual(result["dc_offset"], 123, delta=1)
            self.assertEqual(result["duration_ms"], 300)
            self.assertEqual(result["bad_sample_percent"], 0)
            self.assertGreater(result["all_tone_to_error_db"], 60)

    def test_loss_reduces_whole_stream_quality(self):
        result = analyze(capture(lost=(10, 20)))
        self.assertEqual(result["packets_by_status"], [78, 0, 2, 0])
        self.assertEqual(result["bad_sample_percent"], 2.5)
        self.assertGreater(result["valid_tone_to_error_db"], 60)
        self.assertLess(result["all_tone_to_error_db"], 20)

    def test_rejects_invalid_and_mixed_streams(self):
        valid = capture()
        for data in (
            b"",
            b"\3",
            b"\4\0\0\0",
            valid[:-1],
            valid + b"\3\1\0\1\0",
            valid + b"\3\2\0\2\0\0",
        ):
            with self.subTest(data_length=len(data)), self.assertRaises(ValueError):
                analyze(data)
        for frequency in (0, 4000, float("nan")):
            with self.assertRaises(ValueError):
                analyze(valid, frequency)

    def test_silence_has_no_tone_ratio(self):
        data = (b"\3\1\0\x3c" + bytes(60)) * 4
        self.assertIsNone(analyze(data)["valid_tone_to_error_db"])

# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Optional Bumble tests: use the same PYTHONPATH as tools/hfp_probe.py."""

import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    from hfp_probe import Probe, ScoCredits, tone_pcm
except ModuleNotFoundError as error:
    if error.name != "bumble":
        raise
    ScoCredits = None


@unittest.skipIf(ScoCredits is None, "optional Bumble dependency is not installed")
class TestScoCredits(unittest.TestCase):
    def setUp(self):
        self.credits = ScoCredits()
        self.credits.handle = 384
        self.credits.limit = 3
        self.direction = ScoCredits.Direction.CONTROLLER_TO_HOST

    def test_mixed_acl_and_sco_completions(self):
        packet = bytes.fromhex("04 13 09 02 01 00 04 00 80 01 02 00")
        self.credits.snoop(packet, self.direction)
        self.assertEqual(self.credits.available, 2)
        self.assertEqual(self.credits.completed, 2)

    def test_inactive_link_wrong_direction_and_truncation(self):
        packet = bytes.fromhex("04 13 05 01 80 01 01 00")
        self.credits.snoop(packet, ScoCredits.Direction.HOST_TO_CONTROLLER)
        self.credits.snoop(packet[:-1], self.direction)
        self.credits.handle = None
        self.credits.snoop(packet, self.direction)
        self.assertEqual(self.credits.completed, 0)
        self.assertEqual(self.credits.available, 0)

    def test_credit_limit_and_optional_trace_forwarding(self):
        captured = []

        class Trace:
            def snoop(self, packet, direction):
                captured.append((packet, direction))

        self.credits.trace = Trace()
        packet = bytes.fromhex("04 13 05 01 80 01 04 00")
        self.credits.snoop(packet, self.direction)
        self.assertEqual(self.credits.available, 3)
        self.assertEqual(self.credits.completed, 4)
        self.assertEqual(captured, [(packet, self.direction)])

    def test_tone_is_bounded_and_continuous_across_packets(self):
        whole = tone_pcm(0, 8100)
        chunks = b"".join(
            tone_pcm(offset, min(30, 8100 - offset)) for offset in range(0, 8100, 30)
        )
        self.assertEqual(whole, chunks)
        samples = struct.unpack("<8100h", whole)
        self.assertLessEqual(max(abs(sample) for sample in samples), 2048)
        self.assertGreater(max(samples), 2000)
        self.assertEqual(whole[16000:], bytes(200))

    def test_large_pcm_packet_is_fragmented_without_exceeding_credits(self):
        sent = []
        host = SimpleNamespace(snooper=None, send_sco_sdu=lambda handle, data: sent.append(data))
        probe = Probe(SimpleNamespace(host=host), sco_flow_control=True)
        probe.tx_mtu = 60
        probe.credits.available = 2
        packet = SimpleNamespace(data=bytes(120), connection_handle=384, packet_status=0)
        probe.on_packet(packet)
        self.assertEqual(sent, [bytes(60), bytes(60)])
        self.assertEqual(probe.credits.available, 0)
        probe.on_packet(packet)
        self.assertEqual(probe.tx_skipped, 2)
        self.assertEqual(probe.tx_packets, 2)

    def test_watch_audio_does_not_send_desktop_pcm(self):
        sent = []
        host = SimpleNamespace(snooper=None, send_sco_sdu=lambda handle, data: sent.append(data))
        probe = Probe(SimpleNamespace(host=host), watch_audio=True)
        packet = SimpleNamespace(data=bytes(60), connection_handle=384, packet_status=0)
        probe.on_packet(packet)
        self.assertEqual(sent, [])


if __name__ == "__main__":
    unittest.main()

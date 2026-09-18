# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
"""Portable embedded host tested with independent Bumble protocol encoders."""

import ctypes
import struct
import subprocess
import tempfile
import unittest
from pathlib import Path

from bumble import core, rfcomm, sdp

ROOT = Path(__file__).resolve().parents[2]


def le(*values):
    return struct.pack("<" + "H" * len(values), *values)


class ClassicHostTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        library = Path(cls.directory.name) / "classic.so"
        sources = ROOT / "src/bluetooth-fw/classic_demo"
        subprocess.run(
            [
                "cc",
                "-std=c11",
                "-shared",
                "-fPIC",
                "-Wall",
                "-Wextra",
                "-Wno-unused-parameter",
                "-Werror",
                "-I" + str(ROOT / "include"),
                "-I" + str(sources),
                str(Path(__file__).with_name("classic_demo_harness.c")),
                *(str(sources / f) for f in ("host.c", "profile.c", "sdp.c")),
                "-o",
                str(library),
            ],
            check=True,
        )
        cls.lib = ctypes.CDLL(str(library))
        cls.lib.demo_detail.restype = ctypes.c_char_p

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def setUp(self):
        self.now = 0
        self.lib.demo_init()
        self.boot()

    def tick(self):
        self.now += 1
        self.lib.demo_tick(self.now)

    def receive(self, packet):
        self.lib.demo_receive(bytes(packet), len(packet))
        self.tick()

    def event(self, code, payload):
        self.receive(bytes([4, code, len(payload)]) + payload)

    def pop(self):
        self.tick()
        buf = ctypes.create_string_buffer(700)
        n = self.lib.demo_pop(buf)
        return buf.raw[:n]

    def complete(self, packet):
        opcode = int.from_bytes(packet[1:3], "little")
        result = le(1021) + bytes([60]) + le(4, 7) if opcode == 0x1005 else b""
        self.event(0x0E, bytes([1]) + le(opcode) + b"\0" + result)

    def boot(self):
        opcodes = []
        while packet := self.pop():
            self.assertEqual(packet[0], 1)
            opcodes.append(int.from_bytes(packet[1:3], "little"))
            self.complete(packet)
        self.assertEqual(
            opcodes,
            [
                0x0C03,
                0x0C01,
                0x0C56,
                0x0C24,
                0x0C13,
                0x0C18,
                0x0C26,
                0x1005,
                0x0C2F,
                0x0C52,
                0x0C1A,
            ],
        )
        self.assertEqual(self.lib.demo_flags(), 1)

    def connect(self):
        self.peer = bytes.fromhex("112233445566")
        self.event(4, self.peer + bytes([8, 4, 0x20, 1]))
        p = self.pop()
        self.assertEqual(p[1:3], le(0x0409))
        self.event(0x0F, bytes([0, 1]) + le(0x0409))
        self.event(3, b"\0" + le(1) + self.peer + b"\1\0")
        self.complete(self.pop())

    def l2cap(self, cid, payload, split=None):
        pdu = le(len(payload), cid) + payload
        if split:
            self.receive(b"\2" + le(0x2001, split) + pdu[:split])
            self.receive(b"\2" + le(0x1001, len(pdu) - split) + pdu[split:])
        else:
            self.receive(b"\2" + le(0x2001, len(pdu)) + pdu)

    def drain(self):
        result = []
        while packet := self.pop():
            if packet[0] == 1:
                self.complete(packet)
            else:
                self.assertEqual(packet[0], 2)
                result.append((int.from_bytes(packet[7:9], "little"), packet[9:]))
                self.event(0x13, b"\1" + le(1, 1))
        return result

    def channel(self, psm, remote=0x80, local=0x40):
        self.l2cap(1, b"\2\1" + le(4, psm, remote), split=5)
        responses = self.drain()
        self.assertEqual(responses[0], (1, b"\3\1" + le(8, local, remote, 0, 0)))
        config = responses[1][1]
        self.l2cap(1, bytes([5, config[1]]) + le(6, local, 0, 0))
        self.l2cap(1, b"\4\2" + le(8, local, 0) + b"\1\2" + le(672))
        self.drain()

    def rf_receive(self, frame):
        self.l2cap(0x40, bytes(frame))

    def frames(self):
        return [
            rfcomm.RFCOMM_Frame.from_bytes(data)
            for cid, data in self.drain()
            if cid == 0x80
        ]

    def rf_open(self):
        self.connect()
        self.channel(3)
        self.rf_receive(rfcomm.RFCOMM_Frame.sabm(1, 0))
        self.assertEqual(self.frames()[0].type, rfcomm.FrameType.UA)
        pn = bytes([2, 0xF0, 7, 0, 127, 0, 0, 7])
        mcc = rfcomm.RFCOMM_Frame.make_mcc(rfcomm.MccType.PN, 1, pn)
        self.rf_receive(rfcomm.RFCOMM_Frame.uih(c_r=1, dlci=0, p_f=0, information=mcc))
        response = self.frames()[0]
        self.assertEqual(response.information[3], 0xE0)
        self.rf_receive(rfcomm.RFCOMM_Frame.sabm(1, 2))
        frames = self.frames()
        self.assertEqual(frames[0].type, rfcomm.FrameType.UA)
        self.assertEqual(frames[-1].information, b"AT+BRSF=0\r")

    def at(self, text):
        self.rf_receive(
            rfcomm.RFCOMM_Frame.uih(c_r=1, dlci=2, p_f=0, information=text.encode())
        )
        return [f.information for f in self.frames() if f.dlci == 2 and not f.p_f]

    def ready(self):
        self.rf_open()
        self.assertEqual(self.at("\r\n+BRSF: 512\r\nOK\r\n"), [b"AT+CIND=?\r"])
        self.assertEqual(
            self.at(
                '\r\n+CIND: ("service",(0,1)),("call",(0,1)),("callsetup",(0-3))\r\nOK\r\n'
            ),
            [b"AT+CIND?\r"],
        )
        self.assertEqual(self.at("\r\n+CIND: 1,0,0\r\nOK\r\n"), [b"AT+CMER=3,0,0,1\r"])
        self.assertEqual(self.at("\r\nOK\r\n"), [])
        self.assertTrue(self.lib.demo_flags() & 4)

    def test_dial_answer_hangup_and_disconnect(self):
        self.ready()
        self.assertTrue(self.lib.demo_dial(b"+15551234567"))
        self.assertEqual(self.frames()[0].information, b"ATD+15551234567;\r")
        self.assertFalse(self.lib.demo_dial(b"123"))
        self.at("\r\nOK\r\n+CIEV: 3,2\r\n")
        self.assertTrue(self.lib.demo_hangup())
        self.assertEqual(self.frames()[0].information, b"AT+CHUP\r")
        self.at("\r\nOK\r\n+CIEV: 3,0\r\nRING\r\n")
        self.assertTrue(self.lib.demo_answer())
        self.assertEqual(self.frames()[0].information, b"ATA\r")
        self.at("\r\nOK\r\n+CIEV: 2,1\r\n")
        self.assertTrue(self.lib.demo_flags() & 8)
        self.event(5, b"\0" + le(1) + b"\x13")
        self.drain()
        self.assertEqual(self.lib.demo_flags(), 1)
        self.assertFalse(self.lib.demo_dial(b"123"))

    def test_numbers_cannot_inject_at_commands(self):
        self.ready()
        for number in (b"", b"+", b"12\rATA", b"123;", b"++123", b"1" * 33):
            self.assertFalse(self.lib.demo_dial(number))
        self.assertEqual(self.frames(), [])

    def test_pairing_key_is_ram_only(self):
        self.connect()
        self.event(0x17, self.peer)
        packet = self.pop()
        self.assertEqual(packet[1:3], le(0x040C))
        self.complete(packet)
        key = bytes(range(16))
        self.event(0x18, self.peer + key + b"\4")
        self.event(0x17, self.peer)
        packet = self.pop()
        self.assertEqual(packet[1:3], le(0x040B))
        self.assertEqual(packet[10:], key)
        self.lib.demo_init()
        self.boot()
        self.event(0x17, self.peer)
        self.assertEqual(self.pop()[1:3], le(0x040C))

    def test_sdp_search_attribute_record_is_decodable(self):
        self.connect()
        self.channel(1)
        request = bytes(
            sdp.SDP_ServiceSearchAttributeRequest(
                transaction_id=7,
                service_search_pattern=sdp.DataElement.sequence(
                    [sdp.DataElement.uuid(core.UUID.from_16_bits(0x111E))]
                ),
                maximum_attribute_byte_count=512,
                attribute_id_list=sdp.DataElement.sequence(
                    [sdp.DataElement.unsigned_integer_32(0x0000FFFF)]
                ),
                continuation_state=b"\0",
            )
        )
        self.l2cap(0x40, request)
        response = sdp.SDP_PDU.from_bytes(self.drain()[0][1])
        self.assertEqual(response.transaction_id, 7)
        self.assertEqual(response.continuation_state, b"\0")
        record = sdp.DataElement.from_bytes(response.attribute_lists)
        self.assertIn("111E", str(record).upper())

    def test_command_timeout_and_bad_acl_are_bounded(self):
        self.rf_open()
        errors = self.lib.demo_errors()
        self.receive(b"\2" + le(0x2001, 6) + le(0xFFFF, 0x40) + b"XX")
        self.assertEqual(self.frames(), [])
        self.lib.demo_tick(self.now + 10001)
        self.assertGreater(self.lib.demo_errors(), errors)
        self.assertFalse(self.lib.demo_flags() & 4)

    def test_bad_rfcomm_fcs_cannot_advance_handshake(self):
        self.rf_open()
        frame = bytearray(bytes(rfcomm.RFCOMM_Frame.uih(1, 2, b"\r\nOK\r\n")))
        frame[-1] ^= 1
        self.l2cap(0x40, frame)
        self.assertEqual(self.frames(), [])
        self.assertEqual(self.lib.demo_errors(), 1)
        self.assertEqual(self.at("\r\nOK\r\n"), [b"AT+CIND=?\r"])

    def test_fragmented_at_response_and_credit_replenishment(self):
        self.rf_open()
        self.assertEqual(self.at("\r\n+BRS"), [])
        self.assertEqual(self.at("F: 512\r\nO"), [])
        self.assertEqual(self.at("K\r\n"), [b"AT+CIND=?\r"])
        # The peer replenishes our transmit credits in an otherwise empty frame.
        self.rf_receive(rfcomm.RFCOMM_Frame.uih(1, 2, b"\7", p_f=1))
        self.assertEqual(self.frames(), [])
        self.assertEqual(self.lib.demo_errors(), 0)

    def test_sdp_continuation_respects_peer_limit(self):
        self.connect()
        self.channel(1)
        continuation = b"\0"
        record = bytearray()
        for transaction in range(30):
            request = sdp.SDP_ServiceSearchAttributeRequest(
                transaction_id=transaction,
                service_search_pattern=sdp.DataElement.sequence(
                    [sdp.DataElement.uuid(core.UUID.from_16_bits(0x111E))]
                ),
                maximum_attribute_byte_count=7,
                attribute_id_list=sdp.DataElement.sequence(
                    [sdp.DataElement.unsigned_integer_32(0x0000FFFF)]
                ),
                continuation_state=continuation,
            )
            self.l2cap(0x40, bytes(request))
            response = sdp.SDP_PDU.from_bytes(self.drain()[0][1])
            self.assertLessEqual(len(response.attribute_lists), 7)
            record.extend(response.attribute_lists)
            continuation = response.continuation_state
            if continuation == b"\0":
                break
        else:
            self.fail("SDP continuation did not terminate")
        self.assertIn("111E", str(sdp.DataElement.from_bytes(record)).upper())

    def test_reconnect_keeps_key_but_resets_protocol(self):
        self.ready()
        self.event(0x18, self.peer + bytes(range(16)) + b"\4")
        self.event(5, b"\0" + le(1) + b"\x13")
        self.drain()
        self.assertFalse(self.lib.demo_flags() & 4)
        self.event(0x17, self.peer)
        packet = self.pop()
        self.assertEqual(packet[1:3], le(0x040B))
        self.complete(packet)
        self.ready()


if __name__ == "__main__":
    unittest.main()

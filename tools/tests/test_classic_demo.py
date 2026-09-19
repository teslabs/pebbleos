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
    managed = False

    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        library = Path(cls.directory.name) / "classic.so"
        sources = ROOT / "src/bluetooth-fw/classic"
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
                "-I" + str(sources),
                str(Path(__file__).with_name("classic_demo_harness.c")),
                *(
                    str(sources / f)
                    for f in ("host.c", "l2cap.c", "profile.c", "sdp.c")
                ),
                "-o",
                str(library),
            ],
            check=True,
        )
        cls.lib = ctypes.CDLL(str(library))
        cls.lib.demo_detail.restype = ctypes.c_char_p
        cls.lib.demo_caller_number.restype = ctypes.c_char_p
        cls.lib.demo_waiting_number.restype = ctypes.c_char_p

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def setUp(self):
        self.now = 0
        if self.managed:
            self.lib.demo_init_managed()
        else:
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

    def test_name_changes_update_name_and_eir(self):
        for requested in (b"Pebble test", b"x" * 100, b"Pebble test"):
            self.lib.demo_set_local_name(requested)
            name = requested[:63]
            packet = self.pop()
            self.assertEqual(packet[:4], b"\x01\x13\x0c\xf8")
            self.assertEqual(packet[4:].rstrip(b"\0"), name)
            self.complete(packet)
            packet = self.pop()
            self.assertEqual(packet[:5], b"\x01\x52\x0c\xf1\0")
            self.assertEqual(packet[5:7], bytes([len(name) + 1, 9]))
            self.assertEqual(packet[7 : 7 + len(name)], name)
            self.assertEqual(packet[7 + len(name) : 11 + len(name)], b"\3\3\x1e\x11")
            self.complete(packet)
            self.lib.demo_set_local_name(requested)
            self.assertFalse(self.pop())

    def boot(self):
        opcodes = []
        while packet := self.pop():
            self.assertEqual(packet[0], 1)
            opcodes.append(int.from_bytes(packet[1:3], "little"))
            if opcodes[-1] == 0x0C13:
                self.assertEqual(packet[4:].rstrip(b"\0"), b"Pebble")
            elif opcodes[-1] == 0x0C52:
                self.assertEqual(packet[5:13], b"\7\x09Pebble")
                self.assertEqual(packet[13:17], b"\3\3\x1e\x11")
            elif opcodes[-1] == 0x080F:
                self.assertEqual(packet[4:], le(1))
            self.complete(packet)
        expected = [
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
            0x080F,
            0x0C1A,
        ]
        if self.managed:
            expected = [op for op in expected if op not in (0x0C03, 0x0C01, 0x1005)]
        self.assertEqual(opcodes, expected)
        self.assertEqual(self.lib.demo_flags(), 1)

    def test_role_policy_unsupported_keeps_hfp_available(self):
        for status in (0x01, 0x11):
            with self.subTest(status=status):
                if self.managed:
                    self.lib.demo_init_managed()
                else:
                    self.lib.demo_init()
                saw_policy = False
                while packet := self.pop():
                    opcode = int.from_bytes(packet[1:3], "little")
                    if opcode == 0x080F:
                        saw_policy = True
                        self.event(0x0E, b"\1" + le(opcode) + bytes([status]))
                    else:
                        self.complete(packet)
                self.assertTrue(saw_policy)
                self.assertEqual(self.lib.demo_flags(), 1)
                self.assertEqual(self.lib.demo_errors(), 0)

    def test_role_policy_other_failure_stops_startup(self):
        if self.managed:
            self.lib.demo_init_managed()
        else:
            self.lib.demo_init()
        while packet := self.pop():
            opcode = int.from_bytes(packet[1:3], "little")
            if opcode == 0x080F:
                self.event(0x0E, b"\1" + le(opcode) + b"\x0c")
                break
            self.complete(packet)
        else:
            self.fail("Role policy command was not issued")
        self.assertFalse(self.pop())
        self.assertEqual(self.lib.demo_flags(), 0)
        self.assertEqual(self.lib.demo_errors(), 1)

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

    def rf_open(self, secure=False):
        if secure:
            self.lib.demo_shared_bond(1)
        self.connect()
        if secure:
            self.drain()
            self.event(8, b"\0" + le(1) + b"\1")
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
        self.assertEqual(frames[-1].information, b"AT+BRSF=54\r")

    def at(self, text):
        self.rf_receive(
            rfcomm.RFCOMM_Frame.uih(c_r=1, dlci=2, p_f=0, information=text.encode())
        )
        return [f.information for f in self.frames() if f.dlci == 2 and not f.p_f]

    def ready(self, secure=False, features="512"):
        self.rf_open(secure=secure)
        self.assertEqual(self.at(f"\r\n+BRSF: {features}\r\nOK\r\n"), [b"AT+CIND=?\r"])
        self.assertEqual(
            self.at(
                '\r\n+CIND: ("service",(0,1)),("call",(0,1)),("callsetup",(0-3))\r\nOK\r\n'
            ),
            [b"AT+CIND?\r"],
        )
        self.assertEqual(self.at("\r\n+CIND: 1,0,0\r\nOK\r\n"), [b"AT+CMER=3,0,0,1\r"])
        self.assertEqual(self.at("\r\nOK\r\n"), [b"AT+CLIP=1\r"])
        self.assertEqual(self.at("\r\nOK\r\n"), [b"AT+VGS=15\r"])
        self.assertEqual(self.at("\r\nOK\r\n"), [])
        self.assertTrue(self.lib.demo_flags() & 4)
        self.rf_receive(rfcomm.RFCOMM_Frame.uih(1, 2, b"\7", p_f=1))
        self.frames()

    def ready_three_way(self, enhanced=True, hold="(0,1,1x,2,2x,3)"):
        self.rf_open()
        self.assertEqual(
            self.at(f"+BRSF: {65 if enhanced else 1}\r\nOK\r\n"), [b"AT+CIND=?\r"]
        )
        self.assertEqual(
            self.at(
                '+CIND: ("call",(0,1)),("callsetup",(0-3)),("callheld",(0-2))\r\nOK\r\n'
            ),
            [b"AT+CIND?\r"],
        )
        self.assertEqual(self.at("+CIND: 0,0,0\r\nOK\r\n"), [b"AT+CMER=3,0,0,1\r"])
        self.assertEqual(self.at("OK\r\n"), [b"AT+CHLD=?\r"])
        self.assertEqual(self.at(f"+CHLD: {hold}\r\nOK\r\n"), [b"AT+CLIP=1\r"])
        self.assertEqual(self.at("OK\r\n"), [b"AT+CCWA=1\r"])
        self.rf_receive(rfcomm.RFCOMM_Frame.uih(1, 2, b"\7", p_f=1))
        self.frames()
        self.assertEqual(self.at("OK\r\n"), [b"AT+VGS=15\r"])
        self.assertEqual(self.at("OK\r\n"), [b"AT+CLCC\r"] if enhanced else [])
        if enhanced:
            self.assertEqual(self.at("OK\r\n"), [])
        self.assertTrue(self.lib.demo_flags() & 4)

    def test_waiting_answer_holds_active_call(self):
        self.ready_three_way(enhanced=False)
        self.at('+CIEV: 1,1\r\n+CLIP: "12025550100",145\r\n')
        self.at('+CCWA: "12025550101",145,1\r\n')
        self.assertTrue(self.lib.demo_waiting())
        self.assertEqual(self.lib.demo_caller_number(), b"+12025550100")
        self.assertEqual(self.lib.demo_waiting_number(), b"+12025550101")
        self.assertTrue(self.lib.demo_answer())
        self.assertEqual(
            [f.information for f in self.frames() if f.dlci == 2], [b"AT+CHLD=2\r"]
        )
        self.at("OK\r\n+CIEV: 2,0\r\n+CIEV: 3,1\r\n")
        self.assertFalse(self.lib.demo_waiting())
        self.assertEqual(self.lib.demo_call_held(), 1)
        self.assertEqual(self.lib.demo_caller_number(), b"")
        self.assertTrue(self.lib.demo_call_hold(3))
        self.assertEqual(
            [f.information for f in self.frames() if f.dlci == 2], [b"AT+CHLD=3\r"]
        )

    def test_waiting_identity_rejects_malformed_or_nonvoice_notifications(self):
        self.ready_three_way(enhanced=False)
        self.at("+CIEV: 1,1\r\n")
        for text in (
            '"123",145',
            '"123",145,2',
            '"123",145,1junk',
            '"123;ATD",145,1',
            '"123",-1,1',
        ):
            self.at(f"+CCWA: {text}\r\n")
            self.assertFalse(self.lib.demo_waiting())
            self.assertEqual(self.lib.demo_waiting_number(), b"")
        self.at('+CCWA: "123",145,1,"",1\r\n')
        self.assertTrue(self.lib.demo_waiting())
        self.assertEqual(self.lib.demo_waiting_number(), b"")

    def test_current_call_list_publishes_only_after_ok(self):
        self.ready_three_way()
        self.assertEqual(self.at("+CIEV: 1,1\r\n"), [b"AT+CLCC\r"])
        self.at('+CLCC: 1,0,0,0,0,"12025550100",145\r\n')
        self.assertEqual(self.lib.demo_caller_number(), b"")
        self.at("OK\r\n")
        self.assertEqual(self.lib.demo_caller_number(), b"+12025550100")
        self.assertEqual(self.at("+CIEV: 3,1\r\n"), [b"AT+CLCC\r"])
        self.at('+CLCC: 1,0,1,0,0,"12025550100",145\r\n')
        self.at('+CLCC: 2,1,0,0,0,"12025550101",145\r\nOK\r\n')
        self.assertEqual(self.lib.demo_caller_number(), b"+12025550101")

    def test_indicator_during_call_query_discards_obsolete_snapshot(self):
        self.ready_three_way()
        self.assertEqual(self.at("+CIEV: 1,1\r\n"), [b"AT+CLCC\r"])
        self.at('+CLCC: 1,0,0,0,0,"123",129\r\n')
        self.at("+CIEV: 1,0\r\n")
        self.assertEqual(self.at("OK\r\n"), [b"AT+CLCC\r"])
        self.assertEqual(self.lib.demo_caller_number(), b"")
        self.at("OK\r\n")
        self.assertEqual(self.lib.demo_caller_number(), b"")

    def test_invalid_call_lists_do_not_publish_partial_identity(self):
        self.ready_three_way()
        for invalid in (
            "0,0,0,0,0",
            "-1,0,0,0,0",
            "999999999999999,0,0,0,0",
            "2,0,9,0,0",
            '2,0,0,0,0,"123;ATD",129',
            "1,0,0,0,0",
        ):
            self.assertEqual(self.at("+CIEV: 1,1\r\n"), [b"AT+CLCC\r"])
            self.at('+CLCC: 1,0,0,0,0,"123",129\r\n')
            self.at(f"+CLCC: {invalid}\r\nOK\r\n")
            self.assertEqual(self.lib.demo_caller_number(), b"")
            self.rf_receive(rfcomm.RFCOMM_Frame.uih(1, 2, b"\7", p_f=1))
            self.frames()

    def test_call_hold_only_uses_advertised_actions(self):
        self.ready_three_way(enhanced=False, hold="(1,2)")
        self.at("+CIEV: 1,1\r\n+CIEV: 2,1\r\n")
        self.assertFalse(self.lib.demo_call_hold(0))
        self.assertFalse(self.lib.demo_call_hold(3))
        self.assertFalse(self.lib.demo_call_hold(4))
        self.assertTrue(self.lib.demo_call_hold(1))
        self.assertEqual(
            [f.information for f in self.frames() if f.dlci == 2], [b"AT+CHLD=1\r"]
        )

    def test_negative_ag_features_do_not_enable_call_waiting(self):
        self.ready(features="-1")
        self.at("+CIEV: 2,1\r\n")
        self.at('+CCWA: "123",129,1\r\n')
        self.assertFalse(self.lib.demo_waiting())
        self.assertFalse(self.lib.demo_call_hold(2))

    def test_call_list_overflow_does_not_publish_a_partial_list(self):
        self.ready_three_way()
        self.assertEqual(self.at("+CIEV: 1,1\r\n"), [b"AT+CLCC\r"])
        for index in range(1, 6):
            self.at(f'+CLCC: {index},0,0,0,0,"123",129\r\n')
        self.at("OK\r\n")
        self.assertEqual(self.lib.demo_caller_number(), b"")

    def test_remote_speaker_gain_is_bounded_and_not_echoed(self):
        self.ready()
        for gain in (0, 7, 15):
            self.assertEqual(self.at(f"+VGS: {gain}\r\n"), [])
            self.assertEqual(self.lib.demo_speaker_gain(), gain)
        for invalid in ("-1", "16", "4294967296", "", "7junk", "7,2"):
            self.assertEqual(self.at(f"+VGS: {invalid}\r\n"), [])
            self.assertEqual(self.lib.demo_speaker_gain(), 15)

    def test_remote_gain_preserves_a_queued_local_change(self):
        self.ready()
        self.assertTrue(self.lib.demo_set_speaker_gain(7))
        self.assertEqual(self.frames()[0].information, b"AT+VGS=7\r")
        self.assertTrue(self.lib.demo_set_speaker_gain(3))
        self.assertEqual(self.at("+VGS: 15\r\nOK\r\n"), [b"AT+VGS=3\r"])
        self.assertEqual(self.lib.demo_speaker_gain(), 3)
        self.assertEqual(self.at("+VGS: 7\r\nOK\r\n"), [])
        self.assertEqual(self.lib.demo_speaker_gain(), 7)
        self.at("+VGS: 9\r\n")
        self.assertEqual(self.lib.demo_speaker_gain(), 9)

    def test_local_speaker_gain_coalesces_while_command_pending(self):
        self.assertFalse(self.lib.demo_set_speaker_gain(7))
        self.ready()
        self.assertFalse(self.lib.demo_set_speaker_gain(16))
        self.assertTrue(self.lib.demo_dial(b"+12025550100"))
        self.frames()
        self.assertTrue(self.lib.demo_set_speaker_gain(8))
        self.assertTrue(self.lib.demo_set_speaker_gain(9))
        self.assertFalse(self.frames())
        self.assertEqual(self.at("OK\r\n"), [b"AT+VGS=9\r"])
        self.assertEqual(self.at("OK\r\n"), [])
        self.assertTrue(self.lib.demo_set_speaker_gain(9))
        self.assertFalse(self.frames())

    def test_caller_number_is_normalized_and_cleared(self):
        self.ready()
        self.at('\r\n+CIEV: 3,1\r\nRING\r\n+CLIP: "12025550100",145\r\n')
        self.assertEqual(self.lib.demo_caller_number(), b"+12025550100")
        self.at("\r\n+CIEV: 2,1\r\n+CIEV: 3,0\r\n")
        self.assertEqual(self.lib.demo_caller_number(), b"+12025550100")
        self.at("\r\n+CIEV: 2,0\r\n")
        self.assertEqual(self.lib.demo_caller_number(), b"")

    def test_invalid_and_withheld_caller_identity(self):
        self.ready()
        self.at('\r\nRING\r\n+CLIP: "+12025550100",145\r\n')
        for line in (
            '+CLIP: "555;ATD123",129',
            '+CLIP: "123",999',
            '+CLIP: "' + "1" * 33 + '",129',
            '+CLIP: "123",129junk',
        ):
            self.at("\r\n" + line + "\r\n")
            self.assertEqual(self.lib.demo_caller_number(), b"+12025550100")
        self.at('\r\n+CLIP: "+12025550100",145,"",128,"Hidden",1\r\n')
        self.assertEqual(self.lib.demo_caller_number(), b"")

    def active_on_phone(self):
        self.ready(secure=True)
        self.at("+CIEV: 2,1\r\n")

    def audio_complete(self, status=0, peer=None):
        self.event(
            0x2C,
            bytes([status])
            + le(0x180)
            + (peer or self.peer)
            + bytes([2, 6, 1])
            + le(30, 30)
            + b"\2",
        )

    def test_audio_transfer_requires_encrypted_call(self):
        self.ready()
        self.at("+CIEV: 2,1\r\n")
        self.assertFalse(self.lib.demo_transfer_audio(1))
        self.assertFalse(self.pop())

    def test_foreign_audio_connection_does_not_replace_transfer(self):
        self.active_on_phone()
        self.assertTrue(self.lib.demo_transfer_audio(1))
        self.pop()
        self.event(0x0F, bytes([0, 1]) + le(0x0428))
        self.audio_complete(peer=b"\x55" * 6)
        self.assertEqual(self.lib.demo_audio_state(), 2)
        self.assertEqual(self.pop(), b"\1" + le(0x0406) + b"\3" + le(0x180) + b"\x13")

    def test_audio_transfer_preserves_call_and_acl(self):
        self.assertFalse(self.lib.demo_transfer_audio(1))
        self.active_on_phone()
        self.assertTrue(self.lib.demo_transfer_audio(1))
        self.assertEqual(self.lib.demo_audio_state(), 2)
        self.assertFalse(self.lib.demo_transfer_audio(1))
        command = self.pop()
        self.assertEqual(
            command,
            b"\1"
            + le(0x0428)
            + b"\x11"
            + le(1)
            + struct.pack("<IIHHBH", 8000, 8000, 7, 0x60, 1, 0x03C8),
        )
        self.event(0x0F, bytes([0, 1]) + le(0x0428))
        self.audio_complete()
        self.assertEqual(self.lib.demo_audio_state(), 1)
        self.assertTrue(self.lib.demo_transfer_audio(0))
        self.assertEqual(self.pop(), b"\1" + le(0x0406) + b"\3" + le(0x180) + b"\x13")
        self.event(0x0F, bytes([0, 1]) + le(0x0406))
        self.event(5, b"\0" + le(0x180) + b"\x16")
        self.assertEqual(self.lib.demo_audio_state(), 0)
        self.assertEqual(self.lib.demo_flags() & 14, 14)
        self.assertTrue(self.lib.demo_transfer_audio(0))
        self.assertFalse(self.pop())

    def test_audio_transfer_failure_allows_retry(self):
        self.active_on_phone()
        self.assertTrue(self.lib.demo_transfer_audio(1))
        self.pop()
        self.event(0x0F, bytes([0x0C, 1]) + le(0x0428))
        self.assertEqual(self.lib.demo_audio_state(), 0)
        self.assertTrue(self.lib.demo_transfer_audio(1))
        self.pop()
        self.event(0x0F, bytes([0, 1]) + le(0x0428))
        self.audio_complete(status=0x10, peer=b"\0" * 6)
        self.assertEqual(self.lib.demo_audio_state(), 0)
        self.assertTrue(self.lib.demo_transfer_audio(1))

    def test_audio_transfer_timeout_disconnects_before_retry(self):
        self.active_on_phone()
        self.assertTrue(self.lib.demo_transfer_audio(1))
        self.pop()
        self.event(0x0F, bytes([0, 1]) + le(0x0428))
        self.now += 10001
        self.tick()
        self.assertFalse(self.lib.demo_flags() & 4)
        self.assertEqual(self.lib.demo_audio_state(), 0)
        self.assertFalse(self.lib.demo_transfer_audio(1))
        self.assertEqual(self.pop(), b"\1" + le(0x0406) + b"\3" + le(1) + b"\x13")

    def test_audio_transfer_finishing_after_call_end_is_released(self):
        self.active_on_phone()
        self.assertTrue(self.lib.demo_transfer_audio(1))
        self.pop()
        self.event(0x0F, bytes([0, 1]) + le(0x0428))
        self.at("+CIEV: 2,0\r\n")
        self.audio_complete()
        self.assertEqual(self.pop(), b"\1" + le(0x0406) + b"\3" + le(0x180) + b"\x13")

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

    def test_oversized_line_cannot_inject_a_response_suffix(self):
        self.rf_open()
        for _ in range(6):
            self.assertEqual(self.at("X" * 100), [])
        self.assertEqual(self.at("OK\r\n"), [])
        self.assertEqual(self.lib.demo_errors(), 1)
        self.assertEqual(self.at("OK\r\n"), [b"AT+CIND=?\r"])

    def test_nul_cannot_hide_trailing_response_data(self):
        self.rf_open()
        self.assertEqual(self.at("OK\0junk\r\n"), [])
        self.assertEqual(self.lib.demo_errors(), 1)
        self.assertEqual(self.at("OK\r\n"), [b"AT+CIND=?\r"])

    def test_late_ok_after_timeout_cannot_resume_calls(self):
        self.ready()
        self.assertTrue(self.lib.demo_dial(b"5550100"))
        self.frames()
        self.now += 10001
        self.lib.demo_tick(self.now)
        packet = self.pop()
        self.assertEqual(packet[1:3], le(0x0406))
        self.complete(packet)
        self.assertEqual(self.at("OK\r\n+CIEV: 2,1\r\n"), [])
        self.assertFalse(self.lib.demo_flags() & (4 | 8))
        self.assertFalse(self.lib.demo_dial(b"5550100"))

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
        if self.managed:
            self.lib.demo_init_managed()
        else:
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


class ManagedClassicHostTest(ClassicHostTest):
    managed = True

    def test_full_output_queue_closes_session(self):
        self.ready()
        self.lib.demo_acl_ready(False)
        echo = rfcomm.RFCOMM_Frame.make_mcc(0x08, 1, b"test")
        for _ in range(13):
            self.rf_receive(rfcomm.RFCOMM_Frame.uih(1, 0, echo))
        packet = self.pop()
        self.assertEqual(packet[1:3], le(0x0406))
        self.assertFalse(self.lib.demo_flags() & 4)
        self.assertFalse(self.lib.demo_dial(b"5550100"))
        self.assertEqual(self.lib.demo_errors(), 1)

    def test_shutdown_does_not_reenable_scan(self):
        self.ready()
        self.lib.demo_stop()
        commands = []
        while packet := self.pop():
            commands.append(packet)
            self.complete(packet)
        self.assertEqual([p[1:3] for p in commands], [le(0x0C1A), le(0x0406)])
        self.assertEqual(commands[0][4:], b"\0")
        self.assertFalse(self.lib.demo_stopped())
        self.event(5, b"\0" + le(1) + b"\x13")
        packet = self.pop()
        self.assertEqual(packet[1:3], le(0x0C1A))
        self.assertEqual(packet[4:], b"\0")
        self.complete(packet)
        self.assertTrue(self.lib.demo_stopped())
        self.assertFalse(self.lib.demo_dial(b"123"))

    def test_shutdown_rejects_a_queued_connection_request(self):
        peer = bytes.fromhex("112233445566")
        packet = bytes([4, 4, 10]) + peer + bytes([8, 4, 0x20, 1])
        # Do not poll: the accept command has not reached the shared host yet.
        self.lib.demo_receive(packet, len(packet))
        self.lib.demo_stop()
        commands = []
        while packet := self.pop():
            commands.append(int.from_bytes(packet[1:3], "little"))
            self.complete(packet)
        self.assertEqual(commands, [0x0C1A, 0x040A])
        self.assertTrue(self.lib.demo_stopped())

    def test_shutdown_waits_for_accepted_connection_to_finish(self):
        peer = bytes.fromhex("112233445566")
        self.event(4, peer + bytes([8, 4, 0x20, 1]))
        self.assertEqual(self.pop()[1:3], le(0x0409))
        self.event(0x0F, bytes([0, 1]) + le(0x0409))
        self.lib.demo_stop()
        self.complete(self.pop())
        self.assertFalse(self.lib.demo_stopped())
        self.event(3, b"\0" + le(1) + peer + b"\1\0")
        commands = []
        while packet := self.pop():
            commands.append(int.from_bytes(packet[1:3], "little"))
            self.complete(packet)
        self.assertIn(0x0406, commands)
        self.event(5, b"\0" + le(1) + b"\x13")
        self.complete(self.pop())
        self.assertTrue(self.lib.demo_stopped())

    def test_shutdown_rejects_new_connections(self):
        self.lib.demo_stop()
        self.complete(self.pop())
        peer = bytes.fromhex("112233445566")
        self.event(4, peer + bytes([8, 4, 0x20, 1]))
        packet = self.pop()
        self.assertEqual(packet[1:3], le(0x040A))
        self.complete(packet)
        self.assertTrue(self.lib.demo_stopped())

    def test_acl_backpressure_preserves_signaling(self):
        self.connect()
        self.lib.demo_acl_ready(0)
        self.l2cap(1, b"\2\1" + le(4, 3, 0x80))
        for _ in range(5):
            self.assertEqual(self.pop(), b"")
        self.lib.demo_acl_ready(1)
        responses = self.drain()
        self.assertEqual(len(responses), 2)
        self.assertEqual(responses[0], (1, b"\3\1" + le(8, 0x40, 0x80, 0, 0)))


if __name__ == "__main__":
    unittest.main()


class SharedBondTest(ClassicHostTest):
    managed = True

    def test_shared_link_key_lookup(self):
        self.lib.demo_shared_bond(1)
        peer = bytes.fromhex("112233445566")
        self.event(0x17, peer)
        p = self.pop()
        self.assertEqual(p, b"\1" + le(0x040B) + b"\x16" + peer + b"\xa5" * 16)
        self.complete(p)
        self.lib.demo_shared_bond(0)
        self.event(0x17, peer)
        self.assertEqual(self.pop(), b"\1" + le(0x040C) + b"\6" + peer)

    def test_separate_pairing_rejected(self):
        self.lib.demo_shared_bond(1)
        peer = bytes.fromhex("112233445566")
        self.event(0x31, peer)
        p = self.pop()
        self.assertEqual(p, b"\1" + le(0x0434) + b"\7" + peer + b"\x18")
        self.complete(p)
        # A native key notification cannot replace the shared key.
        self.event(0x18, peer + b"\x42" * 16 + b"\5")
        self.event(0x17, peer)
        self.assertEqual(self.pop()[-16:], b"\xa5" * 16)

    def test_rfcomm_blocked_without_encryption(self):
        self.lib.demo_shared_bond(1)
        self.connect()
        self.drain()
        self.l2cap(1, b"\2\1" + le(4, 3, 0x80))
        self.assertEqual(self.drain(), [(1, b"\3\1" + le(8, 0, 0x80, 3, 0))])
        self.event(8, b"\0" + le(1) + b"\1")
        self.assertEqual(self.lib.demo_encrypted(), 1)
        self.channel(3)

    def test_forgetting_bond_disconnects_classic(self):
        self.lib.demo_shared_bond(1)
        self.connect()
        self.drain()
        self.event(8, b"\0" + le(1) + b"\1")
        self.lib.demo_shared_bond(0)
        p = self.pop()
        self.assertEqual(p, b"\1" + le(0x0406) + b"\3" + le(1) + b"\x13")
        self.assertEqual(self.lib.demo_encrypted(), 0)

    def test_replacing_bond_disconnects_old_encrypted_link(self):
        self.lib.demo_shared_bond(1)
        self.connect()
        self.drain()
        self.event(8, b"\0" + le(1) + b"\1")
        self.assertEqual(self.lib.demo_encrypted(), 1)
        self.lib.demo_replace_bond()
        p = self.pop()
        self.assertEqual(p, b"\1" + le(0x0406) + b"\3" + le(1) + b"\x13")
        self.assertEqual(self.lib.demo_encrypted(), 0)
        self.complete(p)
        self.event(8, b"\0" + le(1) + b"\1")
        self.assertEqual(self.lib.demo_encrypted(), 0)

    def test_replacing_bond_during_authentication_rejects_link(self):
        self.lib.demo_shared_bond(1)
        self.connect()
        self.drain()
        self.lib.demo_replace_bond()
        self.event(6, b"\0" + le(1))
        self.assertEqual(self.pop(), b"\1" + le(0x0406) + b"\3" + le(1) + b"\x13")
        self.assertEqual(self.lib.demo_encrypted(), 0)


class ReconnectTest(ClassicHostTest):
    managed = True

    def start_outgoing(self):
        self.peer = bytes.fromhex("112233445566")
        self.lib.demo_shared_bond(1)
        self.assertTrue(self.lib.demo_connect())
        self.assertFalse(self.lib.demo_connect())
        p = self.pop()
        self.assertEqual(p[1:3], le(0x0405))
        self.assertEqual(p[4:10], self.peer)
        self.event(0x0F, b"\0\1" + le(0x0405))
        self.event(3, b"\0" + le(1) + self.peer + b"\1\0")
        self.drain()
        self.event(6, b"\0" + le(1))
        self.drain()
        self.event(8, b"\0" + le(1) + b"\1")
        return self.drain()[0][1]

    def accept_channel(self, request, local, remote, psm):
        self.assertEqual(request[0], 2)
        self.assertEqual(request[4:], le(psm, local))
        self.l2cap(1, bytes([3, request[1]]) + le(8, remote, local, 0, 0))
        config = self.drain()[0][1]
        self.assertEqual(config[0], 4)
        self.l2cap(1, bytes([5, config[1]]) + le(6, local, 0, 0))
        self.l2cap(1, b"\4\x70" + le(8, local, 0) + b"\1\2" + le(672))
        return self.drain()

    def discovery(self):
        response = self.accept_channel(self.start_outgoing(), 0x40, 0x80, 1)
        return sdp.SDP_PDU.from_bytes(response[-1][1])

    def service_record(self, channel=7):
        seq = sdp.DataElement.sequence
        uuid = lambda value: sdp.DataElement.uuid(core.UUID.from_16_bits(value))
        return bytes(
            seq(
                [
                    seq(
                        [
                            sdp.DataElement.unsigned_integer_16(4),
                            seq(
                                [
                                    seq([uuid(0x100)]),
                                    seq(
                                        [
                                            uuid(3),
                                            sdp.DataElement.unsigned_integer_8(channel),
                                        ]
                                    ),
                                ]
                            ),
                        ]
                    )
                ]
            )
        )

    def sdp_response(self, request, data, continuation=b"\0"):
        self.l2cap(
            0x40,
            bytes(
                sdp.SDP_ServiceSearchAttributeResponse(
                    transaction_id=request.transaction_id,
                    attribute_lists=data,
                    continuation_state=continuation,
                )
            ),
        )
        return self.drain()

    def outgoing_ready(self):
        request = self.discovery()
        self.assertIn("111F", str(request.service_search_pattern).upper())
        packets = self.sdp_response(request, self.service_record())
        packets = self.accept_channel(packets[0][1], 0x41, 0x81, 3)
        frame = rfcomm.RFCOMM_Frame.from_bytes(packets[-1][1])
        self.assertEqual(
            (frame.type, frame.c_r, frame.dlci), (rfcomm.FrameType.SABM, 1, 0)
        )
        self.l2cap(0x41, bytes(rfcomm.RFCOMM_Frame.ua(1, 0)))
        frame = rfcomm.RFCOMM_Frame.from_bytes(self.drain()[0][1])
        self.assertEqual(frame.information[:3], b"\x83\x11\x0e")
        pn = bytes([14, 0xE0, 7, 0, 127, 0, 0, 7])
        mcc = rfcomm.RFCOMM_Frame.make_mcc(rfcomm.MccType.PN, 0, pn)
        self.l2cap(0x41, bytes(rfcomm.RFCOMM_Frame.uih(0, 0, mcc)))
        frame = rfcomm.RFCOMM_Frame.from_bytes(self.drain()[0][1])
        self.assertEqual(
            (frame.type, frame.c_r, frame.dlci), (rfcomm.FrameType.SABM, 1, 14)
        )
        self.l2cap(0x41, bytes(rfcomm.RFCOMM_Frame.ua(1, 14)))
        frames = [rfcomm.RFCOMM_Frame.from_bytes(data) for _, data in self.drain()]
        self.assertEqual(frames[-1].information, b"AT+BRSF=54\r")
        for response, expected in [
            (b"\r\n+BRSF: 512\r\nOK\r\n", b"AT+CIND=?\r"),
            (b'\r\n+CIND: ("call",(0,1)),("callsetup",(0-3))\r\nOK\r\n', b"AT+CIND?\r"),
            (b"\r\n+CIND: 0,0\r\nOK\r\n", b"AT+CMER=3,0,0,1\r"),
            (b"\r\nOK\r\n", b"AT+CLIP=1\r"),
            (b"\r\nOK\r\n", b"AT+VGS=15\r"),
            (b"\r\nOK\r\n", None),
        ]:
            self.l2cap(0x41, bytes(rfcomm.RFCOMM_Frame.uih(0, 14, response)))
            frames = [rfcomm.RFCOMM_Frame.from_bytes(data) for _, data in self.drain()]
            at = [f.information for f in frames if f.dlci == 14 and not f.p_f]
            self.assertEqual(at, [expected] if expected else [])
        self.assertTrue(self.lib.demo_flags() & 4)

    def test_watch_initiates_complete_hfp_session(self):
        self.outgoing_ready()
        self.assertEqual(self.lib.demo_errors(), 0)

    def test_sdp_client_reassembles_continuation(self):
        request = self.discovery()
        record = self.service_record()
        packets = self.sdp_response(request, record[:10], b"\2ab")
        next_request = sdp.SDP_PDU.from_bytes(packets[0][1])
        self.assertEqual(next_request.continuation_state, b"\2ab")
        self.assertNotEqual(next_request.transaction_id, request.transaction_id)
        self.assertEqual(self.sdp_response(request, record[10:]), [])
        packets = self.sdp_response(next_request, record[10:])
        self.assertEqual(packets[0][1][4:], le(3, 0x41))

    def test_invalid_server_channel_disconnects(self):
        request = self.discovery()
        self.sdp_response(request, self.service_record(31))
        self.assertFalse(self.lib.demo_encrypted())
        self.assertFalse(self.lib.demo_flags() & 4)

    def test_stop_cancels_pending_page_before_stopped(self):
        self.lib.demo_shared_bond(1)
        self.assertTrue(self.lib.demo_connect())
        p = self.pop()
        self.event(0x0F, b"\0\1" + p[1:3])
        self.lib.demo_stop()
        commands = []
        while p := self.pop():
            commands.append(int.from_bytes(p[1:3], "little"))
            self.complete(p)
        self.assertIn(0x0408, commands)
        self.assertFalse(self.lib.demo_stopped())
        self.event(3, b"\2" + le(0) + bytes.fromhex("112233445566") + b"\0\0")
        self.assertTrue(self.lib.demo_stopped())

    def test_stop_before_create_was_sent_needs_no_cancel(self):
        self.lib.demo_shared_bond(1)
        self.assertTrue(self.lib.demo_connect())
        self.lib.demo_stop()
        p = self.pop()
        self.assertEqual(p[1:3], le(0x0C1A))
        self.complete(p)
        self.assertEqual(self.pop(), b"")
        self.assertTrue(self.lib.demo_stopped())

    def test_reconnect_waits_and_backs_off_after_failure(self):
        self.lib.demo_shared_bond(1)
        self.lib.demo_reconnect(100, 1)
        self.lib.demo_reconnect(2099, 1)
        self.assertEqual(self.pop(), b"")
        self.lib.demo_reconnect(2100, 1)
        p = self.pop()
        self.assertEqual(p[1:3], le(0x0405))
        self.event(0x0F, b"\0\1" + p[1:3])
        self.event(3, b"\4" + le(0) + bytes.fromhex("112233445566") + b"\0\0")
        self.lib.demo_reconnect(2200, 1)
        self.lib.demo_reconnect(4199, 1)
        self.assertEqual(self.pop(), b"")
        self.lib.demo_reconnect(4200, 1)
        self.assertEqual(self.pop()[1:3], le(0x0405))

    def test_intentional_phone_disconnect_waits_for_new_ble_session(self):
        self.outgoing_ready()
        self.lib.demo_reconnect(self.now, 1)
        self.event(5, b"\0" + le(1) + b"\x13")
        self.drain()
        self.lib.demo_reconnect(self.now + 60001, 1)
        self.assertEqual(self.pop(), b"")
        self.lib.demo_reconnect(self.now + 60002, 0)
        self.lib.demo_reconnect(self.now + 60003, 1)
        self.lib.demo_reconnect(self.now + 62003, 1)
        self.assertEqual(self.pop()[1:3], le(0x0405))

    def test_profile_starts_on_existing_encrypted_acl(self):
        self.lib.demo_shared_bond(1)
        self.connect()
        self.drain()
        self.event(8, b"\0" + le(1) + b"\1")
        self.lib.demo_reconnect(self.now, 1)
        self.assertEqual(self.drain(), [])
        self.lib.demo_reconnect(self.now + 2000, 1)
        self.tick()
        request = self.drain()[0][1]
        self.assertEqual(request[4:], le(1, 0x40))

    def test_discovery_timeout_disconnects(self):
        self.discovery()
        self.now += 30001
        self.tick()
        self.assertEqual(self.pop()[1:3], le(0x0406))
        self.assertFalse(self.lib.demo_encrypted())

    def test_sdp_continuations_are_bounded(self):
        request = self.discovery()
        for _ in range(7):
            packets = self.sdp_response(request, b"x", b"\1a")
            request = sdp.SDP_PDU.from_bytes(packets[0][1])
        self.sdp_response(request, b"x", b"\1a")
        self.assertFalse(self.lib.demo_encrypted())

    def test_bond_replaced_during_page_is_not_used_on_same_attempt(self):
        self.peer = bytes.fromhex("112233445566")
        self.lib.demo_shared_bond(1)
        self.assertTrue(self.lib.demo_connect())
        p = self.pop()
        self.event(0x0F, b"\0\1" + p[1:3])
        self.lib.demo_replace_bond()
        self.event(0x17, self.peer)
        p = self.pop()
        self.assertEqual(p[1:3], le(0x040C))
        self.complete(p)
        self.event(3, b"\0" + le(1) + self.peer + b"\1\0")
        self.complete(self.pop())
        self.assertEqual(self.pop()[1:3], le(0x0406))

    def test_ble_loss_cancels_page(self):
        self.lib.demo_shared_bond(1)
        self.lib.demo_reconnect(100, 1)
        self.lib.demo_reconnect(2100, 1)
        p = self.pop()
        self.event(0x0F, b"\0\1" + p[1:3])
        self.lib.demo_reconnect(2200, 0)
        self.assertEqual(self.pop()[1:3], le(0x0408))

# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import queue
import socket
import sys
import tempfile
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hci_bridge import bridge, monitor_audio
from pebble.pulse2.exceptions import SocketClosed


class Channel:
    mtu = 128

    def __init__(self):
        self.rx = queue.Queue()
        self.tx = queue.Queue()

    def receive(self):
        item = self.rx.get(timeout=2)
        if isinstance(item, Exception):
            raise item
        return item

    def send(self, data):
        self.tx.put(data)

    def close(self):
        self.rx.put(SocketClosed())


class TestHciBridge(unittest.TestCase):
    def setUp(self):
        self.host, self.bridge_socket = socket.socketpair()
        self.host.settimeout(2)
        self.channel = Channel()
        self.errors = []

        def run():
            try:
                bridge(self.bridge_socket, self.channel)
            except (OSError, SocketClosed, queue.Empty) as error:
                self.errors.append(error)

        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.host.close()
        self.channel.close()
        self.thread.join(timeout=3)
        self.bridge_socket.close()
        self.assertFalse(self.thread.is_alive())

    def test_h4_packets_survive_fragmentation_in_both_directions(self):
        # Command, ACL and SCO packets span several PULSE/TCP chunks.
        host_data = (
            bytes.fromhex("01030c00")
            + bytes.fromhex("0201200004")
            + bytes(range(256)) * 4
            + bytes.fromhex("0301003c")
            + bytes(60)
        )
        for start in range(0, len(host_data), 37):
            self.host.sendall(host_data[start : start + 37])
        received = bytearray()
        while len(received) < len(host_data):
            chunk = self.channel.tx.get(timeout=2)
            self.assertLessEqual(len(chunk), self.channel.mtu)
            received.extend(chunk)
        self.assertEqual(bytes(received), host_data)

        controller_data = (
            bytes.fromhex("040e0401030c00") + bytes.fromhex("0301003c") + bytes(60)
        )
        for start in range(0, len(controller_data), 3):
            self.channel.rx.put(controller_data[start : start + 3])
        received.clear()
        while len(received) < len(controller_data):
            received.extend(self.host.recv(11))
        self.assertEqual(bytes(received), controller_data)
        self.assertFalse(self.errors)

    def test_lost_pulse_link_closes_host_stream(self):
        self.channel.rx.put(SocketClosed())
        self.assertEqual(self.host.recv(1), b"")
        self.thread.join(timeout=3)
        self.assertFalse(self.thread.is_alive())
        self.assertFalse(self.errors)


class TestAudioMonitor(unittest.TestCase):
    def test_capture_export_waits_until_call_ends(self):
        class Stop:
            stopped = False

            def is_set(self):
                return self.stopped

            def wait(self, timeout):
                pass

        stopped = Stop()
        commands = []

        class Prompt:
            polls = 0

            def command_and_response(self, command, timeout):
                commands.append(command)
                if command == "bt audio capture":
                    return []
                if command == "bt audio probe":
                    self.polls += 1
                    return [
                        f"local audio active={int(self.polls == 1)} rx=100",
                        "local sample bytes=6 armed=0",
                    ]
                if command == "bt audio dump":
                    stopped.stopped = True
                    return ["pcm: 038001020000"]
                raise AssertionError(command)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "audio.h4"
            monitor_audio(Prompt(), stopped, path)
            self.assertEqual(path.read_bytes(), bytes.fromhex("038001020000"))
        self.assertEqual(
            commands,
            ["bt audio capture", "bt audio probe", "bt audio probe", "bt audio dump"],
        )


if __name__ == "__main__":
    unittest.main()

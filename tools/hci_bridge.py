# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Expose a development watch's PULSE HCI byte stream to one local TCP host."""

import argparse
import contextlib
import socket
import threading
from pathlib import Path

from pebble import commander, pulse2

HCI_PROTOCOL = 0x3E23


def monitor_audio(prompt, stopped, capture_pcm=None):
    try:
        if capture_pcm:
            prompt.command_and_response("bt audio capture", timeout=5)
            print("Armed one short downlink PCM capture", flush=True)
        while not stopped.is_set():
            lines = prompt.command_and_response("bt audio probe", timeout=5)
            idle = any(line.startswith("local audio active=0 ") for line in lines)
            for line in lines:
                print(line, flush=True)
                if (
                    capture_pcm
                    and idle
                    and line.startswith("local sample bytes=")
                    and "bytes=0 " not in line
                    and line.endswith("armed=0")
                ):
                    dump = prompt.command_and_response("bt audio dump", timeout=10)
                    data = b"".join(
                        bytes.fromhex(part[5:]) for part in dump if part.startswith("pcm: ")
                    )
                    Path(capture_pcm).write_bytes(data)
                    print(f"Saved {len(data)} H4 bytes to {capture_pcm}", flush=True)
                    capture_pcm = None
            stopped.wait(5)
    except (
        commander.exceptions.CommandTimedOut,
        pulse2.exceptions.SocketClosed,
        OSError,
    ) as error:
        if not stopped.is_set():
            print(f"Audio diagnostic stopped: {error}", flush=True)


@contextlib.contextmanager
def audio_monitor(link, enabled, capture_pcm=None):
    if not enabled:
        yield
        return
    prompt = commander.apps.Prompt(link)
    stopped = threading.Event()
    thread = threading.Thread(
        target=monitor_audio, args=(prompt, stopped, capture_pcm), daemon=True
    )
    thread.start()
    try:
        yield
    finally:
        stopped.set()
        prompt.close()
        thread.join(timeout=2)


def bridge(connection, channel):
    """Forward unchanged H4 bytes; PULSE and TCP boundaries are not HCI frames."""
    errors = []

    def receive():
        try:
            while True:
                connection.sendall(channel.receive())
        except (OSError, pulse2.exceptions.SocketClosed) as error:
            errors.append(error)
        finally:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    receiver = threading.Thread(target=receive, daemon=True)
    receiver.start()
    try:
        while data := connection.recv(min(channel.mtu, 256)):
            channel.send(data)
    finally:
        channel.close()
        receiver.join(timeout=2)
    if errors and not isinstance(errors[0], pulse2.exceptions.SocketClosed):
        raise errors[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tty", required=True)
    parser.add_argument("--port", type=int, default=12346)
    parser.add_argument(
        "--audio-probe",
        action="store_true",
        help="Poll shared audio metadata; requires CONFIG_BT_HCI_AUDIO_PROBE",
    )
    parser.add_argument(
        "--capture-pcm",
        help="Opt-in short downlink H4 capture to a local file; requires local-audio firmware",
    )
    args = parser.parse_args()

    interface = pulse2.Interface.open_dbgserial(url=args.tty)
    try:
        link = interface.get_link(timeout=10)
        if link is None:
            raise RuntimeError("Watch did not establish a PULSE connection")
        channel = link.open_socket("reliable", HCI_PROTOCOL)
        try:
            with socket.socket() as server:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind(("127.0.0.1", args.port))
                server.listen(1)
                print(f"HCI ready at tcp-client:127.0.0.1:{args.port}", flush=True)
                print("Requires CONFIG_BT_FW_HCI_BRIDGE; accepts one host", flush=True)
                connection, _ = server.accept()
                with connection, audio_monitor(
                    link, args.audio_probe or bool(args.capture_pcm), args.capture_pcm
                ):
                    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    bridge(connection, channel)
        finally:
            channel.close()
    finally:
        interface.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

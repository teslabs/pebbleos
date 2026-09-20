# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Exercise a controller with Bumble's Apache-2.0 HFP host over standard HCI.

Pair from the phone and initiate a call there. The probe accepts CVSD audio,
counts received SCO payloads and sends silence back, without recording voice.
Use --tone for an audible uplink check, or --watch-audio with local-audio
firmware to use the watch microphone and speaker. It does not dial or answer.
"""

import argparse
import asyncio
import contextlib
import logging
import math
import struct
import time

from bumble import hci, hfp, rfcomm
from bumble.device import Device, DeviceConfiguration
from bumble.snoop import BtSnooper, Snooper
from bumble.transport import open_transport

logger = logging.getLogger(__name__)


def tone_pcm(first_sample, sample_count):
    """Quiet 440 Hz, one second on/off; 8 kHz signed little-endian PCM."""
    samples = []
    for sample in range(first_sample, first_sample + sample_count):
        position = sample % 16000
        envelope = max(0, min(1, position / 40, (7999 - position) / 40))
        samples.append(
            round(2048 * envelope * math.sin(2 * math.pi * 440 * sample / 8000))
        )
    return struct.pack(f"<{len(samples)}h", *samples)


class ScoCredits(Snooper):
    """Count synchronous completions without recording packet contents."""

    def __init__(self, trace=None):
        self.trace = trace
        self.handle = None
        self.limit = self.available = self.completed = 0

    def snoop(self, packet, direction):
        if self.trace:
            self.trace.snoop(packet, direction)
        if (
            direction != self.Direction.CONTROLLER_TO_HOST
            or len(packet) < 4
            or packet[:2] != b"\x04\x13"
            or len(packet) != 4 + 4 * packet[3]
        ):
            return
        for offset in range(4, len(packet), 4):
            handle = int.from_bytes(packet[offset : offset + 2], "little")
            count = int.from_bytes(packet[offset + 2 : offset + 4], "little")
            if handle == self.handle:
                self.completed += count
                self.available = min(self.limit, self.available + count)


class Probe:
    def __init__(
        self,
        device,
        legacy_sco=False,
        sco_flow_control=False,
        tone=False,
        watch_audio=False,
    ):
        self.device = device
        self.legacy_sco = legacy_sco
        self.sco_flow_control = sco_flow_control
        self.tone = tone
        self.watch_audio = watch_audio
        self.tx_sample_offset = 0
        self.tx_mtu = 255
        self.credits = ScoCredits(device.host.snooper)
        if sco_flow_control:
            device.host.snooper = self.credits
        self.tx_skipped = 0
        self.tasks = set()
        self.rx_packets = self.rx_bytes = self.bad_packets = self.tx_packets = 0
        self.configuration = hfp.HfConfiguration(
            supported_hf_features=[],
            supported_hf_indicators=[],
            supported_audio_codecs=[hfp.AudioCodec.CVSD],
        )

    def spawn(self, coroutine):
        task = asyncio.create_task(coroutine)
        self.tasks.add(task)

        def done(task):
            self.tasks.discard(task)
            if not task.cancelled() and task.exception():
                logger.error("Probe operation failed: %s", task.exception())

        task.add_done_callback(done)
        return task

    def on_dlc(self, dlc):
        print("RFCOMM connected; negotiating HFP", flush=True)
        protocol = hfp.HfProtocol(dlc, self.configuration)
        protocol.on(
            "ag_indicator", lambda value: print(f"Call indicator: {value}", flush=True)
        )
        protocol.on("ring", lambda: print("Incoming call", flush=True))
        task = self.spawn(self.run_hfp(protocol))
        dlc.on("close", task.cancel)

    async def run_hfp(self, protocol):
        await protocol.initiate_slc()
        print("HFP service-level connection established (CVSD)", flush=True)
        await protocol.run()

    async def accept_audio(self, connection, link_type):
        preset = (
            hfp.DefaultCodecParameters.SCO_CVSD_D1
            if link_type == hci.HCI_Connection_Complete_Event.LinkType.SCO
            else hfp.DefaultCodecParameters.ESCO_CVSD_S1
        )
        parameters = hfp.ESCO_PARAMETERS[preset]
        enhanced = (
            self.device.host.supports_command(
                hci.HCI_ENHANCED_ACCEPT_SYNCHRONOUS_CONNECTION_REQUEST_COMMAND
            )
            and not self.legacy_sco
        )
        print(
            f"Audio requested; accepting {preset.name}, enhanced={enhanced}", flush=True
        )
        if enhanced:
            command = hci.HCI_Enhanced_Accept_Synchronous_Connection_Request_Command(
                bd_addr=connection.peer_address, **parameters.asdict()
            )
        else:
            command = hci.HCI_Accept_Synchronous_Connection_Request_Command(
                bd_addr=connection.peer_address,
                transmit_bandwidth=parameters.transmit_bandwidth,
                receive_bandwidth=parameters.receive_bandwidth,
                max_latency=parameters.max_latency,
                voice_setting=0x0060,
                retransmission_effort=parameters.retransmission_effort,
                packet_type=parameters.packet_type,
            )
        await self.device.send_async_command(command)

    def on_connection(self, connection):
        print("Classic ACL connected", flush=True)
        connection.on(
            "sco_request", lambda kind: self.spawn(self.accept_audio(connection, kind))
        )
        connection.on(
            "disconnection",
            lambda reason: print(f"ACL disconnected: {reason}", flush=True),
        )

    def on_audio(self, link):
        print(
            f"SCO connected: handle={link.handle}, air_mode={link.air_mode.name}",
            flush=True,
        )
        link.on(
            "disconnection",
            lambda reason: print(f"SCO disconnected: {reason}", flush=True),
        )
        if link.air_mode != hci.CodecID.CVSD:
            print("Unexpected codec; skipping PCM transmission", flush=True)
            return
        self.credits.handle = link.handle
        self.credits.available = self.credits.limit
        self.tx_sample_offset = 0
        link.on("disconnection", lambda reason: self.clear_audio())
        link.sink = self.on_packet

    def clear_audio(self):
        self.credits.handle = None
        self.credits.available = 0

    def on_packet(self, packet):
        self.rx_packets += 1
        self.rx_bytes += len(packet.data)
        if (
            packet.packet_status
            != hci.HCI_SynchronousDataPacket.Status.CORRECTLY_RECEIVED_DATA
        ):
            self.bad_packets += 1
        # Pace uplink with the controller's downlink; no voice is saved.
        if self.watch_audio:
            return
        payload = (
            tone_pcm(self.tx_sample_offset, len(packet.data) // 2)
            if self.tone
            else bytes(len(packet.data))
        )
        self.tx_sample_offset += len(packet.data) // 2
        for offset in range(0, len(payload), self.tx_mtu):
            if self.sco_flow_control:
                if not self.credits.available:
                    self.tx_skipped += 1
                    continue
                self.credits.available -= 1
            self.device.host.send_sco_sdu(
                packet.connection_handle, payload[offset : offset + self.tx_mtu]
            )
            self.tx_packets += 1

    async def report(self):
        start = time.monotonic()
        while True:
            await asyncio.sleep(5)
            print(
                f"SCO totals t={time.monotonic() - start:.0f}s rx={self.rx_packets} "
                f"bytes={self.rx_bytes} bad={self.bad_packets} tx={self.tx_packets}",
                flush=True,
            )
            if self.sco_flow_control:
                print(
                    f"SCO credits available={self.credits.available} "
                    f"completed={self.credits.completed} skipped={self.tx_skipped}",
                    flush=True,
                )

    async def run(self, duration):
        server = rfcomm.Server(self.device)
        channel = server.listen(self.on_dlc)
        self.device.sdp_service_records = {
            0x10001: hfp.make_hf_sdp_records(0x10001, channel, self.configuration)
        }
        self.device.on("connection", self.on_connection)
        self.device.on("sco_connection", self.on_audio)
        self.device.on(
            "sco_connection_failure",
            lambda status: print(f"SCO failed: {status}", flush=True),
        )
        await asyncio.wait_for(self.device.power_on(), timeout=30)
        if self.sco_flow_control:
            buffers = await self.device.send_sync_command(
                hci.HCI_Read_Buffer_Size_Command()
            )
            self.credits.limit = buffers.hc_total_num_synchronous_data_packets
            self.tx_mtu = buffers.hc_synchronous_data_packet_length
            if not self.credits.limit or self.tx_mtu < 2 or self.tx_mtu % 2:
                raise RuntimeError("Controller reports no SCO buffers")
            await self.device.send_sync_command(
                hci.HCI_Write_Synchronous_Flow_Control_Enable_Command(
                    synchronous_flow_control_enable=1
                )
            )
            print(
                f"SCO flow control enabled: buffers={self.credits.limit}, "
                f"MTU={buffers.hc_synchronous_data_packet_length}",
                flush=True,
            )
        print("Host ready: pair the phone with 'Obelix HFP test'", flush=True)
        if self.tone:
            print(
                "Uplink tone enabled: quiet 440 Hz, one second on / one second off",
                flush=True,
            )
        if self.watch_audio:
            print(
                "Watch microphone/speaker own SCO audio; desktop uplink disabled",
                flush=True,
            )
        if not self.watch_audio:
            self.spawn(self.report())
        try:
            await asyncio.sleep(duration)
        finally:
            tasks = list(self.tasks)
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            await self.device.power_off()


async def main(args, trace=None):
    async with await open_transport(args.transport) as transport:
        config = DeviceConfiguration(
            name="Obelix HFP test",
            class_of_device=0x200408,
            le_enabled=False,
            classic_enabled=True,
            classic_smp_enabled=False,
            keystore=None,
        )
        device = Device.from_config_with_hci(config, transport.source, transport.sink)
        if trace:
            device.host.snooper = BtSnooper(trace)
        await Probe(
            device,
            args.legacy_sco,
            args.sco_flow_control or args.watch_audio,
            args.tone,
            args.watch_audio,
        ).run(args.duration)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport", default="tcp-client:127.0.0.1:12346")
    parser.add_argument("--duration", type=float, default=300)
    parser.add_argument(
        "--trace", help="Optional private btsnoop capture (includes keys/audio)"
    )
    parser.add_argument(
        "--legacy-sco", action="store_true", help="Use legacy SCO accept command"
    )
    parser.add_argument("--debug", action="store_true")
    audio_mode = parser.add_mutually_exclusive_group()
    audio_mode.add_argument(
        "--tone", action="store_true", help="Send a quiet repeating uplink test tone"
    )
    audio_mode.add_argument(
        "--watch-audio",
        action="store_true",
        help="Watch mic/speaker own audio; requires local audio firmware",
    )
    parser.add_argument(
        "--sco-flow-control",
        action="store_true",
        help="Respect controller SCO packet credits",
    )
    arguments = parser.parse_args()
    logging.basicConfig(level=logging.DEBUG if arguments.debug else logging.WARNING)
    try:
        # Tracing is opt-in because HCI captures include keys and audio payloads.
        with (
            open(arguments.trace, "wb")
            if arguments.trace
            else contextlib.nullcontext() as trace
        ):
            asyncio.run(main(arguments, trace))
    except KeyboardInterrupt:
        pass

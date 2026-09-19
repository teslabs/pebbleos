# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
"""Exercise local Android Telecom calls while checking encrypted BLE/HFP coexistence.

Requires tools/android_hfp_test installed and both watch links connected.
All calls originate in that local test app; this script never dials a number.
"""

import argparse
import re
import subprocess
import time

from pebble import commander, pulse2


def fields(line):
    return {name: int(value) for name, value in re.findall(r"(\w+)=(\d+)", line)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tty", required=True)
    parser.add_argument("--android-serial", required=True)
    parser.add_argument(
        "--device", required=True, help="Watch Bluetooth identity address"
    )
    parser.add_argument(
        "--duration", type=int, default=60, help="Seconds per active call (1..240)"
    )
    parser.add_argument("--repeat", type=int, default=1, help="Test cycles (1..10)")
    parser.add_argument(
        "--scenario", choices=("all", "incoming", "reject", "outgoing"), default="all"
    )
    parser.add_argument(
        "--transfer",
        action="store_true",
        help="Transfer active audio to the phone and back",
    )
    parser.add_argument(
        "--controls",
        action="store_true",
        help="Exercise call volume and microphone mute",
    )
    parser.add_argument(
        "--companion-ping",
        action="store_true",
        help="Send Pebble protocol pings during calls; verify reception in companion logs",
    )
    parser.add_argument(
        "--audio-interval",
        type=int,
        default=0,
        help="Print audio counter samples every N seconds (0 disables, 1..240)",
    )
    parser.add_argument(
        "--max-underrun-bytes",
        type=int,
        help="Fail if speaker DMA underruns grow by more than this during a call",
    )
    args = parser.parse_args()
    if not 1 <= args.duration <= 240 or not 1 <= args.repeat <= 10:
        parser.error("Duration must be 1..240 seconds and repeat must be 1..10")
    if not 0 <= args.audio_interval <= 240:
        parser.error("Audio interval must be 0..240 seconds")
    if args.max_underrun_bytes is not None and args.max_underrun_bytes < 0:
        parser.error("Maximum underrun bytes must be nonnegative")
    if not re.fullmatch(r"(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}", args.device):
        parser.error("Device must be a Bluetooth address")

    def android(command):
        subprocess.run(
            [
                "adb",
                "-s",
                args.android_serial,
                "shell",
                "am",
                "start",
                "-n",
                "com.teslabs.hfptest/.MainActivity",
                "--es",
                "device",
                args.device,
                "--es",
                "command",
                command,
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            timeout=20,
        )

    interface = pulse2.Interface.open_dbgserial(url=args.tty)
    prompt = None
    own_call = False
    initial_errors = None
    try:
        prompt = commander.apps.Prompt(interface.get_link(timeout=10))

        def command(text):
            return prompt.command_and_response(text, timeout=10)

        def status():
            dual = command("bt dual status")
            radio = fields(dual[0])
            if not (radio.get("synced") and radio.get("LE") and radio.get("HFP")):
                raise RuntimeError(f"Lost radio connection: {dual}")
            if not any(
                line.startswith("LE handle=") and "encrypted=1 bonded=1" in line
                for line in dual
            ):
                raise RuntimeError("BLE is not encrypted and bonded")
            if not any(
                line.startswith("Classic scan=") and "encrypted=1" in line
                for line in dual
            ):
                raise RuntimeError("Classic is not encrypted")
            state = fields(command("bt hfp status")[0])
            if not state.get("ready"):
                raise RuntimeError(f"HFP is not ready: {state}")
            if initial_errors is not None and state.get("errors") != initial_errors:
                raise RuntimeError(f"Profile error during test: {state}")
            return state

        def wait_for(predicate, description):
            deadline = time.monotonic() + 20
            while True:
                state = status()
                if predicate(state):
                    return state
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"Timed out waiting for {description}: {state}")
                time.sleep(0.25)

        def idle(state):
            return not (state.get("call") or state.get("setup") or state.get("audio"))

        def audio_counters(lines):
            for line in lines:
                if line.startswith("adapter rx="):
                    return fields(line)
            raise RuntimeError("Missing SCO audio counters")

        def underrun_bytes(lines):
            for line in lines:
                if line.startswith("speaker DMA refills="):
                    return fields(line)["underrun_bytes"]
            raise RuntimeError("Missing speaker DMA counters")

        initial = status()
        initial_errors = initial.get("errors")
        if not idle(initial):
            raise RuntimeError(
                "Watch already has a call; finish it before running this test"
            )
        scenarios = (
            ("incoming", "reject", "outgoing")
            if args.scenario == "all"
            else (args.scenario,)
        )
        for cycle in range(args.repeat):
            for scenario in scenarios:
                print(f"Cycle {cycle + 1}: {scenario}", flush=True)
                own_call = True
                android("outgoing" if scenario == "outgoing" else "incoming")
                wait_for(lambda state: bool(state.get("setup")), "ringing/dialing")
                if scenario != "reject":
                    if scenario == "outgoing":
                        android("active")
                    else:
                        command("bt hfp answer")
                    wait_for(
                        lambda state: state.get("call") and state.get("audio"),
                        "active audio",
                    )
                    if args.transfer:
                        gain = status()["gain"]
                        command("bt hfp mute 1")
                        wait_for(
                            lambda state: state.get("mic_muted"), "microphone mute"
                        )
                        command("bt hfp audio phone")
                        wait_for(
                            lambda state: (
                                state.get("call")
                                and not state.get("audio")
                                and not state.get("audio_pending")
                            ),
                            "audio on phone",
                        )
                        time.sleep(1)
                        if not status().get("call"):
                            raise RuntimeError("Audio transfer ended the call")
                        probe = command("bt audio probe")
                        if not any(
                            line.startswith("local audio active=0 ") for line in probe
                        ):
                            raise RuntimeError(
                                "Watch capture did not stop after transfer"
                            )
                        command("bt hfp audio watch")
                        wait_for(
                            lambda state: (
                                state.get("call")
                                and state.get("audio")
                                and not state.get("audio_pending")
                            ),
                            "audio back on watch",
                        )
                        state = status()
                        if state.get("gain") != gain or not state.get("mic_muted"):
                            raise RuntimeError(
                                "Audio transfer lost volume or local mute state"
                            )
                        command("bt hfp mute 0")
                        wait_for(
                            lambda state: not state.get("mic_muted"),
                            "microphone unmute",
                        )
                        print("Audio transfer in both directions passed", flush=True)
                    initial_audio = command("bt audio probe")
                    before_audio = audio_counters(initial_audio)
                    if args.controls:
                        original_gain = status()["gain"]
                        try:
                            for gain in (7, 3, original_gain):
                                command(f"bt hfp volume {gain}")
                                wait_for(
                                    lambda state, gain=gain: (
                                        state.get("gain") == gain
                                        and not state.get("busy")
                                    ),
                                    f"speaker gain {gain}",
                                )
                            command("bt hfp mute 1")
                            wait_for(
                                lambda state: state.get("mic_muted"), "microphone mute"
                            )
                            time.sleep(2)
                            command("bt hfp mute 0")
                            wait_for(
                                lambda state: not state.get("mic_muted"),
                                "microphone unmute",
                            )
                            print("Volume and local mute controls passed", flush=True)
                        finally:
                            command(f"bt hfp volume {original_gain}")
                            command("bt hfp mute 0")
                    started = time.monotonic()
                    deadline = started + args.duration
                    next_ping = 0
                    next_audio = started + args.audio_interval

                    def sample_audio(lines, started=started):
                        print(f"Audio at {time.monotonic() - started:.1f}s", flush=True)
                        for line in lines:
                            if line.startswith(
                                (
                                    "adapter rx=",
                                    "speaker DMA refills=",
                                    "local capture ",
                                    "local audio active=",
                                    "local echo ",
                                    "local processing ",
                                )
                            ):
                                print(line, flush=True)

                    if args.audio_interval:
                        sample_audio(initial_audio)
                    while time.monotonic() < deadline:
                        if args.companion_ping and time.monotonic() >= next_ping:
                            command("ping")
                            next_ping = time.monotonic() + 5
                        if args.audio_interval and time.monotonic() >= next_audio:
                            sample_audio(command("bt audio probe"))
                            next_audio = time.monotonic() + args.audio_interval
                        state = status()
                        if not state.get("call") or not state.get("audio"):
                            raise RuntimeError(f"Call/audio stopped: {state}")
                        time.sleep(min(1, max(0, deadline - time.monotonic())))
                    audio = command("bt audio probe")
                    for line in audio:
                        print(line, flush=True)
                    print("Microphone:", command("mic read"), flush=True)
                    if args.max_underrun_bytes is not None:
                        delta = underrun_bytes(audio) - underrun_bytes(initial_audio)
                        if delta < 0 or delta > args.max_underrun_bytes:
                            raise RuntimeError(
                                f"Speaker DMA underrun delta: {delta} bytes"
                            )
                    after_audio = audio_counters(audio)
                    received = after_audio["rx"] - before_audio["rx"]
                    bad = after_audio["bad"] - before_audio["bad"]
                    consumed = after_audio["consumed"] - before_audio["consumed"]
                    if received <= 0 or bad * 2 >= received or consumed <= 0:
                        raise RuntimeError(
                            f"SCO audio failed: received={received} bad={bad} sent={consumed}"
                        )
                command("bt hfp hangup")
                wait_for(idle, "call and audio teardown")
                android("hangup")
                own_call = False
                print("PASS", flush=True)
    finally:
        try:
            if own_call:
                android("hangup")
        finally:
            if prompt is not None:
                prompt.close()
            interface.close()


if __name__ == "__main__":
    main()

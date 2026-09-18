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
        "--companion-ping",
        action="store_true",
        help="Send Pebble protocol pings during calls; verify reception in companion logs",
    )
    args = parser.parse_args()
    if not 1 <= args.duration <= 240 or not 1 <= args.repeat <= 10:
        parser.error("Duration must be 1..240 seconds and repeat must be 1..10")
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
                    before_audio = audio_counters(command("bt audio probe"))
                    deadline = time.monotonic() + args.duration
                    next_ping = 0
                    while time.monotonic() < deadline:
                        if args.companion_ping and time.monotonic() >= next_ping:
                            command("ping")
                            next_ping = time.monotonic() + 5
                        state = status()
                        if not state.get("call") or not state.get("audio"):
                            raise RuntimeError(f"Call/audio stopped: {state}")
                        time.sleep(min(1, max(0, deadline - time.monotonic())))
                    audio = command("bt audio probe")
                    for line in audio:
                        print(line, flush=True)
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

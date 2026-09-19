# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
"""Check NimBLE host resynchronization, shared bonds, and local call teardown.

Requires CoreApp connected and tools/android_hfp_test installed. Calls stay
inside the local Android test app and are never answered or dialed externally.
"""

import argparse
import re
import subprocess
import time

from hfp_smoke import fields
from pebble import commander, pulse2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tty", required=True)
    parser.add_argument("--android-serial", required=True)
    parser.add_argument("--device", required=True)
    parser.add_argument("--repeat", type=int, default=1)
    args = parser.parse_args()
    if not 1 <= args.repeat <= 10:
        parser.error("Repeat must be 1..10")
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
    try:
        prompt = commander.apps.Prompt(interface.get_link(timeout=10))

        def command(text):
            return prompt.command_and_response(text, timeout=10)

        def wait_for(probe, predicate, description):
            deadline = time.monotonic() + 45
            while True:
                value = probe()
                if predicate(value):
                    return value
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"Timed out waiting for {description}: {value}")
                time.sleep(1)

        def ready(lines):
            radio = fields(lines[0])
            return (
                radio.get("synced")
                and radio.get("HFP")
                and not radio.get("errors")
                and any("encrypted=1 bonded=1" in line for line in lines)
                and any(
                    line.startswith("Classic scan=") and "encrypted=1" in line
                    for line in lines
                )
            )

        def call_state():
            return fields(command("bt hfp status")[0])

        for cycle in range(args.repeat):
            for ringing in (False, True):
                label = "ringing" if ringing else "idle"
                print(f"Cycle {cycle + 1}: reset while {label}", flush=True)
                before = wait_for(
                    lambda: command("bt dual status"), ready, "encrypted HFP"
                )
                resets = fields(before[0]).get("resets")
                if resets is None:
                    raise RuntimeError(
                        "Firmware needs the host-reset counter diagnostic"
                    )
                state = call_state()
                if state.get("call") or state.get("setup") or state.get("audio"):
                    raise RuntimeError("Watch already has a call; finish it first")
                if ringing:
                    own_call = True
                    android("incoming")
                    wait_for(call_state, lambda s: s.get("setup") == 1, "incoming call")
                command("ble host reset")
                after = wait_for(
                    lambda: command("bt dual status"),
                    lambda lines, resets=resets: (
                        ready(lines) and fields(lines[0]).get("resets") == resets + 1
                    ),
                    "host resync without watch reboot or another pairing",
                )
                print(after[0], flush=True)
                if ringing:
                    wait_for(
                        call_state,
                        lambda s: s.get("setup") == 1,
                        "restored incoming call",
                    )
                    command("bt hfp hangup")
                    wait_for(
                        call_state,
                        lambda s: (
                            not (s.get("call") or s.get("setup") or s.get("audio"))
                        ),
                        "call teardown",
                    )
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

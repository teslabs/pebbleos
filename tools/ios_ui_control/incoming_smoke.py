# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
"""Exercise local CallKit incoming, queued volume/answer, audio and hangup."""

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

from pebble import commander, pulse2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True, help="iPhone UDID or CoreDevice ID")
    parser.add_argument("--tty", required=True, help="Watch debug serial port")
    parser.add_argument("--duration", type=int, default=5, choices=range(1, 61))
    parser.add_argument("--gain", type=int, default=7, choices=range(16))
    parser.add_argument(
        "--capture-pcm",
        type=Path,
        help="Save a bounded H4 capture of the helper's received tone",
    )
    parser.add_argument(
        "--output", type=Path, default=Path("build-ios-ui-control/smoke")
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    control = [
        sys.executable,
        str(Path(__file__).with_name("control.py")),
        "--device",
        args.device,
        "--output",
        str(args.output),
    ]

    def ui(action, **kwargs):
        return control + [
            json.dumps({"action": action, "bundle": "com.teslabs.hfpdriver", **kwargs})
        ]

    def check_idle_app():
        result = subprocess.run(
            ui("snapshot"), capture_output=True, text=True, timeout=55, check=False
        )
        if result.returncode or not re.search(
            r"identifier: 'call-state', label: 'Idle'", result.stdout
        ):
            raise RuntimeError("The UI runner and idle local call app must be ready")

    check_idle_app()
    interface = pulse2.Interface.open_dbgserial(url=args.tty)
    prompt = None
    trigger = None
    requested = False
    try:
        prompt = commander.apps.Prompt(interface.get_link(timeout=10))

        def command(text):
            return prompt.command_and_response(text, timeout=5)

        def state():
            lines = command("bt hfp status")
            if not lines or not lines[0].startswith("HFP available="):
                raise RuntimeError("Watch HFP diagnostics unavailable")
            return {k: int(v) for k, v in re.findall(r"(\w+)=(\d+)", lines[0])}

        def wait(predicate, timeout=20):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                current = state()
                if predicate(current):
                    return current
                time.sleep(0.2)
            raise RuntimeError(f"Watch state timeout: {current}")

        before = state()
        if not before["ready"] or before["call"] or before["setup"] or before["audio"]:
            raise RuntimeError(f"An idle, connected watch is required: {before}")
        initial_gain = max(0, args.gain - 1)
        command(f"bt hfp volume {initial_gain}")
        wait(lambda s: not s["busy"] and s["gain"] == initial_gain)
        with (args.output / "incoming.txt").open("w") as output:
            requested = True
            trigger = subprocess.Popen(
                ui("tap", label="Incoming"), stdout=output, stderr=subprocess.STDOUT
            )
            wait(lambda s: s["setup"] == 1, timeout=45)
            # Exercise back-to-back requests before the volume AT reply arrives.
            command(f"bt hfp volume {args.gain}")
            command("bt hfp answer")
            active = wait(lambda s: s["call"] and s["audio"] and not s["busy"])
            if active["errors"] != before["errors"]:
                raise RuntimeError(f"Answer added a profile error: {active}")
            print("Answered:", active, flush=True)
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                current = state()
                if not current["call"] or not current["audio"]:
                    raise RuntimeError(f"Call audio ended unexpectedly: {current}")
                time.sleep(0.2)
            if args.capture_pcm:
                command("bt audio capture")
                time.sleep(1)
            print("Audio:", command("bt audio probe"), flush=True)
            command("bt hfp hangup")
            idle = wait(
                lambda s: not (s["call"] or s["setup"] or s["audio"] or s["busy"])
            )
            if not idle["ready"] or idle["errors"] != before["errors"]:
                raise RuntimeError(f"Hangup failed: {idle}")
            if args.capture_pcm:
                lines = command("bt audio dump")
                data = bytes.fromhex(
                    "".join(line[5:] for line in lines if line.startswith("pcm: "))
                )
                if not data:
                    raise RuntimeError("No received tone was captured")
                args.capture_pcm.parent.mkdir(parents=True, exist_ok=True)
                args.capture_pcm.write_bytes(data)
            trigger.wait(timeout=55)
            if trigger.returncode:
                raise RuntimeError("The iPhone Incoming UI command failed")
            check_idle_app()
            requested = False
            command(f"bt hfp volume {before['gain']}")
            restored = wait(lambda s: s["gain"] == before["gain"] and not s["busy"])
            if not restored["ready"] or restored["errors"] != before["errors"]:
                raise RuntimeError(f"Volume restoration failed: {restored}")
            print(f"Restored call gain: {restored['gain']}/15", flush=True)
            print("PASS: queued answer, active audio state and hangup", flush=True)
    finally:
        if trigger is not None and trigger.poll() is None:
            trigger.terminate()
            trigger.wait(timeout=5)
        if prompt is not None:
            prompt.close()
        interface.close()
        if requested:
            # End only the helper's local call, even if the watch has rebooted.
            try:
                result = subprocess.run(
                    ui("tap", label="End call"),
                    capture_output=True,
                    timeout=55,
                    check=False,
                )
                if result.returncode:
                    print(
                        "Cleanup not confirmed; local calls expire after 3 minutes",
                        file=sys.stderr,
                    )
            except subprocess.TimeoutExpired:
                print(
                    "Cleanup timed out; local calls expire after 3 minutes",
                    file=sys.stderr,
                )


if __name__ == "__main__":
    main()

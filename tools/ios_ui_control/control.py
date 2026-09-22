# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
import argparse
import json
import pathlib
import subprocess
import sys
import time
import uuid

parser = argparse.ArgumentParser(
    description="Send one command to the running iPhone UI test"
)
parser.add_argument(
    "--device", required=True, help="CoreDevice identifier or iPhone UDID"
)
parser.add_argument(
    "--output", type=pathlib.Path, default=pathlib.Path("build-ios-ui-control/session")
)
parser.add_argument(
    "command", nargs="?", default='{"action":"snapshot"}', help="JSON command object"
)
args = parser.parse_args()
root = args.output.resolve()
root.mkdir(parents=True, exist_ok=True)
base = ["xcrun", "devicectl", "device", "copy"]
options = [
    "--device",
    args.device,
    "--domain-type",
    "appDataContainer",
    "--domain-identifier",
    "com.teslabs.hfpcontrol.xctrunner",
    "--timeout",
    "10",
    "--quiet",
]


def copy(direction, source, dest):
    return subprocess.run(
        base + [direction] + options + ["--source", source, "--destination", str(dest)],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )


try:
    cmd = json.loads(args.command)
    if not isinstance(cmd, dict) or not isinstance(cmd.get("action"), str):
        raise TypeError("Expected an object with an action string")
except (ValueError, TypeError) as error:
    parser.error(str(error))
cmd["id"] = uuid.uuid4().hex
(root / "command.json").write_text(json.dumps(cmd))
result = copy("to", str(root / "command.json"), "Documents/command.json")
if result.returncode:
    sys.exit(result.stderr or result.stdout)
if cmd["action"] == "stop":
    sys.exit(0)
deadline = time.monotonic() + 45
while time.monotonic() < deadline:
    result = copy("from", "Documents/response.json", root / "response.json")
    if result.returncode == 0:
        response = json.loads((root / "response.json").read_text())
        if response["id"] == cmd["id"]:
            print(json.dumps(response))
            for filename in ["tree.txt", "screen.png"]:
                result = copy("from", "Documents/" + filename, root / filename)
                if result.returncode:
                    sys.exit(result.stderr or result.stdout)
            print((root / "tree.txt").read_text())
            if response["result"] != "ok":
                sys.exit(1)
            break
    time.sleep(1)
else:
    sys.exit("No response from UI runner")

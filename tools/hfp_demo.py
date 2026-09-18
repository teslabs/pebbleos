# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
"""Add or update a RAM-only HFP demo contact, or monitor the embedded host.

This is a console helper, not a Bluetooth host. Calls run on the watch and
continue with this tool closed and the serial cable disconnected.
"""

import argparse
import re
import time

from pebble import commander, pulse2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tty", required=True)
    parser.add_argument(
        "--contact-name", help="Display name, including spaces, up to 23 UTF-8 bytes"
    )
    parser.add_argument(
        "--contact-number", help="Up to 32 dial characters, optionally starting with +"
    )
    parser.add_argument(
        "--monitor",
        action="store_true",
        help="Poll host and audio metadata every 5 seconds",
    )
    parser.add_argument("--launch", action="store_true", help="Open the Phone demo app")
    args = parser.parse_args()
    if bool(args.contact_name) != bool(args.contact_number):
        parser.error("Provide both --contact-name and --contact-number")
    if args.contact_name and (
        not args.contact_name.strip()
        or not args.contact_name.isprintable()
        or len(args.contact_name.encode("utf-8")) > 23
    ):
        parser.error("Contact name must be printable and fit in 23 UTF-8 bytes")
    if args.contact_number and (
        not re.fullmatch(r"\+?[0-9*#]{1,32}", args.contact_number)
        or len(args.contact_number) > 32
        or not any(c.isdigit() for c in args.contact_number)
    ):
        parser.error("Invalid phone number")
    interface = pulse2.Interface.open_dbgserial(url=args.tty)
    try:
        link = interface.get_link(timeout=10)
        if link is None:
            raise RuntimeError("Watch did not establish a PULSE connection")
        prompt = commander.apps.Prompt(link)
        try:
            if args.contact_name:
                for line in prompt.command_and_response(
                    f"bt hfp contact hex:{args.contact_name.encode('utf-8').hex()} {args.contact_number}",
                    timeout=5,
                ):
                    print(line, flush=True)
                for line in prompt.command_and_response("bt hfp contacts", timeout=5):
                    print(line, flush=True)
            if args.launch:
                prompt.command_and_response("app launch -194", timeout=5)
            while True:
                for command in ("bt hfp status", "bt audio probe"):
                    for line in prompt.command_and_response(command, timeout=5):
                        print(line, flush=True)
                if not args.monitor:
                    break
                time.sleep(5)
        finally:
            prompt.close()
    finally:
        interface.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Write the Meson cross file for the firmware toolchain.

Meson has to know its compiler before it reads meson.build, and a cross
file cannot look one up or refer to the source tree, so the file is
generated here: `./pbl configure` runs this first and points `meson setup`
at the result.

The compiler is wrapped by ccwrap.sh, which is how every object gets the
name of the source it was built from (see that script). ccache is left out
for the same reason: it would see the wrapper as the compiler.
"""

import argparse
import contextlib
import io
import os
import shutil
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)

from tools.waf.pebble_sdk_locator import activate_sdk

TOOLS = {
    "c": "gcc",
    "cpp": "g++",
    "ar": "gcc-ar",
    "ranlib": "gcc-ranlib",
    "objcopy": "objcopy",
    "objdump": "objdump",
    "strip": "strip",
    "nm": "nm",
    "size": "size",
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cross-compile", default="arm-none-eabi-")
    args = parser.parse_args()

    # The SDK ships the toolchain the firmware is expected to be built with
    # (picolibc included), so its binaries come first.
    notes = io.StringIO()
    with contextlib.redirect_stdout(notes):
        activate_sdk(REPO_ROOT)
    if notes.getvalue().strip():
        print(notes.getvalue().strip(), file=sys.stderr)

    resolved = {}
    for name, tool in TOOLS.items():
        path = shutil.which(args.cross_compile + tool)
        if path is None:
            if name in ("c", "ar", "objcopy"):
                sys.exit(f"{args.cross_compile}{tool} not found on PATH")
            continue
        resolved[name] = path

    wrapper = os.path.join(REPO_ROOT, "tools", "meson", "ccwrap.sh")
    lines = ["[binaries]", f"c = ['{wrapper}', '{resolved.pop('c')}']"]
    if "cpp" in resolved:
        lines.append(f"cpp = ['{wrapper}', '{resolved.pop('cpp')}']")
    for name, path in resolved.items():
        lines.append(f"{name} = '{path}'")
    lines += [
        "",
        "[host_machine]",
        "system = 'none'",
        "cpu_family = 'arm'",
        "cpu = 'arm'",
        "endian = 'little'",
        "",
        "[properties]",
        "# The firmware supplies its own startup code and links against a linker",
        "# script that is only generated later, so a link test cannot succeed.",
        "skip_sanity_check = true",
        "",
        "[built-in options]",
        "# Deterministic archives: no timestamps or UIDs inside .a files.",
        "c_link_args = []",
        "",
    ]

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    main()

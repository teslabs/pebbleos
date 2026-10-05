# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Generate C from a devicetree blob and the dtmap specs.

    python -m tools.dtmap --dtb board.dtb --dtmap-root fw --dtmap-root soc \\
        --header hw.h --source hw.c --kconfig Kconfig.dt
"""

import argparse
import os
import sys

from . import fdt, spec
from .generate import Generator, types_header_name, types_text


def write_if_changed(path, text):
    if os.path.exists(path):
        with open(path) as f:
            if f.read() == text:
                return
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)


def main(argv=None):
    parser = argparse.ArgumentParser(prog="dtmap", description=__doc__.split("\n")[0])
    parser.add_argument("--dtb", required=True)
    parser.add_argument(
        "--dtmap-root",
        action="append",
        default=[],
        help="directory searched for *.dtmap.yaml (repeatable)",
    )
    parser.add_argument(
        "-I",
        "--include-dir",
        action="append",
        default=[],
        help="where 'lookup' headers are searched (repeatable)",
    )
    parser.add_argument(
        "--types-dir",
        help="where the generated types headers go (devicetree/types/...)",
    )
    parser.add_argument(
        "--dtconf",
        action="append",
        default=[],
        help="setting file (repeatable, last wins)",
    )
    parser.add_argument("--header", required=True)
    parser.add_argument("--source", required=True)
    parser.add_argument("--kconfig")
    parser.add_argument("--depfile", help="write the dtmap files used, one per line")
    args = parser.parse_args(argv)

    try:
        paths = spec.find(args.dtmap_root)
        specs = spec.load(paths)
        tree = fdt.load(args.dtb)
        gen = Generator(tree, specs, args.include_dir, spec.load_settings(args.dtconf))
        gen.run()
    except fdt.DtError as err:
        print(f"dtmap: error: {err}", file=sys.stderr)
        return 1

    if args.types_dir:
        for one in {id(x): x for x in specs.values()}.values():
            if any(one.generated_types()):
                write_if_changed(
                    os.path.join(args.types_dir, types_header_name(one)),
                    types_text(one),
                )
    name = os.path.basename(args.dtb)
    write_if_changed(args.header, gen.header_text(name))
    write_if_changed(args.source, gen.source_text(name, os.path.basename(args.header)))
    if args.kconfig:
        write_if_changed(args.kconfig, gen.kconfig_text(name))
    if args.depfile:
        write_if_changed(
            args.depfile, "".join(os.path.abspath(p) + "\n" for p in sorted(paths))
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Build the board devicetree and generate C from it.

``generate`` preprocesses and compiles ``boards/<board>/<board>[_<rev>].dts``,
writes an annotated merged source (``<board>.dts``, every property tagged with
the file and line that set it), and runs dtmap. ``schema`` and ``validate``
check the blob against the Linux bindings plus ours with dt-validate.
"""

import argparse
import glob
import os
import shlex
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)

from tools import boards
from tools.dtmap import fdt, spec
from tools.dtmap.__main__ import write_if_changed
from tools.dtmap.generate import Generator, types_header_name, types_text

LINUX_DT = os.path.join("third_party", "devicetree", "devicetree-rebasing")
DTMAP_ROOTS = ("dts", "soc", "fw", "subsys")


def board_dts(srcdir, board):
    names = [f"{board.name}_{board.revision}.dts"] if board.revision else []
    names.append(f"{board.name}.dts")
    for name in names:
        path = os.path.join(srcdir, "boards", board.name, name)
        if os.path.exists(path):
            return path
    return None


def settings_files(srcdir, board, variant, extra):
    """Setting files, lowest precedence first: board, revision, variant, extra."""
    base = os.path.join(srcdir, "boards", board.name, board.name)
    names = [f"{base}.dtconf.yaml"]
    if board.revision:
        names.append(f"{base}_{board.revision}.dtconf.yaml")
    if variant != "normal":
        names.append(f"{base}_{variant}.dtconf.yaml")
    return [n for n in names if os.path.exists(n)] + list(extra)


def include_dirs(srcdir, board):
    return [
        os.path.join(srcdir, "boards", board.name),
        os.path.join(srcdir, "soc"),
        os.path.join(srcdir, "dts", "include"),
        os.path.join(srcdir, LINUX_DT, "include"),
        os.path.join(srcdir, LINUX_DT, "src"),
    ]


def run(cmd, what):
    res = subprocess.run(cmd, capture_output=True, text=True, check=False)
    output = (res.stdout + res.stderr).strip()
    if res.returncode != 0 or output:
        print(f"devicetree: {what} failed:\n{output}", file=sys.stderr)
        print(f"  command: {shlex.join(cmd)}", file=sys.stderr)
        sys.exit(1)


def read_depfile(path):
    with open(path) as f:
        text = f.read().replace("\\\n", " ")
    _, _, deps = text.partition(":")
    return [d for d in deps.split() if d]


def write_cmake(path, values):
    lines = []
    for key, value in values.items():
        if isinstance(value, list):
            value = ";".join(v.replace(";", "\\;") for v in value)
        lines.append(f'set({key} "{value}")\n')
    write_if_changed(path, "".join(lines))


def cmd_generate(args):
    board = boards.parse_board(args.srcdir, args.board)
    outdir = os.path.join(args.builddir, "devicetree")
    os.makedirs(outdir, exist_ok=True)
    kconfig = os.path.join(outdir, "Kconfig.dt")
    cmake_out = os.path.join(outdir, "devicetree.cmake")

    gen_inc = os.path.join(args.builddir, "generated", "include", "devicetree")
    dtmaps = spec.find(os.path.join(args.srcdir, r) for r in DTMAP_ROOTS)
    tooling = glob.glob(os.path.join(REPO_ROOT, "tools", "dtmap", "*.py"))
    tooling.append(os.path.join(REPO_ROOT, "tools", "dtmap", "dtmap-schema.yaml"))
    try:
        specs = spec.load(dtmaps)
    except fdt.DtError as err:
        print(f"devicetree: dtmap: {err}", file=sys.stderr)
        return 1
    # The types depend on the specs only: every board gets all of them.
    gen_root = os.path.join(args.builddir, "generated", "include")
    for one in {id(x): x for x in specs.values()}.values():
        if any(one.generated_types()):
            write_if_changed(
                os.path.join(gen_root, types_header_name(one)), types_text(one)
            )

    dts = board_dts(args.srcdir, board)
    if dts is None:
        write_if_changed(kconfig, "# No devicetree for this board.\n")
        write_cmake(
            cmake_out,
            {"PBL_DT_ENABLED": "OFF", "PBL_DT_DEPENDS": sorted(dtmaps + tooling)},
        )
        print("no devicetree")
        return 0

    incs = include_dirs(args.srcdir, board)
    pre = os.path.join(outdir, "board.dts.pre")
    dep = os.path.join(outdir, "board.dts.d")
    dtb = os.path.join(outdir, "board.dtb")
    run(
        [args.cc, "-E", "-nostdinc", "-undef", "-D__DTS__", "-x", "assembler-with-cpp"]
        + [f"-I{d}" for d in incs]
        + ["-MD", "-MF", dep, "-MT", pre, "-o", pre, dts],
        "preprocessing",
    )
    run(["dtc", "-@", "-I", "dts", "-O", "dtb", "-o", dtb, pre], "dtc")
    run(
        ["dtc", "-T", "-T", "-I", "dts", "-O", "dts", "-o",
         os.path.join(outdir, f"{board.name}.dts"), pre],
        "dtc (merged source)",
    )  # fmt: skip

    header = os.path.join(gen_inc, "hw.h")
    source = os.path.join(outdir, "hw.c")
    confs = settings_files(args.srcdir, board, args.variant, args.dtconf)
    try:
        gen = Generator(fdt.load(dtb), specs, incs, spec.load_settings(confs))
        gen.run()
    except fdt.DtError as err:
        print(f"devicetree: dtmap: {err}", file=sys.stderr)
        return 1
    name = os.path.relpath(dts, args.srcdir)
    write_if_changed(header, gen.header_text(name))
    write_if_changed(source, gen.source_text(name, "devicetree/hw.h"))
    write_if_changed(kconfig, gen.kconfig_text(name))

    headers = [
        os.path.join(d, h)
        for h in gen.headers
        for d in incs
        if os.path.exists(os.path.join(d, h))
    ]
    write_cmake(
        cmake_out,
        {
            "PBL_DT_ENABLED": "ON",
            "PBL_DT_DTB": dtb,
            "PBL_DT_SOURCE": source,
            "PBL_DT_DEPENDS": sorted(
                set(
                    read_depfile(dep)
                    + dtmaps
                    + tooling
                    + headers
                    + list(gen.tables)
                    + confs
                )
            ),
        },
    )
    print(os.path.relpath(dts, args.srcdir))
    return 0


def vendor_prefixes(linux_bindings, extra, outdir):
    """Write Linux's vendor-prefixes.yaml plus the prefixes listed in
    ``extra`` into ``outdir``, which must come first on the schema path."""
    with open(os.path.join(linux_bindings, "vendor-prefixes.yaml")) as f:
        text = f.read()
    anchor = "  # Keep list in alphabetical order.\n"
    if anchor not in text:
        sys.exit("devicetree: unexpected layout of Linux vendor-prefixes.yaml")
    entries = []
    with open(extra) as f:
        for line in f:
            if not line.strip() or line.startswith("#"):
                continue
            prefix, description = line.rstrip("\n").split("\t", 1)
            if f'"^{prefix},.*":' in text:
                sys.exit(
                    f"devicetree: Linux now has vendor prefix '{prefix}', drop it from {extra}"
                )
            entries.append(f'  "^{prefix},.*":\n    description: {description}\n')
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "vendor-prefixes.yaml"), "w") as f:
        f.write(text.replace(anchor, anchor + "".join(entries)))


def cmd_schema(args):
    prefixes = os.path.join(os.path.dirname(args.output), "vendor-prefixes")
    vendor_prefixes(args.bindings[0], args.vendor_prefixes, prefixes)
    with open(args.output + ".tmp", "w") as out:
        res = subprocess.run(
            [args.dt_mk_schema, "-j", prefixes] + args.bindings,
            stdout=out,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
    # Linux's own registry is shadowed by ours on purpose.
    shadowed = "ignoring duplicate '$id' value 'http://devicetree.org/schemas/vendor-prefixes.yaml'"
    errors = [ln for ln in res.stderr.splitlines() if ln.strip() and shadowed not in ln]
    if res.returncode != 0 or errors:
        print("devicetree: dt-mk-schema failed:\n" + "\n".join(errors), file=sys.stderr)
        return 1
    os.replace(args.output + ".tmp", args.output)
    return 0


def cmd_validate(args):
    run([args.dt_validate, "-m", "-s", args.schema, args.dtb], "dt-validate")
    with open(args.stamp, "w"):
        pass
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("generate")
    p.add_argument("--srcdir", required=True)
    p.add_argument("--builddir", required=True)
    p.add_argument("--board", required=True)
    p.add_argument("--cc", required=True, help="C compiler used as the preprocessor")
    p.add_argument("--variant", default="normal")
    p.add_argument(
        "--dtconf", action="append", default=[], help="extra setting file, applied last"
    )
    p.set_defaults(func=cmd_generate)

    p = sub.add_parser("schema")
    p.add_argument("--dt-mk-schema", required=True)
    p.add_argument("--output", required=True)
    p.add_argument(
        "--vendor-prefixes", required=True, help="vendor prefixes Linux lacks"
    )
    p.add_argument("bindings", nargs="+", help="binding directories, Linux's first")
    p.set_defaults(func=cmd_schema)

    p = sub.add_parser("validate")
    p.add_argument("--dt-validate", required=True)
    p.add_argument("--schema", required=True)
    p.add_argument("--dtb", required=True)
    p.add_argument("--stamp", required=True)
    p.set_defaults(func=cmd_validate)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())

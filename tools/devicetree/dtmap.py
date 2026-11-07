#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Turn a board devicetree into C, as told by the *.dtmap.yaml file of each driver.

The tree stays on the host: a node with status okay whose driver is enabled in Kconfig becomes a
call to the instance macro of its driver, in an instance file the driver includes. The driver owns
the types and the macro, a spec only says where each macro argument comes from.

A labelled node is reached from other code as dt_<label>, of the class type of its driver, declared
in devicetree/devices.h. The class object has to be the first member of the driver's type.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import fdt
from fdt import DtError

SPEC_DIRS = ("drivers", "kernel", "soc")
SPEC_KEYS = {
    "compatible",
    "config",
    "include",
    "type",
    "class",
    "define",
    "args",
    "irqs",
    "provides",
    "pinctrl",
    "export",
}
PINCTRL_KEYS = {"controller", "pin", "flags", "values"}
CLASS_KEYS = {"type", "member", "include"}
IRQ_KEYS = {"irq", "prio", "isr"}
ARG_KINDS = {
    "name",
    "reg",
    "prop",
    "strings",
    "irq",
    "clock",
    "dma",
    "pinctrl",
    "parent",
    "literal",
}
ARG_OPTIONS = {"default", "as", "wrap", "field"}


def c_ident(name):
    return "".join(c if c.isalnum() else "_" for c in name)


def as_list(value):
    return value if isinstance(value, list) else [value]


class Spec:
    def __init__(self, path, data):
        self.path = path
        self._check_keys(data, SPEC_KEYS, "spec")
        for key in ("compatible", "config"):
            if key not in data:
                raise self._error(f"missing '{key}'")
        if len({"define", "type", "class"} & data.keys()) not in (0, 3):
            raise self._error("'define', 'type' and 'class' go together")
        self.compatibles = as_list(data["compatible"])
        self.config = data["config"]
        self.includes = as_list(data.get("include", []))
        self.type = data.get("type")
        self.cls = data.get("class")
        self.define = data.get("define")
        self.args = data.get("args", [])
        self.irqs = data.get("irqs", [])
        self.provides = data.get("provides", {})
        self.pinctrl = data.get("pinctrl")
        self.export = data.get("export")
        for arg in self.args:
            self._check_keys(arg, ARG_KINDS | ARG_OPTIONS, "argument")
            if len(arg.keys() & ARG_KINDS) != 1:
                raise self._error(f"argument {arg}: needs one of {sorted(ARG_KINDS)}")
        for irq in self.irqs:
            self._check_keys(irq, IRQ_KEYS, "irqs entry", required=True)
        if self.pinctrl is not None:
            self._check_keys(self.pinctrl, PINCTRL_KEYS, "pinctrl")
        if self.cls is not None:
            self._check_keys(self.cls, CLASS_KEYS, "class", required=True)

    def _error(self, message):
        return DtError(f"{self.path}: {message}")

    def _check_keys(self, data, known, what, required=False):
        if not isinstance(data, dict):
            raise self._error(f"{what}: {data!r} is not a mapping")
        if data.keys() - known:
            raise self._error(f"{what}: unknown {sorted(data.keys() - known)}")
        if required and known - data.keys():
            raise self._error(f"{what}: missing {sorted(known - data.keys())}")


def find_specs(srcdir):
    paths = []
    for directory in SPEC_DIRS:
        paths += sorted((Path(srcdir) / directory).rglob("*.dtmap.yaml"))
    return paths


def load_specs(paths):
    specs = {}
    for path in paths:
        with open(path) as f:
            spec = Spec(path, yaml.safe_load(f))
        for compatible in spec.compatibles:
            if compatible in specs:
                raise DtError(f"{path}: another spec has '{compatible}'")
            specs[compatible] = spec
    return specs


def instance_file(spec):
    return f"devicetree/instances/{spec.compatibles[0]}.inc"


def instance_files(specs):
    return sorted({instance_file(spec) for spec in specs.values() if spec.define})


def enabled(node):
    while node is not None:
        if not node.status_okay:
            return False
        node = node.parent
    return True


class Device:
    def __init__(self, node, spec):
        self.node = node
        self.spec = spec
        self.label = node.labels[0] if node.labels else None
        self.name = self.label or c_ident(node.name)
        self.sym = f"dt_{self.label}" if self.label else None
        self.inst = f"{self.sym}_inst" if self.label else f"dt_node{c_ident(node.path)}"

    def error(self, message):
        return DtError(f"{self.node.path}: {message}")


class Output:
    def __init__(self):
        self.includes = set()
        self.body = []


class Generator:
    def __init__(self, tree, specs, config):
        self.tree = tree
        self.specs = specs
        self.config = config
        self.outputs = {name: Output() for name in instance_files(specs)}
        self.out = None
        self.declarations = []
        self.unbound = []
        self.devices = {}

    def _device(self, node):
        if node.path not in self.devices:
            spec = next(
                (self.specs[c] for c in node.compatibles if c in self.specs), None
            )
            self.devices[node.path] = Device(node, spec)
        return self.devices[node.path]

    def _bound(self, dev):
        return dev.spec is not None and dev.spec.config in self.config

    def run(self):
        for node in self.tree.nodes():
            if (
                node.parent is None
                or not node.compatibles
                or "simple-bus" in node.compatibles
            ):
                continue
            if not enabled(node):
                continue
            dev = self._device(node)
            if not self._bound(dev):
                self.unbound.append(node.path)
            elif dev.spec.define:
                self._instantiate(dev)

    def _instantiate(self, dev):
        spec = dev.spec
        self.out = self.outputs[instance_file(spec)]
        self.out.includes.update(spec.includes)
        args = [dev.inst] + [self._arg(dev, arg) for arg in spec.args]
        self.out.body.append(f"{spec.define}({', '.join(args)});")
        cls = spec.cls
        if dev.sym:
            self.out.includes.add("stddef.h")
            self.out.body.append(
                f"static_assert(offsetof({spec.type}, {cls['member']}) == 0, "
                f'"{cls["member"]} must come first");'
            )
            self.out.body.append(
                f"extern const {cls['type']} {dev.sym} "
                f'__attribute__((alias("{dev.inst}")));'
            )
            self.declarations.append((cls["include"], cls["type"], dev.sym))
        for irq in spec.irqs:
            self.out.includes.add("pbl/kernel/irq.h")
            line = self._interrupt(dev, irq["irq"], "irq")
            self.out.body.append(
                f"PBL_IRQ_CONNECT_NUM({line}, {irq['prio']}, {irq['isr']}, &{dev.inst}, 0);"
            )
        if spec.export:
            self.out.body.append(
                f"const {cls['type']} *const {spec.export} = &{dev.inst}.{cls['member']};"
            )

    def _provider(self, consumer, node, kind):
        """A node others refer to: disabled or driverless ones break the consumer at build time."""
        dev = self._device(node)
        if not enabled(node):
            raise consumer.error(f"'{kind}' from {node.path}, which is disabled")
        if not self._bound(dev):
            raise consumer.error(
                f"'{kind}' from {node.path}, which has no enabled driver"
            )
        self.out.includes.update(dev.spec.includes)
        return dev

    def _provided(self, consumer, node, kind, cells):
        dev = self._provider(consumer, node, kind)
        if kind not in dev.spec.provides:
            raise consumer.error(f"{node.path} provides no '{kind}'")

        def field(match):
            key = match.group(1)
            if key == "sym":
                if dev.sym is None:
                    raise consumer.error(
                        f"'{kind}' from {node.path}, which needs a label"
                    )
                self.out.includes.add("devicetree/devices.h")
                return dev.sym
            if int(key) >= len(cells):
                raise consumer.error(f"'{kind}' from {node.path} lacks cell {key}")
            return str(cells[int(key)])

        return re.sub(r"\{(sym|\d+)\}", field, dev.spec.provides[kind])

    def _phandles(self, dev, prop, cells_prop):
        values = dev.node.u32s(prop)
        entries = []
        pos = 0
        while pos < len(values):
            target = self.tree.phandle(values[pos], f"{dev.node.path}: '{prop}'")
            count = target.u32(cells_prop)
            entries.append((target, values[pos + 1 : pos + 1 + count]))
            pos += 1 + count
        return entries

    def _interrupt(self, dev, index, kind):
        node = dev.node
        while node is not None and not node.has("interrupt-parent"):
            node = node.parent
        if node is None:
            raise dev.error("no interrupt parent")
        parent = self.tree.phandle(node.u32("interrupt-parent"), dev.node.path)
        count = parent.u32("#interrupt-cells")
        cells = dev.node.u32s("interrupts")
        entry = cells[index * count : (index + 1) * count]
        if len(entry) != count:
            raise dev.error(f"no interrupt {index}")
        return self._provided(dev, parent, kind, entry)

    def _reg(self, dev, index):
        address_cells = dev.node.address_cells
        entry_cells = address_cells + dev.node.size_cells
        cells = dev.node.u32s("reg")
        start = index * entry_cells
        if start + entry_cells > len(cells):
            raise dev.error(f"no reg entry {index}")
        address = 0
        for cell in cells[start : start + address_cells]:
            address = (address << 32) | cell
        return f"{address:#x}"

    def _dma(self, dev, name):
        names = dev.node.strings("dma-names")
        if name not in names:
            raise dev.error(f"no '{name}' in 'dma-names'")
        target, cells = self._phandles(dev, "dmas", "#dma-cells")[names.index(name)]
        init = self._provided(dev, target, "dma", cells)
        sym = f"{dev.inst}_dma_{c_ident(name)}"
        self.out.body.append(f"static const struct pbl_dma_chan {sym} = {init};")
        return f"&{sym}"

    def _pin(self, ctrl, pin, group):
        spec = ctrl.spec.pinctrl
        flags = spec.get("flags", {})
        values = spec.get("values", {})
        unknown = set(group.props) - {"pins", "function", "phandle", *flags, *values}
        if unknown:
            raise DtError(f"{group.path}: unsupported {sorted(unknown)}")
        parts = [spec["pin"].format(pin=pin, function=group.string("function").upper())]
        parts += [name for prop, name in flags.items() if group.has(prop)]
        parts += [
            name.format(value=group.u32(prop))
            for prop, name in values.items()
            if group.has(prop)
        ]
        return " | ".join(parts)

    def _pinctrl(self, dev, name):
        names = dev.node.strings("pinctrl-names")
        if name not in names:
            raise dev.error(f"no '{name}' in 'pinctrl-names'")
        prop = f"pinctrl-{names.index(name)}"
        states = [
            self.tree.phandle(value, f"{dev.node.path}: '{prop}'")
            for value in dev.node.u32s(prop)
        ]
        if len(states) != 1:
            raise dev.error(f"'{prop}' must point at one state")
        ctrl = self._provider(dev, states[0].parent, "pinctrl")
        if ctrl.spec.pinctrl is None:
            raise dev.error(f"{ctrl.node.path} is no pin controller")
        sym = f"{dev.inst}_pinctrl_{c_ident(name)}"
        pins = [
            self._pin(ctrl, pin, group)
            for group in states[0].children
            for pin in group.strings("pins")
        ]
        args = ", ".join([sym, ctrl.spec.pinctrl["controller"], *pins])
        self.out.body.append(f"PBL_PINCTRL_STATE_DEFINE({args});")
        return f"&{sym}"

    def _value(self, dev, arg):
        """The argument as C, or None when the node does not have it."""
        kind = next(iter(arg.keys() & ARG_KINDS))
        key = arg[kind]
        node = dev.node
        if kind == "name":
            return f'"{dev.name}"'
        if kind == "literal":
            return str(key)
        if kind == "reg":
            return self._reg(dev, key)
        if kind == "parent":
            return self._provided(dev, node.parent, key, [])
        if kind == "irq":
            return self._interrupt(dev, key, arg.get("as", "irq"))
        if kind == "clock":
            target, cells = self._phandles(dev, "clocks", "#clock-cells")[key]
            return self._provided(dev, target, "clock", cells)
        if kind == "prop":
            return str(node.u32(key)) if node.has(key) else None
        if kind == "strings":
            return ", ".join(node.strings(key)) if node.has(key) else None
        if kind == "dma":
            return self._dma(dev, key) if node.has("dmas") else None
        if kind == "pinctrl":
            return self._pinctrl(dev, key) if node.has("pinctrl-names") else None
        raise AssertionError(kind)

    def _arg(self, dev, arg):
        value = self._value(dev, arg)
        if value is None:
            if "default" not in arg:
                raise dev.error(f"missing what {arg} needs")
            value = ", ".join(str(item) for item in as_list(arg["default"]))
        if "wrap" in arg:
            value = f"{arg['wrap']}({value})"
        if "field" in arg:
            value = f".{arg['field']} = {value}"
        return value

    def instances(self):
        """The text of each instance file, by name."""
        result = {}
        for name, out in self.outputs.items():
            lines = ["/* Generated from the board devicetree. */", ""]
            lines += [f"#include <{header}>" for header in sorted(out.includes)]
            lines += ["", *out.body]
            result[name] = "\n".join(lines) + "\n"
        return result

    def header(self):
        includes = sorted({include for include, _, _ in self.declarations})
        lines = ["/* Generated from the board devicetree. */", "", "#pragma once", ""]
        lines += [f"#include <{include}>" for include in includes]
        lines.append("")
        lines += [f"extern const {type_} {sym};" for _, type_, sym in self.declarations]
        if self.unbound:
            lines += ["", "/* Enabled, no driver in this configuration:"]
            lines += [f" * {path}" for path in self.unbound]
            lines.append(" */")
        return "\n".join(lines) + "\n"


def write_if_changed(path, content):
    path = Path(path)
    if path.exists() and path.read_bytes() == content:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)


def find_dts(srcdir, board, revision):
    directory = Path(srcdir) / "boards" / board
    names = ([f"{board}_{revision}.dts"] if revision else []) + [f"{board}.dts"]
    return next(
        (directory / name for name in names if (directory / name).exists()), None
    )


def compile_dts(dts, cc, include_dirs, outdir):
    """Compile to outdir/board.dtb, returning it and the sources it came from."""
    dtc = shutil.which("dtc")
    if dtc is None:
        raise DtError(f"{dts} needs dtc: install it")
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    pre, depfile, dtb = (
        outdir / "board.dts.pre",
        outdir / "board.dts.d",
        outdir / "board.dtb",
    )
    cpp = [
        cc,
        "-E",
        "-nostdinc",
        "-undef",
        "-D__DTS__",
        "-x",
        "assembler-with-cpp",
        "-P",
    ]
    cpp += ["-MD", "-MF", str(depfile), "-MT", "board.dtb"]
    cpp += [flag for path in include_dirs for flag in ("-I", str(path))]
    result = subprocess.run(
        [*cpp, "-o", str(pre), str(dts)], capture_output=True, text=True, check=False
    )
    if result.returncode != 0:
        raise DtError(f"preprocessing {dts} failed:\n{result.stderr}")
    tmp = outdir / "board.dtb.new"
    result = subprocess.run(
        [dtc, "-@", "-I", "dts", "-O", "dtb", "-o", str(tmp), str(pre)],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0 or result.stderr:
        raise DtError(f"dtc failed on {dts}:\n{result.stderr}")
    write_if_changed(dtb, tmp.read_bytes())
    tmp.unlink()
    sources = depfile.read_text().replace("\\\n", " ").split(":", 1)[1].split()
    return dtb, sources


def kconfig_symbol(compatible):
    return "DT_HAS_" + re.sub(r"[^A-Z0-9]", "_", compatible.upper()) + "_ENABLED"


def kconfig_text(tree, compatibles):
    present = set()
    if tree is not None:
        for node in tree.nodes():
            if enabled(node):
                present.update(node.compatibles)
    lines = ["# Generated from the board devicetree."]
    for compatible in sorted(compatibles):
        lines += [
            "",
            f"config {kconfig_symbol(compatible)}",
            f"\tdef_bool {'y' if compatible in present else 'n'}",
        ]
    return "\n".join(lines) + "\n"


def configure(srcdir, builddir, board, revision, cc):
    """Prepare what Kconfig needs.

    Writes Kconfig.dt. Returns the dtb or None, the files generated from it, and what it all depends on.
    """
    outdir = Path(builddir) / "devicetree"
    paths = find_specs(srcdir)
    deps = [str(path) for path in paths]
    specs = load_specs(paths)
    tree, dtb = None, None
    dts = find_dts(srcdir, board, revision)
    if dts is not None:
        srcdir = Path(srcdir)
        dtb, sources = compile_dts(
            dts, cc, [srcdir / "boards", srcdir / "soc", srcdir / "include"], outdir
        )
        deps += sources
        tree = fdt.load(dtb)
    kconfig = outdir / "Kconfig.dt"
    write_if_changed(kconfig, kconfig_text(tree, specs).encode())
    return str(dtb) if dtb else None, instance_files(specs), deps


def load_config(path):
    config = set()
    with open(path) as f:
        for line in f:
            match = re.match(r"CONFIG_(\w+)=y$", line.strip())
            if match:
                config.add(match.group(1))
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--srcdir", required=True)
    parser.add_argument("--dtb", required=True)
    parser.add_argument("--config", required=True, help="the .config file")
    parser.add_argument(
        "--outdir", required=True, help="include directory to generate into"
    )
    args = parser.parse_args()

    try:
        specs = load_specs(find_specs(args.srcdir))
        generator = Generator(fdt.load(args.dtb), specs, load_config(args.config))
        generator.run()
    except DtError as error:
        sys.exit(f"devicetree: {error}")

    outputs = {"devicetree/devices.h": generator.header(), **generator.instances()}
    for name, text in outputs.items():
        path = Path(args.outdir) / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)


if __name__ == "__main__":
    main()

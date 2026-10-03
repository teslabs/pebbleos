# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Turn a devicetree and the dtmap specs of its drivers into plain C."""

import os
import re

import yaml

from .fdt import DtError
from .spec import CELLS_PROPS, PROVIDER_OF

SYMBOL_PREFIX = "hw_"


class Missing(Exception):
    pass


class Dropped(Missing):
    """The value maps to nothing; unlike a missing one, it takes no default."""


class C(str):
    """A C expression, emitted verbatim."""


def c_string(value):
    escaped = value.replace("\\", "\\\\").replace('"', '\\"')
    return C(f'"{escaped}"')


def render(value):
    if isinstance(value, C):
        return value
    if isinstance(value, bool):
        return C("true" if value else "false")
    if isinstance(value, int):
        return C(str(value))
    if isinstance(value, str):
        return c_string(value)
    if isinstance(value, (list, tuple)):
        return C("{" + ", ".join(render(v) for v in value) + "}")
    raise DtError(f"cannot render {value!r} as C")


def mount_matrix(raw, where):
    try:
        values = [int(v) for v in raw]
    except ValueError:
        raise DtError(f"{where}: mount-matrix entries must be -1, 0 or 1") from None
    if len(values) != 9:
        raise DtError(f"{where}: mount-matrix needs 9 entries")
    axis_map, axis_dir = [], []
    for row in range(3):
        cells = values[row * 3 : row * 3 + 3]
        nonzero = [(col, v) for col, v in enumerate(cells) if v]
        if len(nonzero) != 1 or nonzero[0][1] not in (-1, 1):
            raise DtError(
                f"{where}: mount-matrix row {row} must be a signed unit vector"
            )
        axis_map.append(nonzero[0][0])
        axis_dir.append(nonzero[0][1])
    return axis_map, axis_dir


CONVERTERS = {
    "mount-matrix-axis-map": lambda raw, where: mount_matrix(raw, where)[0],
    "mount-matrix-axis-dir": lambda raw, where: mount_matrix(raw, where)[1],
}


def symbol_for(node, taken):
    if node.labels:
        base = node.labels[0]
    else:
        base = node.name.replace("@", "_")
    base = re.sub(r"[^A-Za-z0-9_]", "_", base).lower()
    name = base
    n = 1
    while name in taken:
        n += 1
        name = f"{base}_{n}"
    taken.add(name)
    return name


def kconfig_name(compatible):
    return "DT_HAS_" + re.sub(r"[^A-Za-z0-9]", "_", compatible).upper()


class Ctx:
    def __init__(self, node, inst, cells=None, item=None):
        self.node = node
        self.inst = inst
        self.cells = cells
        self.item = item


class Instance:
    """One devicetree node turned into C objects."""

    def __init__(self, node, spec, sym):
        self.node = node
        self.spec = spec
        self.sym = sym
        self.deps = set()
        self.chunks = []

    def object_sym(self, name):
        for obj in self.spec.objects:
            if obj["name"] == name:
                if obj.get("main"):
                    return SYMBOL_PREFIX + self.sym
                return f"{SYMBOL_PREFIX}{self.sym}_{name}"
        raise DtError(f"{self.spec.path}: no object named '{name}'")


class Generator:
    def __init__(self, tree, specs, include_dirs=()):
        self.tree = tree
        self.specs = specs
        self.include_dirs = list(include_dirs)
        self.instances = {}
        self.order = []
        self.irqs = []
        self.headers = {}
        self.tables = {}
        self._extra_syms = set()

    # -- matching ---------------------------------------------------------

    def match(self, node):
        for compatible in node.compatibles:
            spec = self.specs.get(compatible)
            if spec is not None:
                return spec
        return None

    def collect(self):
        taken = set()
        for node in self.tree.nodes():
            if node is self.tree.root or not node.compatibles:
                continue
            if not node.status_okay:
                continue
            if any(not a.status_okay for a in self._ancestors(node)):
                continue
            spec = self.match(node)
            if spec is None:
                raise DtError(
                    f"{node.path}: no dtmap for compatible "
                    + ", ".join(f"'{c}'" for c in node.compatibles)
                )
            inst = Instance(node, spec, symbol_for(node, taken))
            self.instances[node.path] = inst

    @staticmethod
    def _ancestors(node):
        node = node.parent
        while node is not None:
            yield node
            node = node.parent

    def instance(self, node, where):
        inst = self.instances.get(node.path)
        if inst is None:
            raise DtError(f"{where}: {node.path} is disabled or has no dtmap")
        return inst

    # -- value evaluation -------------------------------------------------

    def value(self, value, ctx):
        if isinstance(value, dict):
            return self.expr(value, ctx)
        if isinstance(value, list):
            return C("{" + ", ".join(self.value(v, ctx) for v in value) + "}")
        if isinstance(value, bool):
            return render(value)
        if isinstance(value, int):
            return render(value)
        return C(value)

    def expr(self, e, ctx):
        try:
            raw = self.raw(e, ctx)
        except Dropped:
            raise
        except Missing:
            if "default" in e:
                return self.value(e["default"], ctx)
            raise
        if "format" in e and not isinstance(raw, C):
            fmt = e["format"]
            raw = C(fmt.format(**raw) if isinstance(raw, dict) else fmt.format(raw))
        out = render(raw)
        if "cast" in e:
            out = C(f"({e['cast']}){out}")
        return out

    def raw(self, e, ctx):
        """The value of an expression before formatting, as Python data."""
        if not isinstance(e, dict):
            return e
        where = f"{ctx.node.path} ({ctx.inst.spec.path})"
        raw = self.source(e, ctx, where)
        if isinstance(raw, C):
            return raw
        if "convert" in e:
            try:
                raw = CONVERTERS[e["convert"]](raw, where)
            except KeyError:
                raise DtError(f"{where}: unknown converter '{e['convert']}'") from None
        if "index" in e and isinstance(raw, list) and "prop" in e:
            if e["index"] >= len(raw):
                raise Missing()
            raw = raw[e["index"]]
        if "map" in e:
            try:
                raw = {str(k): v for k, v in e["map"].items()}[str(raw)]
            except KeyError:
                raise DtError(
                    f"{where}: {raw!r} is not one of {list(e['map'])}"
                ) from None
            if raw is None:
                raise Dropped()
            if isinstance(raw, str):
                raw = C(raw)
        if "lookup" in e:
            raw = self.lookup(e["lookup"], raw, where)
        if "table" in e and "key" not in e["table"]:
            raw = self.table_entry(e["table"], [raw], ctx, where)
        if "case" in e:
            raw = raw.upper() if e["case"] == "upper" else raw.lower()
        return raw

    def table_entry(self, table, keys, ctx, where):
        path = os.path.join(os.path.dirname(ctx.inst.spec.path), table["file"])
        if path not in self.tables:
            try:
                with open(path) as f:
                    self.tables[path] = yaml.safe_load(f)
            except OSError as err:
                raise DtError(f"{where}: cannot read table {path}: {err}") from None
        entry = self.tables[path]
        for key in keys:
            if not isinstance(entry, dict) or key not in entry:
                raise DtError(
                    f"{where}: no {' / '.join(map(str, keys))} in {table['file']}"
                )
            entry = entry[key]
        return entry

    def source(self, e, ctx, where):
        node = ctx.node
        if "table" in e and "key" in e["table"]:
            keys = [self.raw(k, ctx) for k in e["table"]["key"]]
            return self.table_entry(e["table"], keys, ctx, where)
        if "or" in e:
            parts = []
            for part in e["or"]:
                try:
                    parts.append(self.value(part, ctx))
                except Missing:
                    pass
            if not parts:
                raise Missing()
            return C(" | ".join(parts))
        if "prop" in e:
            return self.prop(node, e["prop"], e.get("type", "u32"))
        if "reg" in e:
            return self.reg(node, e["reg"], e.get("part", "address"), where)
        if "node" in e:
            what = e["node"]
            if what == "name":
                return node.basename
            if what == "label":
                if not node.labels:
                    raise Missing()
                return node.labels[0]
            if what == "path":
                return node.path
            if node.unit_address is None:
                raise Missing()
            return int(node.unit_address.split(",")[0], 16)
        if "object" in e:
            return C("&" + ctx.inst.object_sym(e["object"]))
        if "parent" in e:
            return self.provided_ref(node.parent, e["parent"], ctx, where)
        if "cell" in e:
            if ctx.cells is None:
                raise DtError(f"{where}: 'cell' outside a specifier")
            if e["cell"] >= len(ctx.cells):
                raise Missing()
            return ctx.cells[e["cell"]]
        if "item" in e:
            if ctx.item is None:
                raise DtError(f"{where}: 'item' outside an element")
            return ctx.item
        if "flags" in e:
            present = [c for prop, c in e["flags"].items() if node.has(prop)]
            if not present:
                raise Missing()
            return C(" | ".join(present))
        if "interrupt" in e:
            controller, cells = self.interrupt(node, e["interrupt"], where)
            return self.specifier(
                controller,
                "interrupt-controller",
                e.get("specifier", "default"),
                cells,
                ctx,
                where,
            )
        if "phandle" in e:
            return self.phandle(e, ctx, where)
        if "pinctrl" in e:
            return self.pinctrl(e["pinctrl"], e.get("part", "array"), ctx, where)
        raise DtError(f"{where}: expression without a source: {e}")

    def prop(self, node, prop, kind):
        if not node.has(prop):
            if kind == "bool":
                return False
            raise Missing()
        try:
            if kind == "bool":
                return True
            if kind == "u32":
                return node.u32(prop)
            if kind == "s32":
                value = node.u32(prop)
                return value - (1 << 32) if value & 0x80000000 else value
            if kind == "u64":
                return node.u64(prop)
            if kind == "string":
                return node.string(prop)
            if kind == "u8-array":
                return node.u8s(prop)
            if kind == "u32-array":
                return node.u32s(prop)
            if kind == "string-array":
                return node.strings(prop)
        except (ValueError, DtError) as err:
            raise DtError(f"{node.path}: '{prop}' is not a {kind}: {err}") from None
        raise DtError(f"unknown property type '{kind}'")

    def reg(self, node, index, part, where):
        if not node.has("reg"):
            raise Missing()
        acells, scells = node.address_cells, node.size_cells
        cells = node.u32s("reg")
        stride = acells + scells
        if stride == 0 or len(cells) % stride:
            raise DtError(f"{where}: 'reg' does not match #address-cells/#size-cells")
        if index >= len(cells) // stride:
            raise Missing()
        entry = cells[index * stride : (index + 1) * stride]

        def join(words):
            value = 0
            for w in words:
                value = (value << 32) | w
            return value

        if part == "address":
            return C(hex(join(entry[:acells])))
        if part == "size":
            if scells == 0:
                raise Missing()
            return C(hex(join(entry[acells:])))
        raise DtError(f"{where}: 'reg' part must be address or size")

    def lookup(self, lookup, raw, where):
        table = self.header(lookup["header"], where)
        prefix = lookup["prefix"]
        names = [
            n[len(prefix) :]
            for n, v in table.items()
            if n.startswith(prefix) and v == raw
        ]
        if len(names) != 1:
            raise DtError(
                f"{where}: {raw} has {len(names)} names with prefix "
                f"{prefix} in {lookup['header']}"
            )
        return names[0]

    def header(self, name, where):
        if name in self.headers:
            return self.headers[name]
        for d in self.include_dirs:
            path = os.path.join(d, name)
            if os.path.exists(path):
                break
        else:
            raise DtError(f"{where}: header '{name}' not found")
        table = {}
        define = re.compile(
            r"^\s*#\s*define\s+(\w+)\s+\(?\s*(0[xX][0-9a-fA-F]+|\d+)[uU]?\s*\)?\s*$"
        )
        with open(path) as f:
            for line in f:
                m = define.match(line)
                if m:
                    table[m.group(1)] = int(m.group(2), 0)
        self.headers[name] = table
        return table

    # -- references -------------------------------------------------------

    def provide(self, provider, key, where):
        inst = self.instance(provider, where)
        provides = inst.spec.provides.get(key)
        if provides is None:
            raise DtError(f"{where}: {provider.path} does not provide '{key}'")
        return inst, provides

    def provided_ref(self, provider, key, ctx, where):
        inst, provides = self.provide(provider, key, where)
        if "object" not in provides:
            raise DtError(f"{where}: '{key}' of {provider.path} has no object")
        if inst is not ctx.inst:
            ctx.inst.deps.add(inst.node.path)
        return C("&" + inst.object_sym(provides["object"]))

    def specifier(self, provider, key, name, cells, ctx, where):
        inst, provides = self.provide(provider, key, where)
        spec = provides.get("specifiers", {}).get(name)
        if spec is None:
            raise DtError(
                f"{where}: '{key}' of {provider.path} has no specifier '{name}'"
            )
        if inst is not ctx.inst:
            ctx.inst.deps.add(inst.node.path)
        sub = Ctx(provider, inst, cells=cells)
        if isinstance(spec, dict) and "init" in spec:
            return C(self.initializer(spec["init"], sub, indent=1))
        return self.value(spec, sub)

    def interrupt_parent(self, node):
        cur = node
        while cur is not None:
            if cur.has("interrupt-parent"):
                return self.tree.phandle(cur.u32("interrupt-parent"), node.path)
            cur = cur.parent
            if cur is not None and cur.has("interrupt-controller") and cur is not node:
                return cur
        raise DtError(f"{node.path}: no interrupt parent")

    def interrupt(self, node, index, where):
        if node.has("interrupts-extended"):
            entries = self.phandle_entries(
                node, "interrupts-extended", "#interrupt-cells"
            )
            if index >= len(entries):
                raise Missing()
            return entries[index]
        if not node.has("interrupts"):
            raise Missing()
        controller = self.interrupt_parent(node)
        n = controller.u32("#interrupt-cells")
        cells = node.u32s("interrupts")
        if len(cells) % n:
            raise DtError(
                f"{where}: 'interrupts' does not match #interrupt-cells of "
                f"{controller.path}"
            )
        if index >= len(cells) // n:
            raise Missing()
        return controller, cells[index * n : (index + 1) * n]

    def phandle_entries(self, node, prop, cells_prop):
        cells = node.u32s(prop)
        out = []
        i = 0
        while i < len(cells):
            provider = self.tree.phandle(cells[i], f"{node.path}: '{prop}'")
            n = provider.u32(cells_prop) if cells_prop else 0
            out.append((provider, cells[i + 1 : i + 1 + n]))
            i += 1 + n
        return out

    def phandle(self, e, ctx, where):
        node = ctx.node
        prop = e["phandle"]
        if not node.has(prop):
            raise Missing()
        family = "gpios" if prop == "gpios" or prop.endswith("-gpios") else prop
        cells_prop = CELLS_PROPS.get(family)
        entries = self.phandle_entries(node, prop, cells_prop)
        if e.get("part") == "count":
            return len(entries)
        index = e.get("index", 0)
        if index >= len(entries):
            raise Missing()
        provider, cells = entries[index]
        key = e.get("provider", PROVIDER_OF.get(family))
        if key is None:
            raise DtError(f"{where}: say which 'provider' '{prop}' points to")
        if "specifier" in e:
            return self.specifier(provider, key, e["specifier"], cells, ctx, where)
        return self.provided_ref(provider, key, ctx, where)

    def pinctrl(self, state, part, ctx, where):
        node = ctx.node
        names = (
            node.strings("pinctrl-names") if node.has("pinctrl-names") else ["default"]
        )
        if state not in names:
            raise Missing()
        prop = f"pinctrl-{names.index(state)}"
        if not node.has(prop):
            raise Missing()
        elements = []
        element_type = None
        for phandle in node.u32s(prop):
            config = self.tree.phandle(phandle, f"{node.path}: '{prop}'")
            controller = config.parent
            while controller is not None and controller.path not in self.instances:
                controller = controller.parent
            if controller is None:
                raise DtError(f"{where}: {config.path} is not under a pin controller")
            inst, provides = self.provide(controller, "pinctrl", where)
            element = provides.get("element")
            if element is None:
                raise DtError(f"{where}: pinctrl of {controller.path} has no element")
            element_type = element["type"]
            if part == "controller":
                return self.provided_ref(controller, "pinctrl", ctx, where)
            each = element["each"]
            groups = [c for c in config.children] or [config]
            for group in groups:
                items = self.prop(group, each["prop"], each.get("type", "u32-array"))
                for item in items if isinstance(items, list) else [items]:
                    sub = Ctx(group, inst, item=item)
                    if "init" in element:
                        elements.append(
                            self.initializer(element["init"], sub, indent=1)
                        )
                    else:
                        elements.append(self.value(element["value"], sub))
        if part == "controller":
            raise Missing()
        if part == "count":
            return len(elements)
        sym = (
            f"{SYMBOL_PREFIX}{ctx.inst.sym}_pinctrl_{re.sub(r'[^a-z0-9_]', '_', state)}"
        )
        if sym not in self._extra_syms:
            self._extra_syms.add(sym)
            body = "".join(f"  {el},\n" for el in elements)
            ctx.inst.chunks.append(
                f"static const {element_type} {sym}[] = {{\n{body}}};\n"
            )
        return C(sym)

    # -- emission ---------------------------------------------------------

    def initializer(self, init, ctx, indent=0):
        tree = {}
        for key, value in init.items():
            try:
                c = self.value(value, ctx)
            except Missing:
                if isinstance(value, dict) and value.get("optional"):
                    continue
                raise DtError(
                    f"{ctx.node.path}: '{key}' of {ctx.inst.spec.path} needs a value "
                    "the node does not have"
                ) from None
            cur = tree
            parts = key.split(".")
            for part in parts[:-1]:
                cur = cur.setdefault(part, {})
            cur[parts[-1]] = c
        return self._format(tree, indent)

    def _format(self, tree, indent):
        pad = "  " * (indent + 1)
        lines = ["{"]
        for key, value in tree.items():
            if isinstance(value, dict):
                value = self._format(value, indent + 1)
            lines.append(f"{pad}.{key} = {value},")
        lines.append("  " * indent + "}")
        return "\n".join(lines)

    def emit_instance(self, inst):
        ctx = Ctx(inst.node, inst)
        objects = []
        for obj in inst.spec.objects:
            sym = inst.object_sym(obj["name"])
            storage = "" if obj.get("main") else "static "
            qual = "const " if obj.get("const", True) else ""
            decl = f"{storage}{qual}{obj['type']} {sym}"
            if "init" in obj:
                objects.append(f"{decl} = {self.initializer(obj['init'], ctx)};\n")
            else:
                objects.append(f"{decl};\n")
        for irq in inst.spec.irqs:
            controller, cells = self.interrupt(
                inst.node, irq["interrupt"], inst.node.path
            )
            _, provides = self.provide(
                controller, "interrupt-controller", inst.node.path
            )
            connect = provides.get("connect")
            if connect is None:
                raise DtError(
                    f"{inst.node.path}: interrupts of {controller.path} cannot be "
                    "connected to a handler"
                )
            line = cells[connect["irq"]]
            prio_cell = connect.get("priority")
            prio = (
                cells[prio_cell]
                if prio_cell is not None and prio_cell < len(cells)
                else 0
            )
            arg = self.value(irq["arg"], ctx)
            flags = irq.get("flags", "0")
            self.irqs.append(
                f"PBL_IRQ_CONNECT_NUM({line}, {prio}, {irq['handler']}, {arg}, {flags});\n"
            )
        inst.chunks.extend(objects)

    def sort(self):
        done, order, active = set(), [], set()

        def visit(path):
            if path in done:
                return
            if path in active:
                raise DtError(f"dependency cycle through {path}")
            active.add(path)
            for dep in sorted(self.instances[path].deps):
                visit(dep)
            active.discard(path)
            done.add(path)
            order.append(self.instances[path])

        for path in self.instances:
            visit(path)
        return order

    def run(self):
        self.collect()
        for inst in self.instances.values():
            self.emit_instance(inst)
        self.order = self.sort()

    def mains(self):
        for inst in self.order:
            for obj in inst.spec.objects:
                if obj.get("main"):
                    qual = "const " if obj.get("const", True) else ""
                    yield inst, f"extern {qual}{obj['type']} {SYMBOL_PREFIX}{inst.sym};"

    def includes(self):
        seen = []
        for inst in self.order:
            for inc in inst.spec.include:
                if inc not in seen:
                    seen.append(inc)
        return seen

    def header_text(self, source):
        out = [
            f"/* Generated by dtmap from {source}. Do not edit. */\n",
            "\n#pragma once\n\n",
        ]
        out += [f"#include <{inc}>\n" for inc in self.includes()]
        mains = list(self.mains())
        if mains:
            out.append("\n")
        for inst, decl in mains:
            out.append(f"/* {inst.node.path} */\n{decl}\n")
        return "".join(out)

    def source_text(self, source, header_name):
        out = [
            f"/* Generated by dtmap from {source}. Do not edit. */\n\n",
            f'#include "{header_name}"\n',
            '#include "pbl/kernel/irq.h"\n',
        ]
        for inst in self.order:
            if not inst.chunks:
                continue
            out.append(f"\n/* {inst.node.path} */\n")
            out += inst.chunks
        if self.irqs:
            out.append("\n")
            out += self.irqs
        return "".join(out)

    def kconfig_text(self, source):
        out = [f"# Generated by dtmap from {source}. Do not edit.\n"]
        names = sorted(
            {kconfig_name(c) for inst in self.order for c in inst.node.compatibles}
        )
        for name in names:
            out.append(f"\nconfig {name}\n\tdef_bool y\n")
        return "".join(out)

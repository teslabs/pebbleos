# Devicetree

Hardware is described in [devicetree](https://www.devicetree.org), exactly
as Linux does it: same source syntax, same bindings, same `dt-bindings`
headers, same tools. A board devicetree in PebbleOS must pass `dt-validate`
against the Linux bindings, unmodified. Nothing PebbleOS-specific goes into
the tree.

C code never reads the tree. At configure time, **dtmap** turns the compiled
blob into plain C, guided by one small spec file per driver that says how a
node becomes that driver's structs. There is no devicetree macro API.

## Files

| Path | Contents |
| --- | --- |
| `third_party/devicetree/devicetree-rebasing` | Linux bindings, `dt-bindings` headers and board sources, a submodule pinned to a `vX.Y-dts` tag |
| `dts/bindings/` | Bindings for hardware Linux does not cover, written in Linux style |
| `dts/bindings/vendor-prefixes.txt` | Vendor prefixes Linux does not have yet |
| `dts/include/dt-bindings/` | Headers for those bindings, also on the C include path |
| `dts/dtmap/` | dtmap specs for generic compatibles (`simple-bus`, the NVIC) |
| `soc/<family>/<soc>/*.dtsi` | SoC description |
| `boards/<board>/<board>.dtsi` | Board description shared by its revisions |
| `boards/<board>/<board>[_<rev>].dts` | The devicetree built for `BOARD=<board>[@<rev>]` |
| `**/*.dtmap.yaml` | dtmap specs, next to the driver they instantiate |

A board without a `.dts` builds as before.

Revisions follow the Linux pattern: each revision is a `.dts` that includes
the shared `.dtsi` and changes what differs through `&label { ... }`.

## Build

At configure time, before Kconfig, the board `.dts` is preprocessed with the
C compiler (`-nostdinc -undef -D__DTS__ -x assembler-with-cpp`, as Linux),
compiled with `dtc -@` and passed to dtmap. Everything lands in
`<build>/devicetree/`:

| File | Contents |
| --- | --- |
| `board.dtb` | The blob |
| `<board>.dts` | The merged source; every property is annotated with the file and line that set it |
| `hw.c` | Generated objects, built into the board library |
| `../generated/include/devicetree/hw.h` | `extern` declarations of the main objects |
| `Kconfig.dt` | `DT_HAS_<COMPATIBLE>` for every compatible of an enabled node |

`dtc` warnings fail the build. At build time `dt-validate` checks the blob
against the Linux bindings plus `dts/bindings/` (it needs `pip install
dtschema`); any finding, including a compatible no binding documents, fails
the build. Processing the ~5000 bindings takes about 30 s, and is only redone
when a binding changes.

Linux rejects properties with an unknown vendor prefix. Until a prefix is
accepted upstream, it is listed in `dts/bindings/vendor-prefixes.txt`, and the
schema step validates against Linux's registry plus those entries (Linux's own
files stay untouched). The build fails once Linux has an entry, so it can be
dropped.

The description is of the hardware, never of what some library wants, and
follows the Linux conventions for it: pins by name with a function string,
clocks and other provider cells as abstract IDs from a `dt-bindings` header.
Register encodings belong to the code: dtmap turns names and IDs into compact
values at build time, from tables next to the dtmap specs (e.g. the pad
function table of a pin controller), so an impossible pin and function pair
fails the build and the firmware carries no lookup tables.

## dtmap

A dtmap spec maps one or more compatibles to C objects:

```yaml
compatible: sifli,sf32lb52-i2c
include:
  - pbl/drivers/i2c/sf32lb.h
objects:
  - name: hal
    type: I2CBusHal
    const: false            # the type is already const
    init:
      pinctrl.ctrl: {pinctrl: default, part: controller}
      pinctrl.pins: {pinctrl: default}
      pinctrl.num_pins: {pinctrl: default, part: count}
      clock: {phandle: clocks, specifier: clock}
      irqn: {interrupt: 0, specifier: irqn}
  - name: state
    type: I2CBusState
    const: false            # no init: zero-initialised RAM
  - name: bus
    type: I2CBus
    const: false
    main: true
    init:
      hal: {object: hal}
      state: {object: state}
      name: {node: label}
irqs:
  - {interrupt: 0, handler: i2c_irq_handler, arg: {object: bus}}
provides:
  i2c-bus:
    object: bus
```

Each object becomes a definition in `hw.c`. The main object is named
`hw_<label>` (`hw_<node-name>_<unit-address>` without a label) and declared in
`hw.h`; the others are `static`, named `hw_<label>_<name>`. Objects are
emitted in the order listed, and nodes after the nodes they reference.

`init` maps fields, dotted for nested ones (`hdl.Init.ClockSpeed`), to
values. A value is a C expression as a string or number, a list (an array
initializer), or an expression with one source:

| Source | Value |
| --- | --- |
| `prop: <name>` | The property, read as `type`: `u32` (default), `s32`, `u64`, `bool`, `string`, `u8-array`, `u32-array`, `string-array`. A missing `bool` is `false` |
| `reg: <n>` | Address of the n-th `reg` entry, or its size with `part: size` |
| `node: name\|label\|path\|unit-address` | The node name without unit address, its first label, path or unit address |
| `object: <name>` | Address of an object of this node |
| `parent: <provides>` | Address of what the parent node provides under that name |
| `phandle: <prop>` | The `index`-th entry (default 0) of a phandle or phandle-array property: with `specifier`, what the provider's specifier makes of the cells; without, the address of what it provides. `part: count` counts the entries |
| `interrupt: <n>` | The n-th interrupt, through the interrupt parent's `interrupt-controller` specifier named `specifier` |
| `pinctrl: <state>` | An array of the pins of a `pinctrl-names` state, built by the pin controller's `pinctrl` element; `part: count` is its length, `part: controller` the address of the controller |
| `flags: {<prop>: <C>, ...}` | The C values of the boolean properties present, or'ed |
| `or: [<value>, ...]` | The values present, or'ed |
| `table: {file: <yaml>, key: [<value>, ...]}` | The entry of a YAML table next to the spec at those keys, e.g. a pad and a function |
| `cell: <n>`, `item: true` | In a specifier: the n-th cell. In a pinctrl element: the current entry of `each` |

and optional modifiers, applied in this order: `index` (pick an element of an
array property), `convert` (`mount-matrix-axis-map`, `mount-matrix-axis-dir`),
`map` (translate a value, `null` meaning absent), `lookup` (turn a number back
into its name in a `dt-bindings` header, minus `prefix`), `table` (the entry
of a YAML table next to the spec, at the value), `case` (`upper`, `lower`),
`format` (a Python format string producing C; the fields of a table entry are
passed by name) and `cast`. A value the node does not have uses `default`, is
left out with `optional: true`, and is an error otherwise.

`provides` is what other nodes can reference. Its keys are the provider kind:
`interrupt-controller`, `gpio-controller`, `clock-controller`, `pinctrl`, a
bus such as `i2c-bus`, or any name a consumer passes as `provider`. Each has
an `object` (what a reference resolves to), named `specifiers` (how the cells
of a reference turn into C: one value, or an `init` producing a struct
initializer), and, for a pin controller, the `element` built for each entry
of the `each` property (e.g. `pinmux`) of every group in a state, with an
`init` or a `value`. An
interrupt controller whose lines are CPU vectors also has `connect`, naming
the line and priority cells: `irqs` then binds handlers with
`PBL_IRQ_CONNECT_NUM()`.

Consumers resolve cells through the provider, as Linux drivers do with
`xlate`: the meaning of `<&gpio1 38 IRQ_TYPE_EDGE_RISING>` belongs to the
`gpio1` dtmap, not to whoever references it.

A spec without objects (`simple-bus`) marks a compatible as known with
nothing to instantiate. An enabled node whose compatibles have no spec is an
error; the first compatible with one wins, so fallback compatibles work as in
Linux.

dtmap specs are themselves checked against `tools/dtmap/dtmap-schema.yaml`.
The tests are in `tools/dtmap/tests` (`python -m pytest tools/dtmap/tests`).

## Adding hardware

1. Describe it in the SoC `.dtsi` or the board `.dts`, using the Linux
   binding of the part. If Linux has none, add one to `dts/bindings/` in
   Linux style, with an example.
2. If no dtmap spec covers the compatible yet, add one next to the driver.
3. Reference the generated object, `hw_<label>`, from `<devicetree/hw.h>`.

Use the merged `<build>/devicetree/<board>.dts` to see where a value comes
from.

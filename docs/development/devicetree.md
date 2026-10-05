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
| `boards/<board>/*.dtconf.yaml` | Instance settings of the board, its revisions and variants |

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
| `hw.c` | Generated instances, built into the board library |
| `../generated/include/devicetree/hw.h` | `extern` declarations of the main objects |
| `../generated/include/devicetree/types/*.h` | Generated types, for every dtmap spec |
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

A dtmap spec says what a driver captures from the nodes of its compatibles.
The data captured from devicetree is a struct the spec declares field by
field, with types: dtmap generates its type and one `const` instance per node,
in ROM. Driver state is a type the driver writes itself; dtmap only allocates
one zero-initialised instance per node, in RAM, and points the config at it.

```yaml
compatible: st,lsm6dso
include:
  - pbl/drivers/imu/lsm6dso/lsm6dso.h   # for hw.c and hw.h
types-include:
  - board/board.h                       # for the field types
config:
  type: struct pbl_lsm6dso_config
  description: ST LSM6DSO accelerometer and gyroscope.
  fields:
    i2c:
      type: I2CSlavePort
      init:
        bus: {parent: i2c-bus}
        address: {reg: 0}
    int1:
      type: ExtiConfig
      from: {interrupt: 0, specifier: exti}
    axis_map:
      type: u8
      count: 3
      from: {prop: mount-matrix, type: string-array,
             convert: mount-matrix-axis-map, default: [0, 1, 2]}
data:
  type: struct LSM6DSOState   # written by the driver
  field: state                # the config field pointing at it (default: data)
```

### Types

Each spec with a `config`, or with providers that build structs, gets a
types header, `<devicetree/types/<first compatible>.h>`:

```c
struct LSM6DSOState;

/** ST LSM6DSO accelerometer and gyroscope. */
struct pbl_lsm6dso_config {
  I2CSlavePort i2c;
  ExtiConfig int1;
  uint8_t axis_map[3];
  /** Driver state. */
  struct LSM6DSOState *state;
};
```

The header depends on the spec only, so it is generated for every spec on
every board, with or without a devicetree, and drivers include it instead of
defining the type. Field types are `u8`, `u16`, `u32`, `u64`, `s8`, `s16`,
`s32`, `s64`, `uintptr`, `bool`, `string` or any C type; `count` makes an
array. Field descriptions become doc comments. Values are checked against
their types at build time: an integer that does not fit, an array of the
wrong length or a reference of another type is an error.

### Instances

For each enabled node, `hw.c` gets the data (`static`, `hw_<label>_data`) and
the config, named `hw_<label>` (`hw_<node-name>_<unit-address>` without a
label) and declared in `hw.h`. Each node is independent: four I2C controllers
give four configs, four data instances and four IRQ bindings, and references
resolve to the instance referenced. Nodes are emitted after the nodes they
reference.

`objects` adds objects of other types, built with `init` (e.g. the common
I2C bus object pointing at the controller config); one of them can be the
`main` object instead of the config, which is then `hw_<label>_config`, and
`export: true` declares one in `hw.h` as `hw_<label>_<name>`.

A node is a hardware block, and a block can expose several interfaces: as in
Linux, there is still one config and one driver state per node, and each
interface is an object over them, which the matching provider kind resolves
to. The RCC, for one, has a clock interface and a reset interface; `clocks`
references reach the first and `resets` references the second.
`irqs` binds handlers to the node's interrupts with `PBL_IRQ_CONNECT_NUM()`,
the main object as argument unless `arg` says otherwise. With `phandle`, it
binds an interrupt of the node a reference points at instead, its index
possibly taken from the reference cells: a peripheral handling its own DMA
channel interrupt binds `{phandle: dmas, interrupt: {cell: 0}}`.

### Values

A field takes a value with `from`, or builds an existing C struct type with
`init`, which maps its fields (dotted for nested ones) to values. A value is
a C expression as a string or number, a list (an array initializer), or an
expression with one source:

| Source | Value |
| --- | --- |
| `prop: <name>` | The property, read as `type`: `u32` (default), `s32`, `u64`, `bool`, `string`, `u8-array`, `u32-array`, `string-array`. A missing `bool` is `false` |
| `reg: <n>` | Address of the n-th `reg` entry, or its size with `part: size` |
| `node: name\|label\|path\|unit-address` | The node name without unit address, its first label, path or unit address |
| `self: true`, `config: true`, `data: true` | Address of the main object, config or data of this node |
| `setting: <name>` | The instance setting, or its default |
| `object: <name>` | Address of a glue object of this node |
| `parent: <kind>` | What the parent node provides as `kind` |
| `phandle: <prop>` | What the provider of the `index`-th entry (default 0) of a phandle or phandle-array property provides, with its cells; `specifier` picks a named view, `provider` the kind when the property name does not imply it. `part: count` counts the entries |
| `interrupt: <n>` | What the interrupt parent provides for the n-th interrupt, as `interrupt-controller`; the index can be an expression, e.g. a cell |
| `pinctrl: <state>` | What the pin controller provides for a `pinctrl-names` state, as `pinctrl` |
| `flags: {<prop>: <C>, ...}` | The C values of the boolean properties present, or'ed |
| `or: [<value>, ...]` | The values present, or'ed |
| `table: {file: <yaml>, key: [<value>, ...]}` | The entry of a YAML table next to the spec at those keys, e.g. a pad and a function |
| `cell: <n>`, `item: true`, `state: pins\|count` | In a provider: the n-th cell of the reference; the current entry of a pin element; the pin array of a state and its length |

and optional modifiers, applied in this order: `index` (pick an element of an
array property), `convert` (`mount-matrix-axis-map`, `mount-matrix-axis-dir`),
`map` (translate a value, `null` meaning absent), `lookup` (turn a number back
into its name in a `dt-bindings` header, minus `prefix`), `table` (the entry
of a YAML table next to the spec, at the value), `case` (`upper`, `lower`),
`format` (a Python format string producing C; the fields of a table entry are
passed by name) and `cast`. A value the node does not have uses `default`, is
left out with `optional: true`, and is an error otherwise.

### Settings

Software choices that differ between instances, such as how many channels a
microphone captures, are settings. They do not go in the devicetree, which
only describes hardware. A spec declares them, with a type, a description,
optional `enum`, `minimum` and `maximum` constraints and a default, which may
be a C expression such as a Kconfig symbol:

```yaml
settings:
  channels:
    type: u8
    description: Channels captured, 1 (left) or 2 (stereo).
    enum: [1, 2]
    default: 1
config:
  fields:
    channels: {type: u8, from: {setting: channels}}
```

Boards set them per instance, by node label, in setting files applied in
this order, later files winning:

| File | Applies to |
| --- | --- |
| `boards/<board>/<board>.dtconf.yaml` | The board |
| `boards/<board>/<board>_<rev>.dtconf.yaml` | A revision |
| `boards/<board>/<board>_<variant>.dtconf.yaml` | A firmware variant, e.g. `prf` |
| `-DDTCONF_OVERLAY=<files>` | A build |

```yaml
pdm1:
  # The PRF mic test captures both microphones.
  channels: 2
```

An unknown label, an unknown setting, a value of the wrong type or outside
its constraints, or a setting used by a field of another type, are errors.
What every instance does the same way, or a driver API guarantees (the mic
captures 16-bit PCM at `MIC_SAMPLE_RATE`), is not a setting; neither is
policy for a whole board, which stays in Kconfig.

### Providers

`provides` is what other nodes can reference, by kind:
`interrupt-controller`, `gpio-controller`, `clock-controller`, `pinctrl`, a
bus such as `i2c-bus`, or any name a consumer passes as `provider`. A plain
kind (`i2c-bus: {}`) resolves to the address of the main object. With a
`type`, a reference resolves to a value of that type, built from the cells of
the reference: a generated struct (`fields`, declared like config fields), an
existing C type (`init`) or a `value`. `specifiers` gives named alternatives,
which consumers pick with `specifier`:

```yaml
provides:
  clock-controller:
    type: struct pbl_clock_sf32lb52
    fields:
      ctrl: {type: const struct pbl_clock_sf32lb52_ctrl *, from: {object: clk}}
      id:
        type: u16
        from: {cell: 0, lookup: {...}, table: {file: sf32lb52-gates.yaml},
               format: "PBL_CLOCK_SF32LB52_GATE({reg:#04x}, {bit})"}
  reset-controller:
    type: struct pbl_reset_sf32lb52
    fields:
      ctrl: {type: const struct pbl_reset_sf32lb52_ctrl *, from: {object: rst}}
      id: ...
```

A consumer field taking that reference declares the same type. Consumers
resolve cells through the provider, as Linux drivers do with `xlate`: the
meaning of `<&gpio1 38 IRQ_TYPE_EDGE_RISING>` belongs to the `gpio1` dtmap,
not to whoever references it.

A pin controller's `pinctrl` also has an `element`, built for each entry of
the `each` property of every group in a state, with an `init` or a `value`;
the elements of a consumer's state become a `static const` array, which the
state value points at (`state: pins`). An interrupt controller whose lines
are CPU vectors has `connect`, naming the line and priority cells.

A spec without a config (`simple-bus`) marks a compatible as known with
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
3. Use the generated type from `<devicetree/types/<compatible>.h>` in the
   driver, and the generated instance, `hw_<label>`, from `<devicetree/hw.h>`.

Use the merged `<build>/devicetree/<board>.dts` to see where a value comes
from.

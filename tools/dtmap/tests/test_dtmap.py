# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import shutil
import subprocess
import textwrap

import pytest

from tools.dtmap import fdt, spec
from tools.dtmap.__main__ import main
from tools.dtmap.generate import Generator, types_text

pytestmark = pytest.mark.skipif(shutil.which("dtc") is None, reason="needs dtc")

INTC = """
compatible: test,intc
provides:
  interrupt-controller:
    connect: {irq: 0, priority: 1}
    specifiers:
      line: {type: u16, value: {cell: 0}}
"""

CLK = """
compatible: test,clk
provides:
  clock-controller:
    type: u16
    value:
      cell: 0
      lookup: {header: test-clk.h, prefix: TEST_CLK_}
      format: "CLK_{}"
"""

PINCTRL = """
compatible: test,pinctrl
config:
  type: struct test_pinctrl
  fields:
    regs: {type: uintptr, from: {reg: 0}}
provides:
  pinctrl:
    type: struct test_pin_state
    description: Pin state.
    fields:
      ctrl: {type: const struct test_pinctrl *, from: {self: true}}
      pins: {type: const Pin *, from: {state: pins}}
      num_pins: {type: u8, from: {state: count}}
    element:
      type: Pin
      each: {prop: pins, type: string-array}
      init:
        pad: {item: true, format: "PAD_{}"}
        func: {prop: function, type: string, case: upper, format: "{}"}
        flags:
          flags: {bias-pull-up: PULL_UP}
          default: NO_PULL
"""

GPIO = """
compatible: test,gpio
config:
  type: struct test_gpio
  fields:
    regs: {type: uintptr, from: {reg: 0}}
provides:
  gpio-controller:
    specifiers:
      pin:
        type: struct test_gpio_pin
        fields:
          port: {type: const struct test_gpio *, from: {self: true}}
          pin: {type: u8, from: {cell: 0}}
          flags: {type: u8, from: {cell: 1}}
"""

BUS = """
compatible: test,bus
include: [test/bus.h]
types-include: [test/pin.h]
config:
  type: struct test_bus
  description: Test bus.
  fields:
    regs: {type: uintptr, description: Registers., from: {reg: 0}}
    speed: {type: u32, from: {prop: clock-frequency, default: 100000}}
    clk: {type: u16, from: {phandle: clocks}}
    irq: {type: u16, from: {interrupt: 0, specifier: line}}
    pinctrl: {type: struct test_pin_state, from: {pinctrl: default}}
    name: {type: string, from: {node: label}}
data: {type: struct test_bus_data}
irqs:
  - {interrupt: 0, handler: bus_isr}
provides:
  bus: {}
"""

SENSOR = """
compatible: ["test,sensor", "test,sensor2"]
include: [test/sensor.h]
config:
  type: struct test_sensor
  fields:
    bus: {type: const struct test_bus *, from: {parent: bus}}
    addr: {type: u8, from: {reg: 0}}
    reset:
      type: struct test_gpio_pin
      from: {phandle: reset-gpios, specifier: pin}
      optional: true
    axis_map:
      type: u8
      count: 3
      from:
        prop: mount-matrix
        type: string-array
        convert: mount-matrix-axis-map
        default: [0, 1, 2]
    axis_dir:
      type: s8
      count: 3
      from:
        prop: mount-matrix
        type: string-array
        convert: mount-matrix-axis-dir
        default: [1, 1, 1]
    fast: {type: bool, from: {prop: fast-mode, type: bool}}
"""

SIMPLE_BUS = "compatible: simple-bus\n"

DTS = """
/dts-v1/;

/ {
	#address-cells = <1>;
	#size-cells = <1>;
	compatible = "test,board";

	intc: interrupt-controller@e000e100 {
		compatible = "test,intc";
		reg = <0xe000e100 0xc00>;
		interrupt-controller;
		#interrupt-cells = <2>;
	};

	soc {
		compatible = "simple-bus";
		#address-cells = <1>;
		#size-cells = <1>;
		interrupt-parent = <&intc>;
		ranges;

		clk: clock-controller@1000 {
			compatible = "test,clk";
			reg = <0x1000 0x100>;
			#clock-cells = <1>;
		};

		pinctrl@2000 {
			compatible = "test,pinctrl";
			reg = <0x2000 0x100>;

			bus0_default: bus0-default-state {
				scl-pins {
					pins = "PA01";
					function = "bus0_scl";
				};
				sda-pins {
					pins = "PA02", "PA03";
					function = "bus0_sda";
					bias-pull-up;
				};
			};
		};

		gpio0: gpio@3000 {
			compatible = "test,gpio";
			reg = <0x3000 0x100>;
			gpio-controller;
			#gpio-cells = <2>;
		};

		bus0: bus@4000 {
			compatible = "test,bus";
			reg = <0x4000 0x100>;
			interrupts = <17 3>;
			clocks = <&clk 2>;
			clock-frequency = <400000>;
			pinctrl-0 = <&bus0_default>;
			pinctrl-names = "default";
			#address-cells = <1>;
			#size-cells = <0>;

			accel: sensor@6a {
				compatible = "test,sensor";
				reg = <0x6a>;
				reset-gpios = <&gpio0 5 1>;
				mount-matrix = "0", "-1", "0",
					       "1", "0", "0",
					       "0", "0", "1";
				fast-mode;
			};

			sensor@30 {
				compatible = "vendor,unknown", "test,sensor2";
				reg = <0x30>;
			};
		};

		bus@5000 {
			compatible = "test,bus";
			reg = <0x5000 0x100>;
			interrupts = <18 1>;
			clocks = <&clk 3>;
			status = "disabled";

			sensor@10 {
				compatible = "test,unmapped";
				reg = <0x10>;
			};
		};
	};
};
"""

HEADER = """
#define TEST_CLK_A 1
#define TEST_CLK_BUS0 2
#define TEST_CLK_BUS1 (3U)
"""

SPECS = {
    "intc": INTC,
    "clk": CLK,
    "pinctrl": PINCTRL,
    "gpio": GPIO,
    "bus": BUS,
    "sensor": SENSOR,
    "simple-bus": SIMPLE_BUS,
}


def compile_dts(tmp_path, source, name="test"):
    dts = tmp_path / f"{name}.dts"
    dtb = tmp_path / f"{name}.dtb"
    dts.write_text(source)
    subprocess.run(
        ["dtc", "-@", "-q", "-I", "dts", "-O", "dtb", "-o", str(dtb), str(dts)],
        check=True,
    )
    return dtb


def write_specs(tmp_path, specs):
    root = tmp_path / "dtmaps"
    root.mkdir(exist_ok=True)
    for name, text in specs.items():
        (root / f"{name}.dtmap.yaml").write_text(textwrap.dedent(text))
    (tmp_path / "test-clk.h").write_text(HEADER)
    return root


def generate(tmp_path, dts=DTS, specs=SPECS):
    dtb = compile_dts(tmp_path, dts)
    root = write_specs(tmp_path, specs)
    gen = Generator(fdt.load(dtb), spec.load(spec.find([root])), [tmp_path])
    gen.run()
    return gen


def test_config_and_data(tmp_path):
    src = generate(tmp_path).source_text("test.dtb", "hw.h")
    assert "static struct test_bus_data hw_bus0_data;" in src
    bus = src.split("const struct test_bus hw_bus0 = {")[1].split("};")[0]
    assert ".regs = 0x4000," in bus
    assert ".speed = 400000," in bus
    assert ".clk = CLK_BUS0," in bus
    assert ".irq = 17," in bus
    assert '.name = "bus0",' in bus
    assert ".data = &hw_bus0_data," in bus
    assert "PBL_IRQ_CONNECT_NUM(17, 3, bus_isr, &hw_bus0, 0);" in src


def test_types(tmp_path):
    specs = spec.load(spec.find([write_specs(tmp_path, SPECS)]))
    bus = types_text(specs["test,bus"])
    assert "#include <test/pin.h>" in bus
    assert "struct test_bus_data;" in bus
    assert "/** Test bus. */\nstruct test_bus {" in bus
    assert "  /** Registers. */\n  uintptr_t regs;" in bus
    assert "  uint16_t clk;" in bus
    assert "  struct test_pin_state pinctrl;" in bus
    assert "  const char *name;" in bus
    assert "  struct test_bus_data *data;" in bus
    pins = types_text(specs["test,pinctrl"])
    assert "struct test_pinctrl {\n  uintptr_t regs;\n};" in pins
    assert "/** Pin state. */\nstruct test_pin_state {" in pins
    assert "  const struct test_pinctrl *ctrl;" in pins
    sensor = types_text(specs["test,sensor"])
    assert "  uint8_t axis_map[3];" in sensor
    assert "  int8_t axis_dir[3];" in sensor


def test_pinctrl(tmp_path):
    src = generate(tmp_path).source_text("test.dtb", "hw.h")
    assert "static const Pin hw_bus0_pinctrl_default[] = {" in src
    pins = src.split("hw_bus0_pinctrl_default[] = {")[1].split("};")[0]
    assert pins.count(".pad =") == 3
    assert ".pad = PAD_PA01,\n    .func = BUS0_SCL,\n    .flags = NO_PULL," in pins
    assert ".pad = PAD_PA03,\n    .func = BUS0_SDA,\n    .flags = PULL_UP," in pins
    state = src.split("  .pinctrl = {")[1].split("},")[0]
    assert ".ctrl = &hw_pinctrl_2000," in state
    assert ".pins = hw_bus0_pinctrl_default," in state
    assert ".num_pins = 3," in state


PINMUX = """
compatible: test,pinctrl
config:
  type: struct test_pinctrl
  fields:
    regs: {type: uintptr, from: {reg: 0}}
provides:
  pinctrl:
    type: struct test_pin_state
    fields:
      ctrl: {type: const struct test_pinctrl *, from: {self: true}}
      pins: {type: const uint32_t *, from: {state: pins}}
      num_pins: {type: u8, from: {state: count}}
    element:
      type: uint32_t
      each: {prop: pinmux}
      value:
        or:
          - {item: true, format: "{:#x}"}
          - flags: {bias-pull-up: PU, input-enable: IE}
          - {prop: slew-rate, map: {0: SLOW, 1: null}, default: SLOW}
          - prop: drive-strength
            map: {2: 0, 4: 1}
            format: "DS({})"
            default: DS(1)
"""

PINMUX_NODE = """
			bus0_default: bus0-default-state {
				pins {
					pinmux = <0x101 0x102>;
					input-enable;
				};
				pu-pins {
					pinmux = <0x203>;
					bias-pull-up;
					slew-rate = <1>;
					drive-strength = <2>;
				};
			};
"""


def test_pinmux(tmp_path):
    start = DTS.index("\t\t\tbus0_default:")
    end = DTS.index("\n\t\t};\n", start) + 1
    dts = DTS[:start] + PINMUX_NODE.lstrip("\n") + DTS[end:]
    specs = dict(SPECS, pinctrl=PINMUX)
    src = generate(tmp_path, dts=dts, specs=specs).source_text("test.dtb", "hw.h")
    pins = src.split("static const uint32_t hw_bus0_pinctrl_default[] = {")[1].split(
        "};"
    )[0]
    assert pins.split() == [
        "0x101", "|", "IE", "|", "SLOW", "|", "DS(1),",
        "0x102", "|", "IE", "|", "SLOW", "|", "DS(1),",
        "0x203", "|", "PU", "|", "DS(0),",
    ]  # fmt: skip


def test_map_unknown_value(tmp_path):
    dts = DTS.replace(
        "clock-frequency = <400000>;", "clock-frequency = <400000>;\n\t\t\tmode = <7>;"
    )
    specs = dict(SPECS)
    specs["bus"] = BUS.replace(
        "    name: {type: string, from: {node: label}}",
        "    name: {type: string, from: {node: label}}\n"
        "    mode: {type: u8, from: {prop: mode, map: {1: A}}}",
    )
    with pytest.raises(fdt.DtError, match="7 is not one of"):
        generate(tmp_path, dts=dts, specs=specs)


def test_child(tmp_path):
    src = generate(tmp_path).source_text("test.dtb", "hw.h")
    accel = src.split("const struct test_sensor hw_accel = {")[1].split("\n};")[0]
    assert ".bus = &hw_bus0," in accel
    assert ".addr = 0x6a," in accel
    assert ".port = &hw_gpio0," in accel
    assert ".pin = 5," in accel
    assert ".axis_map = {1, 0, 2}," in accel
    assert ".axis_dir = {-1, 1, 1}," in accel
    assert ".fast = true," in accel


def test_fallback_compatible_and_defaults(tmp_path):
    src = generate(tmp_path).source_text("test.dtb", "hw.h")
    other = src.split("const struct test_sensor hw_sensor_30 = {")[1].split("\n};")[0]
    assert ".reset" not in other
    assert ".axis_map = {0, 1, 2}," in other
    assert ".fast = false," in other


def test_order_and_header(tmp_path):
    gen = generate(tmp_path)
    paths = [inst.node.path for inst in gen.order]
    assert paths.index("/soc/bus@4000") < paths.index("/soc/bus@4000/sensor@6a")
    assert paths.index("/soc/gpio@3000") < paths.index("/soc/bus@4000/sensor@6a")
    hdr = gen.header_text("test.dtb")
    assert "#include <devicetree/types/test,bus.h>" in hdr
    assert "#include <test/bus.h>" in hdr
    assert "extern const struct test_bus hw_bus0;" in hdr
    assert "extern const struct test_sensor hw_accel;" in hdr
    assert "bus@5000" not in hdr


def test_kconfig(tmp_path):
    text = generate(tmp_path).kconfig_text("test.dtb")
    assert "config DT_HAS_TEST_BUS\n\tdef_bool y" in text
    assert "DT_HAS_VENDOR_UNKNOWN" in text
    assert "DT_HAS_TEST_UNMAPPED" not in text


def test_unmapped_compatible(tmp_path):
    dts = DTS.replace('"vendor,unknown", "test,sensor2"', '"vendor,unknown"')
    with pytest.raises(fdt.DtError, match="no dtmap for compatible 'vendor,unknown'"):
        generate(tmp_path, dts=dts)


def test_missing_property(tmp_path):
    dts = DTS.replace("clocks = <&clk 2>;", "")
    with pytest.raises(fdt.DtError, match="field 'clk' of .* needs a value"):
        generate(tmp_path, dts=dts)


def test_reference_type_mismatch(tmp_path):
    specs = dict(SPECS)
    specs["sensor"] = SENSOR.replace(
        "type: struct test_gpio_pin", "type: struct other_pin"
    )
    with pytest.raises(
        fdt.DtError,
        match="is struct other_pin but the reference is struct test_gpio_pin",
    ):
        generate(tmp_path, specs=specs)


def test_value_range(tmp_path):
    dts = DTS.replace("reg = <0x6a>;", "reg = <300>;").replace(
        "accel: sensor@6a", "accel: sensor@12c"
    )
    specs = dict(SPECS)
    specs["sensor"] = SENSOR.replace(
        "addr: {type: u8, from: {reg: 0}}", "addr: {type: u8, from: {prop: reg}}"
    )
    with pytest.raises(fdt.DtError, match="300 does not fit uint8_t"):
        generate(tmp_path, dts=dts, specs=specs)


def test_array_count(tmp_path):
    specs = dict(SPECS)
    specs["sensor"] = SENSOR.replace("default: [0, 1, 2]", "default: [0, 1]")
    with pytest.raises(fdt.DtError, match="needs 3 values, got 2"):
        generate(tmp_path, specs=specs)


def test_bad_mount_matrix(tmp_path):
    dts = DTS.replace('"0", "-1", "0",', '"0", "-1", "1",')
    with pytest.raises(fdt.DtError, match="mount-matrix row 0"):
        generate(tmp_path, dts=dts)


def test_dependency_cycle(tmp_path):
    specs = dict(SPECS)
    specs["gpio"] = GPIO.replace(
        "    regs: {type: uintptr, from: {reg: 0}}",
        "    regs: {type: uintptr, from: {reg: 0}}\n"
        "    dep: {type: struct test_gpio_pin, from: {phandle: dep-gpios, specifier: pin}}",
    )
    dts = DTS.replace(
        "#gpio-cells = <2>;", "#gpio-cells = <2>;\n\t\t\tdep-gpios = <&gpio1 0 0>;"
    )
    dts = dts.replace(
        "bus0: bus@4000",
        'gpio1: gpio@3100 {\n\t\t\tcompatible = "test,gpio";\n\t\t\treg = <0x3100 0x100>;\n'
        "\t\t\tgpio-controller;\n\t\t\t#gpio-cells = <2>;\n"
        "\t\t\tdep-gpios = <&gpio0 0 0>;\n\t\t};\n\n\t\tbus0: bus@4000",
    )
    with pytest.raises(fdt.DtError, match="dependency cycle"):
        generate(tmp_path, dts=dts, specs=specs)


def test_bad_spec(tmp_path):
    specs = dict(SPECS)
    specs["bus"] = BUS.replace(
        "data: {type: struct test_bus_data}",
        "data: {type: struct test_bus_data, bogus: 1}",
    )
    with pytest.raises(fdt.DtError, match="bogus"):
        generate(tmp_path, specs=specs)


def test_bad_generated_type(tmp_path):
    specs = dict(SPECS)
    specs["bus"] = BUS.replace("type: struct test_bus\n", "type: test_bus\n")
    with pytest.raises(fdt.DtError):
        generate(tmp_path, specs=specs)


def test_duplicate_compatible(tmp_path):
    specs = dict(SPECS)
    specs["bus2"] = BUS
    with pytest.raises(fdt.DtError, match="already mapped"):
        generate(tmp_path, specs=specs)


def test_cli(tmp_path):
    dtb = compile_dts(tmp_path, DTS)
    root = write_specs(tmp_path, SPECS)
    out = tmp_path / "out"
    args = [
        "--dtb", str(dtb), "--dtmap-root", str(root), "-I", str(tmp_path),
        "--types-dir", str(out), "--header", str(out / "hw.h"),
        "--source", str(out / "hw.c"), "--kconfig", str(out / "Kconfig.dt"),
        "--depfile", str(out / "dtmaps.txt"),
    ]  # fmt: skip
    assert main(args) == 0
    assert '#include "hw.h"' in (out / "hw.c").read_text()
    assert "struct test_bus {" in (out / "devicetree/types/test,bus.h").read_text()
    assert "bus.dtmap.yaml" in (out / "dtmaps.txt").read_text()
    bad = compile_dts(tmp_path, DTS.replace('"test,sensor2"', '"nope"'), "bad")
    args[1] = str(bad)
    assert main(args) == 1


TABLE_PINCTRL = PINCTRL.replace(
    """      type: Pin
      each: {prop: pins, type: string-array}
      init:
        pad: {item: true, format: "PAD_{}"}
        func: {prop: function, type: string, case: upper, format: "{}"}
        flags:
          flags: {bias-pull-up: PULL_UP}
          default: NO_PULL
""",
    """      type: uint32_t
      each: {prop: pins, type: string-array}
      value:
        table:
          file: pins.yaml
          key: [{item: true}, {prop: function, type: string}]
        format: "PIN({pad}, {fsel})"
""",
).replace("const Pin *", "const uint32_t *")

TABLE_CLK = """
compatible: test,clk
provides:
  clock-controller:
    type: u16
    value:
      cell: 0
      lookup: {header: test-clk.h, prefix: TEST_CLK_}
      table: {file: clocks.yaml}
      format: "GATE({reg:#x}, {bit})"
"""

PINS_TABLE = """
PA01: {bus0_scl: {pad: 1, fsel: 4}}
PA02: {bus0_sda: {pad: 2, fsel: 4}}
PA03: {bus0_sda: {pad: 3, fsel: 5}}
"""


def table_specs(tmp_path):
    (tmp_path / "dtmaps").mkdir(exist_ok=True)
    (tmp_path / "dtmaps" / "pins.yaml").write_text(PINS_TABLE)
    (tmp_path / "dtmaps" / "clocks.yaml").write_text("BUS0: {reg: 8, bit: 27}\n")
    return dict(SPECS, pinctrl=TABLE_PINCTRL, clk=TABLE_CLK)


def test_tables(tmp_path):
    src = generate(tmp_path, specs=table_specs(tmp_path)).source_text("t.dtb", "hw.h")
    pins = src.split("hw_bus0_pinctrl_default[] = {")[1].split("};")[0]
    assert pins.split() == ["PIN(1,", "4),", "PIN(2,", "4),", "PIN(3,", "5),"]
    assert ".clk = GATE(0x8, 27)," in src


def test_table_unknown_entry(tmp_path):
    dts = DTS.replace('function = "bus0_scl";', 'function = "bus0_sda";')
    with pytest.raises(fdt.DtError, match="no PA01 / bus0_sda in pins.yaml"):
        generate(tmp_path, dts=dts, specs=table_specs(tmp_path))


MULTI = """
compatible: test,clk
include: [test/clk.h]
config:
  type: struct test_rcc
  fields:
    base: {type: uintptr, from: {reg: 0}}
objects:
  - name: clk
    type: struct test_clk_ctrl
    export: true
    init: {rcc: {self: true}}
  - name: rst
    type: struct test_rst_ctrl
    export: true
    init: {rcc: {self: true}}
provides:
  clock-controller:
    type: struct test_clk
    fields:
      ctrl: {type: const struct test_clk_ctrl *, from: {object: clk}}
      id: {type: u16, from: {cell: 0}}
  reset-controller:
    type: struct test_rst
    fields:
      ctrl: {type: const struct test_rst_ctrl *, from: {object: rst}}
      id: {type: u16, from: {cell: 0}}
"""


def multi(tmp_path, clk=MULTI):
    dts = DTS.replace(
        "#clock-cells = <1>;", "#clock-cells = <1>;\n\t\t\t#reset-cells = <1>;"
    )
    dts = dts.replace(
        "clocks = <&clk 2>;", "clocks = <&clk 2>;\n\t\t\tresets = <&clk 5>;"
    )
    bus = BUS.replace(
        "    clk: {type: u16, from: {phandle: clocks}}",
        "    clk: {type: struct test_clk, from: {phandle: clocks}}\n"
        "    rst: {type: struct test_rst, from: {phandle: resets}}",
    )
    return generate(tmp_path, dts=dts, specs=dict(SPECS, clk=clk, bus=bus))


def test_interfaces(tmp_path):
    gen = multi(tmp_path)
    src = gen.source_text("test.dtb", "hw.h")
    assert "const struct test_rcc hw_clk = {" in src
    assert "const struct test_clk_ctrl hw_clk_clk = {\n  .rcc = &hw_clk,\n};" in src
    assert "const struct test_rst_ctrl hw_clk_rst = {" in src
    bus = src.split("const struct test_bus hw_bus0 = {")[1].split("\n};")[0]
    assert ".clk = {\n    .ctrl = &hw_clk_clk,\n    .id = 2,\n  }," in bus
    assert ".rst = {\n    .ctrl = &hw_clk_rst,\n    .id = 5,\n  }," in bus
    hdr = gen.header_text("test.dtb")
    assert "extern const struct test_rcc hw_clk;" in hdr
    assert "extern const struct test_clk_ctrl hw_clk_clk;" in hdr
    assert "extern const struct test_rst_ctrl hw_clk_rst;" in hdr


def test_object_reference_type(tmp_path):
    bad = MULTI.replace("from: {object: rst}", "from: {object: clk}")
    with pytest.raises(
        fdt.DtError, match=r"is const struct test_rst_ctrl \* but the reference"
    ):
        multi(tmp_path, clk=bad)


SETTINGS_SENSOR = SENSOR.replace(
    "config:",
    """settings:
  rate:
    type: u16
    description: Output data rate.
    enum: [26, 52, 104]
    default: 52
  label:
    type: string
    description: Name.
config:""",
).replace(
    "    fast: {type: bool, from: {prop: fast-mode, type: bool}}",
    "    fast: {type: bool, from: {prop: fast-mode, type: bool}}\n"
    "    rate: {type: u16, from: {setting: rate}}\n"
    "    label: {type: string, from: {setting: label}, optional: true}",
)


def settings_gen(tmp_path, *confs):
    paths = []
    for i, text in enumerate(confs):
        path = tmp_path / f"conf{i}.dtconf.yaml"
        path.write_text(textwrap.dedent(text))
        paths.append(path)
    dtb = compile_dts(tmp_path, DTS)
    root = write_specs(tmp_path, dict(SPECS, sensor=SETTINGS_SENSOR))
    gen = Generator(
        fdt.load(dtb),
        spec.load(spec.find([root])),
        [tmp_path],
        spec.load_settings(paths),
    )
    gen.run()
    return gen.source_text("test.dtb", "hw.h")


def test_settings_defaults(tmp_path):
    src = settings_gen(tmp_path)
    accel = src.split("hw_accel = {")[1].split("\n};")[0]
    assert ".rate = 52," in accel
    assert ".label" not in accel


def test_settings_layered(tmp_path):
    src = settings_gen(
        tmp_path,
        "accel: {rate: 26, label: front}\n",
        "accel: {rate: 104}\n",
    )
    accel = src.split("hw_accel = {")[1].split("\n};")[0]
    assert ".rate = 104," in accel
    assert '.label = "front",' in accel
    other = src.split("hw_sensor_30 = {")[1].split("\n};")[0]
    assert ".rate = 52," in other


def test_settings_unknown_label(tmp_path):
    with pytest.raises(fdt.DtError, match="no enabled node is labelled 'gyro'"):
        settings_gen(tmp_path, "gyro: {rate: 26}\n")


def test_settings_unknown_setting(tmp_path):
    with pytest.raises(
        fdt.DtError, match="has no such setting .settings: rate, label."
    ):
        settings_gen(tmp_path, "accel: {speed: 26}\n")


def test_settings_enum(tmp_path):
    with pytest.raises(fdt.DtError, match="25 is not one of"):
        settings_gen(tmp_path, "accel: {rate: 25}\n")


def test_settings_type(tmp_path):
    with pytest.raises(fdt.DtError, match="70000 is not a valid uint16_t"):
        settings_gen(tmp_path, "accel: {rate: 70000}\n")
    with pytest.raises(fdt.DtError, match="'fast' is not a valid uint16_t"):
        settings_gen(tmp_path, "accel: {rate: fast}\n")


def test_settings_field_type(tmp_path):
    bad = SETTINGS_SENSOR.replace(
        "rate: {type: u16, from: {setting: rate}}",
        "rate: {type: u8, from: {setting: rate}}",
    )
    dtb = compile_dts(tmp_path, DTS)
    root = write_specs(tmp_path, dict(SPECS, sensor=bad))
    with pytest.raises(fdt.DtError, match="is uint8_t but the reference is uint16_t"):
        Generator(fdt.load(dtb), spec.load(spec.find([root])), [tmp_path]).run()


DMA = """
compatible: test,dma
provides:
  dma-controller:
    type: struct test_dma
    fields:
      channel: {type: u8, from: {cell: 0}}
      request: {type: u8, from: {cell: 1}}
      irq: {type: u16, from: {interrupt: {cell: 0}, specifier: line}}
"""


def test_referenced_interrupt(tmp_path):
    dts = DTS.replace(
        "\t\tgpio0: gpio@3000 {",
        '\t\tdma0: dma-controller@6000 {\n\t\t\tcompatible = "test,dma";\n'
        "\t\t\treg = <0x6000 0x100>;\n\t\t\tinterrupts = <20 2>, <21 4>;\n"
        "\t\t\t#dma-cells = <2>;\n\t\t};\n\n\t\tgpio0: gpio@3000 {",
    ).replace("clocks = <&clk 2>;", "clocks = <&clk 2>;\n\t\t\tdmas = <&dma0 1 7>;")
    bus = BUS.replace(
        "    name: {type: string, from: {node: label}}",
        "    name: {type: string, from: {node: label}}\n"
        "    dma: {type: struct test_dma, from: {phandle: dmas}}",
    ).replace(
        "  - {interrupt: 0, handler: bus_isr}",
        "  - {interrupt: 0, handler: bus_isr}\n"
        "  - {phandle: dmas, interrupt: {cell: 0}, handler: bus_dma_isr}",
    )
    src = generate(tmp_path, dts=dts, specs=dict(SPECS, bus=bus, dma=DMA)).source_text(
        "t.dtb", "hw.h"
    )
    assert ".dma = {\n    .channel = 1,\n    .request = 7,\n    .irq = 21,\n  }," in src
    assert "PBL_IRQ_CONNECT_NUM(21, 4, bus_dma_isr, &hw_bus0, 0);" in src

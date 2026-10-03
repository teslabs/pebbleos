# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import shutil
import subprocess
import textwrap

import pytest

from tools.dtmap import fdt, spec
from tools.dtmap.__main__ import main
from tools.dtmap.generate import Generator

pytestmark = pytest.mark.skipif(shutil.which("dtc") is None, reason="needs dtc")

INTC = """
compatible: test,intc
provides:
  interrupt-controller:
    connect: {irq: 0, priority: 1}
    specifiers:
      irqn: {cell: 0, cast: IRQn_Type}
"""

CLK = """
compatible: test,clk
provides:
  clock-controller:
    specifiers:
      id:
        cell: 0
        lookup: {header: test-clk.h, prefix: TEST_CLK_}
        format: "CLK_{}"
"""

PINCTRL = """
compatible: test,pinctrl
objects:
  - name: ctrl
    type: PinCtrl
    main: true
    init:
      regs: {reg: 0}
provides:
  pinctrl:
    object: ctrl
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
objects:
  - name: port
    type: GpioPort
    main: true
    init:
      regs: {reg: 0, cast: "void *"}
provides:
  gpio-controller:
    object: port
    specifiers:
      pin:
        init:
          port: {reg: 0, cast: "void *"}
          pin: {cell: 0}
          flags: {cell: 1}
"""

BUS = """
compatible: test,bus
include: [test/bus.h]
objects:
  - name: state
    type: BusState
    const: false
  - name: bus
    type: Bus
    main: true
    init:
      state: {object: state}
      regs: {reg: 0, cast: "BusRegs *"}
      speed: {prop: clock-frequency, default: 100000}
      clk: {phandle: clocks, specifier: id}
      irqn: {interrupt: 0, specifier: irqn}
      pinctrl: {pinctrl: default, part: controller}
      pins: {pinctrl: default}
      num_pins: {pinctrl: default, part: count}
      name: {node: label}
irqs:
  - {interrupt: 0, handler: bus_isr, arg: {object: bus}}
provides:
  bus:
    object: bus
"""

SENSOR = """
compatible: ["test,sensor", "test,sensor2"]
include: [test/sensor.h]
objects:
  - name: cfg
    type: SensorConfig
    main: true
    init:
      bus: {parent: bus}
      addr: {reg: 0}
      reset: {phandle: reset-gpios, specifier: pin, optional: true}
      axis_map:
        prop: mount-matrix
        type: string-array
        convert: mount-matrix-axis-map
        default: [0, 1, 2]
      axis_dir:
        prop: mount-matrix
        type: string-array
        convert: mount-matrix-axis-dir
        default: [1, 1, 1]
      fast: {prop: fast-mode, type: bool}
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


def test_bus_objects(tmp_path):
    src = generate(tmp_path).source_text("test.dtb", "hw.h")
    assert "static BusState hw_bus0_state;" in src
    assert "const Bus hw_bus0 = {" in src
    assert ".regs = (BusRegs *)0x4000," in src
    assert ".speed = 400000," in src
    assert ".clk = CLK_BUS0," in src
    assert ".irqn = (IRQn_Type)17," in src
    assert '.name = "bus0",' in src
    assert "PBL_IRQ_CONNECT_NUM(17, 3, bus_isr, &hw_bus0, 0);" in src


def test_pinctrl(tmp_path):
    src = generate(tmp_path).source_text("test.dtb", "hw.h")
    assert "static const Pin hw_bus0_pinctrl_default[] = {" in src
    pins = src.split("hw_bus0_pinctrl_default[] = {")[1].split("};")[0]
    assert pins.count(".pad =") == 3
    assert ".pad = PAD_PA01,\n    .func = BUS0_SCL,\n    .flags = NO_PULL," in pins
    assert ".pad = PAD_PA03,\n    .func = BUS0_SDA,\n    .flags = PULL_UP," in pins
    assert ".pinctrl = &hw_pinctrl_2000," in src
    assert ".pins = hw_bus0_pinctrl_default," in src
    assert ".num_pins = 3," in src


PINMUX = """
compatible: test,pinctrl
objects:
  - name: ctrl
    type: PinCtrl
    main: true
provides:
  pinctrl:
    object: ctrl
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
        "      name: {node: label}",
        "      name: {node: label}\n      mode: {prop: mode, map: {1: A}}",
    )
    with pytest.raises(fdt.DtError, match="7 is not one of"):
        generate(tmp_path, dts=dts, specs=specs)


def test_child(tmp_path):
    src = generate(tmp_path).source_text("test.dtb", "hw.h")
    accel = src.split("const SensorConfig hw_accel = {")[1].split("};")[0]
    assert ".bus = &hw_bus0," in accel
    assert ".addr = 0x6a," in accel
    assert ".port = (void *)0x3000," in accel
    assert ".pin = 5," in accel
    assert ".axis_map = {1, 0, 2}," in accel
    assert ".axis_dir = {-1, 1, 1}," in accel
    assert ".fast = true," in accel


def test_fallback_compatible_and_defaults(tmp_path):
    src = generate(tmp_path).source_text("test.dtb", "hw.h")
    other = src.split("const SensorConfig hw_sensor_30 = {")[1].split("};")[0]
    assert ".reset" not in other
    assert ".axis_map = {0, 1, 2}," in other
    assert ".fast = false," in other


def test_order_and_header(tmp_path):
    gen = generate(tmp_path)
    paths = [inst.node.path for inst in gen.order]
    assert paths.index("/soc/bus@4000") < paths.index("/soc/bus@4000/sensor@6a")
    assert paths.index("/soc/gpio@3000") < paths.index("/soc/bus@4000/sensor@6a")
    hdr = gen.header_text("test.dtb")
    assert "#include <test/bus.h>" in hdr
    assert "extern const Bus hw_bus0;" in hdr
    assert "extern const SensorConfig hw_accel;" in hdr
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
    with pytest.raises(fdt.DtError, match="'clk' of .* needs a value"):
        generate(tmp_path, dts=dts)


def test_bad_mount_matrix(tmp_path):
    dts = DTS.replace('"0", "-1", "0",', '"0", "-1", "1",')
    with pytest.raises(fdt.DtError, match="mount-matrix row 0"):
        generate(tmp_path, dts=dts)


def test_dependency_cycle(tmp_path):
    specs = dict(SPECS)
    specs["gpio"] = GPIO.replace(
        'regs: {reg: 0, cast: "void *"}',
        'regs: {reg: 0, cast: "void *"}\n      dep: {phandle: dep-gpios, specifier: pin}',
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
    specs["bus"] = BUS.replace("main: true", "main: true\n    bogus: 1")
    with pytest.raises(fdt.DtError, match="bogus"):
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
        "--header", str(out / "hw.h"), "--source", str(out / "hw.c"),
        "--kconfig", str(out / "Kconfig.dt"), "--depfile", str(out / "dtmaps.txt"),
    ]  # fmt: skip
    assert main(args) == 0
    assert '#include "hw.h"' in (out / "hw.c").read_text()
    assert "bus.dtmap.yaml" in (out / "dtmaps.txt").read_text()
    bad = compile_dts(tmp_path, DTS.replace('"test,sensor2"', '"nope"'), "bad")
    args[1] = str(bad)
    assert main(args) == 1


TABLE_PINCTRL = """
compatible: test,pinctrl
objects:
  - name: ctrl
    type: PinCtrl
    main: true
provides:
  pinctrl:
    object: ctrl
    element:
      type: uint32_t
      each: {prop: pins, type: string-array}
      value:
        table:
          file: pins.yaml
          key: [{item: true}, {prop: function, type: string}]
        format: "PIN({pad}, {fsel})"
"""

TABLE_CLK = """
compatible: test,clk
provides:
  clock-controller:
    specifiers:
      id:
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

# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import dtmap
import fdt
from fdt import DtError

pytestmark = pytest.mark.skipif(shutil.which("dtc") is None, reason="needs dtc")

SPECS = {
    "acme,ctrl": {
        "compatible": "acme,ctrl",
        "config": "CTRL",
        "include": "ctrl.h",
        "provides": {"clock": "CLK({0})", "irq": "{0}", "irq-gpio": "PIN({0})"},
        "pinctrl": {
            "controller": "PINCTRL",
            "pin": "{pin}_{function}",
            "flags": {"input-enable": "INPUT"},
            "values": {"drive-strength": "DRIVE_{value}"},
        },
    },
    "acme,bus": {
        "compatible": "acme,bus",
        "config": "BUS",
        "include": "bus.h",
        "type": "struct bus",
        "class": {
            "type": "struct bus_class",
            "member": "bus",
            "include": "bus_class.h",
        },
        "define": "BUS_DEFINE",
        "args": [
            {"name": True},
            {"reg": 0},
            {"clock": 0},
            {"prop": "clock-frequency", "default": 100},
            {"pinctrl": "default", "default": "nullptr"},
        ],
        "irqs": [{"irq": 0, "prio": 5, "isr": "bus_isr"}],
        "provides": {"bus": "&{sym}"},
    },
    "acme,sensor": {
        "compatible": "acme,sensor",
        "config": "SENSOR",
        "include": "sensor.h",
        "type": "struct sensor",
        "class": {
            "type": "struct sensor_class",
            "member": "sensor",
            "include": "sensor_class.h",
        },
        "define": "SENSOR_DEFINE",
        "args": [
            {"parent": "bus"},
            {"reg": 0},
            {"irq": 0, "as": "irq-gpio", "field": "pad"},
            {
                "strings": "mount-matrix",
                "default": [1, 0],
                "wrap": "MAP",
                "field": "axes",
            },
        ],
        "export": "SENSOR",
    },
}

DTS = """
/dts-v1/;
/ {
    #address-cells = <1>;
    #size-cells = <1>;
    ctrl: ctrl@1000 {
        compatible = "acme,ctrl";
        reg = <0x1000 0x100>;
        #clock-cells = <1>;
        #interrupt-cells = <1>;
        interrupt-controller;
        bus_default: state {
            scl {
                pins = "PA1";
                function = "scl";
                input-enable;
                drive-strength = <8>;
            };
        };
    };
    bus: bus@2000 {
        compatible = "acme,bus";
        reg = <0x2000 0x100>;
        interrupt-parent = <&ctrl>;
        interrupts = <7>;
        clocks = <&ctrl 3>;
        pinctrl-0 = <&bus_default>;
        pinctrl-names = "default";
        clock-frequency = <400>;
        %(bus)s
        #address-cells = <1>;
        #size-cells = <0>;
        sensor: sensor@30 {
            compatible = "acme,sensor";
            reg = <0x30>;
            interrupt-parent = <&ctrl>;
            interrupts = <9>;
            mount-matrix = "-1", "0";
            %(sensor)s
        };
    };
};
"""


def generate(
    tmp_path, config=("CTRL", "BUS", "SENSOR"), bus="", sensor="", specs=None, dts=DTS
):
    path = tmp_path / "t.dts"
    path.write_text(dts % {"bus": bus, "sensor": sensor})
    dtb = tmp_path / "t.dtb"
    subprocess.run(
        ["dtc", "-@", "-q", "-I", "dts", "-O", "dtb", "-o", str(dtb), str(path)],
        check=True,
    )
    loaded = {k: dtmap.Spec(k, v) for k, v in (specs or SPECS).items()}
    gen = dtmap.Generator(fdt.load(dtb), loaded, set(config))
    gen.run()
    return gen


def body(gen):
    return "\n".join(line for out in gen.outputs.values() for line in out.body)


def test_instances_go_to_their_driver(tmp_path):
    gen = generate(tmp_path)
    bus = gen.instances()["devicetree/instances/acme,bus.inc"]
    sensor = gen.instances()["devicetree/instances/acme,sensor.inc"]
    assert (
        'BUS_DEFINE(dt_bus_inst, "bus", 0x2000, CLK(3), 400, &dt_bus_inst_pinctrl_default);'
        in bus
    )
    assert (
        "PBL_PINCTRL_STATE_DEFINE(dt_bus_inst_pinctrl_default, PINCTRL, PA1_SCL | INPUT | DRIVE_8);"
        in bus
    )
    assert bus.index("STATE_DEFINE") < bus.index("BUS_DEFINE")
    assert "PBL_IRQ_CONNECT_NUM(7, 5, bus_isr, &dt_bus_inst, 0);" in bus
    assert "#include <ctrl.h>" in bus
    assert "SENSOR_DEFINE" not in bus
    assert (
        "SENSOR_DEFINE(dt_sensor_inst, &dt_bus, 0x30, .pad = PIN(9), .axes = MAP(-1, 0));"
        in sensor
    )
    assert "#include <devicetree/devices.h>" in sensor
    assert "const struct sensor_class *const SENSOR = &dt_sensor_inst.sensor;" in sensor


def test_class_alias(tmp_path):
    bus = generate(tmp_path).instances()["devicetree/instances/acme,bus.inc"]
    assert "static_assert(offsetof(struct bus, bus) == 0" in bus
    assert (
        'extern const struct bus_class dt_bus __attribute__((alias("dt_bus_inst")));'
        in bus
    )


def test_header_only_needs_class_types(tmp_path):
    header = generate(tmp_path).header()
    assert "extern const struct bus_class dt_bus;" in header
    assert "#include <bus_class.h>" in header
    assert "#include <bus.h>" not in header


def test_unlabelled_node_has_no_public_symbol(tmp_path):
    gen = generate(tmp_path, dts=DTS.replace("sensor: sensor@30", "sensor@30"))
    assert "dt_sensor" not in gen.header()
    assert "dt_node_bus_2000_sensor_30" in body(gen)


def test_unlabelled_provider_fails(tmp_path):
    with pytest.raises(DtError, match="needs a label"):
        generate(tmp_path, dts=DTS.replace("bus: bus@2000", "bus@2000"))


def test_kconfig_off_drops_the_device(tmp_path):
    gen = generate(tmp_path, config=("CTRL", "BUS"))
    assert "SENSOR" not in body(gen)
    assert "/bus@2000/sensor@30" in gen.unbound


def test_status_disabled_drops_the_device(tmp_path):
    gen = generate(tmp_path, sensor='status = "disabled";')
    assert "SENSOR" not in body(gen)
    assert not gen.unbound


def test_disabled_bus_drops_its_children(tmp_path):
    gen = generate(tmp_path, bus='status = "disabled";')
    assert "DEFINE" not in body(gen)


def test_missing_provider_driver_fails(tmp_path):
    with pytest.raises(DtError, match="no enabled driver"):
        generate(tmp_path, config=("BUS", "SENSOR"))


def test_provider_lacking_the_kind_fails(tmp_path):
    specs = {k: dict(v) for k, v in SPECS.items()}
    specs["acme,sensor"]["args"] = [{"irq": 0}]
    specs["acme,ctrl"]["provides"] = {}
    with pytest.raises(DtError, match="provides no 'clock'"):
        generate(tmp_path, specs=specs)


def test_default_used_when_absent(tmp_path):
    specs = {k: dict(v) for k, v in SPECS.items()}
    specs["acme,bus"]["args"] = [{"prop": "absent", "default": 100}]
    assert "BUS_DEFINE(dt_bus_inst, 100);" in body(generate(tmp_path, specs=specs))


def test_required_arg_missing_fails(tmp_path):
    specs = {k: dict(v) for k, v in SPECS.items()}
    specs["acme,bus"]["args"] = [{"prop": "absent"}]
    with pytest.raises(DtError, match="missing"):
        generate(tmp_path, specs=specs)


@pytest.mark.parametrize(
    "data",
    [
        {"compatible": "a,b"},
        {"compatible": "a,b", "config": "X", "typo": 1},
        {"compatible": "a,b", "config": "X", "define": "D"},
        {
            "compatible": "a,b",
            "config": "X",
            "type": "t",
            "class": {"type": "c", "member": "m", "include": "i.h"},
            "define": "D",
            "args": [{"reg": 0, "prop": "p"}],
        },
        {
            "compatible": "a,b",
            "config": "X",
            "type": "t",
            "class": {"type": "c", "member": "m", "include": "i.h"},
            "define": "D",
            "args": [{"bogus": 1}],
        },
        {"compatible": "a,b", "config": "X", "irqs": [{"irq": 0}]},
    ],
)
def test_bad_specs_are_rejected(data):
    with pytest.raises(DtError):
        dtmap.Spec("spec", data)


def test_kconfig_symbols(tmp_path):
    gen = generate(tmp_path, sensor='status = "disabled";')
    text = dtmap.kconfig_text(gen.tree, ["acme,bus", "acme,sensor", "acme,absent"])
    assert "config DT_HAS_ACME_BUS_ENABLED\n\tdef_bool y" in text
    assert "config DT_HAS_ACME_SENSOR_ENABLED\n\tdef_bool n" in text
    assert "config DT_HAS_ACME_ABSENT_ENABLED\n\tdef_bool n" in text
    assert "def_bool n" in dtmap.kconfig_text(None, ["acme,bus"])


def test_real_specs_load():
    root = Path(__file__).resolve().parents[3]
    specs = dtmap.load_specs(dtmap.find_specs(root))
    assert "sifli,sf32lb52-i2c" in specs

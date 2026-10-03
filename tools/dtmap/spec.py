# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""dtmap spec files: how the nodes of a compatible become C objects."""

import os

import jsonschema
import yaml

from .fdt import DtError

SCHEMA_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "dtmap-schema.yaml"
)

# Specifier cell count property of each phandle-array property family.
CELLS_PROPS = {
    "clocks": "#clock-cells",
    "resets": "#reset-cells",
    "dmas": "#dma-cells",
    "gpios": "#gpio-cells",
    "pwms": "#pwm-cells",
    "io-channels": "#io-channel-cells",
    "interrupts-extended": "#interrupt-cells",
    "mboxes": "#mbox-cells",
    "phys": "#phy-cells",
    "power-domains": "#power-domain-cells",
}

# The 'provides' key a phandle-array property family resolves through.
PROVIDER_OF = {
    "clocks": "clock-controller",
    "resets": "reset-controller",
    "dmas": "dma-controller",
    "gpios": "gpio-controller",
    "pwms": "pwm-controller",
    "io-channels": "io-channel-controller",
    "interrupts-extended": "interrupt-controller",
    "mboxes": "mbox-controller",
    "phys": "phy-provider",
    "power-domains": "power-domain-controller",
}


class Spec:
    def __init__(self, path, data):
        self.path = path
        compatible = data["compatible"]
        self.compatibles = [compatible] if isinstance(compatible, str) else compatible
        self.include = data.get("include", [])
        self.objects = data.get("objects", [])
        self.irqs = data.get("irqs", [])
        self.provides = data.get("provides", {})
        mains = [o["name"] for o in self.objects if o.get("main")]
        if len(mains) > 1:
            raise DtError(f"{path}: more than one main object: {', '.join(mains)}")
        names = [o["name"] for o in self.objects]
        if len(set(names)) != len(names):
            raise DtError(f"{path}: object names must be unique")


def _validator():
    with open(SCHEMA_PATH) as f:
        schema = yaml.safe_load(f)
    return jsonschema.Draft202012Validator(schema)


def load(paths):
    validator = _validator()
    specs = {}
    for path in sorted(paths):
        with open(path) as f:
            data = yaml.safe_load(f)
        errors = sorted(validator.iter_errors(data), key=lambda e: list(e.path))
        if errors:
            err = errors[0]
            where = "/".join(str(p) for p in err.path) or "(top level)"
            raise DtError(f"{path}: {where}: {err.message}")
        spec = Spec(path, data)
        for compatible in spec.compatibles:
            if compatible in specs:
                raise DtError(
                    f"{path}: '{compatible}' is already mapped by "
                    f"{specs[compatible].path}"
                )
            specs[compatible] = spec
    return specs


def find(roots):
    paths = []
    for root in roots:
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames if not d.startswith((".", "build"))]
            paths += [
                os.path.join(dirpath, f) for f in filenames if f.endswith(".dtmap.yaml")
            ]
    return paths

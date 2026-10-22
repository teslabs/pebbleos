#!/usr/bin/env python3
"""Compare the resource sets of the old tree (resource maps) and the new
tree (YAML) for every board and variant.

usage: verify_resources.py <old root> <new root> <config dir map...>

A config is looked up as <old root>/<build dir>/.config per board; a
board without one is loaded with no Kconfig symbols enabled.
"""

import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable

BOARDS = {
    "obelix": "emery", "qemu_emery": "emery",
    "getafix": "gabbro", "qemu_gabbro": "gabbro",
    "asterix": "flint", "qemu_flint": "flint",
}
CONFIGS = {
    ("obelix", "normal"): "build-obelix",
    ("qemu_emery", "normal"): "build-qemu-emery",
    ("getafix", "normal"): "build-cmp-getafix-dvt-normal",
    ("qemu_gabbro", "normal"): "build-cmp-qemu_gabbro-normal",
    ("asterix", "normal"): "build-asterix",
    ("qemu_flint", "normal"): "build-cmp-qemu_flint-normal",
    ("obelix", "prf"): "build-cmp-obelix-pvt-prf",
}


def dump(root, mode, platform, board, variant, config, out):
    subprocess.run(
        [PY, os.path.join(HERE, "dump_defs.py"), root, mode, platform, board,
         variant, config, out],
        check=True, stdout=subprocess.DEVNULL,
    )
    with open(out) as f:
        return json.load(f)


def main():
    old_root, new_root = sys.argv[1:3]
    outdir = os.path.join(HERE, "verify")
    os.makedirs(outdir, exist_ok=True)
    total = 0
    for variant in ("normal", "prf"):
        for board, platform in BOARDS.items():
            cfg = CONFIGS.get((board, variant))
            cfg = os.path.join(old_root, cfg, ".config") if cfg else "-"
            if cfg != "-" and not os.path.exists(cfg):
                cfg = "-"
            old = dump(old_root, "old", platform, board, variant, cfg,
                       os.path.join(outdir, f"{board}-{variant}-old.json"))
            new = dump(new_root, "new", platform, board, variant, cfg,
                       os.path.join(outdir, f"{board}-{variant}-new.json"))
            diffs = []
            om, nm = old["media"], new["media"]
            for name in sorted(set(om) | set(nm)):
                if name not in nm:
                    diffs.append(f"- {name} ({om[name]['type']}, {om[name]['source']})")
                elif name not in om:
                    diffs.append(f"+ {name} ({nm[name]['type']}, {nm[name]['source']})")
                else:
                    a, b = dict(om[name]), dict(nm[name])
                    a.pop("source"), b.pop("source")
                    if a != b:
                        changed = {k: (a.get(k), b.get(k)) for k in set(a) | set(b) if a.get(k) != b.get(k)}
                        diffs.append(f"~ {name}: {changed}")
            if old["timeline"] != new["timeline"]:
                diffs.append("~ timeline table differs")
            if old["files"] != new["files"]:
                diffs.append("~ files table differs")
            same = len(set(om) & set(nm)) - sum(1 for d in diffs if d.startswith("~ ") and ":" in d)
            cfgname = os.path.relpath(cfg, old_root) if cfg != "-" else "no .config"
            print(f"{board} {variant} ({platform}, {cfgname}): old {len(om)} new {len(nm)}, "
                  f"{same} identical, {len(diffs)} differences")
            for d in diffs:
                print("   ", d)
            total += len(diffs)
    print(f"total differences: {total}")


if __name__ == "__main__":
    main()

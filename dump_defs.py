#!/usr/bin/env python3
"""Dump what a firmware's resource set builds from, for comparison.

usage: dump_defs.py <repo root> old|new <platform> <board> <variant> <config|-> <out.json>

Runs the given root's own loader (the resource maps for "old", the YAML
tree for "new") and writes one record per resource with everything the
generators derive the output from: type, storage, aliases, the hash of
the source bytes and the font options that shape a ttf render.
"""

import argparse
import hashlib
import json
import os
import sys


def sha(path):
    h = hashlib.sha256()
    if os.path.isdir(path):
        for name in sorted(os.listdir(path)):
            if name.endswith(".svg"):
                h.update(name.encode())
                with open(os.path.join(path, name), "rb") as f:
                    h.update(f.read())
    else:
        with open(path, "rb") as f:
            h.update(f.read())
    return h.hexdigest()[:16]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("mode", choices=["old", "new"])
    ap.add_argument("platform")
    ap.add_argument("board")
    ap.add_argument("variant")
    ap.add_argument("config")
    ap.add_argument("out")
    args = ap.parse_args()

    os.chdir(args.root)
    sys.path.insert(0, os.path.join(args.root, "tools", "cmake"))
    # The driver is tools/cmake/resources.py; the generators are the
    # tools/resources package, so load the driver under another name.
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "resources_driver", os.path.join(args.root, "tools", "cmake", "resources.py")
    )
    drv = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(drv)
    from resources.resource_map import resource_generator

    drv.load_generators()
    config = drv._load_config(None if args.config == "-" else args.config)
    ns = argparse.Namespace(
        platform=args.platform, board_name=args.board, variant=args.variant,
        builddir="/nonexistent", node=None,
    )
    bld = drv.make_bld(ns)
    if args.mode == "old":
        _, rd = drv.get_resources_dict(bld, args.variant, config)
    else:
        suffix = ".prf.yaml" if args.variant == "prf" else ".yaml"
        ns.platform_file = os.path.join(drv.RESOURCES_DIR, "platforms", args.platform + suffix)
        ns.fragment = []
        _, rd = drv.load_tree(ns, config)

    records = {}
    for item in rd["media"]:
        for d in resource_generator.definitions_from_dict(bld, item, ""):
            src = os.path.join(drv.RESOURCES_DIR, d.file)
            rec = {
                "type": d.type,
                "storage": d.storage,
                "aliases": sorted(d.aliases),
                "source": os.path.relpath(os.path.realpath(src), args.root),
                "sha": sha(src),
            }
            if d.type == "font":
                rec["extended"] = bool(getattr(d, "extended", False))
                if not d.file.endswith(".pbf"):
                    cl = getattr(d, "character_list", None)
                    rec["font"] = {
                        "regex": getattr(d, "character_regex", None),
                        "list": sha(cl) if cl and os.path.exists(cl) else None,
                        "compat": getattr(d, "compatibility", None),
                        "tracking": getattr(d, "tracking_adjust", None),
                        "pixel_height": getattr(d, "pixel_height", None),
                        "compress": getattr(d, "compress", None),
                    }
            assert d.name not in records, d.name
            records[d.name] = rec

    out = {
        "order": list(records),
        "media": records,
        "timeline": rd.get("timeline") or [],
        "files": rd.get("files") or [],
    }
    with open(args.out, "w") as f:
        json.dump(out, f, indent=1, sort_keys=True)
    print(f"{args.mode} {args.board} {args.variant}: {len(records)} resources")


if __name__ == "__main__":
    main()

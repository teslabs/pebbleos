#!/usr/bin/env python3
"""Compare two resource balls by content, ignoring ids.

usage: compare_balls.py <repo root> <old.resball> <new.resball>
"""

import hashlib
import os
import sys


def load(root, path):
    sys.path.insert(0, os.path.join(root, "tools"))
    from resources.types.resource_ball import ResourceBall

    ball = ResourceBall.load(path)
    objects = {}
    for i, o in enumerate(ball.resource_objects, start=1):
        d = o.definition
        objects[d.name] = (
            d.type, d.storage, tuple(sorted(d.aliases)),
            hashlib.sha256(o.data).hexdigest()[:16], len(o.data), i,
        )
    decls = [d.name for d in ball.resource_declarations]
    return objects, decls


def main():
    root, old_path, new_path = sys.argv[1:4]
    old, old_decls = load(root, old_path)
    new, new_decls = load(root, new_path)
    diffs = 0
    for name in sorted(set(old) | set(new)):
        if name not in new:
            print(f"- {name} {old[name][:2]} {old[name][4]} bytes")
            diffs += 1
        elif name not in old:
            print(f"+ {name} {new[name][:2]} {new[name][4]} bytes")
            diffs += 1
        elif old[name][:5] != new[name][:5]:
            print(f"~ {name}: {old[name][:5]} -> {new[name][:5]}")
            diffs += 1
    if old_decls != new_decls:
        print(f"~ declarations: {old_decls} -> {new_decls}")
        diffs += 1
    renumbered = sum(1 for n in set(old) & set(new) if old[n][5] != new[n][5])
    print(f"{len(old)} old, {len(new)} new, {diffs} content differences, "
          f"{renumbered} resources renumbered")


if __name__ == "__main__":
    main()

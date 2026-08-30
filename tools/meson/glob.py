#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Expand source globs for meson.build.

Meson has no globbing of its own, so the directories that pick their
sources up by pattern shell out to this script at configure time. Paths
are printed one per line, relative to --base, sorted.

Exclusions follow the shapes the resource and vendor trees need: a plain
glob relative to --base ("prompt.c", "voice/*.c"), a recursive glob
("**/system_nrf*.c"), or a directory whose whole subtree goes away
("vendor", "ble_hrm/**").
"""

import argparse
import fnmatch
import glob
import os
import sys


def expand(base, patterns, recurse):
    found = []
    for pattern in patterns:
        if not os.path.isabs(pattern):
            pattern = os.path.join(base, pattern)
        if recurse:
            # GLOB_RECURSE walks the tree below the pattern's directory.
            directory, name = os.path.split(pattern)
            matches = glob.glob(os.path.join(directory, "**", name), recursive=True)
        else:
            matches = glob.glob(pattern)
        found.extend(m for m in matches if os.path.isfile(m))
    return found


def drop(base, paths, patterns):
    for pattern in patterns:
        if pattern.endswith("/**") or not any(c in pattern for c in "*."):
            directory = pattern[: -len("/**")] if pattern.endswith("/**") else pattern
            needle = os.sep + directory.strip(os.sep) + os.sep
            paths = [p for p in paths if needle not in p]
        elif pattern.startswith("**/"):
            paths = [p for p in paths if not fnmatch.fnmatch(p, "*/" + pattern[3:])]
        else:
            excluded = set(expand(base, [pattern], False))
            paths = [p for p in paths if p not in excluded]
    return paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", required=True, help="directory paths are relative to")
    parser.add_argument("--recurse", action="store_true")
    parser.add_argument("--allow-empty", action="store_true")
    parser.add_argument("--exclude", action="append", default=[])
    parser.add_argument(
        "--exclude-list",
        default="",
        help="';'-separated exclusions, for meson.build to pass an array through",
    )
    parser.add_argument("patterns", nargs="+")
    args = parser.parse_args()

    base = os.path.abspath(args.base)
    excluded = args.exclude + [p for p in args.exclude_list.split(";") if p]
    paths = expand(base, args.patterns, args.recurse)
    paths = drop(base, paths, excluded)
    paths = sorted({os.path.relpath(p, base) for p in paths})
    if not paths and not args.allow_empty:
        sys.exit(f"No sources matched {' '.join(args.patterns)} under {base}")
    print("\n".join(paths))


if __name__ == "__main__":
    main()

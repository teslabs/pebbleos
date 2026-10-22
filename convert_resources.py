#!/usr/bin/env python3
"""One-off: rewrite resources/{common,normal,prf}/**/resource_map.json as
YAML fragments and per-platform files, and git-mv the files by type.

Run from the repository root, with the old tree and loader still present.
"""

import collections
import itertools
import json
import os
import re
import subprocess
import sys

import yaml

ROOT = os.getcwd()
RES = os.path.join(ROOT, "resources")
sys.path.insert(0, os.path.join(ROOT, "tools", "cmake"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import wafshim  # noqa: E402

wafshim.setup_path()
from pebble_sdk_platform import pebble_platforms  # noqa: E402
from resources.find_resource_filename import find_most_specific_filename  # noqa: E402

PLATFORMS = ["emery", "gabbro", "flint"]
TAGS = ["bw", "color", "rect", "round"]
BOARDS = {
    "emery": ["obelix", "qemu_emery"],
    "gabbro": ["getafix", "qemu_gabbro"],
    "flint": ["asterix", "qemu_flint"],
}
CANONICAL = {"emery": "obelix", "gabbro": "getafix", "flint": "asterix"}
# Resources that exist only when a feature is enabled; today's maps leave
# them out of one board's copy instead.
CONFIG_GATED = {
    "BLE_HRM_SHARE_REQUEST_LARGE": "SERVICE_BLE_HRM",
    "BLE_HRM_SHARED": "SERVICE_BLE_HRM",
    "BLE_HRM_NOT_SHARED": "SERVICE_BLE_HRM",
    "CONNECTIVITY_SHARING_HRM": "SERVICE_BLE_HRM",
}
# Overrides that only exist to keep ids stable; dropped with the stub.
DROP_FILES = {"normal/base/raw/empty_stub.raw"}
# Resources only one platform's code can load (weather_app_layout.h loads
# the icon sequence under PBL_ROUND); today's maps ship them to others too.
PLATFORM_ONLY = {"WX_WEATHER_ICONS_PDC": ["gabbro"]}
FONT_KEYS = ["characterRegex", "characterList", "compatibility", "extended",
             "trackingAdjust", "pixelHeight", "compress"]
KEY_ORDER = ["name", "type", "file", "targetPlatforms", "builtin", "config",
             "aliases"] + FONT_KEYS


def load_map(path):
    with open(path) as f:
        return json.load(f)


def merged_map(board, variant):
    """resources.py get_resources_dict(), without the PRF forcing."""
    maps = [load_map(os.path.join(RES, "common/base/resource_map.json"))]
    root = "prf/" if variant == "prf" else "normal/"
    specific = load_map(os.path.join(RES, root, "base/resource_map.json"))
    maps.append(specific)
    for rel in (f"common/{board}/resource_map.json", f"{root}{board}/resource_map.json"):
        p = os.path.join(RES, rel)
        if os.path.exists(p):
            maps.append(load_map(p))
    media = []
    for m in maps:
        for item in m["media"]:
            for existing in media:
                if existing["name"] == item["name"]:
                    existing.update(item)
                    break
            else:
                media.append(dict(item))
    return media, specific.get("files"), specific.get("timeline")


class Bld:
    def fatal(self, msg):
        raise RuntimeError(msg)


def resolve(platform, file):
    if file.startswith(".."):
        return os.path.normpath(file)
    env = wafshim.Env()
    env.PLATFORM_NAME = platform
    root = wafshim.Node(RES)
    return find_most_specific_filename(Bld(), env, root, file)


def per_platform(variant):
    """name -> {platform: entry with resolved file}, in first-seen order."""
    table = collections.OrderedDict()
    tables = {}
    files = timeline = None
    for platform in PLATFORMS:
        for board in BOARDS[platform]:
            media, files, timeline = merged_map(board, variant)
            entries = collections.OrderedDict()
            for item in media:
                e = dict(item)
                e["file"] = resolve(platform, e["file"])
                if e["file"] in DROP_FILES:
                    continue
                entries[e["name"]] = e
            tables[board] = entries
        canon = tables[CANONICAL[platform]]
        for board in BOARDS[platform]:
            other = tables[board]
            for n in sorted(set(canon) | set(other)):
                if canon.get(n) != other.get(n):
                    print(f"  note: {variant} {board} differs from {CANONICAL[platform]} on {n}")
        for name, e in canon.items():
            table.setdefault(name, {})[platform] = e
        for board in BOARDS[platform]:
            for name, e in tables[board].items():
                table.setdefault(name, {}).setdefault(platform, e)
    return table, files, timeline


def strip_tags(basename):
    stem, ext = os.path.splitext(basename)
    return stem.split("~")[0] + ext


def choose_keys(groups, platforms):
    """groups: file -> set(platforms). Pick a file-map key per group so the
    loader's rule (platform beats tag, exactly one tag match) reproduces
    it, with as few keys as possible and tags over platform names."""
    options = []
    for file, plats in groups.items():
        cands = []
        for tag in TAGS:
            tagged = {p for p in platforms if tag in pebble_platforms[p]["TAGS"]}
            if plats <= tagged:
                cands.append((tag,))
        if len(plats) == 1:
            cands.append((next(iter(plats)),))
        cands.append(tuple(sorted(plats)))
        options.append([(file, c) for c in cands])
    best = None
    for combo in itertools.product(*options):
        spec = {}
        ok = True
        for file, keys in combo:
            for k in keys:
                if k in spec:
                    ok = False
                spec[k] = file
        if not ok:
            continue
        for p in platforms:
            want = next(f for f, pl in groups.items() if p in pl)
            if p in spec:
                got = spec[p]
            else:
                m = [k for k in spec if k in pebble_platforms[p]["TAGS"]]
                got = spec[m[0]] if len(m) == 1 else None
            if got != want:
                ok = False
        if not ok:
            continue
        score = (len(spec), sum(k in PLATFORMS for k in spec))
        if best is None or score < best[0]:
            best = (score, spec)
    assert best, groups
    return best[1]


def fragment_for(entry, resolved_file):
    t = entry["type"]
    if t == "font":
        return "fonts"
    if t == "vibe":
        return "vibes"
    f = resolved_file
    if (os.path.isdir(os.path.join(RES, f)) or "animated_pdcs/" in f
            or f.endswith("health_app_pulsing_heart.pdc")):
        return "animations"
    if f.startswith("normal/base/raw/"):
        return "data"
    return "images"


def dest_for(fragment, src, key):
    """Where a source file lands, relative to resources/."""
    base = os.path.basename(src)
    parts = [fragment]
    if fragment == "images":
        m = re.search(r"(\d+x\d+)", base)
        if m:
            parts.append(m.group(1))
    if key:
        parts.append(key)
    parts.append(strip_tags(base))
    return "/".join(parts)


def unify(table, moves, fragment=None):
    """One YAML entry per name from its per-platform entries, recording
    the source->destination moves it implies in ``moves``. A file another
    entry already moved is referenced where it went."""
    out = []
    for name, by_plat in table.items():
        plats = [p for p in PLATFORMS if p in by_plat]
        entries = [by_plat[p] for p in plats]
        types = {e["type"] for e in entries}
        if types == {"pbi", "pbi8"}:
            assert all(e["type"] == "pbi8" or "bw" in pebble_platforms[p]["TAGS"]
                       for p, e in by_plat.items()), name
            print(f"  note: {name}: pbi on bw platform declared as pbi8 (same output)")
            types = {"pbi8"}
        assert len(types) == 1, (name, types)
        fixed = {}
        for e in entries:
            for k, v in e.items():
                if k in ("file", "builtin", "_comment", "type", "name"):
                    continue
                if k in fixed and fixed[k] != v:
                    raise SystemExit(f"{name}: {k} differs across platforms: {fixed[k]!r} vs {v!r}")
                fixed[k] = v
        entry = {"name": name, "type": types.pop()}
        groups = collections.OrderedDict()
        for p in plats:
            groups.setdefault(by_plat[p]["file"], set()).add(p)
        frag = fragment or fragment_for(entry, next(iter(groups)))
        if len(groups) == 1:
            keys = {None: next(iter(groups))}
        else:
            keys = dict(choose_keys(groups, set(plats)))
        file_spec = {}
        for key, src in keys.items():
            if src.startswith(".."):
                dest_abs = os.path.normpath(os.path.join(RES, src))
                ref = os.path.relpath(dest_abs, os.path.join(RES, frag))
            else:
                dest = moves.setdefault(src, dest_for(frag, src, key))
                ref = os.path.relpath(dest, frag)
            file_spec[key] = ref
        if None in file_spec:
            entry["file"] = file_spec[None]
        else:
            entry["file"] = {k: file_spec[k] for k in TAGS + PLATFORMS if k in file_spec}
        if name in PLATFORM_ONLY:
            assert set(PLATFORM_ONLY[name]) <= set(plats), name
            plats = PLATFORM_ONLY[name]
            print(f"  note: {name}: now {plats} only")
        if len(plats) < len(PLATFORMS):
            entry["targetPlatforms"] = plats
        builtin = [p for p in plats if by_plat[p].get("builtin")]
        if builtin and len(builtin) == len(plats):
            entry["builtin"] = True
        elif builtin:
            entry["builtin"] = builtin
        if name in CONFIG_GATED:
            entry["config"] = CONFIG_GATED[name]
        if "aliases" in fixed:
            entry["aliases"] = fixed["aliases"]
        is_pbf = all(f.endswith(".pbf") for f in groups)
        for k in FONT_KEYS:
            if k not in fixed or fixed[k] is None:
                continue
            if is_pbf and k in ("characterList", "characterRegex", "compatibility"):
                continue  # a .pbf is used as is; these only shape ttf rendering
            entry[k] = fixed[k]
        if "characterList" in entry:
            src = entry["characterList"]
            dest = moves.setdefault(src, dest_for(frag, src, None))
            entry["characterList"] = os.path.relpath(dest, frag)
        comments = []
        for e in entries:
            for line in e.get("_comment", []):
                if line not in comments:
                    comments.append(line)
        out.append((frag, entry, comments))
    return out


def compatible(normal, prf):
    """A PRF entry the normal declaration already covers, once the PRF
    overrides (builtin, extended) are applied."""
    ignore = {"builtin", "extended", "targetPlatforms", "config"}
    a = {k: v for k, v in normal.items() if k not in ignore}
    b = {k: v for k, v in prf.items() if k not in ignore}
    return a == b


def scalar(v):
    text = yaml.safe_dump(v, default_flow_style=True, allow_unicode=True, width=10**6, sort_keys=False)
    if text.endswith("\n...\n"):
        text = text[:-5]
    return text.strip()


def emit_entry(entry, comments, indent="  "):
    lines = [f"{indent}# {c}" for c in comments]
    first = True
    for k in KEY_ORDER:
        if k not in entry:
            continue
        v = entry[k]
        lead = f"{indent}- " if first else f"{indent}  "
        first = False
        if isinstance(v, dict):
            lines.append(f"{lead}{k}:")
            for kk, vv in v.items():
                lines.append(f"{indent}    {kk}: {scalar(vv)}")
        else:
            lines.append(f"{lead}{k}: {scalar(v)}")
    return lines


def write_yaml(path, text, expect):
    assert yaml.safe_load(text) == expect, path
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"  wrote {os.path.relpath(path, ROOT)}")


def git(*args):
    subprocess.run(["git", *args], check=True, cwd=ROOT)


def main():
    print("normal:")
    normal_table, files, timeline = per_platform("normal")
    print("prf:")
    prf_table, _, _ = per_platform("prf")

    moves = {}
    normal = unify(normal_table, moves)
    by_name = {e["name"]: (frag, e) for frag, e, _ in normal}

    # PRF: select from the normal fragments where the declaration is the
    # same, declare the rest in recovery/.
    prf_only_table = collections.OrderedDict()
    prf_select = collections.defaultdict(list)
    for name, by_plat in prf_table.items():
        if name in by_name:
            frag, normal_entry = by_name[name]
            _, cand, _ = unify({name: by_plat}, dict(moves), fragment=frag)[0]
            if compatible(normal_entry, cand):
                prf_select[frag].append(name)
                continue
        prf_only_table[name] = by_plat
    print("  PRF selects:", dict(prf_select))
    print("  PRF-only:", list(prf_only_table))
    recovery = unify(prf_only_table, moves, fragment="recovery")

    # Files no entry names but that belong with their neighbours.
    extra = {
        "normal/base/images/Pebble_25x25_Paint_brush.LICENSE": "images/25x25/Pebble_25x25_Paint_brush.LICENSE",
        "normal/base/pbf/EMOJI.LICENSE.txt": "fonts/EMOJI.LICENSE.txt",
        "normal/base/pbf/EMOJI.NOTO.OFL.txt": "fonts/EMOJI.NOTO.OFL.txt",
        "normal/base/ttf/LICENSE": "fonts/LICENSE",
        "normal/base/ttf/LICENSE.DroidSerif.txt": "fonts/LICENSE.DroidSerif.txt",
        "normal/base/ttf/LICENSE.Roboto.txt": "fonts/LICENSE.Roboto.txt",
        "prf/base/ttf/LICENSE.sazanami-mincho": "recovery/LICENSE.sazanami-mincho",
        "prf/base/ttf/LICENSE_J.sazanami-mincho": "recovery/LICENSE_J.sazanami-mincho",
        "normal/base/layouts/layouts.json.in": "timeline/layouts.json.in",
        "normal/base/tzdata/timezones_olson.txt": "tzdata/timezones_olson.txt",
        "normal/base/tzdata/update_timezones.py": "tzdata/update_timezones.py",
    }
    for s, d in extra.items():
        assert s not in moves, s
        moves[s] = d
    dests = collections.Counter(moves.values())
    clashes = [d for d, n in dests.items() if n > 1]
    assert not clashes, clashes

    # Fragment files.
    frags = collections.OrderedDict((f, []) for f in ("fonts", "images", "animations", "vibes", "data"))
    for frag, e, c in normal:
        frags[frag].append((e, c))
    frags["recovery"] = [(e, c) for _, e, c in recovery]

    headers = {
        "fonts": "System fonts. A .pbf face is used as it is; the font options only\n# shape the faces rendered from a .ttf.",
        "images": "System images, by size and display variant.",
        "animations": "PDC animations: directories of SVG frames and pre-rendered sequences.",
        "vibes": "Vibration patterns.",
        "data": "Raw data blobs.",
        "recovery": "Resources only the recovery firmware builds.",
    }
    for frag, items in frags.items():
        lines = [f"# {headers[frag]}", "media:"]
        expect = {"media": []}
        for e, c in items:
            lines.append("")
            lines.extend(emit_entry(e, c))
            expect["media"].append(e)
        write_yaml(os.path.join(RES, frag, "resources.yaml"), "\n".join(lines) + "\n", expect)

    lines = ["# The published timeline icons: each id maps to one resource per size.",
             "timeline:"]
    for t in timeline:
        assert set(t) <= {"name", "id", "sizes", "internal"}, t
        lines.append(f"  - name: {t['name']}")
        lines.append(f"    id: {t['id']}")
        lines.append(f"    sizes: {scalar(t['sizes'])}")
        if "internal" in t:
            lines.append(f"    internal: {scalar(t['internal'])}")
    write_yaml(os.path.join(RES, "timeline/resources.yaml"), "\n".join(lines) + "\n",
               {"timeline": timeline})
    lines = ["# Resources stored on the filesystem rather than in the pack: each entry",
             "# is one file holding the listed resources in order.", "files:"]
    for entry in files:
        assert set(entry) == {"name", "resources"}, entry
        lines.append(f"  - name: {entry['name']}")
        lines.append("    resources:")
        lines.extend(f"      - {r}" for r in entry["resources"])
    write_yaml(os.path.join(RES, "lang/resources.yaml"), "\n".join(lines) + "\n",
               {"files": files})

    normal_includes = [f for f in frags if f != "recovery"] + ["timeline", "lang"]
    prf_includes = []
    for frag in frags:
        if frag in prf_select:
            prf_includes.append({"path": frag, "only": prf_select[frag]})
    prf_includes.append("recovery")
    for platform in PLATFORMS:
        data = {"include": normal_includes}
        lines = [f"# The {platform} firmware's resources, in id order.", "include:"]
        lines.extend(f"  - {f}" for f in normal_includes)
        write_yaml(os.path.join(RES, f"platforms/{platform}.yaml"), "\n".join(lines) + "\n", data)
        data = {"overrides": {"builtin": True, "extended": False}, "include": prf_includes}
        lines = [
            f"# The {platform} recovery firmware's resources. It has no resource pack,",
            "# so everything is linked into the image.",
            "overrides:",
            "  builtin: true",
            "  extended: false",
            "include:",
        ]
        for inc in prf_includes:
            if isinstance(inc, str):
                lines.append(f"  - {inc}")
            else:
                lines.append(f"  - path: {inc['path']}")
                lines.append("    only:")
                lines.extend(f"      - {n}" for n in inc["only"])
        write_yaml(os.path.join(RES, f"platforms/{platform}.prf.yaml"), "\n".join(lines) + "\n", data)

    # Move the files, then drop whatever the old tree still holds.
    for src, dest in sorted(moves.items()):
        os.makedirs(os.path.dirname(os.path.join(RES, dest)), exist_ok=True)
        git("mv", os.path.join("resources", src), os.path.join("resources", dest))
    left = subprocess.run(["git", "ls-files", "resources/common", "resources/normal", "resources/prf"],
                          capture_output=True, text=True, check=True, cwd=ROOT).stdout.split()
    print("  deleting:", len(left), "files")
    for f in left:
        print("    ", f)
    git("rm", "-q", *left)
    git("add", "resources")
    print(f"  moved {len(moves)} files")


if __name__ == "__main__":
    main()

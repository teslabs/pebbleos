#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Mass-migrate PBL_LOG_* call sites to per-module registration.

For each .c file under the firmware tree:
  * Inserts PBL_LOG_MODULE_REGISTER(<derived-name>, LOG_LEVEL_DEBUG) after
    the include block, unless a REGISTER/DECLARE is already present.
  * Rewrites PBL_LOG_D_<LEVEL>(LOG_DOMAIN_X, ...) to PBL_LOG_<LEVEL>(...).
  * Rewrites PBL_LOG_D_SYNC_<LEVEL>(LOG_DOMAIN_X, ...) to PBL_LOG_SYNC_<LEVEL>(...).
  * Rewrites RETURN_STATUS_D(LOG_DOMAIN_X, s) to RETURN_STATUS(s) (and _UP_D).
  * Drops #define DEFAULT_LOG_DOMAIN <...> lines.

Module name derivation:
  Default: lowercase filename basename without extension. Non-identifier
  characters become '_'. If the filename starts with a digit, prefix '_'.

Idempotent: running twice is a no-op.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

LEVEL_NAMES = ("ALWAYS", "ERR", "WRN", "INFO", "DBG", "VERBOSE")
SYNC_LEVEL_NAMES = LEVEL_NAMES

PBL_LOG_USE_RE = re.compile(
    # Macros that expand through NEW_LOG_HASH (not _ASSERT) — these reference
    # __pbl_log_module_name and therefore require PBL_LOG_MODULE_REGISTER.
    # PBL_ASSERT / PBL_CROAK / configASSERT use NEW_LOG_HASH_ASSERT (file:line)
    # and intentionally do NOT need REGISTER, so they are excluded here.
    r"\bPBL_LOG_(?:D_)?(?:SYNC_)?(?:" + "|".join(LEVEL_NAMES) + r")\b\s*\("
    r"|\bPBL_LOG_FROM_FAULT_HANDLER(?:_FMT)?\s*\("
    r"|\bPBL_HEXDUMP(?:_D)?(?:_D_SERIAL|_D_PROMPT|_SERIAL|_PROMPT)?\s*\("
    # Common per-subsystem wrapper macros that expand to PBL_LOG_*.
    r"|\b(?:ACTIVITY|BLE|ANIMATION|QEMU|ACCEL|I2C|KALG|VOICE|DLS)_LOG_[A-Z]+\s*\("
    r"|\b(?:BLE|QEMU|DLS|ACTIVITY)_HEXDUMP(?:_VERBOSE)?\s*\("
)
ALREADY_REGISTERED_RE = re.compile(r"\bPBL_LOG_MODULE_(?:REGISTER|DECLARE)\s*\(")
INCLUDE_LINE_RE = re.compile(r"^\s*#\s*include\b")
DEFAULT_DOMAIN_RE = re.compile(r"^\s*#\s*define\s+DEFAULT_LOG_DOMAIN\b.*$\n?", re.MULTILINE)


_TRIVIAL_PARENTS = {"src", "fw", "libos", "boot", "include", "tests",
                    "libutil", "third_party", "applib", "drivers"}


def derive_module(path: Path) -> str:
    """Derive a unique-ish module name from a file path.

    Prefers `<parent>_<stem>` to disambiguate common file names (service.c,
    sf32lb.c, layer.c, port.c). Falls back to just `<stem>` when the parent
    directory is too generic to add information."""
    stem = path.stem.lower()
    parent = path.parent.name.lower() if path.parent and path.parent.name else ""
    parts = [stem]
    if parent and parent != stem and parent not in _TRIVIAL_PARENTS:
        parts = [parent, stem]
    name = "_".join(parts)
    name = re.sub(r"[^a-z0-9_]", "_", name)
    if name and name[0].isdigit():
        name = "_" + name
    return name or "module"


_LEVELS = "|".join(LEVEL_NAMES)
_PBL_D_RE = re.compile(
    r"\bPBL_LOG_D_(" + _LEVELS + r")\s*\(\s*LOG_DOMAIN_[A-Z0-9_]+\s*,\s*"
)
_PBL_D_SYNC_RE = re.compile(
    r"\bPBL_LOG_D_SYNC_(" + _LEVELS + r")\s*\(\s*LOG_DOMAIN_[A-Z0-9_]+\s*,\s*"
)
_RS_D_RE = re.compile(r"\bRETURN_STATUS_D\s*\(\s*LOG_DOMAIN_[A-Z0-9_]+\s*,\s*")
_RS_UP_D_RE = re.compile(r"\bRETURN_STATUS_UP_D\s*\(\s*LOG_DOMAIN_[A-Z0-9_]+\s*,\s*")
_HEXDUMP_D_RE = re.compile(r"\bPBL_HEXDUMP_D\s*\(\s*LOG_DOMAIN_[A-Z0-9_]+\s*,\s*")
_HEXDUMP_D_SERIAL_RE = re.compile(r"\bPBL_HEXDUMP_D_SERIAL\b")
_HEXDUMP_D_PROMPT_RE = re.compile(r"\bPBL_HEXDUMP_D_PROMPT\b")


def rewrite_domain_calls(text: str) -> tuple[str, int]:
    """PBL_LOG_D_X(LOG_DOMAIN_*, ...) -> PBL_LOG_X(...). Same for SYNC and RETURN_STATUS."""
    n = 0
    text, c = _PBL_D_RE.subn(lambda m: f"PBL_LOG_{m.group(1)}(", text)
    n += c
    text, c = _PBL_D_SYNC_RE.subn(lambda m: f"PBL_LOG_SYNC_{m.group(1)}(", text)
    n += c
    text, c = _RS_D_RE.subn("RETURN_STATUS(", text)
    n += c
    text, c = _RS_UP_D_RE.subn("RETURN_STATUS_UP(", text)
    n += c
    text, c = _HEXDUMP_D_RE.subn("PBL_HEXDUMP(", text)
    n += c
    text, c = _HEXDUMP_D_SERIAL_RE.subn("PBL_HEXDUMP_SERIAL", text)
    n += c
    text, c = _HEXDUMP_D_PROMPT_RE.subn("PBL_HEXDUMP_PROMPT", text)
    n += c
    return text, n


def drop_default_domain(text: str) -> tuple[str, int]:
    new, n = DEFAULT_DOMAIN_RE.subn("", text)
    return new, n


_C_BLOCK_COMMENT_OPEN = re.compile(r"/\*")
_C_BLOCK_COMMENT_CLOSE = re.compile(r"\*/")


_INCLUDE_RE = re.compile(r"^\s*#\s*include\b")
_PP_DEFINE_RE = re.compile(r"^\s*#\s*(?:define|undef|pragma|error)\b")
_PP_IF_RE = re.compile(r"^\s*#\s*(?:if|ifdef|ifndef|else|elif|endif)\b")


def find_insert_offset(text: str) -> int:
    """Return offset for inserting REGISTER at unambiguous file scope.

    Walk top-down. Skip blanks, comments, #include, #define/#undef/#pragma/
    #error and their backslash continuations. Stop at any conditional
    compilation directive (#if/#ifdef/#ifndef/#else/#elif/#endif) — REGISTER
    must not be wrapped in a conditional that might compile it out — and at
    any real C statement. Insert REGISTER at the boundary, where it sits
    after #defines / #includes but before any #if-gated section."""
    lines = text.splitlines(keepends=True)
    in_block_comment = False
    in_pp_continuation = False
    last_safe = 0  # one past the last "safe-to-skip" line
    i = 0
    while i < len(lines):
        line = lines[i]
        rstrip = line.rstrip("\r\n")
        stripped = rstrip.strip()

        if in_block_comment:
            if _C_BLOCK_COMMENT_CLOSE.search(line):
                in_block_comment = False
            i += 1
            last_safe = i
            continue

        if in_pp_continuation:
            in_pp_continuation = rstrip.endswith("\\")
            i += 1
            last_safe = i
            continue

        if stripped == "":
            i += 1
            last_safe = i
            continue

        if stripped.startswith("//"):
            i += 1
            last_safe = i
            continue

        if stripped.startswith("/*"):
            if not _C_BLOCK_COMMENT_CLOSE.search(stripped[2:]):
                in_block_comment = True
            i += 1
            last_safe = i
            continue

        if _PP_IF_RE.match(line):
            # Conditional compilation — stop. REGISTER must not be inside.
            break

        if _INCLUDE_RE.match(line) or _PP_DEFINE_RE.match(line):
            in_pp_continuation = rstrip.endswith("\\")
            i += 1
            last_safe = i
            continue

        # Real C code.
        break

    return sum(len(l) for l in lines[:last_safe])


def process(path: Path, dry_run: bool) -> str | None:
    try:
        raw = path.read_bytes()
    except OSError:
        return None
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError:
        return None

    # Detect dominant line ending so we don't accidentally CRLF→LF the file.
    if b"\r\n" in raw:
        eol = "\r\n"
        text = text.replace("\r\n", "\n")
    else:
        eol = "\n"

    if not PBL_LOG_USE_RE.search(text):
        return None  # nothing to migrate

    original = text

    # 1) Drop DEFAULT_LOG_DOMAIN defines.
    text, n_dropped = drop_default_domain(text)

    # 2) Rewrite domain-aware call sites.
    text, n_calls = rewrite_domain_calls(text)

    # 3) Insert REGISTER if neither REGISTER nor DECLARE present.
    n_register = 0
    if not ALREADY_REGISTERED_RE.search(text):
        module = derive_module(path)
        offset = find_insert_offset(text)
        register_line = f"PBL_LOG_MODULE_REGISTER({module}, LOG_LEVEL_DEBUG);\n\n"
        text = text[:offset] + register_line + text[offset:]
        n_register = 1

    if text == original:
        return None

    if not dry_run:
        # Preserve original line endings.
        out = text if eol == "\n" else text.replace("\n", eol)
        path.write_bytes(out.encode("utf-8"))

    return f"  {path}: register={n_register} domain-rewrites={n_calls} default-domain-dropped={n_dropped}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("roots", nargs="+", type=Path,
                    help="directories or files to migrate")
    ap.add_argument("--dry-run", action="store_true",
                    help="report what would change without writing")
    args = ap.parse_args()

    files: list[Path] = []
    for root in args.roots:
        if root.is_file():
            files.append(root)
        else:
            files.extend(root.rglob("*.c"))

    files.sort()
    changed = 0
    for f in files:
        msg = process(f, args.dry_run)
        if msg:
            print(msg)
            changed += 1

    print(f"\n{'Would migrate' if args.dry_run else 'Migrated'} {changed} files.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

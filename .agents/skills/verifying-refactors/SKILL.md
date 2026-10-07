---
name: verifying-refactors
description: Use when making changes to PebbleOS that must not change the generated code (include cleanups, file moves, renames, reformatting, macro rewrites) and need proof of that.
---

# Verifying refactors

Read "Checking that a change does not change the code" in
`docs/development/build_system.md`: compare object hashes before and
after, in every board configuration the change touches.

Agent notes:

- "It builds" and "the tests pass" are not evidence for these changes;
  identical objects are.
- When objects differ, look at why before accepting it: disassemble both
  (`arm-none-eabi-objdump -d`) and diff. Differences limited to log line
  numbers are expected only if lines were added or deleted.
- When removing includes, a header that seems to declare nothing may
  forward to one that does (`#include_next`, `sys/`, `machine/`), and a
  removal that is fine on its own can break together with another one.
  Finish with a full rebuild and comparison of every object.
- Use a separate build directory per configuration and keep them for the
  whole task, so each comparison only rebuilds what changed.

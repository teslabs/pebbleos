---
name: analyzing-coredumps
description: Use when analyzing a PebbleOS crash, coredump, HardFault or stack overflow report, or pulling a coredump from a watch.
---

# Analyzing coredumps

Read the Coredumps section of `docs/development/debugging.md`. It covers
pulling a dump over PULSE, matching the ELF by build ID, converting it
with `tools/readcore.py`, and driving GDB when it has no Python support.

Agent notes:

- Check the build ID before trusting any symbol: an ELF from a different
  build resolves addresses to plausible but wrong functions.
- Run GDB in `--batch` mode with `-ex` commands; interactive sessions
  hang the tool call.
- Do not take the crashing thread's top frame at face value on a stack
  overflow or a fault inside an exception handler. Dump the raw stack
  (`x/256xw $sp`) and resolve the words that fall inside `.text` to
  rebuild the chain.
- When the faulting LR or PC lies outside `.text`, suspect a corrupted or
  overflowed stack before suspecting the code at that address.
- Flashing a watch or erasing its coredump slot changes the device state
  the user may still need; ask before doing either.

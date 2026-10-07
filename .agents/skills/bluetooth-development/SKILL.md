---
name: bluetooth-development
description: Use when changing or debugging PebbleOS Bluetooth code (subsys/bluetooth, NimBLE glue, PPoGATT, GATT client, pairing, advertising or scanning).
---

# Bluetooth development

Read `docs/development/bluetooth.md` before changing code in
`subsys/bluetooth`: it lists NimBLE and controller behaviour that has
caused bugs before. For testing against a phone, read the Bluetooth
section of `docs/development/integration_tests.md`; for a real controller
on QEMU, the Bluetooth section of `docs/development/qemu.md`.

Agent notes:

- Every new driver-initiated `ble_gattc_*` procedure goes through
  `nimble_gattc_op_queue`, even if it looks like it cannot overlap with
  anything else.
- When a symptom looks like a controller or phone bug (wrong data,
  reconnect loops, stalls), first rule out two GATT procedures in flight
  at once and `bt_lock` held across a NimBLE call.
- Get evidence from both sides before settling on a theory: the watch's
  logs (see the console section of `docs/development/debugging.md`) and an
  HCI trace of the phone side (`BUMBLE_SNOOPER`).
- Bluetooth timing on QEMU differs from hardware. A fix for a race needs
  repeated runs (e.g. the affected test in a loop) before it is called
  fixed, and hardware results are worth more than emulator ones.

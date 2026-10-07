---
name: measuring-power
description: Use when measuring or comparing PebbleOS current consumption, investigating battery life or wake-ups, or driving a PPK2 or Analog Discovery.
---

# Measuring power

Read `docs/development/power.md` for measuring by hand and comparing
builds, and the current measurement section of
`docs/development/integration_tests.md` for the power tests.

Agent notes:

- Prefer the integration test power helpers (`power.measure_idle`) over
  new scripts when a test can express the measurement.
- Keep one process owning the PPK2 for the whole session and send it
  measurement requests; every script that opens and closes the port power
  cycles the watch.
- Report medians and the spread over several interleaved runs, with the
  voltage, board, build and watchface. A single run's mean is not a
  result.
- When a change saves power, explain where from: count wake-ups and the
  charge per wake-up in the trace before and after.
- Flashing a watch, power cycling it or wiping it needs the user's OK.

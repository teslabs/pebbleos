# Power measurement

The integration tests measure current with a Nordic Power Profiler Kit II
(PPK2) used as a source meter; see the current measurement section of
{doc}`integration_tests` for wiring it and for the power tests. This page
covers measuring by hand, for investigations the tests do not cover.

## The PPK2

`tests/integration/harness/helpers/power.py` has a `Ppk2` class that can be
used on its own: it supplies the watch at a given voltage and records 1 ms
averages. Run from `tests/integration`:

```python
import time

from harness.helpers.power import Ppk2

ppk = Ppk2("auto", 3800)
ppk.power_on()
time.sleep(90)
with ppk.measure("idle") as m:
    time.sleep(60)
print(m)
```

- The PPK2 cuts the supply when its serial port is closed, so the watch
  stays powered only while a process holds the port open. For a series of
  measurements, keep one long-lived process that owns the PPK2 and have
  the other scripts ask it to measure.
- Only one program can open the port. The nRF Connect Power Profiler app
  keeps it while open; close it first.
- Flashing the watch works while another process holds the PPK2.

## Preparing the watch

A debug build is fine for measuring, as long as the console is not
listening: while the UART receiver is enabled the watch cannot enter deep
sleep. The shell command `sys rx_disable <seconds>` turns it off for the
given time; close the console connection right after, and only measure
inside that time.

- Let the watch settle after boot or flashing: early boot work (storage,
  caches, the phone reconnecting) keeps it busy for over a minute.
- To leave the radio out, put the watch in airplane mode with
  `bt airplane on`.
- Use a watchface that redraws once a minute, e.g. TicToc. A watchface
  showing seconds wakes the watch every second.

## Comparing builds

- The watch does work once a minute (time keeping, calibration, the
  watchface redraw), which shows up as a burst much larger than anything
  else. A 30 s window may or may not contain it, which moves the mean by
  more than many changes are worth. Measure at least 60 s, or compare
  medians, or the mean of the samples below 1 mA.
- Interleave the builds under comparison (A, B, A, B...) rather than
  measuring all runs of one first: battery voltage, temperature and the
  phone's behaviour drift over time.
- Look at the trace, not only the mean. Counting wake-ups and the charge
  of each one (in µC) tells which part of the system changed.

## Waveforms with an oscilloscope

For signals the PPK2 is too slow for (e.g. driving a motor or a speaker), a
Digilent Analog Discovery can be scripted from Python through the WaveForms
SDK (`dwf`) with `ctypes`:

- The WaveForms application holds the device exclusively; quit it before
  opening the device from a script.
- On macOS, the SDK ships inside `WaveForms.app`; link
  `/Library/Frameworks/dwf.framework` to
  `WaveForms.app/Contents/Frameworks/dwf.framework` so the library finds its
  USB transport.
- Record mode streams without losing samples up to about 400 kS/s from
  Python. Send commands to the watch from another thread while recording,
  not before starting it.

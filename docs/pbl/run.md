# `pbl run`

Run the firmware with the board's {ref}`runner <pbl-runners>`: in the
emulator for a QEMU board, or flashed to a watch.

```shell
pbl run
```

Arguments after `--` go to the program the runner starts, e.g. QEMU:

```shell
pbl run -- -d guest_errors
```

## In the emulator

On a QEMU board, the `qemu` runner launches the firmware in the emulator:

```shell
pbl configure --board qemu_emery
pbl build
pbl run
```

The flash image is rebuilt on every launch; `--keep-flash-image` keeps it,
and with it installed apps, data and Bluetooth bondings.

The launched QEMU exposes:

- An interactive QEMU monitor on the launching terminal (`-monitor stdio`)
- A programmatic socket monitor (`-monitor unix:build/qemu-mon.sock`)
- The serial console over TCP on `localhost:12345`, for {doc}`console`
- The Pebble protocol over TCP on `localhost:12344`, for the `pebble` tool
  and {doc}`feed`

UART1 output is also captured to `uart1.log` in the repository root.
{doc}`../development/emulation/emulator` covers what the emulator models and how to
operate it.

## On a watch

On a watch connected through a {doc}`programmer <../programmers/index>`,
running the firmware is flashing it: `pbl run` does what {doc}`flash` does,
and takes the same options.

## Options

```{pbl-command} run
```

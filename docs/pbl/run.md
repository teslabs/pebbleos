# `pbl run`

Run the firmware with the board's {ref}`runner <pbl-runners>`: in the
emulator for a QEMU board, as a host program or in a browser for a native
board, or flashed to a watch.

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

## On the host

On a native board, the `native` runner starts the
{doc}`native simulator <../development/emulation/native_simulator>`, with
its external flash in `pebbleos-flash.bin` in the build directory, kept
across runs. Arguments after `--` are the program's own options, e.g. a
window scale factor:

```shell
pbl configure --board native_emery
pbl build
pbl run -- -s 2
```

`--bt-hci` connects the Bluetooth HCI UART of a `CONFIG_BT_HCI_UART` build
to a controller, as in the emulator.

## In a browser

The `wasm` variant of a native board is run by the `wasm` runner: it serves
the build directory on `localhost`, port 8000 or `--http-port`, and opens
the page in a browser, unless `--no-browser` is given. `Ctrl-C` stops it;
reloading the page picks up a rebuild.

```shell
pbl configure --board native_emery/wasm
pbl build
pbl run
```

## On a watch

On a watch connected through a {doc}`programmer <../programmers/index>`,
running the firmware is flashing it: `pbl run` does what {doc}`flash` does,
and takes the same options.

## Options

```{pbl-command} run
```

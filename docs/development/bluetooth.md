# Bluetooth

The Bluetooth stack lives in `subsys/bluetooth`: the NimBLE host, with the
controller reached through the HCI transport the board selects
(`CONFIG_BT_HCI_*`). On QEMU the default transport is a fake controller
that never forms a link; see {doc}`qemu` to attach a real one, and
{doc}`integration_tests` for testing against a phone.

This page collects behaviour of NimBLE and of the controllers that is not
obvious from their documentation and has caused bugs before.

## NimBLE

### One GATT client procedure per connection

NimBLE hands an incoming ATT response to the first pending procedure that
accepts its opcode. Several procedures share one: service, characteristic
and include discovery and read-by-UUID all consume Read By Type
responses. Two of them running at once on the same connection receive
each other's responses, with no error reported to either.

Driver-initiated `ble_gattc_*` procedures must therefore go through
`nimble_gattc_op_queue`: push the operation with
`nimble_gattc_op_queue_push()`, and call `nimble_gattc_op_queue_complete()`
from its terminal callback so the next one can start. Operations start on
the NimBLE host task. Discovery in `gatt_client_discovery.c` and the
device name reads in `gap_le_device_name.c` follow this pattern.

### Locking

Do not call NimBLE APIs while holding `bt_lock`. The host takes its own
mutex and calls back into code that takes `bt_lock`, so holding both in
the opposite order deadlocks against the host task.

### Notification flow control

`BLE_GAP_EVENT_NOTIFY_TX` is emitted from inside the notify call, on the
caller's stack, with the result of that attempt. It does not mean the
transmit buffers drained, and a send that failed with `BLE_HS_ENOMEM` is
never followed by another event. NimBLE has no callback for buffers being
freed, so recovering from buffer exhaustion takes a retry timer, as
PPoGATT does with `PPOGATT_SEND_RETRY_DELAY_MS`, about one connection
event.

## SF32LB52 controller

### Scanning

The SF32LB52 controller only delivers advertising reports reliably when
the scan window covers the advertiser's interval: with a window shorter
than the advertising interval it accepts the parameters and then reports
little or nothing. The simplest safe choice is a continuous scan, with the
window equal to the interval, which is also what NimBLE uses by default.
Long scan intervals (2 s and above) also lose reports, even with a wide
window.

To save power, start and stop a continuous scan from software rather than
relying on the controller's duty cycle.

## Capturing traffic

Integration tests drive the phone side with
[Bumble](https://google.github.io/bumble/), which writes an HCI trace of
its controller when `BUMBLE_SNOOPER` is set:

```shell
BUMBLE_SNOOPER=btsnoop:file:/tmp/phone.btsnoop pbl itest ...
```

Open the result with Wireshark. It shows the phone's side of the link,
including the ATT exchanges the watch takes part in.

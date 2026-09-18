/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
// Exercise the actual NimBLE extension with deterministic OS/transport substitutes.
#define H_BLE_HS_PRIV_
#define H_NIMBLE_TRANSPORT_
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#define BLE_HS_EINVAL      3
#define BLE_HS_EAGAIN      4
#define BLE_HS_ECONTROLLER 5
struct os_mbuf {
  unsigned length;
  uint8_t data[64];
};
struct ble_hci_ev {
  uint8_t opcode, length, data[255];
};
#define OS_MBUF_PKTLEN(om) ((om)->length)
static unsigned resets, sends, frees, events, received, reset_callbacks, locked;
uint16_t ble_hs_hci_avail_pkts;
static uint16_t get_le16(const void *data) {
  const uint8_t *p = data;
  return p[0] | p[1] << 8;
}
static void ble_hs_lock(void) {
  assert(!locked++);
}
static void ble_hs_unlock(void) {
  assert(locked-- == 1);
}
static void ble_hs_sched_reset(int reason) {
  assert(reason == BLE_HS_ECONTROLLER);
  ++resets;
}
static void ble_hs_wakeup_tx(void) {
}
static void ble_hs_hci_add_avail_pkts(uint16_t n) {
  assert(locked);
  ble_hs_hci_avail_pkts += n;
  assert(ble_hs_hci_avail_pkts <= 5);
}
static int os_mbuf_copydata(const struct os_mbuf *om, unsigned off, unsigned n, void *data) {
  if (off + n > om->length)
    return -1;
  memcpy(data, om->data + off, n);
  return 0;
}
static void os_mbuf_free_chain(struct os_mbuf *om) {
  ++frees;
}
static int ble_transport_to_ll_acl(struct os_mbuf *om) {
  assert(locked);
  ++sends;
  os_mbuf_free_chain(om);
  return 0;
}
static void *ble_hs_conn_find(uint16_t handle) {
  assert(locked);
  return handle == 0x42 ? &sends : NULL;
}
#include "../../third_party/nimble/mynewt-nimble/nimble/host/src/ble_hs_classic.c"

static void event_cb(const struct ble_hci_ev *ev) {
  assert(!locked);
  ++events;
}
static void acl_cb(const struct os_mbuf *om) {
  assert(!locked);
  ++received;
}
static void reset_cb(void) {
  assert(!locked);
  ++reset_callbacks;
}
static const struct ble_hs_classic_callbacks cb = {event_cb, acl_cb, reset_cb};
static struct os_mbuf packet = {.length = 8, .data = {0x11, 0x20, 4, 0}};

static void init(bool shared) {
  ble_hs_classic_register(&cb);
  ble_hs_classic_reset();
  ble_hs_classic_features(0);
  assert(ble_hs_classic_buffers(32, shared ? 5 : 2, shared) == 0);
  ble_hs_hci_avail_pkts = 5;
  struct ble_hci_ev ev = {.opcode = 3, .length = 11, .data = {0, 0x11}};
  ev.data[9] = 1;
  assert(ble_hs_classic_event(&ev));
}
static void complete(unsigned n) {
  struct ble_hci_ev ev = {.opcode = 0x13, .length = 5, .data = {1, 0x11, 0, n, 0}};
  assert(!ble_hs_classic_event(&ev));
}
static void disconnect(void) {
  struct ble_hci_ev ev = {.opcode = 5, .length = 4, .data = {0, 0x11, 0, 0x13}};
  assert(ble_hs_classic_event(&ev));
}
int main(int argc, char **argv) {
  assert(argc == 2);
  int test = atoi(argv[1]);
  init(test != 1);
  switch (test) {
    case 0:                       // Both protocols consume a single shared budget.
      ble_hs_hci_avail_pkts -= 2; // Two outstanding LE packets.
      for (unsigned i = 0; i < 3; ++i)
        assert(!ble_hs_classic_acl_tx(&packet));
      assert(ble_hs_classic_acl_tx(&packet) == BLE_HS_EAGAIN);
      assert(sends == 3 && ble_hs_hci_avail_pkts == 0);
      complete(2);
      assert(ble_hs_hci_avail_pkts == 2);
      disconnect();
      assert(ble_hs_hci_avail_pkts == 3);
      complete(1); // Late completion cannot release disconnected credits again.
      assert(ble_hs_hci_avail_pkts == 3);
      break;
    case 1: // Separate BR and LE pools do not borrow one another's credits.
      assert(!ble_hs_classic_acl_tx(&packet));
      assert(!ble_hs_classic_acl_tx(&packet));
      assert(ble_hs_classic_acl_tx(&packet) == BLE_HS_EAGAIN);
      assert(ble_hs_hci_avail_pkts == 5);
      complete(1);
      assert(!ble_hs_classic_acl_tx(&packet));
      assert(ble_hs_classic_acl_tx(&packet) == BLE_HS_EAGAIN);
      break;
    case 2: { // Mixed completions leave LE entries for NimBLE's LE handler.
      assert(!ble_hs_classic_acl_tx(&packet));
      struct ble_hci_ev ev = {
        .opcode = 0x13,
        .length = 9,
        .data = {2, 0x42, 0, 3, 0, 0x11, 0, 1, 0}
      };
      assert(!ble_hs_classic_event(&ev));
      assert(ble_hs_hci_avail_pkts == 5 && !resets);
      break;
    }
    case 3: // Over-completion requires recovery and cannot inflate the budget.
      assert(!ble_hs_classic_acl_tx(&packet));
      complete(2);
      assert(resets == 1 && ble_hs_hci_avail_pkts == 4);
      break;
    case 4: { // Malformed event and packet lengths cannot alter ownership.
      struct ble_hci_ev ev = {.opcode = 0x13, .length = 4, .data = {1, 0x11, 0, 1}};
      assert(!ble_hs_classic_event(&ev));
      packet.data[2] = 5;
      assert(ble_hs_classic_acl_tx(&packet) == BLE_HS_EINVAL);
      assert(!sends && ble_hs_hci_avail_pkts == 5 && !resets);
      break;
    }
    case 5: // Reset invalidates handles until startup and connection finish again.
      assert(!ble_hs_classic_acl_tx(&packet));
      ble_hs_classic_reset();
      assert(!ble_hs_classic_supported() && !ble_hs_classic_acl_mtu());
      assert(ble_hs_classic_acl_tx(&packet) == BLE_HS_EINVAL);
      assert(!ble_hs_classic_rx_acl(&packet));
      assert(reset_callbacks == 2);
      init(true);
      assert(!ble_hs_classic_acl_tx(&packet));
      break;
    case 6: // Dispatch is by registered handle, never a vendor handle range.
      assert(ble_hs_classic_rx_acl(&packet));
      packet.data[0] = 0x42;
      assert(!ble_hs_classic_rx_acl(&packet));
      assert(received == 1 && frees == 1);
      break;
    case 7: // LE-only controllers do not receive Classic configuration/events.
      ble_hs_classic_features(UINT64_C(1) << 37);
      assert(!ble_hs_classic_supported() && !ble_hs_classic_event_mask());
      break;
    case 8: {
      // A controller cannot assign a live LE handle to a Classic connection.
      ble_hs_classic_reset();
      ble_hs_classic_features(0);
      struct ble_hci_ev ev = {.opcode = 3, .length = 11, .data = {0, 0x42}};
      ev.data[9] = 1;
      assert(ble_hs_classic_event(&ev));
      assert(resets == 1);
      break;
    }
    case 9: {
      // Link_Type is undefined on a failed/canceled connection completion.
      unsigned before = events;
      struct ble_hci_ev ev = {.opcode = 3, .length = 11, .data = {2}};
      assert(ble_hs_classic_event(&ev));
      assert(events == before + 1 && !resets);
      assert(!ble_hs_classic_acl_tx(&packet));
      break;
    }
    default:
      abort();
  }
  assert(!locked);
}

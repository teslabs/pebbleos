/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#ifdef CONFIG_PROMPT
#include <console/prompt.h>
#include <host/ble_hs.h>

void command_ble_host_reset(void) {
  ble_hs_sched_reset(BLE_HS_EAPP);
  prompt_send_response("OK");
}

#ifdef CONFIG_BT_CLASSIC
#include <host/ble_gap.h>
#include <host/ble_store.h>
#include "classic/service.h"
#include <nimble/nimble_port.h>
#include <pbl/bluetooth/hfp.h>
#include "ble_hs_hci_priv.h"
#include <stdio.h>

extern unsigned nimble_host_reset_count(void);

static struct ble_npl_event s_dual_status;
typedef struct {
  unsigned count;
  uint16_t handles[MYNEWT_VAL(BLE_MAX_CONNECTIONS)];
} Connections;

static int collect_connections(uint16_t handle, void *context) {
  Connections *connections = context;
  if (connections->count < MYNEWT_VAL(BLE_MAX_CONNECTIONS))
    connections->handles[connections->count++] = handle;
  return 0;
}

static void report_dual_status(struct ble_npl_event *event) {
  Connections connections = {0};
  ble_gap_conn_foreach_handle(collect_connections, &connections);
  HfpStatus hfp;
  hfp_get_status(&hfp);
  char line[160];
  snprintf(line, sizeof(line),
           "NimBLE enabled=%u synced=%u LE=%u BR=%u HFP=%u SCO=%u errors=%u resets=%u",
           ble_hs_is_enabled(), ble_hs_synced(), connections.count, hfp.connected, hfp.ready,
           hfp.audio, hfp.errors, nimble_host_reset_count());
  prompt_send_response(line);
  for (unsigned i = 0; i < connections.count; ++i) {
    struct ble_gap_conn_desc desc;
    if (!ble_gap_conn_find(connections.handles[i], &desc)) {
      const uint8_t *a = desc.peer_id_addr.val;
      snprintf(line, sizeof(line),
               "LE handle=%u encrypted=%u bonded=%u peer=%02x:%02x:%02x:%02x:%02x:%02x type=%u",
               desc.conn_handle, desc.sec_state.encrypted, desc.sec_state.bonded, a[5], a[4], a[3],
               a[2], a[1], a[0], desc.peer_id_addr.type);
      prompt_send_response(line);
      struct ble_store_key_sec lookup = {.peer_addr = desc.peer_id_addr};
      lookup.peer_addr.type &= 1;
      struct ble_store_value_sec bond;
      if (!ble_store_read_peer_sec(&lookup, &bond)) {
        snprintf(line, sizeof(line), "Bond SC=%u authenticated=%u CTKD=%u CT2=%u key_size=%u",
                 bond.sc, bond.authenticated, bond.ctkd, bond.ct2, bond.key_size);
        prompt_send_response(line);
      }
    }
  }
  if (ble_hs_is_enabled() && ble_hs_synced()) {
    uint8_t scan = 0xff;
    int rc = ble_hs_hci_cmd_tx(0x0c19, NULL, 0, &scan, sizeof(scan));
    BtClassicHost *host = hfp_service_host();
    snprintf(line, sizeof(line),
             "Classic scan=%u status=%d encrypted=%u initiating=%u stage=%u rfcomm_initiator=%u",
             scan, rc, host->encrypted, host->connecting, host->connect_stage,
             host->rfcomm_initiator);
    prompt_send_response(line);
    if (host->handle != BT_CLASSIC_NO_HANDLE) {
      uint8_t handle[] = {host->handle & 0xff, host->handle >> 8};
      uint8_t role[3] = {0, 0, 0xff};
      rc = ble_hs_hci_cmd_tx(0x0809, handle, sizeof(handle), role, sizeof(role));
      snprintf(line, sizeof(line), "Classic role=%u status=%d", role[2], rc);
      prompt_send_response(line);
    }
  }
  prompt_command_finish();
}

void command_bt_dual_status(void) {
  ble_npl_event_init(&s_dual_status, report_dual_status, NULL);
  prompt_command_continues_after_returning();
  ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_dual_status);
}
#endif

#endif

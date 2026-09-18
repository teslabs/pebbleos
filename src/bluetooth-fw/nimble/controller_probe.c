/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <console/prompt.h>
#include <host/ble_hs.h>
#include <nimble/nimble_port.h>
#include <pbl/btutil/hci_probe.h>

#include "ble_hs_hci_priv.h"

static struct ble_npl_event s_probe_event;

static int prv_read(uint16_t opcode, uint8_t *response, uint8_t length, void *context) {
  int rc = ble_hs_hci_cmd_tx(opcode, NULL, 0, response, length);
  prompt_watchdog_feed();
  if (rc >= BLE_HS_ERR_HCI_BASE && rc < BLE_HS_ERR_HCI_BASE + 256) {
    return rc - BLE_HS_ERR_HCI_BASE;
  }
  return -rc;
}

static void prv_output(const char *line, void *context) {
  prompt_send_response(line);
}

static void prv_probe(struct ble_npl_event *event) {
  if (ble_hs_synced()) {
    bt_hci_probe(prv_read, prv_output, NULL);
  } else {
    prompt_send_response("Controller unavailable: Bluetooth host is not synchronized");
  }
  prompt_command_finish();
}

void command_bt_controller_probe(void) {
  if (!ble_hs_synced()) {
    prompt_send_response("Controller unavailable: Bluetooth host is not synchronized");
    return;
  }
  ble_npl_event_init(&s_probe_event, prv_probe, NULL);
  prompt_command_continues_after_returning();
  ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_probe_event);
}

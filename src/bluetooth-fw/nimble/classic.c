/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "classic.h"
#include "nimble_store.h"
#include <host/ble_sm.h>
#include "../classic/service.h"
#include "../hci_bridge/local_audio.h"
#include <host/ble_hs.h>
#include <host/ble_hs_classic.h>
#include <host/ble_hs_mbuf.h>
#include "ble_hs_priv.h"
#include <nimble/nimble_port.h>
#include <pbl/kernel/sem.h>
#include <pbl/kernel/sched.h>
#include <system/passert.h>

static struct ble_npl_callout s_poll;
static struct ble_npl_event s_wake, s_stop;
static PBL_SEM_DEFINE(s_stopped, 0, 1);
static bool s_running, s_stopping;

static void send_command(const uint8_t *p, size_t length, void *context) {
  uint16_t opcode = p[1] | (uint16_t)p[2] << 8;
  int rc = ble_hs_hci_cmd_tx(opcode, p + 4, p[3], NULL, 0);
  uint8_t status = rc >= BLE_HS_ERR_HCI_BASE && rc < BLE_HS_ERR_HCI_BASE + 256
                       ? rc - BLE_HS_ERR_HCI_BASE
                   : rc ? 0x1f
                        : 0;
  // Adapt NimBLE's completed transaction to the portable profile's work queue.
  const uint8_t ack[] = {4, 0x0e, 4, 1, p[1], p[2], status};
  bt_classic_receive(hfp_service_host(), ack, sizeof(ack));
}

static bool send_acl(const uint8_t *p, size_t length, void *context) {
  struct os_mbuf *om = ble_hs_mbuf_acl_pkt();
  if (!om)
    return false;
  if (os_mbuf_append(om, p + 1, length - 1)) {
    os_mbuf_free_chain(om);
    return false;
  }
  int rc = ble_hs_classic_acl_tx(om);
  if (rc) {
    os_mbuf_free_chain(om);
    if (rc != BLE_HS_EAGAIN)
      ble_hs_sched_reset(rc);
  }
  return rc == 0;
}

static void receive_event(const struct ble_hci_ev *event) {
  // Validate the controller's actual BR/EDR encryption key size before HFP.
  if (event->opcode == 0x08 && event->length == 4) {
    const uint8_t *p = (const uint8_t *)(event + 1);
    if (!p[0] && p[3]) {
      uint8_t response[3];
      int rc = ble_hs_hci_cmd_tx(0x1408, p + 1, 2, response, sizeof(response));
      if (rc || memcmp(response, p + 1, 2) || response[2] != 16) {
        const uint8_t failed[] = {4, 0x08, 4, 0x06, p[1], p[2], 0};
        bt_classic_receive(hfp_service_host(), failed, sizeof(failed));
        return;
      }
    }
  }
  uint8_t packet[258] = {4};
  memcpy(packet + 1, event, event->length + 2);
  bt_classic_receive(hfp_service_host(), packet, event->length + 3);
  hfp_service_wake();
}

static void receive_acl(const struct os_mbuf *om) {
  uint8_t packet[BT_CLASSIC_MTU + 9] = {2};
  unsigned length = OS_MBUF_PKTLEN(om);
  if (length <= sizeof(packet) - 1 && !os_mbuf_copydata(om, 0, length, packet + 1)) {
    bt_classic_receive(hfp_service_host(), packet, length + 1);
    hfp_service_wake();
  }
}

static void reset(void) {
  s_running = false;
  hci_local_audio_stop();
  ble_npl_callout_stop(&s_poll);
  bt_classic_reset(hfp_service_host());
  hfp_service_poll(pbl_ticks_to_ms(pbl_uptime_ticks()));
  if (s_stopping) {
    s_stopping = false;
    pbl_sem_give(&s_stopped);
  }
}

static void poll(struct ble_npl_event *event) {
  if (!s_running)
    return;
  BtClassicHost *host = hfp_service_host();
  hfp_service_poll(pbl_ticks_to_ms(pbl_uptime_ticks()));
  if (s_stopping && bt_classic_stopped(host)) {
    ble_hs_classic_reset();
    return;
  }
  bool pending = s_stopping || host->command_count || host->output_count || host->at_tx_length;
  ble_npl_callout_reset(&s_poll, ble_npl_time_ms_to_ticks32(pending ? 10 : 1000));
}

static void stop(struct ble_npl_event *event) {
  if (!s_running) {
    pbl_sem_give(&s_stopped);
    return;
  }
  s_stopping = true;
  bt_classic_stop(hfp_service_host());
  poll(NULL);
}

void nimble_classic_init(void) {
  static const struct ble_hs_classic_callbacks callbacks = {
    .event = receive_event,
    .acl = receive_acl,
    .reset = reset,
  };
  hfp_service_init();
  ble_npl_callout_init(&s_poll, nimble_port_get_dflt_eventq(), poll, NULL);
  ble_npl_event_init(&s_wake, poll, NULL);
  ble_npl_event_init(&s_stop, stop, NULL);
  ble_hs_classic_register(&callbacks);
}

void nimble_classic_start(void) {
  if (!ble_hs_classic_supported())
    return;
  bt_classic_init_managed(hfp_service_host(), send_command, send_acl, ble_hs_classic_acl_mtu(),
                          NULL);
  hfp_service_host()->get_link_key = nimble_store_get_classic_key;
  ble_hs_cfg.sm_ctkd = 1;
  ble_hs_cfg.sm_our_key_dist |= BLE_SM_PAIR_KEY_DIST_ID | BLE_SM_PAIR_KEY_DIST_LINK;
  ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ID | BLE_SM_PAIR_KEY_DIST_LINK;
  s_running = true;
  s_stopping = false;
  hfp_service_wake();
}

void nimble_classic_stop(void) {
  (void)pbl_sem_take(&s_stopped, PBL_NO_WAIT);
  ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_stop);
  PBL_ASSERTN(pbl_sem_take(&s_stopped, PBL_MSEC(10000)) == 0);
}

void hfp_service_wake(void) {
  ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_wake);
}

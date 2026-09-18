/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "service.h"
#include "../classic/service.h"
#include "../hci_bridge/transport.h"
#include "../hci_bridge/local_audio.h"
#include <bluetooth/init.h>
#include <comm/bt_lock.h>

static void send(const uint8_t *data, size_t length, void *context) {
  hci_bridge_transport_write(data, length);
}
void hfp_service_wake(void) {
  hci_bridge_transport_wake();
}
void classic_demo_service_init(void) {
  hfp_service_init();
  bt_classic_init(hfp_service_host(), send, NULL);
}
void classic_demo_service_poll(uint32_t now) {
  hfp_service_poll(now);
}
void hci_bridge_transport_receive(const uint8_t *data, size_t length) {
  if (hci_local_audio_receive(data, length))
    bt_classic_receive(hfp_service_host(), data, length);
}
void bt_driver_init(void) {
  bt_lock_init();
  hci_bridge_transport_init();
}
bool bt_driver_start(BTDriverConfig *config) {
  return true;
}
void bt_driver_stop(void) {
}
void bt_driver_power_down_controller_on_boot(void) {
}

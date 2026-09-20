/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "transport.h"
#ifdef CONFIG_BT_HCI_LOCAL_AUDIO
#include "local_audio.h"
#endif

#include <bluetooth/init.h>
#include <comm/bt_lock.h>
#include <console/pulse_protocol_impl.h>
#include <pbl/util/math.h>
#include <string.h>

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

void pulse_hci_bridge_packet_handler(void *packet, size_t length) {
  hci_bridge_transport_write(packet, length);
}

void hci_bridge_transport_receive(const uint8_t *data, size_t length) {
#ifdef CONFIG_BT_HCI_LOCAL_AUDIO
  if (!hci_local_audio_receive(data, length)) {
    return;
  }
#endif
  while (length) {
    uint8_t *buffer = pulse_reliable_send_begin(PULSE2_HCI_PROTOCOL);
    if (!buffer) {
      return;
    }
    size_t count = MIN(length, pulse_reliable_max_send_size());
    memcpy(buffer, data, count);
    pulse_reliable_send(buffer, count);
    data += count;
    length -= count;
  }
}

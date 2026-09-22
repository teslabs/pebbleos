/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <console/prompt.h>
#include <host/ble_hs.h>
#include <nimble/hci_common.h>
#include <stdio.h>

#define HEX_DUMP_BYTES_PER_LINE 16

void command_bt_controller_probe(void) {
  struct ble_hs_hci_local_info info;
  char line[128];

  if (ble_hs_hci_get_local_info(&info) != 0) {
    prompt_send_response("Controller unavailable: Bluetooth host is not synchronized");
    return;
  }

  prompt_send_response("Controller information as read by the host at startup; capability bits");
  prompt_send_response("do not prove working call audio");
  prompt_send_response_fmt(line, sizeof(line),
                           "version hci=%u revision=%u lmp=%u manufacturer=%u subversion=%u",
                           info.hci_version, info.hci_revision, info.lmp_version, info.manufacturer,
                           info.lmp_subversion);
  if (info.lmp_features) {
    const uint64_t features = info.lmp_features;
    prompt_send_response_fmt(
        line, sizeof(line), "features bredr=%u le=%u sco=%u esco=%u cvsd=%u transparent=%u",
        !(features & BLE_HCI_LMP_FEAT_BR_EDR_NOT_SUPPORTED),
        !!(features & BLE_HCI_LMP_FEAT_LE_SUPPORTED_CONTROLLER),
        !!(features & BLE_HCI_LMP_FEAT_SCO_LINK), !!(features & BLE_HCI_LMP_FEAT_EV3_PACKETS),
        !!(features & BLE_HCI_LMP_FEAT_CVSD_SYNC_DATA),
        !!(features & BLE_HCI_LMP_FEAT_TRANSPARENT_SYNC_DATA));
  } else {
    prompt_send_response("features not read by the host");
  }
  prompt_send_response_fmt(line, sizeof(line), "buffers acl_bytes=%u acl_packets=%u",
                           info.acl_data_len, info.acl_num_pkts);
  for (unsigned offset = 0; offset < sizeof(info.supported_commands);
       offset += HEX_DUMP_BYTES_PER_LINE) {
    char hex[2 * HEX_DUMP_BYTES_PER_LINE + 1];
    for (unsigned j = 0; j < HEX_DUMP_BYTES_PER_LINE; ++j) {
      snprintf(&hex[j * 2], 3, "%02x", info.supported_commands[offset + j]);
    }
    prompt_send_response_fmt(line, sizeof(line), "commands[%u]=%s", offset, hex);
  }
}

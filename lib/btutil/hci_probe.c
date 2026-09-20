/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/btutil/hci_probe.h"

#include <stdio.h>

static uint16_t prv_le16(const uint8_t *bytes) {
  return bytes[0] | ((uint16_t)bytes[1] << 8);
}

bool bt_hci_probe(BtHciProbeRead read, BtHciProbeOutput output, void *context) {
  static const struct {
    uint16_t opcode;
    uint8_t length;
    const char *name;
  } queries[] = {
    {0x1001, 8, "version"}, {0x1002, 64, "commands"}, {0x1003, 8, "features"},
    {0x1005, 7, "buffers"}, {0x0c25, 2, "voice"},
  };
  bool success = true;
  char line[128];

  output("HCI probe v1; read-only; capability bits do not prove working call audio", context);
  for (unsigned i = 0; i < sizeof(queries) / sizeof(queries[0]); ++i) {
    uint8_t response[64] = {0};
    const int rc = read(queries[i].opcode, response, queries[i].length, context);
    snprintf(line, sizeof(line), "hci %04x %s status=%d", queries[i].opcode, queries[i].name, rc);
    output(line, context);
    if (rc != 0) {
      success = false;
      if (rc < 0) {
        output("HCI probe aborted: backend failure", context);
        return false;
      }
      continue;
    }

    for (unsigned offset = 0; offset < queries[i].length; offset += 16) {
      char hex[33];
      unsigned count = queries[i].length - offset;
      if (count > 16) {
        count = 16;
      }
      for (unsigned j = 0; j < count; ++j) {
        snprintf(&hex[j * 2], 3, "%02x", response[offset + j]);
      }
      snprintf(line, sizeof(line), "hci %04x data[%u]=%s", queries[i].opcode, offset, hex);
      output(line, context);
    }

    switch (queries[i].opcode) {
      case 0x1001:
        snprintf(line, sizeof(line),
                 "version hci=%u revision=%u lmp=%u manufacturer=%u subversion=%u", response[0],
                 prv_le16(response + 1), response[3], prv_le16(response + 4),
                 prv_le16(response + 6));
        output(line, context);
        break;
      case 0x1003:
        snprintf(line, sizeof(line),
                 "features bredr=%u le=%u sco=%u esco=%u cvsd=%u transparent=%u",
                 !(response[4] & 0x20), !!(response[4] & 0x40), !!(response[1] & 0x08),
                 !!(response[3] & 0x80), !!(response[2] & 0x01), !!(response[2] & 0x08));
        output(line, context);
        break;
      case 0x1005:
        snprintf(line, sizeof(line),
                 "buffers acl_bytes=%u acl_packets=%u sco_bytes=%u sco_packets=%u",
                 prv_le16(response), prv_le16(response + 3), response[2], prv_le16(response + 5));
        output(line, context);
        break;
      case 0x0c25:
        snprintf(line, sizeof(line), "voice setting=0x%04x (does not identify the audio route)",
                 prv_le16(response));
        output(line, context);
        break;
    }
  }
  output(success ? "HCI probe complete" : "HCI probe incomplete: controller rejected queries",
         context);
  return success;
}

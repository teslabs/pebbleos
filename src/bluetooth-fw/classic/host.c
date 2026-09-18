/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "internal.h"

#include <stdio.h>
#include <string.h>

static uint16_t u16(const uint8_t *p) {
  return p[0] | (uint16_t)p[1] << 8;
}
static void put16(uint8_t *p, unsigned n) {
  p[0] = n;
  p[1] = n >> 8;
}

void bt_classic_error(BtClassicHost *s, const char *reason) {
  ++s->status.errors;
  snprintf(s->status.detail, sizeof(s->status.detail), "%s", reason);
}

static void command(BtClassicHost *s, uint16_t opcode, const void *data, unsigned length) {
  if (length > 255 || s->command_count == 8) {
    bt_classic_error(s, "HCI command queue full");
    s->status.available = false;
    return;
  }
  unsigned index = (s->command_head + s->command_count++) % 8;
  s->commands[index].length = length + 4;
  uint8_t *p = s->commands[index].data;
  p[0] = 1;
  put16(p + 1, opcode);
  p[3] = length;
  if (length)
    memcpy(p + 4, data, length);
}

static void startup(BtClassicHost *s) {
  uint8_t data[248] = {0};
  switch (s->startup++) {
    case 0:
      command(s, 0x0c03, NULL, 0);
      break;
    case 1:
      memset(data, 0xff, 8);
      command(s, 0x0c01, data, 8);
      break;
    case 2:
      data[0] = 1;
      command(s, 0x0c56, data, 1);
      break;
    case 3:
      data[0] = 8;
      data[1] = 4;
      data[2] = 0x20;
      command(s, 0x0c24, data, 3);
      break;
    case 4:
      memcpy(data, "Pebble HFP Demo", 15);
      command(s, 0x0c13, data, sizeof(data));
      break;
    case 5:
      data[1] = 0x60;
      command(s, 0x0c18, data, 2);
      break;
    case 6:
      data[0] = 0x60;
      command(s, 0x0c26, data, 2);
      break;
    case 7:
      if (s->send_acl) {
        startup(s);
        break;
      }
      command(s, 0x1005, NULL, 0);
      break;
    case 8:
      data[0] = 1;
      command(s, 0x0c2f, data, 1);
      break;
    case 9:
      data[1] = 16;
      data[2] = 9;
      memcpy(data + 3, "Pebble HFP Demo", 15);
      data[18] = 3;
      data[19] = 3;
      data[20] = 0x1e;
      data[21] = 0x11;
      command(s, 0x0c52, data, 241);
      break;
    case 10:
      data[0] = 3;
      command(s, 0x0c1a, data, 1);
      break;
    default:
      s->startup = 0xff;
      s->status.available = true;
      snprintf(
          s->status.detail, sizeof(s->status.detail), "%s",
          s->get_link_key ? "Pair over BLE to enable calls" : "Pair in phone Bluetooth settings");
      break;
  }
}

void bt_classic_init(BtClassicHost *s, void (*send)(const uint8_t *, size_t, void *),
                     void *context) {
  *s = (BtClassicHost){
    .send = send,
    .context = context,
    .command_credit = 1,
    .handle = BT_CLASSIC_NO_HANDLE,
    .sco_handle = BT_CLASSIC_NO_HANDLE
  };
  snprintf(s->status.detail, sizeof(s->status.detail), "Starting Bluetooth");
  startup(s);
}

void bt_classic_init_managed(BtClassicHost *s, void (*send)(const uint8_t *, size_t, void *),
                             bool (*acl)(const uint8_t *, size_t, void *), uint16_t acl_mtu,
                             void *context) {
  *s = (BtClassicHost){
    .send = send,
    .send_acl = acl,
    .context = context,
    .command_credit = 1,
    .startup = 2,
    .acl_mtu = acl_mtu > BT_CLASSIC_MTU + 4 ? BT_CLASSIC_MTU + 4 : acl_mtu,
    .handle = BT_CLASSIC_NO_HANDLE,
    .sco_handle = BT_CLASSIC_NO_HANDLE
  };
  startup(s);
}

void bt_classic_reset(BtClassicHost *s) {
  *s = (BtClassicHost){.handle = BT_CLASSIC_NO_HANDLE, .sco_handle = BT_CLASSIC_NO_HANDLE};
}

static void disconnect(BtClassicHost *s) {
  s->handle = s->sco_handle = BT_CLASSIC_NO_HANDLE;
  s->accepting = s->accepting_sco = s->encrypted = s->revoking = false;
  s->acl_inflight = s->receive_length = s->receive_needed = 0;
  s->output_count = s->output_head = 0;
  memset(s->channels, 0, sizeof(s->channels));
  bt_classic_profile_reset(s);
  s->status.connected = s->status.audio = s->status.call = s->status.incoming = false;
  s->status.call_setup = 0;
  snprintf(s->status.detail, sizeof(s->status.detail), "Disconnected; reconnect from phone");
  const uint8_t scan = s->stopping ? 0 : 3;
  command(s, 0x0c1a, &scan, 1);
}

static void disconnect_link(BtClassicHost *s, uint16_t handle) {
  if (handle == BT_CLASSIC_NO_HANDLE)
    return;
  uint8_t data[3] = {0, 0, 0x13};
  put16(data, handle);
  command(s, 0x0406, data, sizeof(data));
}

void bt_classic_disconnect_peer(BtClassicHost *s) {
  if (s->revoking)
    return;
  s->revoking = true;
  s->encrypted = false;
  bt_classic_profile_reset(s);
  s->output_count = 0;
  disconnect_link(s, s->sco_handle);
  disconnect_link(s, s->handle);
}

void bt_classic_stop(BtClassicHost *s) {
  bool cancel_accept = false;
  for (unsigned i = 0; i < s->command_count; ++i) {
    unsigned opcode = u16(s->commands[(s->command_head + i) % 8].data + 1);
    cancel_accept |= opcode == 0x0409;
    if (opcode == 0x0429)
      s->accepting_sco = false;
  }
  s->stopping = true;
  s->startup = 0xff;
  s->status.available = false;
  bt_classic_profile_reset(s);
  s->command_count = s->output_count = 0;
  const uint8_t scan = 0;
  command(s, 0x0c1a, &scan, 1);
  if (cancel_accept) {
    uint8_t data[7];
    memcpy(data, s->peer, 6);
    data[6] = 0x0d;
    command(s, 0x040a, data, sizeof(data));
    s->accepting = false;
  }
  disconnect_link(s, s->sco_handle);
  disconnect_link(s, s->handle);
}

bool bt_classic_stopped(const BtClassicHost *s) {
  return s->stopping && !s->accepting && !s->accepting_sco && s->handle == BT_CLASSIC_NO_HANDLE &&
         s->sco_handle == BT_CLASSIC_NO_HANDLE && !s->command_count && !s->pending_opcode;
}

void bt_classic_poll(BtClassicHost *s, uint32_t now) {
  s->now = now;
  if (s->get_link_key && s->encrypted && !s->revoking && !s->stopping &&
      !s->get_link_key(s->peer, NULL, s->context)) {
    bt_classic_disconnect_peer(s);
  }
  if (s->pending_opcode && (int32_t)(now - s->command_deadline) >= 0) {
    bt_classic_error(s, "Controller command timed out; restart");
    s->status.available = s->status.ready = false;
    s->command_count = 0;
    s->pending_opcode = 0;
    s->command_credit = 0;
  }
  if (!s->pending_opcode && s->command_credit && s->command_count) {
    unsigned i = s->command_head;
    s->pending_opcode = u16(s->commands[i].data + 1);
    s->command_deadline = now + 10000;
    --s->command_credit;
    s->command_head = (i + 1) % 8;
    --s->command_count;
    s->send(s->commands[i].data, s->commands[i].length, s->context);
  }
  bt_classic_profile_poll(s);
  while (s->output_count && (s->send_acl || s->acl_inflight < s->acl_limit)) {
    BtClassicPacket *p = &s->output[s->output_head];
    if (s->send_acl) {
      if (!s->send_acl(p->data, p->length, s->context))
        break;
    } else {
      ++s->acl_inflight;
      s->send(p->data, p->length, s->context);
    }
    s->output_head = (s->output_head + 1) % 12;
    --s->output_count;
  }
}

void bt_classic_receive(BtClassicHost *s, const uint8_t *p, size_t n) {
  if (n && p[0] == 2) {
    if (!s->stopping)
      bt_classic_l2cap_receive(s, p, n);
    return;
  }
  if (n < 3 || p[0] != 4 || n != p[2] + 3u)
    return;
  uint8_t event = p[1];
  p += 3;
  n -= 3;
  uint8_t data[32] = {0};
  if ((event == 0x0e && n >= 4) || (event == 0x0f && n == 4)) {
    unsigned opcode = event == 0x0e ? u16(p + 1) : u16(p + 2);
    unsigned status = event == 0x0e ? p[3] : p[0];
    s->command_credit = event == 0x0e ? p[0] : p[1];
    if (opcode != s->pending_opcode)
      return;
    s->pending_opcode = 0;
    if (status) {
      if (opcode == 0x0409)
        s->accepting = false;
      if (opcode == 0x0429)
        s->accepting_sco = false;
      char text[64];
      snprintf(text, sizeof(text), "HCI %04x failed: %02x", opcode, status);
      bt_classic_error(s, text);
      if (s->startup != 0xff) {
        s->command_count = 0;
        s->startup = 0xff;
        return;
      }
    }
    if (!status && opcode == 0x1005 && n == 11) {
      s->acl_mtu = u16(p + 4);
      if (s->acl_mtu > BT_CLASSIC_MTU + 4)
        s->acl_mtu = BT_CLASSIC_MTU + 4;
      s->acl_limit = u16(p + 7);
    }
    if (s->startup != 0xff)
      startup(s);
  } else if (event == 0x04 && n == 10) {
    if (p[9] == 1) {
      memcpy(data, p, 6);
      if (s->stopping || s->handle != BT_CLASSIC_NO_HANDLE || s->accepting) {
        data[6] = 0x0d;
        command(s, 0x040a, data, 7);
      } else {
        memcpy(s->peer, p, 6);
        s->accepting = true;
        data[6] = 1;
        command(s, 0x0409, data, 7);
      }
    } else if ((p[9] == 0 || p[9] == 2) && s->status.ready && !memcmp(p, s->peer, 6)) {
      memcpy(data, p, 6);
      put16(data + 6, 8000);
      put16(data + 10, 8000);
      put16(data + 14, p[9] == 2 ? 7 : 0xffff);
      put16(data + 16, 0x60);
      data[18] = p[9] == 2 ? 1 : 0;
      put16(data + 19, p[9] == 2 ? 0x03c8 : 4);
      s->accepting_sco = true;
      command(s, 0x0429, data, 21);
    } else {
      memcpy(data, p, 6);
      data[6] = 0x0d;
      command(s, 0x042a, data, 7);
    }
  } else if (event == 3 && n == 11 && p[9] == 1) {
    s->accepting = false;
    if (!p[0]) {
      s->handle = u16(p + 1);
      memcpy(s->peer, p + 3, 6);
      s->status.connected = true;
      snprintf(s->status.detail, sizeof(s->status.detail), "Phone connected; starting HFP");
      data[0] = 0;
      command(s, 0x0c1a, data, 1);
      if (s->stopping) {
        disconnect_link(s, s->handle);
      } else if (s->get_link_key && s->get_link_key(s->peer, NULL, s->context)) {
        put16(data, s->handle);
        command(s, 0x0411, data, 2);
      }
    } else
      bt_classic_error(s, "Phone connection failed");
  } else if (event == 5 && n == 4 && !p[0]) {
    if (u16(p + 1) == s->handle)
      disconnect(s);
    else if (u16(p + 1) == s->sco_handle) {
      s->status.audio = false;
      s->sco_handle = BT_CLASSIC_NO_HANDLE;
    }
  } else if (event == 0x2c && n == 17) {
    s->accepting_sco = false;
    if (!p[0]) {
      s->sco_handle = u16(p + 1);
      s->status.audio = true;
      if (s->stopping)
        disconnect_link(s, s->sco_handle);
    } else
      bt_classic_error(s, "Call audio connection failed");
  } else if (event == 0x06 && n == 3 && u16(p + 1) == s->handle && s->get_link_key) {
    if (p[0]) {
      disconnect_link(s, s->handle);
    } else {
      put16(data, s->handle);
      data[2] = 1;
      command(s, 0x0413, data, 3);
    }
  } else if (event == 0x08 && n == 4 && u16(p + 1) == s->handle && s->get_link_key) {
    s->encrypted = !p[0] && p[3] && s->get_link_key(s->peer, NULL, s->context);
    if (!s->encrypted)
      disconnect_link(s, s->handle);
  } else if (event == 0x31 && n == 6) {
    memcpy(data, p, 6);
    if (s->get_link_key) {
      data[6] = 0x18; // Pairing not allowed: use the shared BLE bond.
      command(s, 0x0434, data, 7);
      snprintf(s->status.detail, sizeof(s->status.detail), "Pair over BLE to enable calls");
    } else {
      data[6] = 3;
      data[8] = 2;
      command(s, 0x042b, data, 9);
    } // NoInputNoOutput, general bonding without MITM.
  } else if (event == 0x33 && n == 10) {
    command(s, s->get_link_key ? 0x042d : 0x042c, p,
            6); // Just Works; no numeric-comparison capability advertised.
  } else if (event == 0x17 && n == 6) {
    memcpy(data, p, 6);
    if (s->get_link_key) {
      bool found = s->get_link_key(p, data + 6, s->context);
      command(s, found ? 0x040b : 0x040c, data, found ? 22 : 6);
    } else if (s->key_valid && !memcmp(p, s->key_peer, 6)) {
      memcpy(data + 6, s->key, 16);
      command(s, 0x040b, data, 22);
    } else
      command(s, 0x040c, data, 6);
  } else if (event == 0x18 && n == 23 && !s->get_link_key) {
    memcpy(s->key_peer, p, 6);
    memcpy(s->key, p + 6, 16);
    s->key_valid = true;
  } else if (event == 0x16 && n == 6)
    command(s, 0x040e, p, 6);
  else if (event == 0x13 && n >= 1 && n == 1u + 4u * p[0]) {
    for (unsigned i = 1; i < n; i += 4)
      if (u16(p + i) == s->handle) {
        unsigned count = u16(p + i + 2);
        s->acl_inflight = count > s->acl_inflight ? 0 : s->acl_inflight - count;
      }
  } else if (event == 0x10) {
    s->status.ready = s->status.available = false;
    bt_classic_error(s, "Controller error; restart watch");
  }
}

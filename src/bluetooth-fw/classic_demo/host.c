/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "host.h"

#include <stdio.h>
#include <string.h>

static uint16_t u16(const uint8_t *p) {
  return p[0] | (uint16_t)p[1] << 8;
}
static void put16(uint8_t *p, unsigned n) {
  p[0] = n;
  p[1] = n >> 8;
}

void classic_demo_error(ClassicDemoHost *s, const char *reason) {
  ++s->status.errors;
  snprintf(s->status.detail, sizeof(s->status.detail), "%s", reason);
}

static void command(ClassicDemoHost *s, uint16_t opcode, const void *data, unsigned length) {
  if (length > 255 || s->command_count == 8) {
    classic_demo_error(s, "HCI command queue full");
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

static void startup(ClassicDemoHost *s) {
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
      snprintf(s->status.detail, sizeof(s->status.detail), "Pair in phone Bluetooth settings");
      break;
  }
}

void classic_demo_init(ClassicDemoHost *s, void (*send)(const uint8_t *, size_t, void *),
                       void *context) {
  *s = (ClassicDemoHost){
    .send = send,
    .context = context,
    .command_credit = 1,
    .handle = CLASSIC_DEMO_NO_HANDLE,
    .sco_handle = CLASSIC_DEMO_NO_HANDLE
  };
  snprintf(s->status.detail, sizeof(s->status.detail), "Starting Bluetooth");
  startup(s);
}

static void disconnect(ClassicDemoHost *s) {
  s->handle = s->sco_handle = CLASSIC_DEMO_NO_HANDLE;
  s->accepting = false;
  s->acl_inflight = s->receive_length = s->receive_needed = 0;
  s->output_count = s->output_head = 0;
  memset(s->channels, 0, sizeof(s->channels));
  classic_demo_profile_reset(s);
  s->status.connected = s->status.audio = s->status.call = s->status.incoming = false;
  s->status.call_setup = 0;
  snprintf(s->status.detail, sizeof(s->status.detail), "Disconnected; reconnect from phone");
  const uint8_t scan = 3;
  command(s, 0x0c1a, &scan, 1);
}

void classic_demo_poll(ClassicDemoHost *s, uint32_t now) {
  s->now = now;
  if (s->pending_opcode && (int32_t)(now - s->command_deadline) >= 0) {
    classic_demo_error(s, "Controller command timed out; restart");
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
  classic_demo_profile_poll(s);
  while (s->output_count && s->acl_inflight < s->acl_limit) {
    ClassicDemoPacket *p = &s->output[s->output_head];
    ++s->acl_inflight;
    s->output_head = (s->output_head + 1) % 12;
    --s->output_count;
    s->send(p->data, p->length, s->context);
  }
}

void classic_demo_l2cap_send(ClassicDemoHost *s, uint16_t cid, const uint8_t *data, size_t length) {
  if (s->handle == CLASSIC_DEMO_NO_HANDLE || !s->acl_mtu || length > CLASSIC_DEMO_MTU)
    return;
  uint8_t pdu[CLASSIC_DEMO_MTU + 4];
  put16(pdu, length);
  put16(pdu + 2, cid);
  if (length)
    memcpy(pdu + 4, data, length);
  unsigned total = length + 4;
  unsigned fragments = (total + s->acl_mtu - 1) / s->acl_mtu;
  if (fragments > 12 - s->output_count) {
    classic_demo_error(s, "ACL queue full");
    return;
  }
  for (unsigned offset = 0; offset < total;) {
    unsigned n = total - offset;
    if (n > s->acl_mtu)
      n = s->acl_mtu;
    ClassicDemoPacket *out = &s->output[(s->output_head + s->output_count++) % 12];
    out->data[0] = 2;
    put16(out->data + 1, s->handle | (offset ? 0x1000 : 0x2000));
    put16(out->data + 3, n);
    memcpy(out->data + 5, pdu + offset, n);
    out->length = n + 5;
    offset += n;
  }
}

static void signal_send(ClassicDemoHost *s, uint8_t code, uint8_t id, const void *data,
                        unsigned n) {
  uint8_t p[40] = {code, id};
  if (n > sizeof(p) - 4)
    return;
  put16(p + 2, n);
  if (n)
    memcpy(p + 4, data, n);
  classic_demo_l2cap_send(s, 1, p, n + 4);
}

static ClassicDemoChannel *find_channel(ClassicDemoHost *s, uint16_t cid) {
  for (unsigned i = 0; i < 4; ++i)
    if (s->channels[i].local == cid && cid)
      return &s->channels[i];
  return NULL;
}

static void signaling(ClassicDemoHost *s, const uint8_t *p, unsigned size) {
  while (size >= 4) {
    unsigned n = u16(p + 2);
    uint8_t code = p[0], id = p[1];
    p += 4;
    size -= 4;
    if (!id || n > size)
      return;
    uint8_t response[32] = {0};
    if (code == 2 && n == 4) {
      uint16_t psm = u16(p), remote = u16(p + 2);
      ClassicDemoChannel *ch = NULL;
      bool duplicate = false;
      for (unsigned i = 0; i < 4; ++i) {
        if (s->channels[i].local && s->channels[i].remote == remote)
          duplicate = true;
        if (!ch && !s->channels[i].local)
          ch = &s->channels[i];
      }
      unsigned result = (psm != 1 && psm != 3) ? 2 : (!ch || remote < 0x40 || duplicate) ? 4 : 0;
      if (!result) {
        unsigned local = 0x40 + (ch - s->channels);
        *ch = (ClassicDemoChannel){.local = local, .remote = remote, .psm = psm, .mtu = 672};
        put16(response, local);
      }
      put16(response + 2, remote);
      put16(response + 4, result);
      signal_send(s, 3, id, response, 8);
      if (!result) {
        put16(response, remote);
        put16(response + 2, 0);
        response[4] = 1;
        response[5] = 2;
        put16(response + 6, CLASSIC_DEMO_MTU);
        if (!++s->signal_id)
          ++s->signal_id;
        ch->config_id = s->signal_id;
        signal_send(s, 4, ch->config_id, response, 8);
      }
    } else if (code == 4 && n >= 4) {
      ClassicDemoChannel *ch = find_channel(s, u16(p));
      if (ch) {
        unsigned result = u16(p + 2) ? 2 : 0;
        for (unsigned off = 4; off < n;) {
          if (off + 2 > n || off + 2u + p[off + 1] > n) {
            result = 2;
            break;
          }
          unsigned type = p[off] & 0x7f, len = p[off + 1];
          if (type == 1 && len == 2) {
            ch->mtu = u16(p + off + 2);
            if (ch->mtu < 48)
              result = 1;
          } else if (type == 4 && len == 9 && p[off + 2] != 0) {
            result = 1; // Basic mode only.
          } else if (type != 2 && type != 3 && type != 4 && !(p[off] & 0x80))
            result = 3;
          off += 2 + len;
        }
        put16(response, ch->remote);
        put16(response + 4, result);
        signal_send(s, 5, id, response, 6);
        ch->peer_configured = result == 0;
      }
    } else if (code == 5 && n >= 6) {
      ClassicDemoChannel *ch = find_channel(s, u16(p));
      if (ch && id == ch->config_id && u16(p + 2) == 0)
        ch->configured = u16(p + 4) == 0;
    } else if (code == 6 && n == 4) {
      ClassicDemoChannel *ch = find_channel(s, u16(p));
      if (ch && ch->remote == u16(p + 2)) {
        signal_send(s, 7, id, p, 4);
        if (ch->psm == 3)
          classic_demo_profile_reset(s);
        *ch = (ClassicDemoChannel){0};
      }
    } else if (code == 8) {
      if (n <= 32)
        signal_send(s, 9, id, p, n);
    } else if (code == 10 && n == 2) {
      unsigned type = u16(p);
      put16(response, type);
      unsigned bytes = 4;
      if (type == 1) {
        put16(response + 4, CLASSIC_DEMO_MTU);
        bytes = 6;
      } else if (type == 2) {
        response[4] = 0x80;
        bytes = 8;
      } else if (type == 3) {
        response[4] = 2;
        bytes = 12;
      } else
        put16(response + 2, 1);
      signal_send(s, 11, id, response, bytes);
    } else if (code != 1 && code != 3 && code != 7 && code != 9 && code != 11) {
      signal_send(s, 1, id, response, 2);
    }
    p += n;
    size -= n;
  }
}

static void acl(ClassicDemoHost *s, const uint8_t *p, unsigned n) {
  if (n < 5 || u16(p + 3) != n - 5 || (u16(p + 1) & 0x0fff) != s->handle)
    return;
  unsigned pb = (p[2] >> 4) & 3;
  p += 5;
  n -= 5;
  if (pb == 0 || pb == 2) {
    s->receive_length = s->receive_needed = 0;
    if (n < 4 || u16(p) > CLASSIC_DEMO_MTU)
      return;
    s->receive_needed = u16(p) + 4;
  } else if (pb != 1 || !s->receive_needed)
    return;
  if (s->receive_length + n > s->receive_needed) {
    s->receive_length = s->receive_needed = 0;
    return;
  }
  memcpy(s->receive + s->receive_length, p, n);
  s->receive_length += n;
  if (s->receive_length != s->receive_needed)
    return;
  uint16_t cid = u16(s->receive + 2);
  n = s->receive_needed - 4;
  p = s->receive + 4;
  s->receive_length = s->receive_needed = 0;
  if (cid == 1)
    signaling(s, p, n);
  else {
    ClassicDemoChannel *ch = find_channel(s, cid);
    if (!ch || !ch->configured || !ch->peer_configured)
      return;
    if (ch->psm == 1)
      classic_demo_sdp(s, ch, p, n);
    else if (ch->psm == 3)
      classic_demo_rfcomm(s, ch, p, n);
  }
}

void classic_demo_receive(ClassicDemoHost *s, const uint8_t *p, size_t n) {
  if (n && p[0] == 2) {
    acl(s, p, n);
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
      char text[64];
      snprintf(text, sizeof(text), "HCI %04x failed: %02x", opcode, status);
      classic_demo_error(s, text);
      if (s->startup != 0xff) {
        s->command_count = 0;
        s->startup = 0xff;
        return;
      }
    }
    if (!status && opcode == 0x1005 && n == 11) {
      s->acl_mtu = u16(p + 4);
      if (s->acl_mtu > CLASSIC_DEMO_MTU + 4)
        s->acl_mtu = CLASSIC_DEMO_MTU + 4;
      s->acl_limit = u16(p + 7);
    }
    if (s->startup != 0xff)
      startup(s);
  } else if (event == 0x04 && n == 10) {
    if (p[9] == 1) {
      memcpy(data, p, 6);
      if (s->handle != CLASSIC_DEMO_NO_HANDLE || s->accepting) {
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
    } else
      classic_demo_error(s, "Phone connection failed");
  } else if (event == 5 && n == 4 && !p[0]) {
    if (u16(p + 1) == s->handle)
      disconnect(s);
    else if (u16(p + 1) == s->sco_handle) {
      s->status.audio = false;
      s->sco_handle = CLASSIC_DEMO_NO_HANDLE;
    }
  } else if (event == 0x2c && n == 17) {
    if (!p[0]) {
      s->sco_handle = u16(p + 1);
      s->status.audio = true;
    } else
      classic_demo_error(s, "Call audio connection failed");
  } else if (event == 0x31 && n == 6) {
    memcpy(data, p, 6);
    data[6] = 3;
    data[8] = 2;
    command(s, 0x042b, data, 9); // NoInputNoOutput, general bonding without MITM.
  } else if (event == 0x33 && n == 10) {
    command(s, 0x042c, p, 6); // Just Works; no numeric-comparison capability advertised.
  } else if (event == 0x17 && n == 6) {
    memcpy(data, p, 6);
    if (s->key_valid && !memcmp(p, s->key_peer, 6)) {
      memcpy(data + 6, s->key, 16);
      command(s, 0x040b, data, 22);
    } else
      command(s, 0x040c, data, 6);
  } else if (event == 0x18 && n == 23) {
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
    classic_demo_error(s, "Controller error; restart watch");
  }
}

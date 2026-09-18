/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "internal.h"
#include <string.h>

static uint16_t u16(const uint8_t *p) {
  return p[0] | (uint16_t)p[1] << 8;
}
static void put16(uint8_t *p, unsigned n) {
  p[0] = n;
  p[1] = n >> 8;
}

bool bt_classic_l2cap_send(BtClassicHost *s, uint16_t cid, const uint8_t *data, size_t length) {
  if (s->revoking || s->handle == BT_CLASSIC_NO_HANDLE || !s->acl_mtu || length > BT_CLASSIC_MTU)
    return false;
  uint8_t pdu[BT_CLASSIC_MTU + 4];
  put16(pdu, length);
  put16(pdu + 2, cid);
  if (length)
    memcpy(pdu + 4, data, length);
  unsigned total = length + 4;
  unsigned fragments = (total + s->acl_mtu - 1) / s->acl_mtu;
  if (fragments > 12 - s->output_count) {
    bt_classic_error(s, "ACL queue full");
    bt_classic_disconnect_peer(s);
    return false;
  }
  for (unsigned offset = 0; offset < total;) {
    unsigned n = total - offset;
    if (n > s->acl_mtu)
      n = s->acl_mtu;
    BtClassicPacket *out = &s->output[(s->output_head + s->output_count++) % 12];
    out->data[0] = 2;
    put16(out->data + 1, s->handle | (offset ? 0x1000 : 0x2000));
    put16(out->data + 3, n);
    memcpy(out->data + 5, pdu + offset, n);
    out->length = n + 5;
    offset += n;
  }
  return true;
}

static void signal_send(BtClassicHost *s, uint8_t code, uint8_t id, const void *data, unsigned n) {
  uint8_t p[40] = {code, id};
  if (n > sizeof(p) - 4)
    return;
  put16(p + 2, n);
  if (n)
    memcpy(p + 4, data, n);
  bt_classic_l2cap_send(s, 1, p, n + 4);
}

static BtClassicChannel *find_channel(BtClassicHost *s, uint16_t cid) {
  for (unsigned i = 0; i < 4; ++i)
    if (s->channels[i].local == cid && cid)
      return &s->channels[i];
  return NULL;
}

static void signaling(BtClassicHost *s, const uint8_t *p, unsigned size) {
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
      BtClassicChannel *ch = NULL;
      bool duplicate = false;
      for (unsigned i = 0; i < 4; ++i) {
        if (s->channels[i].local && s->channels[i].remote == remote)
          duplicate = true;
        if (!ch && !s->channels[i].local)
          ch = &s->channels[i];
      }
      unsigned result = (psm != 1 && psm != 3) ? 2 : (!ch || remote < 0x40 || duplicate) ? 4 : 0;
      if (psm == 3 && s->get_link_key && !s->encrypted)
        result = 3; // Security block; SDP remains available before authentication.
      if (!result) {
        unsigned local = 0x40 + (ch - s->channels);
        *ch = (BtClassicChannel){.local = local, .remote = remote, .psm = psm, .mtu = 672};
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
        put16(response + 6, BT_CLASSIC_MTU);
        if (!++s->signal_id)
          ++s->signal_id;
        ch->config_id = s->signal_id;
        signal_send(s, 4, ch->config_id, response, 8);
      }
    } else if (code == 4 && n >= 4) {
      BtClassicChannel *ch = find_channel(s, u16(p));
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
      BtClassicChannel *ch = find_channel(s, u16(p));
      if (ch && id == ch->config_id && u16(p + 2) == 0)
        ch->configured = u16(p + 4) == 0;
    } else if (code == 6 && n == 4) {
      BtClassicChannel *ch = find_channel(s, u16(p));
      if (ch && ch->remote == u16(p + 2)) {
        signal_send(s, 7, id, p, 4);
        if (ch->psm == 3)
          bt_classic_profile_reset(s);
        *ch = (BtClassicChannel){0};
      }
    } else if (code == 8) {
      if (n <= 32)
        signal_send(s, 9, id, p, n);
    } else if (code == 10 && n == 2) {
      unsigned type = u16(p);
      put16(response, type);
      unsigned bytes = 4;
      if (type == 1) {
        put16(response + 4, BT_CLASSIC_MTU);
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

void bt_classic_l2cap_receive(BtClassicHost *s, const uint8_t *p, unsigned n) {
  if (n < 5 || u16(p + 3) != n - 5 || (u16(p + 1) & 0x0fff) != s->handle)
    return;
  unsigned pb = (p[2] >> 4) & 3;
  p += 5;
  n -= 5;
  if (pb == 0 || pb == 2) {
    s->receive_length = s->receive_needed = 0;
    if (n < 4 || u16(p) > BT_CLASSIC_MTU)
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
    BtClassicChannel *ch = find_channel(s, cid);
    if (!ch || !ch->configured || !ch->peer_configured)
      return;
    if (ch->psm == 1)
      bt_classic_sdp(s, ch, p, n);
    else if (ch->psm == 3 && (!s->get_link_key || s->encrypted))
      bt_classic_rfcomm(s, ch, p, n);
  }
}

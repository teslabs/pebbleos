/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "internal.h"

#include <string.h>

static unsigned be16(const uint8_t *p) {
  return (unsigned)p[0] << 8 | p[1];
}
static void put16(uint8_t *p, unsigned n) {
  p[0] = n >> 8;
  p[1] = n;
}

typedef struct {
  const uint8_t *p;
  unsigned length, total, type;
} Element;

static bool element(const uint8_t *p, unsigned n, Element *e) {
  if (!n)
    return false;
  unsigned descriptor = p[0] & 7, header = 1, length;
  if (descriptor < 5)
    length = 1u << descriptor;
  else {
    unsigned bytes = 1u << (descriptor - 5);
    if (n < 1 + bytes)
      return false;
    length = 0;
    for (unsigned i = 0; i < bytes; ++i)
      length = (length << 8) | p[1 + i];
    header += bytes;
  }
  if (length > n - header)
    return false;
  *e = (Element){.p = p + header, .length = length, .total = header + length, .type = p[0] >> 3};
  return true;
}

static bool search_matches(const Element *sequence) {
  const uint8_t *p = sequence->p;
  unsigned n = sequence->length;
  if (sequence->type != 6 || !n)
    return false;
  while (n) {
    Element e;
    if (!element(p, n, &e) || e.type != 3)
      return false;
    unsigned uuid;
    if (e.length == 2)
      uuid = be16(e.p);
    else if (e.length == 4 && e.p[0] == 0 && e.p[1] == 0)
      uuid = be16(e.p + 2);
    else if (e.length == 16) {
      static const uint8_t base[] = {0, 0, 0x10, 0, 0x80, 0, 0, 0x80, 0x5f, 0x9b, 0x34, 0xfb};
      if (e.p[0] || e.p[1] || memcmp(e.p + 4, base, sizeof(base)))
        return false;
      uuid = be16(e.p + 2);
    } else
      return false;
    if (uuid != 0x111e && uuid != 0x1203 && uuid != 0x0100 && uuid != 3 && uuid != 0x1002)
      return false;
    p += e.total;
    n -= e.total;
  }
  return true;
}

static bool attr_requested(const Element *sequence, unsigned attr) {
  const uint8_t *p = sequence->p;
  unsigned n = sequence->length;
  if (sequence->type != 6)
    return false;
  while (n) {
    Element e;
    if (!element(p, n, &e) || e.type != 1)
      return false;
    if (e.length == 2 && be16(e.p) == attr)
      return true;
    if (e.length == 4 && be16(e.p) <= attr && attr <= be16(e.p + 2))
      return true;
    p += e.total;
    n -= e.total;
  }
  return false;
}

static const struct {
  uint16_t id;
  uint8_t length;
  uint8_t data[20];
} attributes[] = {
  {0x0000, 5, {0x0a, 0, 1, 0, 1}},
  {0x0001, 8, {0x35, 6, 0x19, 0x11, 0x1e, 0x19, 0x12, 3}},
  {0x0004, 14, {0x35, 12, 0x35, 3, 0x19, 1, 0, 0x35, 5, 0x19, 0, 3, 8, 1}},
  {0x0005, 5, {0x35, 3, 0x19, 0x10, 2}},
  {0x0009, 10, {0x35, 8, 0x35, 6, 0x19, 0x11, 0x1e, 9, 1, 7}},
  {0x0100, 10, {0x25, 8, 'H', 'F', 'P', ' ', 'D', 'e', 'm', 'o'}},
  {0x0311, 3, {9, 0, 0}},
};

static void respond(BtClassicHost *s, BtClassicChannel *ch, uint8_t type, const uint8_t *id,
                    const uint8_t *data, unsigned n) {
  uint8_t p[BT_CLASSIC_MTU];
  if (n + 5 > sizeof(p) || n + 5 > ch->mtu)
    return;
  p[0] = type;
  memcpy(p + 1, id, 2);
  put16(p + 3, n);
  memcpy(p + 5, data, n);
  bt_classic_l2cap_send(s, ch->remote, p, n + 5);
}

static void error(BtClassicHost *s, BtClassicChannel *ch, const uint8_t *id, unsigned code) {
  uint8_t p[2];
  put16(p, code);
  respond(s, ch, 1, id, p, 2);
}

static unsigned short_uuid(const Element *e) {
  static const uint8_t base[] = {0, 0, 0x10, 0, 0x80, 0, 0, 0x80, 0x5f, 0x9b, 0x34, 0xfb};
  if (e->type != 3)
    return 0;
  if (e->length == 2)
    return be16(e->p);
  if (e->length == 4 && !e->p[0] && !e->p[1])
    return be16(e->p + 2);
  if (e->length == 16 && !e->p[0] && !e->p[1] && !memcmp(e->p + 4, base, sizeof(base)))
    return be16(e->p + 2);
  return 0;
}

static bool take(Element *sequence, Element *child) {
  if (sequence->type != 6 || !element(sequence->p, sequence->length, child))
    return false;
  sequence->p += child->total;
  sequence->length -= child->total;
  return true;
}

static unsigned server_channel(const uint8_t *p, unsigned n) {
  Element records, record;
  if (!element(p, n, &records) || records.type != 6 || records.total != n)
    return 0;
  while (records.length) {
    if (!take(&records, &record) || record.type != 6)
      return 0;
    while (record.length) {
      Element id, value, protocol, uuid, channel;
      if (!take(&record, &id) || id.type != 1 || id.length != 2 || !take(&record, &value))
        return 0;
      if (be16(id.p) != 4)
        continue;
      if (value.type != 6 || !take(&value, &protocol) || !take(&protocol, &uuid) ||
          short_uuid(&uuid) != 0x0100 || !take(&value, &protocol) || !take(&protocol, &uuid) ||
          short_uuid(&uuid) != 3 || !take(&protocol, &channel) || channel.type != 1 ||
          channel.length != 1 || !channel.p[0] || channel.p[0] > 30)
        return 0;
      return channel.p[0];
    }
  }
  return 0;
}

static void client_request(BtClassicHost *s, BtClassicChannel *ch, const uint8_t *continuation,
                           unsigned length) {
  if (length > 16 || ++s->sdp_rounds > 8 || ch->mtu < 48) {
    bt_classic_disconnect_peer(s);
    return;
  }
  uint8_t p[34] = {6, 0, 0, 0, 0, 0x35, 3, 0x19, 0x11, 0x1f, 0, 0, 0x35, 3, 9, 0, 4, 0};
  put16(p + 1, ++s->sdp_transaction);
  put16(p + 3, 13 + length);
  unsigned limit = sizeof(s->sdp_response) - s->sdp_length;
  if (limit > ch->mtu - 8u)
    limit = ch->mtu - 8;
  if (limit < 7) {
    bt_classic_disconnect_peer(s);
    return;
  }
  put16(p + 10, limit);
  p[17] = length;
  if (length)
    memcpy(p + 18, continuation, length);
  bt_classic_l2cap_send(s, ch->remote, p, 18 + length);
}

void bt_classic_sdp_start(BtClassicHost *s, BtClassicChannel *ch) {
  client_request(s, ch, NULL, 0);
}

static void client_response(BtClassicHost *s, BtClassicChannel *ch, const uint8_t *p, size_t n) {
  if (s->connect_stage != BtClassicConnectSdp || n < 5 || be16(p + 1) != s->sdp_transaction)
    return;
  if (p[0] != 7 || n < 8 || be16(p + 3) != n - 5)
    goto failed;
  unsigned length = be16(p + 5);
  if (length > n - 8 || length > sizeof(s->sdp_response) - s->sdp_length)
    goto failed;
  unsigned continuation = p[7 + length];
  if (continuation > 16 || n != 8u + length + continuation || (!length && continuation))
    goto failed;
  memcpy(s->sdp_response + s->sdp_length, p + 7, length);
  s->sdp_length += length;
  if (continuation) {
    client_request(s, ch, p + 8 + length, continuation);
    return;
  }
  s->server_channel = server_channel(s->sdp_response, s->sdp_length);
  if (!s->server_channel)
    goto failed;
  s->connect_stage = BtClassicConnectRfcomm;
  if (!bt_classic_l2cap_connect(s, 3))
    goto failed;
  return;
failed:
  bt_classic_error(s, "Phone HFP discovery failed");
  bt_classic_disconnect_peer(s);
}

void bt_classic_sdp(BtClassicHost *s, BtClassicChannel *ch, const uint8_t *p, size_t n) {
  if (ch->outgoing) {
    client_response(s, ch, p, n);
    return;
  }
  if (n < 5 || be16(p + 3) != n - 5)
    return;
  uint8_t type = p[0], id[2] = {p[1], p[2]};
  p += 5;
  n -= 5;
  bool match = true;
  Element search, filter;
  if (type == 2 || type == 6) {
    if (!element(p, n, &search) || search.type != 6) {
      error(s, ch, id, 3);
      return;
    }
    match = search_matches(&search);
    p += search.total;
    n -= search.total;
  } else if (type == 4) {
    if (n < 4 || memcmp(p, "\0\1\0\1", 4)) {
      error(s, ch, id, 2);
      return;
    }
    p += 4;
    n -= 4;
  } else {
    error(s, ch, id, 3);
    return;
  }
  if (n < 3) {
    error(s, ch, id, 3);
    return;
  }
  unsigned maximum = be16(p);
  p += 2;
  n -= 2;
  if (!maximum || (type != 2 && maximum < 7)) {
    error(s, ch, id, 3);
    return;
  }
  if (type != 2) {
    if (!element(p, n, &filter) || filter.type != 6) {
      error(s, ch, id, 3);
      return;
    }
    p += filter.total;
    n -= filter.total;
  }
  unsigned offset = 0;
  if (n == 3 && p[0] == 2)
    offset = be16(p + 1);
  else if (n != 1 || p[0]) {
    error(s, ch, id, 5);
    return;
  }
  uint8_t result[256] = {0};
  if (type == 2) {
    if (offset) {
      error(s, ch, id, 5);
      return;
    }
    result[1] = result[3] = match ? 1 : 0;
    if (match) {
      result[5] = result[7] = 1;
    }
    respond(s, ch, 3, id, result, match ? 9 : 5);
    return;
  }
  uint8_t list[192] = {0x35, 0};
  unsigned size = type == 6 && match ? 4 : 2;
  if (size == 4)
    list[2] = 0x35;
  if (match) {
    for (unsigned i = 0; i < sizeof(attributes) / sizeof(attributes[0]); ++i) {
      if (!attr_requested(&filter, attributes[i].id))
        continue;
      list[size++] = 9;
      put16(list + size, attributes[i].id);
      size += 2;
      memcpy(list + size, attributes[i].data, attributes[i].length);
      size += attributes[i].length;
    }
  }
  list[1] = size - 2;
  if (type == 6 && match)
    list[3] = size - 4;
  if (offset >= size) {
    error(s, ch, id, 5);
    return;
  }
  unsigned chunk = size - offset;
  if (chunk > maximum)
    chunk = maximum;
  unsigned mtu = ch->mtu < BT_CLASSIC_MTU ? ch->mtu : BT_CLASSIC_MTU;
  if (chunk > mtu - 10)
    chunk = mtu - 10;
  put16(result, chunk);
  memcpy(result + 2, list + offset, chunk);
  unsigned out = chunk + 2;
  if (offset + chunk < size) {
    result[out++] = 2;
    put16(result + out, offset + chunk);
    out += 2;
  } else
    result[out++] = 0;
  respond(s, ch, type == 6 ? 7 : 5, id, result, out);
}

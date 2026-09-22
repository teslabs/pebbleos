/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "h4_stream.h"

#include <string.h>

void h4_stream_init(H4Stream *stream, uint8_t *buffer, size_t capacity) {
  *stream = (H4Stream){.buffer = buffer, .capacity = capacity};
}

bool h4_stream_feed(H4Stream *s, const uint8_t *data, size_t length, H4PacketHandler handler,
                    void *context) {
  while (length && !s->failed) {
    if (!s->used) {
      static const uint8_t sizes[] = {0, 4, 5, 4, 3, 5};
      if (*data >= sizeof(sizes) || !sizes[*data] || sizes[*data] > s->capacity) {
        s->failed = true;
        break;
      }
      s->needed = sizes[*data];
      s->header = true;
    }
    size_t count = s->needed - s->used;
    if (count > length) {
      count = length;
    }
    memcpy(s->buffer + s->used, data, count);
    s->used += count;
    data += count;
    length -= count;
    if (s->used != s->needed) {
      continue;
    }
    if (s->header) {
      size_t payload;
      switch (s->buffer[0]) {
        case 2:
        case 5:
          payload = s->buffer[3] | ((size_t)s->buffer[4] << 8);
          if (s->buffer[0] == 5) {
            payload &= 0x3fff;
          }
          break;
        case 4:
          payload = s->buffer[2];
          break;
        default:
          payload = s->buffer[3];
          break;
      }
      if (payload > s->capacity - s->needed) {
        s->failed = true;
        break;
      }
      s->needed += payload;
      s->header = false;
    }
    if (s->used == s->needed) {
      handler(s->buffer, s->used, context);
      s->used = 0;
    }
  }
  return !s->failed;
}

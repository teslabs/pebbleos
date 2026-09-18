/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint8_t *buffer;
  size_t capacity;
  size_t used;
  size_t needed;
  bool header;
  bool failed;
} H4Stream;

typedef void (*H4PacketHandler)(uint8_t *packet, size_t length, void *context);

void h4_stream_init(H4Stream *stream, uint8_t *buffer, size_t capacity);
// An invalid type/length poisons the stream until reinitialized; never guess packet boundaries.
bool h4_stream_feed(H4Stream *stream, const uint8_t *data, size_t length, H4PacketHandler handler,
                    void *context);

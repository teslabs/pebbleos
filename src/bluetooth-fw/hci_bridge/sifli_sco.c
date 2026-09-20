/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "sifli_sco.h"

#include <string.h>

_Static_assert(sizeof(SifliAudioRing) == 20, "Ring ABI");
_Static_assert(sizeof(SifliAudioLink) == 20 && offsetof(SifliAudioLink, rx_length) == 14,
               "Link ABI");

static uint16_t prv_u16(const uint8_t *p) {
  return p[0] | (uint16_t)p[1] << 8;
}

static void prv_reset_rings(SifliSco *s) {
  *s->downlink = (SifliAudioRing){
    .read_buffer = s->bus_address + 0x50,
    .write_buffer = s->bus_address + 0x50,
    .capacity = SIFLI_SCO_CAPACITY,
  };
  *s->uplink = (SifliAudioRing){
    .read_buffer = s->bus_address + 0x50 + SIFLI_SCO_CAPACITY,
    .write_buffer = s->bus_address + 0x50 + SIFLI_SCO_CAPACITY,
    .capacity = SIFLI_SCO_CAPACITY,
  };
  __atomic_thread_fence(__ATOMIC_RELEASE);
}

static void prv_close(SifliSco *s) {
  s->active = false;
  s->pcm_requested = false;
  s->tx_bytes = s->completions = s->tx_head = s->tx_count = 0;
  s->decoder = (CvsdCodec){0};
  s->encoder = (CvsdCodec){0};
}

void sifli_sco_init(SifliSco *s, volatile void *memory, uint32_t bus_address, bool pcm_allowed) {
  volatile uint8_t *base = memory;
  *s = (SifliSco){
    .downlink = (volatile void *)base,
    .uplink = (volatile void *)(base + 0x20),
    .link = (volatile void *)(base + SIFLI_SCO_MEMORY_SIZE - sizeof(SifliAudioLink)),
    .downlink_pool = base + 0x50,
    .uplink_pool = base + 0x50 + SIFLI_SCO_CAPACITY,
    .bus_address = bus_address,
    .pcm_allowed = pcm_allowed,
  };
  prv_reset_rings(s);
}

// Validate descriptor addresses without ever dereferencing its pointer fields.
static int prv_available(const SifliAudioRing volatile *ring, uint32_t address) {
  if (ring->capacity != SIFLI_SCO_CAPACITY || ring->read_buffer != address ||
      ring->write_buffer != address) {
    return -1;
  }
  uint32_t read = ring->read_cursor;
  uint32_t write = ring->write_cursor;
  __atomic_thread_fence(__ATOMIC_ACQUIRE);
  unsigned ri = read >> 16, wi = write >> 16;
  unsigned rm = read & 0xffff, wm = write & 0xffff;
  if (ri >= SIFLI_SCO_CAPACITY || wi >= SIFLI_SCO_CAPACITY || (rm != 0 && rm != 0xffff) ||
      (wm != 0 && wm != 0xffff)) {
    return -1;
  }
  if (rm == wm) {
    return wi >= ri ? (int)(wi - ri) : -1;
  }
  return wi <= ri ? (int)(SIFLI_SCO_CAPACITY - ri + wi) : -1;
}

static uint32_t prv_advance(uint32_t cursor, size_t count) {
  unsigned index = (cursor >> 16) + count;
  unsigned mirror = cursor & 0xffff;
  if (index >= SIFLI_SCO_CAPACITY) {
    index -= SIFLI_SCO_CAPACITY;
    mirror ^= 0xffff;
  }
  return (index << 16) | mirror;
}

static void prv_peek(SifliSco *s, uint8_t *data, size_t length) {
  unsigned index = s->downlink->read_cursor >> 16;
  for (size_t i = 0; i < length; ++i) {
    data[i] = s->downlink_pool[(index + i) % SIFLI_SCO_CAPACITY];
  }
}

static bool prv_link_matches(SifliSco *s) {
  return s->active && s->link->status == 0 && s->link->handle == s->handle &&
         s->link->air_mode == 2 && s->link->rx_length * 2 == s->frame_length &&
         s->link->tx_length * 2 == s->frame_length;
}

static void prv_account_tx(SifliSco *s) {
  int available = prv_available(s->uplink, s->bus_address + 0x50 + SIFLI_SCO_CAPACITY);
  if (available < 0 || available > s->tx_bytes) {
    return;
  }
  unsigned consumed = s->tx_bytes - available;
  s->tx_bytes = available;
  while (consumed && s->tx_count) {
    uint16_t *remaining = &s->tx_lengths[s->tx_head];
    if (consumed < *remaining) {
      *remaining -= consumed;
      break;
    }
    consumed -= *remaining;
    s->tx_head = (s->tx_head + 1) % SIFLI_SCO_CREDITS;
    --s->tx_count;
    ++s->tx_consumed;
    if (s->flow_control) {
      ++s->completions;
    }
  }
}

static size_t prv_complete(uint8_t *response, uint16_t opcode, uint8_t status) {
  const uint8_t event[] = {4, 0x0e, 4, 1, opcode & 0xff, opcode >> 8, status};
  memcpy(response, event, sizeof(event));
  return sizeof(event);
}

static size_t prv_reject_async(uint8_t *response, uint16_t opcode) {
  const uint8_t event[] = {4, 0x0f, 4, 0x11, 1, opcode & 0xff, opcode >> 8};
  memcpy(response, event, sizeof(event));
  return sizeof(event);
}

size_t sifli_sco_command(SifliSco *s, const uint8_t *p, size_t length, uint8_t response[8]) {
  if (length < 4 || p[0] != 1 || length != (size_t)p[3] + 4) {
    return 0;
  }
  uint16_t opcode = prv_u16(p + 1);
  switch (opcode) {
    case 0x0c03: // Reset: retire audio immediately; reset rings after controller completion.
      prv_close(s);
      s->flow_control = false;
      break;
    case 0x0c2e: // Read Synchronous Flow Control Enable.
      prv_complete(response, opcode, p[3] ? 0x12 : 0);
      response[2] = 5;
      response[7] = s->flow_control;
      return 8;
    case 0x0c2f: // Write Synchronous Flow Control Enable (only outside a connection).
      if (p[3] != 1 || p[4] > 1) {
        return prv_complete(response, opcode, 0x12);
      }
      if (s->active) {
        return prv_complete(response, opcode, 0x0c);
      }
      s->flow_control = p[4];
      return prv_complete(response, opcode, 0);
    case 0x0c31: // Controller-to-host SCO credits are not implemented by this prototype.
      if (p[3] == 1 && (p[4] & 2)) {
        return prv_complete(response, opcode, 0x11);
      }
      break;
    case 0x0407: // Legacy Add SCO lacks an explicit per-link PCM format.
    case 0x043d: // Enhanced Setup / Accept deferred until their codec paths are validated.
    case 0x043e:
      return prv_reject_async(response, opcode);
    case 0x0428:
    case 0x0429: {
      unsigned voice_offset = opcode == 0x0428 ? 16 : 20;
      unsigned expected = opcode == 0x0428 ? 17 : 21;
      if (!s->pcm_allowed || s->active || p[3] != expected || prv_u16(p + voice_offset) != 0x0060) {
        return prv_reject_async(response, opcode);
      }
      s->pcm_requested = true;
      break;
    }
  }
  return 0;
}

void sifli_sco_event(SifliSco *s, uint8_t *p, size_t length) {
  if (length < 3 || p[0] != 4 || length != (size_t)p[2] + 3) {
    return;
  }
  if (p[1] == 0x10 && length == 4) { // Hardware Error: stop using the failed controller.
    prv_close(s);
  } else if (p[1] == 0x2c && length == 20) { // Synchronous Connection Complete.
    if (p[3]) {
      s->pcm_requested = false;
      return;
    }
    bool supported =
        s->pcm_requested && s->pcm_allowed && p[19] == 2 && (p[13] == 6 || p[13] == 12) &&
        (prv_u16(p + 15) == 30 || prv_u16(p + 15) == 60) && prv_u16(p + 15) == prv_u16(p + 17);
    prv_close(s);
    if (supported) {
      s->active = true;
      s->handle = prv_u16(p + 4) & 0x0fff;
      s->frame_length = prv_u16(p + 15) * 2;
    }
  } else if (p[1] == 5 && length == 7 && p[3] == 0 && (prv_u16(p + 4) & 0x0fff) == s->handle) {
    prv_close(s);
    prv_reset_rings(s);
  } else if (p[1] == 0x0e && length >= 7 && p[6] == 0) {
    uint16_t opcode = prv_u16(p + 4);
    if (opcode == 0x0c03) {
      prv_close(s);
      s->flow_control = false;
      prv_reset_rings(s);
    } else if (opcode == 0x1005 && length == 14) {
      p[9] = SIFLI_SCO_TX_MTU;
      p[12] = SIFLI_SCO_CREDITS;
      p[13] = 0;
    } else if (opcode == 0x1002 && length == 71) {
      p[7] &= ~(1 << 6);  // Add SCO Connection.
      p[7 + 29] &= ~0x18; // Enhanced synchronous setup/accept.
    }
  }
}

size_t sifli_sco_receive(SifliSco *s, uint8_t packet[4 + SIFLI_SCO_MTU]) {
  if (!prv_link_matches(s)) {
    return 0;
  }
  prv_account_tx(s);
  int available = prv_available(s->downlink, s->bus_address + 0x50);
  if (available < 4) {
    return 0;
  }
  uint8_t frame[4 + SIFLI_SCO_MTU];
  prv_peek(s, frame, 4);
  s->last_length = frame[0];
  s->last_status = frame[1];
  unsigned native_length = s->frame_length / (s->software_cvsd ? 2 : 1);
  if (frame[0] != native_length || frame[1] > 3) {
    ++s->malformed;
    s->active = false;
    return 0;
  }
  size_t count = frame[0] + 4;
  if ((size_t)available < count) {
    return 0;
  }
  prv_peek(s, frame, count);
  packet[0] = 3;
  packet[1] = s->handle & 0xff;
  packet[2] = (s->handle >> 8) | (frame[1] << 4);
  packet[3] = s->frame_length;
  if (s->software_cvsd) {
    for (unsigned i = 0; i < native_length; ++i) {
      // Advance the predictor through missing time with alternating bits.
      int16_t sample = cvsd_decode_sample(&s->decoder, frame[1] >= 2 ? 0x55 : frame[4 + i]);
      packet[4 + 2 * i] = frame[1] >= 2 ? 0 : (uint16_t)sample & 0xff;
      packet[5 + 2 * i] = frame[1] >= 2 ? 0 : (uint16_t)sample >> 8;
    }
  } else if (frame[1] >= 2) {
    memset(packet + 4, 0, s->frame_length);
  } else {
    memcpy(packet + 4, frame + 4, frame[0]);
  }
  __atomic_thread_fence(__ATOMIC_RELEASE);
  s->downlink->read_cursor = prv_advance(s->downlink->read_cursor, count);
  ++s->rx_packets;
  s->rx_bytes += s->frame_length;
  s->rx_bad += frame[1] != 0;
  return s->frame_length + 4;
}

void sifli_sco_send(SifliSco *s, const uint8_t *p, size_t length) {
  if (length < 4 || p[0] != 3 || !prv_link_matches(s) || prv_u16(p + 1) != s->handle) {
    ++s->tx_dropped;
    return;
  }
  prv_account_tx(s);
  unsigned native_length = p[3] / (s->software_cvsd ? 2 : 1);
  if (length != (size_t)p[3] + 4 || !p[3] || p[3] > SIFLI_SCO_TX_MTU || (p[3] & 1) ||
      s->tx_count == SIFLI_SCO_CREDITS || s->tx_bytes + native_length > SIFLI_SCO_CAPACITY) {
    ++s->tx_dropped;
    s->completions += s->flow_control;
    return;
  }
  int available = prv_available(s->uplink, s->bus_address + 0x50 + SIFLI_SCO_CAPACITY);
  if (available < 0 || available + native_length > SIFLI_SCO_CAPACITY) {
    ++s->tx_dropped;
    s->completions += s->flow_control;
    return;
  }
  uint32_t cursor = s->uplink->write_cursor;
  unsigned index = cursor >> 16;
  for (unsigned i = 0; i < native_length; ++i) {
    uint8_t byte = s->software_cvsd
                       ? cvsd_encode_sample(&s->encoder, (int16_t)prv_u16(p + 4 + 2 * i))
                       : p[4 + i];
    s->uplink_pool[(index + i) % SIFLI_SCO_CAPACITY] = byte;
  }
  s->tx_lengths[(s->tx_head + s->tx_count) % SIFLI_SCO_CREDITS] = native_length;
  ++s->tx_count;
  s->tx_bytes += native_length;
  ++s->tx_packets;
  __atomic_thread_fence(__ATOMIC_RELEASE);
  s->uplink->write_cursor = prv_advance(cursor, native_length);
}

size_t sifli_sco_completed(SifliSco *s, uint8_t packet[8]) {
  if (!s->active) {
    return 0;
  }
  prv_account_tx(s);
  if (!s->flow_control || !s->completions) {
    return 0;
  }
  const uint8_t event[] = {
    4, 0x13, 5, 1, s->handle & 0xff, s->handle >> 8, s->completions & 0xff, s->completions >> 8
  };
  memcpy(packet, event, sizeof(event));
  s->completions = 0;
  return sizeof(event);
}

/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <pbl/btutil/cvsd.h>

#define SIFLI_SCO_MEMORY_SIZE 1024
#define SIFLI_SCO_CAPACITY    460
#define SIFLI_SCO_MTU         120
#define SIFLI_SCO_TX_MTU      60
#define SIFLI_SCO_CREDITS     7

// SF32LB52 controller ABI. Cursor: index in high 16 bits, mirror (0/ffff) in low 16.
typedef struct {
  uint32_t read_buffer;
  uint32_t write_buffer;
  uint32_t read_cursor;
  uint32_t write_cursor;
  int16_t capacity;
  uint16_t padding;
} SifliAudioRing;

typedef struct {
  uint8_t status, padding;
  uint16_t handle;
  uint8_t address[6];
  uint8_t link_type, interval, retransmission_window, padding2;
  uint16_t rx_length, tx_length;
  uint8_t air_mode, padding3;
} SifliAudioLink;

typedef struct {
  volatile SifliAudioRing *downlink, *uplink;
  volatile SifliAudioLink *link;
  volatile uint8_t *downlink_pool, *uplink_pool;
  uint32_t bus_address;
  bool active, flow_control, pcm_requested, pcm_allowed;
  bool software_cvsd;
  CvsdCodec decoder, encoder;
  uint16_t handle, frame_length;
  uint16_t tx_lengths[SIFLI_SCO_CREDITS];
  uint16_t tx_bytes, completions;
  uint8_t tx_head, tx_count;
  uint32_t rx_packets, rx_bytes, rx_bad, tx_packets, tx_consumed, tx_dropped, malformed;
  uint8_t last_length, last_status;
} SifliSco;

// Call with the controller awake and no synchronous connection. No payload is cleared.
void sifli_sco_init(SifliSco *s, volatile void *memory, uint32_t bus_address, bool pcm_allowed);
// These operations have a single task owner, and require accessible controller RAM.
// Returns a synthesized H4 response length if intercepted; zero means forward the command.
size_t sifli_sco_command(SifliSco *s, const uint8_t *packet, size_t length, uint8_t response[8]);
void sifli_sco_event(SifliSco *s, uint8_t *packet, size_t length);
size_t sifli_sco_receive(SifliSco *s, uint8_t packet[4 + SIFLI_SCO_MTU]);
void sifli_sco_send(SifliSco *s, const uint8_t *packet, size_t length);
size_t sifli_sco_completed(SifliSco *s, uint8_t packet[8]);

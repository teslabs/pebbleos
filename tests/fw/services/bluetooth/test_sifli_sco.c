/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <clar.h>
#include <stdint.h>
#include <string.h>

#include "h4_stream.h"
#include "sifli_sco.h"

static uint32_t s_memory[SIFLI_SCO_MEMORY_SIZE / 4];
static SifliSco s_sco;
static const uint8_t s_connection[] = {
  4, 0x2c, 17, 0, 0x80, 1, 1, 2, 3, 4, 5, 6, 2, 6, 2, 30, 0, 30, 0, 2,
};
static uint8_t s_output[2048];
static size_t s_output_length, s_packets;

void test_sifli_sco__initialize(void) {
  memset(s_memory, 0, sizeof(s_memory));
  sifli_sco_init(&s_sco, s_memory, 0x2040e000, true);
  s_output_length = s_packets = 0;
}

static uint32_t prv_cursor(unsigned absolute) {
  return ((absolute % SIFLI_SCO_CAPACITY) << 16) |
         ((absolute / SIFLI_SCO_CAPACITY) % 2 ? 0xffff : 0);
}

static void prv_connect(void) {
  uint8_t request[25] = {1, 0x29, 4, 21};
  request[20] = 0x60;
  cl_assert_equal_i(sifli_sco_command(&s_sco, request, sizeof(request), s_output), 0);
  *s_sco.link = (SifliAudioLink){
    .handle = 384,
    .link_type = 2,
    .interval = 6,
    .rx_length = 30,
    .tx_length = 30,
    .air_mode = 2,
  };
  uint8_t event[sizeof(s_connection)];
  memcpy(event, s_connection, sizeof(event));
  sifli_sco_event(&s_sco, event, sizeof(event));
  cl_assert(s_sco.active);
}

static void prv_frame(unsigned start, unsigned available, uint8_t status) {
  for (unsigned i = 0; i < 64; ++i) {
    uint8_t byte = i == 0 ? 60 : i == 1 ? status : (uint8_t)i;
    s_sco.downlink_pool[(start + i) % SIFLI_SCO_CAPACITY] = byte;
  }
  s_sco.downlink->write_cursor = prv_cursor(start + available);
}

void test_sifli_sco__downlink_wrap_and_packet_status(void) {
  prv_connect();
  for (unsigned i = 0; i < 30; ++i) {
    uint8_t status = i % 4;
    prv_frame(i * 64, 64, status);
    cl_assert_equal_i(sifli_sco_receive(&s_sco, s_output), 64);
    cl_assert_equal_i(s_output[0], 3);
    cl_assert_equal_i(s_output[1], 0x80);
    cl_assert_equal_i(s_output[2], 1 | (status << 4));
    cl_assert_equal_i(s_output[3], 60);
    for (unsigned j = 0; j < 60; ++j) {
      cl_assert_equal_i(s_output[4 + j], status >= 2 ? 0 : j + 4);
    }
    cl_assert_equal_i(s_sco.downlink->read_cursor, prv_cursor((i + 1) * 64));
    cl_assert_equal_i(sifli_sco_receive(&s_sco, s_output), 0);
  }
  cl_assert_equal_i(s_sco.rx_packets, 30);
  cl_assert_equal_i(s_sco.rx_bytes, 1800);
}

void test_sifli_sco__partial_and_malformed_frames(void) {
  prv_connect();
  prv_frame(0, 30, 0);
  cl_assert_equal_i(sifli_sco_receive(&s_sco, s_output), 0);
  cl_assert_equal_i(s_sco.downlink->read_cursor, 0);
  s_sco.downlink->write_cursor = prv_cursor(64);
  cl_assert_equal_i(sifli_sco_receive(&s_sco, s_output), 64);
  prv_frame(64, 64, 0);
  s_sco.downlink_pool[64] = 30; // Encoded CVSD must not be presented as PCM.
  cl_assert_equal_i(sifli_sco_receive(&s_sco, s_output), 0);
  cl_assert_equal_i(s_sco.malformed, 1);
  cl_assert(!s_sco.active);
  cl_assert_equal_i(s_sco.downlink->read_cursor, prv_cursor(64));
}

void test_sifli_sco__invalid_descriptor_is_not_dereferenced(void) {
  prv_connect();
  prv_frame(0, 64, 0);
  s_sco.downlink->read_buffer = 0xdeadbeef;
  cl_assert_equal_i(sifli_sco_receive(&s_sco, s_output), 0);
  s_sco.downlink->read_buffer = 0x2040e050;
  s_sco.downlink->write_cursor = (500 << 16);
  cl_assert_equal_i(sifli_sco_receive(&s_sco, s_output), 0);
  cl_assert_equal_i(s_sco.rx_packets, 0);
}

static void prv_enable_flow(void) {
  const uint8_t enable[] = {1, 0x2f, 0x0c, 1, 1};
  cl_assert_equal_i(sifli_sco_command(&s_sco, enable, sizeof(enable), s_output), 7);
  cl_assert_equal_i(s_output[6], 0);
}

void test_sifli_sco__software_cvsd_downlink_preserves_state_across_ring_wrap(void) {
  s_sco.software_cvsd = true;
  prv_connect();
  CvsdCodec reference = {0};
  for (unsigned frame = 0; frame < 40; ++frame) {
    unsigned start = frame * 34;
    uint8_t status = frame % 4;
    for (unsigned i = 0; i < 34; ++i) {
      uint8_t byte = i == 0 ? 30 : i == 1 ? status : (uint8_t)(start + i);
      s_sco.downlink_pool[(start + i) % SIFLI_SCO_CAPACITY] = byte;
    }
    s_sco.downlink->write_cursor = prv_cursor(start + 33);
    cl_assert_equal_i(sifli_sco_receive(&s_sco, s_output), 0);
    s_sco.downlink->write_cursor = prv_cursor(start + 34);
    cl_assert_equal_i(sifli_sco_receive(&s_sco, s_output), 64);
    cl_assert_equal_i(s_output[2], 1 | (status << 4));
    cl_assert_equal_i(s_output[3], 60);
    for (unsigned i = 0; i < 30; ++i) {
      int16_t expected = cvsd_decode_sample(&reference, status >= 2 ? 0x55 : start + i + 4);
      int16_t actual = s_output[4 + 2 * i] | (uint16_t)s_output[5 + 2 * i] << 8;
      cl_assert_equal_i(actual, status >= 2 ? 0 : expected);
    }
    cl_assert_equal_i(s_sco.downlink->read_cursor, prv_cursor(start + 34));
  }
  cl_assert_equal_i(s_sco.rx_bytes, 2400);
  const uint8_t reset[] = {1, 3, 0x0c, 0};
  sifli_sco_command(&s_sco, reset, sizeof(reset), s_output);
  CvsdCodec empty = {0};
  cl_assert_equal_m(&s_sco.decoder, &empty, sizeof(empty));
}

void test_sifli_sco__software_cvsd_uplink_credits_count_pcm_packets(void) {
  s_sco.software_cvsd = true;
  prv_enable_flow();
  prv_connect();
  CvsdCodec reference = {0};
  for (unsigned frame = 0; frame < 40; ++frame) {
    uint8_t packet[64] = {3, 0x80, 1, 60};
    for (unsigned i = 0; i < 30; ++i) {
      int16_t sample = (int16_t)((frame * 30 + i) * 117);
      packet[4 + 2 * i] = (uint16_t)sample & 0xff;
      packet[5 + 2 * i] = (uint16_t)sample >> 8;
    }
    sifli_sco_send(&s_sco, packet, sizeof(packet));
    cl_assert_equal_i(s_sco.uplink->write_cursor, prv_cursor((frame + 1) * 30));
    for (unsigned i = 0; i < 30; ++i) {
      uint8_t expected = cvsd_encode_sample(&reference, (int16_t)((frame * 30 + i) * 117));
      cl_assert_equal_i(s_sco.uplink_pool[(frame * 30 + i) % SIFLI_SCO_CAPACITY], expected);
    }
    s_sco.uplink->read_cursor = prv_cursor(frame * 30 + 29);
    cl_assert_equal_i(sifli_sco_completed(&s_sco, s_output), 0);
    s_sco.uplink->read_cursor = prv_cursor((frame + 1) * 30);
    cl_assert_equal_i(sifli_sco_completed(&s_sco, s_output), 8);
    cl_assert_equal_i(s_output[6], 1);
  }
  cl_assert_equal_i(s_sco.tx_consumed, 40);
  cl_assert_equal_i(s_sco.tx_dropped, 0);
  uint8_t disconnect[] = {4, 5, 4, 0, 0x80, 1, 0x13};
  sifli_sco_event(&s_sco, disconnect, sizeof(disconnect));
  CvsdCodec empty = {0};
  cl_assert_equal_m(&s_sco.encoder, &empty, sizeof(empty));
}

static void prv_send(unsigned size) {
  uint8_t packet[124] = {3, 0x80, 1};
  packet[3] = size;
  for (unsigned i = 0; i < size; ++i) {
    packet[i + 4] = i;
  }
  sifli_sco_send(&s_sco, packet, size + 4);
}

void test_sifli_sco__credits_follow_consumption_including_partial_packet(void) {
  prv_enable_flow();
  prv_connect();
  prv_send(60);
  prv_send(30);
  cl_assert_equal_i(sifli_sco_completed(&s_sco, s_output), 0);
  s_sco.uplink->read_cursor = prv_cursor(30);
  cl_assert_equal_i(sifli_sco_completed(&s_sco, s_output), 0);
  s_sco.uplink->read_cursor = prv_cursor(60);
  cl_assert_equal_i(sifli_sco_completed(&s_sco, s_output), 8);
  const uint8_t complete[] = {4, 0x13, 5, 1, 0x80, 1, 1, 0};
  cl_assert_equal_m(s_output, complete, sizeof(complete));
  cl_assert_equal_i(s_sco.tx_count, 1);
  s_sco.uplink->read_cursor = prv_cursor(90);
  cl_assert_equal_i(sifli_sco_completed(&s_sco, s_output), 8);
  cl_assert_equal_i(s_sco.tx_consumed, 2);
  cl_assert_equal_i(s_sco.tx_count, 0);
}

void test_sifli_sco__uplink_wrap_and_bounded_credits(void) {
  prv_enable_flow();
  prv_connect();
  for (unsigned round = 0; round < 8; ++round) {
    for (unsigned i = 0; i < 7; ++i) {
      prv_send(60);
    }
    cl_assert_equal_i(s_sco.tx_count, 7);
    uint32_t before = s_sco.uplink->write_cursor;
    prv_send(60);
    cl_assert_equal_i(s_sco.uplink->write_cursor, before);
    cl_assert_equal_i(s_sco.tx_dropped, round + 1);
    for (unsigned i = 0; i < 420; ++i) {
      cl_assert_equal_i(s_sco.uplink_pool[(round * 420 + i) % SIFLI_SCO_CAPACITY], i % 60);
    }
    s_sco.uplink->read_cursor = prv_cursor((round + 1) * 420);
    cl_assert_equal_i(sifli_sco_completed(&s_sco, s_output), 8);
    cl_assert_equal_i(s_output[6], 8); // Seven consumed packets, one explicitly dropped.
  }
  cl_assert_equal_i(s_sco.tx_consumed, 56);
}

void test_sifli_sco__disconnect_reset_and_stale_handles(void) {
  prv_connect();
  prv_send(60);
  uint8_t disconnect[] = {4, 5, 4, 0, 0x81, 1, 0x13};
  sifli_sco_event(&s_sco, disconnect, sizeof(disconnect));
  cl_assert(s_sco.active); // Other handle does not retire this stream.
  disconnect[4] = 0x80;
  sifli_sco_event(&s_sco, disconnect, sizeof(disconnect));
  cl_assert(!s_sco.active);
  cl_assert_equal_i(s_sco.tx_count, 0);
  cl_assert_equal_i(s_sco.uplink->write_cursor, 0);
  prv_send(60);
  cl_assert_equal_i(s_sco.tx_dropped, 1);
  prv_connect();
  prv_frame(0, 64, 0);
  const uint8_t reset[] = {1, 3, 0x0c, 0};
  sifli_sco_command(&s_sco, reset, sizeof(reset), s_output);
  cl_assert(!s_sco.active);
  cl_assert_equal_i(sifli_sco_receive(&s_sco, s_output), 0);
  cl_assert_equal_i(s_sco.downlink->write_cursor, prv_cursor(64));
  uint8_t complete[] = {4, 0x0e, 4, 1, 3, 0x0c, 0};
  sifli_sco_event(&s_sco, complete, sizeof(complete));
  cl_assert_equal_i(s_sco.downlink->write_cursor, 0);
}

void test_sifli_sco__unsupported_format_and_buffer_reporting(void) {
  uint8_t request[25] = {1, 0x29, 4, 21};
  request[20] = 0x63;
  cl_assert_equal_i(sifli_sco_command(&s_sco, request, sizeof(request), s_output), 7);
  cl_assert_equal_i(s_output[1], 0x0f);
  cl_assert_equal_i(s_output[3], 0x11);
  uint8_t buffers[] = {4, 0x0e, 11, 1, 5, 0x10, 0, 0xfd, 3, 255, 4, 0, 4, 0};
  sifli_sco_event(&s_sco, buffers, sizeof(buffers));
  cl_assert_equal_i(buffers[7], 0xfd); // Preserve controller ACL accounting.
  cl_assert_equal_i(buffers[10], 4);
  cl_assert_equal_i(buffers[9], SIFLI_SCO_TX_MTU);
  cl_assert_equal_i(buffers[12], SIFLI_SCO_CREDITS);
}

static void prv_packet(uint8_t *packet, size_t length, void *context) {
  cl_assert(context == &s_packets);
  cl_assert(s_output_length + length <= sizeof(s_output));
  memcpy(s_output + s_output_length, packet, length);
  s_output_length += length;
  ++s_packets;
}

void test_sifli_sco__hardware_error_retires_native_stream(void) {
  prv_connect();
  uint8_t error[] = {4, 0x10, 1, 45};
  sifli_sco_event(&s_sco, error, sizeof(error));
  cl_assert(!s_sco.active);
  cl_assert_equal_i(sifli_sco_receive(&s_sco, s_output), 0);
}

void test_sifli_sco__h4_arbitrary_fragmentation_and_coalescing(void) {
  const uint8_t stream[] = {1,    3, 0x0c, 0, 2, 1, 0x20, 3, 0, 7, 8,    9, 3,
                            0x80, 1, 2,    0, 0, 4, 0x0e, 4, 1, 3, 0x0c, 0};
  for (unsigned chunk = 1; chunk <= sizeof(stream); ++chunk) {
    H4Stream parser;
    uint8_t buffer[32];
    h4_stream_init(&parser, buffer, sizeof(buffer));
    s_output_length = s_packets = 0;
    for (unsigned offset = 0; offset < sizeof(stream); offset += chunk) {
      unsigned size = sizeof(stream) - offset;
      if (size > chunk) {
        size = chunk;
      }
      cl_assert(h4_stream_feed(&parser, stream + offset, size, prv_packet, &s_packets));
    }
    cl_assert_equal_i(s_packets, 4);
    cl_assert_equal_i(s_output_length, sizeof(stream));
    cl_assert_equal_m(s_output, stream, sizeof(stream));
  }
}

void test_sifli_sco__h4_oversize_and_invalid_types_fail_closed(void) {
  uint8_t buffer[8];
  H4Stream parser;
  h4_stream_init(&parser, buffer, sizeof(buffer));
  const uint8_t large[] = {2, 1, 0, 0xff, 0xff};
  cl_assert(!h4_stream_feed(&parser, large, sizeof(large), prv_packet, &s_packets));
  const uint8_t reset[] = {1, 3, 0x0c, 0};
  cl_assert(!h4_stream_feed(&parser, reset, sizeof(reset), prv_packet, &s_packets));
  cl_assert_equal_i(s_packets, 0);
  h4_stream_init(&parser, buffer, sizeof(buffer));
  const uint8_t invalid[] = {0};
  cl_assert(!h4_stream_feed(&parser, invalid, sizeof(invalid), prv_packet, &s_packets));
}

void test_sifli_sco__large_native_frame_uses_multiple_uplink_packets(void) {
  prv_enable_flow();
  prv_connect();
  s_sco.pcm_requested = true;
  s_sco.link->interval = 12;
  s_sco.link->rx_length = s_sco.link->tx_length = 60;
  uint8_t event[sizeof(s_connection)];
  memcpy(event, s_connection, sizeof(event));
  event[13] = 12;
  event[15] = event[17] = 60;
  sifli_sco_event(&s_sco, event, sizeof(event));
  for (unsigned i = 0; i < 124; ++i) {
    s_sco.downlink_pool[i] = i < 4 ? 0 : i;
  }
  s_sco.downlink_pool[0] = 120;
  s_sco.downlink->write_cursor = prv_cursor(124);
  cl_assert_equal_i(sifli_sco_receive(&s_sco, s_output), 124);
  cl_assert_equal_i(s_output[3], 120);
  prv_send(60);
  prv_send(60);
  s_sco.uplink->read_cursor = prv_cursor(120);
  cl_assert_equal_i(sifli_sco_completed(&s_sco, s_output), 8);
  cl_assert_equal_i(s_output[6], 2);
}

void test_sifli_sco__controller_sized_acl_packet_preserves_embedded_type_bytes(void) {
  uint8_t input[1026] = {2, 1, 0x20, 0xfd, 3};
  for (unsigned i = 5; i < sizeof(input); ++i) {
    input[i] = i;
  }
  H4Stream parser;
  uint8_t buffer[1030];
  h4_stream_init(&parser, buffer, sizeof(buffer));
  for (unsigned offset = 0; offset < sizeof(input); ++offset) {
    cl_assert(h4_stream_feed(&parser, input + offset, 1, prv_packet, &s_packets));
  }
  cl_assert_equal_i(s_packets, 1);
  cl_assert_equal_m(input, s_output, sizeof(input));
}

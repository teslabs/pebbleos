/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <pbl/services/bluetooth/hfp_demo.h>

#define CLASSIC_DEMO_MTU         672
#define CLASSIC_DEMO_NUMBER_SIZE 33
#define CLASSIC_DEMO_NO_HANDLE   0xffff

typedef HfpDemoStatus ClassicDemoStatus;

typedef struct {
  uint16_t local, remote, psm, mtu;
  bool configured, peer_configured;
  uint8_t config_id;
} ClassicDemoChannel;

typedef struct {
  uint16_t length;
  uint8_t data[CLASSIC_DEMO_MTU + 9];
} ClassicDemoPacket;

typedef struct {
  // All operations run on one host task. The callback copies complete H4 packets.
  void (*send)(const uint8_t *, size_t, void *);
  void *context;
  ClassicDemoStatus status;
  uint32_t now, command_deadline, profile_deadline;
  uint16_t handle, sco_handle, acl_mtu, acl_limit, acl_inflight, pending_opcode;
  uint8_t command_credit, startup, signal_id;
  uint8_t peer[6], key_peer[6], key[16];
  bool key_valid, accepting;
  struct {
    uint16_t length;
    uint8_t data[259];
  } commands[8];
  unsigned command_head, command_count;
  ClassicDemoPacket output[12];
  unsigned output_head, output_count;
  uint8_t receive[CLASSIC_DEMO_MTU + 4];
  unsigned receive_length, receive_needed;
  ClassicDemoChannel channels[4];
  uint16_t rfcomm_cid, rfcomm_mtu, rfcomm_credits;
  uint8_t dlci, rx_credits, slc_step;
  bool rfcomm_open, credit_mode, modem_ready, at_pending;
  uint8_t at_tx[96];
  unsigned at_tx_length;
  char at_line[512];
  unsigned at_line_length;
  uint8_t indicator_call, indicator_setup;
} ClassicDemoHost;

void classic_demo_init(ClassicDemoHost *host, void (*send)(const uint8_t *, size_t, void *),
                       void *context);
void classic_demo_poll(ClassicDemoHost *host, uint32_t milliseconds);
void classic_demo_receive(ClassicDemoHost *host, const uint8_t *packet, size_t length);
bool classic_demo_dial(ClassicDemoHost *host, const char *number);
bool classic_demo_answer(ClassicDemoHost *host);
bool classic_demo_hangup(ClassicDemoHost *host);
bool classic_demo_valid_number(const char *number);

// Internal protocol boundaries, still independent of controller and OS APIs.
void classic_demo_error(ClassicDemoHost *host, const char *reason);
void classic_demo_l2cap_send(ClassicDemoHost *host, uint16_t cid, const uint8_t *data, size_t size);
void classic_demo_sdp(ClassicDemoHost *host, ClassicDemoChannel *channel, const uint8_t *data,
                      size_t size);
void classic_demo_rfcomm(ClassicDemoHost *host, ClassicDemoChannel *channel, const uint8_t *data,
                         size_t size);
void classic_demo_profile_poll(ClassicDemoHost *host);
void classic_demo_profile_reset(ClassicDemoHost *host);

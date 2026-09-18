/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BT_CLASSIC_MTU         672
#define BT_CLASSIC_NUMBER_SIZE 33
#define BT_CLASSIC_NO_HANDLE   0xffff

typedef struct {
  bool available, connected, ready, audio, call, incoming, busy;
  unsigned call_setup, errors;
  char detail[64];
} BtClassicStatus;

typedef struct {
  uint16_t local, remote, psm, mtu;
  bool configured, peer_configured;
  uint8_t config_id;
} BtClassicChannel;

typedef struct {
  uint16_t length;
  uint8_t data[BT_CLASSIC_MTU + 9];
} BtClassicPacket;

typedef struct {
  // All operations run on one host task. The callback copies complete H4 packets.
  void (*send)(const uint8_t *, size_t, void *);
  void *context;
  bool (*send_acl)(const uint8_t *, size_t, void *);
  // Optional shared-bond policy: NULL key only checks presence; rejects separate SSP pairing.
  bool (*get_link_key)(const uint8_t peer[6], uint8_t key[16], void *context);
  bool encrypted, revoking;
  BtClassicStatus status;
  uint32_t now, command_deadline, profile_deadline;
  uint16_t handle, sco_handle, acl_mtu, acl_limit, acl_inflight, pending_opcode;
  uint8_t command_credit, startup, signal_id;
  uint8_t peer[6], key_peer[6], key[16];
  bool key_valid, accepting, accepting_sco, stopping;
  struct {
    uint16_t length;
    uint8_t data[259];
  } commands[8];
  unsigned command_head, command_count;
  BtClassicPacket output[12];
  unsigned output_head, output_count;
  uint8_t receive[BT_CLASSIC_MTU + 4];
  unsigned receive_length, receive_needed;
  BtClassicChannel channels[4];
  uint16_t rfcomm_cid, rfcomm_mtu, rfcomm_credits;
  uint8_t dlci, rx_credits, slc_step;
  bool rfcomm_open, credit_mode, modem_ready, at_pending, at_discard;
  uint8_t at_tx[96];
  unsigned at_tx_length;
  char at_line[512];
  unsigned at_line_length;
  uint8_t indicator_call, indicator_setup;
} BtClassicHost;

void bt_classic_init(BtClassicHost *host, void (*send)(const uint8_t *, size_t, void *),
                     void *context);
void bt_classic_poll(BtClassicHost *host, uint32_t milliseconds);
void bt_classic_receive(BtClassicHost *host, const uint8_t *packet, size_t length);
bool bt_classic_dial(BtClassicHost *host, const char *number);
bool bt_classic_answer(BtClassicHost *host);
bool bt_classic_hangup(BtClassicHost *host);
bool bt_classic_valid_number(const char *number);

// The shared host owns reset, event masks and ACL credits. Calls run on its task.
void bt_classic_init_managed(BtClassicHost *host, void (*command)(const uint8_t *, size_t, void *),
                             bool (*acl)(const uint8_t *, size_t, void *), uint16_t acl_mtu,
                             void *context);
void bt_classic_reset(BtClassicHost *host);

void bt_classic_stop(BtClassicHost *host);
bool bt_classic_stopped(const BtClassicHost *host);

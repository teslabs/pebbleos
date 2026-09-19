/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BT_CLASSIC_MTU         672
#define BT_CLASSIC_NUMBER_SIZE 33
#define BT_CLASSIC_NO_HANDLE   0xffff
#define BT_CLASSIC_MAX_CALLS   4

typedef struct {
  uint8_t index, state;
  char number[BT_CLASSIC_NUMBER_SIZE];
} BtClassicCall;

typedef struct {
  bool available, connected, ready, audio, call, incoming, busy;
  unsigned call_setup, call_held, errors;
  uint8_t hold_support;
  bool waiting;
  uint8_t speaker_gain;
  bool audio_pending;
  char detail[64];
  char caller_number[BT_CLASSIC_NUMBER_SIZE];
  char waiting_number[BT_CLASSIC_NUMBER_SIZE];
} BtClassicStatus;

typedef struct {
  uint16_t local, remote, psm, mtu;
  bool configured, peer_configured, outgoing, ready;
  uint8_t config_id, connect_id;
} BtClassicChannel;

typedef struct {
  uint16_t length;
  uint8_t data[BT_CLASSIC_MTU + 9];
} BtClassicPacket;

typedef enum {
  BtClassicConnectIdle,
  BtClassicConnectSecurity,
  BtClassicConnectSdp,
  BtClassicConnectRfcomm,
  BtClassicConnectMux,
  BtClassicConnectPn,
  BtClassicConnectDlc,
  BtClassicConnectSlc,
} BtClassicConnectStage;

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
  char local_name[64];
  bool name_dirty;
  uint8_t active_key[16];
  bool active_key_valid;
  bool key_valid, accepting, accepting_sco, stopping;
  bool connecting, canceling, rfcomm_initiator;
  BtClassicConnectStage connect_stage;
  uint32_t connect_deadline;
  uint16_t sdp_transaction, sdp_length;
  uint8_t server_channel, sdp_rounds;
  uint8_t sdp_response[512];
  uint8_t reconnect_peer[6];
  bool reconnect_valid, reconnect_busy, reconnect_suppressed;
  uint32_t reconnect_at, reconnect_delay;
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
  uint8_t indicator_call, indicator_setup, indicator_held;
  uint32_t ag_features;
  bool calls_dirty, calls_query, calls_invalid, calls_after_ack;
  uint8_t call_count;
  BtClassicCall calls[BT_CLASSIC_MAX_CALLS];
  bool speaker_gain_dirty;
  uint8_t speaker_gain_next;
  bool audio_target;
  uint32_t audio_deadline;
} BtClassicHost;

void bt_classic_init(BtClassicHost *host, void (*send)(const uint8_t *, size_t, void *),
                     void *context);
void bt_classic_poll(BtClassicHost *host, uint32_t milliseconds);
void bt_classic_receive(BtClassicHost *host, const uint8_t *packet, size_t length);
bool bt_classic_dial(BtClassicHost *host, const char *number);
bool bt_classic_answer(BtClassicHost *host);
bool bt_classic_hangup(BtClassicHost *host);
bool bt_classic_set_speaker_gain(BtClassicHost *host, unsigned gain);
bool bt_classic_transfer_audio(BtClassicHost *host, bool to_watch);
// HFP CHLD 0..3; only actions advertised by the phone are accepted.
bool bt_classic_call_hold(BtClassicHost *host, unsigned action);
bool bt_classic_valid_number(const char *number);

// The shared host owns reset, event masks and ACL credits. Calls run on its task.
void bt_classic_init_managed(BtClassicHost *host, void (*command)(const uint8_t *, size_t, void *),
                             bool (*acl)(const uint8_t *, size_t, void *), uint16_t acl_mtu,
                             void *context);
void bt_classic_reset(BtClassicHost *host);
void bt_classic_set_local_name(BtClassicHost *host, const char *name);

// Initiate HFP only for a peer authorized by the shared-bond policy.
bool bt_classic_connect(BtClassicHost *host, const uint8_t peer[6]);
void bt_classic_cancel_connect(BtClassicHost *host);
// Pass the authenticated LE peer, or NULL when no eligible peer is connected.
void bt_classic_reconnect(BtClassicHost *host, const uint8_t *peer, uint32_t milliseconds);

void bt_classic_stop(BtClassicHost *host);
bool bt_classic_stopped(const BtClassicHost *host);

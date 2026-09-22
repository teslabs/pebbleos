/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include <clar.h>
#include <pbl/bluetooth/hfp.h>
#include <string.h>

#include "service.h"
#include "fake_msgq.h"
#include "stubs_mutex.h"
#include "kernel/events.h"
#include "stubs_phone_call_util.h"
#include "stubs_passert.h"

static unsigned s_answers, s_hangups, s_dials, s_polls;
static unsigned s_audio_volume;
static int s_hold_action;
static bool s_audio_muted;
static PhoneEventType s_events[8];
static unsigned s_event_count;

void event_put(PebbleEvent *event) {
  cl_assert_equal_i(event->type, PEBBLE_PHONE_EVENT);
  cl_assert(s_event_count < 8);
  s_events[s_event_count++] = event->phone.type;
}

void bt_classic_reset(BtClassicHost *host) {
  memset(host, 0, sizeof(*host));
}
void bt_classic_poll(BtClassicHost *host, uint32_t now) {
  ++s_polls;
}
void hfp_service_wake(void) {
}
void hci_local_audio_set_controls(unsigned volume, bool muted) {
  s_audio_volume = volume;
  s_audio_muted = muted;
}
bool bt_classic_valid_number(const char *number) {
  return number && *number;
}
bool bt_classic_dial(BtClassicHost *host, const char *number) {
  if (!host->status.ready || host->at_pending)
    return false;
  ++s_dials;
  host->at_pending = true;
  return true;
}
bool bt_classic_answer(BtClassicHost *host) {
  if (!host->status.ready || host->at_pending || !host->status.incoming)
    return false;
  ++s_answers;
  host->at_pending = true;
  return true;
}
bool bt_classic_hangup(BtClassicHost *host) {
  if (!host->status.ready || host->at_pending || !host->status.call)
    return false;
  ++s_hangups;
  host->at_pending = true;
  return true;
}
bool bt_classic_set_speaker_gain(BtClassicHost *host, unsigned gain) {
  host->status.speaker_gain = gain;
  host->at_pending = true;
  return true;
}
bool bt_classic_transfer_audio(BtClassicHost *host, bool to_watch) {
  return true;
}
bool bt_classic_call_hold(BtClassicHost *host, unsigned action) {
  if (!host->status.ready || host->at_pending || !host->status.waiting)
    return false;
  s_hold_action = action;
  host->at_pending = true;
  return true;
}

void test_hfp_service__initialize(void) {
  fake_msgq_reset();
  hfp_service_init();
  hfp_service_poll(0);
  s_answers = s_hangups = s_dials = s_polls = 0;
  s_audio_muted = false;
  s_audio_volume = 0;
  s_hold_action = -1;
  s_event_count = 0;
  hfp_service_host()->status.ready = true;
}
void test_hfp_service__cleanup(void) {
  fake_msgq_reset();
  hfp_service_init();
  hfp_service_poll(0);
}

void test_hfp_service__answer_survives_volume_acknowledgement(void) {
  BtClassicHost *host = hfp_service_host();
  host->status.incoming = true;
  cl_assert(hfp_set_speaker_gain(7));
  cl_assert(hfp_answer());
  hfp_service_poll(1);
  cl_assert_equal_i(s_answers, 0);
  cl_assert_equal_i(host->status.errors, 0);
  cl_assert_equal_i(s_audio_volume, 47);
  hfp_service_poll(2);
  cl_assert_equal_i(s_answers, 0);
  cl_assert_equal_i(s_polls, 2);
  host->at_pending = false;
  hfp_service_poll(3);
  cl_assert_equal_i(s_answers, 1);
  cl_assert_equal_i(host->status.errors, 0);
}

void test_hfp_service__call_commands_remain_ordered(void) {
  BtClassicHost *host = hfp_service_host();
  cl_assert(hfp_dial("123"));
  cl_assert(hfp_hangup());
  hfp_service_poll(1);
  cl_assert_equal_i(s_dials, 1);
  cl_assert_equal_i(s_hangups, 0);
  host->status.call = true;
  host->at_pending = false;
  hfp_service_poll(2);
  cl_assert_equal_i(s_hangups, 1);
  cl_assert_equal_i(host->status.errors, 0);
}

void test_hfp_service__ended_call_is_not_answered_after_ack(void) {
  BtClassicHost *host = hfp_service_host();
  host->status.incoming = true;
  host->at_pending = true;
  cl_assert(hfp_answer());
  hfp_service_poll(1);
  host->status.incoming = false;
  host->at_pending = false;
  hfp_service_poll(2);
  cl_assert_equal_i(s_answers, 0);
  cl_assert_equal_i(host->status.errors, 1);
  host->status.incoming = true;
  hfp_service_poll(3);
  cl_assert_equal_i(s_answers, 0);
}

void test_hfp_service__disconnect_retires_deferred_action(void) {
  BtClassicHost *host = hfp_service_host();
  host->status.incoming = true;
  host->at_pending = true;
  cl_assert(hfp_answer());
  hfp_service_poll(1);
  host->status.ready = false;
  hfp_service_poll(2);
  cl_assert_equal_i(s_answers, 0);
  cl_assert_equal_i(host->status.errors, 1);
  host->status.ready = true;
  host->at_pending = false;
  hfp_service_poll(3);
  cl_assert_equal_i(s_answers, 0);
}

void test_hfp_service__local_mute_does_not_wait_for_at(void) {
  BtClassicHost *host = hfp_service_host();
  host->status.call = true;
  host->at_pending = true;
  cl_assert(hfp_set_mic_muted(true));
  hfp_service_poll(1);
  cl_assert(s_audio_muted);
  cl_assert_equal_i(host->status.errors, 0);
}

void test_hfp_service__reject_waiting_preserves_active_call(void) {
  BtClassicHost *host = hfp_service_host();
  host->status.call = host->status.waiting = true;
  cl_assert(hfp_reject());
  hfp_service_poll(1);
  cl_assert_equal_i(s_hold_action, 0);
  cl_assert_equal_i(s_hangups, 0);
}

void test_hfp_service__expired_waiting_reject_never_hangs_up_active_call(void) {
  BtClassicHost *host = hfp_service_host();
  host->status.call = host->status.waiting = true;
  host->at_pending = true;
  cl_assert(hfp_reject());
  hfp_service_poll(1);
  host->status.waiting = false;
  host->at_pending = false;
  hfp_service_poll(2);
  cl_assert_equal_i(s_hold_action, -1);
  cl_assert_equal_i(s_hangups, 0);
  cl_assert_equal_i(host->status.errors, 1);
}

void test_hfp_service__waiting_popup_returns_to_active_without_end(void) {
  BtClassicHost *host = hfp_service_host();
  host->status.call = host->status.waiting = true;
  host->status.incoming = true;
  hfp_service_poll(1);
  cl_assert_equal_i(s_event_count, 1);
  cl_assert_equal_i(s_events[0], PhoneEventType_Incoming);
  host->status.waiting = host->status.incoming = false;
  hfp_service_poll(2);
  cl_assert_equal_i(s_event_count, 2);
  cl_assert_equal_i(s_events[1], PhoneEventType_Start);
  host->status.waiting = host->status.incoming = true;
  hfp_service_poll(3);
  cl_assert_equal_i(s_event_count, 3);
  cl_assert_equal_i(s_events[2], PhoneEventType_Incoming);
}

void test_hfp_service__deferred_hold_rejects_changed_call_state(void) {
  BtClassicHost *host = hfp_service_host();
  host->status.call = host->status.waiting = true;
  hfp_service_poll(1);
  host->at_pending = true;
  cl_assert(hfp_call_hold(1));
  hfp_service_poll(2);
  host->status.waiting = false;
  host->status.call_held = 1;
  host->at_pending = false;
  hfp_service_poll(3);
  cl_assert_equal_i(s_hold_action, -1);
  cl_assert_equal_i(host->status.errors, 1);
}

/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdbool.h>

typedef struct {
  bool available, connected, ready, audio, call, incoming, busy;
  unsigned call_setup, errors;
  unsigned speaker_gain; // HFP absolute gain, 0..15; watch volume still caps output.
  bool mic_muted, audio_pending;
  char detail[64];
  char caller_number[33];
} HfpStatus;

// Internal call service, not an exported SDK API.
void hfp_get_status(HfpStatus *status);
bool hfp_dial(const char *number);
bool hfp_answer(void);
bool hfp_hangup(void);

bool hfp_set_speaker_gain(unsigned gain);
bool hfp_set_mic_muted(bool muted);

bool hfp_transfer_audio(bool to_watch);

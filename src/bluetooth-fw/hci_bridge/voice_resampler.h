/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  int16_t history[31];
  unsigned position;
  bool phase;
} VoiceResampler;

// Zero-initialize between sessions. Converts mono 16 kHz PCM to 8 kHz PCM.
// Returns true for each available output sample.
bool voice_resampler_push(VoiceResampler *state, int16_t input, int16_t *output);

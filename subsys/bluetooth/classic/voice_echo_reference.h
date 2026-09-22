/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include "voice_resampler.h"
#include <stddef.h>

#define VOICE_ECHO_REFERENCE_SAMPLES 1024

typedef struct {
  VoiceResampler resampler;
  int16_t samples[VOICE_ECHO_REFERENCE_SAMPLES];
  uint32_t next_time;
  uint32_t generation;
  unsigned count;
  bool initialized;
} VoiceEchoReference;

// All times use the same wrapping 16 kHz sample clock. Caller serializes access.
// Feed PCM committed to DAC DMA, including silence inserted on underruns.
void voice_echo_reference_push(VoiceEchoReference *state, const int16_t *samples, size_t count,
                               uint32_t sample_time);
bool voice_echo_reference_read(const VoiceEchoReference *state, uint32_t sample_time,
                               int16_t *samples, size_t count);

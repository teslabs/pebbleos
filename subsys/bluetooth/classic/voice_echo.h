/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdint.h>

#define VOICE_ECHO_TAPS 256

// 8 kHz mono NLMS echo model. References must follow the actual DAC timeline.
// Zero-initialize on discontinuities; caller owns synchronization and alignment.
typedef struct {
  float history[VOICE_ECHO_TAPS * 2];
  float foreground[VOICE_ECHO_TAPS], background[VOICE_ECHO_TAPS];
  float energy, step;
  float mic_power, foreground_power, background_power, echo_power;
  float input_energy, output_energy;
  unsigned position, block_samples, better_blocks;
  uint32_t samples, promotions, resets;
} VoiceEcho;

// A separately adapting background model replaces the foreground only after
// sustained error reduction. No residual suppression or microphone gating.
int16_t voice_echo_process(VoiceEcho *state, int16_t reference, int16_t microphone);

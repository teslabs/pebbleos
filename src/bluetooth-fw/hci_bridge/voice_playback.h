/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  int16_t last;
  uint8_t gain; // Zero selects the original 4x gain.
  unsigned fade_out, fade_in;
  bool lost;
  uint32_t clipped, concealed;
} VoicePlayback;

// Zero-initialize between calls. Input/output are signed 16-bit little-endian PCM.
// Count must be even; output has the same size as input.
void voice_playback_process(VoicePlayback *state, const uint8_t *input, uint8_t *output,
                            size_t count, unsigned packet_status);

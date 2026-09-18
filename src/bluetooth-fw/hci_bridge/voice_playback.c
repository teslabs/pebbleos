/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "voice_playback.h"

#define FADE_SAMPLES 8 // 1 ms at 8 kHz.

void voice_playback_process(VoicePlayback *s, const uint8_t *input, uint8_t *output, size_t count,
                            unsigned packet_status) {
  bool lost = packet_status != 0;
  if (lost && !s->lost) {
    s->fade_out = FADE_SAMPLES;
  } else if (!lost && s->lost) {
    s->fade_in = FADE_SAMPLES;
  }
  s->concealed += lost;
  s->lost = lost;
  int32_t last = s->last;
  for (size_t i = 0; i + 1 < count; i += 2) {
    int32_t sample;
    if (lost) {
      // Fade unavailable audio to silence instead of inserting a hard edge.
      sample = s->fade_out ? last * (int32_t)--s->fade_out / FADE_SAMPLES : 0;
    } else {
      sample = (int16_t)(input[i] | (uint16_t)input[i + 1] << 8);
      sample *= s->gain ? s->gain : 4;
      if (s->fade_in) {
        sample = sample * (FADE_SAMPLES - (int32_t)--s->fade_in) / FADE_SAMPLES;
      }
    }
    if (sample > INT16_MAX) {
      sample = INT16_MAX;
      ++s->clipped;
    } else if (sample < INT16_MIN) {
      sample = INT16_MIN;
      ++s->clipped;
    }
    output[i] = (uint16_t)sample & 0xff;
    output[i + 1] = (uint16_t)sample >> 8;
    if (!lost || !s->fade_out) {
      s->last = sample;
    }
  }
}

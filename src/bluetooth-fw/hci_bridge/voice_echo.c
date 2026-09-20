/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "voice_echo.h"

#include <stdbool.h>
#include <string.h>

static float prv_limit(float value, float limit) {
  return value < -limit ? -limit : value > limit ? limit : value;
}

int16_t voice_echo_process(VoiceEcho *s, int16_t reference, int16_t microphone) {
  float x = reference / 32768.0f;
  float d = microphone / 32768.0f;
  unsigned pos = s->position;
  float oldest = s->history[pos];
  s->energy += x * x - oldest * oldest;
  if (s->energy < 0.0f) {
    s->energy = 0.0f;
  }
  s->history[pos] = s->history[pos + VOICE_ECHO_TAPS] = x;
  const float *history = s->history + pos;
  float foreground = 0.0f;
  for (unsigned i = 0; i < VOICE_ECHO_TAPS; ++i) {
    foreground += s->foreground[i] * history[i];
  }
  float error = d - foreground;
  // Keep output at 8 kHz, but spread model training over three sample periods.
  if (s->samples % 3 == 0) {
    float background = 0.0f;
    for (unsigned i = 0; i < VOICE_ECHO_TAPS; ++i) {
      background += s->background[i] * history[i];
    }
    float adapting_error = d - background;
    float step = s->samples ? s->step : 0.35f;
    float update = step * adapting_error / (s->energy + 0.001f);
    if (s->energy > 0.0001f && microphone != INT16_MIN && microphone != INT16_MAX) {
      for (unsigned i = 0; i < VOICE_ECHO_TAPS; ++i) {
        s->background[i] = prv_limit(s->background[i] + update * history[i], 4.0f);
      }
    }
    s->background_power += 3.0f * adapting_error * adapting_error;
  }
  s->position = (pos + VOICE_ECHO_TAPS - 1) % VOICE_ECHO_TAPS;
  s->mic_power += d * d;
  s->foreground_power += error * error;
  s->echo_power += foreground * foreground;
  s->input_energy += d * d;
  s->output_energy += error * error;
  ++s->samples;
  if (++s->block_samples == 80) {
    // Slow adaptation when independent near-end speech dominates the residual.
    s->step = s->promotions && s->foreground_power > s->echo_power * 0.1f ? 0.04f : 0.35f;
    bool better = s->background_power < s->foreground_power * 0.5f &&
                  s->background_power < s->mic_power * 0.25f && s->mic_power > 0.0001f;
    s->better_blocks = better ? s->better_blocks + 1 : 0;
    if (s->better_blocks >= 10) {
      memcpy(s->foreground, s->background, sizeof(s->foreground));
      ++s->promotions;
      s->better_blocks = 0;
    } else if (s->foreground_power > s->mic_power * 2.0f + 0.001f) {
      memset(s->foreground, 0, sizeof(s->foreground));
      memset(s->background, 0, sizeof(s->background));
      s->better_blocks = 0;
      ++s->resets;
    } else if (s->background_power > s->foreground_power * 2.0f + 0.001f) {
      memcpy(s->background, s->foreground, sizeof(s->background));
      s->better_blocks = 0;
    }
    s->mic_power = s->foreground_power = s->background_power = s->echo_power = 0.0f;
    s->block_samples = 0;
  }
  float output = error * 32768.0f;
  return output < -32768.0f ? INT16_MIN : output > 32767.0f ? INT16_MAX : (int16_t)output;
}

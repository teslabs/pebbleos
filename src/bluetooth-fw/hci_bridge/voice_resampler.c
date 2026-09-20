/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "voice_resampler.h"

bool voice_resampler_push(VoiceResampler *s, int16_t input, int16_t *output) {
  // 31-tap Hamming-windowed sinc, 3.6 kHz cutoff at 16 kHz; unity DC gain in Q15.
  static const int16_t coefficients[] = {
    39,    54,    -44,  -138,  34,    323,   72,   -609,  -397,  957,  1133,
    -1296, -2819, 1544, 10175, 14712, 10175, 1544, -2819, -1296, 1133, 957,
    -397,  -609,  72,   323,   34,    -138,  -44,  54,    39,
  };
  s->history[s->position] = input;
  unsigned newest = s->position;
  s->position = (s->position + 1) % 31;
  s->phase = !s->phase;
  if (s->phase) {
    return false;
  }
  int64_t accumulator = 0;
  for (unsigned i = 0; i < 31; ++i) {
    accumulator += (int32_t)coefficients[i] * s->history[(newest + 31 - i) % 31];
  }
  int32_t value = (accumulator + (accumulator >= 0 ? 16384 : -16384)) / 32768;
  if (value > INT16_MAX) {
    value = INT16_MAX;
  } else if (value < INT16_MIN) {
    value = INT16_MIN;
  }
  *output = value;
  return true;
}

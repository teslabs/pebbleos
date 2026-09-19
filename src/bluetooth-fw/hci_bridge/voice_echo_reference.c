/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "voice_echo_reference.h"

void voice_echo_reference_push(VoiceEchoReference *s, const int16_t *samples, size_t count,
                               uint32_t time) {
  if (!s->initialized || time != s->next_time) {
    ++s->generation;
    s->resampler = (VoiceResampler){0};
    s->count = 0;
    s->initialized = true;
  }
  for (size_t i = 0; i < count; ++i, ++time) {
    // Start each decimation pair on an even sample-clock position.
    if (!s->count && !s->resampler.phase && (time & 1)) {
      continue;
    }
    int16_t sample;
    if (voice_resampler_push(&s->resampler, samples[i], &sample)) {
      s->samples[(time / 2) % VOICE_ECHO_REFERENCE_SAMPLES] = sample;
      if (s->count < VOICE_ECHO_REFERENCE_SAMPLES) {
        ++s->count;
      }
    }
  }
  s->next_time = time;
}

bool voice_echo_reference_read(const VoiceEchoReference *s, uint32_t time, int16_t *samples,
                               size_t count) {
  uint32_t end = s->next_time & ~1u;
  uint32_t distance = end - time;
  if (!s->initialized || (time & 1) || count > s->count || distance > s->count * 2 ||
      distance < count * 2) {
    return false;
  }
  for (size_t i = 0; i < count; ++i) {
    samples[i] = s->samples[(time / 2 + i) % VOICE_ECHO_REFERENCE_SAMPLES];
  }
  return true;
}

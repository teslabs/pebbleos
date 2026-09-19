/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include <clar.h>
#include "voice_echo.h"
#include "voice_echo_reference.h"
#include <math.h>
#include <string.h>

static VoiceEcho s_echo;
static uint32_t s_rng;
static int16_t s_reference[512];

static int16_t prv_noise(unsigned scale) {
  s_rng ^= s_rng << 13;
  s_rng ^= s_rng >> 17;
  s_rng ^= s_rng << 5;
  return (int16_t)s_rng / (int)scale;
}

void test_voice_echo__initialize(void) {
  memset(&s_echo, 0, sizeof(s_echo));
  memset(s_reference, 0, sizeof(s_reference));
  s_rng = 0x12345678;
}

static int16_t prv_far_end(unsigned n, unsigned delay) {
  s_reference[n % 512] = prv_noise(4);
  return s_reference[(n + 512 - delay) % 512] / 2 + s_reference[(n + 512 - delay - 13) % 512] / 4 -
         s_reference[(n + 512 - delay - 29) % 512] / 8;
}

void test_voice_echo__converges_on_delayed_echo_without_suppressing_microphone(void) {
  double before = 0, after = 0;
  for (unsigned n = 0; n < 48000; ++n) {
    int16_t mic = prv_far_end(n, 45);
    int16_t out = voice_echo_process(&s_echo, s_reference[n % 512], mic);
    if (n >= 40000) {
      before += (double)mic * mic;
      after += (double)out * out;
    }
  }
  cl_assert(s_echo.promotions > 0);
  cl_assert(after < before / 1000); // More than 30 dB cancellation of a linear echo.
}

void test_voice_echo__preserves_independent_near_end_during_double_talk(void) {
  for (unsigned n = 0; n < 32000; ++n) {
    int16_t mic = prv_far_end(n, 45);
    voice_echo_process(&s_echo, s_reference[n % 512], mic);
  }
  double voice_energy = 0, distortion = 0;
  for (unsigned n = 32000; n < 64000; ++n) {
    int16_t echo = prv_far_end(n, 45);
    int16_t voice = prv_noise(4);
    int16_t out = voice_echo_process(&s_echo, s_reference[n % 512], echo + voice);
    double error = out - voice;
    voice_energy += (double)voice * voice;
    distortion += error * error;
  }
  cl_assert(distortion < voice_energy / 100); // Near-end distortion below -20 dB.
}

void test_voice_echo__silence_reference_passes_microphone_bit_exact(void) {
  for (int sample = INT16_MIN; sample <= INT16_MAX; sample += 127) {
    cl_assert_equal_i(voice_echo_process(&s_echo, 0, sample), sample);
  }
  cl_assert_equal_i(s_echo.promotions, 0);
}

void test_voice_echo__uncorrelated_audio_does_not_enable_cancellation(void) {
  double input = 0, error = 0;
  for (unsigned n = 0; n < 24000; ++n) {
    int16_t reference = prv_noise(4), mic = prv_noise(4);
    int16_t output = voice_echo_process(&s_echo, reference, mic);
    input += (double)mic * mic;
    error += (double)(output - mic) * (output - mic);
  }
  cl_assert_equal_i(s_echo.promotions, 0);
  cl_assert(error < input / 1000);
}

void test_voice_echo__relearns_changed_echo_path(void) {
  double before = 0, after = 0;
  for (unsigned n = 0; n < 80000; ++n) {
    int16_t mic = prv_far_end(n, n < 32000 ? 45 : 95);
    int16_t out = voice_echo_process(&s_echo, s_reference[n % 512], mic);
    if (n >= 72000) {
      before += (double)mic * mic;
      after += (double)out * out;
    }
  }
  cl_assert(after < before / 100);
}

void test_voice_echo__reference_tracks_dma_time_wrap_and_rejects_missing_samples(void) {
  VoiceEchoReference reference = {0};
  int16_t input[512], output[256];
  for (unsigned i = 0; i < 512; ++i) {
    input[i] = 2000;
  }
  uint32_t start = UINT32_MAX - 511;
  voice_echo_reference_push(&reference, input, 512, start);
  cl_assert(voice_echo_reference_read(&reference, start, output, 256));
  cl_assert_equal_i(output[255], 2000);
  cl_assert(!voice_echo_reference_read(&reference, start - 2, output, 256));
  cl_assert(!voice_echo_reference_read(&reference, 0, output, 1));
  voice_echo_reference_push(&reference, input, 512, 0);
  cl_assert(voice_echo_reference_read(&reference, start, output, 256));
  cl_assert(voice_echo_reference_read(&reference, 0, output, 256));
  for (unsigned i = 0; i < 256; ++i) {
    cl_assert_equal_i(output[i], 2000);
  }
  voice_echo_reference_push(&reference, input, 512, 1024); // Missing DMA block.
  cl_assert(!voice_echo_reference_read(&reference, 0, output, 256));
  cl_assert(!voice_echo_reference_read(&reference, 1025, output, 1));
  cl_assert(voice_echo_reference_read(&reference, 1024, output, 256));
}

void test_voice_echo__reference_retention_is_bounded(void) {
  VoiceEchoReference reference = {0};
  int16_t input[512] = {0}, output[256];
  for (unsigned i = 0; i < 10; ++i) {
    voice_echo_reference_push(&reference, input, 512, i * 512);
  }
  cl_assert(!voice_echo_reference_read(&reference, 0, output, 1));
  cl_assert(!voice_echo_reference_read(&reference, 3070, output, 1));
  cl_assert(voice_echo_reference_read(&reference, 3072, output, 256));
  cl_assert(voice_echo_reference_read(&reference, 4608, output, 256));
}

void test_voice_echo__clipped_input_and_reference_silence_do_not_destabilize_model(void) {
  for (unsigned n = 0; n < 16000; ++n) {
    int16_t microphone = n & 1 ? INT16_MIN : INT16_MAX;
    voice_echo_process(&s_echo, prv_noise(1), microphone);
  }
  cl_assert_equal_i(s_echo.promotions, 0);
  for (unsigned n = 0; n < 1024; ++n) {
    int16_t voice = prv_noise(1);
    cl_assert_equal_i(voice_echo_process(&s_echo, 0, voice), voice);
  }
  for (unsigned i = 0; i < VOICE_ECHO_TAPS; ++i) {
    cl_assert(isfinite(s_echo.foreground[i]));
    cl_assert(isfinite(s_echo.background[i]));
  }
}

/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include <clar.h>
#include <math.h>
#include "cvsd.h"
#include <string.h>

void test_cvsd__predictor_matches_specification_equations(void) {
  CvsdPredictor state = {0};
  double prediction = 0, delta = 10;
  unsigned history = 0;
  uint32_t random = 0x451277;
  for (unsigned i = 0; i < 20000; ++i) {
    random = random * 1664525u + 1013904223u;
    unsigned bit = random >> 31;
    history = ((history << 1) | bit) & 15;
    delta =
        history == 0 || history == 15 ? fmin(delta + 10, 1280) : fmax(delta * 1023.0 / 1024.0, 10);
    double output = fmax(-32768, fmin(32767, prediction + (bit ? -delta : delta)));
    cl_assert(fabs(cvsd_decode_bit(&state, bit) - output) < 3);
    prediction = output * 31.0 / 32.0;
  }
}

void test_cvsd__predictor_saturates_and_recovers(void) {
  CvsdPredictor state = {0};
  int16_t output = 0;
  for (unsigned i = 0; i < 2000; ++i) {
    output = cvsd_decode_bit(&state, 0);
  }
  cl_assert_equal_i(output, INT16_MAX);
  for (unsigned i = 0; i < 2000; ++i) {
    output = cvsd_decode_bit(&state, 1);
  }
  cl_assert_equal_i(output, INT16_MIN);
  for (unsigned i = 0; i < 20000; ++i) {
    output = cvsd_decode_bit(&state, i & 1);
  }
  cl_assert(output > -10 && output < 10);
}

void test_cvsd__tone_round_trip_preserves_waveform(void) {
  const unsigned frequencies[] = {425, 1000, 2500};
  for (unsigned f = 0; f < 3; ++f) {
    CvsdCodec encoder = {0}, decoder = {0};
    double sum_sq = 0, sin_sum = 0, cos_sum = 0;
    for (unsigned i = 0; i < 10000; ++i) {
      double phase = 2 * 3.141592653589793 * frequencies[f] * i / 8000;
      int16_t input = 4000 * sin(phase);
      int16_t output = cvsd_decode_sample(&decoder, cvsd_encode_sample(&encoder, input));
      if (i >= 2000) {
        sum_sq += (double)output * output;
        sin_sum += output * sin(phase);
        cos_sum += output * cos(phase);
      }
    }
    double amplitude_sq = 4 * (sin_sum * sin_sum + cos_sum * cos_sum) / (8000.0 * 8000);
    double signal = amplitude_sq / 2;
    double residual = sum_sq / 8000 - signal;
    cl_assert(amplitude_sq > 2500.0 * 2500);
    cl_assert(signal > residual * 100); // At least 20 dB tone-to-distortion ratio.
  }
}

void test_cvsd__byte_order_and_stream_state_are_continuous(void) {
  CvsdCodec decoder = {0};
  CvsdPredictor predictor = {0};
  for (unsigned i = 0; i < 150; ++i) {
    uint8_t byte = i * 37 + 11;
    cvsd_decode_sample(&decoder, byte);
    for (unsigned bit = 0; bit < 8; ++bit) {
      cvsd_decode_bit(&predictor, (byte >> bit) & 1);
    }
    cl_assert_equal_i(decoder.predictor.accumulator, predictor.accumulator);
    cl_assert_equal_i(decoder.predictor.step, predictor.step);
  }
}

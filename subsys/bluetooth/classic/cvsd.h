/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdint.h>

typedef struct {
  int32_t accumulator, step;
  uint8_t recent;
} CvsdPredictor;

#define CVSD_HISTORY_SAMPLES 128

typedef struct {
  CvsdPredictor predictor;
  int16_t history[CVSD_HISTORY_SAMPLES];
  unsigned position;
} CvsdCodec;

// Zero-initialize each direction at link start; preserve state across packets.
// Bit value 1 means a negative step. State uses ten fractional bits.
int16_t cvsd_decode_bit(CvsdPredictor *state, unsigned bit);

// Mono signed 16-bit PCM at 8 kHz; eight 64 kHz CVSD bits per PCM sample.
// Byte bit 0 is earliest on the wire. Codec instances are direction-specific.
uint8_t cvsd_encode_sample(CvsdCodec *state, int16_t pcm);
int16_t cvsd_decode_sample(CvsdCodec *state, uint8_t cvsd);

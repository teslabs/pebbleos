/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include <pbl/btutil/cvsd.h>

// Bluetooth Core, Vol 2, Part B, section 9.2. Independent fixed-point implementation.
#define SCALE 1024

static int16_t prv_saturate(int32_t value) {
  return value > INT16_MAX ? INT16_MAX : value < INT16_MIN ? INT16_MIN : value;
}

int16_t cvsd_decode_bit(CvsdPredictor *s, unsigned bit) {
  if (!s->step) {
    s->step = 10 * SCALE;
  }
  s->recent = ((s->recent << 1) | (bit & 1)) & 15;
  if (s->recent == 0 || s->recent == 15) {
    s->step += 10 * SCALE;
    if (s->step > 1280 * SCALE) {
      s->step = 1280 * SCALE;
    }
  } else {
    s->step = (s->step * 1023 + 512) / 1024;
    if (s->step < 10 * SCALE) {
      s->step = 10 * SCALE;
    }
  }
  int32_t value = s->accumulator + (bit ? -s->step : s->step);
  if (value > INT16_MAX * SCALE) {
    value = INT16_MAX * SCALE;
  } else if (value < INT16_MIN * SCALE) {
    value = INT16_MIN * SCALE;
  }
  s->accumulator = (value * 31 + (value >= 0 ? 16 : -16)) / 32;
  return value / SCALE;
}

// 128-tap Hamming-windowed sinc at 64 kHz, 3.2 kHz cutoff, unity DC gain in Q15.
static const int16_t s_filter[128] = {
  12,   10,   6,    2,    -2,   -8,   -13,  -18, -21, -24,  -23,  -21,  -15,  -6,   6,    20,
  34,   47,   57,   62,   61,   53,   37,   14,  -15, -48,  -81,  -110, -132, -143, -139, -119,
  -82,  -30,  33,   102,  171,  231,  275,  295, 286, 244,  168,  62,   -67,  -210, -353, -482,
  -579, -630, -621, -540, -382, -146, 163,  535, 953, 1397, 1843, 2265, 2639, 2942, 3155, 3263,
  3265, 3155, 2942, 2639, 2265, 1843, 1397, 953, 535, 163,  -146, -382, -540, -621, -630, -579,
  -482, -353, -210, -67,  62,   168,  244,  286, 295, 275,  231,  171,  102,  33,   -30,  -82,
  -119, -139, -143, -132, -110, -81,  -48,  -15, 14,  37,   53,   61,   62,   57,   47,   34,
  20,   6,    -6,   -15,  -21,  -23,  -24,  -21, -18, -13,  -8,   -2,   2,    6,    10,   12,
};

uint8_t cvsd_encode_sample(CvsdCodec *s, int16_t pcm) {
  unsigned newest = s->position;
  s->history[newest] = pcm;
  s->position = (newest + 1) % 16;
  uint8_t byte = 0;
  for (unsigned phase = 0; phase < 8; ++phase) {
    int64_t sum = 0;
    for (unsigned tap = 0; tap < 16; ++tap) {
      sum += (int32_t)s_filter[phase + 8 * tap] * s->history[(newest + 16 - tap) % 16];
    }
    int16_t input = prv_saturate(sum / 4096); // Interpolation gain is eight.
    unsigned bit = (int32_t)input * SCALE < s->predictor.accumulator;
    byte |= bit << phase;
    cvsd_decode_bit(&s->predictor, bit);
  }
  return byte;
}

int16_t cvsd_decode_sample(CvsdCodec *s, uint8_t byte) {
  unsigned newest = 0;
  for (unsigned bit = 0; bit < 8; ++bit) {
    newest = s->position;
    s->history[newest] = cvsd_decode_bit(&s->predictor, (byte >> bit) & 1);
    s->position = (newest + 1) % 128;
  }
  int64_t sum = 0;
  for (unsigned tap = 0; tap < 128; ++tap) {
    sum += (int32_t)s_filter[tap] * s->history[(newest + 128 - tap) % 128];
  }
  return prv_saturate(sum / 32768);
}

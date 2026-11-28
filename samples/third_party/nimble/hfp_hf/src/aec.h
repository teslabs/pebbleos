/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * The echo of a sample gets to the microphone some time after the sample is given to the speaker,
 * the time the two streams hold it for, then lasts for a while. The filter spans that while only,
 * and gets its samples late by a little less than the time before.
 */
#define AEC_DELAY_MS 18U
#define AEC_SPAN_MS  32U
/* Highest sample rate, in Hz */
#define AEC_RATE_MAX  16000U
#define AEC_TAPS_MAX  (AEC_SPAN_MS * AEC_RATE_MAX / 1000U)
#define AEC_DELAY_MAX (AEC_DELAY_MS * AEC_RATE_MAX / 1000U)
/* What the filter leaves is cleaned up in frames of this many samples, half of them new */
#define AEC_FRAME 128U
#define AEC_HOP   (AEC_FRAME / 2U)
#define AEC_BINS  (AEC_FRAME / 2U + 1U)

/*
 * Acoustic echo canceller: an adaptive filter learns how what the speaker plays gets to the
 * microphone, and takes it out of what the microphone captures.
 *
 * One filter adapts all the time, also while someone talks, which misleads it. A second one does
 * the work: it takes over the first whenever that one does better for a while.
 *
 * The filter leaves some echo, and all of what the speaker distorts. That is turned down frequency
 * by frequency, where the echo the filter expects says there is more of it than of anything else,
 * so that someone talking over the echo still gets through. This delays the microphone by a
 * frame.
 */
struct aec {
  size_t taps;
  float adapting[AEC_TAPS_MAX];
  float working[AEC_TAPS_MAX];
  /* The last samples sent to the speaker, twice over so that they are contiguous */
  float x[2U * AEC_TAPS_MAX];
  size_t pos;
  /* Energy of the samples the filters span */
  float x_energy;
  /* Energy of the current window: microphone, and what each filter leaves of it */
  float mic_energy;
  float adapting_energy;
  float working_energy;
  size_t window;
  /* Windows in a row the adapting filter did better */
  uint8_t better;
  /* The working filter took most of the microphone out at some point: it knows the echo */
  bool learned;
  /* Something else than the echo is in the microphone, most likely someone talking */
  bool talking;
  /* The working filter makes things worse: the microphone goes out as it is */
  bool bypass;
  /* Last input samples and output sample, for the filters around the canceller */
  /* Samples of the speaker on their way to the filter */
  float delayed[AEC_DELAY_MAX];
  size_t delay;
  size_t delay_pos;
  float ref_prev;
  float mic_prev;
  float out_prev;
  /* Smoothed power of the microphone, of what the filter leaves and of the output, while the
   * speaker plays */
  float mic_power;
  float filter_power;
  float out_power;
  /* The last frame of what the filter leaves and of the echo it expects, and the output due */
  float frame_err[AEC_FRAME];
  float frame_echo[AEC_FRAME];
  float overlap[AEC_HOP];
  float ready[AEC_HOP];
  size_t hop_pos;
  /* Per frequency: smoothed power of both, and the gain */
  float err_power[AEC_BINS];
  float echo_power[AEC_BINS];
  float gains[AEC_BINS];
  /* How much of the echo the filter expects it leaves behind, as a share of its power */
  float leak;
  /* Working space of the transform: too much for the stack of whoever calls */
  float re[AEC_FRAME];
  float im[AEC_FRAME];
  float echo_re[AEC_FRAME];
  float echo_im[AEC_FRAME];
  /* Tables of the transform */
  float taper[AEC_FRAME];
  float cos_table[AEC_FRAME / 2U];
  float sin_table[AEC_FRAME / 2U];
};

/* Forget what was learned, and work at a sample rate, in Hz, of AEC_RATE_MAX at most. */
void aec_reset(struct aec *aec, uint32_t rate);

/* Take the echo of ref out of mic, in place: the two are the same instants, sample by sample. */
void aec_process(struct aec *aec, const int16_t *ref, int16_t *mic, size_t count);

/* How much weaker the echo comes out than it went in, in dB, over the last second or so. */
int aec_attenuation_db(const struct aec *aec);

/* The share of it the filter takes out by itself, in dB. */
int aec_filter_attenuation_db(const struct aec *aec);

/* The delay of the strongest echo, in samples. */
size_t aec_delay(const struct aec *aec);

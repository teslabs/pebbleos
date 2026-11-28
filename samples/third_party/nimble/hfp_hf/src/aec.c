/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "aec.h"

#include <math.h>
#include <string.h>

/* Step of the adapting filter towards the error of each sample, normalized by the input energy */
#define STEP 0.5f
/* Speech has most of its energy at the bottom: the filter learns faster with it levelled */
#define EMPHASIS 0.9f
/* The same, while someone talks over an echo the filter already knows */
#define STEP_TALKING 0.05f
/* Keeps the step bounded when the speaker is silent */
#define ENERGY_FLOOR 100.0f
/* The filters are compared over windows of this many samples */
#define WINDOW 160U
/* The adapting filter takes over after leaving this share of what the working one leaves... */
#define BETTER_RATIO 0.7f
/* ...this many windows in a row */
#define BETTER_WINDOWS 4U
/* The adapting filter starts again from the working one when it leaves this much more */
#define WORSE_RATIO 4.0f
/* The speaker counts as playing above this energy per sample */
#define REF_ACTIVE (60.0f * 60.0f)
/* Someone talks: the filter leaves more than this share of the microphone */
#define TALKING_RATIO 0.5f
/* Echo only, no one talking: the filter leaves less than this share of the microphone */
#define ECHO_ONLY_RATIO 0.1f
/* Time constant of the figures reported, per sample */
#define POWER_SMOOTH 0.0005f
/* Per frame: smoothing of the powers and of the leak, and how fast a gain may rise */
#define BIN_SMOOTH  0.5f
#define LEAK_SMOOTH 0.05f
#define GAIN_RISE   0.3f
/* Bounds of the leak */
#define LEAK_MIN 0.003f
#define LEAK_MAX 1.0f
/* What the speaker distorts lands on other frequencies than the echo expected: this share of the
 * mean echo power counts at every frequency */
#define SPREAD 0.5f
/* The echo left is taken to be this many times its estimate, and is turned down to this at most */
#define OVERESTIMATE 3.0f
#define GAIN_MIN     0.05f
#define TWO_PI       6.28318530718f

void aec_reset(struct aec *aec, uint32_t rate) {
  memset(aec, 0, sizeof(*aec));
  aec->taps = AEC_SPAN_MS * rate / 1000U;
  aec->delay = AEC_DELAY_MS * rate / 1000U;
  aec->leak = 0.1f;
  for (size_t i = 0U; i < AEC_BINS; i++) {
    aec->gains[i] = 1.0f;
  }
  /* Applied before and after the transform, the window adds up to one across frames */
  for (size_t i = 0U; i < AEC_FRAME; i++) {
    aec->taper[i] = sinf(TWO_PI * ((float)i + 0.5f) / (2.0f * AEC_FRAME));
  }
  for (size_t i = 0U; i < AEC_FRAME / 2U; i++) {
    aec->cos_table[i] = cosf(TWO_PI * (float)i / AEC_FRAME);
    aec->sin_table[i] = sinf(TWO_PI * (float)i / AEC_FRAME);
  }
}

/* In-place transform of AEC_FRAME complex values; inverse without the scaling. */
static void prv_fft(const struct aec *aec, float *re, float *im, bool inverse) {
  for (size_t i = 1U, j = 0U; i < AEC_FRAME; i++) {
    size_t bit = AEC_FRAME >> 1;
    float t;

    for (; (j & bit) != 0U; bit >>= 1) {
      j ^= bit;
    }
    j ^= bit;
    if (i < j) {
      t = re[i];
      re[i] = re[j];
      re[j] = t;
      t = im[i];
      im[i] = im[j];
      im[j] = t;
    }
  }

  for (size_t len = 2U; len <= AEC_FRAME; len <<= 1) {
    const size_t stride = AEC_FRAME / len;

    for (size_t start = 0U; start < AEC_FRAME; start += len) {
      for (size_t k = 0U; k < len / 2U; k++) {
        const float wr = aec->cos_table[k * stride];
        const float wi = inverse ? aec->sin_table[k * stride] : -aec->sin_table[k * stride];
        const size_t a = start + k;
        const size_t b = a + len / 2U;
        const float tr = re[b] * wr - im[b] * wi;
        const float ti = re[b] * wi + im[b] * wr;

        re[b] = re[a] - tr;
        im[b] = im[a] - ti;
        re[a] += tr;
        im[a] += ti;
      }
    }
  }
}

/* Cleans up the last frame of what the filter left; half a frame of output is then ready. */
static void prv_frame(struct aec *aec) {
  float *re = aec->re, *im = aec->im, *echo_re = aec->echo_re, *echo_im = aec->echo_im;
  float err_total = 0.0f, echo_total = 0.0f, spread;
  const bool playing = aec->x_energy > REF_ACTIVE * (float)aec->taps;

  for (size_t i = 0U; i < AEC_FRAME; i++) {
    re[i] = aec->frame_err[i] * aec->taper[i];
    im[i] = 0.0f;
    echo_re[i] = aec->frame_echo[i] * aec->taper[i];
    echo_im[i] = 0.0f;
  }
  prv_fft(aec, re, im, false);
  prv_fft(aec, echo_re, echo_im, false);

  for (size_t k = 0U; k < AEC_BINS; k++) {
    const float err = re[k] * re[k] + im[k] * im[k];
    const float echo = echo_re[k] * echo_re[k] + echo_im[k] * echo_im[k];

    aec->err_power[k] += BIN_SMOOTH * (err - aec->err_power[k]);
    aec->echo_power[k] += BIN_SMOOTH * (echo - aec->echo_power[k]);
    err_total += aec->err_power[k];
    echo_total += aec->echo_power[k];
  }

  /* While only the speaker is heard, all the filter leaves is echo: that is the leak */
  if (playing && aec->learned && !aec->talking && echo_total > 0.0f) {
    const float leak = fminf(fmaxf(err_total / echo_total, LEAK_MIN), LEAK_MAX);

    aec->leak += LEAK_SMOOTH * (leak - aec->leak);
  }

  spread = SPREAD * echo_total / AEC_BINS;
  for (size_t k = 0U; k < AEC_BINS; k++) {
    const float left = OVERESTIMATE * aec->leak * (aec->echo_power[k] + spread);
    float gain = 1.0f;

    if (playing && aec->err_power[k] > 0.0f) {
      gain = fmaxf(1.0f - left / aec->err_power[k], GAIN_MIN);
    }
    /* Down at once, up slowly: the echo outlasts the frame it was expected in */
    if (gain < aec->gains[k]) {
      aec->gains[k] = gain;
    } else {
      aec->gains[k] += GAIN_RISE * (gain - aec->gains[k]);
    }

    re[k] *= aec->gains[k];
    im[k] *= aec->gains[k];
    if (k > 0U && k < AEC_FRAME / 2U) {
      re[AEC_FRAME - k] = re[k];
      im[AEC_FRAME - k] = -im[k];
    }
  }
  prv_fft(aec, re, im, true);

  for (size_t i = 0U; i < AEC_HOP; i++) {
    aec->ready[i] = aec->overlap[i] + re[i] * aec->taper[i] / AEC_FRAME;
    aec->overlap[i] = re[i + AEC_HOP] * aec->taper[i + AEC_HOP] / AEC_FRAME;
  }
}

static void prv_compare(struct aec *aec) {
  const bool playing = aec->x_energy > REF_ACTIVE * (float)aec->taps;

  if (playing && aec->adapting_energy < BETTER_RATIO * aec->working_energy &&
      aec->adapting_energy < aec->mic_energy) {
    if (++aec->better >= BETTER_WINDOWS) {
      memcpy(aec->working, aec->adapting, sizeof(aec->working));
      aec->better = 0U;
    }
  } else {
    aec->better = 0U;
    if (aec->adapting_energy > WORSE_RATIO * aec->working_energy) {
      memcpy(aec->adapting, aec->working, sizeof(aec->adapting));
    }
  }

  if (playing && aec->working_energy < ECHO_ONLY_RATIO * aec->mic_energy) {
    aec->learned = true;
  }
  aec->talking = aec->learned && aec->working_energy > TALKING_RATIO * aec->mic_energy;
  aec->bypass = aec->working_energy > aec->mic_energy;

  aec->mic_energy = 0.0f;
  aec->adapting_energy = 0.0f;
  aec->working_energy = 0.0f;
  aec->window = 0U;
}

void aec_process(struct aec *aec, const int16_t *ref, int16_t *mic, size_t count) {
  for (size_t n = 0U; n < count; n++) {
    const float x_late = aec->delayed[aec->delay_pos];
    const float x_new = x_late - EMPHASIS * aec->ref_prev;
    const float d = (float)mic[n] - EMPHASIS * aec->mic_prev;
    float x_old, y_adapting, y_working, e_adapting, e_working, step, out;
    const float *x;

    aec->delayed[aec->delay_pos] = (float)ref[n];
    aec->delay_pos = (aec->delay_pos + 1U) % aec->delay;
    aec->ref_prev = x_late;
    aec->mic_prev = (float)mic[n];

    /* The newest sample goes in front of the others: x[0] is now, x[i] is i samples ago */
    aec->pos = (aec->pos == 0U) ? aec->taps - 1U : aec->pos - 1U;
    x_old = aec->x[aec->pos + aec->taps];
    aec->x[aec->pos] = x_new;
    aec->x[aec->pos + aec->taps] = x_new;
    x = &aec->x[aec->pos];

    aec->x_energy += x_new * x_new - x_old * x_old;
    if (aec->x_energy < 0.0f) {
      aec->x_energy = 0.0f;
    }

    y_adapting = 0.0f;
    y_working = 0.0f;
    for (size_t i = 0U; i < aec->taps; i++) {
      y_adapting += aec->adapting[i] * x[i];
      y_working += aec->working[i] * x[i];
    }
    e_adapting = d - y_adapting;
    e_working = d - y_working;

    step = (aec->talking ? STEP_TALKING : STEP) * e_adapting /
           (aec->x_energy + ENERGY_FLOOR * (float)aec->taps);
    for (size_t i = 0U; i < aec->taps; i++) {
      aec->adapting[i] += step * x[i];
    }

    aec->mic_energy += d * d;
    aec->adapting_energy += e_adapting * e_adapting;
    aec->working_energy += e_working * e_working;
    if (++aec->window == WINDOW) {
      prv_compare(aec);
    }

    /* A frame is the half kept from the last one and a new half */
    aec->frame_err[AEC_HOP + aec->hop_pos] = aec->bypass ? d : e_working;
    aec->frame_echo[AEC_HOP + aec->hop_pos] = aec->bypass ? 0.0f : y_working;
    out = aec->ready[aec->hop_pos];
    if (++aec->hop_pos == AEC_HOP) {
      prv_frame(aec);
      memmove(aec->frame_err, &aec->frame_err[AEC_HOP], AEC_HOP * sizeof(float));
      memmove(aec->frame_echo, &aec->frame_echo[AEC_HOP], AEC_HOP * sizeof(float));
      aec->hop_pos = 0U;
    }

    if (aec->x_energy > REF_ACTIVE * (float)aec->taps) {
      aec->mic_power += POWER_SMOOTH * (d * d - aec->mic_power);
      aec->filter_power += POWER_SMOOTH * (e_working * e_working - aec->filter_power);
      aec->out_power += POWER_SMOOTH * (out * out - aec->out_power);
    }

    /* Back to the balance the microphone had */
    out += EMPHASIS * aec->out_prev;
    aec->out_prev = out;

    mic[n] = (int16_t)fmaxf(fminf(out, 32767.0f), -32768.0f);
  }
}

int aec_attenuation_db(const struct aec *aec) {
  if (aec->mic_power < 1.0f || aec->out_power < 1.0f) {
    return 0;
  }

  return (int)(10.0f * log10f(aec->mic_power / aec->out_power));
}

int aec_filter_attenuation_db(const struct aec *aec) {
  if (aec->mic_power < 1.0f || aec->filter_power < 1.0f) {
    return 0;
  }

  return (int)(10.0f * log10f(aec->mic_power / aec->filter_power));
}

size_t aec_delay(const struct aec *aec) {
  size_t delay = 0U;

  for (size_t i = 1U; i < aec->taps; i++) {
    if (fabsf(aec->working[i]) > fabsf(aec->working[delay])) {
      delay = i;
    }
  }

  return aec->delay + delay;
}

/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "local_audio.h"
#include "transport.h"
#include "voice_resampler.h"
#include "voice_playback.h"

#include <board/board.h>
#include <console/prompt.h>
#include <pbl/drivers/mic.h>
#include <pbl/kernel/msgq.h>
#include <pbl/kernel/mutex.h>
#include <pbl/services/notifications/alerts_preferences.h>
#include <pbl/services/new_timer/new_timer.h>
#include <pbl/services/speaker/speaker_service.h>
#include <pbl/services/system_task.h>
#include <pbl/util/math.h>
#include <pbl/util/size.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <system/passert.h>

#define NO_HANDLE UINT16_MAX

typedef struct {
  uint32_t generation;
  uint16_t handle;
  uint8_t length;
  uint8_t data[60];
} MicPacket;

static PBL_MUTEX_DEFINE(s_lock);
static PBL_MSGQ_DEFINE(s_capture, sizeof(MicPacket), 8);
static uint16_t s_handle = NO_HANDLE;
static unsigned s_mtu, s_limit, s_credits;
static bool s_flow_requested, s_flow_enabled, s_running, s_mic_owned;
static uint32_t s_generation;
static bool s_sync_pending;

static void prv_sync(void *context);

// Caller holds s_lock; collapse reset/disconnect bursts into the latest state.
static void prv_schedule_sync(void) {
  if (!s_sync_pending) {
    s_sync_pending = true;
    PBL_ASSERTN(system_task_add_callback(prv_sync, NULL));
  }
}
static VoiceResampler s_resampler;
static VoicePlayback s_playback;
static unsigned s_playback_gain = 1;
static unsigned s_speaker_volume = 100;
static bool s_mic_muted;
static int16_t s_mic_buffer[120];
static MicPacket s_partial;
static uint32_t s_rx_bytes, s_played_bytes, s_tx_packets, s_capture_drops, s_start_failures;
static unsigned s_rx_peak, s_hw_error;
static uint32_t s_quiet_bytes, s_bad_bytes;
static uint32_t s_capture_samples, s_produced_samples, s_completed_samples, s_render_samples;
static unsigned s_peak_capture_queue;
#ifdef CONFIG_PROMPT
static TimerID s_pcm_timer;
static uint32_t s_pcm_generation, s_pcm_remaining;
static uint8_t s_sample[8192];
static unsigned s_sample_length;
static bool s_sample_armed;
#endif

static uint16_t prv_u16(const uint8_t *p) {
  return p[0] | (uint16_t)p[1] << 8;
}

static void prv_capture_ready(void *context) {
  hci_bridge_transport_wake();
}

void hci_local_audio_poll(void) {
  mic_poll(MIC);
}

static void prv_capture(int16_t *samples, size_t count, void *context) {
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  if (!s_running) {
    pbl_mutex_unlock(&s_lock);
    return;
  }
  s_capture_samples += count;
  for (size_t i = 0; i < count; ++i) {
    int16_t sample;
    if (!voice_resampler_push(&s_resampler, samples[i], &sample)) {
      continue;
    }
    ++s_produced_samples;
    if (s_mic_muted)
      sample = 0;
    s_partial.data[s_partial.length++] = (uint16_t)sample & 0xff;
    s_partial.data[s_partial.length++] = (uint16_t)sample >> 8;
    if (s_partial.length == s_mtu) {
      s_partial.generation = s_generation;
      s_partial.handle = s_handle;
      if (pbl_msgq_put(&s_capture, &s_partial, PBL_NO_WAIT) != 0) {
        // Drop oldest capture data so desktop stalls cannot build unbounded latency.
        MicPacket discarded;
        pbl_msgq_get(&s_capture, &discarded, PBL_NO_WAIT);
        ++s_capture_drops;
        PBL_ASSERTN(pbl_msgq_put(&s_capture, &s_partial, PBL_NO_WAIT) == 0);
      }
      s_peak_capture_queue = MAX(s_peak_capture_queue, pbl_msgq_num_used(&s_capture));
      s_partial.length = 0;
    }
  }
  pbl_mutex_unlock(&s_lock);
  hci_bridge_transport_wake();
}

// Start/stop run on the system task; the microphone driver serializes capture callbacks.
static void prv_sync(void *context) {
#ifdef CONFIG_PROMPT
  if (s_pcm_timer) {
    new_timer_stop(s_pcm_timer);
  }
  ++s_pcm_generation;
  s_pcm_remaining = 0;
#endif
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  s_sync_pending = false;
  s_running = false;
  uint32_t generation = s_generation;
  bool start = s_handle != NO_HANDLE && s_flow_enabled && s_limit && s_mtu >= 2;
  unsigned volume = s_speaker_volume;
  pbl_msgq_purge(&s_capture);
  s_resampler = (VoiceResampler){0};
  s_playback = (VoicePlayback){.gain = s_playback_gain};
  s_partial = (MicPacket){0};
  if (start) {
    s_capture_samples = s_produced_samples = s_completed_samples = s_render_samples = 0;
    s_peak_capture_queue = 0;
  }
  pbl_mutex_unlock(&s_lock);

  if (s_mic_owned) {
    mic_stop(MIC);
    s_mic_owned = false;
  }
  speaker_service_stop_for_task(PebbleTask_BTHCI);
  if (!start) {
    return;
  }
  bool opened = speaker_service_stream_open_realtime_owned(
      SpeakerPriorityNotification, volume, SpeakerPcmFormat_8kHz_16bit, PebbleTask_BTHCI);
  if (opened) {
    // One 32 ms refill of headroom absorbs the 3.75 ms SCO packet cadence.
    static const uint8_t silence[512];
    speaker_service_stream_write_owned(PebbleTask_BTHCI, silence, sizeof(silence));
  }
  if (opened && mic_get_channels(MIC) == 1) {
    s_mic_owned = mic_start_polling(MIC, prv_capture, NULL, s_mic_buffer,
                                    ARRAY_LENGTH(s_mic_buffer), prv_capture_ready);
    if (!s_mic_owned) {
      s_mic_owned = mic_start(MIC, prv_capture, NULL, s_mic_buffer, ARRAY_LENGTH(s_mic_buffer));
    }
  }
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  bool ready = opened && s_mic_owned && generation == s_generation && s_handle != NO_HANDLE;
  s_running = ready;
  if (ready)
    speaker_service_set_volume_owned(PebbleTask_BTHCI, s_speaker_volume);
  s_start_failures += !ready;
  pbl_mutex_unlock(&s_lock);
  if (!ready) {
    if (s_mic_owned) {
      mic_stop(MIC);
      s_mic_owned = false;
    }
    speaker_service_stop_for_task(PebbleTask_BTHCI);
  }
}

static void prv_retire(void) {
  s_handle = NO_HANDLE;
  s_running = false;
  s_credits = 0;
  ++s_generation;
  pbl_msgq_purge(&s_capture);
}

void hci_local_audio_set_controls(unsigned speaker_volume, bool mic_muted) {
  if (speaker_volume > 100)
    return;
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  if (s_speaker_volume != speaker_volume) {
    s_speaker_volume = speaker_volume;
    if (s_running)
      speaker_service_set_volume_owned(PebbleTask_BTHCI, speaker_volume);
  }
  if (s_mic_muted != mic_muted) {
    s_mic_muted = mic_muted;
    pbl_msgq_purge(&s_capture);
    s_partial.length = 0;
    s_resampler = (VoiceResampler){0};
  }
  pbl_mutex_unlock(&s_lock);
}

void hci_local_audio_stop(void) {
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  prv_retire();
  prv_schedule_sync();
  pbl_mutex_unlock(&s_lock);
}

void hci_local_audio_command(const uint8_t *p, size_t length) {
  if (length < 4 || p[0] != 1) {
    return;
  }
  bool sync = false;
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  if (prv_u16(p + 1) == 0x0c03) {
    prv_retire();
    s_flow_enabled = s_flow_requested = false;
    s_hw_error = 0;
    sync = true;
  } else if (prv_u16(p + 1) == 0x0c2f && length == 5) {
    s_flow_requested = p[4] == 1;
  }
  if (sync) {
    prv_schedule_sync();
  }
  pbl_mutex_unlock(&s_lock);
}

bool hci_local_audio_receive(const uint8_t *p, size_t length) {
  if (length < 3) {
    return true;
  }
  bool sync = false;
  bool forward = true;
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  if (p[0] == 3 && length >= 4 && length == (size_t)p[3] + 4) {
    forward = false;
    if (s_running && (prv_u16(p + 1) & 0x0fff) == s_handle && !(p[3] & 1)) {
      s_rx_bytes += p[3];
      s_render_samples += p[3] / 2;
      unsigned peak = 0;
      for (size_t i = 4; i < length; i += 2) {
        int sample = (int16_t)prv_u16(p + i);
        unsigned magnitude = sample < 0 ? -sample : sample;
        peak = MAX(peak, magnitude);
      }
      s_rx_peak = MAX(s_rx_peak, peak);
      s_quiet_bytes += peak < 32 ? p[3] : 0;
      s_bad_bytes += p[2] & 0x30 ? p[3] : 0;
#ifdef CONFIG_PROMPT
      if (s_sample_armed && (s_sample_length || (peak >= 128 && !(p[2] & 0x30)))) {
        if (length <= sizeof(s_sample) - s_sample_length) {
          memcpy(s_sample + s_sample_length, p, length);
          s_sample_length += length;
        }
        if (sizeof(s_sample) - s_sample_length < length) {
          s_sample_armed = false;
        }
      }
#endif
      uint8_t playback[254];
      voice_playback_process(&s_playback, p + 4, playback, p[3], (p[2] >> 4) & 3);
      s_played_bytes += speaker_service_stream_write_owned(PebbleTask_BTHCI, playback, p[3]);
    }
  } else if (p[0] == 4 && length == (size_t)p[2] + 3) {
    if (p[1] == 0x10 && length == 4) {
#ifdef CONFIG_PROMPT
      s_sample_armed = false;
#endif
      s_hw_error = p[3];
      prv_retire();
      sync = true;
    } else if (p[1] == 0x2c && length == 20 && p[3] == 0 && p[19] == 2) {
      prv_retire();
      s_handle = prv_u16(p + 4) & 0x0fff;
      s_credits = s_limit;
      sync = true;
    } else if (p[1] == 5 && length == 7 && p[3] == 0 && prv_u16(p + 4) == s_handle) {
#ifdef CONFIG_PROMPT
      s_sample_armed = false;
#endif
      prv_retire();
      sync = true;
    } else if (p[1] == 0x13 && length >= 4 && length == 4u + 4u * p[3]) {
      forward = false;
      for (size_t i = 4; i < length; i += 4) {
        if (prv_u16(p + i) == s_handle) {
          unsigned completed = prv_u16(p + i + 2);
          s_completed_samples += completed * s_mtu / 2;
          s_credits = MIN(s_limit, s_credits + completed);
        } else {
          forward = true;
        }
      }
    } else if (p[1] == 0x0e && length >= 7 && p[6] == 0) {
      uint16_t opcode = prv_u16(p + 4);
      if (opcode == 0x1005 && length == 14) {
        s_mtu = MIN(60, p[9]) & ~1u;
        s_limit = prv_u16(p + 12);
      } else if (opcode == 0x0c2f) {
        s_flow_enabled = s_flow_requested;
      } else if (opcode == 0x0c03) {
        prv_retire();
        sync = true;
      }
    }
  }
  if (sync) {
    prv_schedule_sync();
  }
  pbl_mutex_unlock(&s_lock);
  return forward;
}

size_t hci_local_audio_transmit(uint8_t packet[64]) {
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  MicPacket captured;
  size_t length = 0;
  while (s_running && s_credits && pbl_msgq_get(&s_capture, &captured, PBL_NO_WAIT) == 0) {
    if (captured.generation != s_generation || captured.handle != s_handle) {
      continue;
    }
    packet[0] = 3;
    packet[1] = s_handle & 0xff;
    packet[2] = s_handle >> 8;
    packet[3] = captured.length;
    if (s_mic_muted)
      memset(packet + 4, 0, captured.length);
    else
      memcpy(packet + 4, captured.data, captured.length);
    --s_credits;
    ++s_tx_packets;
    length = captured.length + 4;
    break;
  }
  pbl_mutex_unlock(&s_lock);
  return length;
}

#ifdef CONFIG_PROMPT
void hci_local_audio_report(void) {
  char buffer[160];
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  // Format under the lock, then release before any PULSE writes.
  snprintf(
      buffer, sizeof(buffer),
      "local audio active=%u handle=%u rx_bytes=%lu queued_bytes=%lu tx=%lu drops=%lu failures=%lu",
      s_running, s_handle, (unsigned long)s_rx_bytes, (unsigned long)s_played_bytes,
      (unsigned long)s_tx_packets, (unsigned long)s_capture_drops, (unsigned long)s_start_failures);
  char capture[160];
  snprintf(capture, sizeof(capture),
           "local capture raw=%lu produced=%lu completed=%lu rendered=%lu queued=%u peak_queue=%u",
           (unsigned long)s_capture_samples, (unsigned long)s_produced_samples,
           (unsigned long)s_completed_samples, (unsigned long)s_render_samples,
           (unsigned)pbl_msgq_num_used(&s_capture), s_peak_capture_queue);
  unsigned peak = s_rx_peak;
  unsigned call_volume = s_speaker_volume;
  bool mic_muted = s_mic_muted;
  unsigned hardware_error = s_hw_error;
  VoicePlayback playback = s_playback;
  unsigned sample_length = s_sample_length;
  uint32_t quiet_bytes = s_quiet_bytes, bad_bytes = s_bad_bytes;
  bool sample_armed = s_sample_armed;
  s_rx_peak = 0;
  pbl_mutex_unlock(&s_lock);
  prompt_send_response(buffer);
  prompt_send_response(capture);
  snprintf(buffer, sizeof(buffer),
           "local speaker peak=%u muted=%u volume=%u hardware_error=%u call_volume=%u mic_muted=%u",
           peak, speaker_service_is_muted(), alerts_preferences_get_speaker_volume(),
           hardware_error, call_volume, mic_muted);
  prompt_send_response(buffer);
  snprintf(buffer, sizeof(buffer), "local playback gain=%u concealed=%lu clipped=%lu",
           playback.gain, (unsigned long)playback.concealed, (unsigned long)playback.clipped);
  prompt_send_response(buffer);
  snprintf(buffer, sizeof(buffer), "local signal quiet_bytes=%lu bad_bytes=%lu",
           (unsigned long)quiet_bytes, (unsigned long)bad_bytes);
  prompt_send_response(buffer);
  snprintf(buffer, sizeof(buffer), "local sample bytes=%u armed=%u", sample_length, sample_armed);
  prompt_send_response(buffer);
}

void command_bt_audio_gain(const char *argument) {
  char *end;
  long gain = strtol(argument, &end, 10);
  if (end == argument || *end || gain < 1 || gain > 16) {
    prompt_send_response("Gain must be 1..16");
    return;
  }
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  s_playback_gain = gain;
  s_playback.gain = gain;
  pbl_mutex_unlock(&s_lock);
}

void command_bt_audio_capture(void) {
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  s_sample_length = 0;
  s_sample_armed = true;
  pbl_mutex_unlock(&s_lock);
}

void command_bt_audio_dump(void) {
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  s_sample_armed = false;
  unsigned length = s_sample_length;
  pbl_mutex_unlock(&s_lock);
  // Capture is frozen; never hold the audio lock while writing to PULSE.
  for (unsigned offset = 0; offset < length; offset += 64) {
    char line[134] = "pcm: ";
    unsigned count = MIN(64, length - offset);
    for (unsigned i = 0; i < count; ++i) {
      snprintf(line + 5 + 2 * i, 3, "%02x", s_sample[offset + i]);
    }
    prompt_send_response(line);
  }
}

static void prv_speaker_test(void *context) {
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  bool idle = s_handle == NO_HANDLE;
  pbl_mutex_unlock(&s_lock);
  // Never preempt an active call with a diagnostic tone.
  if (idle) {
    speaker_service_play_tone(660, 600, SpeakerWaveformSine, 32, SpeakerPriorityNotification, 100);
  }
}

void command_bt_audio_speaker_test(void) {
  PBL_ASSERTN(system_task_add_callback(prv_speaker_test, NULL));
}

static void prv_pcm_feed(void *context);

static void prv_pcm_tick(void *context) {
  PBL_ASSERTN(system_task_add_callback(prv_pcm_feed, context));
}

static void prv_pcm_feed(void *context) {
  if ((uintptr_t)context != s_pcm_generation || !s_pcm_remaining) {
    return;
  }
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  bool idle = s_handle == NO_HANDLE;
  pbl_mutex_unlock(&s_lock);
  if (!idle) {
    s_pcm_remaining = 0;
    return;
  }
  // Keep 200 ms of headroom while feeding five seconds of 1 kHz PCM.
  unsigned budget = s_pcm_remaining == 80000 ? 3200 : 1600;
  static const int16_t sine[] = {0, 5793, 8192, 5793, 0, -5793, -8192, -5793};
  uint8_t pcm[128];
  for (unsigned i = 0; i < 64; ++i) {
    pcm[2 * i] = (uint16_t)sine[i % 8] & 0xff;
    pcm[2 * i + 1] = (uint16_t)sine[i % 8] >> 8;
  }
  while (budget && s_pcm_remaining) {
    unsigned count = MIN(sizeof(pcm), MIN(budget, s_pcm_remaining));
    if (speaker_service_stream_write_owned(PebbleTask_BTHCI, pcm, count) != count) {
      s_pcm_remaining = 0;
      speaker_service_stop_for_task(PebbleTask_BTHCI);
      return;
    }
    budget -= count;
    s_pcm_remaining -= count;
  }
  if (s_pcm_remaining) {
    PBL_ASSERTN(new_timer_start(s_pcm_timer, 100, prv_pcm_tick, context, 0));
  } else {
    speaker_service_stream_close_owned(PebbleTask_BTHCI);
  }
}

static void prv_pcm_test(void *context) {
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  bool idle = s_handle == NO_HANDLE;
  pbl_mutex_unlock(&s_lock);
  if (!idle || s_pcm_remaining ||
      !speaker_service_stream_open_owned(SpeakerPriorityNotification, 100,
                                         SpeakerPcmFormat_8kHz_16bit, PebbleTask_BTHCI)) {
    return;
  }
  if (!s_pcm_timer) {
    s_pcm_timer = new_timer_create();
    PBL_ASSERTN(s_pcm_timer);
  }
  ++s_pcm_generation;
  s_pcm_remaining = 80000;
  prv_pcm_feed((void *)(uintptr_t)s_pcm_generation);
}

void command_bt_audio_pcm_test(void) {
  PBL_ASSERTN(system_task_add_callback(prv_pcm_test, NULL));
}

#endif

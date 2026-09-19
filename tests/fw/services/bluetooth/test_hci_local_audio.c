/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include <clar.h>
#include <pbl/drivers/mic.h>
#include <pbl/services/new_timer/new_timer.h>
#include <pbl/services/speaker/speaker_service.h>
#include <string.h>

#include "local_audio.h"
#include "voice_resampler.h"
#include "voice_playback.h"
#include "stubs_passert.h"
#include "fake_mutex.h"
#include "fake_msgq.h"
#include "fake_system_task.h"

static MicDataHandlerCB s_capture;
static void *s_capture_context;
static bool s_mic_busy, s_speaker_open;
static unsigned s_starts, s_stops, s_speaker_bytes, s_tones, s_stream_closes;

extern void command_bt_audio_speaker_test(void);
extern void command_bt_audio_pcm_test(void);
extern void command_bt_audio_capture(void);
extern void command_bt_audio_dump(void);
static unsigned s_dump_bytes;

static NewTimerCallback s_timer_cb;
static void *s_timer_data;

TimerID new_timer_create(void) {
  return 1;
}
bool new_timer_start(TimerID timer, uint32_t timeout, NewTimerCallback cb, void *data,
                     uint32_t flags) {
  cl_assert_equal_i(timeout, 100);
  cl_assert_equal_i(flags, 0);
  s_timer_cb = cb;
  s_timer_data = data;
  return true;
}
bool new_timer_stop(TimerID timer) {
  s_timer_cb = NULL;
  return true;
}

void speaker_service_stream_close_owned(PebbleTask owner) {
  cl_assert_equal_i(owner, PebbleTask_BTHCI);
  ++s_stream_closes;
}

bool speaker_service_is_muted(void) {
  return false;
}
uint8_t alerts_preferences_get_speaker_volume(void) {
  return 100;
}
bool speaker_service_play_tone(uint16_t frequency, uint16_t duration, uint8_t waveform,
                               uint8_t velocity, SpeakerPriority priority, uint8_t volume) {
  ++s_tones;
  return true;
}

bool mic_start(MicDevice *device, MicDataHandlerCB cb, void *context, int16_t *buffer,
               size_t size) {
  if (s_mic_busy) {
    return false;
  }
  s_mic_busy = true;
  s_capture = cb;
  s_capture_context = context;
  ++s_starts;
  return true;
}
void mic_stop(MicDevice *device) {
  s_capture = NULL;
  s_mic_busy = false;
  ++s_stops;
}
uint32_t mic_get_channels(MicDevice *device) {
  return 1;
}
bool speaker_service_stream_open_owned(SpeakerPriority priority, uint8_t volume,
                                       SpeakerPcmFormat format, PebbleTask owner) {
  cl_assert_equal_i(format, SpeakerPcmFormat_8kHz_16bit);
  cl_assert_equal_i(volume, 100);
  cl_assert_equal_i(owner, PebbleTask_BTHCI);
  s_speaker_open = true;
  return true;
}
uint32_t speaker_service_stream_write_owned(PebbleTask owner, const void *data, uint32_t size) {
  cl_assert_equal_i(owner, PebbleTask_BTHCI);
  if (!s_speaker_open) {
    return 0;
  }
  s_speaker_bytes += size;
  return size;
}
void speaker_service_stop_for_task(PebbleTask task) {
  s_speaker_open = false;
}
void hci_bridge_transport_wake(void) {
}
void prompt_send_response(const char *response) {
  if (!strncmp(response, "pcm: ", 5)) {
    s_dump_bytes += (strlen(response) - 5) / 2;
  }
}

void test_hci_local_audio__initialize(void) {
  const uint8_t reset[] = {1, 3, 0x0c, 0};
  hci_local_audio_command(reset, sizeof(reset));
  fake_system_task_callbacks_invoke_pending();
  fake_msgq_reset();
  command_bt_audio_dump();
  s_dump_bytes = 0;
  s_mic_busy = false;
  s_capture = NULL;
  s_starts = s_stops = s_speaker_bytes = s_tones = s_stream_closes = 0;
}
void test_hci_local_audio__cleanup(void) {
  const uint8_t reset[] = {1, 3, 0x0c, 0};
  hci_local_audio_command(reset, sizeof(reset));
  fake_system_task_callbacks_invoke_pending();
  fake_system_task_callbacks_cleanup();
  fake_msgq_reset();
}

static void prv_connect(bool flow) {
  const uint8_t buffers[] = {4, 0x0e, 11, 1, 5, 0x10, 0, 0xfd, 3, 60, 4, 0, 7, 0};
  hci_local_audio_receive(buffers, sizeof(buffers));
  if (flow) {
    const uint8_t command[] = {1, 0x2f, 0x0c, 1, 1};
    const uint8_t complete[] = {4, 0x0e, 4, 1, 0x2f, 0x0c, 0};
    hci_local_audio_command(command, sizeof(command));
    hci_local_audio_receive(complete, sizeof(complete));
  }
  const uint8_t connection[] = {
    4, 0x2c, 17, 0, 0x80, 1, 1, 2, 3, 4, 5, 6, 2, 6, 2, 30, 0, 30, 0, 2,
  };
  hci_local_audio_receive(connection, sizeof(connection));
  fake_system_task_callbacks_invoke_pending();
}

static void prv_capture_frame(void) {
  int16_t input[120];
  for (unsigned i = 0; i < 120; ++i) {
    input[i] = 800;
  }
  cl_assert(s_capture);
  s_capture(input, 120, s_capture_context);
}

void test_hci_local_audio__speaker_and_microphone_use_standard_hci(void) {
  prv_connect(true);
  cl_assert_equal_i(s_starts, 1);
  uint8_t packet[64] = {3, 0x80, 1, 60};
  cl_assert(!hci_local_audio_receive(packet, sizeof(packet)));
  cl_assert_equal_i(s_speaker_bytes, 512 + 60);
  prv_capture_frame();
  cl_assert_equal_i(hci_local_audio_transmit(packet), 64);
  cl_assert_equal_i(packet[0], 3);
  cl_assert_equal_i(packet[1], 0x80);
  cl_assert_equal_i(packet[3], 60);
  cl_assert_equal_i(hci_local_audio_transmit(packet), 64);
  cl_assert_equal_i((int16_t)(packet[4] | packet[5] << 8), 800);
  cl_assert_equal_i(hci_local_audio_transmit(packet), 0);
}

void test_hci_local_audio__credits_and_capture_backlog_are_bounded(void) {
  prv_connect(true);
  for (unsigned i = 0; i < 20; ++i) {
    prv_capture_frame();
  }
  uint8_t packet[64];
  for (unsigned i = 0; i < 7; ++i) {
    cl_assert_equal_i(hci_local_audio_transmit(packet), 64);
  }
  cl_assert_equal_i(hci_local_audio_transmit(packet), 0);
  const uint8_t completed[] = {4, 0x13, 5, 1, 0x80, 1, 7, 0};
  cl_assert(!hci_local_audio_receive(completed, sizeof(completed)));
  cl_assert_equal_i(hci_local_audio_transmit(packet), 64);
  cl_assert_equal_i(hci_local_audio_transmit(packet), 0);
}

void test_hci_local_audio__acl_events_continue_to_external_host(void) {
  prv_connect(true);
  const uint8_t acl_completed[] = {4, 0x13, 5, 1, 1, 0, 2, 0};
  cl_assert(hci_local_audio_receive(acl_completed, sizeof(acl_completed)));
  const uint8_t mixed[] = {4, 0x13, 9, 2, 1, 0, 2, 0, 0x80, 1, 2, 0};
  cl_assert(hci_local_audio_receive(mixed, sizeof(mixed)));
}

void test_hci_local_audio__disconnect_retires_old_capture_before_async_stop(void) {
  prv_connect(true);
  prv_capture_frame();
  const uint8_t disconnect[] = {4, 5, 4, 0, 0x80, 1, 0x13};
  hci_local_audio_receive(disconnect, sizeof(disconnect));
  uint8_t packet[64];
  cl_assert_equal_i(hci_local_audio_transmit(packet), 0);
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(s_stops, 1);
  cl_assert(!s_speaker_open);
  prv_connect(true);
  cl_assert_equal_i(hci_local_audio_transmit(packet), 0);
  prv_capture_frame();
  cl_assert_equal_i(hci_local_audio_transmit(packet), 64);
}

void test_hci_local_audio__microphone_in_use_is_not_stolen(void) {
  s_mic_busy = true;
  prv_connect(true);
  cl_assert_equal_i(s_starts, 0);
  cl_assert_equal_i(s_stops, 0);
  cl_assert(s_mic_busy);
  cl_assert(!s_speaker_open);
}

void test_hci_local_audio__hardware_error_stops_audio(void) {
  prv_connect(true);
  prv_capture_frame();
  const uint8_t error[] = {4, 0x10, 1, 45};
  cl_assert(hci_local_audio_receive(error, sizeof(error)));
  uint8_t packet[64];
  cl_assert_equal_i(hci_local_audio_transmit(packet), 0);
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(s_stops, 1);
  cl_assert(!s_speaker_open);
}

void test_hci_local_audio__diagnostic_tone_does_not_preempt_call(void) {
  command_bt_audio_speaker_test();
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(s_tones, 1);
  prv_connect(true);
  command_bt_audio_speaker_test();
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(s_tones, 1);
}

void test_hci_local_audio__pcm_diagnostic_is_bounded_and_does_not_preempt_call(void) {
  command_bt_audio_pcm_test();
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(s_speaker_bytes, 3200);
  unsigned ticks = 0;
  while (s_timer_cb && ticks++ < 100) {
    NewTimerCallback cb = s_timer_cb;
    s_timer_cb = NULL;
    cb(s_timer_data);
    fake_system_task_callbacks_invoke_pending();
  }
  cl_assert_equal_i(ticks, 48);
  cl_assert_equal_i(s_speaker_bytes, 80000);
  cl_assert_equal_i(s_stream_closes, 1);
  prv_connect(true);
  unsigned bytes = s_speaker_bytes;
  command_bt_audio_pcm_test();
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(s_speaker_bytes, bytes);
  cl_assert_equal_i(s_stream_closes, 1);
}

void test_hci_local_audio__call_cancels_pending_pcm_test_refill(void) {
  command_bt_audio_pcm_test();
  fake_system_task_callbacks_invoke_pending();
  cl_assert(s_timer_cb);
  NewTimerCallback stale = s_timer_cb;
  void *stale_data = s_timer_data;
  prv_connect(true);
  unsigned bytes = s_speaker_bytes;
  stale(stale_data);
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(s_speaker_bytes, bytes);
  cl_assert_equal_i(s_stream_closes, 0);
}

void test_hci_local_audio__flow_control_is_required_before_capture(void) {
  prv_connect(false);
  cl_assert_equal_i(s_starts, 0);
  cl_assert(!s_speaker_open);
}

void test_hci_local_audio__resampler_preserves_dc_and_rejects_aliasing(void) {
  VoiceResampler state = {0};
  int16_t output = 0;
  unsigned outputs = 0;
  for (unsigned i = 0; i < 200; ++i) {
    if (voice_resampler_push(&state, 12000, &output)) {
      ++outputs;
      if (i > 32) {
        cl_assert_equal_i(output, 12000);
      }
    }
  }
  cl_assert_equal_i(outputs, 100);
  state = (VoiceResampler){0};
  const int16_t alias_tone[] = {0, 8485, -12000, 8485, 0, -8485, 12000, -8485};
  for (unsigned i = 0; i < 200; ++i) {
    if (voice_resampler_push(&state, alias_tone[i % 8], &output) && i > 32) {
      cl_assert(output > -150 && output < 150);
    }
  }
}

static int16_t prv_sample(const uint8_t *data, unsigned index) {
  return (int16_t)(data[index * 2] | (uint16_t)data[index * 2 + 1] << 8);
}

void test_hci_local_audio__playback_gain_saturates_without_wrapping(void) {
  VoicePlayback state = {0};
  const uint8_t input[] = {0xe8, 3, 0x18, 0xfc, 0xff, 0x7f, 0, 0x80};
  uint8_t output[sizeof(input)];
  voice_playback_process(&state, input, output, sizeof(input), 0);
  cl_assert_equal_i(prv_sample(output, 0), 4000);
  cl_assert_equal_i(prv_sample(output, 1), -4000);
  cl_assert_equal_i(prv_sample(output, 2), INT16_MAX);
  cl_assert_equal_i(prv_sample(output, 3), INT16_MIN);
  cl_assert_equal_i(state.clipped, 2);
}

void test_hci_local_audio__quiet_ringback_gain_preserves_both_polarities(void) {
  VoicePlayback state = {.gain = 16};
  const uint8_t input[] = {0xe8, 3, 0x18, 0xfc, 0xff, 0x7f, 0, 0x80};
  uint8_t output[sizeof(input)];
  voice_playback_process(&state, input, output, sizeof(input), 0);
  cl_assert_equal_i(prv_sample(output, 0), 16000);
  cl_assert_equal_i(prv_sample(output, 1), -16000);
  cl_assert_equal_i(prv_sample(output, 2), INT16_MAX);
  cl_assert_equal_i(prv_sample(output, 3), INT16_MIN);
  cl_assert_equal_i(state.clipped, 2);
}

void test_hci_local_audio__lost_playback_fades_out_and_recovery_fades_in(void) {
  VoicePlayback state = {0};
  uint8_t input[60], output[60];
  for (unsigned i = 0; i < sizeof(input); i += 2) {
    input[i] = 0xe8;
    input[i + 1] = 3;
  }
  voice_playback_process(&state, input, output, sizeof(input), 0);
  voice_playback_process(&state, input, output, sizeof(input), 2);
  for (unsigned i = 0; i < 30; ++i) {
    cl_assert_equal_i(prv_sample(output, i), i < 8 ? 4000 * (7 - i) / 8 : 0);
  }
  voice_playback_process(&state, input, output, sizeof(input), 3);
  for (unsigned i = 0; i < 30; ++i) {
    cl_assert_equal_i(prv_sample(output, i), 0);
  }
  voice_playback_process(&state, input, output, sizeof(input), 0);
  for (unsigned i = 0; i < 30; ++i) {
    cl_assert_equal_i(prv_sample(output, i), i < 8 ? 4000 * (i + 1) / 8 : 4000);
  }
  cl_assert_equal_i(state.concealed, 2);
}

void test_hci_local_audio__playback_fade_survives_packet_boundaries(void) {
  VoicePlayback whole = {.last = -8000}, split = whole;
  uint8_t input[60] = {0}, expected[60], actual[60];
  voice_playback_process(&whole, input, expected, sizeof(input), 1);
  for (unsigned i = 0; i < sizeof(input); i += 2) {
    voice_playback_process(&split, input + i, actual + i, 2, 1);
  }
  cl_assert_equal_m(expected, actual, sizeof(actual));
}

void test_hci_local_audio__negative_fades_preserve_sign_and_do_not_clip(void) {
  VoicePlayback state = {.last = -4000};
  uint8_t input[60], output[60];
  for (unsigned i = 0; i < sizeof(input); i += 2) {
    input[i] = 0x18;
    input[i + 1] = 0xfc; // -1000, or -4000 after gain.
  }
  voice_playback_process(&state, input, output, sizeof(input), 2);
  for (int i = 0; i < 30; ++i) {
    cl_assert_equal_i(prv_sample(output, i), i < 8 ? -4000 * (7 - i) / 8 : 0);
  }
  voice_playback_process(&state, input, output, sizeof(input), 0);
  for (int i = 0; i < 30; ++i) {
    cl_assert_equal_i(prv_sample(output, i), i < 8 ? -4000 * (i + 1) / 8 : -4000);
  }
  cl_assert_equal_i(state.clipped, 0);
}

void test_hci_local_audio__explicit_capture_waits_for_audio_and_is_bounded(void) {
  prv_connect(true);
  uint8_t packet[64] = {3, 0x80, 1, 60};
  command_bt_audio_capture();
  for (unsigned i = 0; i < 200; ++i) {
    hci_local_audio_receive(packet, sizeof(packet));
  }
  command_bt_audio_dump();
  cl_assert_equal_i(s_dump_bytes, 0);
  command_bt_audio_capture();
  packet[5] = 1;
  for (unsigned i = 0; i < 200; ++i) {
    hci_local_audio_receive(packet, sizeof(packet));
  }
  command_bt_audio_dump();
  cl_assert_equal_i(s_dump_bytes, 8192);
}

void test_hci_local_audio__repeated_stop_queues_only_one_audio_teardown(void) {
  prv_connect(true);
  prv_capture_frame();
  for (unsigned i = 0; i < 100; ++i) {
    hci_local_audio_stop();
  }
  cl_assert_equal_i(list_count(s_system_task_callback_head), 1);
  uint8_t packet[64];
  cl_assert_equal_i(hci_local_audio_transmit(packet), 0);
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(s_stops, 1);
  cl_assert(!s_speaker_open);
  prv_connect(true);
  cl_assert_equal_i(s_starts, 2);
}

void test_hci_local_audio__disconnect_before_start_does_not_start_capture(void) {
  const uint8_t connection[] = {
    4, 0x2c, 17, 0, 0x80, 1, 1, 2, 3, 4, 5, 6, 2, 6, 2, 30, 0, 30, 0, 2,
  };
  const uint8_t disconnect[] = {4, 5, 4, 0, 0x80, 1, 0x13};
  hci_local_audio_receive(connection, sizeof(connection));
  hci_local_audio_receive(disconnect, sizeof(disconnect));
  cl_assert_equal_i(list_count(s_system_task_callback_head), 1);
  fake_system_task_callbacks_invoke_pending();
  cl_assert_equal_i(s_starts, 0);
  cl_assert(!s_speaker_open);
}

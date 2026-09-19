/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include <pbl/drivers/mic.h>

bool mic_start_polling(MicDevice *this, MicDataHandlerCB data_handler, void *context,
                       int16_t *audio_buffer, size_t audio_buffer_len, MicDataReadyCB ready) {
  return false;
}

void mic_poll(MicDevice *this) {
}

bool mic_get_frame_time(MicDevice *this, uint32_t *sample_time) {
  return false;
}

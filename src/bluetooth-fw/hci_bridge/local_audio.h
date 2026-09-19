/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

// Standard H4 interface; no controller-specific audio APIs.
void hci_local_audio_command(const uint8_t *packet, size_t length);
// Returns false when the local audio endpoint owns the packet exclusively.
bool hci_local_audio_receive(const uint8_t *packet, size_t length);
size_t hci_local_audio_transmit(uint8_t packet[64]);
void hci_local_audio_report(void);

// Retire the audio stream even if the controller cannot report disconnection.
void hci_local_audio_stop(void);

// Per-call controls; speaker volume is additionally capped by watch preferences.
void hci_local_audio_set_controls(unsigned speaker_volume, bool mic_muted);

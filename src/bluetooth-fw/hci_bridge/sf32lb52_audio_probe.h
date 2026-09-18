/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stddef.h>
#include <stdint.h>

void hci_bridge_audio_probe_init(void);
size_t hci_bridge_audio_command(const uint8_t *packet, size_t length, uint8_t response[8]);
void hci_bridge_audio_event(uint8_t *packet, size_t length);
size_t hci_bridge_audio_receive(uint8_t packet[124]);
void hci_bridge_audio_send(const uint8_t *packet, size_t length);
size_t hci_bridge_audio_completed(uint8_t packet[8]);

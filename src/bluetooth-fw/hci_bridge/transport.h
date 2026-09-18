/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stddef.h>
#include <stdint.h>

//! Controller-specific transport, exposing standard H4 packets.
void hci_bridge_transport_init(void);
void hci_bridge_transport_write(const uint8_t *data, size_t length);
void hci_bridge_transport_receive(const uint8_t *data, size_t length);
void hci_bridge_transport_wake(void);

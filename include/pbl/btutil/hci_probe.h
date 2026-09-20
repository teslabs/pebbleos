/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

//! Read an HCI command's return parameters, excluding the status byte.
//! Return 0 for an exact-length response, a positive HCI status on rejection,
//! or a negative backend error on transport failure or an invalid response.
typedef int (*BtHciProbeRead)(uint16_t opcode, uint8_t *response, uint8_t length, void *context);
typedef void (*BtHciProbeOutput)(const char *line, void *context);

//! Query controller capabilities without changing controller configuration.
//! The caller must serialize commands with the active host and keep it running.
//! Returns false if any query fails; transport errors abort the remaining queries.
bool bt_hci_probe(BtHciProbeRead read, BtHciProbeOutput output, void *context);

/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

//! Wake the HCI transport task so it services pending audio and completions.
void bt_hci_transport_wake(void);

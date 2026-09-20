/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "host.h"
BtClassicHost *hfp_service_host(void);
void hfp_service_init(void);
void hfp_service_poll(uint32_t now);
void hfp_service_wake(void);

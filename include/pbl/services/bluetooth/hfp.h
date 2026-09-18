/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdbool.h>

typedef struct {
  bool available, connected, ready, audio, call, incoming, busy;
  unsigned call_setup, errors;
  char detail[64];
} HfpStatus;

// Internal call service, not an exported SDK API.
void hfp_get_status(HfpStatus *status);
bool hfp_dial(const char *number);
bool hfp_answer(void);
bool hfp_hangup(void);

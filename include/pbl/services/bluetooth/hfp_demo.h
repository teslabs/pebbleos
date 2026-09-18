/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdbool.h>

typedef struct {
  bool available, connected, ready, audio, call, incoming, busy;
  unsigned call_setup, errors;
  char detail[64];
} HfpDemoStatus;

#define HFP_DEMO_MAX_CONTACTS 8

typedef struct {
  char name[24];
  char number[33];
} HfpDemoContact;

// Internal development service, not an exported SDK API.
void hfp_demo_get_status(HfpDemoStatus *status);
unsigned hfp_demo_get_contacts(HfpDemoContact *contacts, unsigned capacity);
bool hfp_demo_set_contact(const char *name, const char *number);
bool hfp_demo_dial(const char *number);
bool hfp_demo_answer(void);
bool hfp_demo_hangup(void);

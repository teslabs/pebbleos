/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <stdbool.h>

#define PHONE_MAX_CONTACTS 8

typedef struct {
  char name[64];
  char number[33];
} PhoneContact;

// Reads companion-synced favorites on the calling task.
unsigned phone_call_contacts_get(PhoneContact *contacts, unsigned capacity);
unsigned phone_call_contacts_test_revision(void);
bool phone_call_contacts_set_test(const char *name, const char *number);

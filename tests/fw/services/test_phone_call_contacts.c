/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "clar.h"
#include <pbl/services/phone_call_contacts.h>
#include <pbl/services/blob_db/watch_app_prefs_db.h>
#include <pbl/services/contacts/contacts.h>
#include "stubs_mutex.h"
#include "stubs_pbl_malloc.h"
#include <stdlib.h>
#include <string.h>

static Contact s_contacts[4];
static Address s_addresses[4];
static const char *s_numbers[4];
static unsigned s_count, s_freed;

bool uuid_equal(const Uuid *a, const Uuid *b) {
  return !memcmp(a, b, sizeof(*a));
}

SerializedSendTextPrefs *watch_app_prefs_get_send_text(void) {
  SerializedSendTextPrefs *prefs = calloc(1, sizeof(*prefs) + s_count * sizeof(prefs->contacts[0]));
  prefs->num_contacts = s_count;
  for (unsigned i = 0; i < s_count; ++i) {
    prefs->contacts[i].contact_uuid = s_contacts[i].id;
    prefs->contacts[i].address_uuid = s_addresses[i].id;
  }
  return prefs;
}

Contact *contacts_get_contact_by_uuid(const Uuid *uuid) {
  for (unsigned i = 0; i < s_count; ++i)
    if (uuid_equal(uuid, &s_contacts[i].id))
      return &s_contacts[i];
  return NULL;
}

void contacts_free_contact(Contact *contact) {
  ++s_freed;
}

const char *attribute_get_string(const AttributeList *list, AttributeId id, char *fallback) {
  for (unsigned i = 0; i < s_count; ++i) {
    if (list == &s_contacts[i].attr_list)
      return "Test contact";
    if (list == &s_addresses[i].attr_list)
      return s_numbers[i];
  }
  return fallback;
}

void test_phone_call_contacts__initialize(void) {
  s_count = 4;
  s_freed = 0;
  memset(s_contacts, 0, sizeof(s_contacts));
  memset(s_addresses, 0, sizeof(s_addresses));
  for (unsigned i = 0; i < s_count; ++i) {
    memset(&s_contacts[i].id, i + 1, sizeof(Uuid));
    memset(&s_addresses[i].id, i + 5, sizeof(Uuid));
    s_addresses[i].type = AddressTypePhoneNumber;
    s_contacts[i].addr_list = (AddressList){.num_addresses = 1, .addresses = &s_addresses[i]};
  }
  s_numbers[0] = "+1 (202) 555-0100";
  s_numbers[1] = "+12025550100";
  s_numbers[2] = "555.0101";
  s_numbers[3] = "5550102";
}

void test_phone_call_contacts__formats_and_deduplicates(void) {
  PhoneContact result[4];
  cl_assert_equal_i(phone_call_contacts_get(result, 4), 3);
  cl_assert_equal_s(result[0].number, "+12025550100");
  cl_assert_equal_s(result[1].number, "5550101");
  cl_assert_equal_s(result[0].name, "Test contact");
  cl_assert_equal_i(s_freed, 4);
}

void test_phone_call_contacts__bounds_output(void) {
  PhoneContact result[2] = {{}, {.name = "untouched"}};
  cl_assert_equal_i(phone_call_contacts_get(result, 1), 1);
  cl_assert_equal_s(result[1].name, "untouched");
  cl_assert_equal_i(s_freed, 1);
  cl_assert_equal_i(phone_call_contacts_get(NULL, 1), 0);
  cl_assert_equal_i(phone_call_contacts_get(result, 0), 0);
}

void test_phone_call_contacts__rejects_unsafe_or_non_phone_addresses(void) {
  s_numbers[0] = "5550100;AT+CHUP";
  s_numbers[1] = "5550100,12";
  s_numbers[2] = "123456789012345678901234567890123";
  s_addresses[3].type = AddressTypeEmail;
  PhoneContact result[4];
  cl_assert_equal_i(phone_call_contacts_get(result, 4), 0);
  cl_assert_equal_i(s_freed, 4);
}

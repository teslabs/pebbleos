/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include <pbl/services/phone_call_contacts.h>

#include "kernel/pbl_malloc.h"
#include <pbl/kernel/mutex.h>
#include <pbl/services/blob_db/watch_app_prefs_db.h>
#include <pbl/services/contacts/contacts.h>
#include <stdio.h>
#include <string.h>

#ifdef CONFIG_PROMPT
static PBL_MUTEX_DEFINE(s_lock);
static PhoneContact s_test_contacts[PHONE_MAX_CONTACTS];
static unsigned s_test_count, s_test_revision;
#endif

static bool normalize_number(char output[33], const char *input) {
  unsigned length = 0, digits = 0;
  if (!input)
    return false;
  for (; *input; ++input) {
    char c = *input;
    if (c == ' ' || c == '-' || c == '(' || c == ')' || c == '.')
      continue;
    if (length >= 32)
      return false;
    if (c >= '0' && c <= '9')
      ++digits;
    else if (c != '*' && c != '#' && !(c == '+' && length == 0))
      return false;
    output[length++] = c;
  }
  output[length] = 0;
  return digits != 0;
}

#ifdef CONFIG_PROMPT
bool phone_call_contacts_set_test(const char *name, const char *number) {
  PhoneContact contact = {};
  if (!name || !*name || strlen(name) >= sizeof(contact.name) ||
      !normalize_number(contact.number, number))
    return false;
  snprintf(contact.name, sizeof(contact.name), "%s", name);
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  unsigned index = 0;
  while (index < s_test_count && strcmp(s_test_contacts[index].name, name))
    ++index;
  bool available = index < PHONE_MAX_CONTACTS;
  if (available) {
    s_test_contacts[index] = contact;
    s_test_count += index == s_test_count;
    ++s_test_revision;
  }
  pbl_mutex_unlock(&s_lock);
  return available;
}

unsigned phone_call_contacts_test_revision(void) {
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  unsigned revision = s_test_revision;
  pbl_mutex_unlock(&s_lock);
  return revision;
}
#else
unsigned phone_call_contacts_test_revision(void) {
  return 0;
}
#endif

unsigned phone_call_contacts_get(PhoneContact *contacts, unsigned capacity) {
  if (!contacts || !capacity)
    return 0;
  unsigned count = 0;
#ifdef CONFIG_PROMPT
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  count = s_test_count < capacity ? s_test_count : capacity;
  memcpy(contacts, s_test_contacts, count * sizeof(*contacts));
  pbl_mutex_unlock(&s_lock);
#endif
  SerializedSendTextPrefs *prefs = watch_app_prefs_get_send_text();
  if (!prefs)
    return count;
  for (unsigned i = 0; i < prefs->num_contacts && count < capacity; ++i) {
    const SerializedSendTextContact *pref = &prefs->contacts[i];
    Contact *contact = contacts_get_contact_by_uuid(&pref->contact_uuid);
    if (!contact)
      continue;
    for (unsigned j = 0; j < contact->addr_list.num_addresses && count < capacity; ++j) {
      const Address *address = &contact->addr_list.addresses[j];
      if (address->type != AddressTypePhoneNumber || !uuid_equal(&address->id, &pref->address_uuid))
        continue;
      PhoneContact entry = {};
      if (!normalize_number(entry.number,
                            attribute_get_string(&address->attr_list, AttributeIdAddress, "")))
        continue;
      bool duplicate = false;
      for (unsigned k = 0; k < count; ++k)
        duplicate |= !strcmp(contacts[k].number, entry.number);
      if (duplicate)
        continue;
      const char *name = attribute_get_string(&contact->attr_list, AttributeIdTitle, entry.number);
      size_t bytes = strlen(name);
      if (bytes >= sizeof(entry.name)) {
        bytes = sizeof(entry.name) - 1;
        while (bytes && ((unsigned char)name[bytes] & 0xc0) == 0x80)
          --bytes;
      }
      memcpy(entry.name, name, bytes);
      contacts[count++] = entry;
    }
    contacts_free_contact(contact);
  }
  task_free(prefs);
  return count;
}

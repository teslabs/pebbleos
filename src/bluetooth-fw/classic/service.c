/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "service.h"

#include <console/prompt.h>
#include <kernel/events.h>
#include <pbl/kernel/msgq.h>
#include <pbl/kernel/mutex.h>
#include <pbl/services/bluetooth/hfp_demo.h>
#include <pbl/services/phone_call_util.h>
#include <stdio.h>
#include <string.h>

static BtClassicHost s_host;
static PBL_MUTEX_DEFINE(s_lock);
static HfpDemoStatus s_status;
static HfpDemoContact s_contacts[HFP_DEMO_MAX_CONTACTS];
static unsigned s_contact_count;

typedef struct {
  unsigned action;
  char number[BT_CLASSIC_NUMBER_SIZE];
} Request;
static PBL_MSGQ_DEFINE(s_requests, sizeof(Request), 4);

BtClassicHost *hfp_service_host(void) {
  return &s_host;
}

void hfp_service_init(void) {
  hfp_demo_set_contact(CONFIG_BT_HFP_DEMO_CONTACT_1_NAME, CONFIG_BT_HFP_DEMO_CONTACT_1_NUMBER);
  hfp_demo_set_contact(CONFIG_BT_HFP_DEMO_CONTACT_2_NAME, CONFIG_BT_HFP_DEMO_CONTACT_2_NUMBER);
  bt_classic_reset(&s_host);
}

static void notify_call(PhoneEventType type) {
  PebbleEvent event = {
    .type = PEBBLE_PHONE_EVENT,
    .phone = {
      .type = type,
      .source = PhoneCallSource_HFP,
      .caller = type == PhoneEventType_Incoming ? phone_call_util_create_caller(NULL, NULL) : NULL,
    },
  };
  event_put(&event);
}

static void publish_call_state(void) {
  static bool incoming, started;
  const BtClassicStatus *status = &s_host.status;
  if (!incoming && status->ready && status->incoming && !status->call) {
    incoming = true;
    started = false;
    notify_call(PhoneEventType_Incoming);
  }
  if (!incoming)
    return;
  if (status->call && !started) {
    started = true;
    notify_call(PhoneEventType_Start);
  }
  if (!status->ready || (!status->incoming && !status->call && !status->call_setup)) {
    incoming = started = false;
    notify_call(status->ready ? PhoneEventType_End : PhoneEventType_Disconnect);
  }
}

void hfp_service_poll(uint32_t now) {
  Request request;
  while (pbl_msgq_get(&s_requests, &request, PBL_NO_WAIT) == 0) {
    bool accepted = request.action == 0   ? bt_classic_dial(&s_host, request.number)
                    : request.action == 1 ? bt_classic_answer(&s_host)
                                          : bt_classic_hangup(&s_host);
    if (!accepted) {
      ++s_host.status.errors;
      snprintf(s_host.status.detail, sizeof(s_host.status.detail), "Call action unavailable");
    }
  }
  bt_classic_poll(&s_host, now);
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  const BtClassicStatus *status = &s_host.status;
  s_status = (HfpDemoStatus){
    .available = status->available,
    .connected = status->connected,
    .ready = status->ready,
    .audio = status->audio,
    .call = status->call,
    .incoming = status->incoming,
    .busy = status->busy,
    .call_setup = status->call_setup,
    .errors = status->errors,
  };
  snprintf(s_status.detail, sizeof(s_status.detail), "%s", status->detail);
  pbl_mutex_unlock(&s_lock);
  publish_call_state();
}

void hfp_demo_get_status(HfpDemoStatus *status) {
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  *status = s_status;
  pbl_mutex_unlock(&s_lock);
}
unsigned hfp_demo_get_contacts(HfpDemoContact *contacts, unsigned capacity) {
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  unsigned count = s_contact_count < capacity ? s_contact_count : capacity;
  if (count)
    memcpy(contacts, s_contacts, count * sizeof(*contacts));
  pbl_mutex_unlock(&s_lock);
  return count;
}

bool hfp_demo_set_contact(const char *name, const char *number) {
  if (!name || !*name || strlen(name) >= sizeof(s_contacts[0].name) ||
      !bt_classic_valid_number(number))
    return false;
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  unsigned index = 0;
  while (index < s_contact_count && strcmp(s_contacts[index].name, name))
    ++index;
  if (index == HFP_DEMO_MAX_CONTACTS) {
    pbl_mutex_unlock(&s_lock);
    return false;
  }
  snprintf(s_contacts[index].name, sizeof(s_contacts[index].name), "%s", name);
  snprintf(s_contacts[index].number, sizeof(s_contacts[index].number), "%s", number);
  if (index == s_contact_count)
    ++s_contact_count;
  pbl_mutex_unlock(&s_lock);
  return true;
}

static bool request(unsigned action, const char *number) {
  Request r = {.action = action};
  if (number) {
    if (!bt_classic_valid_number(number))
      return false;
    snprintf(r.number, sizeof(r.number), "%s", number);
  }
  if (pbl_msgq_put(&s_requests, &r, PBL_NO_WAIT))
    return false;
  hfp_service_wake();
  return true;
}
bool hfp_demo_dial(const char *number) {
  return request(0, number);
}
bool hfp_demo_answer(void) {
  return request(1, NULL);
}
bool hfp_demo_hangup(void) {
  return request(2, NULL);
}

void command_bt_hfp_status(void) {
  HfpDemoStatus status;
  hfp_demo_get_status(&status);
  char line[192];
  snprintf(line, sizeof(line),
           "HFP available=%u connected=%u ready=%u audio=%u call=%u setup=%u busy=%u errors=%u",
           status.available, status.connected, status.ready, status.audio, status.call,
           status.call_setup, status.busy, status.errors);
  prompt_send_response(line);
  prompt_send_response(status.detail);
}
void command_bt_hfp_contact(const char *name, const char *number) {
  char decoded[sizeof(s_contacts[0].name)];
  if (!strncmp(name, "hex:", 4)) {
    const char *hex = name + 4;
    unsigned length = strlen(hex);
    if (!length || (length & 1) || length / 2 >= sizeof(decoded)) {
      prompt_send_response("Invalid encoded name");
      return;
    }
    for (unsigned i = 0; i < length; i += 2) {
      unsigned value = 0;
      for (unsigned j = 0; j < 2; ++j) {
        char c = hex[i + j];
        int digit = c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
        if (digit < 0) {
          prompt_send_response("Invalid encoded name");
          return;
        }
        value = value * 16 + digit;
      }
      if (value < 32 || value == 127) {
        prompt_send_response("Invalid contact name");
        return;
      }
      decoded[i / 2] = value;
    }
    decoded[length / 2] = 0;
    name = decoded;
  }
  prompt_send_response(hfp_demo_set_contact(name, number) ? "Contact set in RAM"
                                                          : "Invalid contact or list full");
}
void command_bt_hfp_contacts(void) {
  HfpDemoContact contacts[HFP_DEMO_MAX_CONTACTS];
  unsigned count = hfp_demo_get_contacts(contacts, HFP_DEMO_MAX_CONTACTS);
  for (unsigned i = 0; i < count; ++i) {
    char line[80];
    snprintf(line, sizeof(line), "%.23s: %.32s", contacts[i].name, contacts[i].number);
    prompt_send_response(line);
  }
  if (!count)
    prompt_send_response("No contacts");
}
void command_bt_hfp_dial(const char *number) {
  prompt_send_response(hfp_demo_dial(number) ? "Dial queued" : "Invalid number or queue full");
}
void command_bt_hfp_answer(void) {
  hfp_demo_answer();
}
void command_bt_hfp_hangup(void) {
  hfp_demo_hangup();
}

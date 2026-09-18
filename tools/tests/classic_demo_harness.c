/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "host.h"
#include <string.h>

static BtClassicHost s_host;
static BtClassicPacket s_output[64];
static unsigned s_read, s_write;
static void send(const uint8_t *p, size_t n, void *context) {
  if (n > sizeof(s_output[0].data) || s_write - s_read >= 64)
    __builtin_trap();
  BtClassicPacket *out = &s_output[s_write++ % 64];
  memcpy(out->data, p, n);
  out->length = n;
}
void demo_init(void) {
  s_read = s_write = 0;
  bt_classic_init(&s_host, send, NULL);
}
void demo_tick(unsigned now) {
  bt_classic_poll(&s_host, now);
}
void demo_receive(const uint8_t *p, unsigned n) {
  bt_classic_receive(&s_host, p, n);
}
unsigned demo_pop(uint8_t *p) {
  if (s_read == s_write)
    return 0;
  BtClassicPacket *out = &s_output[s_read++ % 64];
  memcpy(p, out->data, out->length);
  return out->length;
}
unsigned demo_flags(void) {
  return s_host.status.available | s_host.status.connected << 1 | s_host.status.ready << 2 |
         s_host.status.call << 3 | s_host.status.incoming << 4 | s_host.status.busy << 5;
}
unsigned demo_errors(void) {
  return s_host.status.errors;
}
const char *demo_detail(void) {
  return s_host.status.detail;
}
int demo_dial(const char *number) {
  return bt_classic_dial(&s_host, number);
}
int demo_answer(void) {
  return bt_classic_answer(&s_host);
}
int demo_hangup(void) {
  return bt_classic_hangup(&s_host);
}
static bool s_acl_ready;
static bool send_acl(const uint8_t *p, size_t n, void *context) {
  if (!s_acl_ready)
    return false;
  send(p, n, context);
  return true;
}
void demo_init_managed(void) {
  s_read = s_write = 0;
  s_acl_ready = true;
  bt_classic_init_managed(&s_host, send, send_acl, 676, NULL);
}
void demo_acl_ready(int ready) {
  s_acl_ready = ready;
}
void demo_stop(void) {
  bt_classic_stop(&s_host);
}
int demo_stopped(void) {
  return bt_classic_stopped(&s_host);
}

static bool s_bond_present;
static uint8_t s_bond_value = 0xa5;
static bool shared_key(const uint8_t peer[6], uint8_t key[16], void *context) {
  const uint8_t expected[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
  if (!s_bond_present || memcmp(peer, expected, 6))
    return false;
  if (key)
    memset(key, s_bond_value, 16);
  return true;
}
void demo_shared_bond(int present) {
  s_host.get_link_key = shared_key;
  s_bond_present = present;
  s_bond_value = 0xa5;
}
void demo_replace_bond(void) {
  s_bond_value = 0x42;
}
unsigned demo_encrypted(void) {
  return s_host.encrypted;
}

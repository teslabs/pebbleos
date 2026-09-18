/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "host.h"
#include <string.h>

static ClassicDemoHost s_host;
static ClassicDemoPacket s_output[64];
static unsigned s_read, s_write;
static void send(const uint8_t *p, size_t n, void *context) {
  if (n > sizeof(s_output[0].data) || s_write - s_read >= 64)
    __builtin_trap();
  ClassicDemoPacket *out = &s_output[s_write++ % 64];
  memcpy(out->data, p, n);
  out->length = n;
}
void demo_init(void) {
  s_read = s_write = 0;
  classic_demo_init(&s_host, send, NULL);
}
void demo_tick(unsigned now) {
  classic_demo_poll(&s_host, now);
}
void demo_receive(const uint8_t *p, unsigned n) {
  classic_demo_receive(&s_host, p, n);
}
unsigned demo_pop(uint8_t *p) {
  if (s_read == s_write)
    return 0;
  ClassicDemoPacket *out = &s_output[s_read++ % 64];
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
  return classic_demo_dial(&s_host, number);
}
int demo_answer(void) {
  return classic_demo_answer(&s_host);
}
int demo_hangup(void) {
  return classic_demo_hangup(&s_host);
}

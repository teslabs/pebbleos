/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <clar.h>
#include <pbl/btutil/hci_probe.h>
#include <stdio.h>
#include <string.h>

static char s_output[4096];
static unsigned s_reads;
static uint16_t s_failed_opcode;
static int s_failure;
static bool s_le_only;
static bool s_no_sco_buffers;

void test_hci_probe__initialize(void) {
  memset(s_output, 0, sizeof(s_output));
  s_reads = 0;
  s_failed_opcode = 0;
  s_failure = 0;
  s_le_only = false;
  s_no_sco_buffers = false;
}

static int prv_read(uint16_t opcode, uint8_t *response, uint8_t length, void *context) {
  cl_assert(context == &s_reads);
  ++s_reads;
  if (opcode == s_failed_opcode) {
    memset(response, 0xff, length);
    return s_failure;
  }
  switch (opcode) {
    case 0x1001: {
      const uint8_t version[] = {12, 0x34, 0x12, 12, 0x78, 0x56, 0xbc, 0x9a};
      cl_assert_equal_i(length, sizeof(version));
      memcpy(response, version, sizeof(version));
      break;
    }
    case 0x1002:
      cl_assert_equal_i(length, 64);
      for (unsigned i = 0; i < length; ++i) {
        response[i] = i;
      }
      break;
    case 0x1003:
      cl_assert_equal_i(length, 8);
      response[4] = 0x40;
      if (s_le_only) {
        response[4] |= 0x20;
      } else {
        response[1] = 0x08;
        response[2] = 0x09;
        response[3] = 0x80;
      }
      break;
    case 0x1005: {
      const uint8_t buffers[] = {0xfd, 0x03, 60, 0x04, 0x01, 0x03, 0x00};
      cl_assert_equal_i(length, sizeof(buffers));
      memcpy(response, buffers, sizeof(buffers));
      if (s_no_sco_buffers) {
        response[2] = response[5] = response[6] = 0;
      }
      break;
    }
    case 0x0c25:
      cl_assert_equal_i(length, 2);
      response[0] = 0x60;
      break;
    default:
      cl_assert(false);
  }
  return 0;
}

static void prv_output(const char *line, void *context) {
  cl_assert(context == &s_reads);
  cl_assert(strlen(line) < 128);
  size_t used = strlen(s_output);
  cl_assert(used + strlen(line) + 2 < sizeof(s_output));
  snprintf(s_output + used, sizeof(s_output) - used, "%s\n", line);
}

void test_hci_probe__dual_mode(void) {
  cl_assert(bt_hci_probe(prv_read, prv_output, &s_reads));
  cl_assert_equal_i(s_reads, 5);
  cl_assert(strstr(s_output, "revision=4660 lmp=12 manufacturer=22136 subversion=39612"));
  cl_assert(strstr(s_output, "features bredr=1 le=1 sco=1 esco=1 cvsd=1 transparent=1"));
  cl_assert(strstr(s_output, "acl_bytes=1021 acl_packets=260 sco_bytes=60 sco_packets=3"));
  cl_assert(strstr(s_output, "voice setting=0x0060"));
  cl_assert(strstr(s_output, "hci 1002 data[48]=303132333435363738393a3b3c3d3e3f"));
  cl_assert(strstr(s_output, "capability bits do not prove working call audio"));
}

void test_hci_probe__le_only(void) {
  s_le_only = true;
  cl_assert(bt_hci_probe(prv_read, prv_output, &s_reads));
  cl_assert(strstr(s_output, "features bredr=0 le=1 sco=0 esco=0 cvsd=0 transparent=0"));
}

void test_hci_probe__no_sco_buffers(void) {
  s_no_sco_buffers = true;
  cl_assert(bt_hci_probe(prv_read, prv_output, &s_reads));
  cl_assert(strstr(s_output, "sco_bytes=0 sco_packets=0"));
}

void test_hci_probe__rejected_query_is_not_decoded(void) {
  s_failed_opcode = 0x1003;
  s_failure = 1;
  cl_assert(!bt_hci_probe(prv_read, prv_output, &s_reads));
  cl_assert_equal_i(s_reads, 5);
  cl_assert(strstr(s_output, "hci 1003 features status=1"));
  cl_assert(!strstr(s_output, "features bredr="));
  cl_assert(!strstr(s_output, "hci 1003 data"));
  cl_assert(strstr(s_output, "HCI probe incomplete"));
}

void test_hci_probe__transport_failure_aborts(void) {
  s_failed_opcode = 0x1002;
  s_failure = -13;
  cl_assert(!bt_hci_probe(prv_read, prv_output, &s_reads));
  cl_assert_equal_i(s_reads, 2);
  cl_assert(strstr(s_output, "HCI probe aborted"));
  cl_assert(!strstr(s_output, "hci 1003"));
}

/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <clar.h>
#include <pbl/drivers/flash.h>
#include <pbl/util/crc32.h>
#include "util/legacy_checksum.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

static uint8_t s_flash[2061];
static bool s_no_heap;
static unsigned s_allocations, s_frees, s_read_bytes;

void *kernel_malloc(size_t size) {
  if (s_no_heap) {
    return NULL;
  }
  ++s_allocations;
  return malloc(size);
}

void kernel_free(void *buffer) {
  ++s_frees;
  free(buffer);
}

void flash_read_bytes(uint8_t *buffer, uint32_t address, uint32_t length) {
  cl_assert(address + length <= sizeof(s_flash));
  cl_assert(length <= (s_no_heap ? 128 : 1024));
  memcpy(buffer, s_flash + address, length);
  s_read_bytes += length;
}

void test_flash_crc__initialize(void) {
  for (unsigned i = 0; i < sizeof(s_flash); ++i) {
    s_flash[i] = (i * 137 + i / 256) & 255;
  }
  s_no_heap = false;
  s_allocations = s_frees = s_read_bytes = 0;
}

static void prv_check(void) {
  const unsigned lengths[] = {0, 1, 127, 128, 129, 1024, 1025, sizeof(s_flash)};
  for (unsigned i = 0; i < sizeof(lengths) / sizeof(lengths[0]); ++i) {
    unsigned length = lengths[i];
    s_read_bytes = 0;
    cl_assert_equal_i(flash_crc32(0, length), crc32(CRC32_INIT, s_flash, length));
    cl_assert_equal_i(s_read_bytes, length);
    LegacyChecksum checksum;
    legacy_defective_checksum_init(&checksum);
    legacy_defective_checksum_update(&checksum, s_flash, length);
    s_read_bytes = 0;
    cl_assert_equal_i(flash_calculate_legacy_defective_checksum(0, length),
                      legacy_defective_checksum_finish(&checksum));
    cl_assert_equal_i(s_read_bytes, length);
  }
  cl_assert_equal_i(s_allocations, s_no_heap ? 0 : 16);
  cl_assert_equal_i(s_allocations, s_frees);
}

void test_flash_crc__checksums_with_heap_buffer(void) {
  prv_check();
}

void test_flash_crc__checksums_without_any_heap_space(void) {
  s_no_heap = true;
  prv_check();
}

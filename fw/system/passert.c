/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "passert.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#include <pbl/kernel/compiler.h>
#include <pbl/kernel/debug.h>
#include <pbl/kernel/fatal.h>
#include <pbl/logging/logging.h>
#include <pbl/util/heap.h>

#include <kernel/fault_handling.h>
#include <kernel/kernel_heap.h>
#include <kernel/pebble_tasks.h>
#include <syscall/syscall.h>
#include <system/reboot_reason.h>

void assert_failed(uint8_t *file, uint32_t line) {
  pbl_panic_bare_at((const char *)file, line);
}

#ifdef CONFIG_SOC_NRF52
[[noreturn]] void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info) {
  PBL_LOG_ALWAYS("nRF error %ld (pc %ld, info %ld)", id, pc, info);
  pbl_fatal_error_handler(pc);
}

[[noreturn]] void app_error_handler_bare(uint32_t error_code) {
  app_error_fault_handler(error_code, (uint32_t)PBL_RETURN_ADDRESS(0), 0);
}
#endif

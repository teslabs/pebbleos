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

[[noreturn]] void pbl_kernel_assert_failed(const char *filename, int line) {
  pbl_panic_bare_at(filename, line);
}

void passert_check_task(PebbleTask expected_task) {
  uintptr_t saved_lr = (uintptr_t)PBL_RETURN_ADDRESS(0);

  if (pebble_task_get_current() != expected_task) {
    PBL_LOG_ALWAYS("LR: %p. Incorrect task! Expected <%s> got <%s>", (void *)saved_lr,
                   pebble_task_get_name(expected_task),
                   pebble_task_get_name(pebble_task_get_current()));
    pbl_fatal_error_handler(saved_lr);
  }
}

void passert_check_not_task(PebbleTask unexpected_task) {
  uintptr_t saved_lr = (uintptr_t)PBL_RETURN_ADDRESS(0);

  if (pebble_task_get_current() == unexpected_task) {
    PBL_LOG_ALWAYS("LR: %p. Incorrect task! Can't be <%s>", (void *)saved_lr,
                   pebble_task_get_name(unexpected_task));
    pbl_fatal_error_handler(saved_lr);
  }
}

void assert_failed(uint8_t *file, uint32_t line) {
  pbl_panic_bare_at((const char *)file, line);
}

[[noreturn]] void croak_oom(size_t bytes, int saved_lr, Heap *heap_ptr) {
  unsigned int used = 0, free_bytes = 0, max_free = 0;
  if (heap_ptr) {
    heap_calc_totals(heap_ptr, &used, &free_bytes, &max_free);
  }
  PBL_LOG_ALWAYS("CROAK OOM: Failed to alloc %zu bytes at LR: 0x%x (used %u, free %u, max_free %u)",
                 bytes, saved_lr, used, free_bytes, max_free);

#ifdef CONFIG_MALLOC_INSTRUMENTATION
  kernel_heap_dump_instrumentation();
#endif

  trigger_oom_fault(bytes, saved_lr, heap_ptr);
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

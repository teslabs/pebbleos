/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/kernel/assert.h>
#include <pbl/kernel/compiler.h>
#include <pbl/kernel/fatal.h>
#include <pbl/logging/logging.h>

#if UNITTEST

#define PBL_ASSERT_TASK(task)
#define PBL_ASSERT_NOT_TASK(task)
#define PBL_ASSERT_RUNNING_FROM_EXPECTED_TASK(task)
#define BREAKPOINT

#else

// Insert a compiled-in breakpoint
#define BREAKPOINT __asm("bkpt")

enum PebbleTask;
void passert_check_task(enum PebbleTask expected_task);
void passert_check_not_task(enum PebbleTask unexpected_task);

#define PBL_ASSERT_TASK(task)     passert_check_task(task);
#define PBL_ASSERT_NOT_TASK(task) passert_check_not_task(task);

// It's useful during development to insert asserts to make sure our callbacks
// are being dispatched as expected. It's wasteful (for codespace) to keep them
// on after that as it's only when code gets edited that the assert gets hit.
#ifdef CHECK_RUNNING_FROM_EXPECTED_TASK
#define PBL_ASSERT_RUNNING_FROM_EXPECTED_TASK(task) PBL_ASSERT_TASK(task)
#else
#define PBL_ASSERT_RUNNING_FROM_EXPECTED_TASK(task)
#endif

#endif // UNITTEST

typedef struct Heap Heap;

[[noreturn]] void croak_oom(size_t bytes, int saved_lr, Heap *heap_ptr);

#define PBL_CROAK_OOM(bytes, saved_lr, heap_ptr) croak_oom(bytes, saved_lr, heap_ptr)

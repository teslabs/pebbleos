/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/kernel/assert.h>
#include <pbl/kernel/compiler.h>
#include <pbl/kernel/fatal.h>
#include <pbl/logging/logging.h>

typedef struct Heap Heap;

[[noreturn]] void croak_oom(size_t bytes, int saved_lr, Heap *heap_ptr);

#define PBL_CROAK_OOM(bytes, saved_lr, heap_ptr) croak_oom(bytes, saved_lr, heap_ptr)

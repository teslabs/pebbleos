/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

//! Number of distinct keys tracked by a pbl_analytics_top
#define PBL_ANALYTICS_TOP_SLOTS 8U

//! Bounded "most frequent key" counter (Space-Saving). When all slots are in
//! use, a new key evicts the least frequent one and inherits its count, so the
//! reported count of the top key is an upper bound. Safe to use from any task.
struct pbl_analytics_top {
  uint32_t total;
  struct {
    uintptr_t key;
    uint32_t count;
  } slots[PBL_ANALYTICS_TOP_SLOTS];
};

void pbl_analytics_top_add(struct pbl_analytics_top *top, uintptr_t key);

//! Read the total and most frequent key, then reset the counter.
void pbl_analytics_top_take(struct pbl_analytics_top *top, uint32_t *total, uintptr_t *key,
                            uint32_t *count);

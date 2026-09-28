/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/analytics/top.h"

#include <string.h>

#include "pbl/kernel/irq.h"

void pbl_analytics_top_add(struct pbl_analytics_top *top, uintptr_t key) {
  pbl_irq_lock();

  top->total++;

  unsigned int min = 0U;
  for (unsigned int i = 0U; i < PBL_ANALYTICS_TOP_SLOTS; i++) {
    if ((top->slots[i].count != 0U) && (top->slots[i].key == key)) {
      top->slots[i].count++;
      pbl_irq_unlock();
      return;
    }
    if (top->slots[i].count < top->slots[min].count) {
      min = i;
    }
  }

  top->slots[min].key = key;
  top->slots[min].count++;

  pbl_irq_unlock();
}

void pbl_analytics_top_take(struct pbl_analytics_top *top, uint32_t *total, uintptr_t *key,
                            uint32_t *count) {
  pbl_irq_lock();

  unsigned int max = 0U;
  for (unsigned int i = 1U; i < PBL_ANALYTICS_TOP_SLOTS; i++) {
    if (top->slots[i].count > top->slots[max].count) {
      max = i;
    }
  }

  *total = top->total;
  *key = top->slots[max].key;
  *count = top->slots[max].count;

  memset(top, 0, sizeof(*top));

  pbl_irq_unlock();
}

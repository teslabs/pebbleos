/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/clock/sf32lb52.h>
#include <pbl/drivers/reset/sf32lb52.h>
#include <pbl/kernel/irq.h>

#include <stdbool.h>

static void prv_update(uintptr_t base, uint16_t line, bool set) {
  volatile uint32_t *reg = (volatile uint32_t *)(base + (line & PBL_CLOCK_SF32LB52_REG_MSK));
  uint32_t mask = 1UL << ((line & PBL_CLOCK_SF32LB52_BIT_MSK) >> PBL_CLOCK_SF32LB52_BIT_POS);

  pbl_irq_lock();
  if (set) {
    *reg |= mask;
  } else {
    *reg &= ~mask;
  }
  pbl_irq_unlock();
}

void pbl_clock_sf32lb52_on(const struct pbl_clock_sf32lb52 *clk) {
  prv_update(clk->ctrl->rcc->base, clk->id, true);
}

void pbl_clock_sf32lb52_off(const struct pbl_clock_sf32lb52 *clk) {
  prv_update(clk->ctrl->rcc->base, clk->id, false);
}

void pbl_reset_sf32lb52_assert(const struct pbl_reset_sf32lb52 *rst) {
  prv_update(rst->ctrl->rcc->base, rst->id, true);
}

void pbl_reset_sf32lb52_deassert(const struct pbl_reset_sf32lb52 *rst) {
  prv_update(rst->ctrl->rcc->base, rst->id, false);
}

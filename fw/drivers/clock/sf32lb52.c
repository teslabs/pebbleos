/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/clock/sf32lb52.h>
#include <pbl/kernel/irq.h>

static volatile uint32_t *prv_enr(const struct pbl_clock_sf32lb52 *clk) {
  return (volatile uint32_t *)(clk->rcc->base + (clk->id & PBL_CLOCK_SF32LB52_REG_MSK));
}

static uint32_t prv_mask(const struct pbl_clock_sf32lb52 *clk) {
  return 1UL << ((clk->id & PBL_CLOCK_SF32LB52_BIT_MSK) >> PBL_CLOCK_SF32LB52_BIT_POS);
}

void pbl_clock_sf32lb52_on(const struct pbl_clock_sf32lb52 *clk) {
  pbl_irq_lock();
  *prv_enr(clk) |= prv_mask(clk);
  pbl_irq_unlock();
}

void pbl_clock_sf32lb52_off(const struct pbl_clock_sf32lb52 *clk) {
  pbl_irq_lock();
  *prv_enr(clk) &= ~prv_mask(clk);
  pbl_irq_unlock();
}

/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

/**
 * @defgroup drivers_clock_sf32lb52 SF32LB52 RCC
 * @ingroup drivers
 * @brief Peripheral clock gates of the SF32LB52 HPSYS RCC.
 * @{
 */

/** @cond INTERNAL_HIDDEN */
#define PBL_CLOCK_SF32LB52_REG_MSK 0x00FFU
#define PBL_CLOCK_SF32LB52_BIT_POS 8U
#define PBL_CLOCK_SF32LB52_BIT_MSK 0x1F00U
/** @endcond */

/**
 * @brief Peripheral clock gate.
 *
 * @param reg ENR register offset.
 * @param bit Bit.
 */
#define PBL_CLOCK_SF32LB52_GATE(reg, bit) \
  ((uint16_t)((reg) | ((bit) << PBL_CLOCK_SF32LB52_BIT_POS)))

/** @brief Reset and clock controller. */
struct pbl_clock_sf32lb52_rcc {
  /** Registers. */
  uintptr_t base;
};

/** @brief Peripheral clock gate. */
struct pbl_clock_sf32lb52 {
  /** Controller. */
  const struct pbl_clock_sf32lb52_rcc *rcc;
  /** Gate, a @ref PBL_CLOCK_SF32LB52_GATE value. */
  uint16_t id;
};

/**
 * @brief Ungate a peripheral clock.
 *
 * @param clk Clock.
 */
void pbl_clock_sf32lb52_on(const struct pbl_clock_sf32lb52 *clk);

/**
 * @brief Gate a peripheral clock.
 *
 * @param clk Clock.
 */
void pbl_clock_sf32lb52_off(const struct pbl_clock_sf32lb52 *clk);

/** @} */

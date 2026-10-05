/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include <devicetree/types/sifli,sf32lb52-rcc.h>

/**
 * @defgroup drivers_clock_sf32lb52 SF32LB52 RCC clocks
 * @ingroup drivers
 * @brief Peripheral clock gates of the SF32LB52 HPSYS RCC.
 *
 * The RCC is one devicetree node exposing two interfaces: these clock gates
 * and the peripheral resets of @ref drivers_reset_sf32lb52.
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

/** @brief Clock interface of the RCC. */
struct pbl_clock_sf32lb52_ctrl {
  /** RCC. */
  const struct pbl_rcc_sf32lb52 *rcc;
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

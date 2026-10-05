/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include <devicetree/types/sifli,sf32lb52-rcc.h>
#include <pbl/drivers/clock/sf32lb52.h>

/**
 * @defgroup drivers_reset_sf32lb52 SF32LB52 RCC resets
 * @ingroup drivers
 * @brief Peripheral resets of the SF32LB52 HPSYS RCC.
 * @{
 */

/**
 * @brief Peripheral reset line, encoded like a clock gate.
 *
 * @param reg RSTR register offset.
 * @param bit Bit.
 */
#define PBL_RESET_SF32LB52_LINE(reg, bit) PBL_CLOCK_SF32LB52_GATE(reg, bit)

/** @brief Reset interface of the RCC. */
struct pbl_reset_sf32lb52_ctrl {
  /** RCC. */
  const struct pbl_rcc_sf32lb52 *rcc;
};

/**
 * @brief Put a peripheral in reset.
 *
 * @param rst Reset line.
 */
void pbl_reset_sf32lb52_assert(const struct pbl_reset_sf32lb52 *rst);

/**
 * @brief Take a peripheral out of reset.
 *
 * @param rst Reset line.
 */
void pbl_reset_sf32lb52_deassert(const struct pbl_reset_sf32lb52 *rst);

/** @} */

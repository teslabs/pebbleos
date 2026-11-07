/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/drivers/clk.h>

#include <dt-bindings/clock/sifli,sf32lb52-hpsys-rcc.h>

/**
 * @defgroup drivers_clk_sf32lb SF32LB RCC clocks
 * @ingroup drivers_clk
 * @brief @ref drivers_clk controller of the SF32LB52 HPSYS RCC.
 *
 * Clock numbers are (register index << 5) | bit, register 0 being ENR1.
 * @{
 */

/** @cond INTERNAL_HIDDEN */
extern const struct pbl_clk_controller pbl_clk_sf32lb_hpsys_rcc;
/** @endcond */

/** HPSYS RCC clock controller. */
#define SF32LB_HPSYS_RCC (&pbl_clk_sf32lb_hpsys_rcc)

/** @cond INTERNAL_HIDDEN */
extern const struct pbl_clk_controller pbl_clk_sf32lb_rc10k;
/** @endcond */

/**
 * @brief RC10K low-power oscillator, calibrated against the 48 MHz crystal.
 *
 * Its rate is the filtered estimate, updated every @c CONFIG_CLK_SF32LB_RC10K_CAL_PERIOD_S
 * seconds from the tick interrupt, and a placeholder until the first gate, with or without
 * Bluetooth; see the RC10K calibration page in the driver documentation. The clock always runs.
 */
#define SF32LB_RC10K (&pbl_clk_sf32lb_rc10k)

/** RC10K clock number. */
#define SF32LB52_CLK_RC10K 0

/** @} */

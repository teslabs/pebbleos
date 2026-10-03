/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

/**
 * @defgroup drivers_pinctrl_sf32lb52 SF32LB52 pin controller
 * @ingroup drivers
 * @brief Pin multiplexing and pad configuration of the SF32LB52 HPSYS pads.
 *
 * A pin is a @ref PBL_PINCTRL_SF32LB52_PIN value or'ed with the pad
 * configuration bits below.
 * @{
 */

/** @cond INTERNAL_HIDDEN */
#define PBL_PINCTRL_SF32LB52_FSEL_MSK  0x0000000FU
#define PBL_PINCTRL_SF32LB52_PORT_POS  12U
#define PBL_PINCTRL_SF32LB52_PORT_MSK  0x00003000U
#define PBL_PINCTRL_SF32LB52_PAD_POS   14U
#define PBL_PINCTRL_SF32LB52_PAD_MSK   0x003FC000U
#define PBL_PINCTRL_SF32LB52_FIELD_POS 22U
#define PBL_PINCTRL_SF32LB52_FIELD_MSK 0x00C00000U
#define PBL_PINCTRL_SF32LB52_PINR_POS  24U
#define PBL_PINCTRL_SF32LB52_PINR_MSK  0xFF000000U
/** @endcond */

/** SA port. */
#define PBL_PINCTRL_SF32LB52_PORT_SA 0U
/** PA port. */
#define PBL_PINCTRL_SF32LB52_PORT_PA 1U

/**
 * @brief Pin function.
 *
 * @param port Port.
 * @param pad Pad.
 * @param fsel Function select.
 * @param pinr HPSYS_CFG PINR register offset of a crossbar function, 0 if none.
 * @param field PINR field (signal) of a crossbar function.
 */
#define PBL_PINCTRL_SF32LB52_PIN(port, pad, fsel, pinr, field) \
  (((uint32_t)(pinr) << PBL_PINCTRL_SF32LB52_PINR_POS) |       \
   ((uint32_t)(field) << PBL_PINCTRL_SF32LB52_FIELD_POS) |     \
   ((uint32_t)(pad) << PBL_PINCTRL_SF32LB52_PAD_POS) |         \
   ((uint32_t)(port) << PBL_PINCTRL_SF32LB52_PORT_POS) | (uint32_t)(fsel))

/** Pull enabled. */
#define PBL_PINCTRL_SF32LB52_PE (1U << 4)
/** Pull-up, with @ref PBL_PINCTRL_SF32LB52_PE. */
#define PBL_PINCTRL_SF32LB52_PS (1U << 5)
/** Input buffer enabled. */
#define PBL_PINCTRL_SF32LB52_IE (1U << 6)
/** Slow slew rate (MODE on PA39-PA42, set by the driver). */
#define PBL_PINCTRL_SF32LB52_SR (1U << 8)
/** Drive strength, by index into 2, 8, 4, 12 and 20 mA (the driver sets DS). */
#define PBL_PINCTRL_SF32LB52_DS_IDX(idx) ((uint32_t)(idx) << 9)

/** @brief Pin controller. */
struct pbl_pinctrl_sf32lb52 {
  /** PAD_SA registers. */
  uintptr_t pad_sa;
  /** PAD_PA registers. */
  uintptr_t pad_pa;
  /** HPSYS_CFG registers, holding the PINR crossbar. */
  uintptr_t cfg;
};

/** @brief Pin configuration state. */
struct pbl_pinctrl_sf32lb52_state {
  /** Pin controller. */
  const struct pbl_pinctrl_sf32lb52 *ctrl;
  /** Pins. */
  const uint32_t *pins;
  /** Number of pins. */
  uint8_t num_pins;
};

/**
 * @brief Apply a pin configuration state.
 *
 * @param state State.
 * @return 0, or -EINVAL for an unsupported drive strength.
 */
int pbl_pinctrl_sf32lb52_apply(const struct pbl_pinctrl_sf32lb52_state *state);

/** @} */

/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/drivers/i2c.h>

#include "board/board.h"

struct pbl_i2c_sf32lb_state {
  I2C_HandleTypeDef hdl;
  bool deepsleep_blocked;
};

struct pbl_i2c_sf32lb {
  struct pbl_i2c_bus bus;
  struct pbl_i2c_sf32lb_state *state;
  Pinmux scl;
  Pinmux sda;
  RCC_MODULE_TYPE module;
  IRQn_Type irqn;
  uint8_t irq_priority;
};

extern const struct pbl_i2c_bus_ops pbl_i2c_sf32lb_ops;

void pbl_i2c_sf32lb_irq_handler(const struct pbl_i2c_bus *bus);

//! Defines the bus @p sym on the controller @p _inst (I2C1..I2C4), in 7-bit
//! master mode at @p _clock_hz, and maps its IRQ.
#define PBL_I2C_SF32LB_DEFINE(sym, _name, _inst, _irq_prio, _clock_hz, _scl, _sda, _deps) \
  PBL_I2C_BUS_STATE_DEFINE(sym);                                                          \
  static struct pbl_i2c_sf32lb_state sym##_sf32lb_state = {                               \
    .hdl = {                                                                              \
      .Instance = _inst,                                                                  \
      .Init = {                                                                           \
        .AddressingMode = I2C_ADDRESSINGMODE_7BIT,                                        \
        .ClockSpeed = (_clock_hz),                                                        \
        .GeneralCallMode = I2C_GENERALCALL_DISABLE,                                       \
      },                                                                                  \
      .Mode = HAL_I2C_MODE_MASTER,                                                        \
      .core = CORE_ID_HCPU,                                                               \
    },                                                                                    \
  };                                                                                      \
  const struct pbl_i2c_sf32lb sym = {                                                     \
    .bus = PBL_I2C_BUS_INIT(sym, _name, &pbl_i2c_sf32lb_ops, _deps),                      \
    .state = &sym##_sf32lb_state,                                                         \
    .scl = _scl,                                                                          \
    .sda = _sda,                                                                          \
    .module = RCC_MOD_##_inst,                                                            \
    .irqn = _inst##_IRQn,                                                                 \
    .irq_priority = (_irq_prio),                                                          \
  };                                                                                      \
  PBL_DEVICE_REGISTER(sym, &sym.bus.dev);                                                 \
  void _inst##_IRQHandler(void) { pbl_i2c_sf32lb_irq_handler(&sym.bus); }                 \
  _Static_assert(IS_VALID_IRQ__##_inst || true, "")

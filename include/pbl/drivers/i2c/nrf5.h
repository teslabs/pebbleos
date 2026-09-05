/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/drivers/gpio/nrf5.h>
#include <pbl/drivers/i2c.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include <nrfx_twim.h>
#pragma GCC diagnostic pop

// Register address + payload of a register write, sent as one transfer.
#define PBL_I2C_NRF5_REG_WRITE_BUF_SIZE 16

struct pbl_i2c_nrf5_state {
  uint8_t reg_write_buf[PBL_I2C_NRF5_REG_WRITE_BUF_SIZE];
};

struct pbl_i2c_nrf5 {
  struct pbl_i2c_bus bus;
  struct pbl_i2c_nrf5_state *state;
  nrfx_twim_t twim;
  nrf_twim_frequency_t frequency;
  struct pbl_gpio scl;
  struct pbl_gpio sda;
};

extern const struct pbl_i2c_bus_ops pbl_i2c_nrf5_ops;

//! Defines the TWIM @p _idx bus @p sym. The IRQ still has to be mapped with
//! IRQ_MAP_NRFX(<irq>, nrfx_twim_<_idx>_irq_handler).
#define PBL_I2C_NRF5_DEFINE(sym, _name, _idx, _freq, _scl, _sda, _deps)  \
  PBL_I2C_BUS_STATE_DEFINE(sym);                                        \
  static struct pbl_i2c_nrf5_state sym##_nrf5_state;                    \
  const struct pbl_i2c_nrf5 sym = {                                     \
    .bus = PBL_I2C_BUS_INIT(sym, _name, &pbl_i2c_nrf5_ops, _deps),      \
    .state = &sym##_nrf5_state,                                         \
    .twim = NRFX_TWIM_INSTANCE(_idx),                                   \
    .frequency = (_freq),                                               \
    .scl = _scl,                                                        \
    .sda = _sda,                                                        \
  };                                                                    \
  PBL_DEVICE_REGISTER(sym, &sym.bus.dev)

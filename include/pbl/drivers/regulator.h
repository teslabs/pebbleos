/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/device.h>
#include "pbl/kernel/mutex.h"

#include <stdbool.h>
#include <stdint.h>

struct pbl_regulator;

struct pbl_regulator_ops {
  //! Optional. Applies the rail's static configuration (voltage, mode).
  int (*init)(const struct pbl_regulator *reg);
  int (*enable)(const struct pbl_regulator *reg);
  int (*disable)(const struct pbl_regulator *reg);
};

struct pbl_regulator_state {
  struct pbl_mutex mutex;
  uint16_t use_count;
};

//! A power rail. Consumers enable and disable it with a use count; a rail
//! marked always_on is switched on at init and stays on.
struct pbl_regulator {
  struct pbl_device dev;
  const struct pbl_regulator_ops *ops;
  struct pbl_regulator_state *state;
  bool always_on;
};

#define PBL_REGULATOR_STATE_DEFINE(sym) \
  PBL_DEVICE_STATE_DEFINE(sym);         \
  static struct pbl_regulator_state sym##_regulator_state

#define PBL_REGULATOR_INIT(sym, _name, _parent, _deps, _ops, _always_on)      \
  {                                                                           \
    .dev = PBL_DEVICE_INIT(sym, _name, pbl_regulator_dev_init, _parent, _deps), \
    .ops = (_ops),                                                            \
    .state = &sym##_regulator_state,                                          \
    .always_on = (_always_on),                                                \
  }

//! The struct pbl_device init of every regulator: ops->init, then the rail is
//! switched to its boot state (on if always_on, off otherwise).
int pbl_regulator_dev_init(const struct pbl_device *dev);

//! @return 0 or a negative errno. Balanced with pbl_regulator_disable().
int pbl_regulator_enable(const struct pbl_regulator *reg);
int pbl_regulator_disable(const struct pbl_regulator *reg);

bool pbl_regulator_is_enabled(const struct pbl_regulator *reg);

/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/device.h>
#include <pbl/drivers/gpio.h>
#include <pbl/drivers/i2c.h>
#include <pbl/drivers/regulator.h>
#include "pbl/kernel/mutex.h"
#include "pbl/services/new_timer/new_timer.h"

#include "board/board.h"

//! Charger configuration
typedef struct {
  //! Charge current (32-800mA, 2mA steps)
  uint16_t chg_current_ma;
  //! Discharge limit (200mA or 1000mA)
  uint16_t dischg_limit_ma;
  //! Termination current (% of charge current, 10 or 20%)
  uint8_t term_current_pct;
  //! Thermistor beta value
  uint16_t thermistor_beta;
  //! NTC HOT threshold in Celsius (charging stops above this)
  uint8_t ntc_hot_celsius;
  //! Termination voltage (3500-3650 or 4000-4450mV, 50mV steps)
  uint16_t vterm_mv;
  //! Termination voltage when the battery is warm/cool, same range
  uint16_t vterm_reduced_mv;
  //! Vbus current limite0
  uint16_t vbus_current_lim0;
  //! Vbus current limite startup
  uint16_t vbus_current_startup;
} Npm1300Config;

//! Maximum discharge current in mA
#define NPM1300_DISCHG_LIMIT_MA_MAX 1000UL

struct pbl_npm1300_state {
  //! Serialises register sequences across the charger, GPIO and regulator functions
  struct pbl_mutex lock;
  uint32_t dischg_limit_ma;
  TimerID debounce_charger_timer;
};

//! The PMIC: an I2C peripheral that is also the parent of its GPIO port and
//! of the regulators defined with PBL_NPM1300_REGULATOR_DEFINE. Its init
//! brings every child up.
struct pbl_npm1300 {
  struct pbl_device dev;
  struct pbl_i2c_dev i2c;
  ExtiConfig irq;
  const Npm1300Config *cfg;
  struct pbl_npm1300_state *state;
  struct pbl_gpio_port gpio;
};

//! The board's PMIC and its charger configuration
extern const struct pbl_npm1300 *const NPM1300;
extern const Npm1300Config NPM1300_CONFIG;

#define NPM1300_GPIO (&NPM1300->gpio)

int pbl_npm1300_init(const struct pbl_device *dev);
extern const struct pbl_gpio_port_ops pbl_npm1300_gpio_ops;

//! Building blocks for a board's PMIC instance: the states, the device and
//! the GPIO port child. The I2C address, IRQ and config are plain fields.
#define PBL_NPM1300_STATE_DEFINE(sym)     \
  PBL_DEVICE_STATE_DEFINE(sym);           \
  PBL_DEVICE_STATE_DEFINE(sym##_gpio);    \
  static struct pbl_npm1300_state sym##_npm1300_state

#define PBL_NPM1300_DEV_INIT(sym, _name, _bus, _deps) \
  PBL_DEVICE_INIT(sym, _name, pbl_npm1300_init, &(_bus)->dev, _deps)

#define PBL_NPM1300_GPIO_INIT(sym, _name)                                        \
  {                                                                              \
    .dev = PBL_DEVICE_INIT(sym##_gpio, _name, NULL, &sym.dev, NULL),             \
    .ops = &pbl_npm1300_gpio_ops,                                                \
  }

#define PBL_NPM1300_REGISTER(sym)          \
  PBL_DEVICE_REGISTER(sym, &sym.dev);      \
  PBL_DEVICE_REGISTER(sym##_gpio, &sym.gpio.dev)

//! Register access. Single accesses are atomic on their own; hold the lock
//! around read-modify-write sequences.
void pbl_npm1300_lock(const struct pbl_npm1300 *pmic);
void pbl_npm1300_unlock(const struct pbl_npm1300 *pmic);
bool pbl_npm1300_read(const struct pbl_npm1300 *pmic, uint16_t reg, uint8_t *val);
bool pbl_npm1300_write(const struct pbl_npm1300 *pmic, uint16_t reg, uint8_t val);
bool pbl_npm1300_update(const struct pbl_npm1300 *pmic, uint16_t reg, uint8_t mask, uint8_t val);

//! Battery discharge current limit: 200 or 1000 mA
bool pbl_npm1300_set_dischg_limit_ma(const struct pbl_npm1300 *pmic, uint32_t ma);

// Regulators

enum pbl_npm1300_rail {
  PBL_NPM1300_BUCK1,
  PBL_NPM1300_BUCK2,
  PBL_NPM1300_LDSW1,
  PBL_NPM1300_LDSW2,
};

struct pbl_npm1300_regulator {
  struct pbl_regulator reg;
  enum pbl_npm1300_rail rail;
  //! Output voltage, 1000-3300mV in 100mV steps
  uint16_t voltage_mv;
  //! LDSW rails only: LDO rather than load switch
  bool ldo;
};

extern const struct pbl_regulator_ops pbl_npm1300_regulator_ops;

//! Defines the rail @p _rail of the PMIC @p _pmic (a struct pbl_npm1300) as
//! the regulator @p sym.reg.
#define PBL_NPM1300_REGULATOR_DEFINE(sym, _name, _pmic, _rail, _voltage_mv, _ldo, _always_on) \
  PBL_REGULATOR_STATE_DEFINE(sym);                                                             \
  const struct pbl_npm1300_regulator sym = {                                                   \
    .reg = PBL_REGULATOR_INIT(sym, _name, &(_pmic)->dev, NULL, &pbl_npm1300_regulator_ops,   \
                              _always_on),                                                    \
    .rail = (_rail),                                                                           \
    .voltage_mv = (_voltage_mv),                                                               \
    .ldo = (_ldo),                                                                             \
  };                                                                                           \
  PBL_DEVICE_REGISTER(sym, &sym.reg.dev)

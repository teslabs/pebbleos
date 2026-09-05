/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/pmic/npm1300.h>

#include <errno.h>

// BUCK block: per-rail registers at base + rail * stride
#define NPM1300_BUCK_ENASET(n) (0x0400 + 2 * (n))
#define NPM1300_BUCK_ENACLR(n) (0x0401 + 2 * (n))
#define NPM1300_BUCK_NORMVOUT(n) (0x0408 + 2 * (n))
#define NPM1300_BUCK_SWCTRLSEL 0x040F
#define NPM1300_BUCK_VOUTSTATUS(n) (0x0410 + (n))

// LDSW block
#define NPM1300_LDSW_TASKSET(n) (0x0800 + 2 * (n))
#define NPM1300_LDSW_TASKCLR(n) (0x0801 + 2 * (n))
#define NPM1300_LDSW_STATUS 0x0804
#define NPM1300_LDSW_STATUS_PWRUPLDSW(n) (1U << (2 * (n)))
#define NPM1300_LDSW_STATUS_PWRUPLDO(n) (1U << (2 * (n) + 1))
#define NPM1300_LDSW_LDOSEL(n) (0x0808 + (n))
#define NPM1300_LDSW_VOUTSEL(n) (0x080C + (n))

static const struct pbl_npm1300_regulator *prv_dev(const struct pbl_regulator *reg) {
  return PBL_CONTAINER_OF(reg, const struct pbl_npm1300_regulator, reg);
}

static const struct pbl_npm1300 *prv_pmic(const struct pbl_regulator *reg) {
  return PBL_CONTAINER_OF(reg->dev.parent, const struct pbl_npm1300, dev);
}

static bool prv_is_buck(const struct pbl_npm1300_regulator *r) {
  return r->rail == PBL_NPM1300_BUCK1 || r->rail == PBL_NPM1300_BUCK2;
}

static unsigned prv_index(const struct pbl_npm1300_regulator *r) {
  return prv_is_buck(r) ? r->rail - PBL_NPM1300_BUCK1 : r->rail - PBL_NPM1300_LDSW1;
}

// Both blocks encode 1.0V + 100mV * code
static uint8_t prv_vout_code(uint16_t mv) {
  return (mv - 1000U) / 100U;
}

// Anomaly 27: when switching a BUCK to SW control, if its NORMVOUT equals the
// VSET pin value (VOUTSTATUS), quiescent current increases by 1mA. Program a
// different value first, switch to SW control, then set the desired one.
static bool prv_buck_init(const struct pbl_npm1300 *pmic, unsigned n, uint8_t vout) {
  uint8_t voutstatus;

  pbl_npm1300_lock(pmic);
  bool ok = pbl_npm1300_read(pmic, NPM1300_BUCK_VOUTSTATUS(n), &voutstatus);
  uint8_t initial_vout = (vout != voutstatus) ? vout : (vout ^ 1);
  ok &= pbl_npm1300_write(pmic, NPM1300_BUCK_NORMVOUT(n), initial_vout);
  ok &= pbl_npm1300_update(pmic, NPM1300_BUCK_SWCTRLSEL, 1U << n, 1U << n);
  if (initial_vout != vout) {
    ok &= pbl_npm1300_write(pmic, NPM1300_BUCK_NORMVOUT(n), vout);
  }
  pbl_npm1300_unlock(pmic);
  return ok;
}

static bool prv_ldsw_init(const struct pbl_npm1300 *pmic, unsigned n, bool ldo, uint8_t vout) {
  uint8_t status;

  pbl_npm1300_lock(pmic);
  bool ok = pbl_npm1300_read(pmic, NPM1300_LDSW_STATUS, &status);
  if (ldo && (status & NPM1300_LDSW_STATUS_PWRUPLDO(n))) {
    // Already up in the requested mode: only adjust the voltage, no glitch
    ok &= pbl_npm1300_write(pmic, NPM1300_LDSW_VOUTSEL(n), vout);
  } else {
    if (status & (NPM1300_LDSW_STATUS_PWRUPLDSW(n) | NPM1300_LDSW_STATUS_PWRUPLDO(n))) {
      ok &= pbl_npm1300_write(pmic, NPM1300_LDSW_TASKCLR(n), 1);
    }
    ok &= pbl_npm1300_write(pmic, NPM1300_LDSW_LDOSEL(n), ldo ? 1 : 0);
    if (ldo) {
      ok &= pbl_npm1300_write(pmic, NPM1300_LDSW_VOUTSEL(n), vout);
    }
  }
  pbl_npm1300_unlock(pmic);
  return ok;
}

static int prv_init(const struct pbl_regulator *reg) {
  const struct pbl_npm1300_regulator *r = prv_dev(reg);
  bool ok;

  if (prv_is_buck(r)) {
    ok = prv_buck_init(prv_pmic(reg), prv_index(r), prv_vout_code(r->voltage_mv));
  } else {
    ok = prv_ldsw_init(prv_pmic(reg), prv_index(r), r->ldo,
                       r->ldo ? prv_vout_code(r->voltage_mv) : 0);
  }
  return ok ? 0 : -EIO;
}

static int prv_set(const struct pbl_regulator *reg, bool on) {
  const struct pbl_npm1300_regulator *r = prv_dev(reg);
  unsigned n = prv_index(r);
  uint16_t task;

  if (prv_is_buck(r)) {
    task = on ? NPM1300_BUCK_ENASET(n) : NPM1300_BUCK_ENACLR(n);
  } else {
    task = on ? NPM1300_LDSW_TASKSET(n) : NPM1300_LDSW_TASKCLR(n);
  }
  return pbl_npm1300_write(prv_pmic(reg), task, 1) ? 0 : -EIO;
}

static int prv_enable(const struct pbl_regulator *reg) {
  return prv_set(reg, true);
}

static int prv_disable(const struct pbl_regulator *reg) {
  return prv_set(reg, false);
}

const struct pbl_regulator_ops pbl_npm1300_regulator_ops = {
  .init = prv_init,
  .enable = prv_enable,
  .disable = prv_disable,
};

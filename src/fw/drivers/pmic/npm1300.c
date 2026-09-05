/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

/* Because nPM1300 also has the battery monitor, we implement both the
 * pmic_* and the battery_* API here.  */

#include <errno.h>
#include <math.h>

#include <pbl/drivers/pmic.h>
#include <pbl/drivers/pmic/npm1300.h>
#include <pbl/drivers/battery.h>

#include "board/board.h"
#include "console/prompt.h"
#include <pbl/drivers/exti.h>
#include <pbl/drivers/i2c.h>
#include "system/passert.h"
#include "kernel/events.h"
#include "kernel/util/delay.h"
#include "kernel/util/sleep.h"
#include "pbl/services/system_task.h"
#include <pbl/logging/logging.h>

PBL_LOG_MODULE_DEFINE(driver_pmic_npm1300, CONFIG_DRIVER_PMIC_LOG_LEVEL);

#define CHARGER_DEBOUNCE_MS 400
#define ADC_POLL_DELAY_MS   5     // Delay between ADC poll iterations to reduce I2C traffic
#define ADC_POLL_TIMEOUT_MS 100   // Max time to wait for ADC measurement

typedef enum {
  PmicRegisters_MAIN_EVENTSADCCLR = 0x0003,
  PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCVBATRDY = 0x01,
  PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCNTCRDY = 0x02,
  PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCIBATRDY = 0x40,
  PmicRegisters_MAIN_EVENTSBCHARGER1CLR = 0x000B,
  PmicRegisters_MAIN_INTENEVENTSBCHARGER1SET = 0x000C,
  PmicRegisters_MAIN_EVENTSBCHARGER1__EVENTCHGCOMPLETED = 16,
  PmicRegisters_MAIN_EVENTSVBUSIN0CLR = 0x0017,
  PmicRegisters_MAIN_INTENEVENTSVBUSIN0SET = 0x0018,
  PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSDETECTED = 1,
  PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSREMOVED = 2,
  PmicRegisters_SYSTEM_TESTACCESS = 0x0123,
  PmicRegisters_SYSTEM_TESTACCESS__VAL0 = 0x44,
  PmicRegisters_SYSTEM_TESTACCESS__VAL1 = 0x90,
  PmicRegisters_SYSTEM_TESTACCESS__VAL2 = 0xFA,
  PmicRegisters_SYSTEM_TESTACCESS__VAL3 = 0xCE,
  PmicRegisters_VBUSIN_TASKUPDATELIMSW = 0x0200,
  PmicRegisters_VBUSIN_TASKUPDATELIMSW__EN = 0x01,
  PmicRegisters_VBUSIN_VBUSINILIM0 = 0x0201,
  PmicRegisters_VBUSIN_VBUSINILIMSTARTUP = 0x0202,
  PmicRegisters_VBUSIN_VBUSINSTATUS = 0x0207,
  PmicRegisters_VBUSIN_VBUSINSTATUS__VBUSINPRESENT = 1,
  PmicRegisters_BCHARGER_TASKRELEASEERROR = 0x0300U,
  PmicRegisters_BCHARGER_TASKCLEARCHGERR = 0x0301U,
  PmicRegisters_BCHARGER_BCHGENABLESET = 0x0304,
  PmicRegisters_BCHARGER_BCHGENABLECLR = 0x0305,
  PmicRegisters_BCHARGER_BCHGISETMSB = 0x0308,
  PmicRegisters_BCHARGER_BCHGISETLSB = 0x0309,
  PmicRegisters_BCHARGER_BCHGISETDISCHARGEMSB = 0x030A,
  PmicRegisters_BCHARGER_BCHGISETDISCHARGELSB = 0x30B,
  PmicRegisters_BCHARGER_BCHGVTERM = 0x030CU,
  PmicRegisters_BCHARGER_BCHGVTERMR = 0x030DU,
  PmicRegisters_BCHARGER_BCHGITERMSEL = 0x030F,
  PmicRegisters_BCHARGER_BCHGITERMSEL__SEL10 = 0U,
  PmicRegisters_BCHARGER_BCHGITERMSEL__SEL20 = 1U,
  PmicRegisters_BCHARGER_NTCHOT = 0x0316U,
  PmicRegisters_BCHARGER_NTCHOTLSB = 0x0317U,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS = 0x0334,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS__COMPLETED = 2,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS__TRICKLECHARGE = 4,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTCURRENT = 8,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTVOLTAGE = 16,
  PmicRegisters_BCHARGER_BCHGERRREASON = 0x0336,
  PmicRegisters_BCHARGER_BCHGDEBUG = 0x0346,
  PmicRegisters_BCHARGER_BCHGDEBUG__DISABLEBATTERYDETECT = 0x04,
  PmicRegisters_BCHARGER_BCHGVBATLOWCHARGE = 0x0350,
  PmicRegisters_ADC_TASKVBATMEASURE  = 0x0500,
  PmicRegisters_ADC_TASKNTCMEASURE   = 0x0501,
  PmicRegisters_ADC_TASKVSYSMEASURE  = 0x0503,
  PmicRegisters_ADC_TASKIBATMEASURE  = 0x0506,
  PmicRegisters_ADC_TASKVBUS7MEASURE = 0x0507,
  PmicRegisters_ADC_ADCIBATMEASSTATUS = 0x0510,
  PmicRegisters_ADC_ADCIBATMEASSTATUS__BCHARGERMODE_MASK = 0x0C,
  PmicRegisters_ADC_ADCIBATMEASSTATUS__BCHARGERMODE_DISCHRG = 0x04,
  PmicRegisters_ADC_ADCIBATMEASSTATUS__BCHARGERMODE_CHRG = 0x0C,
  PmicRegisters_ADC_ADCNTCRSEL = 0x050AU,
  PmicRegisters_ADC_ADCNTCRSEL__ADCNTCRSEL_HIZ = 0x0U,
  PmicRegisters_ADC_ADCNTCRSEL__ADCNTCRSEL_10K = 0x1U,
  PmicRegisters_ADC_ADCVBATRESULTMSB = 0x0511,
  PmicRegisters_ADC_ADCNTCRESULTMSB = 0x512,
  PmicRegisters_ADC_ADCVSYSRESULTMSB = 0x0514,
  PmicRegisters_ADC_ADCGP0RESULTLSBS = 0x0515,
  PmicRegisters_ADC_ADCGP0RESULTLSBS_VBATRESULTLSB_MSK = 0x03,
  PmicRegisters_ADC_ADCGP0RESULTLSBS_VBATRESULTLSB_POS = 0U,
  PmicRegisters_ADC_ADCGP0RESULTLSBS_NTCRESULTLSB_MSK = 0x03,
  PmicRegisters_ADC_ADCGP0RESULTLSBS_NTCRESULTLSB_POS = 2U,
  PmicRegisters_ADC_ADCVBAT2RESULTMSB = 0x0518,
  PmicRegisters_ADC_ADCGP1RESULTLSBS = 0x051a,
  PmicRegisters_ADC_ADCGP1RESULTLSBS_VBAT2RESULTLSB_MSK = 0x03,
  PmicRegisters_ADC_ADCGP1RESULTLSBS_VBAT2RESULTLSB_POS = 0x04,
  PmicRegisters_ADC_ADCIBATMEASEN = 0x0524,
  PmicRegisters_GPIOS_GPIOMODE1 = 0x0601,
  PmicRegisters_GPIOS_GPIOMODE__GPOIRQ = 5,
  PmicRegisters_GPIOS_GPIOOPENDRAIN1 = 0x0615,
  PmicRegisters_ERRLOG_SCRATCH0 = 0x0E01,
  PmicRegisters_ERRLOG_SCRATCH1 = 0x0E02,
  PmicRegisters_BUCK_BUCK1NORMVOUT = 0x0408,
  PmicRegisters_BUCK_BUCK2NORMVOUT = 0x040A,
  PmicRegisters_BUCK_BUCKSTATUS = 0x0434,
  PmicRegisters_SHIP_TASKSHPHLDCFGSTROBE = 0x0B01,
  PmicRegisters_SHIP_TASKENTERSHIPMODE = 0x0B02,
  PmicRegisters_SHIP_SHPHLDCONFIG = 0x0B04,
  PmicRegisters_SHIP_SHPHLDCONFIG__SHPHLDTIM_96MS = 3,
} PmicRegisters;

#define NPM1300_BCHGISETDISCHARGEMSB_200MA 42U
#define NPM1300_BCHGISETDISCHARGELSB_200MA 0U
#define NPM1300_BCHGISETDISCHARGEMSB_1000MA 207U
#define NPM1300_BCHGISETDISCHARGELSB_1000MA 1U

#define NPM1300_BCHARGER_ADC_BITS_RESOLUTION 1023
#define NPM1300_BCHARGER_ADC_CALC_DISCHARGE_MUL 112
#define NPM1300_BCHARGER_ADC_CALC_DISCHARGE_DIV 100
#define NPM1300_BCHARGER_ADC_CALC_CHARGE_MUL 1250
#define NPM1300_BCHARGER_ADC_CALC_CHARGE_DIV -1000
// Full scale voltage for battery voltage measurement
#define NPM1300_ADC_VFS_VBAT_MV 5000UL
// ADC MSB shift
#define NPM1300_ADC_MSB_SHIFT 2U
#define NPM1300_VBUS_CURRENT_DIVISOR 100U

static uint16_t prv_ntc_threshold_code(uint8_t celsius) {
  // Ref: PS v1.1 Section 6.2.5: K_NTCTEMP = round(1024 * R_T / (R_T + R_B))
  float t_k = (float)celsius + 273.15f;
  float exponent = (float)NPM1300->cfg->thermistor_beta *
                   ((1.f / 298.15f) - (1.f / t_k));
  return (uint16_t)((1024.0f / (1.0f + exp(exponent))) + 0.5f);
}

void battery_init(void) {
}

void pbl_npm1300_lock(const struct pbl_npm1300 *pmic) {
  pbl_mutex_lock(&pmic->state->lock, PBL_FOREVER);
}

void pbl_npm1300_unlock(const struct pbl_npm1300 *pmic) {
  pbl_mutex_unlock(&pmic->state->lock);
}

bool pbl_npm1300_read(const struct pbl_npm1300 *pmic, uint16_t reg, uint8_t *val) {
  uint8_t regad[2] = { reg >> 8, reg & 0xFF };

  pbl_npm1300_lock(pmic);
  pbl_i2c_use(&pmic->i2c);
  bool rv = pbl_i2c_write_read_block(&pmic->i2c, 2, regad, 1, val);
  pbl_i2c_release(&pmic->i2c);
  pbl_npm1300_unlock(pmic);
  return rv;
}

bool pbl_npm1300_write(const struct pbl_npm1300 *pmic, uint16_t reg, uint8_t val) {
  uint8_t d[3] = { reg >> 8, reg & 0xFF, val };

  pbl_npm1300_lock(pmic);
  pbl_i2c_use(&pmic->i2c);
  bool rv = pbl_i2c_write_block(&pmic->i2c, 3, d);
  pbl_i2c_release(&pmic->i2c);
  pbl_npm1300_unlock(pmic);
  return rv;
}

bool pbl_npm1300_update(const struct pbl_npm1300 *pmic, uint16_t reg, uint8_t mask, uint8_t val) {
  uint8_t cur;

  pbl_npm1300_lock(pmic);
  bool rv = pbl_npm1300_read(pmic, reg, &cur) &&
            pbl_npm1300_write(pmic, reg, (cur & ~mask) | (val & mask));
  pbl_npm1300_unlock(pmic);
  return rv;
}

static bool prv_read_register(uint16_t register_address, uint8_t *result) {
  return pbl_npm1300_read(NPM1300, register_address, result);
}

static bool prv_write_register(uint16_t register_address, uint8_t datum) {
  return pbl_npm1300_write(NPM1300, register_address, datum);
}

static void prv_handle_charge_state_change(void *null) {
  const bool is_charging = pmic_is_charging();
  const bool is_connected = pmic_is_usb_connected();
  PBL_LOG_DBG("nPM1300 Interrupt: Charging? %s Plugged? %s",
      is_charging ? "YES" : "NO", is_connected ? "YES" : "NO");

  if (is_connected && NPM1300->cfg->vbus_current_lim0 != 0) {
    bool ok = prv_write_register(PmicRegisters_VBUSIN_VBUSINILIM0,
      NPM1300->cfg->vbus_current_lim0/NPM1300_VBUS_CURRENT_DIVISOR);
    ok &= prv_write_register(PmicRegisters_VBUSIN_TASKUPDATELIMSW,
      PmicRegisters_VBUSIN_TASKUPDATELIMSW__EN);
    if (!ok) {
      PBL_LOG_ERR("config vbus limite0 failed");
    }
  }

  PebbleEvent event = {
    .type = PEBBLE_BATTERY_CONNECTION_EVENT,
    .battery_connection = {
      .is_connected = battery_is_usb_connected(),
    },
  };
  event_put(&event);
}

static void prv_clear_pending_interrupts() {
  prv_write_register(PmicRegisters_MAIN_EVENTSBCHARGER1CLR, PmicRegisters_MAIN_EVENTSBCHARGER1__EVENTCHGCOMPLETED);
  prv_write_register(PmicRegisters_MAIN_EVENTSVBUSIN0CLR, PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSDETECTED | PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSREMOVED);
}

static void prv_pmic_state_change_cb(void *null) {
  prv_clear_pending_interrupts();
  new_timer_start(NPM1300->state->debounce_charger_timer, CHARGER_DEBOUNCE_MS,
                  prv_handle_charge_state_change, NULL, 0 /*flags*/);
}

static void prv_npm1300_interrupt_handler(bool *should_context_switch) {
  system_task_add_callback_from_isr(prv_pmic_state_change_cb, NULL, should_context_switch);
}

static void prv_configure_interrupts(void) {
  prv_clear_pending_interrupts();

  exti_configure_pin(NPM1300->irq, ExtiTrigger_Rising, prv_npm1300_interrupt_handler);
  exti_enable(NPM1300->irq);
}

// BCHGVTERM/BCHGVTERMR: 3.50-3.65V in codes 0-3, 4.00-4.45V in codes 4-13
static uint8_t prv_vterm_code(uint16_t mv) {
  return (mv < 4000U) ? (mv - 3500U) / 50U : 4U + (mv - 4000U) / 50U;
}

int pbl_npm1300_init(const struct pbl_device *dev) {
  const struct pbl_npm1300 *pmic = PBL_CONTAINER_OF(dev, const struct pbl_npm1300, dev);
  const Npm1300Config *cfg = pmic->cfg;
  bool ok = true;
  uint8_t val;

  PBL_ASSERTN(pmic == NPM1300);

  pbl_mutex_init(&pmic->state->lock);
  pmic->state->dischg_limit_ma = 0;
  pmic->state->debounce_charger_timer = new_timer_create();

  ok &= prv_write_register(PmicRegisters_MAIN_EVENTSBCHARGER1CLR, PmicRegisters_MAIN_EVENTSBCHARGER1__EVENTCHGCOMPLETED);
  ok &= prv_write_register(PmicRegisters_MAIN_INTENEVENTSBCHARGER1SET, PmicRegisters_MAIN_EVENTSBCHARGER1__EVENTCHGCOMPLETED);
  ok &= prv_write_register(PmicRegisters_MAIN_EVENTSVBUSIN0CLR, PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSDETECTED | PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSREMOVED);
  ok &= prv_write_register(PmicRegisters_MAIN_INTENEVENTSVBUSIN0SET, PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSDETECTED | PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSREMOVED);
  ok &= prv_write_register(PmicRegisters_GPIOS_GPIOMODE1, PmicRegisters_GPIOS_GPIOMODE__GPOIRQ);
  ok &= prv_write_register(PmicRegisters_GPIOS_GPIOOPENDRAIN1, 0);

  ok &= prv_write_register(PmicRegisters_SHIP_SHPHLDCONFIG, PmicRegisters_SHIP_SHPHLDCONFIG__SHPHLDTIM_96MS);
  ok &= prv_write_register(PmicRegisters_SHIP_TASKSHPHLDCFGSTROBE, 1);

  // automatic IBAT measurement after VBAT
  ok &= prv_write_register(PmicRegisters_ADC_ADCIBATMEASEN, 1);

  if ((cfg->chg_current_ma < 32U) || (cfg->chg_current_ma > 800U) ||
      (cfg->chg_current_ma % 2U != 0U)) {
    PBL_LOG_ERR("Invalid charge current: %d mA", cfg->chg_current_ma);
    return -EINVAL;
  }

  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGENABLECLR, 1);

  ok &= prv_write_register(PmicRegisters_BCHARGER_TASKCLEARCHGERR, 1);
  ok &= prv_write_register(PmicRegisters_BCHARGER_TASKRELEASEERROR, 1);

  ok &= prv_write_register(PmicRegisters_ADC_ADCNTCRSEL, PmicRegisters_ADC_ADCNTCRSEL__ADCNTCRSEL_10K);
  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGVTERM, prv_vterm_code(cfg->vterm_mv));
  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGVTERMR, prv_vterm_code(cfg->vterm_reduced_mv));

  {
    uint16_t code = prv_ntc_threshold_code(cfg->ntc_hot_celsius);
    ok &= prv_write_register(PmicRegisters_BCHARGER_NTCHOT, (uint8_t)(code >> 2));
    ok &= prv_write_register(PmicRegisters_BCHARGER_NTCHOTLSB, (uint8_t)(code & 0x3U));
  }

  val = (uint8_t)(cfg->chg_current_ma / 4U);
  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGISETMSB, val);
  val = (cfg->chg_current_ma / 2U) % 2U;
  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGISETLSB, val);

  ok &= pbl_npm1300_set_dischg_limit_ma(pmic, cfg->dischg_limit_ma);

  if (cfg->vbus_current_startup != 0) {
    ok &= prv_write_register(PmicRegisters_VBUSIN_VBUSINILIMSTARTUP,
      cfg->vbus_current_startup/NPM1300_VBUS_CURRENT_DIVISOR);
  }

  if (cfg->term_current_pct == 10U) {
    ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGITERMSEL,
                             PmicRegisters_BCHARGER_BCHGITERMSEL__SEL10);
  } else if(cfg->term_current_pct == 20U) {
    ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGITERMSEL,
                             PmicRegisters_BCHARGER_BCHGITERMSEL__SEL20);
  } else {
    PBL_LOG_ERR("Invalid termination current: %d", cfg->term_current_pct);
    return -EINVAL;
  }

  ok &= prv_write_register(PmicRegisters_SYSTEM_TESTACCESS, 
                           PmicRegisters_SYSTEM_TESTACCESS__VAL0);
  ok &= prv_write_register(PmicRegisters_SYSTEM_TESTACCESS, 
                           PmicRegisters_SYSTEM_TESTACCESS__VAL1);
  ok &= prv_write_register(PmicRegisters_SYSTEM_TESTACCESS, 
                           PmicRegisters_SYSTEM_TESTACCESS__VAL2);
  ok &= prv_write_register(PmicRegisters_SYSTEM_TESTACCESS, 
                           PmicRegisters_SYSTEM_TESTACCESS__VAL3);

  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGDEBUG,
                           PmicRegisters_BCHARGER_BCHGDEBUG__DISABLEBATTERYDETECT);

  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGVBATLOWCHARGE, 1);

  prv_configure_interrupts();

  if (!ok) {
    PBL_LOG_ERR("one or more PMIC transactions failed");
    return -EIO;
  }

  // The GPIO port and the board's regulators
  return pbl_device_init_children(dev) == 0 ? 0 : -EIO;
}

bool pmic_power_off(void) {
  // TODO: review implementation, see GH-238
  if (pmic_is_usb_connected()) {
    PBL_LOG_ERR("USB is connected, cannot power off");
    return false;
  }

  if (!prv_write_register(PmicRegisters_SHIP_TASKENTERSHIPMODE, 1)) {
    PBL_LOG_ERR("Failed to enter ship mode");
    return false;
  }

  // Give enough time for the PMIC to fully power down (tPWRDN = 100ms).
  // We will die here, if we do not, return false and let upper layers handle
  // the shutdown failure.
  delay_us(100000);

  return false;
}

bool pmic_full_power_off(void) {
  return pmic_power_off();
}

uint16_t pmic_get_vsys(void) {
  if (!prv_write_register(PmicRegisters_MAIN_EVENTSADCCLR, 0x08 /* EVENTADCVSYSRDY */)) {
    return 0;
  }
  if (!prv_write_register(PmicRegisters_ADC_TASKVSYSMEASURE, 1)) {
    return 0;
  }
  uint8_t reg = 0;
  uint32_t elapsed = 0;
  while ((reg & 0x08) == 0) {
    if (elapsed >= ADC_POLL_TIMEOUT_MS) {
      return 0;  // Timeout waiting for ADC
    }
    if (!prv_read_register(PmicRegisters_MAIN_EVENTSADCCLR, &reg)) {
      return 0;
    }
    if ((reg & 0x08) == 0) {
      psleep(ADC_POLL_DELAY_MS);
      elapsed += ADC_POLL_DELAY_MS;
    }
  }
  
  uint8_t vsys_msb;
  uint8_t lsbs;
  if (!prv_read_register(PmicRegisters_ADC_ADCVSYSRESULTMSB, &vsys_msb)) {
    return 0;
  }
  if (!prv_read_register(PmicRegisters_ADC_ADCGP0RESULTLSBS, &lsbs)) {
    return 0;
  }
  uint16_t vsys_raw = (vsys_msb << 2) | (lsbs >> 6);
  uint32_t vsys = vsys_raw * 6375 / 1023;
  
  return vsys;
}

int battery_get_millivolts(void) {
  if (!prv_write_register(PmicRegisters_MAIN_EVENTSADCCLR, 0x01 /* EVENTADCVBATRDY */)) {
    return 0;
  }
  if (!prv_write_register(PmicRegisters_ADC_TASKVBATMEASURE, 1)) {
    return 0;
  }
  uint8_t reg = 0;
  uint32_t elapsed = 0;
  while ((reg & 0x01) == 0) {
    if (elapsed >= ADC_POLL_TIMEOUT_MS) {
      return 0;  // Timeout waiting for ADC
    }
    if (!prv_read_register(PmicRegisters_MAIN_EVENTSADCCLR, &reg)) {
      return 0;
    }
    if ((reg & 0x01) == 0) {
      psleep(ADC_POLL_DELAY_MS);
      elapsed += ADC_POLL_DELAY_MS;
    }
  }
  
  uint8_t vbat_msb;
  uint8_t lsbs;
  if (!prv_read_register(PmicRegisters_ADC_ADCVBATRESULTMSB, &vbat_msb)) {
    return 0;
  }
  if (!prv_read_register(PmicRegisters_ADC_ADCGP0RESULTLSBS, &lsbs)) {
    return 0;
  }
  uint16_t vbat_raw = (vbat_msb << 2) | (lsbs & 3);
  uint32_t vbat = vbat_raw * 5000 / 1023;
  
  return vbat;
}

int battery_get_constants(BatteryConstants *constants) {
  uint8_t ibat_status;
  int32_t full_scale_ua;
  uint8_t msb;
  uint8_t lsb;
  uint16_t raw;
  uint8_t reg;

  // Obtain IBAT full scale
  if (!prv_read_register(PmicRegisters_ADC_ADCIBATMEASSTATUS, &ibat_status)) {
    return -1;
  }

  if ((ibat_status & PmicRegisters_ADC_ADCIBATMEASSTATUS__BCHARGERMODE_MASK) ==
      PmicRegisters_ADC_ADCIBATMEASSTATUS__BCHARGERMODE_CHRG) {
    full_scale_ua =
        ((int32_t)NPM1300->cfg->chg_current_ma * 1000 * NPM1300_BCHARGER_ADC_CALC_CHARGE_MUL) /
        NPM1300_BCHARGER_ADC_CALC_CHARGE_DIV;
  } else {
    full_scale_ua =
        ((int32_t)NPM1300->state->dischg_limit_ma * 1000 * NPM1300_BCHARGER_ADC_CALC_DISCHARGE_MUL) /
        NPM1300_BCHARGER_ADC_CALC_DISCHARGE_DIV;
  }

  // Clear the ADC ready events for VBAT, IBAT, and NTC
  if (!prv_write_register(PmicRegisters_MAIN_EVENTSADCCLR,
                          PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCVBATRDY |
                          PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCIBATRDY |
                          PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCNTCRDY)) {
    return -1;
  }

  // Trigger VBAT+IBAT measurement (IBATMEASENABLE is enabled)
  if (!prv_write_register(PmicRegisters_ADC_TASKVBATMEASURE, 1)) {
    return -1;
  }

  // Trigger NTC measurement
  if (!prv_write_register(PmicRegisters_ADC_TASKNTCMEASURE, 1)) {
    return -1;
  }

  // Process the VBAT measurement
  reg = 0U;
  uint32_t elapsed = 0;
  while ((reg & PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCVBATRDY) == 0U) {
    if (elapsed >= ADC_POLL_TIMEOUT_MS) {
      return -1;  // Timeout waiting for VBAT ADC
    }
    if (!prv_read_register(PmicRegisters_MAIN_EVENTSADCCLR, &reg)) {
      return -1;
    }
    if ((reg & PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCVBATRDY) == 0U) {
      psleep(ADC_POLL_DELAY_MS);
      elapsed += ADC_POLL_DELAY_MS;
    }
  }

  if (!prv_read_register(PmicRegisters_ADC_ADCVBATRESULTMSB, &msb)) {
    return -1;
  }

  if (!prv_read_register(PmicRegisters_ADC_ADCGP0RESULTLSBS, &lsb)) {
    return -1;
  }

  raw = (msb << NPM1300_ADC_MSB_SHIFT) |
        ((lsb >> PmicRegisters_ADC_ADCGP0RESULTLSBS_VBATRESULTLSB_POS) &
         PmicRegisters_ADC_ADCGP0RESULTLSBS_VBATRESULTLSB_MSK);

  constants->v_mv = (int32_t)(raw * NPM1300_ADC_VFS_VBAT_MV) / NPM1300_BCHARGER_ADC_BITS_RESOLUTION;

  // Process the IBAT measurement
  elapsed = 0;
  while ((reg & PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCIBATRDY) == 0U) {
    if (elapsed >= ADC_POLL_TIMEOUT_MS) {
      return -1;  // Timeout waiting for IBAT ADC
    }
    if (!prv_read_register(PmicRegisters_MAIN_EVENTSADCCLR, &reg)) {
      return -1;
    }
    if ((reg & PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCIBATRDY) == 0U) {
      psleep(ADC_POLL_DELAY_MS);
      elapsed += ADC_POLL_DELAY_MS;
    }
  }

  if (!prv_read_register(PmicRegisters_ADC_ADCVBAT2RESULTMSB, &msb)) {
    return -1;
  }

  if (!prv_read_register(PmicRegisters_ADC_ADCGP1RESULTLSBS, &lsb)) {
    return -1;
  }

  raw = (msb << NPM1300_ADC_MSB_SHIFT) |
        ((lsb >> PmicRegisters_ADC_ADCGP1RESULTLSBS_VBAT2RESULTLSB_POS) &
         PmicRegisters_ADC_ADCGP1RESULTLSBS_VBAT2RESULTLSB_MSK);

  constants->i_ua = ((int32_t)raw * full_scale_ua) / NPM1300_BCHARGER_ADC_BITS_RESOLUTION;

  // Process the NTC measurement
  elapsed = 0;
  while ((reg & PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCNTCRDY) == 0U) {
    if (elapsed >= ADC_POLL_TIMEOUT_MS) {
      return -1;  // Timeout waiting for NTC ADC
    }
    if (!prv_read_register(PmicRegisters_MAIN_EVENTSADCCLR, &reg)) {
      return -1;
    }
    if ((reg & PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCNTCRDY) == 0U) {
      psleep(ADC_POLL_DELAY_MS);
      elapsed += ADC_POLL_DELAY_MS;
    }
  }

  if (!prv_read_register(PmicRegisters_ADC_ADCNTCRESULTMSB, &lsb)) {
    return -1;
  }

  if (!prv_read_register(PmicRegisters_ADC_ADCGP0RESULTLSBS, &msb)) {
    return -1;
  }

  raw = (lsb << NPM1300_ADC_MSB_SHIFT) |
        ((msb >> PmicRegisters_ADC_ADCGP0RESULTLSBS_NTCRESULTLSB_POS) &
         PmicRegisters_ADC_ADCGP0RESULTLSBS_NTCRESULTLSB_MSK);

  // Ref: PS v1.2 Section 7.1.4: Battery temperature (Kelvin)
  float log_result = logf((1024.f / (float)raw) - 1.0f);
  float inv_temp_k = (1.f / 298.15f) - (log_result / (float)NPM1300->cfg->thermistor_beta);

  constants->t_mc = (int32_t)(1000.0f * ((1.f / inv_temp_k) - 273.15f));

  return 0;
}

bool pmic_set_charger_state(bool enable) {
  return prv_write_register(enable ? PmicRegisters_BCHARGER_BCHGENABLESET : PmicRegisters_BCHARGER_BCHGENABLECLR, 1);
}

void battery_set_charge_enable(bool charging_enabled) {
  pmic_set_charger_state(charging_enabled);
}

void battery_set_fast_charge(bool fast_charge_enabled) {
  /* the PMIC handles this for us */
}

bool pmic_is_charging(void) {
  uint8_t status;
  if (!prv_read_register(PmicRegisters_BCHARGER_BCHGCHARGESTATUS, &status)) {
    return false;
  }

  return (status & (PmicRegisters_BCHARGER_BCHGCHARGESTATUS__TRICKLECHARGE | PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTCURRENT | PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTVOLTAGE)) != 0;
}

bool battery_charge_controller_thinks_we_are_charging_impl(void) {
  return pmic_is_charging();
}

bool pmic_is_usb_connected(void) {
  uint8_t status;
  if (!prv_read_register(PmicRegisters_VBUSIN_VBUSINSTATUS, &status)) {
    return false;
  }

  return (status & PmicRegisters_VBUSIN_VBUSINSTATUS__VBUSINPRESENT) != 0;
}

bool battery_is_usb_connected_impl(void) {
  return pmic_is_usb_connected();
}

void pmic_read_chip_info(uint8_t *chip_id, uint8_t *chip_revision, uint8_t *buck1_vset) {
}

bool pmic_enable_battery_measure(void) {
  return true;
}

bool pmic_disable_battery_measure(void) {
  return true;
}

void set_ldo3_power_state(bool enabled) {
}

void set_4V5_power_state(bool enabled) {
}

void set_6V6_power_state(bool enabled) {
}

int battery_charge_status_get(BatteryChargeStatus *status) {
  uint8_t chg_status;

  if (!prv_read_register(PmicRegisters_BCHARGER_BCHGCHARGESTATUS, &chg_status)) {
    return -1;
  }

  switch (chg_status & (PmicRegisters_BCHARGER_BCHGCHARGESTATUS__COMPLETED |
                        PmicRegisters_BCHARGER_BCHGCHARGESTATUS__TRICKLECHARGE |
                        PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTCURRENT |
                        PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTVOLTAGE)) {
    case PmicRegisters_BCHARGER_BCHGCHARGESTATUS__COMPLETED:
      *status = BatteryChargeStatusComplete;
      break;
    case PmicRegisters_BCHARGER_BCHGCHARGESTATUS__TRICKLECHARGE:
      *status = BatteryChargeStatusTrickle;
      break;
    case PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTCURRENT:
      *status = BatteryChargeStatusCC;
      break;
    case PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTVOLTAGE:
      *status = BatteryChargeStatusCV;
      break;
    default:
      *status = BatteryChargeStatusUnknown;
      break;
  }

  return 0;
}

void command_pmic_read_registers(void) {
  char buffer[64];
#define SAY(x) do { uint8_t reg; int rv = prv_read_register(PmicRegisters_##x, &reg); prompt_send_response_fmt(buffer, sizeof(buffer), "PMIC: " #x " = %02x (rv %d)", reg, rv); } while(0)
  SAY(ERRLOG_SCRATCH0);
  SAY(ERRLOG_SCRATCH1);
  SAY(BUCK_BUCK1NORMVOUT);
  SAY(BUCK_BUCK2NORMVOUT);
  SAY(BUCK_BUCKSTATUS);
  SAY(VBUSIN_VBUSINSTATUS);
  SAY(BCHARGER_BCHGCHARGESTATUS);
  SAY(BCHARGER_BCHGERRREASON);
  prompt_send_response_fmt(buffer, sizeof(buffer), "PMIC: Vsys = %d mV", pmic_get_vsys());
  prompt_send_response_fmt(buffer, sizeof(buffer), "PMIC: Vbat = %d mV", battery_get_millivolts());
}

void command_pmic_status(void) {
}

void command_pmic_rails(void) {
  // TODO: Implement.
}

bool pbl_npm1300_set_dischg_limit_ma(const struct pbl_npm1300 *pmic, uint32_t dischg_limit_ma) {
  uint8_t msb, lsb;

  if (pmic->state->dischg_limit_ma == dischg_limit_ma) {
    return true;
  }

  if (dischg_limit_ma == 200) {
    msb = NPM1300_BCHGISETDISCHARGEMSB_200MA;
    lsb = NPM1300_BCHGISETDISCHARGELSB_200MA;
  } else if (dischg_limit_ma == 1000) {
    msb = NPM1300_BCHGISETDISCHARGEMSB_1000MA;
    lsb = NPM1300_BCHGISETDISCHARGELSB_1000MA;
  } else {
    PBL_LOG_ERR("Invalid discharge limit: %" PRIu32 " mA", dischg_limit_ma);
    return false;
  }

  pbl_npm1300_lock(pmic);
  bool ok = pbl_npm1300_write(pmic, PmicRegisters_BCHARGER_BCHGISETDISCHARGEMSB, msb) &&
            pbl_npm1300_write(pmic, PmicRegisters_BCHARGER_BCHGISETDISCHARGELSB, lsb);
  if (ok) {
    pmic->state->dischg_limit_ma = dischg_limit_ma;
  }
  pbl_npm1300_unlock(pmic);
  return ok;
}

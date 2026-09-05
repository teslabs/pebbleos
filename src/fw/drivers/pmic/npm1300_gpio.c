/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/pmic/npm1300.h>

#include <errno.h>

// One register per pin at GPIOS base + offset + pin
#define NPM1300_GPIO_BASE 0x0600
#define NPM1300_GPIO_MODE 0x00
#define NPM1300_GPIO_PULLUP 0x0A
#define NPM1300_GPIO_PULLDOWN 0x0F
#define NPM1300_GPIO_OPENDRAIN 0x14
#define NPM1300_GPIO_STATUS 0x1E
#define NPM1300_GPIO_MODE_INPUT 0
#define NPM1300_GPIO_MODE_OUTPUT_HIGH 8
#define NPM1300_GPIO_MODE_OUTPUT_LOW 9
#define NPM1300_GPIO_NUM_PINS 5

static uint8_t s_pullup_mask;

static const struct pbl_npm1300 *prv_pmic(const struct pbl_gpio_port *port) {
  return PBL_CONTAINER_OF(port, const struct pbl_npm1300, gpio);
}

static bool prv_write(const struct pbl_npm1300 *pmic, uint16_t offset, uint8_t pin, uint8_t value) {
  return pbl_npm1300_write(pmic, NPM1300_GPIO_BASE + offset + pin, value);
}

static int prv_configure(const struct pbl_gpio_port *port, uint8_t pin, uint32_t flags) {
  const struct pbl_npm1300 *pmic = prv_pmic(port);
  uint8_t mode;

  if (pin >= NPM1300_GPIO_NUM_PINS) {
    return -EINVAL;
  }

  if (flags & PBL_GPIO_OUTPUT) {
    if (flags & PBL_GPIO_OUTPUT_INIT_HIGH) {
      mode = NPM1300_GPIO_MODE_OUTPUT_HIGH;
    } else if (flags & PBL_GPIO_OUTPUT_INIT_LOW) {
      mode = NPM1300_GPIO_MODE_OUTPUT_LOW;
    } else {
      // Keep the level if the pin already is an output
      if (!pbl_npm1300_read(pmic, NPM1300_GPIO_BASE + NPM1300_GPIO_MODE + pin, &mode)) {
        return -EIO;
      }
      if (mode != NPM1300_GPIO_MODE_OUTPUT_HIGH) {
        mode = NPM1300_GPIO_MODE_OUTPUT_LOW;
      }
    }
  } else if (flags & PBL_GPIO_INPUT) {
    mode = NPM1300_GPIO_MODE_INPUT;
  } else {
    return -EINVAL;
  }

  bool pullup = (flags & PBL_GPIO_PULL_UP) != 0U;
  if (pullup) {
    s_pullup_mask |= (1U << pin);
  } else {
    s_pullup_mask &= ~(1U << pin);
  }

  pbl_npm1300_lock(pmic);
  bool ok = prv_write(pmic, NPM1300_GPIO_PULLUP, pin, pullup && mode != NPM1300_GPIO_MODE_OUTPUT_LOW);
  ok &= prv_write(pmic, NPM1300_GPIO_PULLDOWN, pin, (flags & PBL_GPIO_PULL_DOWN) != 0U);
  ok &= prv_write(pmic, NPM1300_GPIO_OPENDRAIN, pin, (flags & PBL_GPIO_OPEN_DRAIN) != 0U);
  ok &= prv_write(pmic, NPM1300_GPIO_MODE, pin, mode);
  pbl_npm1300_unlock(pmic);
  return ok ? 0 : -EIO;
}

static int prv_get(const struct pbl_gpio_port *port, uint8_t pin) {
  uint8_t status;

  if (pin >= NPM1300_GPIO_NUM_PINS) {
    return -EINVAL;
  }
  if (!pbl_npm1300_read(prv_pmic(port), NPM1300_GPIO_BASE + NPM1300_GPIO_STATUS, &status)) {
    return -EIO;
  }
  return (status >> pin) & 1U;
}

static int prv_set(const struct pbl_gpio_port *port, uint8_t pin, bool level) {
  const struct pbl_npm1300 *pmic = prv_pmic(port);

  if (pin >= NPM1300_GPIO_NUM_PINS) {
    return -EINVAL;
  }

  pbl_npm1300_lock(pmic);
  bool ok = prv_write(pmic, NPM1300_GPIO_MODE, pin,
                      level ? NPM1300_GPIO_MODE_OUTPUT_HIGH : NPM1300_GPIO_MODE_OUTPUT_LOW);
  // The pull-up only helps while driving high; drop it when low so nothing flows through it
  if (s_pullup_mask & (1U << pin)) {
    ok &= prv_write(pmic, NPM1300_GPIO_PULLUP, pin, level);
  }
  pbl_npm1300_unlock(pmic);
  return ok ? 0 : -EIO;
}

const struct pbl_gpio_port_ops pbl_npm1300_gpio_ops = {
  .configure = prv_configure,
  .get = prv_get,
  .set = prv_set,
};

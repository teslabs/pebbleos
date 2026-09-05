/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/i2c.h>

#include "kernel/util/sleep.h"
#include "pbl/kernel/mutex.h"
#include "pbl/kernel/sem.h"
#include "pbl/kernel/types.h"
#include <pbl/logging/logging.h>
#include "pbl/services/analytics/analytics.h"
#include "system/passert.h"

PBL_LOG_MODULE_DEFINE(driver_i2c, CONFIG_DRIVER_I2C_LOG_LEVEL);

#define I2C_ERROR_TIMEOUT_MS (1000)

// MFI NACKs while busy. We delay ~1ms between retries so this is approximately a 1000ms timeout.
// The longest operation of the MFi chip is "start signature generation", which seems to take
// 223-224 NACKs, but sometimes for unknown reasons it can take much longer.
#define I2C_NACK_COUNT_MAX (1000)

static const char *prv_name(const struct pbl_i2c_bus *bus) {
  return bus->dev.name;
}

//! Caller must hold the bus mutex
static void prv_bus_reset(const struct pbl_i2c_bus *bus) {
  bus->ops->disable(bus);
  bus->ops->enable(bus);
}

int pbl_i2c_bus_init(const struct pbl_device *dev) {
  const struct pbl_i2c_bus *bus = PBL_CONTAINER_OF(dev, const struct pbl_i2c_bus, dev);

  *bus->state = (struct pbl_i2c_bus_state){0};
  pbl_sem_init(&bus->state->event_sem, 1, 1);
  pbl_mutex_init(&bus->state->mutex);

  return bus->ops->init(bus);
}

void pbl_i2c_bus_event(const struct pbl_i2c_bus *bus, enum pbl_i2c_event event) {
  bus->state->event = event;
  pbl_sem_give(&bus->state->event_sem);
}

void pbl_i2c_use(const struct pbl_i2c_dev *dev) {
  PBL_ASSERTN(dev);
  const struct pbl_i2c_bus *bus = dev->bus;
  PBL_ASSERTN(pbl_device_is_ready(&bus->dev));

  pbl_mutex_lock(&bus->state->mutex, PBL_FOREVER);
  if (bus->state->user_count == 0) {
    bus->ops->enable(bus);
  }
  bus->state->user_count++;
  pbl_mutex_unlock(&bus->state->mutex);
}

void pbl_i2c_release(const struct pbl_i2c_dev *dev) {
  PBL_ASSERTN(dev);
  const struct pbl_i2c_bus *bus = dev->bus;

  pbl_mutex_lock(&bus->state->mutex, PBL_FOREVER);
  if (bus->state->user_count == 0) {
    PBL_LOG_ERR("Attempted release of disabled bus %s", prv_name(bus));
    pbl_mutex_unlock(&bus->state->mutex);
    return;
  }
  bus->state->user_count--;
  if (bus->state->user_count == 0) {
    bus->ops->disable(bus);
  }
  pbl_mutex_unlock(&bus->state->mutex);
}

void pbl_i2c_reset(const struct pbl_i2c_dev *dev) {
  PBL_ASSERTN(dev);
  const struct pbl_i2c_bus *bus = dev->bus;

  pbl_mutex_lock(&bus->state->mutex, PBL_FOREVER);
  if (bus->state->user_count == 0) {
    PBL_LOG_ERR("Attempted reset of disabled bus %s", prv_name(bus));
    pbl_mutex_unlock(&bus->state->mutex);
    return;
  }

  PBL_LOG_WRN("Resetting I2C bus %s", prv_name(bus));

  // Drop this user for the reset so a sole user powers the bus down across it
  bus->state->user_count--;
  prv_bus_reset(bus);
  bus->state->user_count++;

  pbl_mutex_unlock(&bus->state->mutex);
}

bool pbl_i2c_bitbang_recovery(const struct pbl_i2c_dev *dev) {
  PBL_LOG_ERR("I2C bitbang recovery not supported on this platform");
  return false;
}

//! Wait a short amount of time for the busy flag to clear
static bool prv_wait_for_not_busy(const struct pbl_i2c_bus *bus) {
  static const int WAIT_DELAY = 10;  // milliseconds

  if (bus->ops->is_busy(bus)) {
    psleep(WAIT_DELAY);
    if (bus->ops->is_busy(bus)) {
      PBL_LOG_ERR("Timed out waiting for bus %s to become non-busy", prv_name(bus));
      return false;
    }
  }
  return true;
}

//! Caller must hold the bus mutex
static bool prv_do_transfer_locked(const struct pbl_i2c_bus *bus,
                                   const struct pbl_i2c_transfer *transfer) {
  struct pbl_i2c_bus_state *state = bus->state;

  if (state->user_count == 0) {
    PBL_LOG_ERR("Attempted access to disabled bus %s", prv_name(bus));
    return false;
  }

  // The bus is left idle after every transfer; a busy bus here needs a reset
  if (bus->ops->is_busy(bus)) {
    prv_bus_reset(bus);
    if (!prv_wait_for_not_busy(bus)) {
      PBL_LOG_ERR("I2C bus did not recover after reset (%s)", prv_name(bus));
      return false;
    }
  }

  // Take the token so the wait below blocks until the driver gives it back
  PBL_ASSERT(pbl_sem_take(&state->event_sem, PBL_NO_WAIT) == 0, "Could not acquire semaphore token");

  state->transfer = *transfer;
  state->nack_count = 0;
  if (bus->ops->begin_transfer) {
    bus->ops->begin_transfer(bus);
  }

  bool result = false;
  while (true) {
    bus->ops->start_transfer(bus);

    if (pbl_sem_take(&state->event_sem, PBL_MSEC(I2C_ERROR_TIMEOUT_MS)) != 0) {
      bus->ops->abort_transfer(bus);
      PBL_LOG_ERR("Transfer timed out on bus %s", prv_name(bus));
      PBL_ANALYTICS_ADD(i2c_transfer_error_count, 1);
      break;
    }

    if (state->event == PBL_I2C_EVENT_COMPLETE) {
      result = true;
      break;
    } else if (state->event == PBL_I2C_EVENT_ERROR) {
      PBL_LOG_ERR("I2C Error on bus %s", prv_name(bus));
      PBL_ANALYTICS_ADD(i2c_transfer_error_count, 1);
      break;
    } else if (state->nack_count >= I2C_NACK_COUNT_MAX) {
      bus->ops->abort_transfer(bus);
      PBL_LOG_ERR("I2C Error: too many NACKs received on bus %s", prv_name(bus));
      break;
    }

    // NACKed start: the MFi chip does that while busy. Retry after a short delay.
    state->nack_count++;
    psleep(2);
  }

  pbl_sem_give(&state->event_sem);

  // A transfer can complete with the busy flag stuck, which would fail the next one
  if (!prv_wait_for_not_busy(bus)) {
    prv_bus_reset(bus);
  }

  return result;
}

static bool prv_do_transfer(const struct pbl_i2c_dev *dev, enum pbl_i2c_dir dir, bool with_reg,
                            uint8_t reg, uint32_t size, uint8_t *data) {
  const struct pbl_i2c_transfer transfer = {
    .addr = dev->addr,
    .dir = dir,
    .with_reg = with_reg,
    .reg = reg,
    .size = size,
    .data = data,
  };

  pbl_mutex_lock(&dev->bus->state->mutex, PBL_FOREVER);
  bool result = prv_do_transfer_locked(dev->bus, &transfer);
  pbl_mutex_unlock(&dev->bus->state->mutex);

  return result;
}

bool pbl_i2c_read_register(const struct pbl_i2c_dev *dev, uint8_t reg, uint8_t *result) {
  return pbl_i2c_read_register_block(dev, reg, 1, result);
}

bool pbl_i2c_read_register_block(const struct pbl_i2c_dev *dev, uint8_t reg, uint32_t size,
                                 uint8_t *result) {
  PBL_ASSERTN(dev);
  PBL_ASSERTN(result);

  bool ok = prv_do_transfer(dev, PBL_I2C_READ, true, reg, size, result);
  if (!ok) {
    PBL_LOG_ERR("Read failed on bus %s", prv_name(dev->bus));
  }
  return ok;
}

bool pbl_i2c_read_block(const struct pbl_i2c_dev *dev, uint32_t size, uint8_t *result) {
  PBL_ASSERTN(dev);
  PBL_ASSERTN(result);

  bool ok = prv_do_transfer(dev, PBL_I2C_READ, false, 0, size, result);
  if (!ok) {
    PBL_LOG_ERR("Block read failed on bus %s", prv_name(dev->bus));
  }
  return ok;
}

bool pbl_i2c_write_register(const struct pbl_i2c_dev *dev, uint8_t reg, uint8_t value) {
  return pbl_i2c_write_register_block(dev, reg, 1, &value);
}

bool pbl_i2c_write_register_block(const struct pbl_i2c_dev *dev, uint8_t reg, uint32_t size,
                                  const uint8_t *data) {
  PBL_ASSERTN(dev);
  PBL_ASSERTN(data);

  bool ok = prv_do_transfer(dev, PBL_I2C_WRITE, true, reg, size, (uint8_t *)data);
  if (!ok) {
    PBL_LOG_ERR("Write failed on bus %s", prv_name(dev->bus));
  }
  return ok;
}

bool pbl_i2c_write_block(const struct pbl_i2c_dev *dev, uint32_t size, const uint8_t *data) {
  PBL_ASSERTN(dev);
  PBL_ASSERTN(data);

  bool ok = prv_do_transfer(dev, PBL_I2C_WRITE, false, 0, size, (uint8_t *)data);
  if (!ok) {
    PBL_LOG_ERR("Block write failed on bus %s", prv_name(dev->bus));
  }
  return ok;
}

bool pbl_i2c_write_read_block(const struct pbl_i2c_dev *dev, uint32_t write_size,
                              const uint8_t *write_data, uint32_t read_size, uint8_t *read_data) {
  PBL_ASSERTN(dev);
  PBL_ASSERTN(write_data);
  PBL_ASSERTN(read_data);

  const struct pbl_i2c_bus *bus = dev->bus;
  const struct pbl_i2c_transfer write = {
    .addr = dev->addr,
    .dir = PBL_I2C_WRITE,
    .size = write_size,
    .data = (uint8_t *)write_data,
  };
  const struct pbl_i2c_transfer read = {
    .addr = dev->addr,
    .dir = PBL_I2C_READ,
    .size = read_size,
    .data = read_data,
  };

  pbl_mutex_lock(&bus->state->mutex, PBL_FOREVER);
  bool ok = prv_do_transfer_locked(bus, &write) && prv_do_transfer_locked(bus, &read);
  pbl_mutex_unlock(&bus->state->mutex);

  if (!ok) {
    PBL_LOG_ERR("Write-read block failed on bus %s", prv_name(bus));
  }
  return ok;
}

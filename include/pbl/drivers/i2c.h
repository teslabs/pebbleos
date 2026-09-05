/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/device.h>
#include "pbl/kernel/mutex.h"
#include "pbl/kernel/sem.h"

#include <stdbool.h>
#include <stdint.h>

enum pbl_i2c_event {
  PBL_I2C_EVENT_TIMEOUT,
  PBL_I2C_EVENT_COMPLETE,
  PBL_I2C_EVENT_NACK,
  PBL_I2C_EVENT_ERROR,
};

enum pbl_i2c_dir {
  PBL_I2C_READ,
  PBL_I2C_WRITE,
};

struct pbl_i2c_transfer {
  uint16_t addr;
  enum pbl_i2c_dir dir;
  //! Send @c reg first; a read then continues with a repeated start.
  bool with_reg;
  uint8_t reg;
  uint32_t size;
  uint8_t *data;
};

struct pbl_i2c_bus_state {
  struct pbl_i2c_transfer transfer;
  enum pbl_i2c_event event;
  int nack_count;
  int user_count;
  struct pbl_sem event_sem;
  struct pbl_mutex mutex;
};

struct pbl_i2c_bus;

//! Bus driver interface. The class layer owns locking, retries and timeouts;
//! the driver moves bus->state->transfer over the wire and reports how it
//! ended through pbl_i2c_bus_event().
struct pbl_i2c_bus_ops {
  int (*init)(const struct pbl_i2c_bus *bus);
  void (*enable)(const struct pbl_i2c_bus *bus);
  void (*disable)(const struct pbl_i2c_bus *bus);
  bool (*is_busy)(const struct pbl_i2c_bus *bus);
  //! Optional. Once per transfer, before the first start_transfer().
  void (*begin_transfer)(const struct pbl_i2c_bus *bus);
  //! Starts the transfer; called again to retry after a NACK.
  void (*start_transfer)(const struct pbl_i2c_bus *bus);
  void (*abort_transfer)(const struct pbl_i2c_bus *bus);
};

struct pbl_i2c_bus {
  struct pbl_device dev;
  const struct pbl_i2c_bus_ops *ops;
  struct pbl_i2c_bus_state *state;
};

//! A peripheral on a bus, at its 7-bit address.
struct pbl_i2c_dev {
  const struct pbl_i2c_bus *bus;
  uint16_t addr;
};

#define PBL_I2C_DEV(_bus, _addr) { .bus = (_bus), .addr = (_addr) }

//! Building blocks for the PBL_I2C_*_DEFINE macro of a bus driver.
#define PBL_I2C_BUS_STATE_DEFINE(sym) \
  PBL_DEVICE_STATE_DEFINE(sym);       \
  static struct pbl_i2c_bus_state sym##_i2c_state

#define PBL_I2C_BUS_INIT(sym, _name, _ops, _deps)                       \
  {                                                                     \
    .dev = PBL_DEVICE_INIT(sym, _name, pbl_i2c_bus_init, NULL, _deps),  \
    .ops = (_ops),                                                      \
    .state = &sym##_i2c_state,                                          \
  }

//! The struct pbl_device init of every bus: sets the state up, then ops->init.
int pbl_i2c_bus_init(const struct pbl_device *dev);

//! Reports the end of the current transfer. Called by the bus driver, from IRQ.
void pbl_i2c_bus_event(const struct pbl_i2c_bus *bus, enum pbl_i2c_event event);

//! Claims the bus @p dev is on; powers it up on the first user. Must precede
//! any transfer to @p dev, and be balanced with pbl_i2c_release().
void pbl_i2c_use(const struct pbl_i2c_dev *dev);
void pbl_i2c_release(const struct pbl_i2c_dev *dev);

//! Power-cycles and re-initialises the bus @p dev is on.
void pbl_i2c_reset(const struct pbl_i2c_dev *dev);

//! Clocks the bus by hand until the data line recovers.
//! @return true if it did.
bool pbl_i2c_bitbang_recovery(const struct pbl_i2c_dev *dev);

//! Register access: the register address is sent first, with a repeated
//! start before a read. All return true on success.
bool pbl_i2c_read_register(const struct pbl_i2c_dev *dev, uint8_t reg, uint8_t *result);
bool pbl_i2c_read_register_block(const struct pbl_i2c_dev *dev, uint8_t reg, uint32_t size,
                                 uint8_t *result);
bool pbl_i2c_write_register(const struct pbl_i2c_dev *dev, uint8_t reg, uint8_t value);
bool pbl_i2c_write_register_block(const struct pbl_i2c_dev *dev, uint8_t reg, uint32_t size,
                                  const uint8_t *data);

//! Raw access, no register address.
bool pbl_i2c_read_block(const struct pbl_i2c_dev *dev, uint32_t size, uint8_t *result);
bool pbl_i2c_write_block(const struct pbl_i2c_dev *dev, uint32_t size, const uint8_t *data);

//! A write followed by a read, with the bus held across both.
bool pbl_i2c_write_read_block(const struct pbl_i2c_dev *dev, uint32_t write_size,
                              const uint8_t *write_data, uint32_t read_size, uint8_t *read_data);

/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/util/struct.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//! A hardware device: a controller, a bus or a peripheral behind one.
//!
//! Drivers embed this struct in their own device type (e.g. struct
//! pbl_gpio_port embeds it, struct pbl_gpio_nrf5 embeds that) and recover the
//! outer struct with PBL_CONTAINER_OF. The device itself lives in ROM; the
//! runtime state it points to is the only RAM it needs.
//!
//! Boards and SoCs define device instances with the PBL_*_DEFINE macro of the
//! matching driver, which registers them in the device table.
//! pbl_device_init_all() then initialises the whole table, dependencies
//! first: a device's parent (the bus or multi-function device it hangs off)
//! goes before it, a driver's init calls pbl_device_init() on the other
//! devices it uses, and a board adds anything else through the deps list.

enum pbl_device_status {
  PBL_DEVICE_UNINIT,
  PBL_DEVICE_INITIALIZING,
  PBL_DEVICE_READY,
  PBL_DEVICE_FAILED,
};

struct pbl_device_state {
  uint8_t status;
  int res;
};

struct pbl_device {
  const char *name;
  //! Optional. Returns 0 or a negative errno.
  int (*init)(const struct pbl_device *dev);
  //! Optional. Initialised before this device, unless it is the one
  //! initialising it: a parent may bring its children up from its own init.
  const struct pbl_device *parent;
  //! Optional, NULL-terminated. Initialised before this device.
  const struct pbl_device *const *deps;
  struct pbl_device_state *state;
};

//! Static dependency list for struct pbl_device.deps.
#define PBL_DEVICE_DEPS(...) ((const struct pbl_device *const[]){__VA_ARGS__, NULL})

//! Building blocks for the PBL_*_DEFINE macro of a driver. @p sym is the
//! symbol of the device instance being defined.
#define PBL_DEVICE_STATE_DEFINE(sym) static struct pbl_device_state sym##_device_state

#define PBL_DEVICE_INIT(sym, _name, _init, _parent, _deps)                       \
  {                                                                             \
    .name = (_name), .init = (_init), .parent = (_parent), .deps = (_deps),      \
    .state = &sym##_device_state,                                               \
  }

#ifdef UNITTEST
#define PBL_DEVICE_TABLE_SECTION
#else
#define PBL_DEVICE_TABLE_SECTION __attribute__((section(".pbl_devices")))
#endif

//! Adds @p dev, a pointer to the struct pbl_device of @p sym, to the device
//! table that pbl_device_init_all() walks.
#define PBL_DEVICE_REGISTER(sym, dev) \
  static const struct pbl_device *const sym##_device_entry \
      __attribute__((used)) PBL_DEVICE_TABLE_SECTION = (dev)

//! Initialises @p dev, its parent and deps first. A no-op once the device is
//! ready or has failed. A dependency cycle is a programming error and asserts.
//! @return 0, the error the init returned, or -ENODEV when a parent or dep
//! failed.
int pbl_device_init(const struct pbl_device *dev);

bool pbl_device_is_ready(const struct pbl_device *dev);

//! Initialises every device in @p devs. @return the number of failures.
int pbl_device_init_table(const struct pbl_device *const *devs, size_t count);

//! Initialises every registered device. @return the number of failures.
int pbl_device_init_all(void);

//! Initialises every registered device whose parent is @p parent. For a
//! multi-function device to bring its functions up from its own init.
//! @return the number of failures.
int pbl_device_init_children(const struct pbl_device *parent);

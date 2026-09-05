/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/device.h>
#include <pbl/drivers/i2c.h>

struct pbl_opt3001 {
  struct pbl_device dev;
  struct pbl_i2c_dev i2c;
};

//! The board's ambient light sensor
extern const struct pbl_opt3001 *const OPT3001;

int opt3001_init(const struct pbl_device *dev);

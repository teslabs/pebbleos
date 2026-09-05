/* SPDX-FileCopyrightText: 2025 Matthew Wardrop */
/* SPDX-FileCopyrightText: 2025 Bob Wei */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/device.h>
#include <pbl/drivers/i2c.h>

#include "board/board.h"

struct pbl_mmc5603nj {
  struct pbl_device dev;
  struct pbl_i2c_dev i2c;
  MagConfig mag_config;
};

//! The board's magnetometer
extern const struct pbl_mmc5603nj *const MMC5603NJ;

int mmc5603nj_init(const struct pbl_device *dev);

void mag_set_rotated(bool rotated);

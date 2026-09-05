/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <pbl/drivers/i2c.h>

#include "board/board.h"

typedef struct {
  struct pbl_device dev;
  const struct pbl_i2c_dev *i2c;
  const struct pbl_i2c_dev *i2c_boot;
  ExtiConfig int_exti;
  struct pbl_gpio reset;
  uint16_t max_x;
  uint16_t max_y;
  bool invert_x_axis;
  bool invert_y_axis;
} TouchSensor;

int cst816_init(const struct pbl_device *dev);

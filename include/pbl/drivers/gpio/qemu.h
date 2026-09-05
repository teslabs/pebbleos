/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/drivers/gpio.h>

struct pbl_gpio_qemu {
  struct pbl_gpio_port port;
  uintptr_t base;
};

extern const struct pbl_gpio_qemu pbl_gpio_qemu_gpio;

#define QEMU_GPIO (&pbl_gpio_qemu_gpio.port)

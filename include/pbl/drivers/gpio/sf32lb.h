/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/drivers/gpio.h>

#include <bf0_hal.h>

struct pbl_gpio_sf32lb {
  struct pbl_gpio_port port;
  GPIO_TypeDef *regs;
  int pad_base;
  pin_function func_base;
  RCC_MODULE_TYPE module;
};

extern const struct pbl_gpio_sf32lb pbl_gpio_sf32lb_gpio1;

#define SF32LB_GPIO1 (&pbl_gpio_sf32lb_gpio1.port)

//! Register block of the port @p gpio is on, for the HAL calls the EXTI and
//! button drivers still make directly.
static inline GPIO_TypeDef *pbl_gpio_sf32lb_regs(const struct pbl_gpio *gpio) {
  return PBL_CONTAINER_OF(gpio->port, const struct pbl_gpio_sf32lb, port)->regs;
}

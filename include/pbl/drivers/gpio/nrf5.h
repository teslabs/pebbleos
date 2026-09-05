/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/drivers/gpio.h>

#include <hal/nrf_gpio.h>

struct pbl_gpio_nrf5 {
  struct pbl_gpio_port port;
  uint8_t index;
};

extern const struct pbl_gpio_nrf5 pbl_gpio_nrf5_p0;
extern const struct pbl_gpio_nrf5 pbl_gpio_nrf5_p1;

#define NRF_GPIO_P0 (&pbl_gpio_nrf5_p0.port)
#define NRF_GPIO_P1 (&pbl_gpio_nrf5_p1.port)

//! Absolute pin number of @p gpio, as the nrfx driver configs take it.
static inline uint32_t pbl_gpio_nrf5_pin(const struct pbl_gpio *gpio) {
  const struct pbl_gpio_nrf5 *port =
      PBL_CONTAINER_OF(gpio->port, const struct pbl_gpio_nrf5, port);
  return NRF_GPIO_PIN_MAP(port->index, gpio->pin);
}

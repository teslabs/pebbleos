/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <pbl/device.h>

#include <stdbool.h>
#include <stdint.h>

//! Pin flags. The wiring flags describe the pin as routed on the board and
//! are stored in struct pbl_gpio; the rest describe the configuration
//! requested from pbl_gpio_configure() and are OR'ed with them.

// Wiring
#define PBL_GPIO_ACTIVE_LOW (1U << 0)
#define PBL_GPIO_PULL_UP (1U << 1)
#define PBL_GPIO_PULL_DOWN (1U << 2)
#define PBL_GPIO_OPEN_DRAIN (1U << 3)

// Direction and initial level (physical)
#define PBL_GPIO_INPUT (1U << 4)
#define PBL_GPIO_OUTPUT (1U << 5)
#define PBL_GPIO_OUTPUT_INIT_LOW (1U << 6)
#define PBL_GPIO_OUTPUT_INIT_HIGH (1U << 7)
// Initial level is logical: inverted on an active-low pin
#define PBL_GPIO_OUTPUT_INIT_LOGICAL (1U << 8)

#define PBL_GPIO_OUTPUT_LOW (PBL_GPIO_OUTPUT | PBL_GPIO_OUTPUT_INIT_LOW)
#define PBL_GPIO_OUTPUT_HIGH (PBL_GPIO_OUTPUT | PBL_GPIO_OUTPUT_INIT_HIGH)
#define PBL_GPIO_OUTPUT_INACTIVE (PBL_GPIO_OUTPUT_LOW | PBL_GPIO_OUTPUT_INIT_LOGICAL)
#define PBL_GPIO_OUTPUT_ACTIVE (PBL_GPIO_OUTPUT_HIGH | PBL_GPIO_OUTPUT_INIT_LOGICAL)

struct pbl_gpio_port;

//! Port driver interface. Levels and flags are physical: the class layer
//! has already applied PBL_GPIO_ACTIVE_LOW.
struct pbl_gpio_port_ops {
  int (*configure)(const struct pbl_gpio_port *port, uint8_t pin, uint32_t flags);
  //! @return 0 or 1, or a negative errno.
  int (*get)(const struct pbl_gpio_port *port, uint8_t pin);
  int (*set)(const struct pbl_gpio_port *port, uint8_t pin, bool level);
};

//! A GPIO controller: an SoC port, or any peripheral with GPIOs on it.
struct pbl_gpio_port {
  struct pbl_device dev;
  const struct pbl_gpio_port_ops *ops;
};

//! A pin on a port, as wired on the board. A NULL port means not connected.
struct pbl_gpio {
  const struct pbl_gpio_port *port;
  uint8_t pin;
  uint8_t flags;
};

#define PBL_GPIO(_port, _pin, _flags) { .port = (_port), .pin = (_pin), .flags = (_flags) }

static inline bool pbl_gpio_is_valid(const struct pbl_gpio *gpio) {
  return gpio->port != NULL;
}

//! Configures the pin with @p flags OR'ed with its wiring flags. The port
//! must be ready.
int pbl_gpio_configure(const struct pbl_gpio *gpio, uint32_t flags);

//! Logical level: 1 when the pin is active.
int pbl_gpio_get(const struct pbl_gpio *gpio);
int pbl_gpio_set(const struct pbl_gpio *gpio, bool active);

//! Physical level.
int pbl_gpio_get_raw(const struct pbl_gpio *gpio);
int pbl_gpio_set_raw(const struct pbl_gpio *gpio, bool level);

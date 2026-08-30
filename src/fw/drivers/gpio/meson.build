# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = []
if cfg.get('CONFIG_GPIO_QEMU', false)
  sources += driver_shared_srcs['qemu/qemu_gpio_hal.c']
elif cfg.get('CONFIG_GPIO_NRF5', false)
  sources += files('nrf5.c')
elif cfg.get('CONFIG_GPIO_SF32LB', false)
  sources += files('sf32lb.c')
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': sources}]

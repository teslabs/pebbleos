# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = []
if cfg.get('CONFIG_PWM_QEMU', false)
  sources += driver_shared_srcs['qemu/qemu_pwm_hal.c']
elif cfg.get('CONFIG_PWM_NRF5', false)
  sources += files('nrf5.c')
elif cfg.get('CONFIG_PWM_SF32LB', false)
  sources += files('sf32lb.c')
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': sources}]

# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = []
if cfg.get('CONFIG_EXTI_QEMU', false)
  sources += driver_shared_srcs['stubs/exti.c']
elif cfg.get('CONFIG_EXTI_NRF5', false)
  sources += files('nrf5.c')
elif cfg.get('CONFIG_EXTI_SF32LB', false)
  sources += files('sf32lb.c')
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': sources}]

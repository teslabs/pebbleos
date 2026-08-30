# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = []
if cfg.get('CONFIG_RTC_QEMU', false)
  sources += driver_shared_srcs['qemu/qemu_rtc_hal.c']
elif cfg.get('CONFIG_RTC_NRF5', false)
  sources += files('nrf5.c')
elif cfg.get('CONFIG_RTC_SF32LB', false)
  sources += files('sf32lb.c')
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': sources}]

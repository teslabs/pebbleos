# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = []
if cfg.get('CONFIG_DISPLAY_QEMU', false)
  sources += driver_shared_srcs['qemu/qemu_display_hal.c']
elif cfg.get('CONFIG_DISPLAY_JDI_SF32LB', false)
  sources += files('sf32lb/display_jdi.c')
elif cfg.get('CONFIG_DISPLAY_SHARP_LS013B7DH01_NRF5', false)
  sources += files('sharp_ls013b7dh01/sharp_ls013b7dh01_nrf5.c')
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': sources}]

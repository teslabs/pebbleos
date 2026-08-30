# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = []
if cfg.get('CONFIG_MIC_NRF5', false)
  sources += ['nrf5/pdm.c']
elif cfg.get('CONFIG_MIC_SF32LB', false)
  sources += ['sf32lb52/pdm.c']
elif cfg.get('CONFIG_MIC_QEMU', false)
  sources += ['qemu/mic.c']
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

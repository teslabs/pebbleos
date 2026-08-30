# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = ['common.c']
if cfg.get('CONFIG_I2C_NRF5', false)
  sources += ['nrf5.c']
elif cfg.get('CONFIG_I2C_SF32LB', false)
  sources += ['sf32lb.c']
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

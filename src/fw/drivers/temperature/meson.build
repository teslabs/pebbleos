# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = []
if cfg.get('CONFIG_TEMPERATURE_SF32LB', false)
  sources += ['sf32lb.c']
elif cfg.get('CONFIG_TEMPERATURE_STUB', false)
  sources += ['stubs.c']
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

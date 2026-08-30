# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = []
if cfg.get('CONFIG_PMIC_NPM1300', false)
  sources += ['npm1300.c']
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

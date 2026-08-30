# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = []
if cfg.get('CONFIG_AMBIENT_LIGHT_OPT3001', false)
  sources += ['ambient_light_opt3001.c']
elif cfg.get('CONFIG_AMBIENT_LIGHT_W1160', false)
  sources += ['ambient_light_w1160.c']
endif
if sources.length() > 0
  sources += ['ambient_light_common.c']
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

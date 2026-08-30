# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = []
if cfg.get('CONFIG_HRM_GH3X2X', false)
  sources += ['gh3x2x.c']
elif cfg.get('CONFIG_HRM_STUB', false)
  sources += ['stub.c']
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

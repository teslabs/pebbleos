# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = ['analytics.c']
if not cfg.get('CONFIG_RECOVERY_FW', false)
  sources += ['native.c']
  pbl_global_defs += ['ANALYTICS_NATIVE']
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

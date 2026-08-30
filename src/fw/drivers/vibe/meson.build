# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = []
if cfg.get('CONFIG_VIBE_AW86225', false)
  sources += ['vibe_aw86225.c']
elif cfg.get('CONFIG_VIBE_AW8623X', false)
  sources += ['vibe_aw8623x.c']
elif cfg.get('CONFIG_VIBE_DRV2604', false)
  sources += ['vibe_drv2604.c']
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

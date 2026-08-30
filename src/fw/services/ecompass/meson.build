# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if cfg.get('CONFIG_MAG', false)
  pbl_libs += [
    {
      'dir': meson.current_source_dir(),
      'srcs': files('service.c', 'correction.c'),
    },
  ]
endif

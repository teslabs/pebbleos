# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if cfg.get('CONFIG_LIBC_NEWLIB_NANO', false)
  subdir('newlib')
endif
pbl_libs += [
  {
    'dir': meson.current_source_dir(),
    'srcs': files('assert.c', 'syscalls.c'),
  },
]

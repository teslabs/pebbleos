# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_linker_ram_sections += 'default|' + meson.current_source_dir() / 'ramfunc.ld'
pbl_libs += [
  {
    'dir': meson.current_source_dir(),
    'srcs': files('init.c', 'sleep.c', 'freertos.c'),
  },
]

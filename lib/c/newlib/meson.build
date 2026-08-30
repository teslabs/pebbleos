# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_libs += [
  {
    'dir': meson.current_source_dir(),
    'srcs': files('vsnprintf.c'),
    'args': ['-fno-builtin', '-fno-lto', '-Wno-error=char-subscripts'],
  },
]

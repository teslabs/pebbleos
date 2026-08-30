# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

tinymt_sources = 'TinyMT/tinymt/tinymt32.c'
tinymt_includes = 'TinyMT/tinymt'
pbl_global_inc += meson.current_source_dir() / tinymt_includes
pbl_libs += [
  {
    'name': 'tinymt32',
    'srcs': files([tinymt_sources]),
    'kind': 'stlib',
  },
]

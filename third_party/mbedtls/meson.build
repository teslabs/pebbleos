# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

foreach dir : ['mbedtls/include', 'include']
  pbl_global_inc += meson.current_source_dir() / dir
endforeach
pbl_global_defs += ['MBEDTLS_CONFIG_FILE="pebble_mbedtls_config.h"']

sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), 'mbedtls/library/*.c',
  check: true,
).stdout().strip().split('\n')

pbl_libs += [
  {
    'name': 'mbedtls',
    'srcs': files(sources),
    'kind': 'stlib',
    'inc': ['mbedtls/library'],
    'args': ['-Wno-unused-but-set-variable'],
  },
]

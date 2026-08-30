# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '--recurse', '*.c',
  check: true,
).stdout().strip().split('\n')

pbl_libs += [
  {
    'name': 'libbtutil',
    'srcs': files(sources),
    'kind': 'stlib',
    'inc': ['.'],
  },
]

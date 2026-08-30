# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if cfg.get('CONFIG_APPS_WATCH_TICTOC_ROUND', false)
  tictoc_variant = 'round'
elif cfg.get('CONFIG_APPS_WATCH_TICTOC_BW', false)
  tictoc_variant = 'bw'
else
  tictoc_variant = 'default'
endif

variant_sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), tictoc_variant + '/*.c',
  check: true,
).stdout().strip().split('\n')

pbl_libs += [
  {
    'dir': meson.current_source_dir(),
    'srcs': files(['tictoc.c'] + variant_sources),
    'inc': ['.'],
  },
]

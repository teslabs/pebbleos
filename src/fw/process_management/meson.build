# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

excl = []
if cfg.get('CONFIG_RECOVERY_FW', false)
  excl += ['app_custom_icon.c', 'app_menu_data_source.c']
endif
sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '--exclude-list', ';'.join(excl), '*.c',
  check: true,
).stdout().strip().split('\n')
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

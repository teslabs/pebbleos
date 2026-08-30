# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if cfg.get('CONFIG_SHELL_SDK', false)
  source = []
  _glob0 = run_command(
    py, glob_py, '--base', meson.current_source_dir(), 'timeline/*.c',
    check: true,
  ).stdout().strip().split('\n')
  source += _glob0
  _glob1 = run_command(
    py, glob_py, '--base', meson.current_source_dir(), '--recurse', 'launcher/default/*.c',
    check: true,
  ).stdout().strip().split('\n')
  source += _glob1
  source += ['app_fetch_ui.c', 'watchfaces.c']
else
  source = run_command(
    py, glob_py, '--base', meson.current_source_dir(), '--recurse', '*.c',
    check: true,
  ).stdout().strip().split('\n')
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(source)}]

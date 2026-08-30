# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

excludes = []
if not cfg.get('CONFIG_GH3X2X_TUNING_SERVICE_ENABLED', false)
  excludes += ['gh3x2x_tuning_service.c']
endif
driver_source = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '--exclude-list', ';'.join(excludes), '*.c',
  check: true,
).stdout().strip().split('\n')
pbl_libs += [{'name': 'bt_driver', 'srcs': files(driver_source)}]

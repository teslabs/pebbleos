# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if cfg.get('CONFIG_RECOVERY_FW', false)
  source = ['bluetooth_pairing_ui.c']
else
  if cfg.get('CONFIG_HRM', false)
    excl = []
  else
    excl = ['ble_hrm/**']
  endif
  source = run_command(
    py, glob_py, '--base', meson.current_source_dir(), '--recurse', '--exclude-list', ';'.join(excl), '*.c',
    check: true,
  ).stdout().strip().split('\n')
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(source)}]

# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

excl = []
if not cfg.get('CONFIG_BT_ANCS_CLIENT', false)
  excl += ['ble/kernel_le_client/ancs/*.c']
endif
if not cfg.get('CONFIG_BT_AMS_CLIENT', false)
  excl += ['ble/kernel_le_client/ams/*.c']
endif
sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '--recurse', '--exclude-list', ';'.join(excl), '*.c',
  check: true,
).stdout().strip().split('\n')
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

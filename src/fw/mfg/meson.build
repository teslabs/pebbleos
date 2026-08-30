# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

# The active board's manufacturing sources.
if cfg.get('CONFIG_QEMU', false)
  mfg_board = 'qemu'
elif cfg.get('CONFIG_BOARD_ASTERIX', false)
  mfg_board = 'asterix'
elif cfg.get('CONFIG_BOARD_OBELIX', false)
  mfg_board = 'obelix'
elif cfg.get('CONFIG_BOARD_GETAFIX', false)
  mfg_board = 'getafix'
else
  error('No MFG configuration for board ' + pbl_board)
endif

sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '*.c', mfg_board + '/*.c',
  check: true,
).stdout().strip().split('\n')
# Factory mode only goes into the recovery firmware.
if cfg.get('CONFIG_RECOVERY_FW', false)
  mfg_mode_sources = run_command(
    py, glob_py, '--base', meson.current_source_dir(), 'mfg_mode/*.c',
    check: true,
  ).stdout().strip().split('\n')
  sources += mfg_mode_sources
endif

pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

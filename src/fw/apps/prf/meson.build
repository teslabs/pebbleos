# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

excl = []
if not cfg.get('CONFIG_BOARD_ASTERIX', false)
  excl += ['mfg_speaker_asterix.c', 'mfg_mic_asterix.c']
endif
if not cfg.get('CONFIG_BOARD_OBELIX', false)
  excl += ['mfg_speaker_obelix.c', 'mfg_mic_obelix.c']
endif
if not (cfg.get('CONFIG_BOARD_OBELIX', false) and cfg.get('CONFIG_MFG', false))
  excl += ['mfg_hrm_ctr_leakage_obelix.c']
endif
if not cfg.get('CONFIG_BOARD_GETAFIX', false)
  excl += ['mfg_mic_getafix.c']
endif
if not cfg.get('CONFIG_MAG', false)
  excl += ['mfg_mag.c']
endif
if not cfg.get('CONFIG_TOUCH', false)
  excl += ['mfg_touch.c']
endif
sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '--recurse', '--exclude-list', ';'.join(excl), '*.c',
  check: true,
).stdout().strip().split('\n')
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

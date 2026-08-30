# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

excl = []
if not cfg.get('CONFIG_PROMPT', false)
  excl += ['prompt.c', 'prompt_commands.c']
endif
sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '--exclude-list', ';'.join(excl), '*.c',
  check: true,
).stdout().strip().split('\n')
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

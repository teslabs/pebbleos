# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if cfg.get('CONFIG_RECOVERY_FW', false)
  shell = 'prf'
elif cfg.get('CONFIG_SHELL_SDK', false)
  shell = 'sdk'
else
  shell = 'normal'
endif

# The launcher's app registry, and the app id defines the sources use.
app_registry = custom_target(
  'system_app_registry',
  output: ['system_app_registry_list.auto.h', 'system_app_ids.auto.h'],
  command: [
    py, generate_py, 'app-registry',
    '--input', meson.current_source_dir() / shell / 'system_app_registry_list.json',
    '--registry', '@OUTPUT0@',
    '--enum', '@OUTPUT1@',
    '--config', pbl_dotconfig,
    '--board', pbl_board_name,
  ],
  depend_files: [
    meson.current_source_dir() / shell / 'system_app_registry_list.json',
    pbl_dotconfig,
  ],
)
pbl_generated += app_registry

sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '*.c',
  check: true,
).stdout().strip().split('\n')
shell_sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '--recurse', shell + '/*.c',
  check: true,
).stdout().strip().split('\n')

pbl_libs += [
  {
    'dir': meson.current_source_dir(),
    'srcs': files(sources) + files(shell_sources),
  },
]

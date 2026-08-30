# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

subdir('demo')
subdir('system')
if cfg.get('CONFIG_SHELL_SDK', false)
  subdir('sdk')
else
  subdir('watch')
endif
# Every app's translatable strings, whether or not this configuration
# builds the app. The recovery-only apps ship in their own firmware
# variant, so they are left out.
pot_sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '--recurse', '--exclude-list', ';'.join(['prf/**']), '*.c',
  check: true,
).stdout().strip().split('\n')
apps_pot = custom_target(
  'apps.pot',
  output: 'apps.pot',
  input: files(pot_sources),
  command: pbl_xgettext,
)

# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# Stored apps have no resources of their own, but the SDK headers include
# this file unconditionally. It has to sit in the app's src/ build
# directory, and configure_file cannot name a subdirectory, so it is
# written from here.

configure_file(
  output: 'resource_ids.auto.h',
  command: [py, '-c', 'print("#define DEFAULT_MENU_ICON  0")'],
  capture: true,
)

app_sources = files(
  run_command(
    py, glob_py, '--base', meson.current_source_dir(), '--recurse', '*.c',
    check: true,
  ).stdout().strip().split('\n'),
)

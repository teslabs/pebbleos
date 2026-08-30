# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

excl = []
if cfg.get('CONFIG_RECOVERY_FW', false)
  excl += ['resource_storage_file.c']
endif
sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '--exclude-list', ';'.join(excl), '*.c',
  check: true,
).stdout().strip().split('\n')
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

# Resource tables and headers derived from the resource ball.
pbl_generated += custom_target(
  'resource_version.auto.h',
  output: 'resource_version.auto.h',
  command: [py, resources_py, 'version-header', '--output', '@OUTPUT@']
  + (pbl_pbpack.length() > 0 ? ['--pbpack', pbl_pbpack[0].full_path()] : []),
  depends: pbl_pbpack,
)

resource_ids_h = custom_target(
  'resource_ids.auto.h',
  output: 'resource_ids.auto.h',
  input: resource_ball,
  command: [py, resources_py, 'resource-ids', '--ball', '@INPUT@', '--output', '@OUTPUT@'],
)
pbl_generated += resource_ids_h

if not cfg.get('CONFIG_RECOVERY_FW', false)
  pbl_resource_sources += custom_target(
    'timeline_resource_table.auto.c',
    output: 'timeline_resource_table.auto.c',
    input: resource_manifest,
    command: [py, resources_py, 'timeline-table', '--manifest', '@INPUT@', '--output', '@OUTPUT@'],
  )
  pbl_generated += custom_target(
    'timeline_resource_ids.auto.h',
    output: 'timeline_resource_ids.auto.h',
    input: resource_manifest,
    command: [py, resources_py, 'timeline-ids', '--manifest', '@INPUT@', '--output', '@OUTPUT@'],
  )
  pbl_resource_sources += custom_target(
    'pfs_resource_table.auto.c',
    output: 'pfs_resource_table.auto.c',
    input: resource_manifest,
    command: [
      py, resources_py, 'pfs',
      '--manifest', '@INPUT@', '--output', '@OUTPUT@',
      '--include', 'resource/resource_ids.auto.h',
    ],
    depends: resource_ids_h,
  )
endif

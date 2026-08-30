# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

foreach dir : ['.', 'applib/vendor/uPNG', 'applib/vendor/tinflate']
  pbl_global_inc += meson.current_source_dir() / dir
endforeach

# The firmware reports the version it was built from. The generator only
# rewrites the header when the version actually changes; CMake re-ran it on
# every build through a stamp file, which Meson has no equivalent for.
pbl_generated += custom_target(
  'git_version.auto.h',
  output: 'git_version.auto.h',
  input: files('git_version.auto.h.in'),
  command: [py, generate_py, 'git-version', '--template', '@INPUT@', '--output', '@OUTPUT@'],
  build_always_stale: true,
)

subdir('startup')
subdir('drivers')
subdir('board')
subdir('shell')
subdir('services')
subdir('applib')
subdir('mfg')
subdir('comm')
subdir('console')
subdir('debug')
subdir('flash_region')
subdir('kernel')
subdir('popups')
subdir('process_management')
subdir('process_state')
subdir('resource')
subdir('syscall')
subdir('system')
subdir('util')
subdir('apps/core')

if cfg.get('CONFIG_RECOVERY_FW', false)
  subdir('apps/prf')
else
  subdir('apps')
endif

sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '*.c',
  check: true,
).stdout().strip().split('\n')

if not cfg.get('CONFIG_RECOVERY_FW', false)
  # Translatable strings from the firmware core; apps, services and applib
  # have their own catalogs, merged in here.
  pot_sources = run_command(
    py, glob_py, '--base', meson.current_source_dir(), '--recurse', '--exclude-list', ';'.join(['apps', 'services', 'applib']), '*.c', '*.h', '*.def',
    check: true,
  ).stdout().strip().split('\n')
  fw_pot = custom_target(
    'fw.pot',
    output: 'fw.pot',
    input: files(pot_sources),
    command: pbl_xgettext,
  )

endif

pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

# Resource tables and headers derived from the resource ball. They live here
# because src/fw's build directory is on the include path.
pbl_generated += custom_target(
  'font_resource_keys.auto.h',
  output: 'font_resource_keys.auto.h',
  input: resource_ball,
  command: [py, resources_py, 'font-header', '--ball', '@INPUT@', '--output', '@OUTPUT@'],
)
pbl_generated += custom_target(
  'font_resource_table.auto.h',
  output: 'font_resource_table.auto.h',
  input: resource_ball,
  command: [py, resources_py, 'font-table', '--ball', '@INPUT@', '--output', '@OUTPUT@'],
)
pbl_resource_sources += custom_target(
  'builtin_resources.auto.c',
  output: 'builtin_resources.auto.c',
  input: resource_ball,
  command: [
    py, resources_py, 'builtin',
    '--ball', '@INPUT@', '--output', '@OUTPUT@',
    '--include', 'resource/resource_ids.auto.h',
  ],
  depends: resource_ids_h,
)

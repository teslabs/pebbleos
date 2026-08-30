# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

subdir('vendor/tinflate')
subdir('vendor/uPNG')

# The graphics code ships one implementation per screen colour depth.
if cfgval.get('CONFIG_SCREEN_COLOR_DEPTH_BITS', '') == '1'
  excl = ['8_bit/**']
else
  excl = ['1_bit/**']
endif
excl += ['vendor']

if cfg.get('CONFIG_RECOVERY_FW', false)
  excl += [
    'ui/dialogs/bt_conn_dialog.c',
    'ui/dialogs/expandable_dialog.c',
    'ui/option_menu_window.c',
    'ui/progress_window.c',
    'ui/selection_layer.c',
    'ui/date_selection_window.c',
    'ui/time_selection_window.c',
    'ui/time_range_selection_window.c',
    'ui/day_picker.c',
    'voice',
  ]
endif

if not cfg.get('CONFIG_BOARD_GETAFIX', false) and not cfg.get('CONFIG_BOARD_QEMU_GABBRO', false)
  excl += ['ui/window_stack_animation_round*.c']
endif

if not cfg.get('CONFIG_MIC', false)
  excl += ['voice/voice_window.c']
endif

if cfg.get('CONFIG_MAG', false)
  excl += ['compass_service_stub.c']
else
  excl += ['compass_service.c']
endif

if cfg.get('CONFIG_TOUCH', false)
  excl += ['touch_service_stub.c']
else
  excl += ['touch_service.c']
endif

sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '--recurse', '--exclude-list', ';'.join(excl), '*.c',
  check: true,
).stdout().strip().split('\n')

# applib_malloc.json describes the size of every applib object handed out
# to apps; the generator turns it into the allocator's tables.
applib_malloc_templates = []
foreach name : run_command(
  py, glob_py, '--base', pbl_base / 'tools', 'applib_malloc*',
  check: true,
).stdout().strip().split('\n')
  applib_malloc_templates += pbl_base / 'tools' / name
endforeach

applib_malloc = custom_target(
  'applib_malloc',
  output: ['applib_malloc.auto.h', 'applib_malloc.auto.c'],
  input: files('applib_malloc.json'),
  command: [
    py, generate_py, 'applib-malloc',
    '--input', '@INPUT@',
    '--header', '@OUTPUT0@',
    '--impl', '@OUTPUT1@',
    '--min-sdk', pbl_min_sdk_version,
  ],
  depend_files: applib_malloc_templates,
)
pbl_generated += applib_malloc[0]

applib_pot = custom_target(
  'applib.pot',
  output: 'applib.pot',
  input: files(sources),
  command: pbl_xgettext,
)

pbl_libs += [
  {
    'dir': meson.current_source_dir(),
    'srcs': files(sources) + [applib_malloc[1]],
  },
]

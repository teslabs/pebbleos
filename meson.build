# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# PebbleOS firmware build.
#
#   ./pbl configure --board asterix
#   ./pbl build
#
# ./pbl writes the cross file the toolchain is named in -- Meson has to
# know its compiler before any of this runs -- and turns --board and
# -DCONFIG_FOO=y into the options below. See docs/development/build_system.md.
#
# Meson has no user-defined functions, so the build model Zephyr expresses
# as pbl_library()/pbl_library_sources() is spelled out here instead: every
# directory appends what it contributes to the arrays declared below, and
# the libraries and the firmware image are assembled from them at the end,
# once the whole tree has been walked.

project(
  'pebbleos',
  'c',
  version: '1.0',
  meson_version: '>= 1.1.0',
  default_options: [
    # The firmware picks its own optimization, debug and warning flags.
    'buildtype=plain',
    'c_std=c11',
    'warning_level=0',
    'werror=false',
    # ... and its own link line: none of Meson's defaults apply to a
    # bare-metal image with its own linker script.
    'b_staticpic=false',
    'b_pie=false',
    'b_lundef=false',
    'b_asneeded=false',
  ],
)

fs = import('fs')
py = find_program('python3')
cc = meson.get_compiler('c')

pbl_base = meson.current_source_dir()
pbl_build = meson.current_build_dir()

kconfig_py = files('tools/meson/kconfig.py')
generate_py = files('tools/meson/generate.py')
firmware_py = files('tools/meson/firmware.py')
resources_py = files('tools/meson/resources.py')
glob_py = files('tools/meson/glob.py')

board = get_option('board')
variant = get_option('variant')
if board == ''
  error(
    'No board selected. Configure with -Dboard=<board>; the available '
    + 'boards are the directories under boards/.',
  )
endif

# --- Kconfig ---------------------------------------------------------------
#
# Meson cannot read a file out of the build directory, so the script prints
# the configuration it wrote and meson.build parses that into two
# dictionaries: cfg for `if CONFIG_FOO`, cfgval for the value itself.

kconfig_cmd = [
  py,
  kconfig_py,
  '--srcdir', pbl_base,
  '--builddir', pbl_build,
  '--board', board,
  '--variant', variant,
]
# Kept by `./pbl menuconfig`; delete it to go back to the board defaults.
if fs.exists(pbl_build / 'menuconfig.conf')
  kconfig_cmd += ['--fragment', pbl_build / 'menuconfig.conf']
endif
foreach override : get_option('config')
  kconfig_cmd += ['--override', override]
endforeach

kconfig = run_command(kconfig_cmd, check: true)
message('Kconfig: ' + kconfig.stderr().strip())

cfg = {}
cfgval = {}
kconfig_depends = []
foreach line : kconfig.stdout().strip().split('\n')
  parts = line.split('@@')
  if parts[0] == 'PBL_KCONFIG_DEPEND'
    kconfig_depends += parts[1]
  else
    cfgval += {parts[0]: parts[1]}
    cfg += {parts[0]: parts[1] != '' and parts[1] != 'n' and parts[1] != '0'}
  endif
endforeach

# Meson has no equivalent of CMAKE_CONFIGURE_DEPENDS. Reading a file with
# fs.read() is the one thing that puts it on the list Meson re-runs itself
# for, so every Kconfig source is read and the contents thrown away.
foreach dep : kconfig_depends
  fs.read(dep)
endforeach

pbl_autoconf_h = pbl_build / 'autoconf.h'
pbl_dotconfig = pbl_build / '.config'

pbl_board = cfgval['PBL_BOARD']
pbl_board_name = cfgval['PBL_BOARD_NAME']
pbl_board_normalized = cfgval['PBL_BOARD_NORMALIZED']
pbl_board_runners = cfgval['PBL_BOARD_RUNNERS'].split(';')

# The recovery firmware is selected with -Dvariant=prf, which loads
# prj_prf.conf; keep the two spellings in sync.
if variant == 'prf' and not cfg.get('CONFIG_RECOVERY_FW', false)
  error('variant=prf but CONFIG_RECOVERY_FW is not set')
endif

# --- Platform --------------------------------------------------------------

if cfg.get('CONFIG_PLATFORM_EMERY', false)
  pbl_platform = 'emery'
  pbl_min_sdk_version = '3'
elif cfg.get('CONFIG_PLATFORM_FLINT', false)
  pbl_platform = 'flint'
  pbl_min_sdk_version = '2'
elif cfg.get('CONFIG_PLATFORM_GABBRO', false)
  pbl_platform = 'gabbro'
  pbl_min_sdk_version = '3'
else
  error('No platform specified for ' + pbl_board)
endif

if pbl_board_runners.length() > 0 and pbl_board_runners[0] != ''
  pbl_runner = pbl_board_runners[0]
elif cfg.get('CONFIG_QEMU', false)
  pbl_runner = ''
else
  error('Board ' + pbl_board + ' does not define any supported runners')
endif

# Used for pblboot image naming; -1 when the board has no slots.
pbl_slot = cfg.get('CONFIG_PBLBOOT', false) ? cfgval['CONFIG_FIRMWARE_SLOT'] : '-1'

# --- Compiler flags --------------------------------------------------------
#
# add_project_arguments() would reach the stored apps too, which are built
# with the SDK's flags rather than the firmware's, so the flags are
# collected here and handed to each library explicitly.

pbl_c_args = [
  # Meson defines this for the host machine; the firmware has no large-file
  # support to ask for and the toolchain headers should not see it.
  '-U_FILE_OFFSET_BITS',
  '-Wall',
  '-Wextra',
  '-Werror',
  '-Wpointer-arith',
  '-Wno-unused-parameter',
  '-Wno-missing-field-initializers',
  '-Wno-address-of-packed-member',
]

pbl_arch_args = [
  '-fvar-tracking-assignments',
  '-mthumb',
  '-ffreestanding',
  '-ffunction-sections',
  # let --gc-sections drop unreferenced const/data objects too
  '-fdata-sections',
  '-fbuiltin',
  '-fno-builtin-itoa',
]

if cfg.get('CONFIG_DEBUG_INFO', false)
  # -g3 keeps macro definitions; -gdwarf-4 the more detailed debug info.
  pbl_arch_args += ['-g3', '-gdwarf-4']
endif

if cfg.get('CONFIG_COMPILER_SAVE_TEMPS', false)
  pbl_arch_args += '-save-temps=obj'
endif

if cfg.get('CONFIG_LTO', false)
  pbl_arch_args += [
    '-flto',
    '-flto-partition=balanced',
    '--param', 'lto-partitions=128',
    '-fuse-linker-plugin',
    '-fno-if-conversion',
    '-fno-caller-saves',
    '-fira-region=mixed',
    '-finline-functions',
    '-fconserve-stack',
    '--param', 'inline-unit-growth=1',
    '--param', 'max-inline-insns-auto=1',
    '--param', 'max-cse-path-length=1000',
    '--param', 'max-grow-copy-bb-insns=1',
    '-fno-hoist-adjacent-loads',
    '-fno-optimize-sibling-calls',
    '-fno-schedule-insns2',
  ]
endif

pbl_cpu_fpu = ''
if cfg.get('CONFIG_SOC_NRF52', false)
  pbl_arch_args += '-mcpu=cortex-m4'
  pbl_cpu_fpu = 'fpv4-sp-d16'
elif cfg.get('CONFIG_SOC_SF32LB52', false)
  pbl_arch_args += '-mcpu=star-mc1'
  pbl_cpu_fpu = 'fpv5-sp-d16'
elif cfg.get('CONFIG_QEMU', false) and cfg.get('CONFIG_CORTEX_M4', false)
  pbl_arch_args += ['-mcpu=cortex-m4', '-Dsniprintf=snprintf', '-D_USE_LONG_TIME_T']
elif cfg.get('CONFIG_QEMU', false) and cfg.get('CONFIG_CORTEX_M33', false)
  pbl_arch_args += [
    '-mcpu=cortex-m33+nofp+nodsp',
    '-Dsniprintf=snprintf',
    '-D_USE_LONG_TIME_T',
  ]
endif

# QEMU does not have an FPU. Without -mfloat-abi=softfp no FPU instructions
# are emitted, and __SOFTFP__=1 gets defined (a misleading name, but that
# is what the toolchain does).
if cfg.get('CONFIG_QEMU', false)
  pbl_cpu_fpu = ''
endif

if pbl_cpu_fpu != ''
  pbl_arch_args += ['-mfloat-abi=softfp', '-mfpu=' + pbl_cpu_fpu]
endif

# Reproducibility: strip the absolute source-root prefix from every
# embedded path, so binaries do not depend on where the tree sits.
# -ffile-prefix-map covers debug info and __FILE__; -fdebug-prefix-map is
# a subset, listed for toolchains predating -ffile-prefix-map.
# Meson names a source by its path relative to the build directory, so the
# relative spelling has to be mapped as well as the absolute one.
pbl_source_relpath = run_command(
  py, '-c', 'import os, sys; print(os.path.relpath(sys.argv[1], sys.argv[2]))',
  pbl_base, pbl_build,
  check: true,
).stdout().strip()

pbl_arch_args += [
  '-ffile-prefix-map=' + pbl_base + '=.',
  '-fdebug-prefix-map=' + pbl_base + '=.',
  '-ffile-prefix-map=' + pbl_source_relpath + '=.',
  '-fdebug-prefix-map=' + pbl_source_relpath + '=.',
]

if cfg.get('CONFIG_RELEASE', false)
  pbl_optimize = '-Os'
  message('Optimization: release (-Os)')
elif cfg.get('CONFIG_NO_OPTIMIZATIONS', false)
  pbl_optimize = '-O0'
  message('Optimization: none (-O0)')
elif cfg.get('CONFIG_DEBUG_OPTIMIZATIONS', false)
  pbl_optimize = '-Og'
  message('Optimization: debug (-Og)')
else
  pbl_optimize = '-Os'
  message('Optimization: size (-Os)')
endif

pbl_c_args += pbl_arch_args + [pbl_optimize]
pbl_link_args = ['-Wl,--warn-common'] + pbl_arch_args + [pbl_optimize]

# Kconfig reaches every compilation unit, headers included.
pbl_c_args += ['-include', pbl_autoconf_h]

# The PRF variant never ships the JS engine, even when the board's
# defconfig asks for it: undefine the symbol the sources guard on so they
# match what is actually linked.
if cfg.get('CONFIG_MODDABLE_XS', false) and not cfg.get('CONFIG_RECOVERY_FW', false)
  pbl_js_engine = 'moddable'
else
  pbl_js_engine = 'none'
  if cfg.get('CONFIG_MODDABLE_XS', false)
    pbl_c_args += '-UCONFIG_MODDABLE_XS'
    cfg += {'CONFIG_MODDABLE_XS': false}
  endif
endif

# time.h shims the firmware needs ahead of the toolchain's.
pbl_c_args += '-I' + pbl_base / 'src/fw/util/time'

# MAX_FONT_GLYPH_SIZE comes from the SDK platform description.
pbl_max_font_glyph_size = run_command(
  py, generate_py, 'platform-info',
  '--platform', pbl_platform, '--key', 'MAX_FONT_GLYPH_SIZE',
  check: true,
).stdout().strip()
pbl_c_args += '-DMAX_FONT_GLYPH_SIZE=' + pbl_max_font_glyph_size

# Stationary mode is for shipping watch firmware only.
if not cfg.get('CONFIG_RECOVERY_FW', false) and not cfg.get('CONFIG_QEMU', false) and not cfg.get('CONFIG_SHELL_SDK', false)
  pbl_c_args += '-DSTATIONARY_MODE'
endif

pbl_c_args += '-DFIRMWARE_OFFSET=' + cfgval['CONFIG_FIRMWARE_OFFSET']

objcopy = find_program('objcopy')
objcopy_args = ['-S', '-R', '.stack', '-R', '.priv_bss', '-R', '.bss', '-R', '.retained']

# --- C library -------------------------------------------------------------

subdir('meson/libc')

# --- Translatable strings --------------------------------------------------
#
# Every firmware area contributes a .pot file; they are merged into
# build/pebbleos.pot, which `./pbl make_lang` turns into per-language
# catalogs.

xgettext = find_program('xgettext')
msgcat = find_program('msgcat')

xgettext_args = ['-c/', '-k', '--from-code=UTF-8', '--language=C']
# gettext >= 0.22 shells out to git for a reproducible POT-Creation-Date,
# which warns once per source file on out-of-tree build paths.
if run_command(xgettext, '--help', check: true).stdout().contains('--no-git')
  xgettext_args += '--no-git'
endif
foreach keyword : [
  'i18n_noop',
  'i18n_get',
  'i18n_get_with_buffer',
  'sys_i18n_get_with_buffer',
  'i18n_ctx_noop:1c,2',
  'i18n_ctx_get:1c,2',
  'i18n_ctx_get_with_buffer:1c,2',
]
  xgettext_args += '--keyword=' + keyword
endforeach

# The command every .pot target runs; only its input and output differ.
pbl_xgettext = [xgettext, xgettext_args, '-o', '@OUTPUT@', '@INPUT@']

# --- Global registries -----------------------------------------------------
#
# Filled in by the directories walked below, read back once the walk is over.

pbl_libs = []                  # one dict per library
pbl_global_inc = []            # directories every library can include from
pbl_global_defs = []           # macros every library is built with
pbl_extra_link_args = []       # link options contributed from the tree
pbl_imported_libs = []         # vendor-supplied archives
pbl_generated = []             # generated headers every library waits for
pbl_dynamic_resources = []     # .reso files the build itself produces
pbl_services_pot = []          # per-service string catalogs
pbl_firmware_sources = []      # sources compiled straight into the image
pbl_resource_sources = []      # resource tables compiled into the image
pbl_linker_memory = []         # linker script fragments, per hook point
pbl_linker_rom_start = []
pbl_linker_ram_sections = []
pbl_linker_footer = []

# FIXME: remove include/pbl once all modules use the prefix
pbl_global_inc += [pbl_base / 'include', pbl_base / 'include/pbl']

# --- Native SDK shims ------------------------------------------------------
#
# The same generator produces the firmware side of the syscall boundary
# (pebble.auto.c) and the app side (libpebble.a plus the SDK headers), so
# the firmware build depends on it as well. Packaging the shippable SDK is
# a separate, waf-driven step; see docs/development/sdk_export.md.

pbl_sdk_dir = pbl_build / 'sdk' / pbl_platform

if not cfg.get('CONFIG_RECOVERY_FW', false)
  subdir('meson/native_sdk/fw')
  # Only headers belong in pbl_generated: it becomes a dependency whose
  # sources every library inherits, and a .c file there would be compiled
  # into all of them.
  pbl_generated += native_sdk[1]
  pbl_firmware_sources += native_sdk[0]
endif

# --- Stored apps -----------------------------------------------------------
#
# Built before the resource pipeline, which packs their binaries.

if not cfg.get('CONFIG_RECOVERY_FW', false)
  subdir('apps/stored')
endif

# --- Resources -------------------------------------------------------------
#
# Every entry of the board's resource maps becomes a .reso, the .reso files
# are collected into a resource ball, and the firmware's resource tables,
# headers and pbpack are derived from it. The maps are resolved here, at
# configure time, because the list of resources has to be known before the
# targets that build them can be declared; resources/meson.build turns the
# listing into one custom_target per resource.

# The timezone database is built straight into a resource.
tzdata_olson = pbl_base / 'resources/normal/base/tzdata/timezones_olson.txt'
if not cfg.get('CONFIG_RECOVERY_FW', false)
  pbl_dynamic_resources += custom_target(
    'tzdata.bin.reso',
    output: 'tzdata.bin.reso',
    command: [
      py, resources_py, 'tzdata',
      '--input', tzdata_olson, '--output', '@OUTPUT@',
    ],
    depend_files: [tzdata_olson],
  )
endif

dynamic_reso_paths = []
foreach reso : pbl_dynamic_resources
  dynamic_reso_paths += reso.full_path()
endforeach

# resources/meson.build declares the per-resource targets, so their .reso
# files land in that directory's build tree.
manifest_args = [
  '--builddir', pbl_build / 'resources',
  '--platform', pbl_platform,
  '--board-name', pbl_board_name,
  '--variant', variant,
  '--dynamic', dynamic_reso_paths,
]

resource_listing = run_command(
  py, resources_py, 'manifest',
  manifest_args, '--output', pbl_build / 'manifest.pickle', '--listing',
  check: true,
)
message('Resources: ' + resource_listing.stderr().strip())

resource_maps = []
resource_entries = []
foreach line : resource_listing.stdout().strip().split('\n')
  parts = line.split('@@')
  if parts[0] == 'MAP'
    resource_maps += parts[1]
  else
    resource_entries += [[parts[1], parts[2], parts[3]]]
  endif
endforeach

# Same trick as for Kconfig: reading the maps is what makes Meson re-run
# itself when a resource is added, removed or retyped.
resource_map_files = []
foreach map : resource_maps
  fs.read('resources' / map)
  resource_map_files += pbl_base / 'resources' / map
endforeach

resource_manifest = custom_target(
  'manifest.pickle',
  output: 'manifest.pickle',
  command: [py, resources_py, 'manifest', manifest_args, '--output', '@OUTPUT@'],
  depend_files: resource_map_files,
)

pbl_reso = []
subdir('resources')

resource_ball = custom_target(
  'system_resources.resball',
  output: 'system_resources.resball',
  command: [
    py, resources_py, 'ball',
    '--manifest', resource_manifest, '--output', '@OUTPUT@',
  ],
  depends: pbl_reso + pbl_dynamic_resources,
)

pbl_pbpack = []
if not cfg.get('CONFIG_RECOVERY_FW', false)
  pbl_pbpack += custom_target(
    'system_resources.pbpack',
    output: 'system_resources.pbpack',
    input: resource_ball,
    command: [py, resources_py, 'pbpack', '--ball', '@INPUT@', '--output', '@OUTPUT@', '--system'],
    build_by_default: true,
  )
  custom_target(
    'resources.size',
    output: 'resources.size',
    input: pbl_pbpack[0],
    command: [
      py, firmware_py, 'size-resources',
      '--config', pbl_dotconfig, '--pbpack', '@INPUT@', '--report', '@OUTPUT@',
    ],
    build_by_default: true,
  )
  layouts_template = pbl_base / 'resources/normal/base/layouts/layouts.json.in'
  pbl_layouts = [
    custom_target(
      'layouts.json.auto',
      output: 'layouts.json.auto',
      command: [
        py, resources_py, 'layouts',
        '--manifest', resource_manifest,
        '--template', layouts_template,
        '--output', '@OUTPUT@',
      ],
      depend_files: [layouts_template],
      build_by_default: true,
    ),
  ]
else
  pbl_layouts = []
endif

# --- The tree --------------------------------------------------------------

subdir('third_party')
subdir('lib')
subdir('soc')
subdir('subsys')
subdir('src')

# --- Translatable strings --------------------------------------------------

if not cfg.get('CONFIG_RECOVERY_FW', false)
  pot_inputs = [fw_pot, applib_pot, apps_pot]
  if pbl_services_pot.length() > 0
    pot_inputs += services_pot
  endif
  pbl_pot = custom_target(
    'pebbleos.pot',
    output: 'pebbleos.pot',
    input: pot_inputs,
    command: [msgcat, '@INPUT@', '-o', '@OUTPUT@'],
    build_by_default: true,
  )
endif

# --- Linker script ---------------------------------------------------------
#
# Libraries that need additions to the master script (src/fw/linker/
# pebbleos.ld) register fragments against one of its hook points; the
# fragments are aggregated into snippets-<location>.ld files that the
# master script includes.

subdir('meson/linker')

# --- The libraries ---------------------------------------------------------
#
# objects: every object file is linked (the default).
# stlib:   a static library the linker pulls from on demand. Used for
#          third-party code that ships more than the firmware references.

# Generated headers are not tracked per library; every library waits for
# all of them.
pbl_generated_dep = declare_dependency(sources: pbl_generated)

pbl_object_libs = []
pbl_static_libs = []

# A registered directory is included from both its source and its build
# tree, so generated headers are found next to their hand-written
# neighbours.
pbl_inc_args = []
foreach dir : pbl_global_inc
  pbl_inc_args += ['-I' + dir, '-I' + pbl_build / dir.replace(pbl_base + '/', '')]
endforeach

pbl_def_args = []
foreach define : pbl_global_defs
  pbl_def_args += '-D' + define
endforeach

pbl_common_args = pbl_c_args + pbl_def_args + pbl_inc_args

foreach lib : pbl_libs
  # A library is named after its directory unless it named itself.
  lib_dir = lib.get('dir', pbl_base)
  lib_name = lib.get('name', lib_dir.replace(pbl_base + '/', '').replace('/', '__'))

  lib_inc = []
  foreach dir : lib.get('inc', [])
    dir = dir.startswith('/') ? dir : lib_dir / dir
    lib_inc += ['-I' + dir, '-I' + pbl_build / dir.replace(pbl_base + '/', '')]
  endforeach

  lib_defs = []
  foreach define : lib.get('defs', [])
    lib_defs += '-D' + define
  endforeach

  target = static_library(
    lib_name,
    lib['srcs'],
    c_args: pbl_common_args + lib_inc + lib_defs + lib.get('args', []),
    dependencies: pbl_generated_dep,
    build_by_default: false,
    pic: false,
  )
  if lib.get('kind', 'objects') == 'objects'
    pbl_object_libs += target
  else
    pbl_static_libs += target
  endif
endforeach

# --- The firmware image ----------------------------------------------------

# A link group resolves the dependencies between the static libraries
# without having to order them by hand. Meson emits link_with libraries in
# dependency order and never repeats them, so the group is spelled out in
# link_args from the archives' own paths.
fw_link_args = pbl_link_args + [
  '-Wl,--cref',
  '-Wl,-Map=pebbleos.map',
  '-Wl,--gc-sections',
  '-Wl,--undefined=uxTopUsedPriority',
  '-Wl,--build-id=sha1',
  '-Wl,--sort-section=alignment',
  '-Wl,--print-memory-usage',
  # Every allocation goes through the firmware's own heaps.
  '-Wl,--wrap=malloc', '-Wl,--undefined=__wrap_malloc',
  '-Wl,--wrap=realloc', '-Wl,--undefined=__wrap_realloc',
  '-Wl,--wrap=calloc', '-Wl,--undefined=__wrap_calloc',
  '-Wl,--wrap=free', '-Wl,--undefined=__wrap_free',
  '-T' + pbl_ldscript.full_path(),
] + pbl_libc_link_args + pbl_extra_link_args

fw_link_args += '-Wl,--start-group'
foreach lib : pbl_static_libs
  fw_link_args += lib.full_path()
endforeach
fw_link_args += pbl_imported_libs + pbl_libc_libs + ['-Wl,--end-group']

pebbleos_elf = executable(
  'pebbleos.elf',
  pbl_resource_sources + pbl_firmware_sources,
  c_args: pbl_common_args,
  link_whole: pbl_object_libs,
  link_with: pbl_static_libs,
  link_args: fw_link_args,
  link_depends: [pbl_ldscript],
  dependencies: pbl_generated_dep,
  implicit_include_directories: false,
)


if cfg.get('CONFIG_PBLBOOT', false)
  # The bootloader expects an image header ahead of the firmware.
  nohdr_hex = custom_target(
    'pebbleos.nohdr.hex',
    output: 'pebbleos.nohdr.hex',
    input: pebbleos_elf,
    command: [objcopy, objcopy_args, '-O', 'ihex', '@INPUT@', '@OUTPUT@'],
  )
  nohdr_bin = custom_target(
    'pebbleos.nohdr.bin',
    output: 'pebbleos.nohdr.bin',
    input: pebbleos_elf,
    command: [objcopy, objcopy_args, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
  )
  pebbleos_hex = custom_target(
    'pebbleos.hex',
    output: 'pebbleos.hex',
    input: nohdr_hex,
    command: [
      py, generate_py, 'pblboot-header',
      '--input', '@INPUT@', '--output', '@OUTPUT@',
      '--offset', cfgval['CONFIG_FIRMWARE_OFFSET'],
    ],
    build_by_default: true,
  )
  pebbleos_bin = custom_target(
    'pebbleos.bin',
    output: 'pebbleos.bin',
    input: nohdr_bin,
    command: [
      py, generate_py, 'pblboot-header',
      '--input', '@INPUT@', '--output', '@OUTPUT@',
      '--offset', cfgval['CONFIG_FIRMWARE_OFFSET'],
    ],
    build_by_default: true,
  )
else
  pebbleos_hex = custom_target(
    'pebbleos.hex',
    output: 'pebbleos.hex',
    input: pebbleos_elf,
    command: [objcopy, objcopy_args, '-O', 'ihex', '@INPUT@', '@OUTPUT@'],
    build_by_default: true,
  )
  pebbleos_bin = custom_target(
    'pebbleos.bin',
    output: 'pebbleos.bin',
    input: pebbleos_elf,
    command: [objcopy, objcopy_args, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
    build_by_default: true,
  )
endif

# A custom_target runs a single command, so the size check the image steps
# used to end with is a target of its own, with its report as the output.
custom_target(
  'pebbleos.size',
  output: 'pebbleos.size',
  input: pebbleos_bin,
  command: [
    py, firmware_py, 'check-size',
    '--config', pbl_dotconfig, '--firmware', '@INPUT@', '--report', '@OUTPUT@',
  ],
  build_by_default: true,
)

# Hashed log strings: the dictionary the console and the bundle use to
# turn hashes back into messages.
if cfg.get('CONFIG_LOG_HASHED', false)
  fw_loghash = custom_target(
    'pebbleos_loghash_dict.json',
    output: 'pebbleos_loghash_dict.json',
    input: pebbleos_elf,
    command: [py, generate_py, 'loghash', '--elf', '@INPUT@', '--output', '@OUTPUT@'],
  )
  pbl_loghash = custom_target(
    'loghash_dict.json',
    output: 'loghash_dict.json',
    input: fw_loghash,
    command: [py, generate_py, 'loghash-merge', '--output', '@OUTPUT@', '@INPUT@'],
    build_by_default: true,
  )
endif

# --- Bundling and QEMU images ----------------------------------------------

bundle_args = [
  '--config', pbl_dotconfig,
  '--firmware', pebbleos_bin.full_path(),
  '--board', pbl_board_normalized,
  '--outdir', pbl_build,
  '--slot', pbl_slot,
]
bundle_deps = [pebbleos_bin]
if pbl_pbpack.length() > 0
  bundle_args += ['--resources', pbl_pbpack[0].full_path()]
  bundle_args += ['--layouts', pbl_layouts[0].full_path()]
  bundle_deps += [pbl_pbpack[0], pbl_layouts[0]]
endif
if cfg.get('CONFIG_LOG_HASHED', false)
  bundle_args += ['--loghash', pbl_loghash.full_path()]
  bundle_deps += [pbl_loghash]
endif

run_target('bundle', command: [py, firmware_py, 'bundle'] + bundle_args, depends: bundle_deps)

if cfg.get('CONFIG_QEMU', false)
  custom_target(
    'qemu_micro_flash.bin',
    output: 'qemu_micro_flash.bin',
    input: pebbleos_hex,
    command: [py, firmware_py, 'qemu-image-micro', '--input', '@INPUT@', '--output', '@OUTPUT@'],
    build_by_default: false,
  )
  if pbl_pbpack.length() > 0
    custom_target(
      'qemu_spi_flash.bin',
      output: 'qemu_spi_flash.bin',
      input: pbl_pbpack[0],
      command: [
        py, firmware_py, 'qemu-image-spi',
        '--config', pbl_dotconfig, '--pbpack', '@INPUT@', '--output', '@OUTPUT@',
      ],
      build_by_default: false,
    )
  endif
endif

# --- What ./pbl needs to know about this build -----------------------------

run_command(
  py,
  files('tools/meson/pbl_env.py'),
  '--config', pbl_dotconfig,
  '--output', pbl_build / 'pbl_env.json',
  '--set', 'BOARD=' + pbl_board,
  '--set', 'BOARD_NAME=' + pbl_board_name,
  '--set', 'BOARD_REVISION=' + cfgval['PBL_BOARD_REVISION'],
  '--set', 'BOARD_NORMALIZED=' + pbl_board_normalized,
  '--set', 'VARIANT=' + variant,
  '--set', 'PLATFORM_NAME=' + pbl_platform,
  '--set', 'SLOT=' + pbl_slot,
  '--set', 'RUNNER=' + pbl_runner,
  '--set', 'SUPPORTED_RUNNERS=' + cfgval['PBL_BOARD_RUNNERS'],
  check: true,
)

summary(
  {
    'board': pbl_board + ' (' + pbl_platform + ')',
    'variant': variant,
    'libc': pbl_libc_name,
    'JS engine': pbl_js_engine,
  },
  section: 'PebbleOS',
)

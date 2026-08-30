# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# One stored app. Every app needs a meson.build of its own: a
# custom_target output cannot name a subdirectory, so the only way to keep
# each app's generated sources and binaries apart is to give each one its
# own build directory.

app_name = 'golf'

# appinfo.json describes the app to the launcher and the app store.
appinfo_c = custom_target(
  'appinfo.auto.c',
  output: 'appinfo.auto.c',
  input: files('appinfo.json'),
  command: [py, generate_py, 'appinfo', '--input', '@INPUT@', '--output', '@OUTPUT@'],
)

# Stored apps have no message keys of their own, but the SDK headers
# expect the file to exist. resource_ids.auto.h is written by src/.
configure_file(
  output: 'message_keys.auto.h',
  command: [py, '-c', 'pass'],
  capture: true,
)

subdir('src')

app_elf = executable(
  'pebble-app.elf',
  app_sources + [appinfo_c, native_sdk[1]],
  c_args: sdk_flags + [
    '-I' + pbl_sdk_dir / 'include',
    '-I' + pbl_sdk_extra_include,
    '-I' + meson.current_source_dir(),
    '-I' + meson.current_source_dir() / 'src',
    '-I' + meson.current_build_dir(),
  ],
  link_args: [
    '-mcpu=cortex-m3',
    '-mthumb',
    '-Wl,--gc-sections',
    '-Wl,--warn-common',
    '-Os',
    '-Wl,--emit-relocs',
    # pie: true is how the app gets -fPIE; the firmware relocates it from
    # --emit-relocs, so the image itself must not be a PIE ELF.
    '-no-pie',
    '-T' + app_ldscript,
    pbl_sdk_dir / 'lib/libpebble.a',
  ],
  link_depends: [app_ldscript] + native_sdk,
  implicit_include_directories: false,
  pie: true,
  override_options: ['c_std=c99'],
)

app_raw_bin = custom_target(
  'pebble-app.raw.bin',
  output: 'pebble-app.raw.bin',
  input: app_elf,
  command: [objcopy, objcopy_args, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
)

app_bin = custom_target(
  'pebble-app.bin',
  output: 'pebble-app.bin',
  input: app_raw_bin,
  command: [
    py, generate_py, 'inject-metadata',
    '--input', '@INPUT@', '--output', '@OUTPUT@', '--elf', app_elf.full_path(),
  ],
  depends: app_elf,
)

pbl_dynamic_resources += custom_target(
  'app_' + app_name + '.bin.reso',
  output: 'app_' + app_name + '.bin.reso',
  input: app_bin,
  command: [
    py, generate_py, 'app-reso',
    '--input', '@INPUT@', '--output', '@OUTPUT@', '--name', app_name,
  ],
)

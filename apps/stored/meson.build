# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# Apps that ship inside the firmware's resource pack. They are built like
# any third-party app -- against the generated SDK, position independent,
# with the app metadata header stamped into the binary -- and the result
# is wrapped as a resource.
#
# Adding an app here also needs a raw resource entry named
# STORED_APP_<APPNAME> in resources/normal/base/resource_map.json (file:
# normal/raw/app_<appname>.bin) and an entry in the system app registry.
#
# Meson has no functions, so the per-app rules are a foreach over the app
# directories rather than a helper the way the CMake build had it.

# Apps are built the way a third-party app is: the SDK's own flags and
# headers, not the firmware's. Only the configuration is shared.
sdk_version = run_command(
  py, generate_py, 'sdk-version',
  '--input', pbl_base / 'src/fw/process_management/pebble_process_info.h',
  check: true,
).stdout().strip().split('\n')

sdk_flags = [
  '-std=c99',
  '-mcpu=cortex-m3',
  '-mthumb',
  '-ffunction-sections',
  '-fdata-sections',
  '-fcommon',
  '-g',
  '-Os',
]
if sdk_version[0] == '5' and sdk_version[1].to_int() > 19
  # Newer toolchains default time_t to 64-bit; apps expect 32-bit.
  sdk_flags += ['-D_TIME_H_', '-Dtime_t=long']
endif
sdk_flags += [
  '-Wall',
  '-Wextra',
  '-Werror',
  '-Wno-unused-parameter',
  '-Wno-error=unused-function',
  '-Wno-error=unused-variable',
  '-Wno-error=builtin-declaration-mismatch',
  '-Wno-error=format-truncation',
  '-Wno-error=expansion-to-defined',
  '-Wno-error=zero-length-bounds',
  '-Wno-error=cast-function-type',
  '-Wno-error=unused-value',
  '-include', pbl_autoconf_h,
  '-DMAX_FONT_GLYPH_SIZE=' + pbl_max_font_glyph_size,
]

# The platform's feature defines, as third-party apps see them.
foreach define : run_command(
  py, generate_py, 'platform-info',
  '--platform', pbl_platform, '--key', 'DEFINES',
  check: true,
).stdout().strip().split('\n')
  sdk_flags += '-D' + define
endforeach

app_ldscript = meson.current_source_dir() / 'pebble_app.ld'

foreach app : run_command(
  py, glob_py, '--base', meson.current_source_dir(), '*/appinfo.json',
  check: true,
).stdout().strip().split('\n')
  subdir(app.replace('/appinfo.json', ''))
endforeach

# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# The native SDK shims. The same generator produces the firmware side of
# the syscall boundary (pebble.auto.c) and the app side (libpebble.a plus
# the SDK headers), so the firmware build depends on it as well.
# Packaging the shippable SDK is a separate, waf-driven step; see
# docs/development/sdk_export.md.
#
# Only pebble.auto.c is declared as an output: the SDK headers and
# libpebble.a land under build/sdk/<platform>/, which a custom_target
# cannot name (Meson allows no path separator in an output). The stored
# apps depend on the target instead of on the files.
#
# The generator writes its firmware shim to <output_dir>/fw/, and an output
# name may not name a subdirectory either, so this module lives one level
# down -- in meson/native_sdk/fw -- to make that path the build directory
# the target already writes into.

native_sdk_script = pbl_base / 'tools/generate_native_sdk/generate_pebble_native_sdk_files.py'
native_sdk_symbols = pbl_base / 'tools/generate_native_sdk/exported_symbols.json'

# The headers the generator parses, as listed in exported_symbols.json.
native_sdk_headers = []
foreach header : run_command(
  py,
  '-c',
  'import json,os,sys\n'
  + 'root = sys.argv[2]\n'
  + 'for f in json.load(open(sys.argv[1]))["files"]:\n'
  + '    src = os.path.join(root, "src", f)\n'
  + '    print(src if os.path.exists(src) else os.path.join(root, f))\n',
  native_sdk_symbols,
  pbl_base,
  check: true,
).stdout().strip().split('\n')
  native_sdk_headers += header
endforeach

native_sdk = [
  custom_target(
    'pebble.auto.c',
    output: 'pebble.auto.c',
    command: [
      py, native_sdk_script,
      '--sdk-dir=' + pbl_sdk_dir,
      native_sdk_symbols,
      pbl_base / 'src',
      meson.current_build_dir() / '..',
      pbl_platform,
    ],
    depend_files: [native_sdk_symbols] + native_sdk_headers,
  ),
]

# The font keys apps may refer to, from the same whitelist.
native_sdk += custom_target(
  'pebble_fonts.h',
  output: 'pebble_fonts.h',
  command: [
    py, generate_py, 'sdk-fonts-header',
    '--input', native_sdk_symbols,
    '--output', '@OUTPUT@',
    '--platform', pbl_platform,
  ],
  depend_files: [native_sdk_symbols],
  depends: native_sdk[0],
)

pbl_sdk_extra_include = meson.current_build_dir()

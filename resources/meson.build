# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# One target per declared resource. The listing comes from the resource
# maps, resolved at configure time by the root meson.build; all this
# directory does is turn each entry into a custom_target.
#
# Meson refuses a custom_target output name containing a path separator,
# so the .reso files land here under flattened names instead of mirroring
# the resource tree the way the waf and CMake builds did.

foreach entry : resource_entries
  reso_deps = []
  foreach dep : entry[2].split(';')
    reso_deps += files(dep)
  endforeach
  pbl_reso += custom_target(
    entry[0],
    output: entry[0],
    command: [
      py, resources_py, 'reso',
      '--manifest', resource_manifest,
      '--index', entry[1],
      '--output', '@OUTPUT@',
    ],
    depend_files: reso_deps,
  )
endforeach

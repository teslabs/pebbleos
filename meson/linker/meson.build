# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# Linker script assembly. Libraries that need additions to the master
# script (src/fw/linker/pebbleos.ld) register fragments against one of its
# hook points; the fragments are aggregated into snippets-<location>.ld
# files that the master script includes.
#
# Fragments are included sorted by (sort key, registration order). Meson
# cannot sort an array, so the "key|path" entries go to the generator and
# come back sorted. files() refuses a path outside the calling directory,
# so everything here is named by absolute path instead.

linker_dir = pbl_base / 'src/fw/linker'

linker_snippets = []
linker_fragments = []

foreach location : [
  ['memory', pbl_linker_memory],
  ['rom-start', pbl_linker_rom_start],
  ['ram-sections', pbl_linker_ram_sections],
  ['footer', pbl_linker_footer],
]
  linker_snippets += configure_file(
    output: 'snippets-' + location[0] + '.ld',
    command: [py, generate_py, 'linker-snippets', location[1]],
    capture: true,
  )
  foreach entry : location[1]
    linker_fragments += entry.split('|')[1]
  endforeach
endforeach

# Fragments included by the master script and by the SoC fragments.
linker_common = []
foreach name : run_command(
  py, glob_py, '--base', linker_dir, '--recurse', '*.ld',
  check: true,
).stdout().strip().split('\n')
  linker_common += linker_dir / name
endforeach

# The linker script goes through the C preprocessor, which gives it access
# to the Kconfig symbols, #include and #if.
pbl_ldscript = custom_target(
  'pebbleos.ld',
  output: 'pebbleos.ld',
  command: [
    cc.cmd_array(),
    '-x', 'assembler-with-cpp', '-nostdinc', '-undef', '-E', '-P',
    '-I' + meson.current_build_dir(),
    '-I' + linker_dir,
    '-include', pbl_autoconf_h,
    linker_dir / 'pebbleos.ld',
    '-o', '@OUTPUT@',
  ],
  depend_files: linker_snippets + linker_fragments + linker_common + [pbl_autoconf_h],
)

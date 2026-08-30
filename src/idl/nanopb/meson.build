# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
#
# The protobuf schemas the firmware speaks, compiled by nanopb.

nanopb_generator = find_program('nanopb_generator')

protos = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '*.proto',
  check: true,
).stdout().strip().split('\n')

proto_sources = []
proto_headers = []
foreach proto : protos
  proto_name = proto.replace('.proto', '')
  # A schema may import any of its neighbours, and an .options file next to
  # one tunes the generated C types.
  generated = custom_target(
    proto_name + '.pb',
    output: [proto_name + '.pb.c', proto_name + '.pb.h'],
    command: [
      nanopb_generator,
      '-I', meson.current_source_dir(),
      '-D', meson.current_build_dir(),
      meson.current_source_dir() / proto,
    ],
    depend_files: files(protos) + run_command(
      py, glob_py, '--base', meson.current_source_dir(), '--allow-empty', '*.options',
      check: true,
    ).stdout().strip().split('\n'),
  )
  proto_sources += generated[0]
  proto_headers += generated[1]
endforeach

pbl_generated += proto_headers

pbl_libs += [
  {'name': 'proto_schemas', 'srcs': proto_sources, 'kind': 'stlib', 'inc': ['.']},
]

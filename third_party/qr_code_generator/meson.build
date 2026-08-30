# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = 'QR-Code-generator/c/qrcodegen.c'
includes = 'QR-Code-generator/c'
pbl_global_inc += meson.current_source_dir() / includes
pbl_libs += [
  {
    'name': 'qr_code_generator',
    'srcs': files([sources]),
    'kind': 'stlib',
    'args': ['-Wno-type-limits'],
  },
]

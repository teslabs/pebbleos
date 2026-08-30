# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

nanopb_sources = ['nanopb/pb_common.c', 'nanopb/pb_encode.c', 'nanopb/pb_decode.c']
nanopb_includes = 'nanopb'
pbl_global_inc += meson.current_source_dir() / nanopb_includes
pbl_libs += [{'name': 'nanopb', 'srcs': files(nanopb_sources), 'kind': 'stlib'}]

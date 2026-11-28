# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sbc_sources
  libsbc/src/bits.c
  libsbc/src/sbc.c
)

pbl_include_directories(libsbc/include)

pbl_library_named(sbc)
pbl_library_kind(stlib)
pbl_library_sources(${sbc_sources})

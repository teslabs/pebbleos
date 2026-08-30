# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if cfg.get('CONFIG_BOARD_ASTERIX', false)
  sources = ['boards/board_asterix.c']
elif cfg.get('CONFIG_BOARD_OBELIX', false)
  sources = ['boards/board_obelix.c']
elif cfg.get('CONFIG_BOARD_GETAFIX', false)
  sources = ['boards/board_getafix.c', 'displays/display_getafix.c']
elif cfg.get('CONFIG_BOARD_QEMU_EMERY', false)
  sources = ['boards/board_qemu_emery.c']
elif cfg.get('CONFIG_BOARD_QEMU_FLINT', false)
  sources = ['boards/board_qemu_flint.c']
elif cfg.get('CONFIG_BOARD_QEMU_GABBRO', false)
  # qemu_gabbro reuses the getafix round display row info data
  sources = ['boards/board_qemu_gabbro.c', 'displays/display_getafix.c']
else
  error('Unknown board ' + pbl_board)
endif

pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

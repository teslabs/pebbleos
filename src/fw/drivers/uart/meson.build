# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = []
if cfg.get('CONFIG_UART_QEMU', false)
  sources += ['qemu.c']
elif cfg.get('CONFIG_UART_NRF5', false)
  sources += ['nrf5.c']
elif cfg.get('CONFIG_UART_SF32LB', false)
  sources += ['sf32lb.c']
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

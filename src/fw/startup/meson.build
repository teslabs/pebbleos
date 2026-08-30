# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if cfg.get('CONFIG_BOARD_ASTERIX', false)
  # The SoC startup file comes from the nRF HAL, which hands it over
  # because files() only reaches the directory it is called from.
  startup_srcs = hal_nordic_system_src + files('startup_cortex_m.c')
elif cfg.get('CONFIG_BOARD_OBELIX', false) or cfg.get('CONFIG_BOARD_GETAFIX', false)
  startup_srcs = files('startup_sf32lb52.c')
elif cfg.get('CONFIG_QEMU', false)
  startup_srcs = files('startup_qemu.c')
else
  error('No clock configuration file specified for this platform')
endif

pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': startup_srcs}]

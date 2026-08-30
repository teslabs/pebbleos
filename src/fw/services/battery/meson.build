# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = ['battery_monitor.c']
if cfg.get('CONFIG_NRF_FUEL_GAUGE', false)
  sources += ['nrf_fuel_gauge/battery_curve.c', 'nrf_fuel_gauge/battery_state.c']
else
  sources += ['voltage/battery_curve.c', 'voltage/battery_state.c']
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

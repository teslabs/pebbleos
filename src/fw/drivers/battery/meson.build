# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = files('battery_common.c')
if cfg.get('CONFIG_BATTERY_QEMU', false)
  sources += driver_shared_srcs['qemu/qemu_battery.c']
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': sources}]

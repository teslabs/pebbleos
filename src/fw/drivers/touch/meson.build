# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = []
if cfg.get('CONFIG_TOUCH_QEMU', false)
  sources += driver_shared_srcs['qemu/qemu_touch.c']
elif cfg.get('CONFIG_TOUCH_CST816', false)
  sources += files('cst816/cst816.c')
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': sources}]

# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = files('cd_flash_driver.c') + files('flash_api.c') + files('flash_crc.c') + files('flash_erase.c')
if cfg.get('CONFIG_FLASH_QEMU', false)
  sources += driver_shared_srcs['qemu/qemu_flash_hal.c']
elif cfg.get('CONFIG_FLASH_GD25LQ255E', false)
  sources += files('gd25lq255e.c')
elif cfg.get('CONFIG_FLASH_GD25Q256E', false)
  sources += files('gd25q256e.c')
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': sources}]

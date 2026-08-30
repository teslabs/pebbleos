# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

accel_sources = []
if cfg.get('CONFIG_QEMU', false)
  accel_sources += driver_shared_srcs['qemu/qemu_accel.c']
elif cfg.get('CONFIG_ACCEL_LIS2DW12', false)
  accel_sources += files('lis2dw12/lis2dw12.c')
elif cfg.get('CONFIG_ACCEL_LSM6DSO', false)
  accel_sources += files('lsm6dso/lsm6dso.c')
endif
if accel_sources.length() > 0
  pbl_libs += [{'name': 'driver_accel', 'srcs': accel_sources}]
endif
mag_sources = []
if cfg.get('CONFIG_MAG_MMC5603NJ', false)
  mag_sources += ['mmc5603nj/mmc5603nj.c']
endif
if mag_sources.length() > 0
  pbl_libs += [{'name': 'driver_mag', 'srcs': files(mag_sources)}]
endif

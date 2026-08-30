# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if cfg.get('CONFIG_CORTEX_M4', false)
  port = 'ARM_CM4F'
elif cfg.get('CONFIG_CORTEX_M33', false)
  port = 'ARM_CM33'
else
  error('Unrecognized Cortex core for FreeRTOS port')
endif

kernel = 'FreeRTOS-Kernel/FreeRTOS/Source'
sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), kernel + '/*.c', kernel + '/portable/GCC/' + port + '/*.c',
  check: true,
).stdout().strip().split('\n')

foreach dir : ['.'] + [kernel + '/include'] + [kernel + '/portable/GCC/' + port]
  pbl_global_inc += meson.current_source_dir() / dir
endforeach

# SF32LB52: SiFli's HAL programs MPU regions 0..4 and Pebble's static MPU
# setup uses 5..7, so the per-task configurable regions live at 8..11
# (four slots: AppRAM, WorkerRAM and two NULL placeholders, to satisfy the
# FreeRTOS port's multiple-of-four invariant).
if cfg.get('CONFIG_SOC_SF32LB52', false)
  pbl_global_defs += ['FREERTOS_FIRST_MPU_REGION=8', 'FREERTOS_LAST_MPU_REGION=11']
endif

# QEMU CM33 has no vendor-reserved MPU regions, so Pebble's static regions
# take 0..3 and the FreeRTOS task regions 4..7.
if cfg.get('CONFIG_QEMU', false) and cfg.get('CONFIG_CORTEX_M33', false)
  pbl_global_defs += ['FREERTOS_FIRST_MPU_REGION=4', 'FREERTOS_LAST_MPU_REGION=7']
endif

pbl_libs += [{'name': 'freertos', 'srcs': files(sources), 'kind': 'stlib'}]

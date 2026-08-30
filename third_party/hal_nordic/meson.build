# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if cfg.get('CONFIG_HAL_NORDIC_NRF52840', false)
  pbl_global_defs += [
    'NRF52840_XXAA',
    'NRF_CONFIG_NFCT_PINS_AS_GPIOS',
    'CONFIG_GPIO_AS_PINRESET',
    'NRFX_I2S_ENABLED=1',
    'NRFX_I2S0_ENABLED=1',
    'NRFX_GPIOTE_ENABLED=1',
    'NRFX_GPIOTE_CONFIG_NUM_OF_EVT_HANDLERS=4',
    'NRFX_PDM_ENABLED=1',
    'NRFX_PPI_ENABLED=1',
    'NRFX_PWM_ENABLED=1',
    'NRFX_PWM0_ENABLED=1',
    'NRFX_SPIM_ENABLED=1',
    'NRFX_SPIM3_ENABLED=1',
    'NRFX_TIMER_ENABLED=1',
    'NRFX_TIMER1_ENABLED=1',
    'NRFX_TIMER2_ENABLED=1',
    'NRFX_TWIM_ENABLED=1',
    'NRFX_TWIM0_ENABLED=1',
    'NRFX_TWIM1_ENABLED=1',
    'NRFX_UARTE_ENABLED=1',
    'NRFX_UARTE0_ENABLED=1',
  ]
endif
micro_sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '--recurse', '--exclude-list', ';'.join(['**/system_nrf*.c', '**/startup_nrf_common.c', '**/nrfx_twi_twim.c']), 'nrfx/*.c',
  check: true,
).stdout().strip().split('\n')
micro_includes = [
  '.',
  'nrfx',
  'nrfx/hal',
  'nrfx/helpers',
  'nrfx/mdk',
  'nrfx/drivers/include',
  'nrfx/templates',
]
foreach dir : micro_includes
  pbl_global_inc += meson.current_source_dir() / dir
endforeach
pbl_libs += [
  {
    'name': 'hal_nordic',
    'srcs': files(micro_sources),
    'kind': 'stlib',
    'args': ['-Wno-unused-variable'],
  },
]

# The SoC startup file is left out of the HAL library and compiled with the
# firmware's own startup code; files() only reaches the directory it is
# called from, so the source is handed over from here.
hal_nordic_system_src = []
if cfg.get('CONFIG_HAL_NORDIC_NRF52840', false)
  hal_nordic_system_src = files('nrfx/mdk/system_nrf52840.c')
endif

# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_global_defs += ['USE_HAL_DRIVER', 'SOC_BF0_HCPU', 'HAL_TICK_PER_SECOND=1024', 'HAL_NMI_HANLDER_OVERRIDED']
pbl_extra_link_args += ['-Wl,--undefined=HAL_GetTick']
if cfg.get('CONFIG_HAL_SIFLI_SF32LB52', false)
  pbl_global_defs += ['SF32LB52X']
endif
micro_sources = [
  'SiFli-SDK/drivers/hal/bf0_hal.c',
  'SiFli-SDK/drivers/hal/bf0_hal_adc.c',
  'SiFli-SDK/drivers/hal/bf0_hal_cortex.c',
  'SiFli-SDK/drivers/hal/bf0_hal_dma.c',
  'SiFli-SDK/drivers/hal/bf0_hal_efuse.c',
  'SiFli-SDK/drivers/hal/bf0_hal_hlp.c',
  'SiFli-SDK/drivers/hal/bf0_hal_pmu.c',
  'SiFli-SDK/drivers/hal/bf0_hal_rcc.c',
  'SiFli-SDK/drivers/hal/bf0_sys_cfg.c',
]
if cfg.get('CONFIG_HAL_SIFLI_AES', false) or cfg.get('CONFIG_HAL_SIFLI_HASH', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_aes.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_AON', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_hpaon.c', 'SiFli-SDK/drivers/hal/bf0_hal_lpaon.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_ATIM', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_tim_ex.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_AUDCODEC', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_audcodec_m.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_AUDPRC', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_audprc.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_BUSMON', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_busmon.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_CRC', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_crc.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_EPIC', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_epic.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_EXTDMA', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_ext_dma.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_EZIP', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_ezip.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_GPIO', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_gpio.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_GPT', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_tim.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_HCD', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_hcd.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_I2C', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_i2c.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_I2S', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_i2s.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_LCD', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_lcdc.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_LCPU_CONFIG', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_lcpu_config.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_LCPU_PATCH', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_patch.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_LPTIM', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_lptim.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_LRC_CAL', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_lrc_cal.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_MAILBOX', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_mailbox.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_MPI', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_mpi.c', 'SiFli-SDK/drivers/hal/bf0_hal_mpi_ex.c', 'SiFli-SDK/drivers/hal/flash_table.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_PCD', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_pcd.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_PDM', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_pdm.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_PINMUX', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_pinmux.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_PTC', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_ptc.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_RNG', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_rng.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_RTC', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_rtc.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_SD', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_sdmmc.c', 'SiFli-SDK/drivers/hal/bf0_hal_sd_ex.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_SDHCI', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_sdhci.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_SECU', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_secu.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_SPI', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_spi.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_TSEN', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_tsen.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_UART', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_uart.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_WDT', false)
  micro_sources += ['SiFli-SDK/drivers/hal/bf0_hal_wdt.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_IPC_QUEUE', false)
  micro_sources += ['SiFli-SDK/middleware/ipc_queue/common/circular_buf.c', 'SiFli-SDK/middleware/ipc_queue/common/ipc_hw.c', 'SiFli-SDK/middleware/ipc_queue/common/ipc_queue.c']
endif
if cfg.get('CONFIG_HAL_SIFLI_SF32LB52', false)
  micro_sources += ['SiFli-SDK/drivers/cmsis/sf32lb52x/bf0_pin_const.c', 'sf32lb52/system_bf0_ap.c']
  if cfg.get('CONFIG_HAL_SIFLI_IPC_QUEUE', false)
    micro_sources += ['SiFli-SDK/middleware/ipc_queue/porting/sf32lb52x/hcpu/ipc_hw_port.c']
  endif
  if cfg.get('CONFIG_HAL_SIFLI_LCPU_CONFIG', false)
    micro_sources += ['SiFli-SDK/drivers/cmsis/sf32lb52x/lcpu_config_type.c']
  endif
  if cfg.get('CONFIG_HAL_SIFLI_LCPU', false)
    micro_sources += [
      'SiFli-SDK/drivers/cmsis/sf32lb52x/bf0_lcpu_init.c',
      'SiFli-SDK/drivers/cmsis/sf32lb52x/lcpu_patch.c',
      '../nonfree/pebbleos-nonfree/sf32lb52/lcpu_patch_rev_b.c',
      'SiFli-SDK/drivers/cmsis/sf32lb52x/bt_rf_fulcal.c',
      'sf32lb52/lcpu_config.c',
    ]
  endif
endif
micro_includes = [
  'SiFli-SDK/customer/boards/include',
  'SiFli-SDK/drivers/Include',
  'SiFli-SDK/drivers/cmsis/Include',
  'SiFli-SDK/external/CMSIS/Include',
  'SiFli-SDK/middleware/include',
  'SiFli-SDK/middleware/ipc_queue/common',
  'SiFli-SDK/middleware/ipc_queue/include',
]
if cfg.get('CONFIG_HAL_SIFLI_SF32LB52', false)
  micro_includes += ['sf32lb52', 'SiFli-SDK/drivers/cmsis/sf32lb52x', 'SiFli-SDK/middleware/ipc_queue/porting/sf32lb52x/hcpu']
endif
micro_defines = []
if cfg.get('CONFIG_BOARD_OBELIX', false) or cfg.get('CONFIG_BOARD_GETAFIX', false)
  micro_defines += ['LXT_DISABLE']
endif
foreach dir : micro_includes
  pbl_global_inc += meson.current_source_dir() / dir
endforeach
pbl_libs += [
  {
    'name': 'hal_sifli',
    'srcs': files(micro_sources),
    'kind': 'stlib',
    'defs': micro_defines,
    'args': [
  '-Wno-unused-value',
  '-Wno-unused-variable',
  '-Wno-unused-function',
  '-Wno-unused-but-set-variable',
  '-Wno-sign-compare',
  '-Wno-type-limits',
],
  },
]

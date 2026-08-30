# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

nonfree = 'pebbleos-nonfree'

if cfg.get('CONFIG_NRF_FUEL_GAUGE', false)
  if cfg.get('CONFIG_CORTEX_M4', false)
    fuel_gauge_lib = nonfree + '/npm1300/nrf_fuel_gauge/lib/cortex-m4/soft-float'
  elif cfg.get('CONFIG_CORTEX_M33', false)
    fuel_gauge_lib = nonfree + '/npm1300/nrf_fuel_gauge/lib/cortex-m33/soft-float'
  endif
  foreach dir : [nonfree + '/npm1300/nrf_fuel_gauge/include'] + [nonfree + '/npm1300/models']
    pbl_global_inc += meson.current_source_dir() / dir
  endforeach
  pbl_imported_libs += meson.current_source_dir() / fuel_gauge_lib / 'libnrf_fuel_gauge.a'
endif

if cfg.get('CONFIG_CST816_FW', false)
  if cfg.get('CONFIG_BOARD_OBELIX', false)
    pbl_global_inc += meson.current_source_dir() / nonfree + '/cst816/obelix'
  elif cfg.get('CONFIG_BOARD_GETAFIX', false)
    pbl_global_inc += meson.current_source_dir() / nonfree + '/cst816/getafix'
  else
    error('no CST816 firmware for board ' + pbl_board)
  endif
endif

if cfg.get('CONFIG_GH3X2X_ALGO', false)
  gh3x2x = nonfree + '/gh3x2x'
  foreach dir : [gh3x2x + '/algo_lib/algo_params/HR/04_EXCLUSIVE'] + [gh3x2x + '/demo_code/demo_algo_code/goodix_algo_application/inc'] + [gh3x2x + '/demo_code/demo_algo_code/goodix_algo_call/inc'] + [gh3x2x + '/demo_code/demo_algo_code/goodix_algo_call/inc/hr_exc'] + [gh3x2x + '/demo_code/demo_algo_code/goodix_algo_call/inc/spo2_pre_exc'] + [gh3x2x + '/demo_code/demo_kernel_code/driver/inc'] + [gh3x2x + '/demo_code/demo_kernel_code/kernel'] + [gh3x2x + '/demo_code/demo_kernel_code/module/gh_agc'] + [gh3x2x + '/demo_code/demo_kernel_code/module/gh_ecg'] + [gh3x2x + '/demo_code/demo_kernel_code/module/gh_other'] + [gh3x2x + '/demo_code/demo_kernel_code/module/gh_protocol'] + [gh3x2x + '/demo_code/demo_kernel_code/module/gh_soft_adt'] + [gh3x2x + '/demo_code/demo_mp_code/inc']
    pbl_global_inc += meson.current_source_dir() / dir
  endforeach
  # Vendor archives, named after the flags they were built with.
  foreach archive : [
    [gh3x2x + '/algo_lib/COMMON_DL', 'star-mc1_arm-gcc14.2.1_softfp+Os_common_dl'],
    [gh3x2x + '/algo_lib/COMMON_DSP', 'star-mc1_arm-gcc14.2.1_softfp+Os_common_dsp'],
    [gh3x2x + '/algo_lib/HR', 'GitInfo_star-mc1-arm-none-eabi-gcc1421(softfp+Os)'],
    [gh3x2x + '/algo_lib/HRV', 'star-mc1_arm-gcc14.2.1_softfp+Os_hrv'],
    [gh3x2x + '/algo_lib/NADT', 'star-mc1_arm-gcc14.2.1_softfp+Os_nadt'],
    [gh3x2x + '/algo_lib/SPO2', 'star-mc1_arm-gcc14.2.1_softfp+Os_spo2'],
    [gh3x2x + '/drv_lib', 'arm-none-eabi-gcc-softfp-Os-gh_common'],
  ]
    pbl_imported_libs += (
      meson.current_source_dir() / archive[0] / 'lib' + archive[1] + '.a'
    )
  endforeach

  sources = run_command(
    py, glob_py, '--base', meson.current_source_dir(), '--recurse', gh3x2x + '/algo_lib/*.c', gh3x2x + '/demo_code/*.c',
    check: true,
  ).stdout().strip().split('\n')
  pbl_libs += [
    {
      'name': 'gh3x2x',
      'srcs': files(sources),
      'args': ['-Wno-sign-compare'],
    },
  ]
endif

# The SF32LB52 LCPU image is compiled into the NimBLE transport, which lives
# in another directory; files() only reaches the calling one, so the source
# is handed over from here.
nonfree_lcpu_src = []
if cfg.get('CONFIG_SOC_SF32LB52', false)
  nonfree_lcpu_src = files(nonfree / 'sf32lb52/lcpu_52x.c')
endif

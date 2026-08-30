# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

nimble = 'mynewt-nimble'

includes = [nimble + '/nimble/include'] + [nimble + '/nimble/host/include'] + [nimble + '/nimble/host/services/ans/include'] + [nimble + '/nimble/host/services/bas/include'] + [nimble + '/nimble/host/services/bleuart/include'] + [nimble + '/nimble/host/services/gap/include'] + [nimble + '/nimble/host/services/gatt/include'] + [nimble + '/nimble/host/services/ias/include'] + [nimble + '/nimble/host/services/dis/include'] + [nimble + '/nimble/host/services/lls/include'] + [nimble + '/nimble/host/services/tps/include'] + [nimble + '/nimble/host/util/include'] + [nimble + '/nimble/transport/include'] + [nimble + '/nimble/transport/common/hci_h4/include'] + [nimble + '/porting/nimble/include'] + ['port/include']

sources = run_command(
  py, glob_py, '--base', meson.current_source_dir(), '--exclude-list', ';'.join([nimble + '/nimble/host/src/ble_gatts_lcl.c'] + [nimble + '/porting/nimble/src/hal_timer.c']), nimble + '/porting/nimble/src/*.c', nimble + '/nimble/src/*.c', nimble + '/nimble/host/src/*.c', nimble + '/nimble/host/util/src/*.c', nimble + '/nimble/host/services/bas/src/*.c', nimble + '/nimble/host/services/gap/src/*.c', nimble + '/nimble/host/services/gatt/src/*.c', nimble + '/nimble/host/services/dis/src/*.c', nimble + '/nimble/transport/src/*.c', nimble + '/nimble/transport/common/hci_h4/src/*.c', 'port/src/npl_os_pebble.c',
  check: true,
).stdout().strip().split('\n')

defines = []
extra_srcs = []

if cfg.get('CONFIG_SOC_NRF52', false)
  controller_sources = run_command(
    py, glob_py, '--base', meson.current_source_dir(), nimble + '/nimble/controller/src/*.c', nimble + '/nimble/drivers/nrf5x/src/*.c', nimble + '/nimble/drivers/nrf5x/src/nrf52/*.c',
    check: true,
  ).stdout().strip().split('\n')
  sources += controller_sources + [nimble + '/porting/nimble/src/hal_timer.c'] + ['port/src/nrf52.c']
  includes = ['port/include/nrf52'] + [nimble + '/nimble/controller/include'] + [nimble + '/nimble/drivers/nrf5x/include'] + includes
  defines += ['FREERTOS=1', 'PEBBLEOS', 'NRF52_SERIES', 'NIMBLE_CFG_CONTROLLER=1']
  pbl_global_defs += ['NIMBLE_CFG_CONTROLLER=1']
elif cfg.get('CONFIG_SOC_SF32LB52', false)
  sources += ['transport/hci_sf32lb52.c']
  # files() refuses a path outside this directory, so the LCPU image
  # comes from the nonfree directory's own files() (subdir'd first).
  extra_srcs = nonfree_lcpu_src
  includes = ['port/include/sf32lb52'] + includes

  if cfg.get('CONFIG_NIMBLE_HCI_SF32LB52_TRACE_BINARY', false)
    pbl_global_defs += ['NIMBLE_HCI_SF32LB52_TRACE', 'NIMBLE_HCI_SF32LB52_TRACE_BINARY']
  elif cfg.get('CONFIG_NIMBLE_HCI_SF32LB52_TRACE_LOG', false)
    pbl_global_defs += ['NIMBLE_HCI_SF32LB52_TRACE', 'NIMBLE_HCI_SF32LB52_TRACE_LOG']
  endif
endif

foreach dir : includes
  pbl_global_inc += meson.current_source_dir() / dir
endforeach

pbl_libs += [
  {
    'name': 'nimble',
    'srcs': files(sources) + extra_srcs,
    'defs': defines,
    'args': [
  '-Wno-pointer-arith',
  '-Wno-unused-function',
  '-Wno-sign-compare',
  '-Wno-old-style-declaration',
  '-Wimplicit-fallthrough=1',
],
  },
]

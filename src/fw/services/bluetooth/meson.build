# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = [
  'ble_bas.c',
  'bluetooth_ctl.c',
  'bluetooth_persistent_storage_debug.c',
  'bluetooth_prompt.c',
  'bonding.c',
  'dis.c',
  'local_addr.c',
  'local_id.c',
  'pairability.c',
  'pebble_pairing_service.c',
  'pp_ble_control.c',
]
if not cfg.get('CONFIG_RECOVERY_FW', false)
  sources += ['bluetooth_persistent_storage_normal.c']
  if cfg.get('CONFIG_HRM', false)
    sources += ['ble_hrm.c']
  endif
else
  sources += ['bluetooth_persistent_storage_prf.c']
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

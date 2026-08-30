# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_services_pot += custom_target(
  'services_phone_call.pot',
  output: 'services_phone_call.pot',
  input: files('service.c', 'util.c'),
  command: pbl_xgettext,
)
pbl_libs += [
  {
    'dir': meson.current_source_dir(),
    'srcs': files('service.c', 'util.c'),
  },
]

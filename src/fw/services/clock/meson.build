# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_services_pot += custom_target(
  'services_clock.pot',
  output: 'services_clock.pot',
  input: files('service.c'),
  command: pbl_xgettext,
)
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files('service.c')}]

# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = ['alarm.c', 'alarm_pin.c', 'alarm_tones.c']
pbl_services_pot += custom_target(
  'services_alarms.pot',
  output: 'services_alarms.pot',
  input: files(sources),
  command: pbl_xgettext,
)
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

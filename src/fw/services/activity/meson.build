# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = [
  'activity.c',
  'activity_calculators.c',
  'activity_insights.c',
  'activity_metrics.c',
  'activity_sessions.c',
  'health_util.c',
  'hr_util.c',
  'insights_settings.c',
  'kraepelin/activity_algorithm_kraepelin.c',
  'kraepelin/kraepelin_algorithm.c',
  'workout_service.c',
]
pbl_services_pot += custom_target(
  'services_activity.pot',
  output: 'services_activity.pot',
  input: files(sources),
  command: pbl_xgettext,
)
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

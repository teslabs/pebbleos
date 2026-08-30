# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = ['vibe_client.c', 'vibe_intensity.c', 'vibe_score.c', 'vibe_score_info.c']
pbl_services_pot += custom_target(
  'services_vibes.pot',
  output: 'services_vibes.pot',
  input: files(sources),
  command: pbl_xgettext,
)
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

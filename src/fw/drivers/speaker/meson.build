# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = []
if cfg.get('CONFIG_SPEAKER_DA7212', false)
  sources += ['nrf5/da7212.c']
elif cfg.get('CONFIG_SPEAKER_SF32LB', false)
  sources += ['sf32lb52/audec.c', 'sf32lb52/audio.c']
elif cfg.get('CONFIG_SPEAKER_QEMU', false)
  sources += ['qemu/audio.c']
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

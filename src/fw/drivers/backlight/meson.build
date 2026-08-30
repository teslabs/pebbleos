# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

sources = ['console.c']
if cfg.get('CONFIG_BACKLIGHT_AW2016', false)
  sources += ['aw2016.c']
elif cfg.get('CONFIG_BACKLIGHT_AW9364E', false)
  sources += ['aw9364e.c']
elif cfg.get('CONFIG_BACKLIGHT_PWM', false)
  sources += ['pwm.c']
elif cfg.get('CONFIG_BACKLIGHT_QEMU', false)
  sources += ['qemu.c']
endif
pbl_libs += [{'dir': meson.current_source_dir(), 'srcs': files(sources)}]

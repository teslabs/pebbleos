# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if cfg.get('CONFIG_BT_FW_NIMBLE', false)
  controller = 'nimble'
elif cfg.get('CONFIG_BT_FW_STUB', false)
  controller = 'stub'
else
  controller = 'qemu'
endif
subdir(controller)

# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BT_HFP)
  add_subdirectory(classic)
endif()

if(CONFIG_BT_FW_NIMBLE)
  set(controller "nimble")
elseif(CONFIG_BT_FW_STUB)
  set(controller "stub")
elseif(CONFIG_BT_FW_HCI_BRIDGE)
  set(controller "hci_bridge")
elseif(CONFIG_BT_FW_CLASSIC_DEMO)
  set(controller "classic_demo")
else()
  set(controller "qemu")
endif()
add_subdirectory(${controller})

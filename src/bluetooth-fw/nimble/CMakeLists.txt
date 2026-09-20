# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(excludes)
if(NOT CONFIG_BT_CLASSIC)
  list(APPEND excludes classic.c)
endif()
if(NOT CONFIG_BT_CONTROLLER_PROBE)
  list(APPEND excludes controller_probe.c)
endif()
if(NOT CONFIG_GH3X2X_TUNING_SERVICE_ENABLED)
  list(APPEND excludes
    gh3x2x_tuning_service.c
  )
endif()
file(GLOB driver_source CONFIGURE_DEPENDS *.c)
pbl_filter_sources(driver_source EXCLUDE ${excludes})
pbl_library_named(bt_driver)
pbl_library_sources(${driver_source})
if(CONFIG_BT_CONTROLLER_PROBE OR CONFIG_BT_CLASSIC)
  pbl_library_include_directories(
    ${PBL_BASE}/third_party/nimble/mynewt-nimble/nimble/host/src
  )
endif()

if(CONFIG_BT_CLASSIC)
  pbl_library_sources(
    ../hci_bridge/h4_stream.c ../hci_bridge/sf32lb52_audio_probe.c
    ../hci_bridge/sifli_sco.c ../hci_bridge/local_audio.c
    ../hci_bridge/voice_resampler.c ../hci_bridge/voice_playback.c
    ../hci_bridge/voice_echo.c ../hci_bridge/voice_echo_reference.c
  )
endif()

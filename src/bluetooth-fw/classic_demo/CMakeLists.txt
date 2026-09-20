# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
file(GLOB stubs CONFIGURE_DEPENDS ../stub/*.c)
pbl_filter_sources(stubs EXCLUDE ../stub/init.c)
pbl_library_named(bt_driver)
pbl_library_sources(${stubs} service.c
  ../hci_bridge/sf32lb52.c ../hci_bridge/h4_stream.c
  ../hci_bridge/sf32lb52_audio_probe.c ../hci_bridge/sifli_sco.c
  ../hci_bridge/local_audio.c ../hci_bridge/voice_resampler.c ../hci_bridge/voice_playback.c
  ${PBL_BASE}/third_party/nonfree/pebbleos-nonfree/sf32lb52/lcpu_52x.c
)

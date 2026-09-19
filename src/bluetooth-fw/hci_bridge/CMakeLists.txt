# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

file(GLOB sources CONFIGURE_DEPENDS ../stub/*.c)
pbl_filter_sources(sources EXCLUDE ../stub/init.c)
pbl_library_named(bt_driver)
pbl_library_sources(${sources} init.c sf32lb52.c
  ${PBL_BASE}/third_party/nonfree/pebbleos-nonfree/sf32lb52/lcpu_52x.c
)
if(CONFIG_BT_HCI_AUDIO_PROBE OR CONFIG_BT_HCI_AUDIO_ADAPTER)
  pbl_library_sources(sf32lb52_audio_probe.c sifli_sco.c)
endif()
if(CONFIG_BT_HCI_AUDIO_ADAPTER)
  pbl_library_sources(h4_stream.c)
endif()
if(CONFIG_BT_HCI_LOCAL_AUDIO)
  pbl_library_sources(local_audio.c voice_resampler.c voice_playback.c)
  pbl_library_sources(voice_echo.c voice_echo_reference.c)
endif()

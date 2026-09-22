# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_clar_test(test_cvsd
  INCLUDES subsys/bluetooth/classic
  SOURCES
    subsys/bluetooth/classic/cvsd.c
)

pbl_clar_test(test_hfp_service
  INCLUDES subsys/bluetooth/classic
  SOURCES
    subsys/bluetooth/classic/service.c
)

pbl_clar_test(test_hci_local_audio
  DEFINES CONFIG_PROMPT
  INCLUDES subsys/bluetooth/classic
  SOURCES
    subsys/bluetooth/classic/local_audio.c
    subsys/bluetooth/classic/voice_resampler.c
    subsys/bluetooth/classic/voice_playback.c
    subsys/bluetooth/classic/voice_echo.c
    subsys/bluetooth/classic/voice_echo_reference.c
  OVERRIDES dummy_board
)

pbl_clar_test(test_voice_echo
  INCLUDES subsys/bluetooth/classic
  SOURCES
    subsys/bluetooth/classic/voice_echo.c
    subsys/bluetooth/classic/voice_echo_reference.c
    subsys/bluetooth/classic/voice_resampler.c
)

pbl_clar_test(test_sifli_sco
  INCLUDES subsys/bluetooth/sf32lb52
  SOURCES
    subsys/bluetooth/classic/cvsd.c
    subsys/bluetooth/sf32lb52/h4_stream.c
    subsys/bluetooth/sf32lb52/sifli_sco.c
)

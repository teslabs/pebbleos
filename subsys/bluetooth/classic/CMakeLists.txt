# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_library()
pbl_library_sources(cvsd.c host.c l2cap.c profile.c sdp.c service.c)
pbl_library_sources_ifdef(CONFIG_BT_HCI_LOCAL_AUDIO
  local_audio.c voice_echo.c voice_echo_reference.c voice_playback.c voice_resampler.c
)

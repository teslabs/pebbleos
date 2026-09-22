# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_library()
pbl_library_sources(audio_probe.c sifli_sco.c)
pbl_library_sources_ifdef(CONFIG_BT_HCI_AUDIO_ADAPTER h4_stream.c)

# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

set(sources spi_mem.c)
if(CONFIG_SPI_MEM_NRF5_QSPI)
  list(APPEND sources nrf5_qspi.c)
endif()
pbl_library()
pbl_library_sources(${sources})

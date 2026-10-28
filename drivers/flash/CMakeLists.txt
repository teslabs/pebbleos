# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

pbl_library()
pbl_library_sources(
  flash.c
  flash_crc.c
)
pbl_library_sources_ifdef(CONFIG_FLASH_SEC_REGS sec_regs.c)
pbl_library_sources_ifdef(CONFIG_FLASH_GD25LQ255E gd25lq255e.c)
pbl_library_sources_ifdef(CONFIG_FLASH_GD25Q256E gd25q256e.c)
pbl_library_sources_ifdef(CONFIG_FLASH_NRF5_QSPI nrf5_qspi.c)
pbl_library_sources_ifdef(CONFIG_FLASH_SF32LB52_MPI sf32lb52_mpi.c)
pbl_library_sources_ifdef(CONFIG_FLASH_QEMU qemu.c)
pbl_library_sources_ifdef(CONFIG_FLASH_POSIX posix.c)
pbl_library_sources_ifdef(CONFIG_FLASH_SHELL shell.c)

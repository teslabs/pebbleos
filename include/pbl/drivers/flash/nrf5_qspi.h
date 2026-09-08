/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include <pbl/drivers/flash.h>
#include <pbl/drivers/flash/nor_part.h>
#include "pbl/kernel/sem.h"

enum pbl_flash_nrf5_qspi_read_mode {
  PBL_FLASH_NRF5_QSPI_READ_FASTREAD,
  PBL_FLASH_NRF5_QSPI_READ_READ2O,
  PBL_FLASH_NRF5_QSPI_READ_READ2IO,
  PBL_FLASH_NRF5_QSPI_READ_READ4O,
  PBL_FLASH_NRF5_QSPI_READ_READ4IO,
};

enum pbl_flash_nrf5_qspi_write_mode {
  PBL_FLASH_NRF5_QSPI_WRITE_PP,
  PBL_FLASH_NRF5_QSPI_WRITE_PP2O,
  PBL_FLASH_NRF5_QSPI_WRITE_PP4O,
  PBL_FLASH_NRF5_QSPI_WRITE_PP4IO,
};

struct pbl_flash_nrf5_qspi_state {
  struct pbl_flash_device_state flash;
  struct pbl_sem sem;
  bool initialized;
};

struct pbl_flash_nrf5_qspi {
  struct pbl_flash_device dev;
  const struct pbl_flash_nor_part *part;
  uint32_t clk_freq_hz;
  uint32_t cs_gpio;
  uint32_t clk_gpio;
  uint32_t data_gpio[4];
  enum pbl_flash_nrf5_qspi_read_mode read_mode;
  enum pbl_flash_nrf5_qspi_write_mode write_mode;
};

extern const struct pbl_flash_ops pbl_flash_nrf5_qspi_ops;

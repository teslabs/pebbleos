/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <pbl/drivers/flash.h>

#include <bf0_hal.h>

struct pbl_flash_sf32lb52_mpi_state {
  struct pbl_flash_device_state flash;
  QSPI_FLASH_CTX_T ctx;
  DMA_HandleTypeDef hdma;
  qspi_configure_t cfg;
  struct dma_config dma;
  bool initialized;
};

struct pbl_flash_sf32lb52_mpi {
  struct pbl_flash_device dev;
  const char *name;
  uint32_t id;
  uint16_t clk_div;
  //! Deep power-down entry / exit latencies of the part.
  uint32_t dpd_enter_us;
  uint32_t dpd_exit_us;
};

extern const struct pbl_flash_ops pbl_flash_sf32lb52_mpi_ops;

//! Deep power-down around SoC deep sleep. Called with interrupts disabled.
void pbl_flash_sf32lb52_mpi_dpd_enter(const struct pbl_flash_device *dev);
void pbl_flash_sf32lb52_mpi_dpd_exit(const struct pbl_flash_device *dev);

/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include <pbl/drivers/flash.h>
#include <pbl/drivers/flash/nor.h>
#include <pbl/util/misc.h>

#include <bf0_hal.h>

/**
 * @defgroup drivers_flash_sf32lb52_mpi SF32LB52 MPI
 * @ingroup drivers_flash
 * @brief @ref drivers_flash driver for a NOR part on an SF32LB52 MPI controller.
 *
 * The flash is memory-mapped and the CPU executes from it: reads are plain copies, and programs
 * and erases run with interrupts locked, so erases are synchronous and are not suspended.
 * Addresses start at the memory-mapped base.
 *
 * @code{.c}
 * PBL_FLASH_SF32LB52_MPI_DEFINE(s_flash, "flash", &pbl_flash_nor_gd25q256e, FLASH2,
 *                               FLASH2_BASE_ADDR, 32, 0, DMA1_Channel2, DMAC1_CH2_IRQn,
 *                               DMA_REQUEST_1);
 * @endcode
 * @{
 */

/** @brief Driver state. */
struct pbl_flash_sf32lb52_mpi_data {
  /** HAL flash context. */
  QSPI_FLASH_CTX_T ctx;
  /** DMA handle. */
  DMA_HandleTypeDef hdma;
  /** HAL controller configuration. */
  qspi_configure_t cfg;
  /** DMA configuration. */
  struct dma_config dma;
  /** Time to enter deep power-down, in microseconds. */
  uint32_t dpd_enter_us;
  /** Time to leave deep power-down, in microseconds. */
  uint32_t dpd_exit_us;
  /** Controller initialized. */
  bool initialized;
};

/** @brief NOR flash on an SF32LB52 MPI controller. */
struct pbl_flash_sf32lb52_mpi {
  /** Flash. */
  struct pbl_flash flash;
  /** Driver state. */
  struct pbl_flash_sf32lb52_mpi_data *data;
  /** Part. */
  const struct pbl_flash_nor_part *part;
  /** Clock divider. */
  uint16_t clk_div;
};

/** @cond INTERNAL_HIDDEN */
extern const struct pbl_flash_ops pbl_flash_sf32lb52_mpi_ops;
/** @endcond */

/**
 * @brief Define a NOR flash on an SF32LB52 MPI controller.
 *
 * @param sym Symbol of the flash.
 * @param _name Name.
 * @param _part Part, a struct pbl_flash_nor_part.
 * @param _mpi MPI instance, e.g. @c FLASH2.
 * @param _base Memory-mapped base address.
 * @param _msize_mb Size of the memory-mapped window, in MiB.
 * @param _clk_div Clock divider.
 * @param _dma_ch DMA channel instance.
 * @param _dma_irq DMA channel interrupt.
 * @param _dma_req DMA request.
 */
#define PBL_FLASH_SF32LB52_MPI_DEFINE(sym, _name, _part, _mpi, _base, _msize_mb, _clk_div,      \
                                      _dma_ch, _dma_irq, _dma_req)                              \
  PBL_FLASH_STATE_DEFINE(sym);                                                                  \
  static struct pbl_flash_sf32lb52_mpi_data sym##_data = {                                      \
    .cfg =                                                                                      \
        {                                                                                       \
          .Instance = (_mpi),                                                                   \
          .line = HAL_FLASH_QMODE,                                                              \
          .base = (_base),                                                                      \
          .msize = (_msize_mb),                                                                 \
          .SpiMode = SPI_MODE_NOR,                                                              \
        },                                                                                      \
    .dma = {                                                                                    \
      .Instance = (_dma_ch),                                                                    \
      .dma_irq = (_dma_irq),                                                                    \
      .request = (_dma_req),                                                                    \
    },                                                                                          \
  };                                                                                            \
  const struct pbl_flash_sf32lb52_mpi sym = {                                                   \
    .flash = PBL_FLASH_INIT(sym, _name, nullptr, nullptr, &pbl_flash_sf32lb52_mpi_ops, (_base), \
                            &(_part)->geometry, &(_part)->sec_regs),                            \
    .data = &sym##_data,                                                                        \
    .part = (_part),                                                                            \
    .clk_div = (_clk_div),                                                                      \
  };                                                                                            \
  PBL_DEVICE_REGISTER(sym, &sym.flash.dev)

/**
 * @brief Get the driver state of a flash, for the SoC deep sleep code.
 *
 * Deep sleep powers the flash down from RAM, with the HAL context and timings found here.
 *
 * @param flash Flash, defined with PBL_FLASH_SF32LB52_MPI_DEFINE().
 * @return Driver state.
 */
static inline struct pbl_flash_sf32lb52_mpi_data *pbl_flash_sf32lb52_mpi_data(
    const struct pbl_flash *flash) {
  return container_of(flash, const struct pbl_flash_sf32lb52_mpi, flash)->data;
}

/** @} */

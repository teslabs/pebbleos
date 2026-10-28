/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include <pbl/drivers/flash.h>
#include <pbl/drivers/flash/nor.h>
#include <pbl/drivers/spi_mem.h>

/**
 * @defgroup drivers_flash_spi_nor SPI NOR
 * @ingroup drivers_flash
 * @brief @ref drivers_flash driver for a JEDEC NOR part on a @ref drivers_spi_mem controller.
 *
 * The part is described by its SFDP tables (JESD216): size, page size, erase types and times,
 * read modes and quad enable. The board's part description fills in what SFDP does not cover
 * (security registers and latencies) and stands in for SFDP on parts without it. The widest
 * read and program modes both the part and the controller support are used. Addresses start at
 * 0.
 *
 * @code{.c}
 * PBL_FLASH_SPI_NOR_DEFINE(s_flash, "flash", &s_qspi.mem, &pbl_flash_nor_gd25lq255e);
 * @endcode
 * @{
 */

/** @cond INTERNAL_HIDDEN */
struct pbl_flash_spi_nor_data {
  struct pbl_flash_geometry geometry;
  uint8_t addr_nbytes;
  uint8_t read_opcode;
  uint8_t read_dummy;
  uint8_t read_addr_width;
  uint8_t read_data_width;
  uint8_t pp_opcode;
  uint8_t pp_data_width;
  uint8_t sector_opcode;
  uint8_t subsector_opcode;
  bool initialized;
};
/** @endcond */

/** @brief NOR flash on a SPI memory controller. */
struct pbl_flash_spi_nor {
  /** Flash. */
  struct pbl_flash flash;
  /** Driver state. */
  struct pbl_flash_spi_nor_data *data;
  /** Controller. */
  const struct pbl_spi_mem *bus;
  /** Part. */
  const struct pbl_flash_nor_part *part;
};

/** @cond INTERNAL_HIDDEN */
extern const struct pbl_flash_ops pbl_flash_spi_nor_ops;
/** @endcond */

/**
 * @brief Define a NOR flash on a SPI memory controller.
 *
 * @param sym Symbol of the flash.
 * @param _name Name.
 * @param _bus Controller, a struct pbl_spi_mem.
 * @param _part Part, a struct pbl_flash_nor_part.
 */
#define PBL_FLASH_SPI_NOR_DEFINE(sym, _name, _bus, _part)                                  \
  PBL_FLASH_STATE_DEFINE(sym);                                                             \
  static struct pbl_flash_spi_nor_data sym##_data;                                         \
  const struct pbl_flash_spi_nor sym = {                                                   \
    .flash = PBL_FLASH_INIT(sym, _name, &(_bus)->dev, nullptr, &pbl_flash_spi_nor_ops, 0U, \
                            &sym##_data.geometry, &(_part)->sec_regs),                     \
    .data = &sym##_data,                                                                   \
    .bus = (_bus),                                                                         \
    .part = (_part),                                                                       \
  };                                                                                       \
  PBL_DEVICE_REGISTER(sym, &sym.flash.dev)

/** @} */

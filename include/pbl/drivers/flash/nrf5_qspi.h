/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include <pbl/drivers/flash.h>
#include <pbl/drivers/flash/nor.h>
#include <pbl/drivers/gpio.h>
#include <pbl/kernel/sem.h>

/**
 * @defgroup drivers_flash_nrf5_qspi nRF5 QSPI
 * @ingroup drivers_flash
 * @brief @ref drivers_flash driver for a NOR part on the nRF5 QSPI peripheral.
 *
 * Addresses start at 0. The QSPI interrupt is connected by the board:
 *
 * @code{.c}
 * PBL_FLASH_NRF5_QSPI_DEFINE(s_flash, "flash", &pbl_flash_nor_gd25lq255e, 8000000,
 *                            PBL_FLASH_NRF5_QSPI_READ4IO, PBL_FLASH_NRF5_QSPI_PP4O,
 *                            PBL_GPIO(NRF5_GPIO_P0, 19, 0), PBL_GPIO(NRF5_GPIO_P0, 17, 0),
 *                            PBL_GPIO(NRF5_GPIO_P0, 20, 0), PBL_GPIO(NRF5_GPIO_P0, 21, 0),
 *                            PBL_GPIO(NRF5_GPIO_P0, 22, 0), PBL_GPIO(NRF5_GPIO_P0, 23, 0));
 * PBL_IRQ_CONNECT(QSPI, 5, pbl_flash_nrf5_qspi_irq_handler, &s_flash, 0);
 * @endcode
 * @{
 */

/** @brief Read command, as the READOC field of IFCONFIG0. */
enum pbl_flash_nrf5_qspi_read {
  /** Fast read (1-1-1). */
  PBL_FLASH_NRF5_QSPI_FASTREAD,
  /** Dual output read (1-1-2). */
  PBL_FLASH_NRF5_QSPI_READ2O,
  /** Dual I/O read (1-2-2). */
  PBL_FLASH_NRF5_QSPI_READ2IO,
  /** Quad output read (1-1-4). */
  PBL_FLASH_NRF5_QSPI_READ4O,
  /** Quad I/O read (1-4-4). */
  PBL_FLASH_NRF5_QSPI_READ4IO,
};

/** @brief Program command, as the WRITEOC field of IFCONFIG0. */
enum pbl_flash_nrf5_qspi_write {
  /** Page program (1-1-1). */
  PBL_FLASH_NRF5_QSPI_PP,
  /** Dual input page program (1-1-2). */
  PBL_FLASH_NRF5_QSPI_PP2O,
  /** Quad input page program (1-1-4). */
  PBL_FLASH_NRF5_QSPI_PP4O,
  /** Quad I/O page program (1-4-4). */
  PBL_FLASH_NRF5_QSPI_PP4IO,
};

/** @cond INTERNAL_HIDDEN */
struct pbl_flash_nrf5_qspi_data {
  struct pbl_sem done;
  bool initialized;
};
/** @endcond */

/** @brief NOR flash on the nRF5 QSPI peripheral. */
struct pbl_flash_nrf5_qspi {
  /** Flash. */
  struct pbl_flash flash;
  /** Driver state. */
  struct pbl_flash_nrf5_qspi_data *data;
  /** Part. */
  const struct pbl_flash_nor_part *part;
  /** Clock frequency, in Hz. */
  uint32_t freq_hz;
  /** Read command. */
  enum pbl_flash_nrf5_qspi_read read;
  /** Program command. */
  enum pbl_flash_nrf5_qspi_write write;
  /** SCK pin. */
  struct pbl_gpio sck;
  /** CSN pin. */
  struct pbl_gpio csn;
  /** IO0 to IO3 pins. */
  struct pbl_gpio io[4];
};

/** @cond INTERNAL_HIDDEN */
extern const struct pbl_flash_ops pbl_flash_nrf5_qspi_ops;
/** @endcond */

/**
 * @brief Define a NOR flash on the nRF5 QSPI peripheral.
 *
 * @param sym Symbol of the flash.
 * @param _name Name.
 * @param _part Part, a struct pbl_flash_nor_part.
 * @param _freq_hz Clock frequency, in Hz: 2 to 32 MHz.
 * @param _read Read command, a @ref pbl_flash_nrf5_qspi_read.
 * @param _write Program command, a @ref pbl_flash_nrf5_qspi_write.
 * @param _sck SCK pin, from PBL_GPIO().
 * @param _csn CSN pin, from PBL_GPIO().
 * @param _io0 IO0 pin, from PBL_GPIO().
 * @param _io1 IO1 pin, from PBL_GPIO().
 * @param _io2 IO2 pin, from PBL_GPIO().
 * @param _io3 IO3 pin, from PBL_GPIO().
 */
#define PBL_FLASH_NRF5_QSPI_DEFINE(sym, _name, _part, _freq_hz, _read, _write, _sck, _csn, _io0, \
                                   _io1, _io2, _io3)                                             \
  PBL_FLASH_STATE_DEFINE(sym);                                                                   \
  static struct pbl_flash_nrf5_qspi_data sym##_data = {.done = PBL_SEM_INITIALIZER(0, 1)};       \
  const struct pbl_flash_nrf5_qspi sym = {                                                       \
    .flash = PBL_FLASH_INIT(sym, _name, nullptr, nullptr, &pbl_flash_nrf5_qspi_ops, 0U,          \
                            &(_part)->geometry, &(_part)->sec_regs),                             \
    .data = &sym##_data,                                                                         \
    .part = (_part),                                                                             \
    .freq_hz = (_freq_hz),                                                                       \
    .read = (_read),                                                                             \
    .write = (_write),                                                                           \
    .sck = _sck,                                                                                 \
    .csn = _csn,                                                                                 \
    .io = {_io0, _io1, _io2, _io3},                                                              \
  };                                                                                             \
  PBL_DEVICE_REGISTER(sym, &sym.flash.dev)

/**
 * @brief QSPI interrupt handler, to connect with PBL_IRQ_CONNECT().
 *
 * @param qspi Flash.
 */
void pbl_flash_nrf5_qspi_irq_handler(const struct pbl_flash_nrf5_qspi *qspi);

/** @} */

/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//! SPI memory bus: a controller executes command/address/dummy/data
//! transactions against a memory (NOR, NAND, PSRAM) without knowing the
//! device's protocol. Modelled after Linux spi-mem.

struct pbl_spi_mem_device;

enum pbl_spi_mem_data_dir {
  PBL_SPI_MEM_NO_DATA,
  PBL_SPI_MEM_DATA_IN,
  PBL_SPI_MEM_DATA_OUT,
};

struct pbl_spi_mem_op {
  struct {
    uint8_t opcode;
    uint8_t buswidth;
  } cmd;
  struct {
    uint8_t nbytes;
    uint8_t buswidth;
    uint32_t val;
  } addr;
  struct {
    uint8_t nbytes;
    uint8_t buswidth;
  } dummy;
  struct {
    enum pbl_spi_mem_data_dir dir;
    uint8_t buswidth;
    size_t nbytes;
    union {
      void *in;
      const void *out;
    } buf;
  } data;
};

#define PBL_SPI_MEM_OP_CMD(op, width) {.opcode = (op), .buswidth = (width)}
#define PBL_SPI_MEM_OP_ADDR(n, value, width) {.nbytes = (n), .val = (value), .buswidth = (width)}
#define PBL_SPI_MEM_OP_NO_ADDR {0}
#define PBL_SPI_MEM_OP_DUMMY(n, width) {.nbytes = (n), .buswidth = (width)}
#define PBL_SPI_MEM_OP_NO_DUMMY {0}
#define PBL_SPI_MEM_OP_DATA_IN(n, p, width) \
  {.dir = PBL_SPI_MEM_DATA_IN, .nbytes = (n), .buf.in = (p), .buswidth = (width)}
#define PBL_SPI_MEM_OP_DATA_OUT(n, p, width) \
  {.dir = PBL_SPI_MEM_DATA_OUT, .nbytes = (n), .buf.out = (p), .buswidth = (width)}
#define PBL_SPI_MEM_OP_NO_DATA {0}

#define PBL_SPI_MEM_OP(c, a, d, dat)             \
  (struct pbl_spi_mem_op) {                      \
    .cmd = c, .addr = a, .dummy = d, .data = dat \
  }

struct pbl_spi_mem_ops {
  int (*init)(const struct pbl_spi_mem_device *dev);
  //! Whether the controller can execute @p op as described.
  bool (*supports_op)(const struct pbl_spi_mem_device *dev, const struct pbl_spi_mem_op *op);
  //! Clamp op->data.nbytes to what one transaction can carry. Optional.
  int (*adjust_op_size)(const struct pbl_spi_mem_device *dev, struct pbl_spi_mem_op *op);
  int (*exec_op)(const struct pbl_spi_mem_device *dev, const struct pbl_spi_mem_op *op);
  //! Memory-mapped read window, when the controller has one. Optional.
  int (*dirmap_read)(const struct pbl_spi_mem_device *dev, uint32_t addr, void *buf, size_t len);
  //! Controller power state, around MCU stop mode. Optional.
  void (*set_power)(const struct pbl_spi_mem_device *dev, bool on);
};

struct pbl_spi_mem_device_state {
  bool initialized;
  //! Set when transactions must not use interrupts or block on the OS.
  bool polling;
};

struct pbl_spi_mem_device {
  struct pbl_spi_mem_device_state *state;
  const struct pbl_spi_mem_ops *ops;
  //! The CPU executes from this memory: ops that leave it busy return only
  //! once it is readable again, and it must not be reset or reconfigured.
  bool xip;
};

//! The bus the board's storage NOR flash sits on.
extern const struct pbl_spi_mem_device *const SPI_MEM_NOR;

int pbl_spi_mem_init(const struct pbl_spi_mem_device *dev);
bool pbl_spi_mem_supports_op(const struct pbl_spi_mem_device *dev, const struct pbl_spi_mem_op *op);
int pbl_spi_mem_adjust_op_size(const struct pbl_spi_mem_device *dev, struct pbl_spi_mem_op *op);
//! Executes @p op. -ENOTSUP if the controller cannot run it.
int pbl_spi_mem_exec_op(const struct pbl_spi_mem_device *dev, const struct pbl_spi_mem_op *op);
bool pbl_spi_mem_has_dirmap(const struct pbl_spi_mem_device *dev);
int pbl_spi_mem_dirmap_read(const struct pbl_spi_mem_device *dev, uint32_t addr, void *buf,
                            size_t len);
void pbl_spi_mem_set_power(const struct pbl_spi_mem_device *dev, bool on);
//! Use polling instead of interrupts from now on (coredump).
void pbl_spi_mem_set_polling(const struct pbl_spi_mem_device *dev, bool polling);

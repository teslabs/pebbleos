/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <pbl/device.h>

/**
 * @defgroup drivers_spi_mem SPI memory
 * @ingroup drivers
 * @brief SPI memory controller class.
 *
 * A SPI memory controller runs operations made of a command, an address, dummy cycles and data
 * against a memory (NOR flash, PSRAM, ...) without knowing its protocol, which belongs to the
 * memory's driver. Each phase has its own bus width. A controller may not support every
 * operation, or may only carry part of the data in one go: drivers check with
 * pbl_spi_mem_supports_op() and split with pbl_spi_mem_adjust_op_size().
 *
 * @code{.c}
 * uint8_t sr;
 * struct pbl_spi_mem_op op = PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(0x05, 1), PBL_SPI_MEM_OP_NO_ADDR,
 *                                           PBL_SPI_MEM_OP_NO_DUMMY,
 *                                           PBL_SPI_MEM_OP_DATA_IN(1, &sr, 1));
 *
 * ret = pbl_spi_mem_exec_op(bus, &op);
 * @endcode
 *
 * Operations are issued by the single memory on the controller, which serializes them; they
 * block until done, polling instead of sleeping once pbl_spi_mem_set_polling() is set.
 * @{
 */

struct pbl_spi_mem;

/** @brief Data direction of an operation. */
enum pbl_spi_mem_data_dir {
  /** No data phase. */
  PBL_SPI_MEM_NO_DATA,
  /** Data read from the memory. */
  PBL_SPI_MEM_DATA_IN,
  /** Data written to the memory. */
  PBL_SPI_MEM_DATA_OUT,
};

/** @brief An operation. */
struct pbl_spi_mem_op {
  /** Command phase. */
  struct {
    /** Opcode. */
    uint8_t opcode;
    /** Bus width. */
    uint8_t buswidth;
  } cmd;
  /** Address phase. */
  struct {
    /** Number of address bytes, 0 for none. */
    uint8_t nbytes;
    /** Bus width. */
    uint8_t buswidth;
    /** Address. */
    uint32_t val;
  } addr;
  /** Dummy phase. */
  struct {
    /** Number of dummy bytes, 0 for none. */
    uint8_t nbytes;
    /** Bus width. */
    uint8_t buswidth;
  } dummy;
  /** Data phase. */
  struct {
    /** Direction. */
    enum pbl_spi_mem_data_dir dir;
    /** Bus width. */
    uint8_t buswidth;
    /** Number of bytes. */
    size_t nbytes;
    /** Buffer. */
    union {
      /** Buffer to read into. */
      void *in;
      /** Data to write. */
      const void *out;
    } buf;
  } data;
};

/** @brief Command phase, for PBL_SPI_MEM_OP(). */
#define PBL_SPI_MEM_OP_CMD(_opcode, _buswidth) {.opcode = (_opcode), .buswidth = (_buswidth)}
/** @brief Address phase, for PBL_SPI_MEM_OP(). */
#define PBL_SPI_MEM_OP_ADDR(_nbytes, _val, _buswidth) \
  {.nbytes = (_nbytes), .buswidth = (_buswidth), .val = (_val)}
/** @brief No address phase. */
#define PBL_SPI_MEM_OP_NO_ADDR \
  {                            \
  }
/** @brief Dummy phase, for PBL_SPI_MEM_OP(). */
#define PBL_SPI_MEM_OP_DUMMY(_nbytes, _buswidth) {.nbytes = (_nbytes), .buswidth = (_buswidth)}
/** @brief No dummy phase. */
#define PBL_SPI_MEM_OP_NO_DUMMY \
  {                             \
  }
/** @brief Data phase reading from the memory, for PBL_SPI_MEM_OP(). */
#define PBL_SPI_MEM_OP_DATA_IN(_nbytes, _buf, _buswidth) \
  {.dir = PBL_SPI_MEM_DATA_IN, .buswidth = (_buswidth), .nbytes = (_nbytes), .buf.in = (_buf)}
/** @brief Data phase writing to the memory, for PBL_SPI_MEM_OP(). */
#define PBL_SPI_MEM_OP_DATA_OUT(_nbytes, _buf, _buswidth) \
  {.dir = PBL_SPI_MEM_DATA_OUT, .buswidth = (_buswidth), .nbytes = (_nbytes), .buf.out = (_buf)}
/** @brief No data phase. */
#define PBL_SPI_MEM_OP_NO_DATA \
  {                            \
  }

/** @brief Build an operation from its phases. */
#define PBL_SPI_MEM_OP(_cmd, _addr, _dummy, _data)              \
  (struct pbl_spi_mem_op) {                                     \
    .cmd = _cmd, .addr = _addr, .dummy = _dummy, .data = _data, \
  }

/** @brief Controller driver operations. */
struct pbl_spi_mem_ops {
  /** Optional. Bring the controller up. Returns 0 or a negative errno. */
  int (*init)(const struct pbl_spi_mem *mem);
  /** Check whether the controller can run @p op. */
  bool (*supports_op)(const struct pbl_spi_mem *mem, const struct pbl_spi_mem_op *op);
  /** Optional. Reduce the data of @p op to what one run carries. Returns 0 or a negative errno. */
  int (*adjust_op_size)(const struct pbl_spi_mem *mem, struct pbl_spi_mem_op *op);
  /** Run a supported operation. Returns 0 or a negative errno. */
  int (*exec_op)(const struct pbl_spi_mem *mem, const struct pbl_spi_mem_op *op);
  /** Optional. Power down, with interrupts disabled. */
  void (*power_down)(const struct pbl_spi_mem *mem);
  /** Optional. Power up, with interrupts disabled. */
  void (*power_up)(const struct pbl_spi_mem *mem);
};

/** @cond INTERNAL_HIDDEN */
struct pbl_spi_mem_state {
  bool polling;
};

extern const struct pbl_device_class pbl_spi_mem_class;
int pbl_spi_mem_dev_init(const struct pbl_device *dev);
/** @endcond */

/** @brief A SPI memory controller. */
struct pbl_spi_mem {
  /** Device. */
  struct pbl_device dev;
  /** Driver operations. */
  const struct pbl_spi_mem_ops *ops;
  /** Runtime state. */
  struct pbl_spi_mem_state *state;
};

/**
 * @brief Define the class state of controller @p sym, for the @c PBL_SPI_MEM_*_DEFINE() macro of
 * a driver.
 *
 * @param sym Symbol of the controller instance.
 */
#define PBL_SPI_MEM_STATE_DEFINE(sym) \
  PBL_DEVICE_STATE_DEFINE(sym);       \
  static struct pbl_spi_mem_state sym##_spi_mem_state

/**
 * @brief Initializer for the struct pbl_spi_mem of controller @p sym.
 *
 * @param sym Symbol of the controller instance, with its state defined by
 * PBL_SPI_MEM_STATE_DEFINE().
 * @param _name Name.
 * @param _deps Dependencies from PBL_DEVICE_DEPS(), or NULL.
 * @param _ops Driver operations.
 */
#define PBL_SPI_MEM_INIT(sym, _name, _deps, _ops)                                  \
  {                                                                                \
    .dev = PBL_DEVICE_INIT_CLASS(sym, _name, pbl_spi_mem_dev_init, nullptr, _deps, \
                                 &pbl_spi_mem_class),                              \
    .ops = (_ops),                                                                 \
    .state = &sym##_spi_mem_state,                                                 \
  }

/**
 * @brief Check whether the controller can run an operation.
 *
 * @param mem Controller.
 * @param op Operation.
 * @return True if pbl_spi_mem_exec_op() accepts it.
 */
bool pbl_spi_mem_supports_op(const struct pbl_spi_mem *mem, const struct pbl_spi_mem_op *op);

/**
 * @brief Reduce the data of an operation to what one run carries.
 *
 * @param mem Controller.
 * @param op Operation, whose @c data.nbytes may be reduced.
 * @return 0 or a negative errno.
 */
int pbl_spi_mem_adjust_op_size(const struct pbl_spi_mem *mem, struct pbl_spi_mem_op *op);

/**
 * @brief Run an operation.
 *
 * @param mem Controller.
 * @param op Operation.
 * @return 0, @c -ENOTSUP if the controller cannot run it, or another negative errno.
 */
int pbl_spi_mem_exec_op(const struct pbl_spi_mem *mem, const struct pbl_spi_mem_op *op);

/**
 * @brief Power the controller down, with interrupts disabled.
 *
 * @param mem Controller.
 */
void pbl_spi_mem_power_down(const struct pbl_spi_mem *mem);

/**
 * @brief Power the controller up, with interrupts disabled.
 *
 * @param mem Controller.
 */
void pbl_spi_mem_power_up(const struct pbl_spi_mem *mem);

/**
 * @brief Poll for completion instead of sleeping, e.g. from a fault handler.
 *
 * @param mem Controller.
 * @param polling True to poll.
 */
void pbl_spi_mem_set_polling(const struct pbl_spi_mem *mem, bool polling);

/**
 * @brief Check whether the controller polls for completion, for drivers.
 *
 * @param mem Controller.
 * @return True if polling.
 */
static inline bool pbl_spi_mem_is_polling(const struct pbl_spi_mem *mem) {
  return mem->state->polling;
}

/** @} */

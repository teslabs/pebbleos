/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <pbl/drivers/spi_mem.h>
#include <pbl/logging/logging.h>
#include <pbl/util/misc.h>

PBL_LOG_MODULE_DEFINE(driver_spi_mem, CONFIG_DRIVER_SPI_MEM_LOG_LEVEL);

PBL_DEVICE_CLASS_DEFINE(pbl_spi_mem_class, "spi_mem");

int pbl_spi_mem_dev_init(const struct pbl_device *dev) {
  const struct pbl_spi_mem *mem = container_of(dev, const struct pbl_spi_mem, dev);

  if (mem->ops->init == nullptr) {
    return 0;
  }

  return mem->ops->init(mem);
}

bool pbl_spi_mem_supports_op(const struct pbl_spi_mem *mem, const struct pbl_spi_mem_op *op) {
  return mem->ops->supports_op(mem, op);
}

int pbl_spi_mem_adjust_op_size(const struct pbl_spi_mem *mem, struct pbl_spi_mem_op *op) {
  if (mem->ops->adjust_op_size == nullptr) {
    return 0;
  }

  return mem->ops->adjust_op_size(mem, op);
}

int pbl_spi_mem_exec_op(const struct pbl_spi_mem *mem, const struct pbl_spi_mem_op *op) {
  if (!mem->ops->supports_op(mem, op)) {
    return -ENOTSUP;
  }

  return mem->ops->exec_op(mem, op);
}

void pbl_spi_mem_power_down(const struct pbl_spi_mem *mem) {
  if (mem->ops->power_down != nullptr) {
    mem->ops->power_down(mem);
  }
}

void pbl_spi_mem_power_up(const struct pbl_spi_mem *mem) {
  if (mem->ops->power_up != nullptr) {
    mem->ops->power_up(mem);
  }
}

void pbl_spi_mem_set_polling(const struct pbl_spi_mem *mem, bool polling) {
  mem->state->polling = polling;
}

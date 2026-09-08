/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/spi_mem.h>

#include <errno.h>

int pbl_spi_mem_init(const struct pbl_spi_mem_device *dev) {
  if (dev->state->initialized) {
    return 0;
  }

  int ret = dev->ops->init(dev);
  if (ret == 0) {
    dev->state->initialized = true;
  }

  return ret;
}

bool pbl_spi_mem_supports_op(const struct pbl_spi_mem_device *dev,
                             const struct pbl_spi_mem_op *op) {
  return dev->ops->supports_op(dev, op);
}

int pbl_spi_mem_adjust_op_size(const struct pbl_spi_mem_device *dev, struct pbl_spi_mem_op *op) {
  if (dev->ops->adjust_op_size == NULL) {
    return 0;
  }
  return dev->ops->adjust_op_size(dev, op);
}

int pbl_spi_mem_exec_op(const struct pbl_spi_mem_device *dev, const struct pbl_spi_mem_op *op) {
  if (!dev->ops->supports_op(dev, op)) {
    return -ENOTSUP;
  }
  return dev->ops->exec_op(dev, op);
}

bool pbl_spi_mem_has_dirmap(const struct pbl_spi_mem_device *dev) {
  return dev->ops->dirmap_read != NULL;
}

int pbl_spi_mem_dirmap_read(const struct pbl_spi_mem_device *dev, uint32_t addr, void *buf,
                            size_t len) {
  if (dev->ops->dirmap_read == NULL) {
    return -ENOTSUP;
  }
  return dev->ops->dirmap_read(dev, addr, buf, len);
}

void pbl_spi_mem_set_power(const struct pbl_spi_mem_device *dev, bool on) {
  if (dev->ops->set_power != NULL) {
    dev->ops->set_power(dev, on);
  }
}

void pbl_spi_mem_set_polling(const struct pbl_spi_mem_device *dev, bool polling) {
  dev->state->polling = polling;
}

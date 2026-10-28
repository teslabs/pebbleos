/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <inttypes.h>
#include <string.h>

#include <pbl/drivers/flash/sf32lb52_mpi.h>
#include <pbl/kernel/irq.h>
#include <pbl/logging/logging.h>
#include <pbl/mcu/cache.h>
#include <pbl/util/math.h>
#include <pbl/util/misc.h>

#include <kernel/pbl_malloc.h>

PBL_LOG_MODULE_DECLARE(driver_flash, CONFIG_DRIVER_FLASH_LOG_LEVEL);

// Bits 15-12 of a security register address are its one-based index
#define SEC_REG_IDX(addr) (((addr) >> 12U) - 1U)

// OTP accesses are word sized and aligned
#define OTP_WORD_SIZE  4U
#define OTP_LB_INVALID 0xFFU

static const struct pbl_flash_sf32lb52_mpi *prv_mpi(const struct pbl_flash *flash) {
  return container_of(flash, const struct pbl_flash_sf32lb52_mpi, flash);
}

static FLASH_HandleTypeDef *prv_handle(const struct pbl_flash *flash) {
  return &prv_mpi(flash)->data->ctx.handle;
}

static void prv_invalidate(uint32_t addr, size_t len) {
  SCB_InvalidateDCache_by_Addr((void *)addr, len);
  SCB_InvalidateICache_by_Addr((void *)addr, len);
}

static int prv_init(const struct pbl_flash *flash) {
  const struct pbl_flash_sf32lb52_mpi *mpi = prv_mpi(flash);
  struct pbl_flash_sf32lb52_mpi_data *data = mpi->data;
  HAL_StatusTypeDef res;

  if (!data->initialized) {
    data->ctx.dual_mode = 1;
    res = HAL_FLASH_Init(&data->ctx, &data->cfg, &data->hdma, &data->dma, mpi->clk_div);
    if (res != HAL_OK) {
      return -EIO;
    }

    if (data->ctx.dev_id != mpi->part->id) {
      PBL_LOG_ERR("Flash is not %s (id 0x%06" PRIx32 ")", mpi->part->name, data->ctx.dev_id);
    }

    data->dpd_enter_us = mpi->part->dpd_enter_us;
    data->dpd_exit_us = mpi->part->dpd_exit_us;
    data->initialized = true;
  }

  // DMA completion needs interrupts, which a coredump cannot rely on
  data->ctx.handle.dma = pbl_flash_in_coredump(flash) ? nullptr : &data->hdma;

  return 0;
}

static int prv_read(const struct pbl_flash *flash, uint32_t addr, void *buf, size_t len) {
  memcpy(buf, (const void *)addr, len);

  return 0;
}

static int prv_write_locked(const struct pbl_flash *flash, uint32_t addr, const uint8_t *src,
                            size_t len) {
  FLASH_HandleTypeDef *hflash = prv_handle(flash);
  const uint32_t page_size = flash->geometry->page_size;
  uint32_t offset = addr - flash->base;

  while (len > 0U) {
    size_t chunk = MIN(len, page_size - (offset % page_size));
    int res;

    pbl_irq_lock();
    res = HAL_QSPIEX_WRITE_PAGE(hflash, offset, (uint8_t *)src, chunk);
    pbl_irq_unlock();
    if (res != (int)chunk) {
      return -EIO;
    }

    offset += chunk;
    src += chunk;
    len -= chunk;
  }

  return 0;
}

static int prv_write(const struct pbl_flash *flash, uint32_t addr, const void *buf, size_t len) {
  const uint8_t *src = buf;
  uint8_t *bounce = nullptr;
  uintptr_t flush_addr;
  size_t flush_size;
  int ret;

  // The DMA cannot read from the flash being programmed or from TCM, nor cross a 1 MiB boundary
  if (IS_SAME_FLASH_ADDR(src, addr) || IS_SPI_NONDMA_RAM_ADDR(src) ||
      IS_DMA_ACCROSS_1M_BOUNDARY((uint32_t)src, len)) {
    bounce = kernel_malloc_check(len);
    memcpy(bounce, src, len);
    src = bounce;
  }

  // The DMA bypasses the D-cache
  flush_addr = (uintptr_t)src;
  flush_size = len;
  dcache_align(&flush_addr, &flush_size);
  dcache_flush((const void *)flush_addr, flush_size);

  ret = prv_write_locked(flash, addr, src, len);

  prv_invalidate(addr, len);

  if (bounce != nullptr) {
    kernel_free(bounce);
  }

  return ret;
}

static int prv_erase(const struct pbl_flash *flash, uint32_t addr, uint32_t size) {
  FLASH_HandleTypeDef *hflash = prv_handle(flash);
  uint32_t offset = addr - flash->base;
  int res;

  pbl_irq_lock();
  if (size == flash->geometry->sector_size) {
    res = HAL_QSPIEX_BLK64_ERASE(hflash, offset);
  } else if (size == flash->geometry->subsector_size) {
    res = HAL_QSPIEX_SECT_ERASE(hflash, offset);
  } else {
    res = -1;
  }
  pbl_irq_unlock();

  prv_invalidate(addr, size);

  return (res == 0) ? 0 : -EIO;
}

static void prv_power_down(const struct pbl_flash *flash) {
  HAL_FLASH_NOP_CMD(prv_handle(flash));
}

static int prv_sec_reg_check(const struct pbl_flash *flash, uint32_t addr) {
  const struct pbl_flash_sec_regs *regs = flash->sec_regs;

  for (uint8_t i = 0U; i < regs->count; i++) {
    if (addr >= regs->addrs[i] && addr < regs->addrs[i] + regs->size) {
      return 0;
    }
  }

  return -EINVAL;
}

static int prv_sec_reg_read(const struct pbl_flash *flash, uint32_t addr, uint8_t *val) {
  uint8_t word[OTP_WORD_SIZE] = {};
  uint32_t offset = addr % OTP_WORD_SIZE;
  int ret;
  int res;

  ret = prv_sec_reg_check(flash, addr);
  if (ret != 0) {
    return ret;
  }

  pbl_irq_lock();
  res = HAL_QSPI_READ_OTP(prv_handle(flash), addr - offset, word, sizeof(word));
  pbl_irq_unlock();
  if (res != (int)sizeof(word)) {
    return -EIO;
  }

  *val = word[offset];

  return 0;
}

static int prv_sec_reg_write(const struct pbl_flash *flash, uint32_t addr, uint8_t val) {
  uintptr_t flush_addr = (uintptr_t)&val;
  size_t flush_size = sizeof(val);
  int ret;
  int res;

  ret = prv_sec_reg_check(flash, addr);
  if (ret != 0) {
    return ret;
  }

  dcache_align(&flush_addr, &flush_size);
  dcache_flush((const void *)flush_addr, flush_size);

  pbl_irq_lock();
  res = HAL_QSPI_WRITE_OTP(prv_handle(flash), addr, &val, sizeof(val));
  pbl_irq_unlock();

  return (res == (int)sizeof(val)) ? 0 : -EIO;
}

static int prv_sec_reg_erase(const struct pbl_flash *flash, uint32_t addr) {
  int ret;
  int res;

  ret = prv_sec_reg_check(flash, addr);
  if (ret != 0) {
    return ret;
  }

  pbl_irq_lock();
  res = HAL_QSPI_ERASE_OTP(prv_handle(flash), addr);
  pbl_irq_unlock();

  return (res == 0) ? 0 : -EIO;
}

static int prv_sec_reg_is_locked(const struct pbl_flash *flash, uint32_t addr, bool *locked) {
  uint8_t lb;
  int ret;

  ret = prv_sec_reg_check(flash, addr);
  if (ret != 0) {
    return ret;
  }

  pbl_irq_lock();
  lb = HAL_QSPI_GET_OTP_LB(prv_handle(flash));
  pbl_irq_unlock();
  if (lb == OTP_LB_INVALID) {
    return -EIO;
  }

  *locked = (lb & PBL_BIT(SEC_REG_IDX(addr))) != 0U;

  return 0;
}

#ifdef CONFIG_RECOVERY_FW
static int prv_sec_reg_lock(const struct pbl_flash *flash, uint32_t addr) {
  int ret;
  int res;

  ret = prv_sec_reg_check(flash, addr);
  if (ret != 0) {
    return ret;
  }

  pbl_irq_lock();
  res = HAL_QSPI_LOCK_OTP(prv_handle(flash), addr);
  pbl_irq_unlock();

  return (res == 0) ? 0 : -EIO;
}
#endif

const struct pbl_flash_ops pbl_flash_sf32lb52_mpi_ops = {
  .init = prv_init,
  .read = prv_read,
  .write = prv_write,
  .erase = prv_erase,
  .power_down = prv_power_down,
  .sec_reg_read = prv_sec_reg_read,
  .sec_reg_write = prv_sec_reg_write,
  .sec_reg_erase = prv_sec_reg_erase,
  .sec_reg_is_locked = prv_sec_reg_is_locked,
#ifdef CONFIG_RECOVERY_FW
  .sec_reg_lock = prv_sec_reg_lock,
#endif
};

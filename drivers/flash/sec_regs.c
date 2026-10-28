/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <pbl/drivers/flash.h>

static void prv_lock(const struct pbl_flash *flash) {
  if (!flash->state->coredump) {
    pbl_mutex_lock(&flash->state->lock, PBL_FOREVER);
  }
}

static void prv_unlock(const struct pbl_flash *flash) {
  if (!flash->state->coredump) {
    pbl_mutex_unlock(&flash->state->lock);
  }
}

static int prv_check(const struct pbl_flash *flash) {
  if (flash->sec_regs == nullptr) {
    return -ENOTSUP;
  }

  if (!flash->state->coredump && !pbl_device_is_ready(&flash->dev)) {
    return -ENODEV;
  }

  return 0;
}

int pbl_flash_sec_reg_read(const struct pbl_flash *flash, uint32_t addr, uint8_t *val) {
  int ret;

  ret = prv_check(flash);
  if (ret != 0) {
    return ret;
  }

  prv_lock(flash);
  ret = flash->ops->sec_reg_read(flash, addr, val);
  prv_unlock(flash);

  return ret;
}

int pbl_flash_sec_reg_write(const struct pbl_flash *flash, uint32_t addr, uint8_t val) {
  int ret;

  ret = prv_check(flash);
  if (ret != 0) {
    return ret;
  }

  prv_lock(flash);
  ret = flash->ops->sec_reg_write(flash, addr, val);
  prv_unlock(flash);

  return ret;
}

int pbl_flash_sec_reg_erase(const struct pbl_flash *flash, uint32_t addr) {
  int ret;

  ret = prv_check(flash);
  if (ret != 0) {
    return ret;
  }

  prv_lock(flash);
  ret = flash->ops->sec_reg_erase(flash, addr);
  prv_unlock(flash);

  return ret;
}

int pbl_flash_sec_reg_is_locked(const struct pbl_flash *flash, uint32_t addr, bool *locked) {
  int ret;

  ret = prv_check(flash);
  if (ret != 0) {
    return ret;
  }

  prv_lock(flash);
  ret = flash->ops->sec_reg_is_locked(flash, addr, locked);
  prv_unlock(flash);

  return ret;
}

int pbl_flash_sec_reg_lock(const struct pbl_flash *flash, uint32_t addr) {
  int ret;

  ret = prv_check(flash);
  if (ret != 0) {
    return ret;
  }

  if (flash->ops->sec_reg_lock == nullptr) {
    return -ENOTSUP;
  }

  prv_lock(flash);
  ret = flash->ops->sec_reg_lock(flash, addr);
  prv_unlock(flash);

  return ret;
}

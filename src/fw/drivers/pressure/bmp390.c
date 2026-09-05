/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include "board/board.h"
#include <pbl/drivers/pressure.h>
#include <pbl/drivers/pressure/bmp390.h>
#include <pbl/drivers/i2c.h>
#include <pbl/logging/logging.h>

PBL_LOG_MODULE_DEFINE(driver_pressure_bmp390, CONFIG_DRIVER_PRESSURE_LOG_LEVEL);

#define BMP390_CHIP_ID 0x00
#define BMP390_CHIP_ID_VALUE 0x60
#define BMP390_PWR_CTRL 0x1B


static bool prv_read_register(const struct pbl_i2c_dev *i2c, uint8_t register_address, uint8_t *result) {
  pbl_i2c_use(i2c);
  bool rv = pbl_i2c_write_block(i2c, 1, &register_address);
  if (rv)
    rv = pbl_i2c_read_block(i2c, 1, result);
  pbl_i2c_release(i2c);
  return rv;
}

static bool prv_write_register(const struct pbl_i2c_dev *i2c, uint8_t register_address, uint8_t datum) {
  pbl_i2c_use(i2c);
  uint8_t d[2] = { register_address, datum };
  bool rv = pbl_i2c_write_block(i2c, 2, d);
  pbl_i2c_release(i2c);
  return rv;
}

int bmp390_init(const struct pbl_device *dev) {
  const struct pbl_bmp390 *bmp = PBL_CONTAINER_OF(dev, const struct pbl_bmp390, dev);
  bool rv;
  uint8_t result;

  rv = prv_read_register(&bmp->i2c, BMP390_CHIP_ID, &result);
  if (!rv || result != BMP390_CHIP_ID_VALUE) {
    PBL_LOG_DBG("BMP390 probe failed; rv %d, result 0x%02x", rv, result);
    return -ENODEV;
  }

  PBL_LOG_DBG("found the BMP390, setting to low power");
  (void) prv_write_register(&bmp->i2c, BMP390_PWR_CTRL, 0);
  return 0;
}

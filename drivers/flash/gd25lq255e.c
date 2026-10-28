/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/flash/nor.h>
#include <pbl/util/size.h>

static const uint32_t s_sec_regs[] = {
  0x2000U,
  0x3000U,
};

const struct pbl_flash_nor_part pbl_flash_nor_gd25lq255e = {
  .name = "GD25LQ255E",
  .id = 0x1960c8U,
  .geometry =
      {
        .size = 32U * 1024U * 1024U,
        .page_size = 256U,
        .sector_size = 64U * 1024U,
        .subsector_size = 4U * 1024U,
        .sector_erase_ms = 150U,
        .subsector_erase_ms = 50U,
      },
  .sec_regs =
      {
        .addrs = s_sec_regs,
        .count = ARRAY_LENGTH(s_sec_regs),
        .size = 1024U,
      },
  .qe = PBL_FLASH_NOR_QE_SR2_BIT1_WR_SR1,
  .reset_ms = 12U,
  .suspend_us = 20U,
  .dpd_enter_us = 3U,
  .dpd_exit_us = 20U,
};

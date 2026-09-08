/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include <pbl/drivers/flash.h>

struct pbl_flash_qemu {
  struct pbl_flash_device dev;
  //! Controller registers of the QEMU pebble-extflash device.
  uintptr_t regs;
};

extern const struct pbl_flash_ops pbl_flash_qemu_ops;

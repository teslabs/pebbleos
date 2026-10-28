/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <pbl/drivers/flash/spi_nor.h>
#include <pbl/util/size.h>

#include <clar.h>
#include <fake_new_timer.h>

// Fake JEDEC NOR chip behind a fake controller
///////////////////////////////////////////////////////////

#define CHIP_SIZE      (32U * 1024U * 1024U)
#define CHIP_ID        0x1960c8U
#define SFDP_BFPT_ADDR 0x30U
#define MAX_RUN        64U
#define SEC_REG_SIZE   1024U

static uint8_t *s_mem;
static uint8_t s_sfdp[0x100];
static bool s_has_sfdp;
static uint8_t s_sr1;
static uint8_t s_sr2;
static bool s_wel;
static bool s_4byte;
static int s_wrsr_count;
static bool s_quad_controller;
static uint8_t s_sec[3][SEC_REG_SIZE];

static uint32_t prv_sec_idx(uint32_t addr) {
  return (addr >> 12) - 1U;
}

static bool prv_supports_op(const struct pbl_spi_mem *mem, const struct pbl_spi_mem_op *op) {
  if (op->data.buswidth == 4U || op->addr.buswidth == 4U) {
    return s_quad_controller;
  }

  return op->data.buswidth <= 1U && op->addr.buswidth <= 1U;
}

static int prv_adjust_op_size(const struct pbl_spi_mem *mem, struct pbl_spi_mem_op *op) {
  op->data.nbytes = MIN(op->data.nbytes, MAX_RUN);
  return 0;
}

static int prv_exec_op(const struct pbl_spi_mem *mem, const struct pbl_spi_mem_op *op) {
  uint8_t *in = op->data.buf.in;
  const uint8_t *out = op->data.buf.out;
  uint32_t addr = op->addr.val;
  size_t n = op->data.nbytes;

  if (op->addr.nbytes != 0U && op->cmd.opcode != PBL_FLASH_NOR_CMD_RDSFDP) {
    cl_assert_equal_i(op->addr.nbytes, s_4byte ? 4 : 3);
  }

  switch (op->cmd.opcode) {
    case PBL_FLASH_NOR_CMD_RSTEN:
    case PBL_FLASH_NOR_CMD_RST:
    case PBL_FLASH_NOR_CMD_SUSPEND:
    case PBL_FLASH_NOR_CMD_RESUME:
    case PBL_FLASH_NOR_CMD_DPD:
    case PBL_FLASH_NOR_CMD_RDPD:
      return 0;
    case PBL_FLASH_NOR_CMD_WREN:
      s_wel = true;
      return 0;
    case PBL_FLASH_NOR_CMD_EN4B:
      s_4byte = true;
      return 0;
    case PBL_FLASH_NOR_CMD_RDID:
      in[0] = (uint8_t)CHIP_ID;
      in[1] = (uint8_t)(CHIP_ID >> 8);
      in[2] = (uint8_t)(CHIP_ID >> 16);
      return 0;
    case PBL_FLASH_NOR_CMD_RDSFDP:
      if (s_has_sfdp) {
        memcpy(in, &s_sfdp[addr], n);
      } else {
        memset(in, 0xFF, n);
      }
      return 0;
    case PBL_FLASH_NOR_CMD_RDSR:
      in[0] = s_sr1;
      return 0;
    case PBL_FLASH_NOR_CMD_RDSR2:
      in[0] = s_sr2;
      return 0;
    case PBL_FLASH_NOR_CMD_WRSR:
      cl_assert(s_wel);
      s_wel = false;
      s_wrsr_count++;
      s_sr1 = out[0];
      if (n > 1U) {
        s_sr2 = out[1];
      }
      return 0;
    case PBL_FLASH_NOR_CMD_FAST_READ:
    case 0xEBU:
    case 0x6BU:
      memcpy(in, &s_mem[addr], n);
      return 0;
    case PBL_FLASH_NOR_CMD_PP:
    case PBL_FLASH_NOR_CMD_PP_1_1_4:
      cl_assert(s_wel);
      s_wel = false;
      for (size_t i = 0U; i < n; i++) {
        s_mem[addr + i] &= out[i];
      }
      return 0;
    case PBL_FLASH_NOR_CMD_SE:
      cl_assert(s_wel);
      s_wel = false;
      memset(&s_mem[addr & ~0xFFFU], 0xFF, 0x1000U);
      return 0;
    case PBL_FLASH_NOR_CMD_BE:
      cl_assert(s_wel);
      s_wel = false;
      memset(&s_mem[addr & ~0xFFFFU], 0xFF, 0x10000U);
      return 0;
    case PBL_FLASH_NOR_CMD_RDSCUR:
      in[0] = s_sec[prv_sec_idx(addr)][addr & (SEC_REG_SIZE - 1U)];
      return 0;
    case PBL_FLASH_NOR_CMD_PRSCUR:
      cl_assert(s_wel);
      s_wel = false;
      s_sec[prv_sec_idx(addr)][addr & (SEC_REG_SIZE - 1U)] &= out[0];
      return 0;
    case PBL_FLASH_NOR_CMD_ERSCUR:
      cl_assert(s_wel);
      s_wel = false;
      memset(s_sec[prv_sec_idx(addr)], 0xFF, SEC_REG_SIZE);
      return 0;
    default:
      return -ENOTSUP;
  }
}

static const struct pbl_spi_mem_ops s_bus_ops = {
  .supports_op = prv_supports_op,
  .adjust_op_size = prv_adjust_op_size,
  .exec_op = prv_exec_op,
};

PBL_SPI_MEM_STATE_DEFINE(s_bus);
static const struct pbl_spi_mem s_bus = PBL_SPI_MEM_INIT(s_bus, "bus", nullptr, &s_bus_ops);
PBL_DEVICE_REGISTER(s_bus, &s_bus.dev);

PBL_FLASH_SPI_NOR_DEFINE(s_nor, "nor", &s_bus, &pbl_flash_nor_gd25lq255e);

static void prv_put32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

// 32 MiB, 4/32/64 KiB erases, 1-1-2, 1-2-2, 1-1-4 and 1-4-4 reads, 256 byte pages, QE at SR2[1]
static void prv_build_sfdp(void) {
  uint32_t bfpt[16] = {};

  memset(s_sfdp, 0xFF, sizeof(s_sfdp));
  prv_put32(&s_sfdp[0], 0x50444653U);
  s_sfdp[4] = 6U;
  s_sfdp[5] = 1U;
  s_sfdp[6] = 0U;
  s_sfdp[7] = 0xFFU;
  s_sfdp[8] = 0x00U;
  s_sfdp[9] = 6U;
  s_sfdp[10] = 1U;
  s_sfdp[11] = ARRAY_LENGTH(bfpt);
  s_sfdp[12] = SFDP_BFPT_ADDR;
  s_sfdp[13] = 0U;
  s_sfdp[14] = 0U;
  s_sfdp[15] = 0xFFU;

  bfpt[0] = 0x01U | (0x20U << 8) | (1U << 16) | (1U << 17) | (1U << 20) | (1U << 21) | (1U << 22);
  bfpt[1] = (CHIP_SIZE * 8U) - 1U;
  bfpt[2] = (0xEBU << 8) | (2U << 5) | 4U | ((0x6BU << 8) | 8U) << 16;
  bfpt[3] = (0x3BU << 8) | 8U | ((0xBBU << 8) | 4U) << 16;
  bfpt[7] = 12U | (0x20U << 8) | ((15U | (0x52U << 8)) << 16);
  bfpt[8] = 16U | (0xD8U << 8);
  // Typical erase times: 4 KiB 45 ms, 32 KiB 120 ms, 64 KiB 150 ms
  bfpt[9] = ((((1U << 5) | 2U) << 4) | (((1U << 5) | 7U) << 11) | (((1U << 5) | 9U) << 18));
  bfpt[10] = 8U << 4;
  bfpt[14] = 1U << 20;

  for (size_t i = 0U; i < ARRAY_LENGTH(bfpt); i++) {
    prv_put32(&s_sfdp[SFDP_BFPT_ADDR + (i * 4U)], bfpt[i]);
  }
}

static void prv_reset(bool sfdp, bool quad) {
  s_bus_device_state = (struct pbl_device_state){};
  s_bus_spi_mem_state = (struct pbl_spi_mem_state){};
  s_nor_device_state = (struct pbl_device_state){};
  s_nor_flash_state = (struct pbl_flash_state){};
  s_nor_data = (struct pbl_flash_spi_nor_data){};

  memset(s_mem, 0xFF, CHIP_SIZE);
  memset(s_sec, 0xFF, sizeof(s_sec));
  s_has_sfdp = sfdp;
  s_quad_controller = quad;
  s_sr1 = 0U;
  s_sr2 = 0U;
  s_wel = false;
  s_4byte = false;
  s_wrsr_count = 0;
  prv_build_sfdp();
}

void test_spi_nor__initialize(void) {
  s_mem = malloc(CHIP_SIZE);
  cl_assert(s_mem != nullptr);
  prv_reset(true, true);
}

void test_spi_nor__cleanup(void) {
  free(s_mem);
  stub_new_timer_cleanup();
}

void test_spi_nor__sfdp_geometry(void) {
  cl_assert_equal_i(pbl_device_init(&s_nor.flash.dev), 0);

  cl_assert_equal_i(s_nor_data.geometry.size, CHIP_SIZE);
  cl_assert_equal_i(s_nor_data.geometry.page_size, 256);
  cl_assert_equal_i(s_nor_data.geometry.subsector_size, 0x1000);
  cl_assert_equal_i(s_nor_data.geometry.sector_size, 0x10000);
  cl_assert_equal_i(s_nor_data.subsector_opcode, 0x20);
  cl_assert_equal_i(s_nor_data.sector_opcode, 0xD8);
  cl_assert_equal_i(s_nor_data.geometry.subsector_erase_ms, 3 * 16);
  cl_assert_equal_i(s_nor_data.geometry.sector_erase_ms, 10 * 16);
  cl_assert_equal_i(s_nor_data.addr_nbytes, 4);
  cl_assert(s_4byte);
}

void test_spi_nor__quad_enabled_once(void) {
  cl_assert_equal_i(pbl_device_init(&s_nor.flash.dev), 0);

  cl_assert_equal_i(s_nor_data.read_opcode, 0xEB);
  cl_assert_equal_i(s_nor_data.read_data_width, 4);
  cl_assert_equal_i(s_nor_data.pp_opcode, PBL_FLASH_NOR_CMD_PP_1_1_4);
  cl_assert(s_sr2 & PBL_FLASH_NOR_SR2_QE);
  cl_assert_equal_i(s_wrsr_count, 1);
}

void test_spi_nor__quad_already_enabled(void) {
  s_sr2 = PBL_FLASH_NOR_SR2_QE;
  cl_assert_equal_i(pbl_device_init(&s_nor.flash.dev), 0);
  cl_assert_equal_i(s_wrsr_count, 0);
}

void test_spi_nor__single_line_controller(void) {
  prv_reset(true, false);
  cl_assert_equal_i(pbl_device_init(&s_nor.flash.dev), 0);

  cl_assert_equal_i(s_nor_data.read_opcode, PBL_FLASH_NOR_CMD_FAST_READ);
  cl_assert_equal_i(s_nor_data.pp_opcode, PBL_FLASH_NOR_CMD_PP);
  cl_assert_equal_i(s_wrsr_count, 0);
}

void test_spi_nor__part_fallback_without_sfdp(void) {
  prv_reset(false, false);
  cl_assert_equal_i(pbl_device_init(&s_nor.flash.dev), 0);

  cl_assert_equal_i(s_nor_data.geometry.size, pbl_flash_nor_gd25lq255e.geometry.size);
  cl_assert_equal_i(s_nor_data.geometry.sector_size, 0x10000);
  cl_assert_equal_i(s_nor_data.subsector_opcode, PBL_FLASH_NOR_CMD_SE);
}

void test_spi_nor__read_write_erase(void) {
  const struct pbl_flash *flash = &s_nor.flash;
  uint8_t data[300];
  uint8_t back[sizeof(data)];

  cl_assert_equal_i(pbl_device_init(&flash->dev), 0);

  for (size_t i = 0U; i < sizeof(data); i++) {
    data[i] = (uint8_t)(i * 7U);
  }

  // Crosses a page boundary
  cl_assert_equal_i(pbl_flash_write(flash, 0x1F0U, data, sizeof(data)), 0);
  cl_assert_equal_i(pbl_flash_read(flash, 0x1F0U, back, sizeof(back)), 0);
  cl_assert_equal_m(back, data, sizeof(data));

  cl_assert_equal_i(pbl_flash_erase(flash, 0x0U, 0x1000U), 0);
  cl_assert(pbl_flash_is_erased(flash, 0x0U, 0x1000U));
}

void test_spi_nor__security_registers(void) {
  const struct pbl_flash *flash = &s_nor.flash;
  uint8_t val;
  bool locked;

  cl_assert_equal_i(pbl_device_init(&flash->dev), 0);

  cl_assert_equal_i(pbl_flash_sec_reg_write(flash, 0x2005U, 0x5AU), 0);
  cl_assert_equal_i(pbl_flash_sec_reg_read(flash, 0x2005U, &val), 0);
  cl_assert_equal_i(val, 0x5A);
  cl_assert_equal_i(pbl_flash_sec_reg_erase(flash, 0x2000U), 0);
  cl_assert_equal_i(pbl_flash_sec_reg_read(flash, 0x2005U, &val), 0);
  cl_assert_equal_i(val, 0xFF);

  cl_assert_equal_i(pbl_flash_sec_reg_read(flash, 0x5000U, &val), -EINVAL);

  s_sr2 = PBL_BIT(PBL_FLASH_NOR_SR2_LB_SHIFT + 1U);
  cl_assert_equal_i(pbl_flash_sec_reg_is_locked(flash, 0x2000U, &locked), 0);
  cl_assert(locked);
  cl_assert_equal_i(pbl_flash_sec_reg_is_locked(flash, 0x3000U, &locked), 0);
  cl_assert(!locked);
}

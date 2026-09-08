/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"
#include "fake_new_timer.h"
#include "stubs_analytics.h"
#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_pebble_tasks.h"
#include "stubs_sem.h"
#include "stubs_sleep.h"
#include "stubs_task_watchdog.h"

#include <pbl/drivers/flash.h>
#include <pbl/drivers/flash/nor_part.h>
#include <pbl/drivers/spi_mem.h>

#include <errno.h>
#include <string.h>

void watchdog_feed(void) {}
void delay_us(uint32_t us) {}
void spi_nor_reset_for_test(void);

// Fake JEDEC NOR chip behind a fake spi_mem bus
///////////////////////////////////////////////////////////

#define CHIP_SIZE 0x40000
#define CHIP_PAGE 256
#define CHIP_ID 0x1960c8

// A controller with the nRF52 QSPI limits: single-line commands carry at
// most 8 bytes of address + dummy + data.
#define CINSTR_MAX 8

static uint8_t s_mem[CHIP_SIZE];
static uint8_t s_otp[3][1024];
static uint8_t s_sr1;
static uint8_t s_sr2;
static bool s_4byte;
static bool s_powered_down;
static bool s_reset_enabled;
static int s_resets;
static int s_erase_polls_left;
static bool s_erase_suspended;
static bool s_sfdp_available;
static bool s_limit_cinstr;
static int s_exec_count;
static uint8_t s_last_read_opcode;
static uint8_t s_last_pp_opcode;
static uint8_t s_last_addr_nbytes;
static int s_power_calls;

#define SR1_WIP 0x01
#define SR1_WEL 0x02
#define SR2_QE 0x02
#define SR2_SUS 0x80

// SFDP: header + one parameter header + a 16 DWORD BFPT
static uint8_t s_sfdp[0x30 + 16 * 4];

static void prv_put32(uint8_t *p, uint32_t v) {
  p[0] = v;
  p[1] = v >> 8;
  p[2] = v >> 16;
  p[3] = v >> 24;
}

static void prv_build_sfdp(void) {
  memset(s_sfdp, 0xff, sizeof(s_sfdp));
  // SFDP header: signature, minor 6, major 1, NPH 0 (one header)
  prv_put32(&s_sfdp[0], 0x50444653);
  s_sfdp[4] = 6;
  s_sfdp[5] = 1;
  s_sfdp[6] = 0;
  s_sfdp[7] = 0xff;
  // Parameter header: BFPT id 0x00/0xFF, rev 1.6, 16 DWORDs at 0x30
  s_sfdp[8] = 0x00;
  s_sfdp[9] = 6;
  s_sfdp[10] = 1;
  s_sfdp[11] = 16;
  s_sfdp[12] = 0x30;
  s_sfdp[13] = 0;
  s_sfdp[14] = 0;
  s_sfdp[15] = 0xff;

  uint32_t dw[16] = {0};
  // DW1: 4K erase supported with 0x20, 3-or-4 byte addressing, 1-1-2, 1-2-2,
  // 1-4-4 and 1-1-4 fast reads
  dw[0] = 0x1 | (0x20 << 8) | (1UL << 17) | (1UL << 16) | (1UL << 20) | (1UL << 21) | (1UL << 22);
  // DW2: density in bits, N+1 encoding
  dw[1] = (CHIP_SIZE * 8UL) - 1;
  // DW3: 1-4-4 = EB with 4 wait states + 2 mode clocks; 1-1-4 = 6B with 8
  dw[2] = (4 | (2 << 5) | (0xEB << 8)) | ((8 | (0x6B << 8)) << 16);
  // DW4: 1-1-2 = 3B with 8; 1-2-2 = BB with 4
  dw[3] = (8 | (0x3B << 8)) | ((4 | (0xBB << 8)) << 16);
  // DW8/9: erase types 4K/0x20, 32K/0x52, 64K/0xD8
  dw[7] = (12 | (0x20 << 8)) | ((15 | (0x52 << 8)) << 16);
  dw[8] = (16 | (0xD8 << 8));
  // DW10: typical times: 4K = 3*16ms, 32K = 5*16ms, 64K = 1*128ms
  dw[9] = ((2 | (1 << 5)) << 4) | ((4 | (1 << 5)) << 11) | ((0 | (2 << 5)) << 18);
  // DW11: page size 2^8
  dw[10] = (8 << 4);
  // DW15: QER = S2B1v1
  dw[14] = (1UL << 20);
  // DW16: enter 4-byte with B7, soft reset 66/99
  dw[15] = (1UL << 24) | (1UL << 12);
  for (int i = 0; i < 16; i++) {
    prv_put32(&s_sfdp[0x30 + i * 4], dw[i]);
  }
}

static uint32_t prv_addr(const struct pbl_spi_mem_op *op) {
  return op->addr.val;
}

static int prv_fake_exec_op(const struct pbl_spi_mem_device *dev, const struct pbl_spi_mem_op *op) {
  uint8_t *in = op->data.buf.in;
  const uint8_t *out = op->data.buf.out;
  size_t n = op->data.nbytes;

  s_exec_count++;

  if (s_powered_down && op->cmd.opcode != 0xAB) {
    return -EIO;
  }

  switch (op->cmd.opcode) {
    case 0x06:
      s_sr1 |= SR1_WEL;
      return 0;
    case 0x04:
      s_sr1 &= ~SR1_WEL;
      return 0;
    case 0x05:
      // Erases complete after a few status polls
      if ((s_sr1 & SR1_WIP) && !s_erase_suspended && --s_erase_polls_left <= 0) {
        s_sr1 &= ~SR1_WIP;
      }
      in[0] = s_sr1;
      return 0;
    case 0x35:
      in[0] = s_sr2;
      return 0;
    case 0x01:
      cl_assert(s_sr1 & SR1_WEL);
      s_sr1 = (s_sr1 & (SR1_WIP | SR1_WEL)) | (out[0] & ~(SR1_WIP | SR1_WEL));
      if (n >= 2) {
        s_sr2 = (s_sr2 & SR2_SUS) | (out[1] & ~SR2_SUS);
      }
      s_sr1 &= ~SR1_WEL;
      return 0;
    case 0x9F:
      cl_assert_equal_i(n, 3);
      in[0] = CHIP_ID & 0xff;
      in[1] = (CHIP_ID >> 8) & 0xff;
      in[2] = (CHIP_ID >> 16) & 0xff;
      return 0;
    case 0x5A: {
      cl_assert_equal_i(op->addr.nbytes, 3);
      cl_assert_equal_i(op->dummy.nbytes, 1);
      uint32_t a = prv_addr(op);
      for (size_t i = 0; i < n; i++) {
        in[i] = (s_sfdp_available && a + i < sizeof(s_sfdp)) ? s_sfdp[a + i] : 0xff;
      }
      return 0;
    }
    case 0x0B:
    case 0x3B:
    case 0xBB:
    case 0x6B:
    case 0xEB: {
      cl_assert_equal_i(op->addr.nbytes, s_4byte ? 4 : 3);
      s_last_read_opcode = op->cmd.opcode;
      s_last_addr_nbytes = op->addr.nbytes;
      uint32_t a = prv_addr(op);
      cl_assert(a + n <= CHIP_SIZE);
      memcpy(in, &s_mem[a], n);
      return 0;
    }
    case 0x02:
    case 0x32: {
      cl_assert(s_sr1 & SR1_WEL);
      cl_assert_equal_i(op->addr.nbytes, s_4byte ? 4 : 3);
      s_last_pp_opcode = op->cmd.opcode;
      uint32_t a = prv_addr(op);
      cl_assert(a + n <= CHIP_SIZE);
      // Programming never crosses a page
      cl_assert((a % CHIP_PAGE) + n <= CHIP_PAGE);
      for (size_t i = 0; i < n; i++) {
        s_mem[a + i] &= out[i];
      }
      s_sr1 &= ~SR1_WEL;
      return 0;
    }
    case 0x20:
    case 0x52:
    case 0xD8: {
      cl_assert(s_sr1 & SR1_WEL);
      cl_assert_equal_i(op->addr.nbytes, s_4byte ? 4 : 3);
      uint32_t size = (op->cmd.opcode == 0x20)   ? 0x1000
                      : (op->cmd.opcode == 0x52) ? 0x8000
                                                 : 0x10000;
      uint32_t a = prv_addr(op);
      cl_assert_equal_i(a % size, 0);
      cl_assert(a + size <= CHIP_SIZE);
      memset(&s_mem[a], 0xff, size);
      s_sr1 = (s_sr1 & ~SR1_WEL) | SR1_WIP;
      s_erase_polls_left = 3;
      return 0;
    }
    case 0x75:
      cl_assert(s_sr1 & SR1_WIP);
      s_erase_suspended = true;
      s_sr1 &= ~SR1_WIP;
      s_sr2 |= SR2_SUS;
      return 0;
    case 0x7A:
      cl_assert(s_erase_suspended);
      s_erase_suspended = false;
      s_sr1 |= SR1_WIP;
      s_sr2 &= ~SR2_SUS;
      return 0;
    case 0xB7:
      s_4byte = true;
      return 0;
    case 0xB9:
      s_powered_down = true;
      return 0;
    case 0xAB:
      s_powered_down = false;
      return 0;
    case 0x66:
      s_reset_enabled = true;
      return 0;
    case 0x99:
      cl_assert(s_reset_enabled);
      s_reset_enabled = false;
      s_resets++;
      s_sr1 = 0;
      s_sr2 &= ~SR2_SUS;
      s_4byte = false;
      return 0;
    case 0x48: {
      cl_assert_equal_i(op->dummy.nbytes, 1);
      uint32_t a = prv_addr(op);
      in[0] = s_otp[(a >> 12) - 1][a & 0x3ff];
      return 0;
    }
    case 0x42: {
      cl_assert(s_sr1 & SR1_WEL);
      uint32_t a = prv_addr(op);
      s_otp[(a >> 12) - 1][a & 0x3ff] &= out[0];
      s_sr1 &= ~SR1_WEL;
      return 0;
    }
    case 0x44: {
      cl_assert(s_sr1 & SR1_WEL);
      uint32_t a = prv_addr(op);
      memset(s_otp[(a >> 12) - 1], 0xff, sizeof(s_otp[0]));
      s_sr1 &= ~SR1_WEL;
      return 0;
    }
    default:
      cl_fail("unexpected opcode");
      return -ENOTSUP;
  }
}

static bool prv_fake_supports_op(const struct pbl_spi_mem_device *dev,
                                 const struct pbl_spi_mem_op *op) {
  // Quad program only as 1-1-4; no dual reads
  if (op->cmd.opcode == 0x38 || op->cmd.opcode == 0xA2) {
    return false;
  }
  if (op->cmd.opcode == 0x3B || op->cmd.opcode == 0xBB) {
    return false;
  }
  return true;
}

static int prv_fake_adjust_op_size(const struct pbl_spi_mem_device *dev,
                                   struct pbl_spi_mem_op *op) {
  if (s_limit_cinstr && op->addr.buswidth <= 1 && op->data.buswidth <= 1 &&
      op->cmd.opcode == 0x5A) {
    size_t max = CINSTR_MAX - op->addr.nbytes - op->dummy.nbytes;
    if (op->data.nbytes > max) {
      op->data.nbytes = max;
    }
  }
  return 0;
}

static int prv_fake_init(const struct pbl_spi_mem_device *dev) {
  return 0;
}

static void prv_fake_set_power(const struct pbl_spi_mem_device *dev, bool on) {
  s_power_calls++;
}

static const struct pbl_spi_mem_ops s_fake_bus_ops = {
    .init = prv_fake_init,
    .supports_op = prv_fake_supports_op,
    .adjust_op_size = prv_fake_adjust_op_size,
    .exec_op = prv_fake_exec_op,
    .set_power = prv_fake_set_power,
};

static struct pbl_spi_mem_device_state s_fake_bus_state;
static const struct pbl_spi_mem_device s_fake_bus = {
    .state = &s_fake_bus_state,
    .ops = &s_fake_bus_ops,
};
const struct pbl_spi_mem_device *const SPI_MEM_NOR = &s_fake_bus;

static void prv_fire_timers(int max) {
  for (int i = 0; i < max; i++) {
    TimerID timer = stub_new_timer_get_next();
    if (timer == TIMER_INVALID_ID) {
      return;
    }
    stub_new_timer_fire(timer);
  }
}

// Tests
///////////////////////////////////////////////////////////

void test_spi_nor__initialize(void) {
  memset(s_mem, 0xff, sizeof(s_mem));
  memset(s_otp, 0xff, sizeof(s_otp));
  s_sr1 = 0;
  s_sr2 = 0;
  s_4byte = false;
  s_powered_down = false;
  s_reset_enabled = false;
  s_resets = 0;
  s_erase_polls_left = 0;
  s_erase_suspended = false;
  s_sfdp_available = true;
  s_limit_cinstr = true;
  s_exec_count = 0;
  s_last_read_opcode = 0;
  s_last_pp_opcode = 0;
  s_last_addr_nbytes = 0;
  s_power_calls = 0;
  prv_build_sfdp();
  memset(&s_fake_bus_state, 0, sizeof(s_fake_bus_state));

  spi_nor_reset_for_test();
}

void test_spi_nor__cleanup(void) {
  stub_new_timer_cleanup();
}

void test_spi_nor__probes_geometry_from_sfdp(void) {
  cl_assert_equal_i(pbl_flash_init(FLASH), 0);
  cl_assert_equal_i(s_resets, 1);
  cl_assert_equal_i(FLASH->geometry->size, CHIP_SIZE);
  cl_assert_equal_i(FLASH->geometry->page_size, CHIP_PAGE);
  cl_assert_equal_i(FLASH->geometry->subsector_size, 0x1000);
  cl_assert_equal_i(FLASH->geometry->sector_size, 0x10000);
  cl_assert_equal_i(FLASH->geometry->subsector_erase_ms, 48);
  cl_assert_equal_i(FLASH->geometry->sector_erase_ms, 128);
  // 3-byte addressing for a small part, so no EN4B
  cl_assert(!s_4byte);
  // Quad read/program selected and QE set through SR1+SR2
  cl_assert(s_sr2 & SR2_QE);
}

void test_spi_nor__falls_back_to_part_table_without_sfdp(void) {
  s_sfdp_available = false;
  cl_assert_equal_i(pbl_flash_init(FLASH), 0);
  cl_assert_equal_i(FLASH->geometry->size, PBL_FLASH_NOR_PART.geometry.size);
  cl_assert_equal_i(FLASH->geometry->sector_erase_ms, PBL_FLASH_NOR_PART.geometry.sector_erase_ms);
  // A 32 MB part needs 4-byte addressing
  cl_assert(s_4byte);
}

void test_spi_nor__read_uses_quad_io(void) {
  pbl_flash_init(FLASH);
  memset(&s_mem[0x100], 0xA5, 16);

  uint8_t buf[16];
  cl_assert_equal_i(pbl_flash_read(FLASH, 0x100, buf, sizeof(buf)), 0);
  cl_assert_equal_i(s_last_read_opcode, 0xEB);
  cl_assert_equal_i(s_last_addr_nbytes, 3);
  for (int i = 0; i < 16; i++) {
    cl_assert_equal_i(buf[i], 0xA5);
  }
}

void test_spi_nor__write_splits_pages_and_waits(void) {
  pbl_flash_init(FLASH);

  uint8_t data[600];
  for (int i = 0; i < 600; i++) {
    data[i] = i & 0xff;
  }
  cl_assert_equal_i(pbl_flash_write(FLASH, 0x0F0, data, sizeof(data)), 0);
  cl_assert_equal_i(s_last_pp_opcode, 0x32);
  cl_assert_equal_i(memcmp(&s_mem[0x0F0], data, sizeof(data)), 0);
  // WEL is consumed by each page program
  cl_assert(!(s_sr1 & SR1_WEL));
}

void test_spi_nor__erase_range_polls_status(void) {
  pbl_flash_init(FLASH);
  memset(s_mem, 0x00, sizeof(s_mem));

  // [0x1000, 0x21000): 15 subsectors, one sector, one subsector
  cl_assert_equal_i(pbl_flash_erase(FLASH, 0x1000, 0x20000), 0);
  for (uint32_t a = 0; a < 0x1000; a++) {
    cl_assert_equal_i(s_mem[a], 0x00);
  }
  for (uint32_t a = 0x1000; a < 0x21000; a++) {
    cl_assert_equal_i(s_mem[a], 0xff);
  }
  cl_assert_equal_i(s_mem[0x21000], 0x00);
  cl_assert(!(s_sr1 & SR1_WIP));
}

void test_spi_nor__read_during_erase_suspends(void) {
  pbl_flash_init(FLASH);
  memset(s_mem, 0x00, sizeof(s_mem));
  s_mem[0x30000] = 0x42;

  pbl_flash_erase_async(FLASH, 0x10000, 0x10000, NULL, NULL);
  cl_assert(s_sr1 & SR1_WIP);

  uint8_t v;
  pbl_flash_read(FLASH, 0x30000, &v, 1);
  cl_assert_equal_i(v, 0x42);
  cl_assert(s_erase_suspended);

  // The resume timer puts the erase back
  stub_new_timer_fire(FLASH->state->resume_timer);
  cl_assert(!s_erase_suspended);
  cl_assert(s_sr1 & SR1_WIP);
}

static void prv_status_cb(void *ctx, int status) {
  *(int *)ctx = status;
}

void test_spi_nor__async_erase_completes(void) {
  int status = -1;

  pbl_flash_init(FLASH);
  memset(s_mem, 0x00, sizeof(s_mem));
  pbl_flash_erase_async(FLASH, 0x0, 0x1000, prv_status_cb, &status);
  prv_fire_timers(20);
  cl_assert_equal_i(status, 0);
  cl_assert_equal_i(s_mem[0x0], 0xff);
}

void test_spi_nor__power_down_and_up(void) {
  pbl_flash_init(FLASH);
  pbl_flash_power_down(FLASH);
  cl_assert(s_powered_down);
  cl_assert_equal_i(s_power_calls, 1);
  pbl_flash_power_up(FLASH);
  cl_assert(!s_powered_down);
  cl_assert_equal_i(s_power_calls, 2);
}

void test_spi_nor__security_registers(void) {
  pbl_flash_init(FLASH);
  uint32_t reg = FLASH->sec_regs->addrs[0];
  uint8_t v;

  cl_assert_equal_i(pbl_flash_sec_reg_read(FLASH, reg + 4, &v), 0);
  cl_assert_equal_i(v, 0xff);
  cl_assert_equal_i(pbl_flash_sec_reg_write(FLASH, reg + 4, 0x3c), 0);
  cl_assert_equal_i(pbl_flash_sec_reg_read(FLASH, reg + 4, &v), 0);
  cl_assert_equal_i(v, 0x3c);
  cl_assert_equal_i(pbl_flash_sec_reg_erase(FLASH, reg), 0);
  cl_assert_equal_i(pbl_flash_sec_reg_read(FLASH, reg + 4, &v), 0);
  cl_assert_equal_i(v, 0xff);
  cl_assert_equal_i(pbl_flash_sec_reg_read(FLASH, 0x9000, &v), -EINVAL);

  bool locked = true;
  cl_assert_equal_i(pbl_flash_sec_reg_is_locked(FLASH, reg, &locked), 0);
  cl_assert(!locked);
}

void test_spi_nor__sfdp_read_is_chunked_for_small_controllers(void) {
  s_limit_cinstr = true;
  cl_assert_equal_i(pbl_flash_init(FLASH), 0);
  cl_assert_equal_i(FLASH->geometry->size, CHIP_SIZE);

  spi_nor_reset_for_test();
  s_limit_cinstr = false;
  int with_limit = s_exec_count;
  s_exec_count = 0;
  cl_assert_equal_i(pbl_flash_init(FLASH), 0);
  cl_assert(s_exec_count < with_limit);
}

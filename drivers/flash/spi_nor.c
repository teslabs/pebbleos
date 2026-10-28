/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include <pbl/drivers/flash/spi_nor.h>
#include <pbl/kernel/delay.h>
#include <pbl/logging/logging.h>
#include <pbl/util/math.h>
#include <pbl/util/misc.h>
#include <pbl/util/size.h>

PBL_LOG_MODULE_DECLARE(driver_flash, CONFIG_DRIVER_FLASH_LOG_LEVEL);

#define US_PER_MS            1000U
#define ID_LEN               3U
#define ADDR_3BYTE           3U
#define ADDR_4BYTE           4U
#define ADDR_4BYTE_THRESHOLD (16U * 1024U * 1024U)
#define FAST_READ_DUMMY      1U
#define SEC_REG_READ_DUMMY   1U

// SFDP (JESD216)
#define SFDP_SIGNATURE       0x50444653U
#define SFDP_HEADER_LEN      16U
#define SFDP_ADDR_NBYTES     3U
#define SFDP_DUMMY           1U
#define SFDP_BFPT_ID         0xFF00U
#define SFDP_BFPT_MAX_DWORDS 16U
#define SFDP_NUM_ERASE_TYPES 4U

#define BFPT_DW1  0U
#define BFPT_DW2  1U
#define BFPT_DW3  2U
#define BFPT_DW4  3U
#define BFPT_DW8  7U
#define BFPT_DW10 9U
#define BFPT_DW11 10U
#define BFPT_DW15 14U
#define BFPT_DW16 15U

#define BFPT_DW1_READ_1_1_2      PBL_BIT(16)
#define BFPT_DW1_READ_1_2_2      PBL_BIT(20)
#define BFPT_DW1_READ_1_4_4      PBL_BIT(21)
#define BFPT_DW1_READ_1_1_4      PBL_BIT(22)
#define BFPT_DW2_DENSITY_EXP     PBL_BIT(31)
#define BFPT_DW11_PAGE_SHIFT(dw) (((dw) >> 4) & 0xFU)
#define BFPT_DW15_QER(dw)        (((dw) >> 20) & 0x7U)
#define BFPT_DW16_EN4B_WREN      PBL_BIT(25)

// JESD216 quad enable requirements
#define QER_NONE        0U
#define QER_SR2_BIT1_V1 1U
#define QER_SR1_BIT6    2U
#define QER_SR2_BIT1_V4 4U
#define QER_SR2_BIT1_V5 5U
#define QER_SR2_BIT1_V6 6U

// Erase time units of BFPT DWORD 10, in milliseconds
static const uint16_t s_erase_time_units_ms[] = {1U, 16U, 128U, 1000U};

// Bits 15-12 of a security register address are its one-based index
#define SEC_REG_LB(addr) PBL_BIT(PBL_FLASH_NOR_SR2_LB_SHIFT + ((addr) >> 12U) - 1U)

struct read_mode {
  uint8_t opcode;
  uint8_t dummy_cycles;
  uint8_t addr_width;
  uint8_t data_width;
};

struct bfpt {
  uint32_t dw[SFDP_BFPT_MAX_DWORDS];
  uint8_t ndwords;
};

static const struct pbl_flash_spi_nor *prv_nor(const struct pbl_flash *flash) {
  return container_of(flash, const struct pbl_flash_spi_nor, flash);
}

static int prv_cmd(const struct pbl_flash_spi_nor *nor, uint8_t opcode) {
  struct pbl_spi_mem_op op = PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(opcode, 1), PBL_SPI_MEM_OP_NO_ADDR,
                                            PBL_SPI_MEM_OP_NO_DUMMY, PBL_SPI_MEM_OP_NO_DATA);

  return pbl_spi_mem_exec_op(nor->bus, &op);
}

static int prv_read_reg(const struct pbl_flash_spi_nor *nor, uint8_t opcode, void *buf,
                        size_t len) {
  struct pbl_spi_mem_op op =
      PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(opcode, 1), PBL_SPI_MEM_OP_NO_ADDR, PBL_SPI_MEM_OP_NO_DUMMY,
                     PBL_SPI_MEM_OP_DATA_IN(len, buf, 1));

  return pbl_spi_mem_exec_op(nor->bus, &op);
}

static int prv_write_reg(const struct pbl_flash_spi_nor *nor, uint8_t opcode, const void *buf,
                         size_t len) {
  struct pbl_spi_mem_op op =
      PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(opcode, 1), PBL_SPI_MEM_OP_NO_ADDR, PBL_SPI_MEM_OP_NO_DUMMY,
                     PBL_SPI_MEM_OP_DATA_OUT(len, buf, 1));

  return pbl_spi_mem_exec_op(nor->bus, &op);
}

static int prv_busy(const struct pbl_flash_spi_nor *nor, bool *busy) {
  uint8_t sr;
  int ret;

  ret = prv_read_reg(nor, PBL_FLASH_NOR_CMD_RDSR, &sr, sizeof(sr));
  if (ret != 0) {
    return ret;
  }

  *busy = (sr & PBL_FLASH_NOR_SR_WIP) != 0U;

  return 0;
}

static int prv_wait_idle(const struct pbl_flash_spi_nor *nor, bool yield) {
  bool busy;
  int ret;

  while (true) {
    ret = prv_busy(nor, &busy);
    if (ret != 0 || !busy) {
      return ret;
    }

    if (yield && !pbl_flash_in_coredump(&nor->flash)) {
      pbl_sleep_ms(0U);
    }
  }
}

static int prv_write_reg_wait(const struct pbl_flash_spi_nor *nor, uint8_t opcode, const void *buf,
                              size_t len) {
  int ret;

  ret = prv_cmd(nor, PBL_FLASH_NOR_CMD_WREN);
  if (ret != 0) {
    return ret;
  }

  ret = prv_write_reg(nor, opcode, buf, len);
  if (ret != 0) {
    return ret;
  }

  return prv_wait_idle(nor, false);
}

// Runs an addressed operation in as many pieces as the controller needs
static int prv_exec_chunked(const struct pbl_flash_spi_nor *nor, struct pbl_spi_mem_op *op) {
  size_t remaining = op->data.nbytes;
  int ret;

  while (remaining > 0U) {
    op->data.nbytes = remaining;
    ret = pbl_spi_mem_adjust_op_size(nor->bus, op);
    if (ret != 0) {
      return ret;
    }

    if (op->data.nbytes == 0U) {
      return -EINVAL;
    }

    ret = pbl_spi_mem_exec_op(nor->bus, op);
    if (ret != 0) {
      return ret;
    }

    remaining -= op->data.nbytes;
    op->addr.val += op->data.nbytes;
    op->data.buf.in = (uint8_t *)op->data.buf.in + op->data.nbytes;
  }

  return 0;
}

static uint32_t prv_le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int prv_read_sfdp(const struct pbl_flash_spi_nor *nor, uint32_t addr, void *buf,
                         size_t len) {
  struct pbl_spi_mem_op op =
      PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(PBL_FLASH_NOR_CMD_RDSFDP, 1),
                     PBL_SPI_MEM_OP_ADDR(SFDP_ADDR_NBYTES, addr, 1),
                     PBL_SPI_MEM_OP_DUMMY(SFDP_DUMMY, 1), PBL_SPI_MEM_OP_DATA_IN(len, buf, 1));

  return prv_exec_chunked(nor, &op);
}

// Returns 0 with the Basic Flash Parameter Table, or -ENOENT if the part has no usable SFDP
static int prv_read_bfpt(const struct pbl_flash_spi_nor *nor, struct bfpt *bfpt) {
  uint8_t hdr[SFDP_HEADER_LEN];
  uint8_t raw[SFDP_BFPT_MAX_DWORDS * sizeof(uint32_t)];
  uint16_t id;
  uint32_t ptp;
  int ret;

  ret = prv_read_sfdp(nor, 0U, hdr, sizeof(hdr));
  if (ret != 0) {
    return ret;
  }

  if (prv_le32(hdr) != SFDP_SIGNATURE) {
    return -ENOENT;
  }

  // The first parameter header is the BFPT
  id = (uint16_t)(hdr[8] | (hdr[15] << 8));
  ptp = (uint32_t)hdr[12] | ((uint32_t)hdr[13] << 8) | ((uint32_t)hdr[14] << 16);
  if (id != SFDP_BFPT_ID || hdr[11] == 0U) {
    return -ENOENT;
  }

  bfpt->ndwords = MIN(hdr[11], SFDP_BFPT_MAX_DWORDS);
  ret = prv_read_sfdp(nor, ptp, raw, bfpt->ndwords * sizeof(uint32_t));
  if (ret != 0) {
    return ret;
  }

  for (uint8_t i = 0U; i < bfpt->ndwords; i++) {
    bfpt->dw[i] = prv_le32(&raw[i * sizeof(uint32_t)]);
  }

  return 0;
}

static struct read_mode prv_bfpt_read_mode(uint16_t half, uint8_t addr_width, uint8_t data_width) {
  return (struct read_mode){
    .opcode = (uint8_t)(half >> 8),
    .dummy_cycles = (uint8_t)((half & 0x1FU) + ((half >> 5) & 0x7U)),
    .addr_width = addr_width,
    .data_width = data_width,
  };
}

static void prv_apply_bfpt(const struct pbl_flash_spi_nor *nor, const struct bfpt *bfpt,
                           uint8_t *qer) {
  struct pbl_flash_spi_nor_data *data = nor->data;
  struct pbl_flash_geometry *geo = &data->geometry;
  uint32_t dw2 = bfpt->dw[BFPT_DW2];
  uint64_t bits;
  uint32_t small = 0U;
  uint32_t large = 0U;

  bits = ((dw2 & BFPT_DW2_DENSITY_EXP) != 0U) ? (1ULL << (dw2 & ~BFPT_DW2_DENSITY_EXP))
                                              : ((uint64_t)dw2 + 1U);
  geo->size = (uint32_t)(bits / 8U);

  if (bfpt->ndwords > BFPT_DW11) {
    geo->page_size = 1U << BFPT_DW11_PAGE_SHIFT(bfpt->dw[BFPT_DW11]);
  }

  if (bfpt->ndwords > BFPT_DW15) {
    *qer = BFPT_DW15_QER(bfpt->dw[BFPT_DW15]);
  }

  // The smallest erase type is the subsector, the largest the sector
  for (uint8_t i = 0U; i < SFDP_NUM_ERASE_TYPES; i++) {
    uint32_t dw = bfpt->dw[BFPT_DW8 + (i / 2U)];
    uint16_t half = (uint16_t)(((i % 2U) != 0U) ? (dw >> 16) : dw);
    uint8_t shift = (uint8_t)half;
    uint32_t size = (shift != 0U) ? (1U << shift) : 0U;
    uint16_t time_ms = 0U;

    if (size == 0U) {
      continue;
    }

    if (bfpt->ndwords > BFPT_DW10) {
      uint32_t t = (bfpt->dw[BFPT_DW10] >> (4U + (i * 7U))) & 0x7FU;
      time_ms = (uint16_t)(((t & 0x1FU) + 1U) * s_erase_time_units_ms[(t >> 5) & 0x3U]);
    }

    if (small == 0U || size < small) {
      small = size;
      geo->subsector_size = size;
      data->subsector_opcode = (uint8_t)(half >> 8);
      if (time_ms != 0U) {
        geo->subsector_erase_ms = time_ms;
      }
    }

    if (size > large) {
      large = size;
      geo->sector_size = size;
      data->sector_opcode = (uint8_t)(half >> 8);
      if (time_ms != 0U) {
        geo->sector_erase_ms = time_ms;
      }
    }
  }
}

static int prv_set_quad_enable(const struct pbl_flash_spi_nor *nor, uint8_t qer) {
  uint8_t sr[2];
  int ret;

  switch (qer) {
    case QER_NONE:
      return 0;
    case QER_SR1_BIT6:
      ret = prv_read_reg(nor, PBL_FLASH_NOR_CMD_RDSR, &sr[0], 1U);
      if (ret != 0 || (sr[0] & PBL_FLASH_NOR_SR_QE) != 0U) {
        return ret;
      }
      sr[0] |= PBL_FLASH_NOR_SR_QE;
      return prv_write_reg_wait(nor, PBL_FLASH_NOR_CMD_WRSR, sr, 1U);
    case QER_SR2_BIT1_V1:
    case QER_SR2_BIT1_V4:
    case QER_SR2_BIT1_V5:
      ret = prv_read_reg(nor, PBL_FLASH_NOR_CMD_RDSR2, &sr[1], 1U);
      if (ret != 0 || (sr[1] & PBL_FLASH_NOR_SR2_QE) != 0U) {
        return ret;
      }
      ret = prv_read_reg(nor, PBL_FLASH_NOR_CMD_RDSR, &sr[0], 1U);
      if (ret != 0) {
        return ret;
      }
      sr[1] |= PBL_FLASH_NOR_SR2_QE;
      return prv_write_reg_wait(nor, PBL_FLASH_NOR_CMD_WRSR, sr, sizeof(sr));
    case QER_SR2_BIT1_V6:
      ret = prv_read_reg(nor, PBL_FLASH_NOR_CMD_RDSR2, &sr[1], 1U);
      if (ret != 0 || (sr[1] & PBL_FLASH_NOR_SR2_QE) != 0U) {
        return ret;
      }
      sr[1] |= PBL_FLASH_NOR_SR2_QE;
      return prv_write_reg_wait(nor, PBL_FLASH_NOR_CMD_WRSR2, &sr[1], 1U);
    default:
      return -ENOTSUP;
  }
}

static uint8_t prv_part_qer(const struct pbl_flash_nor_part *part) {
  switch (part->qe) {
    case PBL_FLASH_NOR_QE_SR2_BIT1_WR_SR1:
      return QER_SR2_BIT1_V1;
    case PBL_FLASH_NOR_QE_SR1_BIT6:
      return QER_SR1_BIT6;
    case PBL_FLASH_NOR_QE_SR2_BIT1:
      return QER_SR2_BIT1_V6;
    default:
      return QER_NONE;
  }
}

static bool prv_can_read(const struct pbl_flash_spi_nor *nor, const struct read_mode *mode) {
  struct pbl_spi_mem_op op = PBL_SPI_MEM_OP(
      PBL_SPI_MEM_OP_CMD(mode->opcode, 1),
      PBL_SPI_MEM_OP_ADDR(nor->data->addr_nbytes, 0U, mode->addr_width),
      PBL_SPI_MEM_OP_DUMMY(mode->dummy_cycles * mode->addr_width / 8U, mode->addr_width),
      PBL_SPI_MEM_OP_DATA_IN(nor->data->geometry.page_size, nullptr, mode->data_width));

  return mode->opcode != 0U && pbl_spi_mem_supports_op(nor->bus, &op);
}

static bool prv_can_program(const struct pbl_flash_spi_nor *nor, uint8_t opcode,
                            uint8_t data_width) {
  struct pbl_spi_mem_op op =
      PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(opcode, 1),
                     PBL_SPI_MEM_OP_ADDR(nor->data->addr_nbytes, 0U, 1), PBL_SPI_MEM_OP_NO_DUMMY,
                     PBL_SPI_MEM_OP_DATA_OUT(nor->data->geometry.page_size, nullptr, data_width));

  return pbl_spi_mem_supports_op(nor->bus, &op);
}

// Picks the widest read and program modes both the part and the controller support
static int prv_select_modes(const struct pbl_flash_spi_nor *nor, const struct bfpt *bfpt,
                            uint8_t qer) {
  struct pbl_flash_spi_nor_data *data = nor->data;
  struct read_mode modes[5];
  size_t n = 0U;
  bool quad = false;

  if (bfpt != nullptr) {
    uint32_t dw1 = bfpt->dw[BFPT_DW1];

    if ((dw1 & BFPT_DW1_READ_1_4_4) != 0U && qer != QER_NONE) {
      modes[n++] = prv_bfpt_read_mode((uint16_t)bfpt->dw[BFPT_DW3], 4U, 4U);
    }
    if ((dw1 & BFPT_DW1_READ_1_1_4) != 0U && qer != QER_NONE) {
      modes[n++] = prv_bfpt_read_mode((uint16_t)(bfpt->dw[BFPT_DW3] >> 16), 1U, 4U);
    }
    if ((dw1 & BFPT_DW1_READ_1_2_2) != 0U) {
      modes[n++] = prv_bfpt_read_mode((uint16_t)(bfpt->dw[BFPT_DW4] >> 16), 2U, 2U);
    }
    if ((dw1 & BFPT_DW1_READ_1_1_2) != 0U) {
      modes[n++] = prv_bfpt_read_mode((uint16_t)bfpt->dw[BFPT_DW4], 1U, 2U);
    }
  }

  modes[n++] = (struct read_mode){
    .opcode = PBL_FLASH_NOR_CMD_FAST_READ,
    .dummy_cycles = FAST_READ_DUMMY * 8U,
    .addr_width = 1U,
    .data_width = 1U,
  };

  for (size_t i = 0U; i < n; i++) {
    if (prv_can_read(nor, &modes[i])) {
      data->read_opcode = modes[i].opcode;
      data->read_dummy = modes[i].dummy_cycles * modes[i].addr_width / 8U;
      data->read_addr_width = modes[i].addr_width;
      data->read_data_width = modes[i].data_width;
      quad = modes[i].data_width == 4U;
      break;
    }
  }

  if (qer != QER_NONE && prv_can_program(nor, PBL_FLASH_NOR_CMD_PP_1_1_4, 4U)) {
    data->pp_opcode = PBL_FLASH_NOR_CMD_PP_1_1_4;
    data->pp_data_width = 4U;
    quad = true;
  } else if (prv_can_program(nor, PBL_FLASH_NOR_CMD_PP, 1U)) {
    data->pp_opcode = PBL_FLASH_NOR_CMD_PP;
    data->pp_data_width = 1U;
  } else {
    return -ENOTSUP;
  }

  return quad ? prv_set_quad_enable(nor, qer) : 0;
}

static int prv_probe(const struct pbl_flash_spi_nor *nor) {
  const struct pbl_flash_nor_part *part = nor->part;
  struct pbl_flash_spi_nor_data *data = nor->data;
  uint8_t qer = prv_part_qer(part);
  struct bfpt bfpt;
  bool have_bfpt;
  bool en4b_wren = false;
  int ret;

  data->geometry = part->geometry;
  data->sector_opcode = PBL_FLASH_NOR_CMD_BE;
  data->subsector_opcode = PBL_FLASH_NOR_CMD_SE;

  ret = prv_read_bfpt(nor, &bfpt);
  if (ret != 0 && ret != -ENOENT) {
    return ret;
  }

  have_bfpt = ret == 0;
  if (have_bfpt) {
    prv_apply_bfpt(nor, &bfpt, &qer);
    en4b_wren = bfpt.ndwords > BFPT_DW16 && (bfpt.dw[BFPT_DW16] & BFPT_DW16_EN4B_WREN) != 0U;
  } else {
    PBL_LOG_DBG("No SFDP, using the %s description", part->name);
  }

  if (data->geometry.size == 0U || data->geometry.page_size == 0U ||
      data->geometry.subsector_size == 0U || data->geometry.sector_size == 0U) {
    return -ENODEV;
  }

  data->addr_nbytes = (data->geometry.size > ADDR_4BYTE_THRESHOLD) ? ADDR_4BYTE : ADDR_3BYTE;
  if (data->addr_nbytes == ADDR_4BYTE) {
    if (en4b_wren) {
      ret = prv_cmd(nor, PBL_FLASH_NOR_CMD_WREN);
      if (ret != 0) {
        return ret;
      }
    }

    ret = prv_cmd(nor, PBL_FLASH_NOR_CMD_EN4B);
    if (ret != 0) {
      return ret;
    }
  }

  return prv_select_modes(nor, have_bfpt ? &bfpt : nullptr, qer);
}

static int prv_init(const struct pbl_flash *flash) {
  const struct pbl_flash_spi_nor *nor = prv_nor(flash);
  uint8_t id[ID_LEN];
  uint32_t jedec_id;
  int ret;

  pbl_spi_mem_set_polling(nor->bus, pbl_flash_in_coredump(flash));

  if (nor->data->initialized) {
    return 0;
  }

  if (!pbl_device_is_ready(&nor->bus->dev)) {
    return -ENODEV;
  }

  // Abort any program or erase left over from before the reset
  ret = prv_cmd(nor, PBL_FLASH_NOR_CMD_RSTEN);
  if (ret != 0) {
    return ret;
  }

  ret = prv_cmd(nor, PBL_FLASH_NOR_CMD_RST);
  if (ret != 0) {
    return ret;
  }

  if (pbl_flash_in_coredump(flash)) {
    pbl_busy_wait_us(nor->part->reset_ms * US_PER_MS);
  } else {
    pbl_sleep_ms(nor->part->reset_ms);
  }

  ret = prv_read_reg(nor, PBL_FLASH_NOR_CMD_RDID, id, sizeof(id));
  if (ret != 0) {
    return ret;
  }

  jedec_id = (uint32_t)id[0] | ((uint32_t)id[1] << 8) | ((uint32_t)id[2] << 16);
  if (jedec_id != nor->part->id) {
    PBL_LOG_WRN("Flash is not %s (id 0x%06" PRIx32 ")", nor->part->name, jedec_id);
  }

  ret = prv_probe(nor);
  if (ret != 0) {
    return ret;
  }

  PBL_LOG_DBG("%" PRIu32 " bytes, erase %" PRIu32 "/%" PRIu32 ", read 0x%02x, program 0x%02x",
              nor->data->geometry.size, nor->data->geometry.subsector_size,
              nor->data->geometry.sector_size, nor->data->read_opcode, nor->data->pp_opcode);

  nor->data->initialized = true;

  return 0;
}

static int prv_read(const struct pbl_flash *flash, uint32_t addr, void *buf, size_t len) {
  const struct pbl_flash_spi_nor *nor = prv_nor(flash);
  const struct pbl_flash_spi_nor_data *data = nor->data;
  struct pbl_spi_mem_op op = PBL_SPI_MEM_OP(
      PBL_SPI_MEM_OP_CMD(data->read_opcode, 1),
      PBL_SPI_MEM_OP_ADDR(data->addr_nbytes, addr - flash->base, data->read_addr_width),
      PBL_SPI_MEM_OP_DUMMY(data->read_dummy, data->read_addr_width),
      PBL_SPI_MEM_OP_DATA_IN(len, buf, data->read_data_width));

  return prv_exec_chunked(nor, &op);
}

static int prv_write(const struct pbl_flash *flash, uint32_t addr, const void *buf, size_t len) {
  const struct pbl_flash_spi_nor *nor = prv_nor(flash);
  const struct pbl_flash_spi_nor_data *data = nor->data;
  const uint32_t page_size = data->geometry.page_size;
  uint32_t offset = addr - flash->base;
  const uint8_t *src = buf;
  int ret;

  while (len > 0U) {
    struct pbl_spi_mem_op op =
        PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(data->pp_opcode, 1),
                       PBL_SPI_MEM_OP_ADDR(data->addr_nbytes, offset, 1), PBL_SPI_MEM_OP_NO_DUMMY,
                       PBL_SPI_MEM_OP_DATA_OUT(MIN(len, page_size - (offset % page_size)), src,
                                               data->pp_data_width));

    ret = pbl_spi_mem_adjust_op_size(nor->bus, &op);
    if (ret != 0) {
      return ret;
    }

    if (op.data.nbytes == 0U) {
      return -EINVAL;
    }

    ret = prv_cmd(nor, PBL_FLASH_NOR_CMD_WREN);
    if (ret != 0) {
      return ret;
    }

    ret = pbl_spi_mem_exec_op(nor->bus, &op);
    if (ret != 0) {
      return ret;
    }

    ret = prv_wait_idle(nor, true);
    if (ret != 0) {
      return ret;
    }

    offset += op.data.nbytes;
    src += op.data.nbytes;
    len -= op.data.nbytes;
  }

  return 0;
}

static int prv_erase(const struct pbl_flash *flash, uint32_t addr, uint32_t size) {
  const struct pbl_flash_spi_nor *nor = prv_nor(flash);
  const struct pbl_flash_spi_nor_data *data = nor->data;
  struct pbl_spi_mem_op op;
  uint8_t opcode;
  int ret;

  if (size == data->geometry.sector_size) {
    opcode = data->sector_opcode;
  } else if (size == data->geometry.subsector_size) {
    opcode = data->subsector_opcode;
  } else {
    return -EINVAL;
  }

  ret = prv_cmd(nor, PBL_FLASH_NOR_CMD_WREN);
  if (ret != 0) {
    return ret;
  }

  op = PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(opcode, 1),
                      PBL_SPI_MEM_OP_ADDR(data->addr_nbytes, addr - flash->base, 1),
                      PBL_SPI_MEM_OP_NO_DUMMY, PBL_SPI_MEM_OP_NO_DATA);

  return pbl_spi_mem_exec_op(nor->bus, &op);
}

static int prv_erase_status(const struct pbl_flash *flash) {
  const struct pbl_flash_spi_nor *nor = prv_nor(flash);
  uint8_t sr2;
  bool busy;
  int ret;

  ret = prv_busy(nor, &busy);
  if (ret != 0) {
    return ret;
  }

  if (busy) {
    return -EBUSY;
  }

  ret = prv_read_reg(nor, PBL_FLASH_NOR_CMD_RDSR2, &sr2, sizeof(sr2));
  if (ret != 0) {
    return ret;
  }

  return ((sr2 & PBL_FLASH_NOR_SR2_SUS) != 0U) ? -EAGAIN : 0;
}

static int prv_erase_suspend(const struct pbl_flash *flash) {
  const struct pbl_flash_spi_nor *nor = prv_nor(flash);
  bool busy;
  int ret;

  ret = prv_busy(nor, &busy);
  if (ret != 0) {
    return ret;
  }

  if (!busy) {
    return 1;
  }

  ret = prv_cmd(nor, PBL_FLASH_NOR_CMD_SUSPEND);
  if (ret != 0) {
    return ret;
  }

  pbl_busy_wait_us(nor->part->suspend_us);

  return 0;
}

static int prv_erase_resume(const struct pbl_flash *flash) {
  return prv_cmd(prv_nor(flash), PBL_FLASH_NOR_CMD_RESUME);
}

static void prv_power_down(const struct pbl_flash *flash) {
  const struct pbl_flash_spi_nor *nor = prv_nor(flash);

  (void)prv_cmd(nor, PBL_FLASH_NOR_CMD_DPD);
  pbl_busy_wait_us(nor->part->dpd_enter_us);
  pbl_spi_mem_power_down(nor->bus);
}

static void prv_power_up(const struct pbl_flash *flash) {
  const struct pbl_flash_spi_nor *nor = prv_nor(flash);

  pbl_spi_mem_power_up(nor->bus);
  (void)prv_cmd(nor, PBL_FLASH_NOR_CMD_RDPD);
  pbl_busy_wait_us(nor->part->dpd_exit_us);
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
  const struct pbl_flash_spi_nor *nor = prv_nor(flash);
  struct pbl_spi_mem_op op;
  int ret;

  ret = prv_sec_reg_check(flash, addr);
  if (ret != 0) {
    return ret;
  }

  op = PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(PBL_FLASH_NOR_CMD_RDSCUR, 1),
                      PBL_SPI_MEM_OP_ADDR(nor->data->addr_nbytes, addr, 1),
                      PBL_SPI_MEM_OP_DUMMY(SEC_REG_READ_DUMMY, 1),
                      PBL_SPI_MEM_OP_DATA_IN(sizeof(*val), val, 1));

  return pbl_spi_mem_exec_op(nor->bus, &op);
}

static int prv_sec_reg_write(const struct pbl_flash *flash, uint32_t addr, uint8_t val) {
  const struct pbl_flash_spi_nor *nor = prv_nor(flash);
  struct pbl_spi_mem_op op;
  int ret;

  ret = prv_sec_reg_check(flash, addr);
  if (ret != 0) {
    return ret;
  }

  ret = prv_cmd(nor, PBL_FLASH_NOR_CMD_WREN);
  if (ret != 0) {
    return ret;
  }

  op = PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(PBL_FLASH_NOR_CMD_PRSCUR, 1),
                      PBL_SPI_MEM_OP_ADDR(nor->data->addr_nbytes, addr, 1), PBL_SPI_MEM_OP_NO_DUMMY,
                      PBL_SPI_MEM_OP_DATA_OUT(sizeof(val), &val, 1));
  ret = pbl_spi_mem_exec_op(nor->bus, &op);
  if (ret != 0) {
    return ret;
  }

  return prv_wait_idle(nor, false);
}

static int prv_sec_reg_erase(const struct pbl_flash *flash, uint32_t addr) {
  const struct pbl_flash_spi_nor *nor = prv_nor(flash);
  struct pbl_spi_mem_op op;
  int ret;

  ret = prv_sec_reg_check(flash, addr);
  if (ret != 0) {
    return ret;
  }

  ret = prv_cmd(nor, PBL_FLASH_NOR_CMD_WREN);
  if (ret != 0) {
    return ret;
  }

  op = PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(PBL_FLASH_NOR_CMD_ERSCUR, 1),
                      PBL_SPI_MEM_OP_ADDR(nor->data->addr_nbytes, addr, 1), PBL_SPI_MEM_OP_NO_DUMMY,
                      PBL_SPI_MEM_OP_NO_DATA);
  ret = pbl_spi_mem_exec_op(nor->bus, &op);
  if (ret != 0) {
    return ret;
  }

  return prv_wait_idle(nor, false);
}

static int prv_sec_reg_is_locked(const struct pbl_flash *flash, uint32_t addr, bool *locked) {
  const struct pbl_flash_spi_nor *nor = prv_nor(flash);
  uint8_t sr2;
  int ret;

  ret = prv_sec_reg_check(flash, addr);
  if (ret != 0) {
    return ret;
  }

  ret = prv_read_reg(nor, PBL_FLASH_NOR_CMD_RDSR2, &sr2, sizeof(sr2));
  if (ret != 0) {
    return ret;
  }

  *locked = (sr2 & SEC_REG_LB(addr)) != 0U;

  return 0;
}

#ifdef CONFIG_RECOVERY_FW
static int prv_sec_reg_lock(const struct pbl_flash *flash, uint32_t addr) {
  const struct pbl_flash_spi_nor *nor = prv_nor(flash);
  uint8_t sr[2];
  int ret;

  ret = prv_sec_reg_check(flash, addr);
  if (ret != 0) {
    return ret;
  }

  ret = prv_read_reg(nor, PBL_FLASH_NOR_CMD_RDSR, &sr[0], 1U);
  if (ret != 0) {
    return ret;
  }

  ret = prv_read_reg(nor, PBL_FLASH_NOR_CMD_RDSR2, &sr[1], 1U);
  if (ret != 0) {
    return ret;
  }

  sr[1] |= SEC_REG_LB(addr);

  return prv_write_reg_wait(nor, PBL_FLASH_NOR_CMD_WRSR, sr, sizeof(sr));
}
#endif

const struct pbl_flash_ops pbl_flash_spi_nor_ops = {
  .init = prv_init,
  .read = prv_read,
  .write = prv_write,
  .erase = prv_erase,
  .erase_status = prv_erase_status,
  .erase_suspend = prv_erase_suspend,
  .erase_resume = prv_erase_resume,
  .power_down = prv_power_down,
  .power_up = prv_power_up,
  .sec_reg_read = prv_sec_reg_read,
  .sec_reg_write = prv_sec_reg_write,
  .sec_reg_erase = prv_sec_reg_erase,
  .sec_reg_is_locked = prv_sec_reg_is_locked,
#ifdef CONFIG_RECOVERY_FW
  .sec_reg_lock = prv_sec_reg_lock,
#endif
};

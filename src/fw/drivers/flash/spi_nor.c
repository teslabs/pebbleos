/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/flash.h>
#include <pbl/drivers/flash/nor_part.h>
#include <pbl/drivers/spi_mem.h>

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include <pbl/logging/logging.h>
#include "kernel/util/delay.h"
#include "kernel/util/sleep.h"
#include "pbl/util/math.h"
#include "pbl/util/misc.h"
#include "system/passert.h"

#ifdef CONFIG_SOC_SF32LB52
#include <bf0_hal.h>
#endif

PBL_LOG_MODULE_DECLARE(driver_flash, CONFIG_DRIVER_FLASH_LOG_LEVEL);

// Generic JEDEC SPI NOR driver, after Linux spi-nor: the part is described by
// its SFDP tables (JESD216) with the Kconfig-selected part table filling in
// what SFDP does not cover (security registers, latencies, quirks).

// Standard opcodes
#define OP_WREN 0x06
#define OP_RDSR1 0x05
#define OP_RDSR2 0x35
#define OP_WRSR 0x01
#define OP_WRSR2 0x31
#define OP_RDID 0x9F
#define OP_RDSFDP 0x5A
#define OP_READ_FAST 0x0B
#define OP_PP 0x02
#define OP_PP_1_1_4 0x32
#define OP_ERASE_4K 0x20
#define OP_ERASE_64K 0xD8
#define OP_ERASE_SUSPEND 0x75
#define OP_ERASE_RESUME 0x7A
#define OP_EN4B 0xB7
#define OP_DPD 0xB9
#define OP_RDPD 0xAB
#define OP_RESET_EN 0x66
#define OP_RESET 0x99
#define OP_ERASE_SEC 0x44
#define OP_PROGRAM_SEC 0x42
#define OP_READ_SEC 0x48

#define SR1_WIP (1U << 0)
#define SR1_BP_MASK 0x7C
#define SR2_QE (1U << 1)
#define SR2_SUS (1U << 7)

// Bits 15-12 of a security register address are its one-based index; the
// lock bits sit at SR2[5:3].
#define SEC_ADDR_TO_LB(addr) ((1U << (((addr) >> 12U) - 1U)) << 3U)

#define ADDR_4BYTE_THRESHOLD 0x1000000UL

#define DEFAULT_RESET_LATENCY_MS 30
#define DEFAULT_SUSPEND_LATENCY_US 20
#define DEFAULT_DPD_LATENCY_US 30
#define DEFAULT_SECTOR_ERASE_MS 150
#define DEFAULT_SUBSECTOR_ERASE_MS 50

// SFDP (JESD216)
#define SFDP_SIGNATURE 0x50444653UL
#define SFDP_BFPT_ID 0xFF00
#define SFDP_BFPT_MAX_DWORDS 16

#define BFPT_DW1_ERASE_4K_SUPPORTED(dw) (((dw) & 0x3) == 0x1)
#define BFPT_DW1_ERASE_4K_OPCODE(dw) (((dw) >> 8) & 0xFF)
#define BFPT_DW1_ADDR_BYTES(dw) (((dw) >> 17) & 0x3)
#define BFPT_DW1_READ_1_1_2 (1UL << 16)
#define BFPT_DW1_READ_1_2_2 (1UL << 20)
#define BFPT_DW1_READ_1_4_4 (1UL << 21)
#define BFPT_DW1_READ_1_1_4 (1UL << 22)
#define BFPT_DW11_PAGE_SIZE(dw) (1UL << (((dw) >> 4) & 0xF))
#define BFPT_DW15_QER(dw) (((dw) >> 20) & 0x7)
#define BFPT_DW16_EN4B_B7 (1UL << 24)
#define BFPT_DW16_EN4B_WREN_B7 (1UL << 25)

struct read_op {
  uint8_t opcode;
  uint8_t dummy_cycles;
  uint8_t addr_width;
  uint8_t data_width;
};

struct erase_type {
  uint8_t opcode;
  uint32_t size;
  uint32_t time_ms;
};

struct spi_nor_state {
  struct pbl_flash_device_state flash;
  const struct pbl_spi_mem_device *bus;
  struct pbl_flash_geometry geometry;
  struct read_op read;
  uint8_t pp_opcode;
  uint8_t pp_data_width;
  uint8_t addr_nbytes;
  uint8_t erase_sector_opcode;
  uint8_t erase_subsector_opcode;
  bool en4b_needs_wren;
  bool initialized;
};

struct spi_nor {
  struct pbl_flash_device dev;
  //! Optional: fills in what SFDP does not describe.
  const struct pbl_flash_nor_part *part;
};

static inline const struct spi_nor *prv_cfg(const struct pbl_flash_device *dev) {
  return container_of(dev, const struct spi_nor, dev);
}

static inline struct spi_nor_state *prv_state(const struct pbl_flash_device *dev) {
  return container_of(dev->state, struct spi_nor_state, flash);
}

static inline const struct pbl_spi_mem_device *prv_bus(const struct pbl_flash_device *dev) {
  return prv_state(dev)->bus;
}

// Basic register-level helpers
///////////////////////////////////////////////////////////

static int prv_cmd(const struct pbl_flash_device *dev, uint8_t opcode) {
  struct pbl_spi_mem_op op = PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(opcode, 1), PBL_SPI_MEM_OP_NO_ADDR,
                                            PBL_SPI_MEM_OP_NO_DUMMY, PBL_SPI_MEM_OP_NO_DATA);
  return pbl_spi_mem_exec_op(prv_bus(dev), &op);
}

static int prv_read_reg(const struct pbl_flash_device *dev, uint8_t opcode, void *buf, size_t len) {
  struct pbl_spi_mem_op op =
      PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(opcode, 1), PBL_SPI_MEM_OP_NO_ADDR, PBL_SPI_MEM_OP_NO_DUMMY,
                     PBL_SPI_MEM_OP_DATA_IN(len, buf, 1));
  return pbl_spi_mem_exec_op(prv_bus(dev), &op);
}

static int prv_write_reg(const struct pbl_flash_device *dev, uint8_t opcode, const void *buf,
                         size_t len) {
  struct pbl_spi_mem_op op =
      PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(opcode, 1), PBL_SPI_MEM_OP_NO_ADDR, PBL_SPI_MEM_OP_NO_DUMMY,
                     PBL_SPI_MEM_OP_DATA_OUT(len, buf, 1));
  return pbl_spi_mem_exec_op(prv_bus(dev), &op);
}

static int prv_read_sr1(const struct pbl_flash_device *dev, uint8_t *sr1) {
  return prv_read_reg(dev, OP_RDSR1, sr1, 1);
}

static int prv_read_sr2(const struct pbl_flash_device *dev, uint8_t *sr2) {
  return prv_read_reg(dev, OP_RDSR2, sr2, 1);
}

static int prv_busy(const struct pbl_flash_device *dev, bool *busy) {
  uint8_t sr1;
  int ret = prv_read_sr1(dev, &sr1);
  if (ret != 0) {
    return ret;
  }
  *busy = (sr1 & SR1_WIP) != 0;
  return 0;
}

static int prv_wait_idle(const struct pbl_flash_device *dev) {
  bool busy;
  int ret;

  do {
    ret = prv_busy(dev, &busy);
    if (ret != 0) {
      return ret;
    }
  } while (busy);

  return 0;
}

static int prv_write_enable(const struct pbl_flash_device *dev) {
  return prv_cmd(dev, OP_WREN);
}

// Executes an addressed op in chunks the bus can carry.
static int prv_exec_chunked(const struct pbl_flash_device *dev, struct pbl_spi_mem_op *op) {
  const struct pbl_spi_mem_device *bus = prv_bus(dev);
  size_t remaining = op->data.nbytes;

  while (remaining > 0) {
    op->data.nbytes = remaining;
    int ret = pbl_spi_mem_adjust_op_size(bus, op);
    if (ret != 0) {
      return ret;
    }
    PBL_ASSERTN(op->data.nbytes > 0);
    ret = pbl_spi_mem_exec_op(bus, op);
    if (ret != 0) {
      return ret;
    }
    remaining -= op->data.nbytes;
    op->addr.val += op->data.nbytes;
    if (op->data.dir == PBL_SPI_MEM_DATA_IN) {
      op->data.buf.in = (uint8_t *)op->data.buf.in + op->data.nbytes;
    } else {
      op->data.buf.out = (const uint8_t *)op->data.buf.out + op->data.nbytes;
    }
  }

  return 0;
}

// SFDP
///////////////////////////////////////////////////////////

static int prv_read_sfdp(const struct pbl_flash_device *dev, uint32_t addr, void *buf, size_t len) {
  struct pbl_spi_mem_op op =
      PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(OP_RDSFDP, 1), PBL_SPI_MEM_OP_ADDR(3, addr, 1),
                     PBL_SPI_MEM_OP_DUMMY(1, 1), PBL_SPI_MEM_OP_DATA_IN(len, buf, 1));
  return prv_exec_chunked(dev, &op);
}

static uint32_t prv_le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

struct bfpt {
  uint32_t dw[SFDP_BFPT_MAX_DWORDS];
  uint8_t ndwords;
};

//! @return 0 with @p bfpt filled, -ENOENT when the part has no usable SFDP.
static int prv_get_bfpt(const struct pbl_flash_device *dev, struct bfpt *bfpt) {
  uint8_t hdr[16];

  int ret = prv_read_sfdp(dev, 0, hdr, sizeof(hdr));
  if (ret != 0) {
    return ret;
  }
  if (prv_le32(hdr) != SFDP_SIGNATURE) {
    return -ENOENT;
  }

  // First parameter header must be the Basic Flash Parameter Table
  uint16_t id = hdr[8] | (hdr[15] << 8);
  uint8_t ndwords = hdr[11];
  uint32_t ptp = hdr[12] | (hdr[13] << 8) | (hdr[14] << 16);
  if (id != SFDP_BFPT_ID || ndwords == 0) {
    return -ENOENT;
  }

  bfpt->ndwords = MIN(ndwords, SFDP_BFPT_MAX_DWORDS);
  uint8_t raw[SFDP_BFPT_MAX_DWORDS * 4];
  ret = prv_read_sfdp(dev, ptp, raw, bfpt->ndwords * 4);
  if (ret != 0) {
    return ret;
  }
  for (uint8_t i = 0; i < bfpt->ndwords; i++) {
    bfpt->dw[i] = prv_le32(&raw[i * 4]);
  }

  return 0;
}

static struct read_op prv_bfpt_read(uint32_t half, uint8_t opcode_default, uint8_t addr_width,
                                    uint8_t data_width) {
  struct read_op r = {
      .opcode = (half >> 8) & 0xFF,
      .dummy_cycles = (half & 0x1F) + ((half >> 5) & 0x7),
      .addr_width = addr_width,
      .data_width = data_width,
  };
  if (r.opcode == 0) {
    r.opcode = opcode_default;
  }
  return r;
}

static void prv_bfpt_erase_types(const struct bfpt *bfpt, struct erase_type types[4]) {
  for (int i = 0; i < 4; i++) {
    uint32_t dw = bfpt->dw[7 + i / 2];
    uint32_t half = (i % 2) ? (dw >> 16) : (dw & 0xFFFF);
    uint8_t shift = half & 0xFF;
    types[i].opcode = (half >> 8) & 0xFF;
    types[i].size = (shift != 0) ? (1UL << shift) : 0;
    types[i].time_ms = 0;

    if (bfpt->ndwords >= 10 && types[i].size != 0) {
      static const uint32_t units_ms[] = {1, 16, 128, 1000};
      uint32_t t = (bfpt->dw[9] >> (4 + i * 7)) & 0x7F;
      types[i].time_ms = ((t & 0x1F) + 1) * units_ms[(t >> 5) & 0x3];
    }
  }
}

// Probe / configuration
///////////////////////////////////////////////////////////

static bool prv_bus_can(const struct pbl_flash_device *dev, const struct pbl_spi_mem_op *op) {
  return pbl_spi_mem_supports_op(prv_bus(dev), op);
}

static bool prv_bus_can_read(const struct pbl_flash_device *dev, const struct read_op *r,
                             uint8_t addr_nbytes) {
  uint8_t dummy_bytes = r->dummy_cycles * r->addr_width / 8;
  struct pbl_spi_mem_op op = PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(r->opcode, 1),
                                            PBL_SPI_MEM_OP_ADDR(addr_nbytes, 0, r->addr_width),
                                            PBL_SPI_MEM_OP_DUMMY(dummy_bytes, r->addr_width),
                                            PBL_SPI_MEM_OP_DATA_IN(256, NULL, r->data_width));
  return prv_bus_can(dev, &op);
}

static bool prv_bus_can_program(const struct pbl_flash_device *dev, uint8_t opcode,
                                uint8_t data_width, uint8_t addr_nbytes) {
  struct pbl_spi_mem_op op =
      PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(opcode, 1), PBL_SPI_MEM_OP_ADDR(addr_nbytes, 0, 1),
                     PBL_SPI_MEM_OP_NO_DUMMY, PBL_SPI_MEM_OP_DATA_OUT(256, NULL, data_width));
  return prv_bus_can(dev, &op);
}

static int prv_set_quad_enable(const struct pbl_flash_device *dev, enum pbl_flash_nor_qer qer) {
  uint8_t sr[2];
  int ret;

  switch (qer) {
    case PBL_FLASH_NOR_QER_NONE:
      return 0;
    case PBL_FLASH_NOR_QER_S1B6:
      ret = prv_read_sr1(dev, &sr[0]);
      if (ret != 0) {
        return ret;
      }
      if (sr[0] & (1U << 6)) {
        return 0;
      }
      sr[0] |= (1U << 6);
      prv_write_enable(dev);
      ret = prv_write_reg(dev, OP_WRSR, sr, 1);
      break;
    case PBL_FLASH_NOR_QER_S2B1v1:
    case PBL_FLASH_NOR_QER_S2B1v4:
    case PBL_FLASH_NOR_QER_S2B1v5:
      // Writing SR2 requires writing SR1 as well
      ret = prv_read_sr1(dev, &sr[0]);
      if (ret == 0) {
        ret = prv_read_sr2(dev, &sr[1]);
      }
      if (ret != 0) {
        return ret;
      }
      if (sr[1] & SR2_QE) {
        return 0;
      }
      sr[1] |= SR2_QE;
      prv_write_enable(dev);
      ret = prv_write_reg(dev, OP_WRSR, sr, 2);
      break;
    case PBL_FLASH_NOR_QER_S2B1v6:
      ret = prv_read_sr2(dev, &sr[1]);
      if (ret != 0) {
        return ret;
      }
      if (sr[1] & SR2_QE) {
        return 0;
      }
      sr[1] |= SR2_QE;
      prv_write_enable(dev);
      ret = prv_write_reg(dev, OP_WRSR2, &sr[1], 1);
      break;
    default:
      return -ENOTSUP;
  }

  if (ret != 0) {
    return ret;
  }
  return prv_wait_idle(dev);
}

// Picks the widest read and program ops both the part and the bus support.
static int prv_select_ops(const struct pbl_flash_device *dev, const struct bfpt *bfpt,
                          enum pbl_flash_nor_qer qer) {
  struct spi_nor_state *st = prv_state(dev);
  struct read_op candidates[5];
  int n = 0;
  uint32_t dw1 = (bfpt != NULL) ? bfpt->dw[0] : 0;

  if (bfpt != NULL) {
    if (dw1 & BFPT_DW1_READ_1_4_4) {
      candidates[n++] = prv_bfpt_read(bfpt->dw[2] & 0xFFFF, 0xEB, 4, 4);
    }
    if (dw1 & BFPT_DW1_READ_1_1_4) {
      candidates[n++] = prv_bfpt_read(bfpt->dw[2] >> 16, 0x6B, 1, 4);
    }
    if (dw1 & BFPT_DW1_READ_1_2_2) {
      candidates[n++] = prv_bfpt_read(bfpt->dw[3] >> 16, 0xBB, 2, 2);
    }
    if (dw1 & BFPT_DW1_READ_1_1_2) {
      candidates[n++] = prv_bfpt_read(bfpt->dw[3] & 0xFFFF, 0x3B, 1, 2);
    }
  }
  candidates[n++] =
      (struct read_op){.opcode = OP_READ_FAST, .dummy_cycles = 8, .addr_width = 1, .data_width = 1};

  bool quad = false;
  for (int i = 0; i < n; i++) {
    if (prv_bus_can_read(dev, &candidates[i], st->addr_nbytes)) {
      st->read = candidates[i];
      quad = candidates[i].data_width == 4;
      break;
    }
  }

  if (prv_bus_can_program(dev, OP_PP_1_1_4, 4, st->addr_nbytes)) {
    st->pp_opcode = OP_PP_1_1_4;
    st->pp_data_width = 4;
    quad = true;
  } else if (prv_bus_can_program(dev, OP_PP, 1, st->addr_nbytes)) {
    st->pp_opcode = OP_PP;
    st->pp_data_width = 1;
  } else {
    return -ENOTSUP;
  }

  if (quad) {
    return prv_set_quad_enable(dev, qer);
  }
  return 0;
}

static int prv_configure(const struct pbl_flash_device *dev) {
  const struct spi_nor *cfg = prv_cfg(dev);
  const struct pbl_flash_nor_part *part = cfg->part;
  struct spi_nor_state *st = prv_state(dev);
  struct bfpt bfpt;
  const struct bfpt *sfdp = NULL;
  enum pbl_flash_nor_qer qer = (part != NULL) ? part->qer : PBL_FLASH_NOR_QER_NONE;

  // Defaults, refined by SFDP and the part table below
  st->geometry = (struct pbl_flash_geometry){
      .page_size = 256,
      .sector_size = 0x10000,
      .subsector_size = 0x1000,
      .sector_erase_ms = DEFAULT_SECTOR_ERASE_MS,
      .subsector_erase_ms = DEFAULT_SUBSECTOR_ERASE_MS,
  };
  st->erase_sector_opcode = OP_ERASE_64K;
  st->erase_subsector_opcode = OP_ERASE_4K;
  st->en4b_needs_wren = false;
  if (part != NULL) {
    st->geometry = part->geometry;
  }

  if (prv_get_bfpt(dev, &bfpt) == 0) {
    sfdp = &bfpt;
    uint32_t dw2 = bfpt.dw[1];
    uint64_t bits = (dw2 & 0x80000000UL) ? (1ULL << (dw2 & 0x7FFFFFFF)) : ((uint64_t)dw2 + 1);
    st->geometry.size = (uint32_t)(bits / 8);

    if (bfpt.ndwords >= 11) {
      st->geometry.page_size = BFPT_DW11_PAGE_SIZE(bfpt.dw[10]);
    }
    if (bfpt.ndwords >= 15) {
      qer = (enum pbl_flash_nor_qer)BFPT_DW15_QER(bfpt.dw[14]);
    }
    if (bfpt.ndwords >= 16) {
      st->en4b_needs_wren = (bfpt.dw[15] & BFPT_DW16_EN4B_WREN_B7) != 0;
    }

    // Smallest erase is the subsector, largest the sector
    struct erase_type types[4];
    prv_bfpt_erase_types(&bfpt, types);
    const struct erase_type *small = NULL;
    const struct erase_type *large = NULL;
    for (int i = 0; i < 4; i++) {
      if (types[i].size == 0) {
        continue;
      }
      if (small == NULL || types[i].size < small->size) {
        small = &types[i];
      }
      if (large == NULL || types[i].size > large->size) {
        large = &types[i];
      }
    }
    if (small != NULL && large != NULL && small != large) {
      st->geometry.subsector_size = small->size;
      st->erase_subsector_opcode = small->opcode;
      st->geometry.sector_size = large->size;
      st->erase_sector_opcode = large->opcode;
      if (small->time_ms != 0) {
        st->geometry.subsector_erase_ms = small->time_ms;
      }
      if (large->time_ms != 0) {
        st->geometry.sector_erase_ms = large->time_ms;
      }
    }
    PBL_LOG_DBG("SFDP: %" PRIu32 " bytes, page %" PRIu32 ", erase %" PRIu32 "/%" PRIu32,
                st->geometry.size, st->geometry.page_size, st->geometry.subsector_size,
                st->geometry.sector_size);
  } else if (part == NULL) {
    PBL_LOG_ERR("No SFDP and no part table");
    return -ENODEV;
  }

  if (st->geometry.size == 0) {
    return -ENODEV;
  }

  st->addr_nbytes = (st->geometry.size > ADDR_4BYTE_THRESHOLD) ? 4 : 3;
  if (st->addr_nbytes == 4) {
    if (st->en4b_needs_wren) {
      prv_write_enable(dev);
    }
    int ret = prv_cmd(dev, OP_EN4B);
    if (ret != 0) {
      return ret;
    }
  }

  return prv_select_ops(dev, sfdp, qer);
}

// Some parts ship with block protection bits set
static int prv_clear_block_protect(const struct pbl_flash_device *dev) {
  uint8_t sr1;

  int ret = prv_read_sr1(dev, &sr1);
  if (ret != 0 || (sr1 & SR1_BP_MASK) == 0) {
    return ret;
  }

  sr1 &= ~SR1_BP_MASK;
  ret = prv_write_enable(dev);
  if (ret == 0) {
    ret = prv_write_reg(dev, OP_WRSR, &sr1, 1);
  }
  if (ret == 0) {
    ret = prv_wait_idle(dev);
  }
  return ret;
}

static int prv_init(const struct pbl_flash_device *dev) {
  const struct spi_nor *cfg = prv_cfg(dev);
  struct spi_nor_state *st = prv_state(dev);
  int ret;

  if (st->initialized) {
    pbl_spi_mem_set_polling(prv_bus(dev), dev->state->coredump);
    return 0;
  }

  st->bus = SPI_MEM_NOR;
  ret = pbl_spi_mem_init(prv_bus(dev));
  if (ret != 0) {
    return ret;
  }

  // Reset the part to stop any program or erase in progress from before
  // reboot, unless we are executing from it.
  if (!prv_bus(dev)->xip) {
    prv_cmd(dev, OP_RESET_EN);
    prv_cmd(dev, OP_RESET);
    psleep((cfg->part != NULL) ? cfg->part->reset_latency_ms : DEFAULT_RESET_LATENCY_MS);
  }

  uint8_t id[3];
  ret = prv_read_reg(dev, OP_RDID, id, sizeof(id));
  if (ret != 0) {
    return ret;
  }
  uint32_t jedec_id = id[0] | (id[1] << 8) | (id[2] << 16);
  if (cfg->part != NULL && jedec_id != cfg->part->id) {
    PBL_LOG_ERR("Flash is not %s (id: 0x%06" PRIx32 ")", cfg->part->name, jedec_id);
  }

  ret = prv_configure(dev);
  if (ret == 0) {
    ret = prv_clear_block_protect(dev);
  }
  if (ret != 0) {
    return ret;
  }

  st->initialized = true;

  return 0;
}

// Flash ops
///////////////////////////////////////////////////////////

static int prv_read(const struct pbl_flash_device *dev, uint32_t addr, void *buf, size_t len) {
  struct spi_nor_state *st = prv_state(dev);

  addr -= dev->base;
  if (pbl_spi_mem_has_dirmap(prv_bus(dev))) {
    return pbl_spi_mem_dirmap_read(prv_bus(dev), addr, buf, len);
  }

  uint8_t dummy_bytes = st->read.dummy_cycles * st->read.addr_width / 8;
  struct pbl_spi_mem_op op =
      PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(st->read.opcode, 1),
                     PBL_SPI_MEM_OP_ADDR(st->addr_nbytes, addr, st->read.addr_width),
                     PBL_SPI_MEM_OP_DUMMY(dummy_bytes, st->read.addr_width),
                     PBL_SPI_MEM_OP_DATA_IN(len, buf, st->read.data_width));
  return prv_exec_chunked(dev, &op);
}

static int prv_write(const struct pbl_flash_device *dev, uint32_t addr, const void *buf,
                     size_t len) {
  struct spi_nor_state *st = prv_state(dev);
  const uint8_t *src = buf;

  addr -= dev->base;

  while (len > 0) {
    size_t chunk = MIN(len, st->geometry.page_size - (addr % st->geometry.page_size));
    struct pbl_spi_mem_op op = PBL_SPI_MEM_OP(
        PBL_SPI_MEM_OP_CMD(st->pp_opcode, 1), PBL_SPI_MEM_OP_ADDR(st->addr_nbytes, addr, 1),
        PBL_SPI_MEM_OP_NO_DUMMY, PBL_SPI_MEM_OP_DATA_OUT(chunk, src, st->pp_data_width));

    // One page program per transaction, even when the bus splits it
    int ret = pbl_spi_mem_adjust_op_size(prv_bus(dev), &op);
    if (ret != 0) {
      return ret;
    }
    chunk = op.data.nbytes;

    ret = prv_write_enable(dev);
    if (ret == 0) {
      ret = pbl_spi_mem_exec_op(prv_bus(dev), &op);
    }
    if (ret == 0) {
      ret = prv_wait_idle(dev);
    }
    if (ret != 0) {
      return ret;
    }

    addr += chunk;
    src += chunk;
    len -= chunk;
  }

  return 0;
}

static int prv_erase_begin(const struct pbl_flash_device *dev, uint32_t addr, size_t size) {
  struct spi_nor_state *st = prv_state(dev);
  uint8_t opcode;

  addr -= dev->base;

  if (size == st->geometry.sector_size) {
    opcode = st->erase_sector_opcode;
  } else if (size == st->geometry.subsector_size) {
    opcode = st->erase_subsector_opcode;
  } else {
    return -EINVAL;
  }

  int ret = prv_write_enable(dev);
  if (ret != 0) {
    return ret;
  }

  struct pbl_spi_mem_op op =
      PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(opcode, 1), PBL_SPI_MEM_OP_ADDR(st->addr_nbytes, addr, 1),
                     PBL_SPI_MEM_OP_NO_DUMMY, PBL_SPI_MEM_OP_NO_DATA);
  ret = pbl_spi_mem_exec_op(prv_bus(dev), &op);
  if (ret != 0) {
    return ret;
  }

  // An XIP bus only returns once the part is readable again
  return prv_bus(dev)->xip ? 1 : 0;
}

static int prv_erase_status(const struct pbl_flash_device *dev) {
  bool busy;
  uint8_t sr2;

  int ret = prv_busy(dev, &busy);
  if (ret != 0) {
    return ret;
  }
  if (busy) {
    return -EBUSY;
  }

  ret = prv_read_sr2(dev, &sr2);
  if (ret != 0) {
    return ret;
  }
  if (sr2 & SR2_SUS) {
    return -EAGAIN;
  }

  return 0;
}

static int prv_erase_suspend(const struct pbl_flash_device *dev) {
  const struct pbl_flash_nor_part *part = prv_cfg(dev)->part;
  bool busy;

  int ret = prv_busy(dev, &busy);
  if (ret != 0) {
    return ret;
  }
  if (!busy) {
    return 1;
  }

  ret = prv_cmd(dev, OP_ERASE_SUSPEND);
  if (ret != 0) {
    return ret;
  }
  delay_us((part != NULL) ? part->suspend_to_read_latency_us : DEFAULT_SUSPEND_LATENCY_US);

  return 0;
}

static int prv_erase_resume(const struct pbl_flash_device *dev) {
  return prv_cmd(dev, OP_ERASE_RESUME);
}

static void prv_power_down(const struct pbl_flash_device *dev) {
  const struct spi_nor *cfg = prv_cfg(dev);

  prv_cmd(dev, OP_DPD);
  delay_us((cfg->part != NULL) ? cfg->part->standby_to_low_power_latency_us
                               : DEFAULT_DPD_LATENCY_US);
  pbl_spi_mem_set_power(prv_bus(dev), false);
}

static void prv_power_up(const struct pbl_flash_device *dev) {
  const struct spi_nor *cfg = prv_cfg(dev);

  pbl_spi_mem_set_power(prv_bus(dev), true);
  prv_cmd(dev, OP_RDPD);
  delay_us((cfg->part != NULL) ? cfg->part->low_power_to_standby_latency_us
                               : DEFAULT_DPD_LATENCY_US);
}

// Security registers (part table required: SFDP does not describe them)
///////////////////////////////////////////////////////////

static int prv_sec_reg_check(const struct pbl_flash_device *dev, uint32_t addr) {
  const struct pbl_flash_sec_regs *regs = dev->sec_regs;

  if (regs == NULL || regs->count == 0) {
    return -ENOTSUP;
  }
  for (uint8_t i = 0; i < regs->count; i++) {
    if (addr >= regs->addrs[i] && addr < regs->addrs[i] + regs->size) {
      return 0;
    }
  }
  return -EINVAL;
}

static int prv_sec_reg_read(const struct pbl_flash_device *dev, uint32_t addr, uint8_t *val) {
  int ret = prv_sec_reg_check(dev, addr);
  if (ret != 0) {
    return ret;
  }

  struct pbl_spi_mem_op op = PBL_SPI_MEM_OP(
      PBL_SPI_MEM_OP_CMD(OP_READ_SEC, 1), PBL_SPI_MEM_OP_ADDR(prv_state(dev)->addr_nbytes, addr, 1),
      PBL_SPI_MEM_OP_DUMMY(1, 1), PBL_SPI_MEM_OP_DATA_IN(1, val, 1));
  return pbl_spi_mem_exec_op(prv_bus(dev), &op);
}

static int prv_sec_reg_write(const struct pbl_flash_device *dev, uint32_t addr, uint8_t val) {
  int ret = prv_sec_reg_check(dev, addr);
  if (ret != 0) {
    return ret;
  }

  struct pbl_spi_mem_op op =
      PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(OP_PROGRAM_SEC, 1),
                     PBL_SPI_MEM_OP_ADDR(prv_state(dev)->addr_nbytes, addr, 1),
                     PBL_SPI_MEM_OP_NO_DUMMY, PBL_SPI_MEM_OP_DATA_OUT(1, &val, 1));
  ret = prv_write_enable(dev);
  if (ret == 0) {
    ret = pbl_spi_mem_exec_op(prv_bus(dev), &op);
  }
  if (ret == 0) {
    ret = prv_wait_idle(dev);
  }
  return ret;
}

static int prv_sec_reg_erase(const struct pbl_flash_device *dev, uint32_t addr) {
  int ret = prv_sec_reg_check(dev, addr);
  if (ret != 0) {
    return ret;
  }

  struct pbl_spi_mem_op op =
      PBL_SPI_MEM_OP(PBL_SPI_MEM_OP_CMD(OP_ERASE_SEC, 1),
                     PBL_SPI_MEM_OP_ADDR(prv_state(dev)->addr_nbytes, addr, 1),
                     PBL_SPI_MEM_OP_NO_DUMMY, PBL_SPI_MEM_OP_NO_DATA);
  ret = prv_write_enable(dev);
  if (ret == 0) {
    ret = pbl_spi_mem_exec_op(prv_bus(dev), &op);
  }
  if (ret == 0) {
    ret = prv_wait_idle(dev);
  }
  return ret;
}

static int prv_sec_reg_is_locked(const struct pbl_flash_device *dev, uint32_t addr, bool *locked) {
  uint8_t sr2;

  int ret = prv_sec_reg_check(dev, addr);
  if (ret == 0) {
    ret = prv_read_sr2(dev, &sr2);
  }
  if (ret != 0) {
    return ret;
  }
  *locked = (sr2 & SEC_ADDR_TO_LB(addr)) != 0;
  return 0;
}

#ifdef CONFIG_RECOVERY_FW
static int prv_sec_reg_lock(const struct pbl_flash_device *dev, uint32_t addr) {
  uint8_t sr[2];

  int ret = prv_sec_reg_check(dev, addr);
  if (ret == 0) {
    ret = prv_read_sr1(dev, &sr[0]);
  }
  if (ret == 0) {
    ret = prv_read_sr2(dev, &sr[1]);
  }
  if (ret != 0) {
    return ret;
  }
  sr[1] |= SEC_ADDR_TO_LB(addr);
  ret = prv_write_enable(dev);
  if (ret == 0) {
    ret = prv_write_reg(dev, OP_WRSR, sr, 2);
  }
  if (ret == 0) {
    ret = prv_wait_idle(dev);
  }
  return ret;
}
#endif

static const struct pbl_flash_ops s_ops = {
    .init = prv_init,
    .read = prv_read,
    .write = prv_write,
    .erase_begin = prv_erase_begin,
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

#if defined(CONFIG_SPI_MEM_SF32LB52_MPI)
#define FLASH_BASE_(n) FLASH##n##_BASE_ADDR
#define FLASH_BASE(n) FLASH_BASE_(n)
#define NOR_BASE FLASH_BASE(CONFIG_SF32LB52_MPI_INSTANCE)
#else
#define NOR_BASE 0
#endif

static struct spi_nor_state s_flash_state;
static const struct spi_nor s_flash = {
    .dev =
        {
            .state = &s_flash_state.flash,
            .ops = &s_ops,
            .base = NOR_BASE,
            .geometry = &s_flash_state.geometry,
#ifdef PBL_FLASH_NOR_PART
            .sec_regs = &PBL_FLASH_NOR_PART.sec_regs,
#endif
        },
#ifdef PBL_FLASH_NOR_PART
    .part = &PBL_FLASH_NOR_PART,
#endif
};
const struct pbl_flash_device *const FLASH = &s_flash.dev;

#if UNITTEST
void spi_nor_reset_for_test(void) {
  memset(&s_flash_state, 0, sizeof(s_flash_state));
}
#endif

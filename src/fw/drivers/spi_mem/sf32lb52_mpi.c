/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/spi_mem.h>

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include "pbl/kernel/irq.h"
#include "pbl/mcu/cache.h"
#include "pbl/util/math.h"
#include "pbl/util/misc.h"
#include "system/passert.h"

#include <bf0_hal.h>

// Native driver for the SF32LB52 MPI controller in QSPI NOR mode, after the
// Zephyr sifli,sf32lb-mpi-qspi-nor driver. The CPU executes from the memory
// behind this controller, so this file lives in RAM (see ramfunc.ld) and any
// op that leaves the part busy waits for it to become readable again, with
// interrupts disabled, before returning.

#define MPI_INSTANCE_(n) FLASH##n
#define MPI_INSTANCE(n) MPI_INSTANCE_(n)
#define MPI_MEM_BASE_(n) FLASH##n##_BASE_ADDR
#define MPI_MEM_BASE(n) MPI_MEM_BASE_(n)
#define DMA_CHANNEL_(n) DMA1_Channel##n
#define DMA_CHANNEL(n) DMA_CHANNEL_(n)
#define DMA_REQUEST_(n) DMA_REQUEST_##n
#define DMA_REQUEST(n) DMA_REQUEST_(n)

#define FIFO_SIZE 64U
#define PAGE_SIZE 256U
#define MAX_DUMMY_CYCLES 31U

#define CCR_IMODE(m) ((uint32_t)(m) << MPI_CCR1_IMODE_Pos)
#define CCR_ADMODE(m) ((uint32_t)(m) << MPI_CCR1_ADMODE_Pos)
#define CCR_ADSIZE(n) ((uint32_t)((n) - 1U) << MPI_CCR1_ADSIZE_Pos)
#define CCR_ABMODE(m) ((uint32_t)(m) << MPI_CCR1_ABMODE_Pos)
#define CCR_ABSIZE(n) ((uint32_t)((n) - 1U) << MPI_CCR1_ABSIZE_Pos)
#define CCR_DCYC(n) ((uint32_t)(n) << MPI_CCR1_DCYC_Pos)
#define CCR_DMODE(m) ((uint32_t)(m) << MPI_CCR1_DMODE_Pos)
#define CCR_FMODE_WRITE (1UL << MPI_CCR1_FMODE_Pos)

#define MODE_NONE 0U
#define MODE_SINGLE 1U

#define OP_RDSR 0x05
#define SR_WIP 0x01

struct sf32lb52_mpi_state {
  struct pbl_spi_mem_device_state spi_mem;
  DMA_HandleTypeDef hdma;
};

struct sf32lb52_mpi {
  struct pbl_spi_mem_device dev;
  MPI_TypeDef *mpi;
  uintptr_t mem_base;
  DMA_Channel_TypeDef *dma_channel;
  uint32_t dma_request;
  uint8_t psclr;
  bool invert_rx_clk;
};

// Sources the DMA cannot read from go through here.
static uint8_t __attribute__((aligned(4))) s_bounce_buf[PAGE_SIZE];

static inline const struct sf32lb52_mpi *prv_cfg(const struct pbl_spi_mem_device *dev) {
  return container_of(dev, const struct sf32lb52_mpi, dev);
}

static inline struct sf32lb52_mpi_state *prv_state(const struct pbl_spi_mem_device *dev) {
  return container_of(dev->state, struct sf32lb52_mpi_state, spi_mem);
}

static uint32_t prv_mode(uint8_t buswidth) {
  switch (buswidth) {
    case 0:
      return MODE_NONE;
    case 1:
      return MODE_SINGLE;
    case 2:
      return 2U;
    default:
      return 3U;
  }
}

static uint32_t prv_dummy_cycles(const struct pbl_spi_mem_op *op) {
  return (op->dummy.buswidth != 0) ? (op->dummy.nbytes * 8U / op->dummy.buswidth) : 0;
}

// Encodes an op as a CCRx value. When the address phase is followed by dummy
// cycles, the first byte's worth is sent as an "alternate byte" of 0xFF so the
// part never sees a continuous-read mode pattern on the bus.
static uint32_t prv_ccr(const struct pbl_spi_mem_op *op) {
  uint32_t ccr = CCR_IMODE(prv_mode(op->cmd.buswidth));
  uint32_t dummy = prv_dummy_cycles(op);

  if (op->addr.nbytes != 0) {
    uint32_t alt_cycles = 8U / op->addr.buswidth;

    ccr |= CCR_ADMODE(prv_mode(op->addr.buswidth)) | CCR_ADSIZE(op->addr.nbytes);
    if (dummy >= alt_cycles) {
      ccr |= CCR_ABMODE(prv_mode(op->addr.buswidth)) | CCR_ABSIZE(1);
      dummy -= alt_cycles;
    }
  }
  ccr |= CCR_DCYC(dummy);

  if (op->data.nbytes != 0) {
    ccr |= CCR_DMODE(prv_mode(op->data.buswidth));
    if (op->data.dir == PBL_SPI_MEM_DATA_OUT) {
      ccr |= CCR_FMODE_WRITE;
    }
  }

  return ccr;
}

static bool prv_width_ok(uint8_t w) {
  return w == 0 || w == 1 || w == 2 || w == 4;
}

static bool prv_supports_op(const struct pbl_spi_mem_device *dev, const struct pbl_spi_mem_op *op) {
  if (op->cmd.buswidth != 1 || !prv_width_ok(op->addr.buswidth) ||
      !prv_width_ok(op->dummy.buswidth) || !prv_width_ok(op->data.buswidth)) {
    return false;
  }
  if (op->addr.nbytes > 4) {
    return false;
  }
  if (prv_dummy_cycles(op) > MAX_DUMMY_CYCLES + 8U) {
    return false;
  }
  // Reads without an address cannot be split into FIFO-sized chunks
  if (op->data.dir == PBL_SPI_MEM_DATA_IN && op->addr.nbytes == 0 && op->data.nbytes > FIFO_SIZE) {
    return false;
  }
  return true;
}

static int prv_adjust_op_size(const struct pbl_spi_mem_device *dev, struct pbl_spi_mem_op *op) {
  if (op->data.dir == PBL_SPI_MEM_DATA_OUT) {
    op->data.nbytes = MIN(op->data.nbytes, PAGE_SIZE);
  }
  return 0;
}

// Low level: everything below may run while the part is busy and must not
// touch the flash.
///////////////////////////////////////////////////////////

static void prv_wait_tcf(MPI_TypeDef *mpi) {
  while ((mpi->SR & MPI_SR_TCF) == 0) {
  }
  mpi->SCR = MPI_SCR_TCFC;
}

static void prv_issue(MPI_TypeDef *mpi, uint8_t opcode, uint32_t ccr, uint32_t addr) {
  mpi->CCR1 = ccr;
  mpi->AR1 = addr;
  mpi->CMDR1 = opcode;
  prv_wait_tcf(mpi);
}

// Issues a command and, through the controller's second command slot,
// polls RDSR until WIP clears. The part is unreadable in between.
static void prv_issue_ready_wait(MPI_TypeDef *mpi, uint8_t opcode, uint32_t ccr, uint32_t addr) {
  uint32_t cr;

  mpi->CCR2 = CCR_IMODE(MODE_SINGLE) | CCR_DMODE(MODE_SINGLE);
  mpi->CMDR2 = OP_RDSR;
  mpi->DLR2 = 0;
  mpi->SMKR = SR_WIP;
  mpi->SMR = 0;

  cr = mpi->CR;
  cr |= MPI_CR_CMD2E | MPI_CR_SME2;
  mpi->CR = cr;

  mpi->AR1 = addr;
  mpi->CCR1 = ccr;
  mpi->CMDR1 = opcode;
  while ((mpi->SR & MPI_SR_SMF) == 0) {
  }
  mpi->SCR = MPI_SCR_SMFC | MPI_SCR_TCFC;

  cr &= ~(MPI_CR_CMD2E | MPI_CR_SME2);
  mpi->CR = cr;
}

static void prv_fifo_push(MPI_TypeDef *mpi, const uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; i += 4U) {
    uint32_t word = 0xFFFFFFFFUL;
    size_t n = MIN(4U, len - i);
    for (size_t j = 0; j < n; j++) {
      word = (word & ~(0xFFUL << (8U * j))) | ((uint32_t)buf[i + j] << (8U * j));
    }
    mpi->DR = word;
  }
}

static void prv_fifo_pop(MPI_TypeDef *mpi, uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; i += 4U) {
    uint32_t word = mpi->DR;
    size_t n = MIN(4U, len - i);
    for (size_t j = 0; j < n; j++) {
      buf[i + j] = (word >> (8U * j)) & 0xFF;
    }
  }
}

static void prv_read(MPI_TypeDef *mpi, const struct pbl_spi_mem_op *op, uint32_t ccr) {
  uint8_t *buf = op->data.buf.in;
  uint32_t addr = op->addr.val;
  size_t len = op->data.nbytes;

  do {
    size_t chunk = MIN(len, FIFO_SIZE);

    mpi->DLR1 = chunk - 1U;
    prv_issue(mpi, op->cmd.opcode, ccr, addr);
    prv_fifo_pop(mpi, buf, chunk);

    len -= chunk;
    buf += chunk;
    addr += chunk;
  } while (len > 0);
}

static bool prv_dma_can_read(const struct sf32lb52_mpi *cfg, const void *buf, size_t len) {
  return !IS_SPI_NONDMA_RAM_ADDR(buf) && !IS_DMA_ACCROSS_1M_BOUNDARY((uint32_t)buf, len) &&
         !IS_SAME_FLASH_ADDR(buf, cfg->mem_base);
}

static int prv_write(const struct pbl_spi_mem_device *dev, const struct pbl_spi_mem_op *op,
                     uint32_t ccr) {
  const struct sf32lb52_mpi *cfg = prv_cfg(dev);
  struct sf32lb52_mpi_state *state = prv_state(dev);
  MPI_TypeDef *mpi = cfg->mpi;
  const uint8_t *src = op->data.buf.out;
  size_t len = op->data.nbytes;
  bool use_dma = len > FIFO_SIZE;
  HAL_StatusTypeDef res = HAL_OK;

  if (use_dma) {
    if (!prv_dma_can_read(cfg, src, len)) {
      memcpy(s_bounce_buf, src, len);
      src = s_bounce_buf;
    }
    uintptr_t flush_addr = (uintptr_t)src;
    size_t flush_size = len;
    dcache_align(&flush_addr, &flush_size);
    dcache_flush((const void *)flush_addr, flush_size);

    res = HAL_DMA_DeInit(&state->hdma);
    if (res == HAL_OK) {
      res = HAL_DMA_Init(&state->hdma);
    }
    if (res != HAL_OK) {
      return -EIO;
    }
    mpi->CR |= MPI_CR_DMAE;
    mpi->DLR1 = len - 1U;
    res = HAL_DMA_Start(&state->hdma, (uint32_t)src, (uint32_t)&mpi->DR, len);
    if (res != HAL_OK) {
      mpi->CR &= ~MPI_CR_DMAE;
      return -EIO;
    }
  } else {
    mpi->DLR1 = len - 1U;
    prv_fifo_push(mpi, src, len);
  }

  // Programs and register writes leave the part busy
  pbl_irq_lock();
  prv_issue_ready_wait(mpi, op->cmd.opcode, ccr, op->addr.val);
  pbl_irq_unlock();

  if (use_dma) {
    res = HAL_DMA_PollForTransfer(&state->hdma, HAL_DMA_FULL_TRANSFER, 1000);
    mpi->CR &= ~MPI_CR_DMAE;
  }

  if (op->addr.nbytes != 0) {
    void *mapped = (void *)(cfg->mem_base + op->addr.val);
    SCB_InvalidateDCache_by_Addr(mapped, len);
    SCB_InvalidateICache_by_Addr(mapped, len);
  }

  return (res == HAL_OK) ? 0 : -EIO;
}

static int prv_exec_op(const struct pbl_spi_mem_device *dev, const struct pbl_spi_mem_op *op) {
  MPI_TypeDef *mpi = prv_cfg(dev)->mpi;
  uint32_t ccr = prv_ccr(op);

  switch (op->data.dir) {
    case PBL_SPI_MEM_DATA_IN:
      prv_read(mpi, op, ccr);
      return 0;
    case PBL_SPI_MEM_DATA_OUT:
      return prv_write(dev, op, ccr);
    default:
      break;
  }

  if (op->addr.nbytes != 0) {
    // Addressed commands without data are erases: wait for the part, then
    // drop whatever the caches hold of the erased range.
    pbl_irq_lock();
    prv_issue_ready_wait(mpi, op->cmd.opcode, ccr, op->addr.val);
    pbl_irq_unlock();
    dcache_invalidate_all();
    icache_invalidate_all();
  } else {
    prv_issue(mpi, op->cmd.opcode, ccr, 0);
  }

  return 0;
}

static int prv_dirmap_read(const struct pbl_spi_mem_device *dev, uint32_t addr, void *buf,
                           size_t len) {
  memcpy(buf, (const void *)(prv_cfg(dev)->mem_base + addr), len);
  return 0;
}

static int prv_init(const struct pbl_spi_mem_device *dev) {
  const struct sf32lb52_mpi *cfg = prv_cfg(dev);
  struct sf32lb52_mpi_state *state = prv_state(dev);
  MPI_TypeDef *mpi = cfg->mpi;
  uint32_t val;

  // The boot ROM already configured the memory-mapped read path we are
  // executing from; only the indirect command path is set up here.
  mpi->TIMR = 0xFF;
  mpi->CIR = 0x50005000;
  mpi->ABR1 = 0xFF;
  mpi->HRABR = 0xFF;

  val = mpi->FIFOCR;
  val &= ~MPI_FIFOCR_TXSLOTS_Msk;
  val |= 1UL << MPI_FIFOCR_TXSLOTS_Pos;
  mpi->FIFOCR = val;

  val = mpi->MISCR;
  val &= ~MPI_MISCR_RXCLKINV_Msk;
  val |= (cfg->invert_rx_clk ? 1UL : 0UL) << MPI_MISCR_RXCLKINV_Pos;
  mpi->MISCR = val;

  mpi->PSCLR = cfg->psclr;

  val = mpi->CR;
  val &= ~MPI_CR_DFM;
  val |= MPI_CR_EN;
  mpi->CR = val;

  state->hdma.Instance = cfg->dma_channel;
  state->hdma.Init.Request = cfg->dma_request;
  state->hdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
  state->hdma.Init.PeriphInc = DMA_PINC_DISABLE;
  state->hdma.Init.MemInc = DMA_MINC_ENABLE;
  state->hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  state->hdma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  state->hdma.Init.Mode = DMA_NORMAL;
  state->hdma.Init.Priority = DMA_PRIORITY_MEDIUM;
  state->hdma.Init.BurstSize = 0;

  return 0;
}

static const struct pbl_spi_mem_ops s_ops = {
    .init = prv_init,
    .supports_op = prv_supports_op,
    .adjust_op_size = prv_adjust_op_size,
    .exec_op = prv_exec_op,
    .dirmap_read = prv_dirmap_read,
};

static struct sf32lb52_mpi_state s_mpi_state;
static const struct sf32lb52_mpi s_mpi = {
    .dev =
        {
            .state = &s_mpi_state.spi_mem,
            .ops = &s_ops,
            .xip = true,
        },
    .mpi = MPI_INSTANCE(CONFIG_SF32LB52_MPI_INSTANCE),
    .mem_base = MPI_MEM_BASE(CONFIG_SF32LB52_MPI_INSTANCE),
    .dma_channel = DMA_CHANNEL(CONFIG_SF32LB52_MPI_DMA_CHANNEL),
    .dma_request = DMA_REQUEST(CONFIG_SF32LB52_MPI_DMA_REQUEST),
    .psclr = CONFIG_SF32LB52_MPI_CLK_DIV,
#ifdef CONFIG_SF32LB52_MPI_INVERT_RX_CLK
    .invert_rx_clk = true,
#endif
};
const struct pbl_spi_mem_device *const SPI_MEM_NOR = &s_mpi.dev;

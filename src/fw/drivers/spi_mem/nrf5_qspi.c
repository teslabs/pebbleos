/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/spi_mem.h>

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <pbl/logging/logging.h>
#include "pbl/kernel/sem.h"
#include "pbl/soc/nrf/sleep.h"
#include "pbl/util/math.h"
#include "pbl/util/misc.h"
#include "system/passert.h"

#include <hal/nrf_qspi.h>
#include <nrfx.h>

PBL_LOG_MODULE_DECLARE(driver_flash, CONFIG_DRIVER_FLASH_LOG_LEVEL);

// The nRF52 QSPI peripheral has three kinds of transactions:
//  - custom instructions: a single-line opcode plus up to 8 data bytes, no
//    separate address phase (address bytes go out as data);
//  - READ/WRITE/ERASE tasks with a hardwired opcode per configured mode, an
//    address, and the peripheral's own dummy cycles;
// so supports_op() only admits ops that map onto one of those.

// NOTE: This driver does not cover anomaly 244, which may cause data corruption
// if HF clock source is switching between HFXO and HFINT (e.g. by BLE). This
// issue has not been observed at operating speeds <= 8MHz, therefore, no
// workaround is implemented here. A warning log will be emitted if driver is
// initialized at higher frequencies.

// Interrupt-driven transfers are only worth the context switch above this size.
#define MIN_IRQ_XFER_SIZE 256U

#define CINSTR_MAX_DATA 8U

// Word-aligned bounce buffer for data that is not in RAM (the peripheral can
// only DMA from RAM).
static uint8_t __attribute__((aligned(4))) s_bounce_buf[32];

struct nrf5_qspi_state {
  struct pbl_spi_mem_device_state spi_mem;
  struct pbl_sem sem;
  nrf_qspi_readoc_t readoc;
  nrf_qspi_writeoc_t writeoc;
  nrf_qspi_addrmode_t addrmode;
  bool active;
};

struct nrf5_qspi {
  struct pbl_spi_mem_device dev;
  uint32_t clk_freq_hz;
  uint32_t csn_pin;
  uint32_t sck_pin;
  uint32_t io_pins[4];
};

static struct nrf5_qspi_state *s_state;

static inline const struct nrf5_qspi *prv_cfg(const struct pbl_spi_mem_device *dev) {
  return container_of(dev, const struct nrf5_qspi, dev);
}

static inline struct nrf5_qspi_state *prv_state(const struct pbl_spi_mem_device *dev) {
  return container_of(dev->state, struct nrf5_qspi_state, spi_mem);
}

void QSPI_IRQHandler(void) {
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  pbl_sem_give(&s_state->sem);
}

static void prv_wait_ready_polling(void) {
  while (!nrf_qspi_event_check(NRF_QSPI, NRF_QSPI_EVENT_READY)) {
  }
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
}

// Workaround for nRF52 Anomaly 215: trigger TASKS_ACTIVATE and wait for
// EVENTS_READY before accessing any QSPI register with an offset above 0x600.
static void prv_workaround_215_apply(void) {
  nrf_qspi_pins_t pins;
  nrf_qspi_pins_t disconnected_pins = {
      .sck_pin = NRF_QSPI_PIN_NOT_CONNECTED,
      .csn_pin = NRF_QSPI_PIN_NOT_CONNECTED,
      .io0_pin = NRF_QSPI_PIN_NOT_CONNECTED,
      .io1_pin = NRF_QSPI_PIN_NOT_CONNECTED,
      .io2_pin = NRF_QSPI_PIN_NOT_CONNECTED,
      .io3_pin = NRF_QSPI_PIN_NOT_CONNECTED,
  };

  // Disconnect pins to not wait for response from external memory
  nrf_qspi_pins_get(NRF_QSPI, &pins);
  nrf_qspi_pins_set(NRF_QSPI, &disconnected_pins);

  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_ACTIVATE);
  prv_wait_ready_polling();

  nrf_qspi_pins_set(NRF_QSPI, &pins);
}

// Opcodes the READ/WRITE/ERASE tasks issue for each configured mode.
static bool prv_readoc_for(uint8_t opcode, nrf_qspi_readoc_t *readoc) {
  switch (opcode) {
    case 0x0B:
      *readoc = NRF_QSPI_READOC_FASTREAD;
      return true;
    case 0x3B:
      *readoc = NRF_QSPI_READOC_READ2O;
      return true;
    case 0xBB:
      *readoc = NRF_QSPI_READOC_READ2IO;
      return true;
    case 0x6B:
      *readoc = NRF_QSPI_READOC_READ4O;
      return true;
    case 0xEB:
      *readoc = NRF_QSPI_READOC_READ4IO;
      return true;
    default:
      return false;
  }
}

static bool prv_writeoc_for(uint8_t opcode, nrf_qspi_writeoc_t *writeoc) {
  switch (opcode) {
    case 0x02:
      *writeoc = NRF_QSPI_WRITEOC_PP;
      return true;
    case 0xA2:
      *writeoc = NRF_QSPI_WRITEOC_PP2O;
      return true;
    case 0x32:
      *writeoc = NRF_QSPI_WRITEOC_PP4O;
      return true;
    case 0x38:
      *writeoc = NRF_QSPI_WRITEOC_PP4IO;
      return true;
    default:
      return false;
  }
}

static bool prv_erase_len_for(uint8_t opcode, nrf_qspi_erase_len_t *len) {
  switch (opcode) {
    case 0x20:
      *len = NRF_QSPI_ERASE_LEN_4KB;
      return true;
    case 0xD8:
      *len = NRF_QSPI_ERASE_LEN_64KB;
      return true;
    case 0xC7:
      *len = NRF_QSPI_ERASE_LEN_ALL;
      return true;
    default:
      return false;
  }
}

static bool prv_addr_ok(const struct pbl_spi_mem_op *op) {
  return op->addr.nbytes == 3 || op->addr.nbytes == 4;
}

static bool prv_is_cinstr(const struct pbl_spi_mem_op *op) {
  return op->cmd.buswidth == 1 && op->addr.buswidth <= 1 && op->dummy.buswidth <= 1 &&
         op->data.buswidth <= 1 &&
         op->addr.nbytes + op->dummy.nbytes + op->data.nbytes <= CINSTR_MAX_DATA;
}

static bool prv_is_read(const struct pbl_spi_mem_op *op) {
  nrf_qspi_readoc_t readoc;

  return op->data.dir == PBL_SPI_MEM_DATA_IN && prv_addr_ok(op) &&
         prv_readoc_for(op->cmd.opcode, &readoc);
}

static bool prv_is_program(const struct pbl_spi_mem_op *op) {
  nrf_qspi_writeoc_t writeoc;

  return op->data.dir == PBL_SPI_MEM_DATA_OUT && prv_addr_ok(op) && op->dummy.nbytes == 0 &&
         prv_writeoc_for(op->cmd.opcode, &writeoc);
}

static bool prv_is_erase(const struct pbl_spi_mem_op *op) {
  nrf_qspi_erase_len_t len;

  return op->data.dir == PBL_SPI_MEM_NO_DATA && op->dummy.nbytes == 0 &&
         prv_erase_len_for(op->cmd.opcode, &len) &&
         (op->addr.nbytes == 0 ? len == NRF_QSPI_ERASE_LEN_ALL : prv_addr_ok(op));
}

static bool prv_supports_op(const struct pbl_spi_mem_device *dev, const struct pbl_spi_mem_op *op) {
  return prv_is_cinstr(op) || prv_is_read(op) || prv_is_program(op) || prv_is_erase(op);
}

static void prv_set_addrmode(struct nrf5_qspi_state *state, uint8_t nbytes) {
  nrf_qspi_addrmode_t addrmode = (nbytes == 4) ? NRF_QSPI_ADDRMODE_32BIT : NRF_QSPI_ADDRMODE_24BIT;

  if (addrmode != state->addrmode) {
    nrf_qspi_prot_conf_t conf = {
        .readoc = state->readoc,
        .writeoc = state->writeoc,
        .addrmode = addrmode,
        .dpmconfig = false,
    };
    nrf_qspi_ifconfig0_set(NRF_QSPI, &conf);
    state->addrmode = addrmode;
  }
}

static void prv_set_modes(struct nrf5_qspi_state *state, nrf_qspi_readoc_t readoc,
                          nrf_qspi_writeoc_t writeoc) {
  if (readoc != state->readoc || writeoc != state->writeoc) {
    nrf_qspi_prot_conf_t conf = {
        .readoc = readoc,
        .writeoc = writeoc,
        .addrmode = state->addrmode,
        .dpmconfig = false,
    };
    nrf_qspi_ifconfig0_set(NRF_QSPI, &conf);
    state->readoc = readoc;
    state->writeoc = writeoc;
  }
}

static void prv_exec_cinstr(const struct pbl_spi_mem_op *op) {
  uint8_t out[CINSTR_MAX_DATA] = {0};
  uint8_t in[CINSTR_MAX_DATA];
  size_t len = 0;

  for (int i = op->addr.nbytes - 1; i >= 0; i--) {
    out[len++] = (op->addr.val >> (8 * i)) & 0xFF;
  }
  len += op->dummy.nbytes;
  size_t data_off = len;
  if (op->data.dir == PBL_SPI_MEM_DATA_OUT) {
    memcpy(&out[len], op->data.buf.out, op->data.nbytes);
  }
  len += op->data.nbytes;

  nrf_qspi_cinstr_conf_t conf = {
      .opcode = op->cmd.opcode,
      .length = len + 1U,
      .io2_level = true,
      .io3_level = true,
  };

  nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
  // Accessing registers with offset > 0x600 requires the anomaly 215 workaround
  prv_workaround_215_apply();
  if (len > 0) {
    nrf_qspi_cinstrdata_set(NRF_QSPI, conf.length, out);
  }
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  nrf_qspi_cinstr_transfer_start(NRF_QSPI, &conf);
  prv_wait_ready_polling();

  if (op->data.dir == PBL_SPI_MEM_DATA_IN) {
    nrf_qspi_cinstrdata_get(NRF_QSPI, conf.length, in);
    memcpy(op->data.buf.in, &in[data_off], op->data.nbytes);
  }
}

static bool prv_use_irq(const struct pbl_spi_mem_device *dev, size_t len) {
  return len > MIN_IRQ_XFER_SIZE && !dev->state->polling;
}

static void prv_xfer(const struct pbl_spi_mem_device *dev, nrf_qspi_task_t task, size_t len) {
  bool irq = prv_use_irq(dev, len);

  if (irq) {
    nrf_qspi_int_enable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
    soc_nrf_sleep_full_block();
  } else {
    nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
  }
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  nrf_qspi_task_trigger(NRF_QSPI, task);

  if (irq) {
    pbl_sem_take(&prv_state(dev)->sem, PBL_FOREVER);
    soc_nrf_sleep_full_release();
  } else {
    prv_wait_ready_polling();
  }
}

// The READ task needs a word-aligned buffer and length: unaligned head and
// tail bytes go through a bounce word.
static void prv_exec_read(const struct pbl_spi_mem_device *dev, const struct pbl_spi_mem_op *op) {
  struct nrf5_qspi_state *state = prv_state(dev);
  nrf_qspi_readoc_t readoc = NRF_QSPI_READOC_FASTREAD;
  uint8_t __attribute__((aligned(4))) b_buf[4];
  uint8_t *buf = op->data.buf.in;
  uint32_t addr = op->addr.val;
  size_t len = op->data.nbytes;
  size_t pre = MIN((4U - ((uintptr_t)buf % 4U)) % 4U, len);
  size_t suf = (len - pre) % 4U;
  size_t mid = len - pre - suf;

  prv_readoc_for(op->cmd.opcode, &readoc);
  prv_set_modes(state, readoc, state->writeoc);
  prv_set_addrmode(state, op->addr.nbytes);

  if (pre != 0U) {
    nrf_qspi_read_buffer_set(NRF_QSPI, b_buf, 4U, addr);
    prv_xfer(dev, NRF_QSPI_TASK_READSTART, 4U);
    memcpy(buf, b_buf, pre);
    addr += pre;
    buf += pre;
  }

  if (mid != 0U) {
    nrf_qspi_read_buffer_set(NRF_QSPI, buf, mid, addr);
    prv_xfer(dev, NRF_QSPI_TASK_READSTART, mid);
    addr += mid;
    buf += mid;
  }

  if (suf != 0U) {
    nrf_qspi_read_buffer_set(NRF_QSPI, b_buf, 4U, addr);
    prv_xfer(dev, NRF_QSPI_TASK_READSTART, 4U);
    memcpy(buf, b_buf, suf);
  }
}

static bool prv_busy(void) {
  // The peripheral tracks the memory's WIP bit for us
  return (nrf_qspi_status_reg_get(NRF_QSPI) & QSPI_STATUS_SREG_Msk) & 0x1;
}

// The WRITE task needs a word-aligned RAM buffer and length. Pieces that do
// not qualify go through a bounce word padded with 0xFF, which leaves the
// neighbouring bytes untouched.
static void prv_exec_program(const struct pbl_spi_mem_device *dev,
                             const struct pbl_spi_mem_op *op) {
  struct nrf5_qspi_state *state = prv_state(dev);
  nrf_qspi_writeoc_t writeoc = NRF_QSPI_WRITEOC_PP;
  uint8_t __attribute__((aligned(4))) b_buf[4];
  const uint8_t *buf = op->data.buf.out;
  uint32_t addr = op->addr.val;
  size_t len = op->data.nbytes;

  prv_writeoc_for(op->cmd.opcode, &writeoc);
  prv_set_modes(state, state->readoc, writeoc);
  prv_set_addrmode(state, op->addr.nbytes);

  if (!nrfx_is_in_ram(buf)) {
    memcpy(s_bounce_buf, buf, len);
    buf = s_bounce_buf;
  }

  size_t pre = MIN((4U - ((uintptr_t)buf % 4U)) % 4U, len);
  size_t suf = (len - pre) % 4U;
  size_t mid = len - pre - suf;

  if (pre != 0U) {
    memset(&b_buf[pre], 0xff, sizeof(b_buf) - pre);
    memcpy(b_buf, buf, pre);
    nrf_qspi_write_buffer_set(NRF_QSPI, b_buf, 4U, addr);
    prv_xfer(dev, NRF_QSPI_TASK_WRITESTART, 4U);
    addr += pre;
    buf += pre;
  }

  if (mid != 0U) {
    while (prv_busy()) {
    }
    nrf_qspi_write_buffer_set(NRF_QSPI, buf, mid, addr);
    prv_xfer(dev, NRF_QSPI_TASK_WRITESTART, mid);
    addr += mid;
    buf += mid;
  }

  if (suf != 0U) {
    while (prv_busy()) {
    }
    memset(&b_buf[suf], 0xff, 4U - suf);
    memcpy(b_buf, buf, suf);
    nrf_qspi_write_buffer_set(NRF_QSPI, b_buf, 4U, addr);
    prv_xfer(dev, NRF_QSPI_TASK_WRITESTART, 4U);
  }
}

static void prv_exec_erase(const struct pbl_spi_mem_device *dev, const struct pbl_spi_mem_op *op) {
  struct nrf5_qspi_state *state = prv_state(dev);
  nrf_qspi_erase_len_t len = NRF_QSPI_ERASE_LEN_4KB;

  prv_erase_len_for(op->cmd.opcode, &len);
  if (op->addr.nbytes != 0) {
    prv_set_addrmode(state, op->addr.nbytes);
  }

  nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
  nrf_qspi_erase_ptr_set(NRF_QSPI, op->addr.val, len);
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_ERASESTART);
  prv_wait_ready_polling();
}

static int prv_exec_op(const struct pbl_spi_mem_device *dev, const struct pbl_spi_mem_op *op) {
  if (prv_is_read(op)) {
    prv_exec_read(dev, op);
  } else if (prv_is_program(op)) {
    prv_exec_program(dev, op);
  } else if (prv_is_erase(op)) {
    prv_exec_erase(dev, op);
  } else if (prv_is_cinstr(op)) {
    prv_exec_cinstr(op);
  } else {
    return -ENOTSUP;
  }

  return 0;
}

static int prv_adjust_op_size(const struct pbl_spi_mem_device *dev, struct pbl_spi_mem_op *op) {
  if (prv_is_program(op) && !nrfx_is_in_ram(op->data.buf.out)) {
    op->data.nbytes = MIN(op->data.nbytes, sizeof(s_bounce_buf));
  } else if (prv_is_cinstr(op)) {
    op->data.nbytes = MIN(op->data.nbytes, CINSTR_MAX_DATA - op->addr.nbytes - op->dummy.nbytes);
  }
  return 0;
}

static void prv_set_power(const struct pbl_spi_mem_device *dev, bool on) {
  if (on) {
    nrf_qspi_enable(NRF_QSPI);
    nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
    nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
    nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_ACTIVATE);
    prv_wait_ready_polling();
  } else {
    nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
    nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_DEACTIVATE);
    nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
    nrf_qspi_disable(NRF_QSPI);
  }
}

static int prv_init(const struct pbl_spi_mem_device *dev) {
  const struct nrf5_qspi *cfg = prv_cfg(dev);
  struct nrf5_qspi_state *state = prv_state(dev);
  nrf_qspi_pins_t conf_pins;
  nrf_qspi_prot_conf_t conf_prot;
  nrf_qspi_phy_conf_t conf_phy;

  // QSPI clock is 32MHz, we have dividers from 1 to 16
  PBL_ASSERTN(cfg->clk_freq_hz <= 32000000UL && cfg->clk_freq_hz >= 200000UL);
  if (cfg->clk_freq_hz > 8000000UL) {
    PBL_LOG_WRN(
        "QSPI initialized at %lu Hz, which may cause data corruption if HF clock source switches "
        "(anomaly 244)",
        cfg->clk_freq_hz);
  }

  conf_pins.sck_pin = cfg->sck_pin;
  conf_pins.csn_pin = cfg->csn_pin;
  conf_pins.io0_pin = cfg->io_pins[0];
  conf_pins.io1_pin = cfg->io_pins[1];
  conf_pins.io2_pin = cfg->io_pins[2];
  conf_pins.io3_pin = cfg->io_pins[3];
  nrf_qspi_pins_set(NRF_QSPI, &conf_pins);

  state->readoc = NRF_QSPI_READOC_FASTREAD;
  state->writeoc = NRF_QSPI_WRITEOC_PP;
  state->addrmode = NRF_QSPI_ADDRMODE_24BIT;
  conf_prot.readoc = state->readoc;
  conf_prot.writeoc = state->writeoc;
  conf_prot.addrmode = state->addrmode;
  conf_prot.dpmconfig = false;
  nrf_qspi_ifconfig0_set(NRF_QSPI, &conf_prot);

  conf_phy.sck_delay = 5U;
  conf_phy.dpmen = false;
  conf_phy.spi_mode = NRF_QSPI_MODE_0;
  conf_phy.sck_freq = (32000000UL / cfg->clk_freq_hz) - 1U;
  nrf_qspi_ifconfig1_set(NRF_QSPI, &conf_phy);

  s_state = state;
  pbl_sem_init(&state->sem, 0, 1);
  NVIC_SetPriority(QSPI_IRQn, 5);
  NVIC_EnableIRQ(QSPI_IRQn);

  prv_set_power(dev, true);

  return 0;
}

static const struct pbl_spi_mem_ops s_ops = {
    .init = prv_init,
    .supports_op = prv_supports_op,
    .adjust_op_size = prv_adjust_op_size,
    .exec_op = prv_exec_op,
    .set_power = prv_set_power,
};

static struct nrf5_qspi_state s_qspi_state;
static const struct nrf5_qspi s_qspi = {
    .dev =
        {
            .state = &s_qspi_state.spi_mem,
            .ops = &s_ops,
        },
    .clk_freq_hz = CONFIG_NRF5_QSPI_CLK_FREQ_HZ,
    .csn_pin = CONFIG_NRF5_QSPI_CSN_PIN,
    .sck_pin = CONFIG_NRF5_QSPI_SCK_PIN,
    .io_pins =
        {
            CONFIG_NRF5_QSPI_IO0_PIN,
            CONFIG_NRF5_QSPI_IO1_PIN,
            CONFIG_NRF5_QSPI_IO2_PIN,
            CONFIG_NRF5_QSPI_IO3_PIN,
        },
};
const struct pbl_spi_mem_device *const SPI_MEM_NOR = &s_qspi.dev;

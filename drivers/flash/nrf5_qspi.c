/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include <pbl/drivers/flash/nrf5_qspi.h>
#include <pbl/drivers/gpio/nrf5.h>
#include <pbl/kernel/compiler.h>
#include <pbl/kernel/delay.h>
#include <pbl/kernel/irq.h>
#include <pbl/logging/logging.h>
#include <pbl/soc/nrf/sleep.h>
#include <pbl/util/math.h>
#include <pbl/util/misc.h>
#include <pbl/util/size.h>

#include <hal/nrf_qspi.h>

PBL_LOG_MODULE_DECLARE(driver_flash, CONFIG_DRIVER_FLASH_LOG_LEVEL);

// Anomaly 244 (data corruption when the HF clock source switches) has not been seen at or below
// this frequency, so it is not worked around.
#define ANOMALY_244_MAX_HZ 8000000U

#define QSPI_BASE_HZ   32000000U
#define QSPI_MIN_HZ    (QSPI_BASE_HZ / 16U)
#define QSPI_SCK_DELAY 5U

// Interrupt-driven transfers only pay off above this size
#define MIN_IRQ_XFER_SIZE 256U

#define CINSTR_MAX_DATA      8U
#define ID_LEN               3U
#define ADDR_4BYTE_THRESHOLD (16U * 1024U * 1024U)
#define WORD_SIZE            sizeof(uint32_t)

// EasyDMA only reaches data RAM
#define IS_IN_RAM(p) (((uintptr_t)(p) & 0xe0000000U) == 0x20000000U)

// Bits 15-12 of a security register address are its one-based index
#define SEC_REG_IDX(addr) (((addr) >> 12U) - 1U)

static uint8_t PBL_ALIGNED(4) s_bounce_buf[32];

static const nrf_qspi_readoc_t s_readoc[] = {
  [PBL_FLASH_NRF5_QSPI_FASTREAD] = NRF_QSPI_READOC_FASTREAD,
  [PBL_FLASH_NRF5_QSPI_READ2O] = NRF_QSPI_READOC_READ2O,
  [PBL_FLASH_NRF5_QSPI_READ2IO] = NRF_QSPI_READOC_READ2IO,
  [PBL_FLASH_NRF5_QSPI_READ4O] = NRF_QSPI_READOC_READ4O,
  [PBL_FLASH_NRF5_QSPI_READ4IO] = NRF_QSPI_READOC_READ4IO,
};

static const nrf_qspi_writeoc_t s_writeoc[] = {
  [PBL_FLASH_NRF5_QSPI_PP] = NRF_QSPI_WRITEOC_PP,
  [PBL_FLASH_NRF5_QSPI_PP2O] = NRF_QSPI_WRITEOC_PP2O,
  [PBL_FLASH_NRF5_QSPI_PP4O] = NRF_QSPI_WRITEOC_PP4O,
  [PBL_FLASH_NRF5_QSPI_PP4IO] = NRF_QSPI_WRITEOC_PP4IO,
};

static const struct pbl_flash_nrf5_qspi *prv_qspi(const struct pbl_flash *flash) {
  return container_of(flash, const struct pbl_flash_nrf5_qspi, flash);
}

static bool prv_is_4byte(const struct pbl_flash_nrf5_qspi *qspi) {
  return qspi->part->geometry.size > ADDR_4BYTE_THRESHOLD;
}

static bool prv_uses_quad(const struct pbl_flash_nrf5_qspi *qspi) {
  return qspi->read == PBL_FLASH_NRF5_QSPI_READ2IO || qspi->read == PBL_FLASH_NRF5_QSPI_READ4O ||
         qspi->read == PBL_FLASH_NRF5_QSPI_READ4IO || qspi->write == PBL_FLASH_NRF5_QSPI_PP4O ||
         qspi->write == PBL_FLASH_NRF5_QSPI_PP4IO;
}

void pbl_flash_nrf5_qspi_irq_handler(const struct pbl_flash_nrf5_qspi *qspi) {
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  pbl_sem_give(&qspi->data->done);
}

static void prv_wait_ready(void) {
  while (!nrf_qspi_event_check(NRF_QSPI, NRF_QSPI_EVENT_READY)) {
  }
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
}

static void prv_activate(void) {
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_ACTIVATE);
  prv_wait_ready();
}

// nRF52 anomaly 215: after XIP, accessing a QSPI register above offset 0x600 halts the CPU unless
// the peripheral is activated again first. Pins are disconnected meanwhile so that nothing waits
// on the memory.
static void prv_anomaly_215_workaround(void) {
  nrf_qspi_pins_t pins;
  nrf_qspi_pins_t disconnected = {
    .sck_pin = NRF_QSPI_PIN_NOT_CONNECTED,
    .csn_pin = NRF_QSPI_PIN_NOT_CONNECTED,
    .io0_pin = NRF_QSPI_PIN_NOT_CONNECTED,
    .io1_pin = NRF_QSPI_PIN_NOT_CONNECTED,
    .io2_pin = NRF_QSPI_PIN_NOT_CONNECTED,
    .io3_pin = NRF_QSPI_PIN_NOT_CONNECTED,
  };

  nrf_qspi_pins_get(NRF_QSPI, &pins);
  nrf_qspi_pins_set(NRF_QSPI, &disconnected);
  prv_activate();
  nrf_qspi_pins_set(NRF_QSPI, &pins);
}

static void prv_cinstr(uint8_t opcode, const void *tx, void *rx, size_t len) {
  nrf_qspi_cinstr_conf_t conf = {
    .opcode = opcode,
    .length = len + 1U,
    .io2_level = true,
    .io3_level = true,
  };

  nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
  prv_anomaly_215_workaround();

  if (tx != nullptr) {
    nrf_qspi_cinstrdata_set(NRF_QSPI, conf.length, tx);
  }

  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  nrf_qspi_cinstr_transfer_start(NRF_QSPI, &conf);
  prv_wait_ready();

  if (rx != nullptr) {
    nrf_qspi_cinstrdata_get(NRF_QSPI, conf.length, rx);
  }
}

static uint8_t prv_read_sr(uint8_t opcode) {
  uint8_t sr;

  prv_cinstr(opcode, nullptr, &sr, sizeof(sr));

  return sr;
}

static bool prv_is_busy(void) {
  return (prv_read_sr(PBL_FLASH_NOR_CMD_RDSR) & PBL_FLASH_NOR_SR_WIP) != 0U;
}

static void prv_wait_idle(void) {
  while (prv_is_busy()) {
  }
}

static void prv_write_enable(void) {
  prv_cinstr(PBL_FLASH_NOR_CMD_WREN, nullptr, nullptr, 0U);
}

static bool prv_use_irq(const struct pbl_flash_nrf5_qspi *qspi, size_t len) {
  return len > MIN_IRQ_XFER_SIZE && !pbl_flash_in_coredump(&qspi->flash);
}

static void prv_xfer_start(bool irq) {
  if (irq) {
    nrf_qspi_int_enable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
    soc_nrf_sleep_full_block();
  } else {
    nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
  }

  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
}

static void prv_xfer_wait(const struct pbl_flash_nrf5_qspi *qspi, bool irq) {
  if (irq) {
    pbl_sem_take(&qspi->data->done, PBL_FOREVER);
    soc_nrf_sleep_full_release();
  } else {
    prv_wait_ready();
  }
}

static void prv_read_aligned(const struct pbl_flash_nrf5_qspi *qspi, uint32_t offset, void *buf,
                             size_t len) {
  bool irq = prv_use_irq(qspi, len);

  prv_xfer_start(irq);
  nrf_qspi_read_buffer_set(NRF_QSPI, buf, len, offset);
  nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_READSTART);
  prv_xfer_wait(qspi, irq);
}

static void prv_write_aligned(const struct pbl_flash_nrf5_qspi *qspi, uint32_t offset,
                              const void *buf, size_t len) {
  bool irq = prv_use_irq(qspi, len);

  prv_xfer_start(irq);
  nrf_qspi_write_buffer_set(NRF_QSPI, buf, len, offset);
  nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_WRITESTART);
  prv_xfer_wait(qspi, irq);
}

static void prv_split(const void *buf, size_t len, size_t *pre, size_t *mid, size_t *suf) {
  *pre = MIN((WORD_SIZE - ((uintptr_t)buf % WORD_SIZE)) % WORD_SIZE, len);
  *suf = (len - *pre) % WORD_SIZE;
  *mid = len - *pre - *suf;
}

static int prv_init(const struct pbl_flash *flash) {
  const struct pbl_flash_nrf5_qspi *qspi = prv_qspi(flash);
  const struct pbl_flash_nor_part *part = qspi->part;
  nrf_qspi_pins_t pins;
  nrf_qspi_prot_conf_t prot = {};
  nrf_qspi_phy_conf_t phy = {};
  uint32_t id = 0U;
  uint8_t sr[2];

  if (qspi->data->initialized) {
    return 0;
  }

  if (qspi->freq_hz > QSPI_BASE_HZ || qspi->freq_hz < QSPI_MIN_HZ ||
      qspi->read >= ARRAY_LENGTH(s_readoc) || qspi->write >= ARRAY_LENGTH(s_writeoc)) {
    return -EINVAL;
  }

  if (qspi->freq_hz > ANOMALY_244_MAX_HZ) {
    PBL_LOG_WRN("QSPI at %" PRIu32 " Hz is exposed to anomaly 244", qspi->freq_hz);
  }

  pins.sck_pin = pbl_gpio_nrf5_pin(&qspi->sck);
  pins.csn_pin = pbl_gpio_nrf5_pin(&qspi->csn);
  pins.io0_pin = pbl_gpio_nrf5_pin(&qspi->io[0]);
  pins.io1_pin = pbl_gpio_nrf5_pin(&qspi->io[1]);
  pins.io2_pin = pbl_gpio_nrf5_pin(&qspi->io[2]);
  pins.io3_pin = pbl_gpio_nrf5_pin(&qspi->io[3]);
  nrf_qspi_pins_set(NRF_QSPI, &pins);

  prot.readoc = s_readoc[qspi->read];
  prot.writeoc = s_writeoc[qspi->write];
  prot.addrmode = prv_is_4byte(qspi) ? NRF_QSPI_ADDRMODE_32BIT : NRF_QSPI_ADDRMODE_24BIT;
  nrf_qspi_ifconfig0_set(NRF_QSPI, &prot);

  phy.sck_delay = QSPI_SCK_DELAY;
  phy.spi_mode = NRF_QSPI_MODE_0;
  phy.sck_freq = (QSPI_BASE_HZ / qspi->freq_hz) - 1U;
  nrf_qspi_ifconfig1_set(NRF_QSPI, &phy);

  nrf_qspi_enable(NRF_QSPI);
  prv_activate();

  // Abort any program or erase left over from before the reset
  prv_cinstr(PBL_FLASH_NOR_CMD_RSTEN, nullptr, nullptr, 0U);
  prv_cinstr(PBL_FLASH_NOR_CMD_RST, nullptr, nullptr, 0U);
  if (pbl_flash_in_coredump(flash)) {
    pbl_busy_wait_us(part->reset_ms * 1000U);
  } else {
    pbl_sleep_ms(part->reset_ms);
  }

  prv_cinstr(PBL_FLASH_NOR_CMD_RDID, nullptr, &id, ID_LEN);
  if (id != part->id) {
    PBL_LOG_WRN("Flash is not %s (id 0x%06" PRIx32 ")", part->name, id);
  }

  if (prv_is_4byte(qspi)) {
    prv_cinstr(PBL_FLASH_NOR_CMD_EN4B, nullptr, nullptr, 0U);
  }

  if (prv_uses_quad(qspi)) {
    switch (part->qe) {
      case PBL_FLASH_NOR_QE_NONE:
        break;
      case PBL_FLASH_NOR_QE_SR1_BIT6:
        sr[0] = prv_read_sr(PBL_FLASH_NOR_CMD_RDSR) | PBL_FLASH_NOR_SR_QE;
        prv_cinstr(PBL_FLASH_NOR_CMD_WRSR, sr, nullptr, 1U);
        break;
      case PBL_FLASH_NOR_QE_SR2_BIT1_WR_SR1:
        sr[0] = prv_read_sr(PBL_FLASH_NOR_CMD_RDSR);
        sr[1] = prv_read_sr(PBL_FLASH_NOR_CMD_RDSR2) | PBL_FLASH_NOR_SR2_QE;
        prv_cinstr(PBL_FLASH_NOR_CMD_WRSR, sr, nullptr, sizeof(sr));
        break;
      case PBL_FLASH_NOR_QE_SR2_BIT1:
        sr[1] = prv_read_sr(PBL_FLASH_NOR_CMD_RDSR2) | PBL_FLASH_NOR_SR2_QE;
        prv_cinstr(PBL_FLASH_NOR_CMD_WRSR2, &sr[1], nullptr, 1U);
        break;
      default:
        return -EINVAL;
    }
  }

  pbl_irq_enable(PBL_IRQN(QSPI));

  qspi->data->initialized = true;

  return 0;
}

static int prv_read(const struct pbl_flash *flash, uint32_t addr, void *buf, size_t len) {
  const struct pbl_flash_nrf5_qspi *qspi = prv_qspi(flash);
  uint8_t PBL_ALIGNED(4) word[WORD_SIZE];
  uint32_t offset = addr - flash->base;
  uint8_t *dst = buf;
  size_t pre;
  size_t mid;
  size_t suf;

  // EasyDMA needs word-aligned buffers and lengths
  prv_split(buf, len, &pre, &mid, &suf);

  if (pre != 0U) {
    prv_read_aligned(qspi, offset, word, sizeof(word));
    memcpy(dst, word, pre);
    offset += pre;
    dst += pre;
  }

  if (mid != 0U) {
    prv_read_aligned(qspi, offset, dst, mid);
    offset += mid;
    dst += mid;
  }

  if (suf != 0U) {
    prv_read_aligned(qspi, offset, word, sizeof(word));
    memcpy(dst, word, suf);
  }

  return 0;
}

static size_t prv_write_page(const struct pbl_flash_nrf5_qspi *qspi, uint32_t offset,
                             const uint8_t *src, size_t len) {
  const uint32_t page_size = qspi->part->geometry.page_size;
  uint8_t PBL_ALIGNED(4) word[WORD_SIZE];
  size_t pre;
  size_t mid;
  size_t suf;

  len = MIN(len, page_size - (offset % page_size));

  if (!IS_IN_RAM(src)) {
    len = MIN(len, sizeof(s_bounce_buf));
    memcpy(s_bounce_buf, src, len);
    src = s_bounce_buf;
  }

  prv_split(src, len, &pre, &mid, &suf);

  prv_write_enable();

  // The padding bytes program nothing
  if (pre != 0U) {
    memset(word, 0xFF, sizeof(word));
    memcpy(word, src, pre);
    prv_write_aligned(qspi, offset, word, sizeof(word));
    offset += pre;
    src += pre;
  }

  if (mid != 0U) {
    prv_wait_idle();
    prv_write_aligned(qspi, offset, src, mid);
    offset += mid;
    src += mid;
  }

  if (suf != 0U) {
    prv_wait_idle();
    memset(word, 0xFF, sizeof(word));
    memcpy(word, src, suf);
    prv_write_aligned(qspi, offset, word, sizeof(word));
  }

  return len;
}

static int prv_write(const struct pbl_flash *flash, uint32_t addr, const void *buf, size_t len) {
  const struct pbl_flash_nrf5_qspi *qspi = prv_qspi(flash);
  uint32_t offset = addr - flash->base;
  const uint8_t *src = buf;

  while (len > 0U) {
    size_t written = prv_write_page(qspi, offset, src, len);

    while (prv_is_busy()) {
      if (!pbl_flash_in_coredump(flash)) {
        pbl_sleep_ms(0U);
      }
    }

    offset += written;
    src += written;
    len -= written;
  }

  return 0;
}

static int prv_erase(const struct pbl_flash *flash, uint32_t addr, uint32_t size) {
  const struct pbl_flash_geometry *geo = flash->geometry;
  nrf_qspi_erase_len_t len;

  if (size == geo->subsector_size) {
    len = NRF_QSPI_ERASE_LEN_4KB;
  } else if (size == geo->sector_size) {
    len = NRF_QSPI_ERASE_LEN_64KB;
  } else {
    return -EINVAL;
  }

  prv_write_enable();

  nrf_qspi_erase_ptr_set(NRF_QSPI, addr - flash->base, len);
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_ERASESTART);
  prv_wait_ready();

  soc_nrf_sleep_full_block();

  return 0;
}

static int prv_erase_status(const struct pbl_flash *flash) {
  if (prv_is_busy()) {
    return -EBUSY;
  }

  if ((prv_read_sr(PBL_FLASH_NOR_CMD_RDSR2) & PBL_FLASH_NOR_SR2_SUS) != 0U) {
    return -EAGAIN;
  }

  soc_nrf_sleep_full_release();

  return 0;
}

static int prv_erase_suspend(const struct pbl_flash *flash) {
  const struct pbl_flash_nrf5_qspi *qspi = prv_qspi(flash);

  if (!prv_is_busy()) {
    return 1;
  }

  prv_cinstr(PBL_FLASH_NOR_CMD_SUSPEND, nullptr, nullptr, 0U);
  pbl_busy_wait_us(qspi->part->suspend_us);

  return 0;
}

static int prv_erase_resume(const struct pbl_flash *flash) {
  prv_cinstr(PBL_FLASH_NOR_CMD_RESUME, nullptr, nullptr, 0U);

  return 0;
}

static void prv_power_down(const struct pbl_flash *flash) {
  const struct pbl_flash_nrf5_qspi *qspi = prv_qspi(flash);

  prv_cinstr(PBL_FLASH_NOR_CMD_DPD, nullptr, nullptr, 0U);
  pbl_busy_wait_us(qspi->part->dpd_enter_us);

  nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
  nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_DEACTIVATE);
  nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
  nrf_qspi_disable(NRF_QSPI);
}

static void prv_power_up(const struct pbl_flash *flash) {
  const struct pbl_flash_nrf5_qspi *qspi = prv_qspi(flash);

  nrf_qspi_enable(NRF_QSPI);
  nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
  prv_activate();

  prv_cinstr(PBL_FLASH_NOR_CMD_RDPD, nullptr, nullptr, 0U);
  pbl_busy_wait_us(qspi->part->dpd_exit_us);
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

static size_t prv_put_addr(const struct pbl_flash_nrf5_qspi *qspi, uint32_t addr, uint8_t *out) {
  size_t n = prv_is_4byte(qspi) ? 4U : 3U;

  for (size_t i = 0U; i < n; i++) {
    out[i] = (uint8_t)(addr >> (8U * (n - 1U - i)));
  }

  return n;
}

static int prv_sec_reg_read(const struct pbl_flash *flash, uint32_t addr, uint8_t *val) {
  const struct pbl_flash_nrf5_qspi *qspi = prv_qspi(flash);
  uint8_t out[CINSTR_MAX_DATA] = {};
  uint8_t in[CINSTR_MAX_DATA];
  size_t n;
  int ret;

  ret = prv_sec_reg_check(flash, addr);
  if (ret != 0) {
    return ret;
  }

  // Address, a dummy byte, then the data
  n = prv_put_addr(qspi, addr, out);
  prv_cinstr(PBL_FLASH_NOR_CMD_RDSCUR, out, in, n + 2U);
  *val = in[n + 1U];

  return 0;
}

static int prv_sec_reg_write(const struct pbl_flash *flash, uint32_t addr, uint8_t val) {
  const struct pbl_flash_nrf5_qspi *qspi = prv_qspi(flash);
  uint8_t out[CINSTR_MAX_DATA];
  size_t n;
  int ret;

  ret = prv_sec_reg_check(flash, addr);
  if (ret != 0) {
    return ret;
  }

  n = prv_put_addr(qspi, addr, out);
  out[n] = val;

  prv_write_enable();
  prv_cinstr(PBL_FLASH_NOR_CMD_PRSCUR, out, nullptr, n + 1U);
  prv_wait_idle();

  return 0;
}

static int prv_sec_reg_erase(const struct pbl_flash *flash, uint32_t addr) {
  const struct pbl_flash_nrf5_qspi *qspi = prv_qspi(flash);
  uint8_t out[CINSTR_MAX_DATA];
  size_t n;
  int ret;

  ret = prv_sec_reg_check(flash, addr);
  if (ret != 0) {
    return ret;
  }

  n = prv_put_addr(qspi, addr, out);

  prv_write_enable();
  prv_cinstr(PBL_FLASH_NOR_CMD_ERSCUR, out, nullptr, n);
  prv_wait_idle();

  return 0;
}

static int prv_sec_reg_is_locked(const struct pbl_flash *flash, uint32_t addr, bool *locked) {
  int ret;

  ret = prv_sec_reg_check(flash, addr);
  if (ret != 0) {
    return ret;
  }

  *locked = (prv_read_sr(PBL_FLASH_NOR_CMD_RDSR2) &
             PBL_BIT(PBL_FLASH_NOR_SR2_LB_SHIFT + SEC_REG_IDX(addr))) != 0U;

  return 0;
}

#ifdef CONFIG_RECOVERY_FW
static int prv_sec_reg_lock(const struct pbl_flash *flash, uint32_t addr) {
  uint8_t sr[2];
  int ret;

  ret = prv_sec_reg_check(flash, addr);
  if (ret != 0) {
    return ret;
  }

  sr[0] = prv_read_sr(PBL_FLASH_NOR_CMD_RDSR);
  sr[1] = prv_read_sr(PBL_FLASH_NOR_CMD_RDSR2) |
          PBL_BIT(PBL_FLASH_NOR_SR2_LB_SHIFT + SEC_REG_IDX(addr));
  prv_cinstr(PBL_FLASH_NOR_CMD_WRSR, sr, nullptr, sizeof(sr));

  return 0;
}
#endif

const struct pbl_flash_ops pbl_flash_nrf5_qspi_ops = {
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

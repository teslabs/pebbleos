/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <pbl/drivers/i2c/sf32lb.h>
#include <pbl/kernel/assert.h>
#include <pbl/kernel/irq.h>
#include <pbl/mcu/cache.h>
#include <pbl/soc/sf32lb/sleep.h>
#include <pbl/util/misc.h>

static const struct pbl_i2c_sf32lb *prv_i2c(const struct pbl_i2c_bus *bus) {
  return container_of(bus, const struct pbl_i2c_sf32lb, bus);
}

// Block deep sleep while a transfer is in flight. The flag keeps the release
// exactly-once across the IRQ, kickoff-failure and abort paths.
static void prv_deepsleep_block(const struct pbl_i2c_sf32lb *i2c) {
  i2c->state->deepsleep_blocked = true;
  soc_sf32lb_sleep_block(SOC_SF32LB_DEEPSLEEP);
}

static void prv_deepsleep_allow(const struct pbl_i2c_sf32lb *i2c) {
  struct pbl_i2c_sf32lb_state *state = i2c->state;

  pbl_irq_lock();
  bool blocked = state->deepsleep_blocked;
  state->deepsleep_blocked = false;
  pbl_irq_unlock();

  if (blocked) {
    soc_sf32lb_sleep_release(SOC_SF32LB_DEEPSLEEP);
  }
}

//! Makes a completed DMA read visible to the CPU; stops an aborted one.
static void prv_dma_finish(const struct pbl_i2c_sf32lb *i2c, bool completed) {
  struct pbl_i2c_sf32lb_state *state = i2c->state;

  if (state->dma_data == nullptr) {
    return;
  }

  if (completed) {
    dcache_invalidate(state->dma_data, DCACHE_ROUND_UP(state->dma_size));
  } else {
    HAL_DMA_Abort(&state->hdma_rx);
  }
  state->dma_data = nullptr;
}

void pbl_i2c_sf32lb_irq_handler(const struct pbl_i2c_sf32lb *i2c) {
  I2C_HandleTypeDef *hdl = &i2c->state->hdl;
  HAL_I2C_StateTypeDef state;
  int status;

  if (hdl->XferISR == nullptr) {
    return;
  }

  (void)hdl->XferISR(hdl, 0, 0);

  state = HAL_I2C_GetState(hdl);
  if ((state == HAL_I2C_STATE_BUSY_TX) || (state == HAL_I2C_STATE_BUSY_RX)) {
    return;
  } else if (state == HAL_I2C_STATE_READY) {
    status = 0;
  } else {
    status = -EIO;
  }

  prv_deepsleep_allow(i2c);
  prv_dma_finish(i2c, status == 0);

  pbl_i2c_bus_complete(&i2c->bus, status);
}

void pbl_i2c_sf32lb_dma_irq_handler(const struct pbl_i2c_sf32lb *i2c) {
  HAL_DMA_IRQHandler(&i2c->state->hdma_rx);
}

static void prv_abort(const struct pbl_i2c_bus *bus) {
  const struct pbl_i2c_sf32lb *i2c = prv_i2c(bus);

  HAL_I2C_Reset(&i2c->state->hdl);

  prv_deepsleep_allow(i2c);
  prv_dma_finish(i2c, false);
}

// Memory address of a register access: a 1 or 2-byte write followed by another message
static bool prv_mem_addr(const struct pbl_i2c_msg *msgs, uint8_t num_msgs, uint16_t *mem_addr,
                         uint16_t *mem_size) {
  if (num_msgs != 2U || (msgs[0].flags & PBL_I2C_MSG_READ) != 0U) {
    return false;
  }

  if (msgs[0].len == 1U) {
    *mem_addr = msgs[0].buf[0];
    *mem_size = I2C_MEMADD_SIZE_8BIT;
  } else if (msgs[0].len == 2U) {
    *mem_addr = (uint16_t)((msgs[0].buf[0] << 8) | msgs[0].buf[1]);
    *mem_size = I2C_MEMADD_SIZE_16BIT;
  } else {
    return false;
  }

  return true;
}

static int prv_start(const struct pbl_i2c_bus *bus, uint16_t addr, struct pbl_i2c_msg *msgs,
                     uint8_t num_msgs) {
  const struct pbl_i2c_sf32lb *i2c = prv_i2c(bus);
  I2C_HandleTypeDef *hdl = &i2c->state->hdl;
  struct pbl_i2c_msg *data = &msgs[num_msgs - 1U];
  bool read = (data->flags & PBL_I2C_MSG_READ) != 0U;
  uint16_t mem_addr;
  uint16_t mem_size;
  HAL_StatusTypeDef ret;

  if (num_msgs == 1U) {
    prv_deepsleep_block(i2c);
    if (read) {
      ret = HAL_I2C_Master_Receive_IT(hdl, addr, data->buf, data->len);
    } else {
      ret = HAL_I2C_Master_Transmit_IT(hdl, addr, data->buf, data->len);
    }
  } else if (prv_mem_addr(msgs, num_msgs, &mem_addr, &mem_size) &&
             (read || (data->flags & PBL_I2C_MSG_NOSTART) != 0U)) {
    prv_deepsleep_block(i2c);
    if (read && (data->flags & PBL_I2C_MSG_DMA_SAFE) != 0U &&
        i2c->state->hdma_rx.Instance != nullptr && data->len >= I2C_SF32LB_DMA_MIN_BYTES) {
      // Dropping the buffer's lines now keeps a dirty one from being evicted over the DMA data.
      i2c->state->dma_data = data->buf;
      i2c->state->dma_size = data->len;
      dcache_invalidate(data->buf, DCACHE_ROUND_UP(data->len));
      HAL_DMA_Init(&i2c->state->hdma_rx);
      ret = HAL_I2C_Mem_Read_DMA(hdl, addr, mem_addr, mem_size, data->buf, data->len);
    } else if (read) {
      ret = HAL_I2C_Mem_Read_IT(hdl, addr, mem_addr, mem_size, data->buf, data->len);
    } else {
      ret = HAL_I2C_Mem_Write_IT(hdl, addr, mem_addr, mem_size, data->buf, data->len);
    }
  } else {
    return -ENOTSUP;
  }

  if (ret != HAL_OK) {
    HAL_I2C_Reset(hdl);
    prv_deepsleep_allow(i2c);
    prv_dma_finish(i2c, false);
    return -EIO;
  }

  return 0;
}

static void prv_enable(const struct pbl_i2c_bus *bus) {
  const struct pbl_i2c_sf32lb *i2c = prv_i2c(bus);

  HAL_RCC_EnableModule(i2c->module);
  __HAL_I2C_ENABLE(&i2c->state->hdl);
}

static void prv_disable(const struct pbl_i2c_bus *bus) {
  const struct pbl_i2c_sf32lb *i2c = prv_i2c(bus);

  __HAL_I2C_DISABLE(&i2c->state->hdl);
  HAL_RCC_DisableModule(i2c->module);
}

static int prv_init(const struct pbl_i2c_bus *bus) {
  const struct pbl_i2c_sf32lb *i2c = prv_i2c(bus);
  I2C_HandleTypeDef *hdl = &i2c->state->hdl;
  int ret;

  ret = pbl_pinctrl_select_state(&i2c->pinctrl, PBL_PINCTRL_STATE_DEFAULT);
  if (ret != 0) {
    return ret;
  }

  HAL_RCC_EnableModule(i2c->module);
  if (HAL_I2C_Init(hdl) != HAL_OK) {
    return -EIO;
  }

  if (i2c->state->hdma_rx.Instance != nullptr) {
    PBL_ASSERTN(dcache_line_size() <= DCACHE_LINE_SIZE_MAX);
    DMA_InitTypeDef *init = &i2c->state->hdma_rx.Init;
    init->Direction = DMA_PERIPH_TO_MEMORY;
    init->PeriphInc = DMA_PINC_DISABLE;
    init->MemInc = DMA_MINC_ENABLE;
    init->PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    init->MemDataAlignment = DMA_MDATAALIGN_BYTE;
    init->Mode = DMA_NORMAL;
    init->Priority = DMA_PRIORITY_HIGH;
    __HAL_LINKDMA(hdl, hdmarx, i2c->state->hdma_rx);
    pbl_irq_enable(i2c->dma_irqn);
  }

  pbl_irq_enable(i2c->irqn);

  return 0;
}

const struct pbl_i2c_bus_ops pbl_i2c_sf32lb_ops = {
  .init = prv_init,
  .enable = prv_enable,
  .disable = prv_disable,
  .start = prv_start,
  .abort = prv_abort,
};

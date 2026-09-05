/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/i2c/sf32lb.h>

#include "pbl/kernel/irq.h"
#include "pbl/soc/sf32lb/sleep.h"
#include "system/passert.h"

static const struct pbl_i2c_sf32lb *prv_dev(const struct pbl_i2c_bus *bus) {
  return PBL_CONTAINER_OF(bus, const struct pbl_i2c_sf32lb, bus);
}

static I2C_HandleTypeDef *prv_hdl(const struct pbl_i2c_bus *bus) {
  return &prv_dev(bus)->state->hdl;
}

// Block deep sleep while a transfer is in flight. The flag keeps the release
// exactly-once across the IRQ, kickoff-failure and abort paths.
static void prv_deepsleep_block(const struct pbl_i2c_bus *bus) {
  prv_dev(bus)->state->deepsleep_blocked = true;
  soc_sf32lb_sleep_block(SOC_SF32LB_DEEPSLEEP);
}

static void prv_deepsleep_allow(const struct pbl_i2c_bus *bus) {
  struct pbl_i2c_sf32lb_state *state = prv_dev(bus)->state;
  pbl_irq_lock();
  bool blocked = state->deepsleep_blocked;
  state->deepsleep_blocked = false;
  pbl_irq_unlock();
  if (blocked) {
    soc_sf32lb_sleep_release(SOC_SF32LB_DEEPSLEEP);
  }
}

void pbl_i2c_sf32lb_irq_handler(const struct pbl_i2c_bus *bus) {
  I2C_HandleTypeDef *hdl = prv_hdl(bus);
  HAL_I2C_StateTypeDef state;
  enum pbl_i2c_event event;

  if (hdl->XferISR == NULL) {
    return;
  }

  (void)hdl->XferISR(hdl, 0, 0);

  state = HAL_I2C_GetState(hdl);
  if ((state == HAL_I2C_STATE_BUSY_TX) || (state == HAL_I2C_STATE_BUSY_RX)) {
    return;
  } else if (state == HAL_I2C_STATE_READY) {
    event = PBL_I2C_EVENT_COMPLETE;
  } else {
    event = PBL_I2C_EVENT_ERROR;
  }

  prv_deepsleep_allow(bus);
  pbl_i2c_bus_event(bus, event);
}

static int prv_init(const struct pbl_i2c_bus *bus) {
  const struct pbl_i2c_sf32lb *i2c = prv_dev(bus);
  HAL_StatusTypeDef ret;

  HAL_PIN_Set(i2c->scl.pad, i2c->scl.func, i2c->scl.flags, 1);
  HAL_PIN_Set(i2c->sda.pad, i2c->sda.func, i2c->sda.flags, 1);

  HAL_RCC_EnableModule(i2c->module);
  ret = HAL_I2C_Init(&i2c->state->hdl);
  PBL_ASSERTN(ret == HAL_OK);

  HAL_NVIC_SetPriority(i2c->irqn, i2c->irq_priority, 0);
  NVIC_EnableIRQ(i2c->irqn);
  return 0;
}

static void prv_enable(const struct pbl_i2c_bus *bus) {
  HAL_RCC_EnableModule(prv_dev(bus)->module);
  __HAL_I2C_ENABLE(prv_hdl(bus));
}

static void prv_disable(const struct pbl_i2c_bus *bus) {
  __HAL_I2C_DISABLE(prv_hdl(bus));
  HAL_RCC_DisableModule(prv_dev(bus)->module);
}

static bool prv_is_busy(const struct pbl_i2c_bus *bus) {
  return HAL_I2C_GetState(prv_hdl(bus)) != HAL_I2C_STATE_READY;
}

static void prv_begin_transfer(const struct pbl_i2c_bus *bus) {
  prv_deepsleep_block(bus);
}

static void prv_start_transfer(const struct pbl_i2c_bus *bus) {
  I2C_HandleTypeDef *hdl = prv_hdl(bus);
  struct pbl_i2c_transfer *transfer = &bus->state->transfer;
  HAL_StatusTypeDef ret;

  if (transfer->with_reg) {
    if (transfer->dir == PBL_I2C_READ) {
      ret = HAL_I2C_Mem_Read_IT(hdl, transfer->addr, transfer->reg, I2C_MEMADD_SIZE_8BIT,
                                transfer->data, transfer->size);
    } else {
      ret = HAL_I2C_Mem_Write_IT(hdl, transfer->addr, transfer->reg, I2C_MEMADD_SIZE_8BIT,
                                 transfer->data, transfer->size);
    }
  } else {
    if (transfer->dir == PBL_I2C_READ) {
      ret = HAL_I2C_Master_Receive_IT(hdl, transfer->addr, transfer->data, transfer->size);
    } else {
      ret = HAL_I2C_Master_Transmit_IT(hdl, transfer->addr, transfer->data, transfer->size);
    }
  }

  if (ret != HAL_OK) {
    HAL_I2C_Reset(hdl);
    prv_deepsleep_allow(bus);
    pbl_i2c_bus_event(bus, PBL_I2C_EVENT_ERROR);
  }
}

static void prv_abort_transfer(const struct pbl_i2c_bus *bus) {
  HAL_I2C_Reset(prv_hdl(bus));
  prv_deepsleep_allow(bus);
}

const struct pbl_i2c_bus_ops pbl_i2c_sf32lb_ops = {
  .init = prv_init,
  .enable = prv_enable,
  .disable = prv_disable,
  .is_busy = prv_is_busy,
  .begin_transfer = prv_begin_transfer,
  .start_transfer = prv_start_transfer,
  .abort_transfer = prv_abort_transfer,
};

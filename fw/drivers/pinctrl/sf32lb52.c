/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/pinctrl/sf32lb52.h>

#include <errno.h>
#include <stdbool.h>

#define FIELD_GET(msk, val) (((val) & (msk)) / ((msk) & ~((msk) << 1)))

#define DS_MSK     (3U << 9)
#define DS_POS     9U
#define DS_IDX_MSK (7U << 9)
#define CFG_MSK                                                                        \
  (PBL_PINCTRL_SF32LB52_FSEL_MSK | PBL_PINCTRL_SF32LB52_PE | PBL_PINCTRL_SF32LB52_PS | \
   PBL_PINCTRL_SF32LB52_IE | PBL_PINCTRL_SF32LB52_SR | DS_MSK)

#define FSEL_I2C        4U
#define PINR_I2C1_FIRST 0x48U
#define PINR_I2C4_LAST  0x54U

static inline uint32_t prv_read(uintptr_t addr) {
  return *(volatile uint32_t *)addr;
}

static inline void prv_write(uintptr_t addr, uint32_t val) {
  *(volatile uint32_t *)addr = val;
}

static int prv_configure(const struct pbl_pinctrl_sf32lb52 *ctrl, uint32_t pin) {
  uint32_t port = FIELD_GET(PBL_PINCTRL_SF32LB52_PORT_MSK, pin);
  uint32_t pad_num = FIELD_GET(PBL_PINCTRL_SF32LB52_PAD_MSK, pin);
  uint32_t ds_idx = FIELD_GET(DS_IDX_MSK, pin);
  uint32_t pinr_offset = FIELD_GET(PBL_PINCTRL_SF32LB52_PINR_MSK, pin);
  // PA39-PA42 have a single drive strength bit (4 or 20 mA) and use bit 8 as
  // the GPIO/I2C mode select instead of the slew rate.
  bool pa39_42 = (port == PBL_PINCTRL_SF32LB52_PORT_PA) && (pad_num >= 39U) && (pad_num <= 42U);
  uint32_t cfg_msk = CFG_MSK;
  uint32_t ds_reg;
  uintptr_t pad;
  uint32_t val;

  if (pa39_42) {
    if (ds_idx == 4U) {
      ds_reg = 1U;
    } else if ((ds_idx == 0U) || (ds_idx == 2U)) {
      ds_reg = 0U;
    } else {
      return -EINVAL;
    }
  } else {
    if (ds_idx == 4U) {
      return -EINVAL;
    }
    ds_reg = ds_idx;
  }

  if (pinr_offset != 0U) {
    uint32_t shift = 8U * FIELD_GET(PBL_PINCTRL_SF32LB52_FIELD_MSK, pin);

    val = prv_read(ctrl->cfg + pinr_offset);
    val &= ~(0xFFU << shift);
    val |= pad_num << shift;
    prv_write(ctrl->cfg + pinr_offset, val);
  }

  switch (port) {
    case PBL_PINCTRL_SF32LB52_PORT_SA:
      pad = ctrl->pad_sa;
      break;
    case PBL_PINCTRL_SF32LB52_PORT_PA:
      pad = ctrl->pad_pa;
      break;
    default:
      return -EINVAL;
  }
  pad += pad_num * 4U;

  if (pa39_42) {
    cfg_msk &= ~PBL_PINCTRL_SF32LB52_SR;
  }

  val = prv_read(pad);
  val &= ~cfg_msk;
  val |= pin & (cfg_msk & ~DS_MSK);
  val |= ds_reg << DS_POS;
  if (pa39_42) {
    val &= ~PBL_PINCTRL_SF32LB52_SR;
    if ((FIELD_GET(PBL_PINCTRL_SF32LB52_FSEL_MSK, pin) == FSEL_I2C) &&
        (pinr_offset >= PINR_I2C1_FIRST) && (pinr_offset <= PINR_I2C4_LAST)) {
      val |= PBL_PINCTRL_SF32LB52_SR;
    }
  }
  prv_write(pad, val);

  return 0;
}

int pbl_pinctrl_sf32lb52_apply(const struct pbl_pinctrl_sf32lb52_state *state) {
  for (uint8_t i = 0U; i < state->num_pins; i++) {
    int ret = prv_configure(state->ctrl, state->pins[i]);
    if (ret < 0) {
      return ret;
    }
  }

  return 0;
}

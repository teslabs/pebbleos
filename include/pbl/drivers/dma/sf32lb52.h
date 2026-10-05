/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include <devicetree/types/sifli,sf32lb52-dmac.h>

/**
 * @defgroup drivers_dma_sf32lb52 SF32LB52 DMA
 * @ingroup drivers
 * @brief Channels of the SF32LB52 DMA controller.
 * @{
 */

/**
 * @brief Registers of a DMA channel.
 *
 * @param dma Channel.
 * @return Address of its CCR register.
 */
static inline uintptr_t pbl_dma_sf32lb52_channel_regs(const struct pbl_dma_sf32lb52_channel *dma) {
  return dma->regs + 0x08U + (0x14U * dma->channel);
}

/** @} */

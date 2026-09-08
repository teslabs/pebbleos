/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "board/board.h"
#include "flash_region/flash_region.h"
#include <pbl/drivers/flash/qemu.h>

// UART device for debug serial
#include <pbl/drivers/uart/qemu.h>
#include <pbl/drivers/mic/qemu/mic_definitions.h>

static UARTDeviceState s_dbg_uart_state = {};

static struct UARTDevice DBG_UART_DEVICE = {
  .state = &s_dbg_uart_state,
  .base_addr = QEMU_UART2_BASE,
  .irqn = UART2_IRQn,
  .irq_priority = 5,
};

UARTDevice *const DBG_UART = (UARTDevice *)&DBG_UART_DEVICE;

// QEMU control protocol UART (UART1)
static UARTDeviceState s_qemu_uart_state = {};

static struct UARTDevice QEMU_UART_DEVICE = {
  .state = &s_qemu_uart_state,
  .base_addr = QEMU_UART1_BASE,
  .irqn = UART1_IRQn,
  .irq_priority = 6,
};

UARTDevice *const QEMU_UART = (UARTDevice *)&QEMU_UART_DEVICE;

static struct pbl_flash_device_state s_flash_state;
static const struct pbl_flash_qemu s_flash = {
    .dev =
        {
            .state = &s_flash_state,
            .ops = &pbl_flash_qemu_ops,
            .base = FLASH_REGION_BASE_ADDRESS,
            .size = BOARD_NOR_FLASH_SIZE,
            .sector_size = SECTOR_SIZE_BYTES,
            .subsector_size = SUBSECTOR_SIZE_BYTES,
        },
    .regs = QEMU_EXTFLASH_BASE,
};
const struct pbl_flash_device *const FLASH = &s_flash.dev;

// Display device - QEMU framebuffer
static QemuDisplayDevice s_display = {
  .base_addr = QEMU_DISPLAY_BASE,
  .fb_addr = QEMU_DISPLAY_FB_BASE,
  .width = 260,
  .height = 260,
  .bpp = 8,
  .irqn = DISPLAY_IRQn,
  .irq_priority = 5,
};

DisplayDevice *const DISPLAY = &s_display;

const BoardConfigPower BOARD_CONFIG_POWER = {
  .low_power_threshold = 2U,
  .battery_capacity_hours = 168U,
};

const BoardConfig BOARD_CONFIG = {
  .backlight_on_percent = 100,
  .ambient_light_dark_threshold = 150,
  .ambient_k_delta_threshold = 25,
};

const BoardConfigButton BOARD_CONFIG_BUTTON = {
  .buttons = {
    [BUTTON_ID_BACK]   = { "Back",   NULL, 0, GPIO_PuPd_NOPULL, true },
    [BUTTON_ID_UP]     = { "Up",     NULL, 1, GPIO_PuPd_UP, false },
    [BUTTON_ID_SELECT] = { "Select", NULL, 2, GPIO_PuPd_UP, false },
    [BUTTON_ID_DOWN]   = { "Down",   NULL, 3, GPIO_PuPd_UP, false },
  },
  .timer = NULL,
  .timer_irqn = TIMER0_IRQn,
};

// Microphone (QEMU stub — feeds silence on a timer)
static MicDeviceState s_mic_state;
static MicDevice MIC_DEVICE = {
  .state = &s_mic_state,
  .channels = 1,
};
MicDevice * const MIC = &MIC_DEVICE;

// IRQ handler trampolines
IRQ_MAP(UART2, uart_irq_handler, DBG_UART);
IRQ_MAP(UART1, uart_irq_handler, QEMU_UART);

void board_early_init(void) {
}

void board_init(void) {
}

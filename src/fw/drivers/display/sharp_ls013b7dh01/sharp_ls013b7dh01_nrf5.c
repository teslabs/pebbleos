#include "sharp_ls013b7dh01.h"

#include "applib/graphics/gtypes.h"
#include "board/board.h"
#include "debug/power_tracking.h"
#include "drivers/dma.h"
#include "drivers/gpio.h"
#include "drivers/periph_config.h"
#include "drivers/pwm.h"
#include "drivers/spi.h"
#include "kernel/util/sleep.h"
#include "kernel/util/stop.h"
#include "os/tick.h"
#include "services/common/analytics/analytics.h"
#include "system/logging.h"
#include "system/passert.h"
#include "util/bitset.h"
#include "util/net.h"
#include "util/reverse.h"
#include "util/units.h"


#define NRF5_COMPATIBLE
#include <mcu.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_rtc.h>
#include <nrfx_gpiote.h>
#include <nrfx_gppi.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// GPIO constants
static const unsigned int DISP_MODE_STATIC = 0x00;
static const unsigned int DISP_MODE_WRITE = 0x80;
static const unsigned int DISP_MODE_CLEAR = 0x20;

// We want the SPI clock to run at 1MHz by default
static uint32_t s_spi_clock_hz;

static bool s_initialized = false;

static volatile int s_spidma_waiting = 0;
static volatile int s_spidma_immediate = 0;

// DMA state
static DisplayContext s_display_context;
static uint32_t s_dma_line_buffer[DISP_DMA_BUFFER_SIZE_WORDS];

static SemaphoreHandle_t s_dma_update_in_progress_semaphore;

static void prv_spim_evt_handler(nrfx_spim_evt_t const *evt, void *ctx);
static void prv_display_context_init(DisplayContext* context);
static bool prv_do_dma_update(void);

static bool s_spim_initialized = false;

// Broadly, these two routines bracket enable_ and disable_chip_select, but
// in the full update use case, we only set up and tear down once to save
// some cycles (and avoid having to set up and tear down once per scanline).
static void prv_enable_spim(void) {
  PBL_ASSERTN(!s_spim_initialized);
  
  // Due to nRF52840 erratum 195, SPIM3 needs to be fully disabled
  // (uninit'ed) to stop drawing current, so rather than just configuring
  // the SPI peripheral once on boot, we configure it before every
  // transaction, then shut it down again.
  
  nrfx_spim_config_t config = NRFX_SPIM_DEFAULT_CONFIG(
    BOARD_CONFIG_DISPLAY.clk.gpio_pin,
    BOARD_CONFIG_DISPLAY.mosi.gpio_pin,
    NRF_SPIM_PIN_NOT_CONNECTED,
    NRF_SPIM_PIN_NOT_CONNECTED);
  config.frequency = NRFX_MHZ_TO_HZ(2);

  /* spim4 has hardware SS but it is tricky to convince NRFX to expose it to
   * us; for now, we use the classic enable chip select mechanism */
#if 0
  config.use_hw_ss = 1;
  config.ss_duration = 256; /* 4 us * 64MHz */
#endif

  nrfx_err_t err = nrfx_spim_init(&BOARD_CONFIG_DISPLAY.spi, &config, prv_spim_evt_handler, NULL);
  PBL_ASSERTN(err == NRFX_SUCCESS);
  
  s_spim_initialized = true;
}

static void prv_disable_spim(void) {
  PBL_ASSERTN(s_spim_initialized);
  
  // includes WAR for erratum 195, in nrfx driver
  nrfx_spim_uninit(&BOARD_CONFIG_DISPLAY.spi);

  s_spim_initialized = false;
}

static void prv_enable_chip_select(void) {
  gpio_output_set(&BOARD_CONFIG_DISPLAY.cs, true);
  // setup time > 3us
  // this produces a setup time of ~7us
  for (volatile int i = 0; i < 32; i++);
}

static void prv_disable_chip_select(void) {
  // delay while last byte is emitted by the SPI peripheral (~7us)
  for (volatile int i = 0; i < 48; i++);
  gpio_output_set(&BOARD_CONFIG_DISPLAY.cs, false);
  // hold time > 1us
  // this produces a delay of ~3.5us
  for (volatile int i = 0; i < 16; i++);
}


static void prv_display_start(void) {
  periph_config_acquire_lock();

  gpio_output_init(&BOARD_CONFIG_DISPLAY.cs, GPIO_OType_PP, GPIO_Speed_50MHz);

  gpio_output_init(&BOARD_CONFIG_DISPLAY.on_ctrl,
                   BOARD_CONFIG_DISPLAY.on_ctrl_otype,
                   GPIO_Speed_50MHz);

  // +5V to LCD_DISP pin (Set this pin low to turn off the display)
  gpio_output_set(&BOARD_CONFIG_DISPLAY.on_ctrl, true);

  periph_config_release_lock();
}

uint32_t display_baud_rate_change(uint32_t new_frequency_hz) {
  // Take the semaphore so that we can be sure that we are not interrupting a transfer
  xSemaphoreTake(s_dma_update_in_progress_semaphore, portMAX_DELAY);

  uint32_t old_spi_clock_hz = s_spi_clock_hz;
  s_spi_clock_hz = new_frequency_hz;

  xSemaphoreGive(s_dma_update_in_progress_semaphore);
  return old_spi_clock_hz;
}

static uint8_t s_ppi_ch[2];
static uint8_t s_gpiote_ch;
static const nrfx_gpiote_t s_gpiote = NRFX_GPIOTE_INSTANCE(0);

static void prv_extcomin_toggle_init(void) {
  nrfx_err_t err;
  uint32_t psel;
  NRF_RTC_Type *rtc;
  const nrfx_gpiote_t *gpiote;
  NRF_GPIOTE_Type *gpiote_reg;
  uint32_t pulse_end_event_address, period_end_event_address;
  nrf_gpiote_task_t pulse_end_task, period_end_task;
  uint32_t pulse_end_task_address, period_end_task_address, clear_task_address;
  uint32_t ppi_mask;

  psel = NRF_GPIO_PIN_MAP(1, 15);
  rtc = NRF_RTC2;
  gpiote = &s_gpiote;
  gpiote_reg = gpiote->p_reg;

  if (!nrfx_gpiote_init_check(gpiote)) {
    nrfx_err_t err = nrfx_gpiote_init(gpiote, NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
    PBL_ASSERTN(err == NRFX_SUCCESS);
  }

  err = nrfx_gppi_channel_alloc(&s_ppi_ch[0]);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  err = nrfx_gppi_channel_alloc(&s_ppi_ch[1]);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  ppi_mask = (1UL << s_ppi_ch[0]) | (1UL << s_ppi_ch[1]);

  err = nrfx_gpiote_channel_alloc(gpiote, &s_gpiote_ch);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  nrf_gpio_pin_write(psel, 0);
  nrf_gpio_cfg_output(psel);

  nrf_rtc_prescaler_set(rtc, NRF_RTC_FREQ_TO_PRESCALER(32768));
  nrf_rtc_event_enable(rtc,
                       (NRF_RTC_INT_COMPARE0_MASK |
                        NRF_RTC_INT_COMPARE1_MASK));

  nrf_gpio_pin_write(psel, 0);
  nrf_gpiote_te_default(gpiote_reg, s_gpiote_ch);
  nrf_rtc_task_trigger(rtc, NRF_RTC_TASK_STOP);

  nrfx_gppi_channels_disable(ppi_mask);

  nrf_rtc_event_clear(rtc, nrf_rtc_compare_event_get(1));
  nrf_rtc_event_clear(rtc, nrf_rtc_compare_event_get(0));

  nrf_rtc_cc_set(rtc, 1, 128 - 1);
  nrf_rtc_cc_set(rtc, 0, 32768 / 120 - 1);
  nrf_rtc_task_trigger(rtc, NRF_RTC_TASK_CLEAR);

  gpiote_reg->CONFIG[s_gpiote_ch] =
    (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos) |
    (psel << 8U) |
    (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
    (1UL << GPIOTE_CONFIG_OUTINIT_Pos);

  pulse_end_task  = nrf_gpiote_set_task_get(s_gpiote_ch);
  period_end_task = nrf_gpiote_clr_task_get(s_gpiote_ch);

  pulse_end_task_address = nrf_gpiote_task_address_get(gpiote_reg, pulse_end_task);
  period_end_task_address = nrf_gpiote_task_address_get(gpiote_reg, period_end_task);

  clear_task_address = nrf_rtc_event_address_get(rtc, NRF_RTC_TASK_CLEAR);
  pulse_end_event_address = nrf_rtc_event_address_get(rtc, nrf_rtc_compare_event_get(1));
  period_end_event_address = nrf_rtc_event_address_get(rtc, nrf_rtc_compare_event_get(0));

  nrfx_gppi_fork_endpoint_setup(s_ppi_ch[1], clear_task_address);

  nrfx_gppi_channel_endpoints_setup(s_ppi_ch[0], pulse_end_event_address, pulse_end_task_address);
  nrfx_gppi_channel_endpoints_setup(s_ppi_ch[1], period_end_event_address, period_end_task_address);
  nrfx_gppi_channels_enable(ppi_mask);

  nrf_rtc_task_trigger(rtc, NRF_RTC_TASK_START);
}

void display_init(void) {
  if (s_initialized) {
    return;
  }

  //s_spi_clock_hz = MHZ_TO_HZ(1);

  //prv_display_context_init(&s_display_context);

  //vSemaphoreCreateBinary(s_dma_update_in_progress_semaphore);

  //prv_display_start();

  prv_extcomin_toggle_init();

  s_initialized = true;
}

static void prv_display_context_init(DisplayContext* context) {
  context->state = DISPLAY_STATE_IDLE;
  context->get_next_row = NULL;
  context->complete = NULL;
}

static void prv_spim_evt_handler(nrfx_spim_evt_t const *evt, void *ctx) {
  s_spidma_waiting = 0;
  if (!s_spidma_immediate) {
    bool needs_switch = prv_do_dma_update();
    portEND_SWITCHING_ISR(needs_switch);
  }
}

static void prv_display_write_async(const uint8_t *buf, size_t len) {
  nrfx_spim_xfer_desc_t desc = {
    .p_tx_buffer = buf,
    .tx_length = len
  };

  PBL_ASSERTN(!s_spidma_waiting);
    
  s_spidma_waiting = 1;
  s_spidma_immediate = 0;

  nrfx_err_t err = nrfx_spim_xfer(&BOARD_CONFIG_DISPLAY.spi, &desc, 0);
  PBL_ASSERTN(err == NRFX_SUCCESS);
}

static void prv_display_write_sync(const uint8_t *buf, size_t len) {
  nrfx_spim_xfer_desc_t desc = {
    .p_tx_buffer = buf,
    .tx_length = len
  };

  PBL_ASSERTN(!s_spidma_waiting);
    
  s_spidma_waiting = 1;
  s_spidma_immediate = 1;

  nrfx_err_t err = nrfx_spim_xfer(&BOARD_CONFIG_DISPLAY.spi, &desc, 0);
  PBL_ASSERTN(err == NRFX_SUCCESS);
  
  while (s_spidma_waiting)
    /* XXX: ... yield, or something.  maybe a semaphore would be nicer here.  it should be fast, though */;
  s_spidma_immediate = 0;
}

// Clear-all mode is entered by sending 0x04 to the panel
void display_clear(void) {
  uint8_t buf[] = { DISP_MODE_CLEAR, 0x00 };
  prv_enable_spim();
  prv_enable_chip_select();
  prv_display_write_sync(buf, sizeof(buf));
  prv_disable_chip_select();
  prv_disable_spim();
}

void display_set_enabled(bool enabled) {
    gpio_output_set(&BOARD_CONFIG_DISPLAY.on_ctrl, enabled);
}

bool display_update_in_progress(void) {
  if (xSemaphoreTake(s_dma_update_in_progress_semaphore, 0) == pdPASS) {
    xSemaphoreGive(s_dma_update_in_progress_semaphore);
    return false;
  }
  return true;
}

// Static mode is entered by sending 0x00 to the panel
static void prv_display_enter_static(void) {
  uint8_t buf[] = { DISP_MODE_STATIC, 0x00, 0x00 };
  prv_enable_spim();
  prv_enable_chip_select();
  prv_display_write_sync(buf, sizeof(buf));
  prv_disable_chip_select();
  prv_disable_spim();
}

void display_update(NextRowCallback nrcb, UpdateCompleteCallback uccb) {
  PBL_ASSERTN(nrcb != NULL);
  PBL_ASSERTN(uccb != NULL);
  stop_mode_disable(InhibitorDisplay);
  xSemaphoreTake(s_dma_update_in_progress_semaphore, portMAX_DELAY);
  analytics_stopwatch_start(ANALYTICS_APP_METRIC_DISPLAY_WRITE_TIME, AnalyticsClient_App);
  analytics_inc(ANALYTICS_DEVICE_METRIC_DISPLAY_UPDATES_PER_HOUR, AnalyticsClient_System);

  power_tracking_start(PowerSystemMcuDma1);

  prv_enable_spim();

  prv_display_context_init(&s_display_context);
  s_display_context.get_next_row = nrcb;
  s_display_context.complete = uccb;

  prv_do_dma_update();

  // Block while we wait for the update to finish.
  TickType_t ticks = milliseconds_to_ticks(4000); // DMA should be fast
  if (xSemaphoreTake(s_dma_update_in_progress_semaphore, ticks) != pdTRUE) {
    uint32_t pri_mask = __get_PRIMASK();
    PBL_CROAK("display DMA failed: 0x%" PRIx32, pri_mask);
  }

  power_tracking_stop(PowerSystemMcuDma1);

  /* needs to not happen from the ISR, because write_sync depends on the ISR to be called again */
  uint8_t buf[] = { 0x00 };
  prv_display_write_sync(buf, sizeof(buf));
  prv_disable_chip_select();

  prv_disable_spim();

  prv_display_enter_static();

  xSemaphoreGive(s_dma_update_in_progress_semaphore);
  stop_mode_enable(InhibitorDisplay);
  analytics_stopwatch_stop(ANALYTICS_APP_METRIC_DISPLAY_WRITE_TIME);
}

void display_pulse_vcom(void) {
}

#if DISPLAY_ORIENTATION_ROTATED_180
//!
//! memcpy the src buffer to dst and reverse the bits
//! to match the display order
//!
static void prv_memcpy_reverse_bytes(uint8_t* dst, uint8_t* src, int bytes) {
    // Skip the mode selection and column address bytes
    dst+=2;
    while (bytes--) {
        *dst++ = reverse_byte(*src++);
    }
}
#else
//!
//! memcpy the src buffer to dst backwards (i.e. the highest src byte
//! is the lowest byte in dst.
//!
static void prv_memcpy_backwards(uint32_t* dst, uint32_t* src, int length) {
  dst += length - 1;
  while (length--) {
    *dst-- = ntohl(*src++);
  }
}
#endif


static bool prv_do_dma_update(void) {
  DisplayRow r;

  PBL_ASSERTN(s_display_context.get_next_row != NULL);
  bool is_end_of_buffer = !s_display_context.get_next_row(&r);

  switch (s_display_context.state) {
  case DISPLAY_STATE_IDLE:
  {
    if (is_end_of_buffer) {
      // If nothing has been modified, bail out early
      return false;
    }
    
    prv_enable_chip_select();

    s_display_context.state = DISPLAY_STATE_WRITING;

#if DISPLAY_ORIENTATION_ROTATED_180
      prv_memcpy_reverse_bytes((uint8_t*)s_dma_line_buffer, r.data, DISP_LINE_BYTES);
      s_dma_line_buffer[0] &= ~(0xffff);
      s_dma_line_buffer[0] |= (DISP_MODE_WRITE | reverse_byte(r.address + 1) << 8);
#else
      prv_memcpy_backwards(s_dma_line_buffer, (uint32_t*)r.data, DISP_LINE_WORDS);
      s_dma_line_buffer[0] &= ~(0xffff);
      s_dma_line_buffer[0] |= (DISP_MODE_WRITE | reverse_byte(167 - r.address + 1) << 8);
#endif
    prv_display_write_async(((uint8_t*) s_dma_line_buffer), DISP_DMA_BUFFER_SIZE_BYTES);

    break;
  }
  case DISPLAY_STATE_WRITING:
  {
    if (is_end_of_buffer) {
      s_display_context.complete();

      signed portBASE_TYPE was_higher_priority_task_woken = pdFALSE;
      xSemaphoreGiveFromISR(s_dma_update_in_progress_semaphore, &was_higher_priority_task_woken);

      return was_higher_priority_task_woken != pdFALSE;
    }

#if DISPLAY_ORIENTATION_ROTATED_180
    prv_memcpy_reverse_bytes((uint8_t*)s_dma_line_buffer, r.data, DISP_LINE_BYTES);
    s_dma_line_buffer[0] &= ~(0xffff);
    s_dma_line_buffer[0] |= (DISP_MODE_WRITE | reverse_byte(r.address + 1) << 8);
#else
    prv_memcpy_backwards(s_dma_line_buffer, (uint32_t*)r.data, DISP_LINE_WORDS);
    s_dma_line_buffer[0] &= ~(0xffff);
    s_dma_line_buffer[0] |= reverse_byte(167 - r.address + 1) << 8;
#endif
    prv_display_write_async(((uint8_t*) s_dma_line_buffer) + 1, DISP_DMA_BUFFER_SIZE_BYTES - 1);
    break;
  }
  default:
    WTF;
  }
  return false;
}

void display_show_splash_screen(void) {
  // The bootloader has already drawn the splash screen for us; nothing to do!
}

// Stubs for display offset
void display_set_offset(GPoint offset) {}

GPoint display_get_offset(void) { return GPointZero; }

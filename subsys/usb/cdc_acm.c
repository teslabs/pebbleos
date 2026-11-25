/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <stdatomic.h>
#include <string.h>

#include <pbl/drivers/udc.h>
#include <pbl/kernel/irq.h>
#include <pbl/usb/cdc_acm.h>
#include <pbl/usb/usbd.h>
#include <pbl/util/math.h>
#include <pbl/util/size.h>

#define IFACE_COMM 0U
#define IFACE_DATA 1U

#define STRING_NAME 1U

#define EP_NOTIFY   (PBL_UDC_EP_IN | 1U)
#define EP_DATA_OUT 2U
#define EP_DATA_IN  (PBL_UDC_EP_IN | 3U)

#define CDC_CLASS            0x02U
#define CDC_SUBCLASS_ACM     0x02U
#define CDC_PROTOCOL_AT      0x01U
#define CDC_DATA_CLASS       0x0aU
#define CDC_FUNC_HEADER      0x00U
#define CDC_FUNC_CALL_MGMT   0x01U
#define CDC_FUNC_ACM         0x02U
#define CDC_FUNC_UNION       0x06U
#define CDC_ACM_CAP_LINE     0x02U
#define CDC_SET_LINE_CODING  0x20U
#define CDC_GET_LINE_CODING  0x21U
#define CDC_SET_CONTROL_LINE 0x22U
#define CDC_SEND_BREAK       0x23U

// Largest bulk packet, of high speed
#define BUF_LEN 512U
#define TX_RING 1024U

static const uint8_t s_desc[] = {
  // Association of the two interfaces
  8,
  PBL_USB_DESC_IAD,
  IFACE_COMM,
  2,
  CDC_CLASS,
  CDC_SUBCLASS_ACM,
  CDC_PROTOCOL_AT,
  STRING_NAME,
  // Communication interface, with its functional descriptors and notification endpoint
  9,
  PBL_USB_DESC_INTERFACE,
  IFACE_COMM,
  0,
  1,
  CDC_CLASS,
  CDC_SUBCLASS_ACM,
  CDC_PROTOCOL_AT,
  STRING_NAME,
  5,
  PBL_USB_DESC_CS_INTERFACE,
  CDC_FUNC_HEADER,
  0x10,
  0x01,
  5,
  PBL_USB_DESC_CS_INTERFACE,
  CDC_FUNC_CALL_MGMT,
  0,
  IFACE_DATA,
  4,
  PBL_USB_DESC_CS_INTERFACE,
  CDC_FUNC_ACM,
  CDC_ACM_CAP_LINE,
  5,
  PBL_USB_DESC_CS_INTERFACE,
  CDC_FUNC_UNION,
  IFACE_COMM,
  IFACE_DATA,
  7,
  PBL_USB_DESC_ENDPOINT,
  EP_NOTIFY,
  PBL_UDC_EP_INTERRUPT,
  16,
  0,
  16,
  // Data interface
  9,
  PBL_USB_DESC_INTERFACE,
  IFACE_DATA,
  0,
  2,
  CDC_DATA_CLASS,
  0,
  0,
  0,
  7,
  PBL_USB_DESC_ENDPOINT,
  EP_DATA_OUT,
  PBL_UDC_EP_BULK,
  64,
  0,
  0,
  7,
  PBL_USB_DESC_ENDPOINT,
  EP_DATA_IN,
  PBL_UDC_EP_BULK,
  64,
  0,
  0,
};

static const char *const s_strings[] = {
  CONFIG_USB_CDC_ACM_NAME,
};

// Transfers on each data endpoint, so that the controller goes on to the next one while the last
// one is handed over
#define XFERS PBL_UDC_EP_QUEUE_LEN

static struct {
  atomic_bool configured;
  // Transfers sending, the oldest one first
  uint8_t tx_n;
  uint8_t tx_first;
  size_t tx_len[XFERS];
  uint16_t tx_head;
  uint16_t tx_tail;
  // The receive buffer the next transfer that ends fills
  uint8_t rx_next;
  // 115200 baud, 8N1, until the host sets its own
  uint8_t line_coding[7];
  uint8_t tx_ring[TX_RING];
  uint8_t tx_buf[XFERS][BUF_LEN];
  uint8_t rx_buf[XFERS][BUF_LEN];
} s_acm = {
  .line_coding = {0x00, 0xc2, 0x01, 0x00, 0, 0, 8},
};

static uint16_t prv_ring_used(void) {
  return (uint16_t)((s_acm.tx_head - s_acm.tx_tail) % TX_RING);
}

// With the IRQ lock held: a transfer of up to a buffer, of what the ring holds, or of nothing to
// end the one before it
static void prv_tx_queue(size_t len) {
  uint8_t idx = (uint8_t)((s_acm.tx_first + s_acm.tx_n) % XFERS);

  for (size_t i = 0U; i < len; i++) {
    s_acm.tx_buf[idx][i] = s_acm.tx_ring[s_acm.tx_tail];
    s_acm.tx_tail = (uint16_t)((s_acm.tx_tail + 1U) % TX_RING);
  }

  s_acm.tx_len[idx] = len;
  if (pbl_udc_ep_queue(UDC, EP_DATA_IN, s_acm.tx_buf[idx], len) == 0) {
    s_acm.tx_n++;
  }
}

// With the IRQ lock held: what the ring holds goes out, with a second transfer queued behind the
// one under way, which the controller starts as soon as that one ends
static void prv_tx_start(void) {
  uint16_t used = prv_ring_used();

  if (s_acm.tx_n < XFERS && used != 0U) {
    prv_tx_queue(PBL_MIN((size_t)used, (size_t)BUF_LEN));
  }
}

static void prv_rx_queue(uint8_t idx) {
  (void)pbl_udc_ep_queue(UDC, EP_DATA_OUT, s_acm.rx_buf[idx], pbl_usbd_bulk_mps());
}

static void prv_configured(bool on) {
  pbl_irq_lock();
  atomic_store(&s_acm.configured, on);
  s_acm.tx_n = 0U;
  s_acm.tx_first = 0U;
  s_acm.rx_next = 0U;
  if (on) {
    prv_tx_start();
  }
  pbl_irq_unlock();

  if (on) {
    for (uint8_t i = 0U; i < XFERS; i++) {
      prv_rx_queue(i);
    }
  }
}

static int prv_class_request(const struct pbl_usb_setup *setup, uint8_t **data, uint16_t *len) {
  switch (setup->bRequest) {
    case CDC_SET_LINE_CODING:
    case CDC_GET_LINE_CODING:
      *data = s_acm.line_coding;
      *len = sizeof(s_acm.line_coding);
      return 0;
    case CDC_SET_CONTROL_LINE:
    case CDC_SEND_BREAK:
      return 0;
    default:
      return -ENOTSUP;
  }
}

// A transfer that fills its packets ends with a zero-length one, unless more follows, for the host
// to take it now
static void prv_xfer_done(uint8_t ep, int ret) {
  size_t len;

  if (ep == EP_DATA_OUT) {
    uint8_t idx = s_acm.rx_next;

    s_acm.rx_next = (uint8_t)((idx + 1U) % XFERS);
    for (int i = 0; i < ret; i++) {
      (void)pbl_uart_receive(&pbl_usb_cdc_acm, s_acm.rx_buf[idx][i]);
    }
    if (ret >= 0 && atomic_load(&s_acm.configured)) {
      prv_rx_queue(idx);
    }
    return;
  }

  if (ep != EP_DATA_IN) {
    return;
  }

  pbl_irq_lock();
  if (s_acm.tx_n == 0U) {
    pbl_irq_unlock();
    return;
  }
  len = s_acm.tx_len[s_acm.tx_first];
  s_acm.tx_first = (uint8_t)((s_acm.tx_first + 1U) % XFERS);
  s_acm.tx_n--;
  if (ret >= 0 && atomic_load(&s_acm.configured)) {
    if (prv_ring_used() != 0U) {
      prv_tx_start();
    } else if (s_acm.tx_n == 0U && len != 0U && (len % pbl_usbd_bulk_mps()) == 0U) {
      prv_tx_queue(0U);
    }
  }
  pbl_irq_unlock();
}

PBL_USBD_FUNCTION_DEFINE(cdc_acm) = {
  .desc = s_desc,
  .desc_len = sizeof(s_desc),
  .strings = s_strings,
  .num_strings = PBL_ARRAY_SIZE(s_strings),
  .first_iface = IFACE_COMM,
  .num_ifaces = 2U,
  .configured = prv_configured,
  .class_request = prv_class_request,
  .xfer_done = prv_xfer_done,
};

static int prv_init(const struct pbl_uart *uart) {
  return pbl_usbd_enable();
}

static int prv_configure(const struct pbl_uart *uart, const struct pbl_uart_config *cfg) {
  return 0;
}

// Waits for room in the ring, as long as the host keeps the port configured
static void prv_poll_out(const struct pbl_uart *uart, uint8_t c) {
  while (atomic_load(&s_acm.configured)) {
    pbl_irq_lock();
    if (prv_ring_used() < TX_RING - 1U) {
      s_acm.tx_ring[s_acm.tx_head] = c;
      s_acm.tx_head = (uint16_t)((s_acm.tx_head + 1U) % TX_RING);
      prv_tx_start();
      pbl_irq_unlock();
      return;
    }
    pbl_irq_unlock();
  }
}

static int prv_rx_enable(const struct pbl_uart *uart) {
  return 0;
}

static void prv_rx_disable(const struct pbl_uart *uart) {
}

static const struct pbl_uart_ops s_ops = {
  .init = prv_init,
  .configure = prv_configure,
  .poll_out = prv_poll_out,
  .rx_enable = prv_rx_enable,
  .rx_disable = prv_rx_disable,
};

struct pbl_usb_cdc_acm {
  struct pbl_uart uart;
};

PBL_UART_STATE_DEFINE(pbl_usb_cdc_acm);
PBL_DEVICE_INSTANCE(pbl_usb_cdc_acm, struct pbl_usb_cdc_acm, struct pbl_uart, uart){
  .uart = PBL_UART_INIT(pbl_usb_cdc_acm, "usb_acm", nullptr, &s_ops, 115200),
};

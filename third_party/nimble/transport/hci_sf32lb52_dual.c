/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include <bf0_hal.h>
#include <ipc_queue.h>
#include <kernel/pebble_tasks.h>
#include <kernel/pbl_malloc.h>
#include <pbl/kernel/mutex.h>
#include <pbl/kernel/sem.h>
#include <pbl/kernel/thread.h>
#include <system/passert.h>
#include <pbl/util/size.h>
#include <os/os_mbuf.h>
#include <nimble/transport.h>
#include <nimble/transport_impl.h>
#include <host/ble_hs.h>
#include <nimble/nimble_npl.h>
#include "../../../src/bluetooth-fw/hci_bridge/h4_stream.h"
#include "../../../src/bluetooth-fw/hci_bridge/local_audio.h"
#include "../../../src/bluetooth-fw/hci_bridge/sf32lb52_audio_probe.h"

static ipc_queue_handle_t s_port = IPC_QUEUE_INVALID_HANDLE;
static PBL_MUTEX_DEFINE(s_io);
static PBL_SEM_DEFINE(s_received, 0, 1);
PBL_THREAD_STACK_DEFINE(s_stack, 4096);
static H4Stream s_stream;
static uint8_t s_receive[1030];

typedef struct {
  uint16_t length;
  uint8_t data[1030];
} PendingPacket;
// Preserve reliable traffic while the host drains its pools; never block HCI RX.
static PendingPacket s_pending[8];
static unsigned s_head, s_count;

#ifdef CONFIG_PROMPT
// Metadata only: retain controller ordering in a crash dump without keys or audio.
typedef struct {
  uint32_t ticks;
  uint16_t code, value;
  uint8_t status, detail;
} HciHistoryEntry;
static volatile HciHistoryEntry *s_hci_history;
#define HCI_HISTORY_COUNT 64
static volatile unsigned s_hci_history_count;
static bool s_hci_history_frozen;
static volatile uint32_t s_hci_hw_error[5];

static void trace_hci(const uint8_t *p, size_t length) {
  if (!s_hci_history || s_hci_history_frozen || length < 4 || (p[0] != 1 && p[0] != 4) ||
      (p[0] == 4 && p[1] == 0x13))
    return;
  unsigned index = s_hci_history_count++ % HCI_HISTORY_COUNT;
  s_hci_history[index].ticks = ble_npl_time_get();
  s_hci_history[index].code = p[0] == 1 ? (0x8000 | p[1] | (p[2] << 8)) : p[1];
  s_hci_history[index].status = 0;
  s_hci_history[index].value = 0;
  s_hci_history[index].detail = 0;
  if (p[0] == 4) {
    switch (p[1]) {
      case 3:
      case 5:
      case 6:
      case 8:
      case 0x0e:
      case 0x0f:
      case 0x10:
      case 0x12:
      case 0x2c:
      case 0x30:
      case 0x3e:
        s_hci_history[index].status = p[3];
    }
  }
  if (p[0] == 4 && length >= 7) {
    if (p[1] == 0x0e || p[1] == 5) {
      s_hci_history[index].value = p[4] | (p[5] << 8);
      s_hci_history[index].detail = p[6];
    } else if (p[1] == 0x2c && length == 20) {
      s_hci_history[index].value = p[4] | (p[5] << 8);
      s_hci_history[index].detail = p[12];
    } else if (p[1] == 0x0f) {
      s_hci_history[index].value = p[5] | (p[6] << 8);
    }
  }
  if (p[0] == 4 && p[1] == 0x10) {
    s_hci_history_frozen = true;
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
    s_hci_hw_error[0] = hwp_bt_mac->BTERRORTYPESTAT;
    s_hci_hw_error[1] = hwp_bt_mac->DMERRORTYPESTAT;
    s_hci_hw_error[2] = hwp_bt_mac->BLEERRORTYPESTAT;
    s_hci_hw_error[3] = hwp_bt_mac->BTDEBUGADDMIN;
    s_hci_hw_error[4] = hwp_bt_mac->BTDEBUGADDMAX;
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
  }
}
#else
#define trace_hci(p, length) ((void)0)
#endif

extern void lcpu_power_on(void);
extern uint8_t lcpu_power_off(void);
extern void lcpu_custom_nvds_config(void);

void hci_bridge_transport_wake(void) {
  pbl_sem_give(&s_received);
}

static int32_t received(ipc_queue_handle_t port, size_t length) {
  hci_bridge_transport_wake();
  return 0;
}

static void write_raw(const void *data, size_t length) {
  const uint8_t *p = data;
  while (length) {
    size_t written = ipc_queue_write(s_port, p, length, 10);
    PBL_ASSERTN(written && written <= length);
    p += written;
    length -= written;
  }
}

static bool deliver(const uint8_t *p, size_t length) {
  if (p[0] == 4) {
    bool ack = p[1] == 0x0e || p[1] == 0x0f;
    void *event = ble_transport_alloc_evt(0);
    // The one outstanding command has already returned its TX buffer.
    if (!event && ack)
      event = ble_transport_alloc_cmd();
    if (!event)
      return false;
    PBL_ASSERTN(length - 1 <= MYNEWT_VAL(BLE_TRANSPORT_EVT_SIZE));
    memcpy(event, p + 1, length - 1);
    ble_transport_to_hs_evt(event);
    return true;
  }
  PBL_ASSERTN(p[0] == 2);
  struct os_mbuf *om = ble_transport_alloc_acl_from_ll();
  if (!om)
    return false;
  if (os_mbuf_append(om, p + 1, length - 1)) {
    os_mbuf_free_chain(om);
    return false;
  }
  ble_transport_to_hs_acl(om);
  return true;
}

static void controller_packet(uint8_t *p, size_t length, void *context) {
  trace_hci(p, length);
  if (p[0] == 4) {
    // Unsolicited SiFli boot notification, not an acknowledgement to NimBLE.
    if (length >= 6 && p[1] == 0x0e && p[4] == 0x11 && p[5] == 0xfc)
      return;
    hci_bridge_audio_event(p, length);
  }
  if (!hci_local_audio_receive(p, length))
    return;
  bool ack = p[0] == 4 && (p[1] == 0x0e || p[1] == 0x0f);
  if ((!s_count || ack) && deliver(p, length))
    return;
  // Losing reliable HCI traffic requires recovery, never silent corruption.
  PBL_ASSERTN(!ack && s_count < ARRAY_LENGTH(s_pending));
  PendingPacket *pending = &s_pending[(s_head + s_count++) % ARRAY_LENGTH(s_pending)];
  pending->length = length;
  memcpy(pending->data, p, length);
}

static void receive_task(void *context) {
  uint8_t buffer[256];
  for (;;) {
    pbl_sem_take(&s_received, hci_bridge_audio_active() || s_count ? PBL_MSEC(5) : PBL_FOREVER);
    hci_local_audio_poll();
    pbl_mutex_lock(&s_io, PBL_FOREVER);
    if (s_port == IPC_QUEUE_INVALID_HANDLE) {
      pbl_mutex_unlock(&s_io);
      continue;
    }
    size_t length;
    // Bound each pass so a stream of ACL traffic cannot starve synchronous audio.
    for (unsigned i = 0; i < 16; ++i) {
      length = ipc_queue_read(s_port, buffer, sizeof(buffer));
      if (!length)
        break;
      PBL_ASSERTN(h4_stream_feed(&s_stream, buffer, length, controller_packet, NULL));
    }
    while (s_count && deliver(s_pending[s_head].data, s_pending[s_head].length)) {
      s_head = (s_head + 1) % ARRAY_LENGTH(s_pending);
      --s_count;
    }
    hci_bridge_audio_pump(hci_local_audio_receive, hci_local_audio_transmit);
    pbl_mutex_unlock(&s_io);
  }
}

void ble_transport_ll_reinit(void) {
  pbl_mutex_lock(&s_io, PBL_FOREVER);
#ifdef CONFIG_PROMPT
  // Allocate after the boot splash releases its temporary frame buffer.
  if (!s_hci_history)
    s_hci_history = kernel_zalloc(HCI_HISTORY_COUNT * sizeof(*s_hci_history));
#endif
  h4_stream_init(&s_stream, s_receive, sizeof(s_receive));
  s_head = s_count = 0;
  ipc_queue_cfg_t cfg = {
    .qid = 0,
    .tx_buf_size = HCPU2LCPU_MB_CH1_BUF_SIZE,
    .tx_buf_addr = HCPU2LCPU_MB_CH1_BUF_START_ADDR,
    .tx_buf_addr_alias = HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH1_BUF_START_ADDR),
    .rx_ind = received,
  };
  cfg.rx_buf_addr = __HAL_SYSCFG_GET_REVID() < HAL_CHIP_REV_ID_A4
                        ? LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_START_ADDR)
                        : LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_REV_B_START_ADDR);
  s_port = ipc_queue_init(&cfg);
  PBL_ASSERTN(s_port != IPC_QUEUE_INVALID_HANDLE);
  NVIC_SetPriority(LCPU2HCPU_IRQn, 5);
  PBL_ASSERTN(ipc_queue_open(s_port) == 0);
  lcpu_custom_nvds_config();
  lcpu_power_on();
  hci_bridge_audio_probe_init();
  pbl_mutex_unlock(&s_io);
}

void ble_transport_ll_init(void) {
  ble_transport_ll_reinit();
  struct pbl_thread_attr attr = {
    .name = "NimbleHCI",
    .entry = receive_task,
    .prio = PBL_PRIO_IDLE + 3,
    .privileged = true,
    .stack = s_stack,
    .stack_size = sizeof(s_stack),
  };
  PBL_ASSERTN(pebble_task_create(PebbleTask_BTHCI, &attr));
}

void ble_transport_ll_deinit(void) {
  pbl_mutex_lock(&s_io, PBL_FOREVER);
  const uint8_t reset[] = {1, 3, 12, 0};
  hci_local_audio_command(reset, sizeof(reset));
  uint8_t response[8];
  hci_bridge_audio_command(reset, sizeof(reset), response);
  NVIC_DisableIRQ(LCPU2HCPU_IRQn);
  ipc_queue_close(s_port);
  ipc_queue_deinit(s_port);
  s_port = IPC_QUEUE_INVALID_HANDLE;
  HAL_HPAON_WakeCore(CORE_ID_LCPU);
  lcpu_power_off();
  HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
  pbl_mutex_unlock(&s_io);
}

int ble_transport_to_ll_cmd_impl(void *buf) {
  const uint8_t *cmd = buf;
  uint8_t packet[259] = {1};
  size_t length = cmd[2] + 4;
  memcpy(packet + 1, cmd, length - 1);
  // Free before publishing an acknowledgement, including synthetic SCO replies.
  ble_transport_free(buf);
  pbl_mutex_lock(&s_io, PBL_FOREVER);
  trace_hci(packet, length);
  hci_local_audio_command(packet, length);
  uint8_t response[8];
  size_t response_length = hci_bridge_audio_command(packet, length, response);
  if (response_length)
    controller_packet(response, response_length, NULL);
  else
    write_raw(packet, length);
  pbl_mutex_unlock(&s_io);
  return 0;
}

int ble_transport_to_ll_acl_impl(struct os_mbuf *om) {
  const uint8_t type = 2;
  pbl_mutex_lock(&s_io, PBL_FOREVER);
  write_raw(&type, 1);
  for (struct os_mbuf *part = om; part; part = SLIST_NEXT(part, om_next))
    write_raw(part->om_data, part->om_len);
  pbl_mutex_unlock(&s_io);
  os_mbuf_free_chain(om);
  return 0;
}

int ble_transport_to_ll_iso_impl(struct os_mbuf *om) {
  os_mbuf_free_chain(om);
  return BLE_HS_ENOTSUP;
}

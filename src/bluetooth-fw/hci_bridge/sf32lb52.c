/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "transport.h"
#include "sf32lb52_audio_probe.h"
#ifdef CONFIG_BT_HCI_LOCAL_AUDIO
#include "local_audio.h"
#endif
#ifdef CONFIG_BT_HCI_AUDIO_ADAPTER
#include "h4_stream.h"
#include <pbl/kernel/msgq.h>
#endif

#include <bf0_hal.h>
#include <ipc_queue.h>
#include <kernel/pebble_tasks.h>
#include <pbl/kernel/sem.h>
#include <pbl/kernel/thread.h>
#include <system/passert.h>
#ifdef CONFIG_BT_FW_CLASSIC_DEMO
#include "../classic_demo/service.h"
#include <pbl/kernel/sched.h>
#endif

static ipc_queue_handle_t s_port = IPC_QUEUE_INVALID_HANDLE;
static PBL_SEM_DEFINE(s_received, 0, 1);
#ifdef CONFIG_BT_FW_CLASSIC_DEMO
PBL_THREAD_STACK_DEFINE(s_stack, 8192);
#else
PBL_THREAD_STACK_DEFINE(s_stack, 4096);
#endif

extern uint8_t lcpu_power_on(void);
extern void lcpu_custom_nvds_config(void);

void hci_bridge_transport_wake(void) {
  pbl_sem_give(&s_received);
}

static int32_t prv_received(ipc_queue_handle_t port, size_t length) {
  hci_bridge_transport_wake();
  return 0;
}

static void prv_write_raw(const uint8_t *data, size_t length) {
  PBL_ASSERTN(s_port != IPC_QUEUE_INVALID_HANDLE);
  while (length) {
    size_t written = ipc_queue_write(s_port, data, length, 10);
    // A truncated H4 packet cannot be recovered by forwarding the next packet.
    PBL_ASSERTN(written > 0 && written <= length);
    data += written;
    length -= written;
  }
}

#ifdef CONFIG_BT_HCI_AUDIO_ADAPTER
typedef struct {
  uint16_t length;
  uint8_t data[1030];
} H4Packet;

static H4Packet s_host_packet;
static uint8_t s_controller_buffer[1030];
static H4Stream s_host_stream, s_controller_stream;
static PBL_MSGQ_DEFINE(s_host_queue, sizeof(H4Packet), 16);

static void prv_enqueue(uint8_t *packet, size_t length, void *context) {
  PBL_ASSERTN(packet[0] == 1 || packet[0] == 2 || packet[0] == 3);
  s_host_packet.length = length;
  // A control/ACL packet must never be silently dropped from an H4 stream.
  PBL_ASSERTN(pbl_msgq_put(&s_host_queue, &s_host_packet, PBL_NO_WAIT) == 0);
  hci_bridge_transport_wake();
}

static void prv_controller_packet(uint8_t *packet, size_t length, void *context) {
  if (packet[0] == 4) {
    hci_bridge_audio_event(packet, length);
  }
  hci_bridge_transport_receive(packet, length);
}

static void prv_send_host_packet(const H4Packet *packet) {
#ifdef CONFIG_BT_HCI_LOCAL_AUDIO
  hci_local_audio_command(packet->data, packet->length);
#endif
  if (packet->data[0] == 3) {
#ifndef CONFIG_BT_HCI_LOCAL_AUDIO
    hci_bridge_audio_send(packet->data, packet->length);
#endif
    // Local mode owns the SCO uplink; the desktop host must use --watch-audio.
    return;
  }
  if (packet->data[0] == 1) {
    uint8_t response[8];
    size_t length = hci_bridge_audio_command(packet->data, packet->length, response);
    if (length) {
      hci_bridge_transport_receive(response, length);
      return;
    }
  }
  prv_write_raw(packet->data, packet->length);
}
#endif

void hci_bridge_transport_write(const uint8_t *data, size_t length) {
#ifdef CONFIG_BT_HCI_AUDIO_ADAPTER
  PBL_ASSERTN(h4_stream_feed(&s_host_stream, data, length, prv_enqueue, NULL));
#else
  prv_write_raw(data, length);
#endif
}

static void prv_receive_task(void *context) {
  uint8_t buffer[256];
#ifdef CONFIG_BT_FW_CLASSIC_DEMO
  classic_demo_service_init();
#endif
  while (true) {
#ifdef CONFIG_BT_FW_CLASSIC_DEMO
    classic_demo_service_poll(pbl_ticks_to_ms(pbl_uptime_ticks()));
#endif
#ifdef CONFIG_BT_HCI_AUDIO_ADAPTER
    pbl_sem_take(&s_received, PBL_MSEC(10));
    H4Packet packet;
    for (unsigned i = 0; i < 16 && pbl_msgq_get(&s_host_queue, &packet, PBL_NO_WAIT) == 0; ++i) {
      prv_send_host_packet(&packet);
    }
#else
    pbl_sem_take(&s_received, PBL_FOREVER);
#endif
    size_t length;
    while ((length = ipc_queue_read(s_port, buffer, sizeof(buffer))) != 0) {
#ifdef CONFIG_BT_HCI_AUDIO_ADAPTER
      PBL_ASSERTN(
          h4_stream_feed(&s_controller_stream, buffer, length, prv_controller_packet, NULL));
#else
      hci_bridge_transport_receive(buffer, length);
#endif
    }
#ifdef CONFIG_BT_HCI_AUDIO_ADAPTER
    length = hci_bridge_audio_completed(buffer);
    if (length) {
      hci_bridge_transport_receive(buffer, length);
    }
#ifdef CONFIG_BT_HCI_LOCAL_AUDIO
    for (unsigned i = 0; i < 7; ++i) {
      length = hci_local_audio_transmit(buffer);
      if (!length) {
        break;
      }
      hci_bridge_audio_send(buffer, length);
    }
#endif
    for (unsigned i = 0; i < 4; ++i) {
      length = hci_bridge_audio_receive(buffer);
      if (!length) {
        break;
      }
      hci_bridge_transport_receive(buffer, length);
    }
#endif
  }
}

void hci_bridge_transport_init(void) {
#ifdef CONFIG_BT_HCI_AUDIO_ADAPTER
  h4_stream_init(&s_host_stream, s_host_packet.data, sizeof(s_host_packet.data));
  h4_stream_init(&s_controller_stream, s_controller_buffer, sizeof(s_controller_buffer));
#endif
  ipc_queue_cfg_t config = {
    .qid = 0,
    .tx_buf_size = HCPU2LCPU_MB_CH1_BUF_SIZE,
    .tx_buf_addr = HCPU2LCPU_MB_CH1_BUF_START_ADDR,
    .tx_buf_addr_alias = HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH1_BUF_START_ADDR),
    .rx_ind = prv_received,
  };
  if (__HAL_SYSCFG_GET_REVID() < HAL_CHIP_REV_ID_A4) {
    config.rx_buf_addr = LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_START_ADDR);
  } else {
    config.rx_buf_addr = LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_REV_B_START_ADDR);
  }
  s_port = ipc_queue_init(&config);
  PBL_ASSERTN(s_port != IPC_QUEUE_INVALID_HANDLE);
  PBL_ASSERTN(ipc_queue_open(s_port) == 0);
  NVIC_SetPriority(LCPU2HCPU_IRQn, 5);
  NVIC_EnableIRQ(LCPU2HCPU_IRQn);

  lcpu_custom_nvds_config();
  lcpu_power_on();
#ifdef CONFIG_BT_HCI_AUDIO_PROBE
  hci_bridge_audio_probe_init();
#endif

  struct pbl_thread_attr attr = {
    .name = "ClassicHCI",
    .entry = prv_receive_task,
    .prio = PBL_PRIO_IDLE + 3,
    .privileged = true,
    .stack = s_stack,
    .stack_size = sizeof(s_stack),
  };
  PBL_ASSERTN(pebble_task_create(PebbleTask_BTHCI, &attr));
}

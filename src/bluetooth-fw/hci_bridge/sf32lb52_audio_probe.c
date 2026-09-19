/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "sf32lb52_audio_probe.h"
#include "sifli_sco.h"
#include "transport.h"
#ifdef CONFIG_BT_HCI_LOCAL_AUDIO
#include "local_audio.h"
#include <pbl/drivers/speaker/sf32lb52/audio_definitions.h>
#endif

#include <bf0_hal.h>
#include <bf0_hal_lcpu_config.h>
#include <console/prompt.h>
#include <ipc_queue.h>
#include <pbl/kernel/mutex.h>
#include <system/passert.h>

_Static_assert(LCPU_HCPU_AUDIO_MEM_SIZE == SIFLI_SCO_MEMORY_SIZE, "Audio memory ABI");

static SifliSco s_sco;
static PBL_MUTEX_DEFINE(s_lock);
static volatile uint32_t s_notifications;
static uint32_t s_soft_cvsd;
static HAL_StatusTypeDef s_config_status;

static int32_t prv_notify(ipc_queue_handle_t port, size_t length) {
  ++s_notifications;
#ifdef CONFIG_BT_HCI_AUDIO_ADAPTER
  hci_bridge_transport_wake();
#endif
  return 0;
}

static void prv_lock(void) {
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  // Shared audio RAM is inaccessible while the LCPU power domain is asleep.
  HAL_HPAON_WakeCore(CORE_ID_LCPU);
  __DMB();
}

static void prv_unlock(void) {
  __DMB();
  HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
  pbl_mutex_unlock(&s_lock);
}

void hci_bridge_audio_probe_init(void) {
  uint16_t length = sizeof(s_soft_cvsd);
#ifdef CONFIG_BT_HCI_SOFTWARE_CVSD
  uint32_t mode = 0x5a5aa5a5;
  PBL_ASSERTN(HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_SOFT_CVSD, (uint8_t *)&mode, sizeof(mode)) ==
              HAL_OK);
#endif
  s_config_status =
      HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_SOFT_CVSD, (uint8_t *)&s_soft_cvsd, &length);
  prv_lock();
  sifli_sco_init(&s_sco, (void *)LCPU_AUDIO_MEM_START_ADDR, LCPU_AUDIO_MEM_START_ADDR,
                 s_config_status == HAL_OK && (s_soft_cvsd == 0 || s_soft_cvsd == 0x5a5aa5a5));
  s_sco.software_cvsd = s_config_status == HAL_OK && s_soft_cvsd == 0x5a5aa5a5;
  prv_unlock();
  ipc_queue_cfg_t config = {.qid = 6, .rx_ind = prv_notify};
  ipc_queue_handle_t port = ipc_queue_init(&config);
  PBL_ASSERTN(port != IPC_QUEUE_INVALID_HANDLE);
  PBL_ASSERTN(ipc_queue_open(port) == 0);
}

size_t hci_bridge_audio_command(const uint8_t *packet, size_t length, uint8_t response[8]) {
  prv_lock();
  size_t result = sifli_sco_command(&s_sco, packet, length, response);
  prv_unlock();
  return result;
}

void hci_bridge_audio_event(uint8_t *packet, size_t length) {
  prv_lock();
  sifli_sco_event(&s_sco, packet, length);
  prv_unlock();
}

size_t hci_bridge_audio_receive(uint8_t packet[124]) {
  prv_lock();
  size_t length = sifli_sco_receive(&s_sco, packet);
  prv_unlock();
  return length;
}

void hci_bridge_audio_send(const uint8_t *packet, size_t length) {
  prv_lock();
  sifli_sco_send(&s_sco, packet, length);
  prv_unlock();
}

size_t hci_bridge_audio_completed(uint8_t packet[8]) {
  prv_lock();
  size_t length = sifli_sco_completed(&s_sco, packet);
  prv_unlock();
  return length;
}

bool hci_bridge_audio_active(void) {
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  bool active = s_sco.active;
  pbl_mutex_unlock(&s_lock);
  return active;
}

void hci_bridge_audio_pump(bool (*receive)(const uint8_t *, size_t),
                           size_t (*transmit)(uint8_t *)) {
  pbl_mutex_lock(&s_lock, PBL_FOREVER);
  if (!s_sco.active) {
    pbl_mutex_unlock(&s_lock);
    return;
  }
  // Waking the LCPU incurs a fixed delay; share it across the bounded batch.
  HAL_HPAON_WakeCore(CORE_ID_LCPU);
  __DMB();
  uint8_t packet[124];
  size_t length = sifli_sco_completed(&s_sco, packet);
  if (length) {
    receive(packet, length);
  }
  for (unsigned i = 0; i < SIFLI_SCO_CREDITS; ++i) {
    length = transmit(packet);
    if (!length) {
      break;
    }
    sifli_sco_send(&s_sco, packet, length);
  }
  for (unsigned i = 0; i < 4; ++i) {
    length = sifli_sco_receive(&s_sco, packet);
    if (!length) {
      break;
    }
    receive(packet, length);
  }
  prv_unlock();
}

#ifdef CONFIG_PROMPT
static void prv_report_ring(const char *name, const SifliAudioRing *ring) {
  char buffer[128];
  prompt_send_response_fmt(
      buffer, sizeof(buffer), "%s size=%d read=%u/%u write=%u/%u buffers=%08lx/%08lx", name,
      ring->capacity, (unsigned)(ring->read_cursor & 0xffff), (unsigned)(ring->read_cursor >> 16),
      (unsigned)(ring->write_cursor & 0xffff), (unsigned)(ring->write_cursor >> 16),
      (unsigned long)ring->read_buffer, (unsigned long)ring->write_buffer);
}

void command_bt_audio_probe(void) {
  char buffer[160];
  prv_lock();
  SifliAudioRing downlink = *s_sco.downlink;
  SifliAudioRing uplink = *s_sco.uplink;
  SifliAudioLink link = *s_sco.link;
  const struct {
    bool active, pcm_allowed;
    uint8_t last_length, last_status, tx_count;
    uint32_t malformed, rx_packets, rx_bytes, rx_bad, tx_packets, tx_consumed, tx_dropped;
  } stats = {
    s_sco.active,   s_sco.pcm_allowed, s_sco.last_length, s_sco.last_status,
    s_sco.tx_count, s_sco.malformed,   s_sco.rx_packets,  s_sco.rx_bytes,
    s_sco.rx_bad,   s_sco.tx_packets,  s_sco.tx_consumed, s_sco.tx_dropped,
  };
  uint32_t notifications = s_notifications;
  prv_unlock();
  prompt_send_response_fmt(
      buffer, sizeof(buffer), "audio mailbox notifications=%lu soft_cvsd=%08lx config_status=%u",
      (unsigned long)notifications, (unsigned long)s_soft_cvsd, s_config_status);
  prv_report_ring("downlink", &downlink);
  prv_report_ring("uplink", &uplink);
  prompt_send_response_fmt(
      buffer, sizeof(buffer),
      "audio link status=%u handle=%u type=%u interval=%u rx=%u tx=%u air=%u retx=%u", link.status,
      link.handle, link.link_type, link.interval, link.rx_length, link.tx_length, link.air_mode,
      link.retransmission_window);
  prompt_send_response_fmt(
      buffer, sizeof(buffer),
      "adapter active=%u pcm_allowed=%u header_length=%u header_status=%u malformed=%lu",
      stats.active, stats.pcm_allowed, stats.last_length, stats.last_status,
      (unsigned long)stats.malformed);
  prompt_send_response_fmt(
      buffer, sizeof(buffer),
      "adapter rx=%lu bytes=%lu bad=%lu tx=%lu consumed=%lu dropped=%lu queued=%u",
      (unsigned long)stats.rx_packets, (unsigned long)stats.rx_bytes, (unsigned long)stats.rx_bad,
      (unsigned long)stats.tx_packets, (unsigned long)stats.tx_consumed,
      (unsigned long)stats.tx_dropped, stats.tx_count);
#ifdef CONFIG_BT_HCI_LOCAL_AUDIO
  hci_local_audio_report();
  prompt_send_response_fmt(buffer, sizeof(buffer),
                           "speaker DMA refills=%lu underrun_bytes=%lu write_drops=%lu",
                           (unsigned long)AUDIO->state->diagnostic_refills,
                           (unsigned long)AUDIO->state->diagnostic_underrun_bytes,
                           (unsigned long)AUDIO->state->diagnostic_write_drops);
  prompt_send_response_fmt(buffer, sizeof(buffer), "speaker DMA peak=%lu signal_samples=%lu",
                           (unsigned long)AUDIO->state->diagnostic_peak,
                           (unsigned long)AUDIO->state->diagnostic_signal_samples);
#endif
}

#endif

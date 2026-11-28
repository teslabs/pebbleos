/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "aec.h"

#include <errno.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <pbl/console/console.h>
#include <pbl/drivers/audio.h>
#include <pbl/drivers/charger.h>
#include <pbl/fs/fs.h>
#include <pbl/fs/pfs.h>
#include <pbl/kernel/sem.h>
#include <pbl/kernel/thread.h>
#include <pbl/kernel/types.h>
#include <pbl/kernel/work.h>
#include <pbl/shell/shell.h>
#include <pbl/util/math.h>
#include <pbl/util/size.h>
#include <pbl/util/trig.h>

#include <host/ble_br_gap.h>
#include <host/ble_br_hfp_hf.h>
#include <host/ble_br_sco.h>
#include <host/ble_hs.h>
#include <host/util/util.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_pebble.h>
#include <sbc.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>

#define NAME "PebbleOS HF"

/* Advertising flags of a device that takes LE and Classic connections at once */
#define ADV_F_DUAL_MODE_CONTROLLER 0x08U
#define ADV_F_DUAL_MODE_HOST       0x10U

/* Audio, hands-free */
#define CLASS_OF_DEVICE 0x200408U

/* Call audio: 16-bit samples at 8 kHz, both ways */
/* Narrowband speech; wideband speech doubles the rate, and the frames that go with it */
#define RATE      8000U
#define RATE_WIDE (2U * RATE)
/* A wideband frame on air: a header, the coded samples and a spare byte */
#define MSBC_HEADER        0x01U
#define MSBC_HEADER_LEN    2U
#define MSBC_SYNC          0xadU
#define MSBC_FRAME_LEN     60U
#define PERIOD_FRAMES      80U
#define PERIODS            2U
#define FIFO_FRAMES        2048U
#define RECONNECT_ATTEMPTS 5U
/* An idle link is put in sniff mode after a while, to meet twice a second: units of 0.625 ms */
#define SNIFF_IDLE_MS  5000U
#define SNIFF_INTERVAL 798U
#define SNIFF_ATTEMPT  4U
#define SNIFF_TIMEOUT  1U
/* A phone that picks a shorter interval than this is asked again, a few times */
#define SNIFF_INTERVAL_MIN 100U
#define SNIFF_RETRIES      3U
#define RECONNECT_DELAY_MS 5000U
/* Microphone gain during a call: the speaker, right next to it, must not saturate it */
#define MIC_GAIN_MDB 18000

/* Played before the first received samples, to absorb the jitter of their arrival */
#define PREFILL_FRAMES 240U

/* What a step of the volume of the phone changes the speaker gain by, below a top that keeps the
 * speaker out of distortion
 */
#define VOLUME_STEP_MDB   2000
#define VOLUME_TOP_OFFSET 6000

/* A tone sent in place of the microphone, and a stretch of audio kept, to check the call audio */
#define TONE_AMPLITUDE 4000
#define DUMP_FRAMES    4000U
#define DUMP_LINE      16U

enum dump_source {
  DUMP_NONE,
  DUMP_RECEIVED,
  DUMP_MIC,
};

/* Samples between the HCI thread and the period callback of a stream, one writer and one reader */
struct fifo {
  int16_t samples[FIFO_FRAMES];
  atomic_uint head;
  atomic_uint tail;
};

/* Where the bonds are kept */
static struct pbl_fs_mount s_mount = {.mnt_point = "/", .ops = &pbl_fs_pfs_ops};

static PBL_SEM_DEFINE(s_synced, 0, 1);
static uint8_t s_own_addr_type;

static uint16_t s_acl_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_hfp_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_sco_handle = BLE_HS_CONN_HANDLE_NONE;
static uint8_t s_peer[BLE_DEV_ADDR_LEN];
static bool s_peer_known;
static bool s_connect_hfp;
/* Attempts left to get back to the bonded phone after a reset */
static uint8_t s_reconnects;
static bool s_sniffing;
static uint8_t s_sniff_retries;
/* Whether phones can also find and pair with the device over Bluetooth Classic */
static bool s_classic_pairing;

static struct fifo s_playback;
static struct fifo s_capture;
/* What the speaker plays, period by period as the microphone captures: the echo to take out */
static struct fifo s_reference;
static struct aec s_aec;
static atomic_bool s_aec_enabled = true;
static void *s_playback_buf;
static void *s_capture_buf;
static bool s_audio_running;

static atomic_uint s_tone_hz;
static uint32_t s_tone_phase;
static atomic_uint s_beep_hz;
static uint32_t s_beep_phase;
static int32_t s_speaker_gain_max_mdb;
static int16_t s_dump[DUMP_FRAMES];
static atomic_uint s_dump_len;
static atomic_uint s_dump_source;

static atomic_uint s_rx_packets;
static atomic_uint s_rx_bad;
static atomic_uint s_tx_packets;
static atomic_uint s_underruns;
static atomic_uint s_overruns;
static atomic_uint s_tx_gaps;
static bool s_capture_primed;

/* Whether the call is in wideband speech, and the state of its codec */
static bool s_wideband;
static sbc_t s_sbc_rx;
static sbc_t s_sbc_tx;
static uint8_t s_msbc[MSBC_FRAME_LEN];
static size_t s_msbc_len;
static size_t s_msbc_lost;
static uint8_t s_msbc_count;

static uint32_t prv_rate(void) {
  return s_wideband ? RATE_WIDE : RATE;
}

/* A number of frames at the narrowband rate, at the rate of the call */
static size_t prv_scaled(size_t frames) {
  return s_wideband ? 2U * frames : frames;
}

static void prv_fifo_reset(struct fifo *fifo) {
  atomic_store(&fifo->head, 0U);
  atomic_store(&fifo->tail, 0U);
}

static size_t prv_fifo_len(struct fifo *fifo) {
  return atomic_load(&fifo->head) - atomic_load(&fifo->tail);
}

static size_t prv_fifo_put(struct fifo *fifo, const int16_t *samples, size_t count) {
  unsigned int head = atomic_load(&fifo->head);
  size_t space = FIFO_FRAMES - (head - atomic_load(&fifo->tail));

  count = PBL_MIN(count, space);
  for (size_t i = 0U; i < count; i++) {
    fifo->samples[(head + i) % FIFO_FRAMES] = samples != nullptr ? samples[i] : 0;
  }
  atomic_store(&fifo->head, head + count);

  return count;
}

static size_t prv_fifo_get(struct fifo *fifo, int16_t *samples, size_t count) {
  unsigned int tail = atomic_load(&fifo->tail);
  size_t used = atomic_load(&fifo->head) - tail;

  count = PBL_MIN(count, used);
  for (size_t i = 0U; i < count; i++) {
    samples[i] = fifo->samples[(tail + i) % FIFO_FRAMES];
  }
  atomic_store(&fifo->tail, tail + count);

  return count;
}

/* Run in an interrupt, each time the hardware is done with a period. */
static void prv_playback_done(const struct pbl_audio *audio, enum pbl_audio_dir dir, void *period,
                              const struct pbl_audio_pos *pos, void *arg) {
  int16_t *samples = period;
  const size_t frames = prv_scaled(PERIOD_FRAMES);
  size_t got = prv_fifo_get(&s_playback, samples, frames);

  if (got < frames) {
    memset(&samples[got], 0, (frames - got) * sizeof(*samples));
    atomic_fetch_add(&s_underruns, 1U);
  }
  (void)prv_fifo_put(&s_reference, samples, frames);

  (void)pbl_audio_period_ready(audio, dir, period);
}

static void prv_capture_done(const struct pbl_audio *audio, enum pbl_audio_dir dir, void *period,
                             const struct pbl_audio_pos *pos, void *arg) {
  if (prv_fifo_put(&s_capture, period, prv_scaled(PERIOD_FRAMES)) < prv_scaled(PERIOD_FRAMES)) {
    atomic_fetch_add(&s_overruns, 1U);
  }

  (void)pbl_audio_period_ready(audio, dir, period);
}

static const struct pbl_audio_stream s_playback_stream = {
  .rate = RATE,
  .channels = 1U,
  .period_frames = PERIOD_FRAMES,
  .periods = PERIODS,
  .period_done = prv_playback_done,
};

static const struct pbl_audio_stream s_capture_stream = {
  .rate = RATE,
  .channels = 1U,
  .period_frames = PERIOD_FRAMES,
  .periods = PERIODS,
  .period_done = prv_capture_done,
};

/* Both rates are needed: the phone picks the one of each call */
#define RATES (pbl_audio_rate_bit(RATE) | pbl_audio_rate_bit(RATE_WIDE))

static int prv_audio_init(void) {
  struct pbl_audio_caps caps;
  int ret;

  ret = pbl_audio_get_caps(SPEAKER, PBL_AUDIO_PLAYBACK, &caps);
  if (ret == 0 && (caps.rates & RATES) != RATES) {
    ret = -ENOTSUP;
  }
  if (ret == 0) {
    s_speaker_gain_max_mdb = caps.gain_max_mdb - VOLUME_TOP_OFFSET;
    ret = pbl_audio_set_gain(SPEAKER, PBL_AUDIO_PLAYBACK, s_speaker_gain_max_mdb);
  }
  if (ret == 0) {
    ret = pbl_audio_set_mute(SPEAKER, PBL_AUDIO_PLAYBACK, false);
  }
  if (ret == 0) {
    ret = pbl_audio_get_caps(MIC, PBL_AUDIO_CAPTURE, &caps);
  }
  if (ret == 0 && (caps.rates & RATES) != RATES) {
    ret = -ENOTSUP;
  }
  if (ret == 0) {
    ret = pbl_audio_set_gain(MIC, PBL_AUDIO_CAPTURE, MIC_GAIN_MDB);
  }
  if (ret != 0) {
    return ret;
  }

  /* The rings must be reachable by DMA: allocate them with the class. */
  s_playback_buf = pbl_audio_buf_alloc(2U * pbl_audio_ring_bytes(&s_playback_stream));
  s_capture_buf = pbl_audio_buf_alloc(2U * pbl_audio_ring_bytes(&s_capture_stream));
  if (s_playback_buf == nullptr || s_capture_buf == nullptr) {
    return -ENOMEM;
  }

  return 0;
}

static void prv_audio_start(void) {
  struct pbl_audio_stream stream;
  int ret;

  if (s_audio_running) {
    return;
  }

  prv_fifo_reset(&s_playback);
  prv_fifo_reset(&s_capture);
  prv_fifo_reset(&s_reference);
  aec_reset(&s_aec, prv_rate());
  sbc_reset(&s_sbc_rx);
  sbc_reset(&s_sbc_tx);
  s_msbc_len = 0U;
  s_msbc_lost = 0U;
  s_capture_primed = false;
  (void)prv_fifo_put(&s_playback, nullptr, prv_scaled(PREFILL_FRAMES));

#ifdef CONFIG_CHARGER_HIGH_LOAD
  /* The speaker draws more than the battery gives by default. */
  (void)pbl_charger_high_load_request(CHARGER);
#endif

  stream = s_playback_stream;
  stream.rate = prv_rate();
  stream.period_frames = prv_scaled(PERIOD_FRAMES);
  stream.buf = s_playback_buf;
  memset(stream.buf, 0, pbl_audio_ring_bytes(&stream));
  ret = pbl_audio_start(SPEAKER, PBL_AUDIO_PLAYBACK, &stream);
  if (ret == 0) {
    stream = s_capture_stream;
    stream.rate = prv_rate();
    stream.period_frames = prv_scaled(PERIOD_FRAMES);
    stream.buf = s_capture_buf;
    ret = pbl_audio_start(MIC, PBL_AUDIO_CAPTURE, &stream);
    if (ret != 0) {
      (void)pbl_audio_stop(SPEAKER, PBL_AUDIO_PLAYBACK);
    }
  }
  if (ret != 0) {
    printf("hfp_hf: starting the audio failed (%d)\n", ret);
    return;
  }

  s_audio_running = true;
}

static void prv_audio_stop(void) {
  if (!s_audio_running) {
    return;
  }

  (void)pbl_audio_stop(MIC, PBL_AUDIO_CAPTURE);
  (void)pbl_audio_stop(SPEAKER, PBL_AUDIO_PLAYBACK);
#ifdef CONFIG_CHARGER_HIGH_LOAD
  (void)pbl_charger_high_load_release(CHARGER);
#endif

  s_audio_running = false;
}

static void prv_dump(enum dump_source source, const int16_t *samples, size_t count) {
  unsigned int len;

  if (atomic_load(&s_dump_source) != source) {
    return;
  }

  len = atomic_load(&s_dump_len);
  count = PBL_MIN(count, DUMP_FRAMES - len);
  memcpy(&s_dump[len], samples, count * sizeof(*samples));
  atomic_store(&s_dump_len, len + count);
  if (len + count == DUMP_FRAMES) {
    atomic_store(&s_dump_source, DUMP_NONE);
  }
}

static void prv_tone(int16_t *samples, size_t count, unsigned int hz, uint32_t *phase) {
  /* Phase in 1/65536 turns, scaled by 2^16 to keep the fraction. */
  const uint32_t step = (uint32_t)(((uint64_t)hz * TRIG_MAX_ANGLE << 16) / prv_rate());

  for (size_t i = 0U; i < count; i++) {
    *phase += step;
    samples[i] = (int16_t)(sin_lookup((int32_t)(*phase >> 16)) * TONE_AMPLITUDE / TRIG_MAX_RATIO);
  }
}

/* Plays the samples of the phone, and replaces them by as many from the microphone. */
static void prv_exchange(int16_t *samples, size_t count) {
  int16_t reference[SBC_MSBC_SAMPLES];
  unsigned int tone_hz = atomic_load(&s_tone_hz);
  unsigned int beep_hz = atomic_load(&s_beep_hz);
  size_t got;

  prv_dump(DUMP_RECEIVED, samples, count);
  if (beep_hz != 0U) {
    prv_tone(samples, count, beep_hz, &s_beep_phase);
  }
  (void)prv_fifo_put(&s_playback, samples, count);

  /* The microphone delivers a period at a time: send silence until some is ahead */
  if (!s_capture_primed && prv_fifo_len(&s_capture) >= prv_scaled(PREFILL_FRAMES)) {
    s_capture_primed = true;
  }
  got = s_capture_primed ? prv_fifo_get(&s_capture, samples, count) : 0U;
  /* The reference goes at the pace of the microphone, so that the two stay aligned */
  if (prv_fifo_get(&s_reference, reference, got) == got && atomic_load(&s_aec_enabled)) {
    aec_process(&s_aec, reference, samples, got);
  }
  if (got < count) {
    memset(&samples[got], 0, (count - got) * sizeof(int16_t));
    if (s_capture_primed) {
      s_capture_primed = false;
      atomic_fetch_add(&s_tx_gaps, 1U);
    }
  }
  prv_dump(DUMP_MIC, samples, count);
  if (tone_hz != 0U) {
    prv_tone(samples, count, tone_hz, &s_tone_phase);
  }
}

/* A wideband frame: what the phone sent is decoded and played, the microphone coded and sent. */
static void prv_msbc_frame(uint16_t sco_handle, const uint8_t *coded) {
  /* The second byte of the header counts the frames, two bits written twice over */
  static const uint8_t counts[] = {0x08U, 0x38U, 0xc8U, 0xf8U};
  int16_t samples[SBC_MSBC_SAMPLES];
  uint8_t frame[MSBC_FRAME_LEN];
  struct sbc_frame desc;

  if (coded == nullptr ||
      sbc_decode(&s_sbc_rx, coded, SBC_MSBC_SIZE, &desc, samples, 1, nullptr, 0) != 0) {
    /* Silence in place of what was lost keeps the time */
    atomic_fetch_add(&s_rx_bad, 1U);
    memset(samples, 0, sizeof(samples));
  }

  prv_exchange(samples, SBC_MSBC_SAMPLES);

  memset(frame, 0, sizeof(frame));
  frame[0] = MSBC_HEADER;
  frame[1] = counts[s_msbc_count++ % PBL_ARRAY_SIZE(counts)];
  desc = (struct sbc_frame){.msbc = true};
  if (sbc_encode(&s_sbc_tx, samples, 1, nullptr, 0, &desc, &frame[MSBC_HEADER_LEN],
                 SBC_MSBC_SIZE) == 0 &&
      ble_br_sco_tx(sco_handle, frame, sizeof(frame)) == 0) {
    atomic_fetch_add(&s_tx_packets, 1U);
  }
}

/*
 * Wideband speech comes as a stream of coded frames, each behind a two byte header, in packets
 * that need not hold a whole one.
 */
static void prv_msbc_rx(uint16_t sco_handle, const uint8_t *data, uint16_t len, uint8_t status) {
  if (status != BLE_BR_SCO_STATUS_GOOD) {
    /* Whatever frame this was part of is lost: as many frames of silence as the time it took */
    s_msbc_len = 0U;
    s_msbc_lost += len;
    while (s_msbc_lost >= MSBC_FRAME_LEN) {
      s_msbc_lost -= MSBC_FRAME_LEN;
      prv_msbc_frame(sco_handle, nullptr);
    }
    return;
  }

  for (uint16_t i = 0U; i < len; i++) {
    s_msbc[s_msbc_len++] = data[i];

    /* A frame starts with its header, then the sync word of the codec */
    if ((s_msbc_len == 1U && s_msbc[0] != MSBC_HEADER) ||
        (s_msbc_len == 2U && (s_msbc[1] & 0x0fU) != 0x08U) ||
        (s_msbc_len == 3U && s_msbc[2] != MSBC_SYNC)) {
      s_msbc_len = 0U;
      continue;
    }

    if (s_msbc_len == MSBC_FRAME_LEN) {
      prv_msbc_frame(sco_handle, &s_msbc[MSBC_HEADER_LEN]);
      s_msbc_len = 0U;
    }
  }
}

/* Runs on the HCI thread, once per packet: what came is played, what was captured goes back. */
static void prv_sco_rx(uint16_t sco_handle, const uint8_t *data, uint16_t len, uint8_t status,
                       void *arg) {
  int16_t samples[BLE_BR_SCO_MAX_DATA_LEN / sizeof(int16_t)];
  size_t count = len / sizeof(int16_t);

  atomic_fetch_add(&s_rx_packets, 1U);

  if (s_wideband) {
    prv_msbc_rx(sco_handle, data, len, status);
    return;
  }

  if (status == BLE_BR_SCO_STATUS_GOOD) {
    memcpy(samples, data, count * sizeof(int16_t));
  } else {
    /* Silence in place of what was lost keeps the time */
    atomic_fetch_add(&s_rx_bad, 1U);
    memset(samples, 0, count * sizeof(int16_t));
  }

  /* A packet may hold more than the echo canceller takes at once */
  for (size_t done = 0U; done < count; done += SBC_MSBC_SAMPLES) {
    prv_exchange(&samples[done], PBL_MIN(count - done, (size_t)SBC_MSBC_SAMPLES));
  }

  if (ble_br_sco_tx(sco_handle, (const uint8_t *)samples, count * sizeof(int16_t)) == 0) {
    atomic_fetch_add(&s_tx_packets, 1U);
  }
}

static void prv_print_addr(const char *what, const uint8_t *addr) {
  printf("hfp_hf: %s %02x:%02x:%02x:%02x:%02x:%02x\n", what, addr[5], addr[4], addr[3], addr[2],
         addr[1], addr[0]);
}

/* The phone may not answer at once after a reset, e.g. while it still holds the old connection */
static void prv_reconnect(struct pbl_work *work) {
  if (s_acl_handle != BLE_HS_CONN_HANDLE_NONE) {
    return;
  }

  s_connect_hfp = true;
  if (ble_br_gap_connect(s_peer) != 0) {
    s_connect_hfp = false;
  }
}

static PBL_DELAYED_WORK_DEFINE(s_reconnect, prv_reconnect);

/* The phone may put the idle link in sniff mode itself; if it has not by now, ask for it */
static void prv_sniff(struct pbl_work *work) {
  if (s_acl_handle == BLE_HS_CONN_HANDLE_NONE || s_sniffing ||
      s_sco_handle != BLE_HS_CONN_HANDLE_NONE) {
    return;
  }

  (void)ble_br_gap_sniff(s_acl_handle, SNIFF_INTERVAL, SNIFF_INTERVAL, SNIFF_ATTEMPT,
                         SNIFF_TIMEOUT);
}

static PBL_DELAYED_WORK_DEFINE(s_sniff, prv_sniff);

static void prv_sniff_later(void) {
  (void)pbl_delayed_work_reschedule(&pbl_sys_workq, &s_sniff, PBL_MSEC(SNIFF_IDLE_MS));
}

/* Runs on the host thread. */
static void prv_br_gap_event(const struct ble_br_gap_event *event, void *arg) {
  switch (event->type) {
    case BLE_BR_GAP_EVENT_CONNECT:
      if (event->connect.status != 0) {
        printf("hfp_hf: classic connection failed (%d)\n", event->connect.status);
        s_connect_hfp = false;
        if (s_reconnects > 0U) {
          s_reconnects--;
          (void)pbl_delayed_work_reschedule(&pbl_sys_workq, &s_reconnect,
                                            PBL_MSEC(RECONNECT_DELAY_MS));
        }
        break;
      }
      s_reconnects = 0U;
      s_sniffing = false;
      s_sniff_retries = SNIFF_RETRIES;
      prv_sniff_later();
      s_acl_handle = event->connect.conn_handle;
      memcpy(s_peer, event->connect.addr, sizeof(s_peer));
      s_peer_known = true;
      prv_print_addr("classic connected to", event->connect.addr);
      if (s_connect_hfp) {
        int rc = ble_br_hfp_hf_connect(s_acl_handle);

        s_connect_hfp = false;
        if (rc != 0) {
          printf("hfp_hf: connecting hands-free failed (%d)\n", rc);
        }
      }
      break;
    case BLE_BR_GAP_EVENT_DISCONNECT:
      printf("hfp_hf: classic disconnected, reason 0x%03x\n", event->disconnect.reason);
      s_acl_handle = BLE_HS_CONN_HANDLE_NONE;
      break;
    case BLE_BR_GAP_EVENT_MODE_CHANGE:
      s_sniffing = event->mode_change.sniff;
      if (s_sniffing) {
        printf("hfp_hf: classic link sniffs every %u ms\n",
               (unsigned int)event->mode_change.interval * 5U / 8U);
        if (event->mode_change.interval < SNIFF_INTERVAL_MIN && s_sniff_retries > 0U) {
          s_sniff_retries--;
          (void)ble_br_gap_exit_sniff(s_acl_handle);
        }
      } else {
        printf("hfp_hf: classic link active\n");
        prv_sniff_later();
      }
      break;
    case BLE_BR_GAP_EVENT_ENC_CHANGE:
      printf("hfp_hf: classic encryption %s (%d)\n", event->enc_change.encrypted ? "on" : "off",
             event->enc_change.status);
      break;
    case BLE_BR_GAP_EVENT_USER_CONFIRM:
      printf("hfp_hf: confirming passkey %06u\n", (unsigned int)event->user_confirm.numeric_value);
      (void)ble_br_gap_user_confirm(event->user_confirm.addr, true);
      break;
    case BLE_BR_GAP_EVENT_PAIRING_COMPLETE:
      printf("hfp_hf: classic pairing %s (%d)\n",
             event->pairing_complete.status == 0 ? "done" : "failed",
             event->pairing_complete.status);
      break;
    case BLE_BR_GAP_EVENT_LINK_KEY:
      /* From a Classic pairing, or derived from the keys of an LE one */
      printf("hfp_hf: link key of type %u stored\n", event->link_key.key_type);
      memcpy(s_peer, event->link_key.addr, sizeof(s_peer));
      s_peer_known = true;
      break;
    default:
      break;
  }
}

static const char *const s_indicators[BLE_BR_HFP_HF_IND_COUNT] = {
  [BLE_BR_HFP_HF_IND_SERVICE] = "service",     [BLE_BR_HFP_HF_IND_CALL] = "call",
  [BLE_BR_HFP_HF_IND_CALLSETUP] = "callsetup", [BLE_BR_HFP_HF_IND_CALLHELD] = "callheld",
  [BLE_BR_HFP_HF_IND_SIGNAL] = "signal",       [BLE_BR_HFP_HF_IND_ROAM] = "roam",
  [BLE_BR_HFP_HF_IND_BATTCHG] = "battchg",
};

/* Runs on the host thread. */
static void prv_hfp_event(const struct ble_br_hfp_hf_event *event, void *arg) {
  switch (event->type) {
    case BLE_BR_HFP_HF_EVENT_CONNECT:
      if (event->connect.status != 0) {
        printf("hfp_hf: hands-free connection failed (%d)\n", event->connect.status);
        break;
      }
      s_hfp_handle = event->conn_handle;
      printf("hfp_hf: hands-free connected\n");
      break;
    case BLE_BR_HFP_HF_EVENT_DISCONNECT:
      s_hfp_handle = BLE_HS_CONN_HANDLE_NONE;
      printf("hfp_hf: hands-free disconnected\n");
      break;
    case BLE_BR_HFP_HF_EVENT_INDICATOR:
      printf("hfp_hf: %s %u\n", s_indicators[event->indicator.id], event->indicator.value);
      break;
    case BLE_BR_HFP_HF_EVENT_RING:
      printf("hfp_hf: ring\n");
      break;
    case BLE_BR_HFP_HF_EVENT_CLIP:
      printf("hfp_hf: call from \"%s\"\n", event->clip.number);
      break;
    case BLE_BR_HFP_HF_EVENT_AUDIO_CONNECT:
      if (event->audio_connect.status != 0) {
        printf("hfp_hf: audio connection failed (%d)\n", event->audio_connect.status);
        break;
      }
      atomic_store(&s_rx_packets, 0U);
      atomic_store(&s_rx_bad, 0U);
      atomic_store(&s_tx_packets, 0U);
      atomic_store(&s_underruns, 0U);
      atomic_store(&s_overruns, 0U);
      atomic_store(&s_tx_gaps, 0U);
      s_sco_handle = event->audio_connect.sco_handle;
      s_wideband = event->audio_connect.codec == BLE_BR_HFP_HF_CODEC_MSBC;
      prv_audio_start();
      printf("hfp_hf: audio connected, %s\n", s_wideband ? "wideband" : "narrowband");
      break;
    case BLE_BR_HFP_HF_EVENT_AUDIO_DISCONNECT:
      s_sco_handle = BLE_HS_CONN_HANDLE_NONE;
      prv_audio_stop();
      prv_sniff_later();
      printf("hfp_hf: audio disconnected, %u packets received (%u bad), %u sent\n",
             atomic_load(&s_rx_packets), atomic_load(&s_rx_bad), atomic_load(&s_tx_packets));
      break;
    case BLE_BR_HFP_HF_EVENT_VOLUME:
      printf("hfp_hf: volume %u\n", event->volume.gain);
      (void)pbl_audio_set_gain(
          SPEAKER, PBL_AUDIO_PLAYBACK,
          s_speaker_gain_max_mdb -
              (BLE_BR_HFP_HF_VOLUME_MAX - event->volume.gain) * VOLUME_STEP_MDB);
      break;
    case BLE_BR_HFP_HF_EVENT_CMD_COMPLETE:
      printf("hfp_hf: command %s\n", event->cmd_complete.status == 0 ? "accepted" : "rejected");
      break;
    default:
      break;
  }
}

static int prv_gap_event(struct ble_gap_event *event, void *arg);

static void prv_advertise(void) {
  const char *name = ble_svc_gap_device_name();
  struct ble_hs_adv_fields fields = {
    .flags = BLE_HS_ADV_F_DISC_GEN,
    .name = (const uint8_t *)name,
    .name_len = strlen(name),
    .name_is_complete = 1,
  };
  const struct ble_gap_adv_params params = {
    .conn_mode = BLE_GAP_CONN_MODE_UND,
    .disc_mode = BLE_GAP_DISC_MODE_GEN,
  };
  int rc;

  /* A phone told so pairs over Classic; otherwise over LE, which gives it both keys */
  if (s_classic_pairing) {
    fields.flags |= ADV_F_DUAL_MODE_CONTROLLER | ADV_F_DUAL_MODE_HOST;
  }

  rc = ble_gap_adv_set_fields(&fields);
  if (rc == 0) {
    rc = ble_gap_adv_start(s_own_addr_type, nullptr, BLE_HS_FOREVER, &params, prv_gap_event,
                           nullptr);
  }
  if (rc != 0) {
    printf("hfp_hf: advertising failed (%d)\n", rc);
    return;
  }

  printf("hfp_hf: advertising as %s\n", name);
}

/* Runs on the host thread. */
static int prv_gap_event(struct ble_gap_event *event, void *arg) {
  struct ble_gap_conn_desc desc;

  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status != 0) {
        printf("hfp_hf: LE connection failed (%d)\n", event->connect.status);
        prv_advertise();
        break;
      }
      printf("hfp_hf: LE connected, handle %u\n", event->connect.conn_handle);
      break;
    case BLE_GAP_EVENT_DISCONNECT:
      printf("hfp_hf: LE disconnected, reason 0x%03x\n", event->disconnect.reason);
      prv_advertise();
      break;
    case BLE_GAP_EVENT_MTU:
      printf("hfp_hf: LE MTU %u\n", event->mtu.value);
      break;
    case BLE_GAP_EVENT_ENC_CHANGE:
      if (event->enc_change.status != 0 ||
          ble_gap_conn_find(event->enc_change.conn_handle, &desc) != 0) {
        printf("hfp_hf: LE encryption failed (%d)\n", event->enc_change.status);
        break;
      }
      printf("hfp_hf: LE encrypted, %s%s\n", desc.sec_state.bonded ? "bonded" : "not bonded",
             desc.sec_state.authenticated ? ", authenticated" : "");
      break;
    case BLE_GAP_EVENT_REPEAT_PAIRING:
      /* A bonded peer that lost its keys: forget ours, and pair again. */
      if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
        (void)ble_store_util_delete_peer(&desc.peer_id_addr);
      }
      return BLE_GAP_REPEAT_PAIRING_RETRY;
    default:
      break;
  }

  return 0;
}

static int prv_cmd_answer(const struct pbl_shell *sh, size_t argc, char **argv) {
  return ble_br_hfp_hf_answer(s_hfp_handle) == 0 ? 0 : -EIO;
}

static int prv_cmd_hangup(const struct pbl_shell *sh, size_t argc, char **argv) {
  return ble_br_hfp_hf_hangup(s_hfp_handle) == 0 ? 0 : -EIO;
}

static int prv_cmd_dial(const struct pbl_shell *sh, size_t argc, char **argv) {
  return ble_br_hfp_hf_dial(s_hfp_handle, argv[1]) == 0 ? 0 : -EIO;
}

static int prv_cmd_volume(const struct pbl_shell *sh, size_t argc, char **argv) {
  unsigned long gain;
  int ret;

  ret = pbl_shell_strtoul(argv[1], &gain);
  if (ret < 0 || gain > BLE_BR_HFP_HF_VOLUME_MAX) {
    pbl_shell_error(sh, "not a gain: %s", argv[1]);
    return -EINVAL;
  }

  return ble_br_hfp_hf_set_volume(s_hfp_handle, (uint8_t)gain) == 0 ? 0 : -EIO;
}

static int prv_cmd_audio(const struct pbl_shell *sh, size_t argc, char **argv) {
  int rc;

  if (strcmp(argv[0], "on") == 0) {
    rc = ble_br_hfp_hf_audio_connect(s_hfp_handle);
  } else {
    rc = ble_br_hfp_hf_audio_disconnect(s_hfp_handle);
  }

  return rc == 0 ? 0 : -EIO;
}

static int prv_cmd_tone(const struct pbl_shell *sh, size_t argc, char **argv) {
  unsigned long hz;

  if (pbl_shell_strtoul(argv[1], &hz) < 0 || hz >= RATE / 2U) {
    pbl_shell_error(sh, "not a tone: %s", argv[1]);
    return -EINVAL;
  }

  atomic_store(&s_tone_hz, (unsigned int)hz);

  return 0;
}

static int prv_cmd_beep(const struct pbl_shell *sh, size_t argc, char **argv) {
  unsigned long hz;

  if (pbl_shell_strtoul(argv[1], &hz) < 0 || hz >= RATE / 2U) {
    pbl_shell_error(sh, "not a tone: %s", argv[1]);
    return -EINVAL;
  }

  atomic_store(&s_beep_hz, (unsigned int)hz);

  return 0;
}

static int prv_cmd_gain(const struct pbl_shell *sh, size_t argc, char **argv) {
  bool mic = strcmp(argv[0], "mic") == 0;
  long db;
  int ret;

  if (pbl_shell_strtol(argv[1], &db) < 0) {
    pbl_shell_error(sh, "not a gain: %s", argv[1]);
    return -EINVAL;
  }

  ret = pbl_audio_set_gain(mic ? MIC : SPEAKER, mic ? PBL_AUDIO_CAPTURE : PBL_AUDIO_PLAYBACK,
                           (int32_t)db * 1000);
  if (ret != 0) {
    pbl_shell_error(sh, "setting the gain failed (%d)", ret);
  }

  return ret;
}

static const struct pbl_shell_cmd s_gain_cmds[] = {
  PBL_SHELL_CMD_ARG(mic, nullptr, "Microphone gain: mic <dB>", prv_cmd_gain, 2, 0),
  PBL_SHELL_CMD_ARG(speaker, nullptr, "Speaker gain: speaker <dB>", prv_cmd_gain, 2, 0),
  PBL_SHELL_SUBCMD_SET_END,
};

static int prv_cmd_keep(const struct pbl_shell *sh, size_t argc, char **argv) {
  atomic_store(&s_dump_source, DUMP_NONE);
  atomic_store(&s_dump_len, 0U);
  atomic_store(&s_dump_source, strcmp(argv[0], "mic") == 0 ? DUMP_MIC : DUMP_RECEIVED);

  return 0;
}

static int prv_cmd_dump(const struct pbl_shell *sh, size_t argc, char **argv) {
  char line[DUMP_LINE * 4U + 1U];

  if (atomic_load(&s_dump_source) != DUMP_NONE || atomic_load(&s_dump_len) != DUMP_FRAMES) {
    pbl_shell_error(sh, "nothing kept");
    return -ENOENT;
  }

  for (size_t i = 0U; i < DUMP_FRAMES; i += DUMP_LINE) {
    for (size_t j = 0U; j < DUMP_LINE; j++) {
      snprintf(&line[j * 4U], 5U, "%04x", (uint16_t)s_dump[i + j]);
    }
    pbl_shell_print(sh, "pcm %s", line);
  }

  return 0;
}

static const struct pbl_shell_cmd s_keep_cmds[] = {
  PBL_SHELL_CMD(mic, nullptr, "Keep what the microphone captures", prv_cmd_keep),
  PBL_SHELL_CMD(received, nullptr, "Keep what the phone sends", prv_cmd_keep),
  PBL_SHELL_SUBCMD_SET_END,
};

static int prv_cmd_echo(const struct pbl_shell *sh, size_t argc, char **argv) {
  atomic_store(&s_aec_enabled, strcmp(argv[0], "on") == 0);

  return 0;
}

static const struct pbl_shell_cmd s_echo_cmds[] = {
  PBL_SHELL_CMD(off, nullptr, "Send the microphone as it is", prv_cmd_echo),
  PBL_SHELL_CMD(on, nullptr, "Take the speaker out of the microphone", prv_cmd_echo),
  PBL_SHELL_SUBCMD_SET_END,
};

static const struct pbl_shell_cmd s_audio_cmds[] = {
  PBL_SHELL_CMD(off, nullptr, "Give the audio back to the phone", prv_cmd_audio),
  PBL_SHELL_CMD(on, nullptr, "Take the audio of the call", prv_cmd_audio),
  PBL_SHELL_SUBCMD_SET_END,
};

/* Connects to the phone seen last, as it does itself once paired. */
static int prv_cmd_connect(const struct pbl_shell *sh, size_t argc, char **argv) {
  int rc;

  if (!s_peer_known) {
    pbl_shell_error(sh, "no phone connected so far");
    return -ENOENT;
  }

  if (s_acl_handle != BLE_HS_CONN_HANDLE_NONE) {
    rc = ble_br_hfp_hf_connect(s_acl_handle);
  } else {
    s_connect_hfp = true;
    rc = ble_br_gap_connect(s_peer);
  }

  return rc == 0 ? 0 : -EIO;
}

/* Lets a phone find the device and pair with it over Classic, instead of over LE. */
static int prv_cmd_discoverable(const struct pbl_shell *sh, size_t argc, char **argv) {
  int rc;

  s_classic_pairing = true;
  rc = ble_br_gap_set_scan_mode(BLE_BR_GAP_SCAN_INQUIRY | BLE_BR_GAP_SCAN_PAGE);
  if (rc == 0 && ble_gap_adv_active()) {
    (void)ble_gap_adv_stop();
    prv_advertise();
  }

  return rc == 0 ? 0 : -EIO;
}

static int prv_cmd_disconnect(const struct pbl_shell *sh, size_t argc, char **argv) {
  return ble_br_gap_terminate(s_acl_handle, BLE_ERR_REM_USER_CONN_TERM) == 0 ? 0 : -EIO;
}

static int prv_cmd_status(const struct pbl_shell *sh, size_t argc, char **argv) {
  pbl_shell_print(sh, "classic %s, hands-free %s, audio %s",
                  s_acl_handle != BLE_HS_CONN_HANDLE_NONE ? "connected" : "disconnected",
                  s_hfp_handle != BLE_HS_CONN_HANDLE_NONE ? "connected" : "disconnected",
                  s_sco_handle != BLE_HS_CONN_HANDLE_NONE ? "connected" : "disconnected");
  pbl_shell_print(sh,
                  "audio: %u packets received (%u bad), %u sent, %u underruns, %u overruns, "
                  "%u gaps",
                  atomic_load(&s_rx_packets), atomic_load(&s_rx_bad), atomic_load(&s_tx_packets),
                  atomic_load(&s_underruns), atomic_load(&s_overruns), atomic_load(&s_tx_gaps));
  pbl_shell_print(sh, "echo: canceller %s, %d dB taken out (%d dB by the filter), %u samples late",
                  atomic_load(&s_aec_enabled) ? "on" : "off", aec_attenuation_db(&s_aec),
                  aec_filter_attenuation_db(&s_aec), (unsigned int)aec_delay(&s_aec));
  return 0;
}

static const struct pbl_shell_cmd s_hfp_cmds[] = {
  PBL_SHELL_CMD(answer, nullptr, "Answer the incoming call", prv_cmd_answer),
  PBL_SHELL_CMD(audio, s_audio_cmds, "Move the audio of the call", nullptr),
  PBL_SHELL_CMD_ARG(beep, nullptr, "Play a tone on the speaker: beep <Hz>, 0 to stop", prv_cmd_beep,
                    2, 0),
  PBL_SHELL_CMD(connect, nullptr, "Connect to the phone seen last", prv_cmd_connect),
  PBL_SHELL_CMD_ARG(dial, nullptr, "Place a call: dial <number>", prv_cmd_dial, 2, 0),
  PBL_SHELL_CMD(disconnect, nullptr, "Disconnect from the phone", prv_cmd_disconnect),
  PBL_SHELL_CMD(discoverable, nullptr, "Let phones pair over Classic", prv_cmd_discoverable),
  PBL_SHELL_CMD(dump, nullptr, "Print the half second of call audio kept", prv_cmd_dump),
  PBL_SHELL_CMD(echo, s_echo_cmds, "Turn the echo canceller on or off", nullptr),
  PBL_SHELL_CMD(gain, s_gain_cmds, "Set a gain", nullptr),
  PBL_SHELL_CMD(hangup, nullptr, "Reject or end the call", prv_cmd_hangup),
  PBL_SHELL_CMD(keep, s_keep_cmds, "Keep half a second of call audio", nullptr),
  PBL_SHELL_CMD(status, nullptr, "Show the connections and the audio counters", prv_cmd_status),
  PBL_SHELL_CMD_ARG(tone, nullptr, "Send a tone for the microphone: tone <Hz>, 0 to stop",
                    prv_cmd_tone, 2, 0),
  PBL_SHELL_CMD_ARG(volume, nullptr, "Set the speaker gain: volume <0-15>", prv_cmd_volume, 2, 0),
  PBL_SHELL_SUBCMD_SET_END,
};

PBL_SHELL_CMD_REGISTER(hfp, s_hfp_cmds, "Hands-free calls", nullptr);

/* The phone of the first link key in the store, if any: the one to connect to after a reset. */
static int prv_bonded_peer(int obj_type, union ble_store_value *value, void *cookie) {
  memcpy(s_peer, value->br_link_key.peer_addr.val, sizeof(s_peer));
  s_peer_known = true;

  return 1;
}

static void prv_on_sync(void) {
  pbl_sem_give(&s_synced);
}

static void prv_on_reset(int reason) {
  printf("hfp_hf: host reset, reason %d\n", reason);
}

int main(void) {
  uint8_t addr[6];
  int rc;

  rc = prv_audio_init();
  if (rc != 0) {
    printf("hfp_hf: no call audio (%d)\n", rc);
    return rc;
  }

  rc = pbl_fs_mount(&s_mount);
  if (rc != 0) {
    printf("hfp_hf: no storage for the bonds (%d)\n", rc);
    return rc;
  }

  nimble_port_init();
  ble_hs_cfg.sync_cb = prv_on_sync;
  ble_hs_cfg.reset_cb = prv_on_reset;
  /* A new bond replaces the oldest one when the store is full. */
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
  ble_store_config_init();

  /* The services go in before the host starts. */
  ble_svc_gap_init();
  ble_svc_gatt_init();

  rc = ble_svc_gap_device_name_set(NAME);
  if (rc == 0) {
    ble_br_gap_set_event_cb(prv_br_gap_event, nullptr);
    ble_br_sco_set_rx_cb(prv_sco_rx, nullptr);
    rc = ble_br_gap_set_name(NAME);
  }
  if (rc == 0) {
    rc = ble_br_gap_set_class_of_device(CLASS_OF_DEVICE);
  }
  if (rc == 0) {
    rc = ble_br_hfp_hf_init(prv_hfp_event, nullptr);
  }
  if (rc != 0) {
    printf("hfp_hf: adding the services failed (%d)\n", rc);
    return rc;
  }

  nimble_port_pebble_init();

  pbl_sem_take(&s_synced, PBL_FOREVER);

  rc = ble_hs_util_ensure_addr(0);
  if (rc == 0) {
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
  }
  if (rc == 0) {
    rc = ble_hs_id_copy_addr(s_own_addr_type, addr, nullptr);
  }
  if (rc != 0) {
    printf("hfp_hf: no address (%d)\n", rc);
    return rc;
  }

  prv_print_addr("address", addr);

  if (!ble_br_gap_supported()) {
    printf("hfp_hf: the controller has no Bluetooth Classic\n");
    return -ENOTSUP;
  }

  prv_advertise();

  /* A phone pairs over LE, which gives it the Classic key too, and then connects over Classic */
  rc = ble_br_gap_set_scan_mode(BLE_BR_GAP_SCAN_PAGE);
  if (rc != 0) {
    printf("hfp_hf: becoming connectable failed (%d)\n", rc);
    return rc;
  }

  printf("hfp_hf: connectable over Classic\n");

  /* Back to the phone it is bonded with, which may also be the one to connect first */
  (void)ble_store_iterate(BLE_STORE_OBJ_TYPE_BR_LINK_KEY, prv_bonded_peer, nullptr);
  if (s_peer_known) {
    prv_print_addr("connecting to", s_peer);
    s_reconnects = RECONNECT_ATTEMPTS;
    prv_reconnect(nullptr);
  }

  /* The calls are driven from the shell. */
  pbl_console_enable_prompt();

  while (true) {
    pbl_thread_sleep(PBL_SEC(60));
  }

  return 0;
}

/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <console/prompt.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <host/ble_hs_adv.h>
#include <host/ble_uuid.h>
#include <system/logging.h>
#include <util/math.h>

void command_ble_host_reset(void) {
  ble_hs_sched_reset(BLE_HS_EAPP);
  prompt_send_response("OK");
}

static const ble_uuid128_t s_pebble_index_svc_uuid =
    BLE_UUID128_INIT(0xc3, 0xb0, 0xbc, 0x00, 0xf9, 0x2d, 0x4a, 0xf4,
                     0x94, 0x4e, 0x00, 0x37, 0x9b, 0x5c, 0x7b, 0x60);

typedef struct __attribute__((packed)) {
  uint32_t cacheable_state_fingerprint;
  uint8_t truncated_collection_count;
  uint8_t flags;
} HaversineMfgData;

#define HAVERSINE_FLAG_IS_DARK              (1u << 3)
#define HAVERSINE_FLAG_HAS_DEBUG_INFO       (1u << 4)
#define HAVERSINE_FLAG_IN_COLLECTION_STATE  (1u << 5)
#define HAVERSINE_FLAG_NEEDS_SERVICING      (1u << 6)
#define HAVERSINE_FLAG_IS_MOVING            (1u << 7)

static bool s_scanning;

#define HAVERSINE_TRACK_MAX 4
typedef struct {
  bool valid;
  ble_addr_t addr;
  uint8_t last_count;
} HaversineSeen;
static HaversineSeen s_seen[HAVERSINE_TRACK_MAX];
static uint8_t s_seen_next;

static int prv_seen_find(const ble_addr_t *addr) {
  for (int i = 0; i < HAVERSINE_TRACK_MAX; i++) {
    if (s_seen[i].valid && ble_addr_cmp(&s_seen[i].addr, addr) == 0) {
      return i;
    }
  }
  return -1;
}

static int prv_seen_insert(const ble_addr_t *addr, uint8_t count) {
  int slot = s_seen_next;
  s_seen[slot].valid = true;
  s_seen[slot].addr = *addr;
  s_seen[slot].last_count = count;
  s_seen_next = (s_seen_next + 1) % HAVERSINE_TRACK_MAX;
  return slot;
}

static bool prv_adv_contains_uuid(const struct ble_hs_adv_fields *fields,
                                  const ble_uuid_t *target) {
  for (int i = 0; i < fields->num_uuids16; i++) {
    if (ble_uuid_cmp(&fields->uuids16[i].u, target) == 0) {
      return true;
    }
  }
  for (int i = 0; i < fields->num_uuids32; i++) {
    if (ble_uuid_cmp(&fields->uuids32[i].u, target) == 0) {
      return true;
    }
  }
  for (int i = 0; i < fields->num_uuids128; i++) {
    if (ble_uuid_cmp(&fields->uuids128[i].u, target) == 0) {
      return true;
    }
  }
  return false;
}

static int prv_scan_gap_event(struct ble_gap_event *event, void *arg) {
  struct ble_hs_adv_fields fields;
  char name[32];
  int rc;

  switch (event->type) {
    case BLE_GAP_EVENT_DISC:
      rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
      if (rc != 0) {
        return 0;
      }
      if (!prv_adv_contains_uuid(&fields, &s_pebble_index_svc_uuid.u)) {
        return 0;
      }

      if (fields.name && fields.name_len > 0) {
        size_t len = MIN(fields.name_len, sizeof(name) - 1);
        memcpy(name, fields.name, len);
        name[len] = '\0';
      } else {
        name[0] = '\0';
      }

      PBL_LOG_D_INFO(LOG_DOMAIN_BT,
                     "Pebble Index: %02x:%02x:%02x:%02x:%02x:%02x type=%u",
                     event->disc.addr.val[5], event->disc.addr.val[4],
                     event->disc.addr.val[3], event->disc.addr.val[2],
                     event->disc.addr.val[1], event->disc.addr.val[0],
                     event->disc.addr.type);
      PBL_LOG_D_INFO(LOG_DOMAIN_BT, "  rssi=%d name=\"%s\"", event->disc.rssi, name);

      if (!fields.mfg_data || fields.mfg_data_len != 2 + sizeof(HaversineMfgData)) {
        PBL_LOG_D_INFO(LOG_DOMAIN_BT, "  unexpected mfg_data_len=%u",
                       fields.mfg_data ? fields.mfg_data_len : 0);
        return 0;
      }

      uint16_t company_id = fields.mfg_data[0] | ((uint16_t)fields.mfg_data[1] << 8);
      HaversineMfgData hav;
      memcpy(&hav, &fields.mfg_data[2], sizeof(hav));

      int slot = prv_seen_find(&event->disc.addr);
      bool new_data;
      bool first_seen;
      if (slot < 0) {
        prv_seen_insert(&event->disc.addr, hav.truncated_collection_count);
        first_seen = true;
        new_data = false;
      } else {
        first_seen = false;
        new_data = (s_seen[slot].last_count != hav.truncated_collection_count);
        s_seen[slot].last_count = hav.truncated_collection_count;
      }

      PBL_LOG_D_INFO(LOG_DOMAIN_BT,
                     "  company=0x%04x fingerprint=0x%08lx count=%u flags=0x%02x",
                     company_id, (unsigned long)hav.cacheable_state_fingerprint,
                     hav.truncated_collection_count, hav.flags);
      if (new_data) {
        PBL_LOG_D_INFO(LOG_DOMAIN_BT, "  [NEW DATA]");
      } else if (first_seen) {
        PBL_LOG_D_INFO(LOG_DOMAIN_BT, "  [first]");
      }
      PBL_LOG_D_INFO(LOG_DOMAIN_BT,
                     "  dark=%u dbg=%u collecting=%u svc=%u moving=%u",
                     !!(hav.flags & HAVERSINE_FLAG_IS_DARK),
                     !!(hav.flags & HAVERSINE_FLAG_HAS_DEBUG_INFO),
                     !!(hav.flags & HAVERSINE_FLAG_IN_COLLECTION_STATE),
                     !!(hav.flags & HAVERSINE_FLAG_NEEDS_SERVICING),
                     !!(hav.flags & HAVERSINE_FLAG_IS_MOVING));
      return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
      s_scanning = false;
      PBL_LOG_D_INFO(LOG_DOMAIN_BT, "ble scan: complete reason=%d",
                     event->disc_complete.reason);
      return 0;
  }
  return 0;
}

// 0.625ms units, so 96 = 60ms.
#define PEBBLE_INDEX_SCAN_ITVL  96
#define PEBBLE_INDEX_SCAN_WIN   96

void command_pebble_index_scan_start(void) {
  struct ble_gap_disc_params disc_params = {0};
  uint8_t own_addr_type;
  char buf[64];
  int rc;

  if (s_scanning) {
    prompt_send_response("scan already in progress");
    return;
  }

  memset(s_seen, 0, sizeof(s_seen));
  s_seen_next = 0;

  rc = ble_hs_id_infer_auto(0, &own_addr_type);
  if (rc != 0) {
    prompt_send_response_fmt(buf, sizeof(buf), "infer addr failed (rc=%d)", rc);
    return;
  }

  disc_params.passive = 1;
  disc_params.itvl = PEBBLE_INDEX_SCAN_ITVL;
  disc_params.window = PEBBLE_INDEX_SCAN_WIN;
  disc_params.filter_duplicates = 0;

  rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, prv_scan_gap_event, NULL);
  if (rc != 0) {
    prompt_send_response_fmt(buf, sizeof(buf), "ble_gap_disc failed (rc=%d)", rc);
    return;
  }

  s_scanning = true;
  prompt_send_response("scanning for Pebble Index...");
}

void command_pebble_index_scan_stop(void) {
  char buf[64];
  int rc;

  if (!s_scanning) {
    prompt_send_response("not scanning");
    return;
  }

  rc = ble_gap_disc_cancel();
  if (rc != 0) {
    prompt_send_response_fmt(buf, sizeof(buf), "cancel failed (rc=%d)", rc);
    return;
  }

  s_scanning = false;
  prompt_send_response("OK");
}

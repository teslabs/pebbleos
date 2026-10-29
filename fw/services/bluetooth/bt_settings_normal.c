/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/kernel/mutex.h>
#include <pbl/logging/logging.h>
#include <pbl/services/bluetooth/bt_settings.h>
#include <pbl/services/filesystem/pfs.h>
#include <pbl/services/settings/settings_file.h>
#include <pbl/util/size.h>
#include <pbl/util/units.h>

#include <kernel/events.h>

PBL_LOG_MODULE_DECLARE(service_bluetooth, CONFIG_SERVICE_BLUETOOTH_LOG_LEVEL);

#define SETTINGS_FILE_NAME "bt_settings"
#define SETTINGS_FILE_SIZE PBL_KIB(1)

#define LEGACY_FILE_NAME "gap_bonding_db"
#define LEGACY_FILE_SIZE PBL_KIB(4)

static const char AIRPLANE_MODE_KEY[] = "airplane";
static const char UNFAITHFUL_KEY[] = "unfaithful";
static const char CAPABILITIES_KEY[] = "capabilities";

static const struct {
  const char *legacy_key;
  size_t legacy_key_len;
  const char *key;
  size_t key_len;
} s_legacy_keys[] = {
  {"AIRPLANE_MODE", sizeof("AIRPLANE_MODE"), AIRPLANE_MODE_KEY, sizeof(AIRPLANE_MODE_KEY)},
  {"IS_UNFAITHFUL", sizeof("IS_UNFAITHFUL"), UNFAITHFUL_KEY, sizeof(UNFAITHFUL_KEY)},
  {"SYSTEM_CAPABILITIES", sizeof("SYSTEM_CAPABILITIES"), CAPABILITIES_KEY,
   sizeof(CAPABILITIES_KEY)},
};

static PBL_MUTEX_DEFINE(s_mutex);

//! @note s_mutex must be held to access it.
static PebbleProtocolCapabilities s_capabilities;

static bool prv_get(const char *key, size_t key_len, void *val, size_t val_len) {
  SettingsFile fd;
  bool ret = false;

  pbl_mutex_lock(&s_mutex, PBL_FOREVER);

  if (settings_file_open(&fd, SETTINGS_FILE_NAME, SETTINGS_FILE_SIZE) != S_SUCCESS) {
    goto unlock;
  }

  ret = settings_file_get_len(&fd, key, key_len) == (int)val_len &&
        settings_file_get(&fd, key, key_len, val, val_len) == S_SUCCESS;
  settings_file_close(&fd);

unlock:
  pbl_mutex_unlock(&s_mutex);

  return ret;
}

static void prv_set(const char *key, size_t key_len, const void *val, size_t val_len) {
  SettingsFile fd;

  pbl_mutex_lock(&s_mutex, PBL_FOREVER);

  if (settings_file_open(&fd, SETTINGS_FILE_NAME, SETTINGS_FILE_SIZE) != S_SUCCESS) {
    PBL_LOG_ERR("Could not open the Bluetooth settings");
    goto unlock;
  }

  if (settings_file_set(&fd, key, key_len, val, val_len) != S_SUCCESS) {
    PBL_LOG_ERR("Could not store a Bluetooth setting");
  }
  settings_file_close(&fd);

unlock:
  pbl_mutex_unlock(&s_mutex);
}

static bool prv_get_bool(const char *key, size_t key_len, bool default_value) {
  bool value;

  if (!prv_get(key, key_len, &value, sizeof(value))) {
    return default_value;
  }

  return value;
}

static bool prv_legacy_file_exists(void) {
  int fd = pfs_open(LEGACY_FILE_NAME, OP_FLAG_READ, FILE_TYPE_STATIC, 0);

  if (fd < 0) {
    return false;
  }

  pfs_close(fd);

  return true;
}

//! Older firmware kept these settings in the bonding database.
static void prv_migrate(void) {
  SettingsFile legacy;

  if (!prv_legacy_file_exists() ||
      settings_file_open(&legacy, LEGACY_FILE_NAME, LEGACY_FILE_SIZE) != S_SUCCESS) {
    return;
  }

  for (size_t i = 0; i < ARRAY_LENGTH(s_legacy_keys); i++) {
    const char *legacy_key = s_legacy_keys[i].legacy_key;
    const size_t legacy_key_len = s_legacy_keys[i].legacy_key_len;
    const int len = settings_file_get_len(&legacy, legacy_key, legacy_key_len);
    uint8_t val[sizeof(PebbleProtocolCapabilities)];

    if (len <= 0 || len > (int)sizeof(val) ||
        settings_file_get(&legacy, legacy_key, legacy_key_len, val, len) != S_SUCCESS) {
      continue;
    }

    prv_set(s_legacy_keys[i].key, s_legacy_keys[i].key_len, val, len);
    settings_file_delete(&legacy, legacy_key, legacy_key_len);
  }

  settings_file_close(&legacy);
}

void bt_settings_init(void) {
  prv_migrate();

  PebbleProtocolCapabilities capabilities = {};
  (void)prv_get(CAPABILITIES_KEY, sizeof(CAPABILITIES_KEY), &capabilities, sizeof(capabilities));

  pbl_mutex_lock(&s_mutex, PBL_FOREVER);
  s_capabilities = capabilities;
  pbl_mutex_unlock(&s_mutex);
}

bool bt_settings_get_airplane_mode(void) {
  return prv_get_bool(AIRPLANE_MODE_KEY, sizeof(AIRPLANE_MODE_KEY), false);
}

void bt_settings_set_airplane_mode(bool enable) {
  prv_set(AIRPLANE_MODE_KEY, sizeof(AIRPLANE_MODE_KEY), &enable, sizeof(enable));
}

bool bt_settings_is_unfaithful(void) {
  return prv_get_bool(UNFAITHFUL_KEY, sizeof(UNFAITHFUL_KEY), true);
}

void bt_settings_set_unfaithful(bool is_unfaithful) {
  PBL_LOG_INFO("Marking the watch as %s", is_unfaithful ? "unfaithful" : "faithful");
  prv_set(UNFAITHFUL_KEY, sizeof(UNFAITHFUL_KEY), &is_unfaithful, sizeof(is_unfaithful));
}

void bt_settings_get_capabilities(PebbleProtocolCapabilities *capabilities_out) {
  pbl_mutex_lock(&s_mutex, PBL_FOREVER);
  *capabilities_out = s_capabilities;
  pbl_mutex_unlock(&s_mutex);
}

void bt_settings_set_capabilities(const PebbleProtocolCapabilities *capabilities) {
  const PebbleProtocolCapabilities new_capabilities =
      capabilities ? *capabilities : (PebbleProtocolCapabilities){};
  PebbleProtocolCapabilities diff;

  pbl_mutex_lock(&s_mutex, PBL_FOREVER);
  diff.flags = s_capabilities.flags ^ new_capabilities.flags;
  s_capabilities = new_capabilities;
  pbl_mutex_unlock(&s_mutex);

  if (!diff.flags) {
    return;
  }

  prv_set(CAPABILITIES_KEY, sizeof(CAPABILITIES_KEY), &new_capabilities, sizeof(new_capabilities));

  PebbleEvent event = {
    .type = PEBBLE_CAPABILITIES_CHANGED_EVENT,
    .capabilities.flags_diff = diff,
  };
  event_put(&event);
}

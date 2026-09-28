/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include <string.h>

#include "pbl/services/analytics/backend.h"
#include "pbl/services/data_logging/data_logging_service.h"
#include <pbl/drivers/rtc.h>
#include "pbl/util/build_id.h"

#include "stubs_logging.h"
#include "stubs_mutex.h"
#include "stubs_passert.h"
#include "stubs_prompt.h"
#include "stubs_rtc.h"

void pbl_analytics__native_heartbeat(void);

extern const struct pbl_analytics_backend_ops pbl_analytics__native_ops;

const ElfExternalNote TINTIN_BUILD_ID = {
  .name_length = 4,
  .data_length = BUILD_ID_EXPECTED_LEN,
  .type = 3,
};

static int s_dls_create_count;
static DataLoggingSession *s_dls_create_result;
static int s_dls_log_count;
// The backend keeps its session across tests, so they all share this one
static uint8_t s_session_storage;
static uint8_t s_last_record[1024];
static uint32_t s_last_record_size;

DataLoggingSession *dls_create(uint32_t tag, DataLoggingItemType item_type, uint16_t item_size,
                               bool buffered, bool resume, const Uuid *uuid) {
  s_dls_create_count++;
  s_last_record_size = item_size;
  return s_dls_create_result;
}

DataLoggingResult dls_log(DataLoggingSession *session, const void *data, uint32_t num_items) {
  cl_assert_equal_p(session, s_dls_create_result);
  cl_assert(s_last_record_size <= sizeof(s_last_record));
  memcpy(s_last_record, data, s_last_record_size);
  s_dls_log_count++;
  return DATA_LOGGING_SUCCESS;
}

void test_analytics_native__initialize(void) {
  s_dls_create_count = 0;
  s_dls_create_result = NULL;
  s_dls_log_count = 0;
}

void test_analytics_native__session_unavailable_retries_next_heartbeat(void) {
  pbl_analytics__native_heartbeat();
  cl_assert_equal_i(s_dls_create_count, 1);
  cl_assert_equal_i(s_dls_log_count, 0);

  s_dls_create_result = (DataLoggingSession *)&s_session_storage;
  pbl_analytics__native_heartbeat();
  cl_assert_equal_i(s_dls_create_count, 2);
  cl_assert_equal_i(s_dls_log_count, 1);

  pbl_analytics__native_heartbeat();
  cl_assert_equal_i(s_dls_create_count, 2);
  cl_assert_equal_i(s_dls_log_count, 2);
}

static bool prv_record_contains(const void *needle, size_t len) {
  for (size_t i = 0; i + len <= s_last_record_size; i++) {
    if (memcmp(&s_last_record[i], needle, len) == 0) {
      return true;
    }
  }
  return false;
}

void test_analytics_native__state_metrics_persist_across_heartbeats(void) {
  static const char *uuid = "{01234567-89ab-cdef-0123-456789abcdef}";
  static const char *fw_version = "v9.9.9-test";
  const uint32_t tick_subscribed = 0xA5A5A5A5;
  const uint32_t notifications = 0x5A5A5A5A;

  s_dls_create_result = (DataLoggingSession *)&s_session_storage;

  pbl_analytics__native_ops.set_string(PBL_ANALYTICS_KEY(watchface_uuid), uuid);
  pbl_analytics__native_ops.set_string(PBL_ANALYTICS_KEY(fw_version), fw_version);
  pbl_analytics__native_ops.set_unsigned(PBL_ANALYTICS_KEY(app_tick_timer_second_subscribed),
                                         tick_subscribed);
  pbl_analytics__native_ops.set_unsigned(PBL_ANALYTICS_KEY(notification_received_count),
                                         notifications);

  pbl_analytics__native_heartbeat();
  cl_assert(prv_record_contains(uuid, strlen(uuid)));
  cl_assert(prv_record_contains(fw_version, strlen(fw_version)));
  cl_assert(prv_record_contains(&tick_subscribed, sizeof(tick_subscribed)));
  cl_assert(prv_record_contains(&notifications, sizeof(notifications)));

  pbl_analytics__native_heartbeat();
  cl_assert(prv_record_contains(uuid, strlen(uuid)));
  cl_assert(!prv_record_contains(fw_version, strlen(fw_version)));
  cl_assert(prv_record_contains(&tick_subscribed, sizeof(tick_subscribed)));
  cl_assert(!prv_record_contains(&notifications, sizeof(notifications)));
}

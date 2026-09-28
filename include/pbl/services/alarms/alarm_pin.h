/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/util/uuid.h"
#include "pbl/services/alarms/alarm.h"
#include "system/status_codes.h"

#include <stddef.h>

status_t alarm_pin_add(time_t alarm_time, AlarmId id, AlarmType type, AlarmKind kind,
                       Uuid *uuid_out);

void alarm_pin_remove(Uuid *alarm_id);

status_t alarm_pin_remove_untracked_future(time_t now, const Uuid *tracked, size_t tracked_count);

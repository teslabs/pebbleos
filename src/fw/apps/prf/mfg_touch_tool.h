/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

//! @file mfg_touch_tool.h
//!
//! Temporary MFG tool that reports raw touch and gesture events on screen.

#include "process_management/pebble_process_md.h"

const PebbleProcessMd *mfg_touch_tool_app_get_info(void);

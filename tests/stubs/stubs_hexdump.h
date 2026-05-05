/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "system/hexdump.h"
#include "system/logging.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>

void hexdump_using_serial(int level, const char *module, const char *line_buffer) { }

void hexdump_using_prompt(int level, const char *module, const char *line_buffer) { }

void hexdump_using_pbllog(int level, const char *module, const char *line_buffer) { }

void hexdump_log(int level, const uint8_t *data, size_t length) { }

void hexdump_log_src(const char *module, int level,
                     const uint8_t *data, size_t length, HexdumpLineCallback cb) { }

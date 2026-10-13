/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <applib/fonts/fonts.h>

void system_resource_init(void);

bool system_resource_is_valid(void);

//! Invalidates the cached system and language pack fonts after a pack or the language changed.
void system_resource_language_changed(void);

GFont system_resource_get_font(const char *font_key);

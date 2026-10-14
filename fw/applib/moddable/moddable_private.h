/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdbool.h>

struct sxMachine;

//! Whether @p the is the running machine, with the XS stack it was created with.
bool moddable_machine_is_current(const struct sxMachine *the);

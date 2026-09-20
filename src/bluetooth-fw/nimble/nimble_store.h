/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef _NIMBLE_STORE_H_
#define _NIMBLE_STORE_H_

#include <stdbool.h>
#include <stdint.h>

// NULL key checks bond presence without deriving key material.
bool nimble_store_get_classic_key(const uint8_t peer[6], uint8_t key[16], void *context);
void nimble_store_init(void);
void nimble_store_unload(void);

#endif

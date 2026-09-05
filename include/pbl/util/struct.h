/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stddef.h>

//! Returns `struct_ptr->field_name` if `struct_ptr` isn't NULL, otherwise returns default_value
#define NULL_SAFE_FIELD_ACCESS(struct_ptr, field_name, default_value) \
  ((struct_ptr) ? ((struct_ptr)->field_name) : (default_value))

//! Pointer to the enclosing @p type from a pointer to its @p member. @p ptr
//! must be compatible with the member's type, so pass a const @p type for a
//! const @p ptr.
#define PBL_CONTAINER_OF(ptr, type, member) \
  ((type *)((char *)(1 ? (ptr) : &((type *)0)->member) - offsetof(type, member)))

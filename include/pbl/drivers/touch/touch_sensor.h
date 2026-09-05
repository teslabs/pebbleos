/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once


//! Enable or disable touch sensor interrupts.
//! When disabled, no touch events will be processed.
//! @param enabled true to enable, false to disable
void touch_sensor_set_enabled(bool enabled);

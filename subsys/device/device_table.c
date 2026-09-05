/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/device.h>

extern const struct pbl_device *const __pbl_devices_start[];
extern const struct pbl_device *const __pbl_devices_end[];

int pbl_device_init_all(void) {
  return pbl_device_init_table(__pbl_devices_start,
                               (size_t)(__pbl_devices_end - __pbl_devices_start));
}

int pbl_device_init_children(const struct pbl_device *parent) {
  int failures = 0;
  for (const struct pbl_device *const *dev = __pbl_devices_start; dev < __pbl_devices_end; dev++) {
    if ((*dev)->parent == parent && pbl_device_init(*dev) != 0) {
      failures++;
    }
  }
  return failures;
}

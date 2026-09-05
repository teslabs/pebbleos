/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/device.h>
#include <pbl/logging/logging.h>
#include "pbl/os/assert.h"

#include <errno.h>

PBL_LOG_MODULE_DEFINE(device, CONFIG_DEVICE_LOG_LEVEL);

static int prv_init_deps(const struct pbl_device *dev) {
  const struct pbl_device *parent = dev->parent;

  if (parent != NULL && parent->state->status != PBL_DEVICE_INITIALIZING &&
      pbl_device_init(parent) != 0) {
    PBL_LOG_ERR("%s: parent %s failed", dev->name, parent->name);
    return -ENODEV;
  }

  if (dev->deps == NULL) {
    return 0;
  }
  for (const struct pbl_device *const *dep = dev->deps; *dep != NULL; dep++) {
    if (pbl_device_init(*dep) != 0) {
      PBL_LOG_ERR("%s: dependency %s failed", dev->name, (*dep)->name);
      return -ENODEV;
    }
  }
  return 0;
}

int pbl_device_init(const struct pbl_device *dev) {
  struct pbl_device_state *state = dev->state;

  switch (state->status) {
    case PBL_DEVICE_READY:
    case PBL_DEVICE_FAILED:
      return state->res;
    case PBL_DEVICE_INITIALIZING:
      // A parent bringing up a child that is already on its way: the
      // initiator finishes it once the parent returns. Anything else is a cycle.
      OS_ASSERT(dev->parent != NULL && dev->parent->state->status == PBL_DEVICE_INITIALIZING);
      return 0;
    default:
      break;
  }

  state->status = PBL_DEVICE_INITIALIZING;

  int res = prv_init_deps(dev);
  if (res == 0 && dev->init != NULL) {
    res = dev->init(dev);
  }

  state->res = res;
  if (res == 0) {
    state->status = PBL_DEVICE_READY;
    PBL_LOG_DBG("%s ready", dev->name);
  } else {
    state->status = PBL_DEVICE_FAILED;
    PBL_LOG_ERR("%s init failed: %d", dev->name, res);
  }
  return res;
}

bool pbl_device_is_ready(const struct pbl_device *dev) {
  return dev->state->status == PBL_DEVICE_READY;
}

int pbl_device_init_table(const struct pbl_device *const *devs, size_t count) {
  int failures = 0;
  for (size_t i = 0; i < count; i++) {
    if (pbl_device_init(devs[i]) != 0) {
      failures++;
    }
  }
  return failures;
}

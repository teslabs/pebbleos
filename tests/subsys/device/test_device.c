/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include <pbl/device.h>
#include "pbl/os/assert.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stubs_logging.h"

static bool s_expect_assert;
static jmp_buf s_assert_jmp;

NORETURN os_assertion_failed(const char *filename, int line) {
  if (s_expect_assert) {
    longjmp(s_assert_jmp, 1);
  }
  fprintf(stderr, "assert at %s:%d\n", filename, line);
  abort();
}

NORETURN os_assertion_failed_lr(const char *filename, int line, uint32_t lr) {
  os_assertion_failed(filename, line);
}

// A driver type embedding struct pbl_device, as real drivers do.
struct test_dev {
  struct pbl_device dev;
  int fail_with;
  char tag;
};

static char s_order[16];
static int s_init_calls;

static int prv_init(const struct pbl_device *dev) {
  const struct test_dev *td = PBL_CONTAINER_OF(dev, const struct test_dev, dev);
  s_init_calls++;
  size_t n = strlen(s_order);
  s_order[n] = td->tag;
  s_order[n + 1] = '\0';
  return td->fail_with;
}

#define TEST_DEV(sym, _tag, _parent, _deps)                                    \
  PBL_DEVICE_STATE_DEFINE(sym);                                                \
  static struct test_dev sym = {                                               \
    .dev = PBL_DEVICE_INIT(sym, #sym, prv_init, _parent, _deps), .tag = _tag,  \
  }

// gpio <- i2c <- pmic ; sensor depends on i2c and gpio ; loner has no init
TEST_DEV(s_gpio, 'g', NULL, NULL);
TEST_DEV(s_i2c, 'i', NULL, PBL_DEVICE_DEPS(&s_gpio.dev));
TEST_DEV(s_pmic, 'p', &s_i2c.dev, NULL);
TEST_DEV(s_sensor, 's', &s_i2c.dev, PBL_DEVICE_DEPS(&s_gpio.dev));
PBL_DEVICE_STATE_DEFINE(s_loner);
static struct test_dev s_loner = {
  .dev = PBL_DEVICE_INIT(s_loner, "loner", NULL, NULL, NULL), .tag = 'l',
};

// An MFD that brings its child up from its own init
static struct test_dev s_mfd;
TEST_DEV(s_mfd_child, 'c', &s_mfd.dev, NULL);
static int prv_mfd_init(const struct pbl_device *dev) {
  int res = prv_init(dev);
  return res != 0 ? res : pbl_device_init(&s_mfd_child.dev);
}
PBL_DEVICE_STATE_DEFINE(s_mfd);
static struct test_dev s_mfd = {
  .dev = PBL_DEVICE_INIT(s_mfd, "mfd", prv_mfd_init, &s_pmic.dev, NULL), .tag = 'm',
};

// Cycle: a -> b -> a, closed at test time
TEST_DEV(s_cycle_a, 'a', NULL, NULL);
TEST_DEV(s_cycle_b, 'b', NULL, PBL_DEVICE_DEPS(&s_cycle_a.dev));

static struct test_dev *const s_all[] = {
  &s_sensor, &s_pmic, &s_loner, &s_gpio, &s_i2c, &s_mfd, &s_mfd_child, &s_cycle_a, &s_cycle_b,
};

void test_device__initialize(void) {
  s_order[0] = '\0';
  s_init_calls = 0;
  s_expect_assert = false;
  for (size_t i = 0; i < sizeof(s_all) / sizeof(s_all[0]); i++) {
    *s_all[i]->dev.state = (struct pbl_device_state){0};
    s_all[i]->fail_with = 0;
  }
}

void test_device__cleanup(void) {}

void test_device__deps_first(void) {
  cl_assert_equal_i(pbl_device_init(&s_pmic.dev), 0);
  cl_assert_equal_s(s_order, "gip");
  cl_assert(pbl_device_is_ready(&s_gpio.dev));
  cl_assert(pbl_device_is_ready(&s_i2c.dev));
  cl_assert(pbl_device_is_ready(&s_pmic.dev));
  cl_assert(!pbl_device_is_ready(&s_sensor.dev));
}

void test_device__init_is_idempotent(void) {
  cl_assert_equal_i(pbl_device_init(&s_i2c.dev), 0);
  cl_assert_equal_i(pbl_device_init(&s_i2c.dev), 0);
  cl_assert_equal_i(pbl_device_init(&s_gpio.dev), 0);
  cl_assert_equal_i(s_init_calls, 2);
}

void test_device__table_order_is_irrelevant(void) {
  const struct pbl_device *const table[] = {
    &s_sensor.dev, &s_pmic.dev, &s_loner.dev, &s_gpio.dev, &s_i2c.dev,
  };
  cl_assert_equal_i(pbl_device_init_table(table, 5), 0);
  cl_assert_equal_s(s_order, "gisp");
  cl_assert(pbl_device_is_ready(&s_loner.dev));
  cl_assert_equal_i(s_init_calls, 4);
}

void test_device__parent_first(void) {
  cl_assert_equal_i(pbl_device_init(&s_mfd.dev), 0);
  cl_assert_equal_s(s_order, "gipmc");
  cl_assert(pbl_device_is_ready(&s_mfd_child.dev));
  cl_assert_equal_i(s_init_calls, 5);
}

void test_device__child_first_pulls_parent(void) {
  cl_assert_equal_i(pbl_device_init(&s_mfd_child.dev), 0);
  cl_assert_equal_s(s_order, "gipmc");
  cl_assert_equal_i(s_init_calls, 5);
}

void test_device__failed_parent_fails_children(void) {
  s_pmic.fail_with = -EIO;
  cl_assert_equal_i(pbl_device_init(&s_mfd_child.dev), -ENODEV);
  cl_assert_equal_i(pbl_device_init(&s_mfd.dev), -ENODEV);
  cl_assert_equal_s(s_order, "gip");
}

void test_device__failure_propagates(void) {
  s_i2c.fail_with = -EIO;
  const struct pbl_device *const table[] = {&s_pmic.dev, &s_sensor.dev, &s_gpio.dev};
  cl_assert_equal_i(pbl_device_init_table(table, 3), 2);
  cl_assert_equal_s(s_order, "gi");
  cl_assert_equal_i(pbl_device_init(&s_i2c.dev), -EIO);
  cl_assert_equal_i(pbl_device_init(&s_pmic.dev), -ENODEV);
  cl_assert_equal_i(pbl_device_init(&s_sensor.dev), -ENODEV);
  cl_assert(pbl_device_is_ready(&s_gpio.dev));
  cl_assert(!pbl_device_is_ready(&s_i2c.dev));
  cl_assert_equal_i(s_init_calls, 2);
}

void test_device__cycle_asserts(void) {
  s_cycle_a.dev.deps = PBL_DEVICE_DEPS(&s_cycle_b.dev);

  s_expect_assert = true;
  if (setjmp(s_assert_jmp) == 0) {
    pbl_device_init(&s_cycle_a.dev);
    cl_fail("cycle not detected");
  }
  s_cycle_a.dev.deps = NULL;
}

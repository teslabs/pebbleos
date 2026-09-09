/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "applib/platform.h"
#include "applib/preferred_content_size.h"
#include "kernel/pebble_tasks.h"
#include "process_management/app_install_types.h"
#include "shell/system_theme.h"

// Fakes
///////////////

// Controllable replacements for stubs_pebble_tasks.h and stubs_syscalls.h so the tests can
// simulate a third-party app

static PebbleTask s_current_task = PebbleTask_KernelMain;

PebbleTask pebble_task_get_current(void) {
  return s_current_task;
}

struct pbl_thread *pebble_task_get_thread(PebbleTask task) {
  return NULL;
}

const char *pebble_task_get_name(PebbleTask task) {
  return NULL;
}

void pebble_task_unregister(PebbleTask task) {
}

struct pbl_thread *pebble_task_create(PebbleTask pebble_task, struct pbl_thread_attr *attr) {
  return NULL;
}

//! System install ids are negative, app-db (third-party) install ids are positive
static AppInstallId s_current_process_id = (AppInstallId)(-1);

AppInstallId sys_process_manager_get_current_process_id(void) {
  return s_current_process_id;
}

// Stubs
///////////////

#include "stubs_analytics.h"
#include "stubs_app_install_manager.h"
#include "stubs_fonts.h"
#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_process_manager.h"
#include "stubs_shell_prefs.h"

// Setup
///////////////

void test_system_theme__initialize(void) {
  s_current_task = PebbleTask_KernelMain;
  s_current_process_id = (AppInstallId)(-1);
  system_theme_set_content_size(PreferredContentSizeDefault);
}

// Tests
///////////////

PreferredContentSize prv_convert_content_size_between_platforms(PreferredContentSize size,
                                                                PlatformType from_platform,
                                                                PlatformType to_platform);

void test_system_theme__convert_content_size_between_platforms(void) {
  // Converting between the same platform should be return the input
  cl_assert_equal_i(prv_convert_content_size_between_platforms(PreferredContentSizeMedium,
                                                               PlatformTypeBasalt,
                                                               PlatformTypeBasalt),
                    PreferredContentSizeMedium);

  // Converting between two platforms with the same default content size should return the input
  cl_assert_equal_i(prv_convert_content_size_between_platforms(PreferredContentSizeMedium,
                                                               PlatformTypeDiorite,
                                                               PlatformTypeBasalt),
                    PreferredContentSizeMedium);

  // Passing in an invalid from_platform or to_platform should assert
  cl_assert_passert(prv_convert_content_size_between_platforms(PreferredContentSizeSmall,
                                                               PlatformTypeGabbro + 1,
                                                               PlatformTypeBasalt));
  cl_assert_passert(prv_convert_content_size_between_platforms(PreferredContentSizeSmall,
                                                               PlatformTypeBasalt,
                                                               PlatformTypeGabbro + 1));

  // Converting from Emery to Basalt should return one size smaller
  cl_assert_equal_i(prv_convert_content_size_between_platforms(PreferredContentSizeLarge,
                                                               PlatformTypeEmery,
                                                               PlatformTypeBasalt),
                    PreferredContentSizeMedium);

  // Converting from Aplite to Emery should return one size larger
  cl_assert_equal_i(prv_convert_content_size_between_platforms(PreferredContentSizeLarge,
                                                               PlatformTypeAplite,
                                                               PlatformTypeEmery),
                    PreferredContentSizeExtraLarge);

  // Converting from Diorite to Emery (one size up) with the maximum size should be clipped
  cl_assert_equal_i(prv_convert_content_size_between_platforms(PreferredContentSizeExtraLarge,
                                                               PlatformTypeDiorite,
                                                               PlatformTypeEmery),
                    PreferredContentSizeExtraLarge);

  // Converting from Emery to Chalk (one size down) with the minimum size should be clipped
  cl_assert_equal_i(prv_convert_content_size_between_platforms(PreferredContentSizeSmall,
                                                               PlatformTypeEmery,
                                                               PlatformTypeChalk),
                    PreferredContentSizeSmall);
}

//! The size a process renders at follows the user's preference for system processes, but stays at
//! the runtime platform's default for third-party apps.
void test_system_theme__content_size_for_process(void) {
  const PreferredContentSize platform_default =
      system_theme_get_default_content_size_for_runtime_platform();

  system_theme_set_content_size(PreferredContentSizeExtraLarge);

  // Kernel tasks and system apps follow the user's preference
  cl_assert_equal_i(system_theme_get_content_size_for_process(), PreferredContentSizeExtraLarge);
  s_current_task = PebbleTask_App;
  cl_assert_equal_i(system_theme_get_content_size_for_process(), PreferredContentSizeExtraLarge);

  // A third-party app keeps the platform default
  s_current_process_id = (AppInstallId)1;
  cl_assert_equal_i(system_theme_get_content_size_for_process(), platform_default);
}

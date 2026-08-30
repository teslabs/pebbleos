// @generated -- DO NOT EDIT

#include "system_app_ids.auto.h"
#include "process_management/pebble_process_md.h"
#include "resource/resource_ids.auto.h"

extern const PebbleProcessMd *tictoc_get_app_info(void);
extern const PebbleProcessMd *kickstart_get_app_info(void);
extern const PebbleProcessMd *low_power_face_get_app_info(void);
extern const PebbleProcessMd *settings_get_app_info(void);
extern const PebbleProcessMd *music_app_get_info(void);
extern const PebbleProcessMd *notifications_app_get_info(void);
extern const PebbleProcessMd *alarms_app_get_info(void);
extern const PebbleProcessMd *watchfaces_get_app_info(void);
extern const PebbleProcessMd *quick_launch_setup_get_app_info(void);
extern const PebbleProcessMd *timeline_get_app_info(void);
extern const PebbleProcessMd *timeline_past_get_app_info(void);
extern const PebbleProcessMd *launcher_menu_app_get_app_info(void);
extern const PebbleProcessMd *weather_app_get_info(void);
extern const PebbleProcessMd *workout_app_get_info(void);
extern const PebbleProcessMd *battery_critical_get_app_info(void);
extern const PebbleProcessMd *health_app_get_info(void);
extern const PebbleProcessMd *send_text_app_get_info(void);
extern const PebbleProcessMd *reminder_app_get_info(void);
extern const PebbleProcessMd *quiet_time_toggle_get_app_info(void);
extern const PebbleProcessMd *backlight_state_toggle_get_app_info(void);
extern const PebbleProcessMd *motion_backlight_toggle_get_app_info(void);
extern const PebbleProcessMd *airplane_mode_toggle_get_app_info(void);
extern const PebbleProcessMd *sports_app_get_info(void);
extern const PebbleProcessMd *timeline_full_get_app_info(void);
extern const PebbleProcessMd *notifications_clear_history_app_get_info(void);


static const AppRegistryEntry APP_RECORDS[] = {

  // System Apps
  {
    .id = APP_ID_TICTOC,
    .type = AppInstallStorageFw,
    .md_fn = &tictoc_get_app_info,
    .color.argb = GColorClearARGB8,
  },
  {
    .id = APP_ID_KICKSTART,
    .type = AppInstallStorageFw,
    .md_fn = &kickstart_get_app_info,
    .color.argb = GColorClearARGB8,
  },
  {
    .id = APP_ID_LOW_POWER_FACE,
    .type = AppInstallStorageFw,
    .md_fn = &low_power_face_get_app_info,
    .color.argb = GColorClearARGB8,
  },
  {
    .id = APP_ID_SETTINGS,
    .type = AppInstallStorageFw,
    .md_fn = &settings_get_app_info,
    .color.argb = GColorLightGrayARGB8,
  },
  {
    .id = APP_ID_MUSIC,
    .type = AppInstallStorageFw,
    .md_fn = &music_app_get_info,
    .color.argb = GColorOrangeARGB8,
  },
  {
    .id = APP_ID_NOTIFICATIONS,
    .type = AppInstallStorageFw,
    .md_fn = &notifications_app_get_info,
    .color.argb = GColorSunsetOrangeARGB8,
  },
  {
    .id = APP_ID_ALARMS,
    .type = AppInstallStorageFw,
    .md_fn = &alarms_app_get_info,
    .color.argb = GColorJaegerGreenARGB8,
  },
  {
    .id = APP_ID_WATCHFACES,
    .type = AppInstallStorageFw,
    .md_fn = &watchfaces_get_app_info,
    .color.argb = GColorJazzberryJamARGB8,
  },
  {
    .id = APP_ID_QUICK_LAUNCH_SETUP,
    .type = AppInstallStorageFw,
    .md_fn = &quick_launch_setup_get_app_info,
    .color.argb = GColorClearARGB8,
  },
  {
    .id = APP_ID_TIMELINE,
    .type = AppInstallStorageFw,
    .md_fn = &timeline_get_app_info,
    .color.argb = GColorClearARGB8,
  },
  {
    .id = APP_ID_TIMELINE_PAST,
    .type = AppInstallStorageFw,
    .md_fn = &timeline_past_get_app_info,
    .color.argb = GColorClearARGB8,
  },
  {
    .id = APP_ID_LAUNCHER_MENU,
    .type = AppInstallStorageFw,
    .md_fn = &launcher_menu_app_get_app_info,
    .color.argb = GColorClearARGB8,
  },
  {
    .id = APP_ID_WEATHER,
    .type = AppInstallStorageFw,
    .md_fn = &weather_app_get_info,
    .color.argb = GColorBlueMoonARGB8,
  },
  {
    .id = APP_ID_WORKOUT,
    .type = AppInstallStorageFw,
    .md_fn = &workout_app_get_info,
    .color.argb = GColorYellowARGB8,
  },
  {
    .id = APP_ID_BATTERY_CRITICAL,
    .type = AppInstallStorageFw,
    .md_fn = &battery_critical_get_app_info,
    .color.argb = GColorClearARGB8,
  },
  {
    .id = APP_ID_HEALTH_APP,
    .type = AppInstallStorageFw,
    .md_fn = &health_app_get_info,
    .color.argb = GColorSunsetOrangeARGB8,
  },
  {
    .id = APP_ID_SEND_TEXT,
    .type = AppInstallStorageFw,
    .md_fn = &send_text_app_get_info,
    .color.argb = GColorClearARGB8,
  },
  {
    .id = APP_ID_REMINDERS,
    .type = AppInstallStorageFw,
    .md_fn = &reminder_app_get_info,
    .color.argb = GColorChromeYellowARGB8,
  },
  {
    .id = APP_ID_QUIET_TIME_TOGGLE,
    .type = AppInstallStorageFw,
    .md_fn = &quiet_time_toggle_get_app_info,
    .color.argb = GColorClearARGB8,
  },
  {
    .id = APP_ID_BACKLIGHT_STATE_TOGGLE_UUID,
    .type = AppInstallStorageFw,
    .md_fn = &backlight_state_toggle_get_app_info,
    .color.argb = GColorClearARGB8,
  },
  {
    .id = APP_ID_MOTION_BACKLIGHT_TOGGLE,
    .type = AppInstallStorageFw,
    .md_fn = &motion_backlight_toggle_get_app_info,
    .color.argb = GColorClearARGB8,
  },
  {
    .id = APP_ID_AIRPLANE_MODE_TOGGLE,
    .type = AppInstallStorageFw,
    .md_fn = &airplane_mode_toggle_get_app_info,
    .color.argb = GColorClearARGB8,
  },
  {
    .id = APP_ID_SPORTS,
    .type = AppInstallStorageFw,
    .md_fn = &sports_app_get_info,
    .color.argb = GColorClearARGB8,
  },
  {
    .id = APP_ID_TIMELINE_FULL,
    .type = AppInstallStorageFw,
    .md_fn = &timeline_full_get_app_info,
    .color.argb = GColorClearARGB8,
  },
  {
    .id = APP_ID_NOTIFICATIONS_CLEAR_HISTORY,
    .type = AppInstallStorageFw,
    .md_fn = &notifications_clear_history_app_get_info,
    .color.argb = GColorClearARGB8,
  },

  // Resource (stored) Apps
  {
    .id = APP_ID_GOLF,
    .type = AppInstallStorageResources,
    .name = "Golf",
    .uuid = { 0xcf, 0x1e, 0x81, 0x6a, 0x9d, 0xb0, 0x45, 0x11, 0xbb, 0xb8, 0xf6, 0x0c, 0x48, 0xca, 0x8f, 0xac },
    .bin_resource_id = RESOURCE_ID_STORED_APP_GOLF,
    .icon_resource_id = DEFAULT_MENU_ICON,
    .color.argb = GColorClearARGB8,
  },
};

/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "mfg_touch_tool.h"

#include "applib/app.h"
#include "applib/event_service_client.h"
#include "applib/touch_service.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"
#include "console/console_internal.h"
#include "kernel/events.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/bluetooth/bluetooth_ctl.h"
#include "pbl/services/idle_watchdog.h"
#include "pbl/services/touch/touch.h"
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"

#include <stdio.h>

#define STATUS_STRING_LEN 128

typedef struct {
  Window window;
  TextLayer title;
  TextLayer status;
  char status_string[STATUS_STRING_LEN];
  EventServiceInfo gesture_info;
  uint32_t touch_count;
  uint32_t gesture_count;
  int16_t last_x;
  int16_t last_y;
  bool touch_enabled;
} AppData;

static void prv_update_display(AppData *data) {
  sniprintf(data->status_string, sizeof(data->status_string),
            "Touch: %s\nEvents: %" PRIu32 "\nLast: (%" PRId16 ", %" PRId16 ")\n"
            "Gestures: %" PRIu32 "\n\nSEL: toggle",
            data->touch_enabled ? "ON" : "OFF", data->touch_count,
            data->last_x, data->last_y, data->gesture_count);
  text_layer_set_text(&data->status, data->status_string);
}

static void prv_touch_event_handler(const TouchEvent *event, void *context) {
  AppData *data = app_state_get_user_data();

  data->touch_count++;
  data->last_x = event->x;
  data->last_y = event->y;

  prv_update_display(data);
}

static void prv_gesture_event_handler(PebbleEvent *e, void *context) {
  AppData *data = app_state_get_user_data();

  data->gesture_count++;

  prv_update_display(data);
}

static void prv_select_click_handler(ClickRecognizerRef recognizer, void *context) {
  AppData *data = app_state_get_user_data();

  data->touch_enabled = !data->touch_enabled;
  touch_service_set_globally_enabled(data->touch_enabled);

  prv_update_display(data);
}

static void prv_config_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_select_click_handler);
}

static void prv_handle_init(void) {
  AppData *data = app_malloc_check(sizeof(AppData));
  *data = (AppData){
    .touch_enabled = true,
  };

  app_state_set_user_data(data);

  // Disable power consumers and idle watchdog for the duration of the tool.
  serial_console_set_rx_enabled(false);
  bt_ctl_set_enabled(false);
  prf_idle_watchdog_stop();

  Window *window = &data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);
  window_set_click_config_provider(window, prv_config_provider);

  Layer *window_layer = &window->layer;
  GRect bounds = window_layer->bounds;

  TextLayer *title = &data->title;
  const int16_t title_y = PBL_IF_ROUND_ELSE(10, 0);
  text_layer_init(title, &GRect(0, title_y, bounds.size.w, 24));
  text_layer_set_font(title, fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD));
  text_layer_set_text_alignment(title, GTextAlignmentCenter);
  text_layer_set_text(title, "TOUCH TOOL");
  layer_add_child(window_layer, &title->layer);

  TextLayer *status = &data->status;
  const int16_t status_y = PBL_IF_ROUND_ELSE(40, 25);
  const int16_t status_x = PBL_IF_ROUND_ELSE(15, 5);
  text_layer_init(status, &GRect(status_x, status_y, bounds.size.w - (status_x * 2),
                                 bounds.size.h - status_y));
  text_layer_set_font(status, fonts_get_system_font(FONT_KEY_GOTHIC_18));
  text_layer_set_text_alignment(status, PBL_IF_ROUND_ELSE(GTextAlignmentCenter,
                                                          GTextAlignmentLeft));
  layer_add_child(window_layer, &status->layer);

  touch_service_set_globally_enabled(true);
  touch_service_subscribe(prv_touch_event_handler, data);

  data->gesture_info = (EventServiceInfo){
    .type = PEBBLE_GESTURE_EVENT,
    .handler = prv_gesture_event_handler,
  };
  event_service_client_subscribe(&data->gesture_info);

  prv_update_display(data);

  app_window_stack_push(window, true);
}

static void s_main(void) {
  prv_handle_init();

  app_event_loop();

  AppData *data = app_state_get_user_data();
  event_service_client_unsubscribe(&data->gesture_info);
  touch_service_unsubscribe();

  serial_console_set_rx_enabled(true);
  bt_ctl_set_enabled(true);
  prf_idle_watchdog_start();
}

const PebbleProcessMd *mfg_touch_tool_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main,
    // UUID: 7f2c9a40-3b18-4e6d-9c21-5a8e1f0b6d34
    .common.uuid = { 0x7f, 0x2c, 0x9a, 0x40, 0x3b, 0x18, 0x4e, 0x6d,
                     0x9c, 0x21, 0x5a, 0x8e, 0x1f, 0x0b, 0x6d, 0x34 },
    .name = "MfgTouchTool",
  };
  return (const PebbleProcessMd *)&s_app_info;
}

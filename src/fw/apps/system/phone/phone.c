/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "applib/app.h"
#include "applib/app_timer.h"
#include "applib/touch_service.h"
#include "applib/ui/ui.h"
#include "applib/ui/action_menu_window_private.h"
#include "kernel/pbl_malloc.h"
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"
#include <pbl/services/bluetooth/hfp.h>
#include <pbl/services/phone_call_contacts.h>
#include <pbl/services/blob_db/api.h>
#include <pbl/services/event_service.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define KEY_PLUS       12
#define KEY_CLEAR      13
#define KEY_CALL       14
#define KEY_COUNT      15
#define CONTACT_HEIGHT 57
#define PAGE_TOP       38
#define ACCENT         GColorCobaltBlue

enum {
  CallEnd,
  CallMute,
  CallQuieter,
  CallLouder,
  CallTransfer,
  CallMore,
  CallControlCount
};

static const char s_digits[] = "123456789*0#+";

typedef struct {
  Window window;
  Layer canvas;
  AppTimer *timer;
  HfpStatus status;
  PhoneContact contacts[PHONE_MAX_CONTACTS];
  unsigned contact_count, contact_revision;
  EventServiceInfo contacts_event;
  unsigned page;
  unsigned focused_key;
  unsigned focused_contact;
  int scroll;
  int scroll_start;
  int touch_target;
  bool touching;
  bool moved;
  bool dialed_here;
  ActionMenuConfig call_menu;
  ActionMenu *call_menu_window;
  HfpStatus call_menu_status;
  GPoint touch_start;
  char number[33];
  const char *notice;
} AppData;

static void text(GContext *ctx, const char *value, const char *font, GRect rect, GColor color,
                 GTextAlignment alignment) {
  graphics_context_set_text_color(ctx, color);
  graphics_draw_text(ctx, value, fonts_get_system_font(font), rect,
                     GTextOverflowModeTrailingEllipsis, alignment, NULL);
}

static void rounded(GContext *ctx, GRect rect, GColor color, int radius) {
  graphics_context_set_fill_color(ctx, color);
  graphics_fill_round_rect(ctx, &rect, radius, GCornersAll);
}

static bool call_in_progress(const HfpStatus *status) {
  return status->call || status->incoming || status->call_setup;
}

static GRect key_rect(AppData *d, unsigned key) {
  GSize size = d->canvas.bounds.size;
  if (key == KEY_CLEAR)
    return GRect(size.w - 47, PAGE_TOP, 43, 33);
  if (key == KEY_PLUS)
    return GRect(8, size.h - 41, 43, 33);
  if (key == KEY_CALL)
    return GRect(57, size.h - 41, size.w - 65, 33);
  int width = size.w - 12;
  int height = size.h - 119;
  unsigned column = key % 3, row = key / 3;
  return GRect(6 + column * width / 3, 74 + row * height / 4,
               (column + 1) * width / 3 - column * width / 3 - 4,
               (row + 1) * height / 4 - row * height / 4 - 3);
}

static void draw_connection(AppData *d, GContext *ctx) {
  GSize size = d->canvas.bounds.size;
  int x = size.w / 2, y = size.h / 3;
  graphics_context_set_fill_color(ctx, ACCENT);
  graphics_fill_circle(ctx, GPoint(x, y), 29);
  graphics_context_set_stroke_color(ctx, GColorWhite);
  graphics_context_set_stroke_width(ctx, 3);
  graphics_draw_line(ctx, GPoint(x - 10, y - 12), GPoint(x + 11, y + 10));
  graphics_draw_line(ctx, GPoint(x + 11, y + 10), GPoint(x, y + 20));
  graphics_draw_line(ctx, GPoint(x, y + 20), GPoint(x, y - 20));
  graphics_draw_line(ctx, GPoint(x, y - 20), GPoint(x + 11, y - 10));
  graphics_draw_line(ctx, GPoint(x + 11, y - 10), GPoint(x - 10, y + 12));
  graphics_context_set_stroke_width(ctx, 1);
  text(ctx, d->status.connected ? "Connecting..." : "Connect your phone", FONT_KEY_GOTHIC_24_BOLD,
       GRect(6, y + 40, size.w - 12, 32), GColorBlack, GTextAlignmentCenter);
  text(ctx, "Pair with the companion app\nand enable phone calls", FONT_KEY_GOTHIC_18,
       GRect(13, y + 76, size.w - 26, 65), GColorDarkGray, GTextAlignmentCenter);
}

static void draw_header(AppData *d, GContext *ctx) {
  int width = d->canvas.bounds.size.w;
  for (unsigned page = 0; page < 2; ++page) {
    GRect tab = GRect(page * width / 2 + 3, 2, width / 2 - 6, 29);
    text(ctx, page ? "Contacts" : "Dialer", FONT_KEY_GOTHIC_18_BOLD, tab,
         d->page == page ? ACCENT : GColorDarkGray, GTextAlignmentCenter);
    if (d->page == page) {
      GRect line = GRect(tab.origin.x + 14, 31, tab.size.w - 28, 3);
      rounded(ctx, line, ACCENT, 1);
    }
  }
}

static void draw_dialer(AppData *d, GContext *ctx) {
  GSize size = d->canvas.bounds.size;
  unsigned length = strlen(d->number);
  const char *number = length > 13 ? d->number + length - 13 : d->number;
  text(ctx,
       d->notice ? d->notice
       : length  ? number
                 : "Phone number",
       FONT_KEY_GOTHIC_18_BOLD, GRect(8, PAGE_TOP + 3, size.w - 58, 29),
       length ? GColorBlack : GColorDarkGray, GTextAlignmentCenter);
  for (unsigned i = 0; i < KEY_COUNT; ++i) {
    GRect rect = key_rect(d, i);
    bool focused = i == d->focused_key;
    if (i == KEY_CLEAR) {
      if (focused)
        rounded(ctx, rect, GColorLightGray, 7);
      int x = rect.origin.x + rect.size.w / 2;
      int y = rect.origin.y + rect.size.h / 2;
      graphics_context_set_stroke_color(ctx, GColorRed);
      graphics_context_set_stroke_width(ctx, 3);
      graphics_draw_line(ctx, GPoint(x - 6, y - 6), GPoint(x + 6, y + 6));
      graphics_draw_line(ctx, GPoint(x - 6, y + 6), GPoint(x + 6, y - 6));
      graphics_context_set_stroke_width(ctx, 1);
      continue;
    }
    GColor color = i == KEY_CALL ? GColorIslamicGreen : focused ? ACCENT : GColorLightGray;
    rounded(ctx, rect, color, 7);
    if (i == KEY_CALL && focused) {
      graphics_context_set_stroke_color(ctx, GColorBlack);
      graphics_draw_round_rect(ctx, &rect, 7);
    }
    char digit[] = {i < KEY_CLEAR ? s_digits[i] : 0, 0};
    const char *label = i == KEY_CALL ? "Call" : digit;
    bool small = i >= KEY_CLEAR;
    rect.origin.y += (rect.size.h - (small ? 22 : 28)) / 2 - 2;
    text(ctx, label, small ? FONT_KEY_GOTHIC_18_BOLD : FONT_KEY_GOTHIC_24_BOLD, rect,
         focused || i == KEY_CALL ? GColorWhite : GColorBlack, GTextAlignmentCenter);
  }
}

static int max_scroll(AppData *d) {
  int max = (int)d->contact_count * CONTACT_HEIGHT - (d->canvas.bounds.size.h - PAGE_TOP);
  return max > 0 ? max : 0;
}

static void clamp_scroll(AppData *d) {
  if (d->scroll < 0)
    d->scroll = 0;
  if (d->scroll > max_scroll(d))
    d->scroll = max_scroll(d);
}

static void draw_contacts(AppData *d, GContext *ctx) {
  GSize size = d->canvas.bounds.size;
  if (!d->contact_count) {
    text(ctx, "No contacts yet", FONT_KEY_GOTHIC_24_BOLD,
         GRect(8, size.h / 2 - 15, size.w - 16, 36), GColorDarkGray, GTextAlignmentCenter);
    return;
  }
  for (unsigned i = 0; i < d->contact_count; ++i) {
    int y = PAGE_TOP + i * CONTACT_HEIGHT - d->scroll;
    if (y + CONTACT_HEIGHT <= PAGE_TOP || y >= size.h)
      continue;
    bool focused = d->focused_contact == i;
    rounded(ctx, GRect(6, y + 2, size.w - 12, CONTACT_HEIGHT - 4),
            focused ? ACCENT : GColorLightGray, 7);
    GColor color = focused ? GColorWhite : GColorBlack;
    text(ctx, d->contacts[i].name, FONT_KEY_GOTHIC_24_BOLD, GRect(14, y, size.w - 26, 30), color,
         GTextAlignmentLeft);
    text(ctx, d->contacts[i].number, FONT_KEY_GOTHIC_18, GRect(14, y + 28, size.w - 26, 24), color,
         GTextAlignmentLeft);
  }
  // Keep scrolling rows beneath the fixed tabs.
  GRect header = GRect(0, 0, size.w, PAGE_TOP);
  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_fill_rect(ctx, &header);
}

static GRect call_control_rect(AppData *d, unsigned control) {
  GSize size = d->canvas.bounds.size;
  if (control == CallMore)
    return GRect(size.w - 38, 0, 34, 30);
  if (control == CallEnd)
    return GRect(15, size.h - 50, size.w - 30, 40);
  if (control == CallMute || control == CallTransfer)
    return GRect(control == CallMute ? 15 : size.w / 2 + 3, size.h - 96, (size.w - 36) / 2, 40);
  return GRect(control == CallQuieter ? 10 : size.w - 50, size.h - 142, 40, 36);
}

static void draw_call(AppData *d, GContext *ctx) {
  GSize size = d->canvas.bounds.size;
  const char *title = d->status.waiting          ? "Call waiting"
                      : d->status.call_held == 2 ? "Call on hold"
                      : d->status.call ? (d->status.audio ? "Call in progress" : "Call on phone")
                      : d->status.incoming ? "Incoming call"
                                           : "Calling...";
  bool more = d->status.call && d->status.hold_support;
  text(ctx, d->notice ? d->notice : title, FONT_KEY_GOTHIC_24_BOLD,
       GRect(8, 0, size.w - (more ? 46 : 16), 32), GColorBlack, GTextAlignmentCenter);
  const char *number = d->status.waiting          ? d->status.waiting_number
                       : *d->status.caller_number ? d->status.caller_number
                       : d->dialed_here           ? d->number
                                                  : "";
  const char *caller = *number ? number : "On your phone";
  if (*number) {
    for (unsigned i = 0; i < d->contact_count; ++i) {
      if (!strcmp(number, d->contacts[i].number)) {
        caller = d->contacts[i].name;
        break;
      }
    }
  }
  text(ctx, caller, FONT_KEY_GOTHIC_24_BOLD, GRect(8, 32, size.w - 16, 52), GColorDarkGray,
       GTextAlignmentCenter);
  char volume[24];
  snprintf(volume, sizeof(volume), "Volume %u%%", (d->status.speaker_gain * 100 + 7) / 15);
  text(ctx, volume, FONT_KEY_GOTHIC_18, GRect(48, size.h - 136, size.w - 96, 26), GColorDarkGray,
       GTextAlignmentCenter);
  const char *labels[] = {
    "End call",
    !d->status.audio      ? "Watch mic"
    : d->status.mic_muted ? "Unmute"
                          : "Mute",
    "-",
    "+",
    d->status.audio_pending ? "Switching"
    : d->status.audio       ? "Use phone"
                            : "Use watch",
    "..."
  };
  for (unsigned i = 0; i < CallControlCount; ++i) {
    if (i == CallMore && !more)
      continue;
    GRect rect = call_control_rect(d, i);
    GColor color = i == CallEnd                                              ? GColorRed
                   : i == CallMute && d->status.audio && d->status.mic_muted ? ACCENT
                                                                             : GColorLightGray;
    rounded(ctx, rect, color, 8);
    bool disabled = i == CallMute && !d->status.audio;
    bool small = i == CallTransfer || disabled;
    if (small)
      rect.origin.y += 4;
    text(ctx, labels[i], small ? FONT_KEY_GOTHIC_18_BOLD : FONT_KEY_GOTHIC_24_BOLD, rect,
         disabled                                                 ? GColorDarkGray
         : i == CallEnd || (i == CallMute && d->status.mic_muted) ? GColorWhite
                                                                  : GColorBlack,
         GTextAlignmentCenter);
  }
}

static void call_menu_closed(ActionMenu *menu, const ActionMenuItem *item, void *context) {
  AppData *d = context;
  task_free(action_menu_get_root_level(menu));
  d->call_menu.root_level = NULL;
  d->call_menu_window = NULL;
  app_touch_navigation_enable(false);
}

static void call_menu_action(ActionMenu *menu, const ActionMenuItem *item, void *context) {
  AppData *d = context;
  HfpStatus status;
  hfp_get_status(&status);
  if (status.waiting != d->call_menu_status.waiting ||
      status.call_held != d->call_menu_status.call_held ||
      status.call != d->call_menu_status.call || !status.ready)
    return;
  hfp_call_hold((uintptr_t)item->action_data);
}

static void show_call_menu(AppData *d) {
  hfp_get_status(&d->status);
  if (!d->status.ready || !d->status.call || !d->status.hold_support || d->call_menu.root_level)
    return;
  ActionMenuLevel *level = task_zalloc_check(sizeof(*level) + 4 * sizeof(ActionMenuItem));
  level->display_mode = ActionMenuLevelDisplayModeWide;
  const char *labels[] = {
    d->status.waiting ? "Reject waiting call" : "End held call",
    d->status.waiting ? "End & answer" : "End & resume",
    d->status.waiting          ? "Hold & answer"
    : d->status.call_held == 2 ? "Resume call"
    : d->status.call_held      ? "Swap calls"
                               : "Hold call",
    "Merge calls",
  };
  for (unsigned action = 0; action < 4; ++action) {
    bool allowed = action == 2 || d->status.waiting || d->status.call_held;
    if (action == 3)
      allowed = d->status.call_held == 1 && !d->status.waiting;
    if (!allowed || !(d->status.hold_support & (1u << action)))
      continue;
    level->items[level->num_items++] = (ActionMenuItem){
      .label = labels[action],
      .perform_action = call_menu_action,
      .action_data = (void *)(uintptr_t)action,
    };
  }
  if (!level->num_items) {
    task_free(level);
    return;
  }
  d->call_menu = (ActionMenuConfig){
    .context = d,
    .colors.background = ACCENT,
    .root_level = level,
    .did_close = call_menu_closed,
  };
  app_touch_navigation_enable(true);
  d->call_menu_status = d->status;
  d->call_menu_window = app_action_menu_open(&d->call_menu);
}

static void draw(Layer *layer, GContext *ctx) {
  AppData *d = app_state_get_user_data();
  graphics_context_set_fill_color(ctx, GColorWhite);
  graphics_fill_rect(ctx, &layer->bounds);
  if (!d->status.ready)
    draw_connection(d, ctx);
  else if (call_in_progress(&d->status))
    draw_call(d, ctx);
  else {
    if (d->page)
      draw_contacts(d, ctx);
    else
      draw_dialer(d, ctx);
    draw_header(d, ctx);
  }
}

static void dial(AppData *d, const char *number) {
  hfp_get_status(&d->status);
  if (!d->status.ready || d->status.busy || call_in_progress(&d->status))
    return;
  if (hfp_dial(number)) {
    d->dialed_here = true;
    if (number != d->number)
      snprintf(d->number, sizeof(d->number), "%s", number);
    d->notice = "Calling...";
  } else
    d->notice = "Enter a number";
}

static void activate(AppData *d, unsigned target) {
  hfp_get_status(&d->status);
  if (!d->status.ready)
    return;
  d->notice = NULL;
  if (call_in_progress(&d->status)) {
    if (target == CallMore)
      show_call_menu(d);
    else if (target == CallEnd && !d->status.busy)
      hfp_hangup();
    else if (target == CallMute && d->status.audio)
      hfp_set_mic_muted(!d->status.mic_muted);
    else if (target == CallTransfer && d->status.call && !d->status.audio_pending)
      hfp_transfer_audio(!d->status.audio);
    else if (target == CallQuieter && d->status.speaker_gain > 0)
      hfp_set_speaker_gain(d->status.speaker_gain - 1);
    else if (target == CallLouder && d->status.speaker_gain < 15)
      hfp_set_speaker_gain(d->status.speaker_gain + 1);
  } else if (d->page) {
    if (target < d->contact_count)
      dial(d, d->contacts[target].number);
  } else {
    unsigned length = strlen(d->number);
    if (target < KEY_CLEAR && length < sizeof(d->number) - 1) {
      char c = s_digits[target];
      if (c != '+' || !length) {
        d->number[length] = c;
        d->number[length + 1] = 0;
      }
    } else if (target == KEY_CLEAR)
      d->number[0] = 0;
    else if (target == KEY_CALL)
      dial(d, d->number);
  }
  layer_mark_dirty(&d->canvas);
}

static void change_page(AppData *d, unsigned page) {
  if (!d->status.ready || call_in_progress(&d->status))
    return;
  d->page = page;
  d->notice = NULL;
  layer_mark_dirty(&d->canvas);
}

static void move_focus(AppData *d, int direction) {
  if (call_in_progress(&d->status)) {
    activate(d, direction < 0 ? CallLouder : CallQuieter);
    return;
  }
  if (!d->status.ready || call_in_progress(&d->status))
    return;
  if (d->page && d->contact_count) {
    d->focused_contact = (d->focused_contact + d->contact_count + direction) % d->contact_count;
    int y = d->focused_contact * CONTACT_HEIGHT;
    if (y < d->scroll)
      d->scroll = y;
    if (y + CONTACT_HEIGHT > d->scroll + d->canvas.bounds.size.h - PAGE_TOP)
      d->scroll = y + CONTACT_HEIGHT - (d->canvas.bounds.size.h - PAGE_TOP);
    clamp_scroll(d);
  } else
    d->focused_key = (d->focused_key + KEY_COUNT + direction) % KEY_COUNT;
  layer_mark_dirty(&d->canvas);
}

static void up(ClickRecognizerRef recognizer, void *context) {
  move_focus(context, -1);
}
static void down(ClickRecognizerRef recognizer, void *context) {
  move_focus(context, 1);
}
static void choose(ClickRecognizerRef recognizer, void *context) {
  AppData *d = context;
  activate(d, call_in_progress(&d->status) ? CallEnd
              : d->page                    ? d->focused_contact
                                           : d->focused_key);
}
static void switch_page(ClickRecognizerRef recognizer, void *context) {
  AppData *d = context;
  if (call_in_progress(&d->status))
    activate(d, d->status.call && d->status.hold_support ? CallMore : CallMute);
  else
    change_page(d, !d->page);
}
static void clicks(void *context) {
  window_single_repeating_click_subscribe(BUTTON_ID_UP, 180, up);
  window_single_repeating_click_subscribe(BUTTON_ID_DOWN, 180, down);
  window_single_click_subscribe(BUTTON_ID_SELECT, choose);
  window_long_click_subscribe(BUTTON_ID_SELECT, 600, switch_page, NULL);
}

static int target_at(AppData *d, GPoint point) {
  if (call_in_progress(&d->status)) {
    for (unsigned i = 0; i < CallControlCount; ++i) {
      if (i == CallMore && !(d->status.call && d->status.hold_support))
        continue;
      GRect rect = call_control_rect(d, i);
      if (grect_contains_point(&rect, &point))
        return i;
    }
    return -1;
  }
  if (point.y < PAGE_TOP)
    return -1;
  if (d->page) {
    int row = (point.y - PAGE_TOP + d->scroll) / CONTACT_HEIGHT;
    return row < (int)d->contact_count ? row : -1;
  }
  for (unsigned key = 0; key < KEY_COUNT; ++key) {
    GRect rect = key_rect(d, key);
    if (grect_contains_point(&rect, &point))
      return key;
  }
  return -1;
}

static void touch(const TouchEvent *event, void *context) {
  AppData *d = context;
  if (event->non_navigational || !d->status.ready) {
    d->touching = false;
    return;
  }
  GPoint origin = layer_convert_point_to_screen(&d->canvas, GPointZero);
  GPoint point = GPoint(event->x - origin.x, event->y - origin.y);
  if (event->type == TouchEvent_Touchdown) {
    d->touching = true;
    d->moved = false;
    d->touch_start = point;
    d->scroll_start = d->scroll;
    d->touch_target = target_at(d, point);
    if (d->touch_target >= 0) {
      if (d->page)
        d->focused_contact = d->touch_target;
      else
        d->focused_key = d->touch_target;
    }
  } else if (d->touching) {
    int dx = point.x - d->touch_start.x, dy = point.y - d->touch_start.y;
    if (abs(dx) > 12 || abs(dy) > 12)
      d->moved = true;
    if (d->page && d->moved && abs(dy) > abs(dx) && !call_in_progress(&d->status)) {
      d->scroll = d->scroll_start - dy;
      clamp_scroll(d);
    }
    if (event->type == TouchEvent_Liftoff) {
      d->touching = false;
      if (abs(dx) >= 30 && abs(dx) > abs(dy))
        change_page(d, dx < 0 ? 1 : 0);
      else if (!d->moved) {
        if (!call_in_progress(&d->status) && point.y < PAGE_TOP && d->touch_start.y < PAGE_TOP)
          change_page(d, point.x >= d->canvas.bounds.size.w / 2);
        else if (d->touch_target >= 0 && target_at(d, point) == d->touch_target)
          activate(d, d->touch_target);
      }
    }
  }
  layer_mark_dirty(&d->canvas);
}

static void refresh_contacts(AppData *d) {
  memset(d->contacts, 0, sizeof(d->contacts));
  d->contact_count = phone_call_contacts_get(d->contacts, PHONE_MAX_CONTACTS);
  d->contact_revision = phone_call_contacts_test_revision();
  if (d->focused_contact >= d->contact_count)
    d->focused_contact = 0;
  clamp_scroll(d);
  d->touching = false;
  layer_mark_dirty(&d->canvas);
}

static void contacts_changed(PebbleEvent *event, void *context) {
  if (event->blob_db.db_id == BlobDBIdContacts || event->blob_db.db_id == BlobDBIdWatchAppPrefs)
    refresh_contacts(context);
}

static void appear(Window *window) {
  AppData *d = window_get_user_data(window);
  d->touching = false;
  refresh_contacts(d);
  touch_service_subscribe(touch, d);
}
static void disappear(Window *window) {
  AppData *d = window_get_user_data(window);
  d->touching = false;
  touch_service_unsubscribe();
}

static void tick(void *context) {
  AppData *d = context;
  HfpStatus status;
  hfp_get_status(&status);
  unsigned revision = phone_call_contacts_test_revision();
  if (revision != d->contact_revision)
    refresh_contacts(d);
  bool changed = memcmp(&status, &d->status, sizeof(status));
  if (changed) {
    if (d->call_menu_window &&
        (!status.ready || status.call != d->status.call || status.waiting != d->status.waiting ||
         status.call_held != d->status.call_held))
      action_menu_close(d->call_menu_window, false);
    if (status.errors != d->status.errors)
      d->notice = "Call unavailable";
    else if (status.call_setup || status.call || !status.ready)
      d->notice = NULL;
    if (!status.ready || status.incoming || status.call_held ||
        (!call_in_progress(&status) && call_in_progress(&d->status)))
      d->dialed_here = false;
    d->status = status;
    clamp_scroll(d);
    d->touching = false;
    layer_mark_dirty(&d->canvas);
  }
  d->timer = app_timer_register(250, tick, d);
}

static void prv_main(void) {
  AppData *d = app_malloc_check(sizeof(*d));
  memset(d, 0, sizeof(*d));
  app_state_set_user_data(d);
  hfp_get_status(&d->status);
  window_init(&d->window, WINDOW_NAME("Phone"));
  window_set_user_data(&d->window, d);
  window_set_fullscreen(&d->window, true);
  window_set_window_handlers(&d->window,
                             &(WindowHandlers){.appear = appear, .disappear = disappear});
  window_set_click_config_provider_with_context(&d->window, clicks, d);
  layer_init(&d->canvas, &d->window.layer.bounds);
  layer_set_update_proc(&d->canvas, draw);
  layer_add_child(&d->window.layer, &d->canvas);
  // Raw gestures own taps and paging; the system bridge must not also synthesize clicks.
  app_touch_navigation_enable(false);
  d->contacts_event = (EventServiceInfo){
    .type = PEBBLE_BLOBDB_EVENT,
    .handler = contacts_changed,
    .context = d,
  };
  event_service_client_subscribe(&d->contacts_event);
  app_window_stack_push(&d->window, true);
  d->timer = app_timer_register(250, tick, d);
  app_event_loop();
  app_timer_cancel(d->timer);
  event_service_client_unsubscribe(&d->contacts_event);
  touch_service_unsubscribe();
  layer_deinit(&d->canvas);
  window_deinit(&d->window);
  app_free(d);
}

const PebbleProcessMd *phone_get_app_info(void) {
  static const PebbleProcessMdSystem info = {.common.main_func = prv_main, .name = "Phone"};
  return (const PebbleProcessMd *)&info;
}

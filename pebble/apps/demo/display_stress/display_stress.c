/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "display_stress.h"

#include <stdio.h>
#include <string.h>

#include <pbl/kernel/time.h>
#include <pbl/util/math.h>
#include <pbl/util/size.h>

#include <applib/app.h>
#include <applib/app_malloc.h>
#include <applib/app_timer.h>
#include <applib/fonts/fonts.h>
#include <applib/graphics/graphics.h>
#include <applib/graphics/gtypes.h>
#include <applib/graphics/text.h>
#include <applib/platform.h>
#include <applib/ui/app_window_stack.h>
#include <applib/ui/window.h>
#include <process_state/app_state/app_state.h>

#define REDRAW_INTERVAL_MS 5U

enum phase {
  PHASE_MOVING,
  PHASE_STILL,
  PHASE_WHITE,
  PHASE_COUNT,
};

static const uint32_t s_phase_ms[PHASE_COUNT] = {
  [PHASE_MOVING] = 60000U,
  [PHASE_STILL] = 60000U,
  [PHASE_WHITE] = 10000U,
};

static const uint16_t s_cell_sizes[] = {2, 4, 10, 13, 26, 52, 130};
#define DEFAULT_CELL_SIZE_IDX 4U

typedef struct {
  Window window;
  AppTimer *redraw_timer;
  AppTimer *phase_timer;
  enum phase phase;
  bool show_fps;
  size_t cell_idx;
  uint32_t rng;
  uint32_t frame_seed;
  uint32_t frames;
  uint32_t fps_x10;
  uint64_t fps_start_ms;
  char fps_text[48];
} AppData;

static uint32_t prv_rand(AppData *data) {
  uint32_t x = data->rng;

  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  data->rng = x;

  return x;
}

static void prv_draw_grid(AppData *data, GContext *ctx) {
  GBitmap *fb = graphics_capture_frame_buffer(ctx);
  if (!fb) {
    return;
  }

  const GRect bounds = gbitmap_get_bounds(fb);
  const int16_t cell = s_cell_sizes[data->cell_idx];
  const int16_t cols = MIN((bounds.size.w + cell - 1) / cell, PBL_DISPLAY_WIDTH);
  uint8_t colors[PBL_DISPLAY_WIDTH];

  for (int16_t band = 0; band < bounds.size.h; band += cell) {
    for (int16_t c = 0; c < cols; c++) {
      colors[c] = 0xC0 | (prv_rand(data) & 0x3F);
    }

    const int16_t band_end = MIN(band + cell, bounds.size.h);
    for (int16_t y = band; y < band_end; y++) {
      const GBitmapDataRowInfo row = gbitmap_get_data_row_info(fb, y);
      const int16_t max_x = MIN(row.max_x, bounds.size.w - 1);

      for (int16_t c = 0; c < cols; c++) {
        const int16_t x0 = MAX(c * cell, row.min_x);
        const int16_t x1 = MIN((c + 1) * cell - 1, max_x);
        if (x0 <= x1) {
          memset(&row.data[x0], colors[c], (size_t)(x1 - x0 + 1));
        }
      }
    }
  }

  graphics_release_frame_buffer(ctx, fb);
}

static void prv_draw_fps(AppData *data, GContext *ctx, const GRect *bounds) {
  const GRect box = GRect(bounds->size.w / 2 - 50, bounds->size.h / 2 - 30, 100, 60);

  snprintf(data->fps_text, sizeof(data->fps_text), "%lu.%lu fps\n%u px",
           (unsigned long)(data->fps_x10 / 10U), (unsigned long)(data->fps_x10 % 10U),
           (unsigned)s_cell_sizes[data->cell_idx]);

  graphics_context_set_fill_color(ctx, GColorBlack);
  graphics_fill_rect(ctx, &box);
  graphics_context_set_text_color(ctx, GColorWhite);
  graphics_draw_text(ctx, data->fps_text, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD), box,
                     GTextOverflowModeWordWrap, GTextAlignmentCenter, nullptr);
}

static void prv_update_fps(AppData *data) {
  const uint64_t now = pbl_realtime_ms();
  const uint64_t elapsed = now - data->fps_start_ms;

  data->frames++;
  if (elapsed >= 1000U) {
    data->fps_x10 = (uint32_t)((data->frames * 10000U) / elapsed);
    data->frames = 0;
    data->fps_start_ms = now;
  }
}

static void prv_update_proc(Layer *layer, GContext *ctx) {
  AppData *data = app_state_get_user_data();

  if (data->phase == PHASE_WHITE) {
    graphics_context_set_fill_color(ctx, GColorWhite);
    graphics_fill_rect(ctx, &layer->bounds);
    return;
  }

  if (data->phase == PHASE_MOVING) {
    data->frame_seed = data->rng;
    prv_update_fps(data);
  } else {
    data->rng = data->frame_seed;
  }

  prv_draw_grid(data, ctx);

  if (data->show_fps) {
    prv_draw_fps(data, ctx, &layer->bounds);
  }
}

static void prv_redraw_cb(void *ctx) {
  AppData *data = ctx;

  layer_mark_dirty(window_get_root_layer(&data->window));
  data->redraw_timer = app_timer_register(REDRAW_INTERVAL_MS, prv_redraw_cb, data);
}

static void prv_phase_cb(void *ctx) {
  AppData *data = ctx;

  data->phase = (data->phase + 1) % PHASE_COUNT;
  if (data->phase == PHASE_MOVING) {
    data->frames = 0;
    data->fps_start_ms = pbl_realtime_ms();
    prv_redraw_cb(data);
  } else {
    app_timer_cancel(data->redraw_timer);
    data->redraw_timer = nullptr;
    if (data->phase == PHASE_WHITE) {
      layer_mark_dirty(window_get_root_layer(&data->window));
    }
  }

  data->phase_timer = app_timer_register(s_phase_ms[data->phase], prv_phase_cb, data);
}

static void prv_select_click(ClickRecognizerRef recognizer, void *ctx) {
  AppData *data = ctx;

  data->show_fps = !data->show_fps;
  layer_mark_dirty(window_get_root_layer(&data->window));
}

static void prv_up_click(ClickRecognizerRef recognizer, void *ctx) {
  AppData *data = ctx;

  if (data->cell_idx + 1U < ARRAY_LENGTH(s_cell_sizes)) {
    data->cell_idx++;
  }
}

static void prv_down_click(ClickRecognizerRef recognizer, void *ctx) {
  AppData *data = ctx;

  if (data->cell_idx > 0U) {
    data->cell_idx--;
  }
}

static void prv_click_config(void *ctx) {
  window_single_click_subscribe(BUTTON_ID_SELECT, prv_select_click);
  window_single_click_subscribe(BUTTON_ID_UP, prv_up_click);
  window_single_click_subscribe(BUTTON_ID_DOWN, prv_down_click);
}

static void s_main(void) {
  AppData *data = app_zalloc_check(sizeof(AppData));

  app_state_set_user_data(data);

  data->cell_idx = DEFAULT_CELL_SIZE_IDX;
  data->show_fps = true;
  data->fps_start_ms = pbl_realtime_ms();
  data->rng = (uint32_t)data->fps_start_ms | 1U;

  Window *window = &data->window;
  window_init(window, WINDOW_NAME("Display Stress"));
  window_set_fullscreen(window, true);
  window_set_click_config_provider_with_context(window, prv_click_config, data);
  layer_set_update_proc(window_get_root_layer(window), prv_update_proc);
  app_window_stack_push(window, false);

  prv_redraw_cb(data);
  data->phase_timer = app_timer_register(s_phase_ms[PHASE_MOVING], prv_phase_cb, data);

  app_event_loop();
}

const PebbleProcessMd *display_stress_get_app_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
    .common.uuid =
        {0xd6, 0x73, 0x48, 0x1e, 0x01, 0xd0, 0x49, 0x62, 0xa7, 0xf2, 0xd6, 0x29, 0x38, 0xd8, 0x9c,
         0x7f},
    .common.main_func = s_main,
    .name = "Display Stress",
  };
  return (const PebbleProcessMd *)&s_app_info;
}

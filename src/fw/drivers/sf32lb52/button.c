/* SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/button.h>

#include "board/board.h"
#include "console/prompt.h"
#include <pbl/drivers/gpio.h>
#include "kernel/events.h"
#include "system/passert.h"

static bool s_rotated_180 = false;

void button_set_rotated(bool rotated_180) {
  s_rotated_180 = rotated_180;
}

bool button_is_pressed(ButtonId id) {
  if (s_rotated_180 && (id == BUTTON_ID_UP)) {
    id = BUTTON_ID_DOWN;
  } else if (s_rotated_180 && (id == BUTTON_ID_DOWN)) {
    id = BUTTON_ID_UP;
  }

  return pbl_gpio_get(&BOARD_CONFIG_BUTTON.buttons[id].gpio) != 0;
}

uint8_t button_get_state_bits(void) {
  uint8_t button_state = 0x00;
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    button_state |= (button_is_pressed(i) ? 0x01 : 0x00) << i;
  }
  return button_state;
}

void button_init(void) {
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    pbl_gpio_configure(&BOARD_CONFIG_BUTTON.buttons[i].gpio, PBL_GPIO_INPUT);
  }
}

void command_button_read(const char* button_id_str) {
  int button = atoi(button_id_str);

  if (button < 0 || button >= NUM_BUTTONS) {
    prompt_send_response("Invalid button");
    return;
  }

  if (button_is_pressed(button)) {
    prompt_send_response("down");
  } else {
    prompt_send_response("up");
  }
}

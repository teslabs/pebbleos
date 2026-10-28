/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>

#include <pbl/drivers/flash.h>
#include <pbl/drivers/otp.h>

#define FLASH_ERASE_VAL 0xFFU
#define OTP_SLOT_SIZE   32U
#define SEC_REG_IDX     0x0U

static char s_slot[NUM_OTP_SLOTS][OTP_SLOT_SIZE];

static uint32_t prv_slot_addr(const uint8_t index) {
  return FLASH->sec_regs->addrs[SEC_REG_IDX] + index * OTP_SLOT_SIZE;
}

char *otp_get_slot(const uint8_t index) {
  if (index >= NUM_OTP_SLOTS || FLASH->sec_regs == nullptr) {
    return nullptr;
  }

  for (uint8_t i = 0U; i < OTP_SLOT_SIZE; i++) {
    if (pbl_flash_sec_reg_read(FLASH, prv_slot_addr(index) + i, (uint8_t *)&s_slot[index][i]) !=
        0) {
      return nullptr;
    }
  }

  return s_slot[index];
}

uint8_t *otp_get_lock(const uint8_t index) {
  return nullptr;
}

bool otp_is_locked(const uint8_t index) {
  bool locked;

  if (FLASH->sec_regs == nullptr) {
    return false;
  }

  if (pbl_flash_sec_reg_is_locked(FLASH, FLASH->sec_regs->addrs[SEC_REG_IDX], &locked) != 0) {
    return false;
  }

  return locked;
}

OtpWriteResult otp_write_slot(const uint8_t index, const char *value) {
  char *existing_val;
  size_t len;

  if (index >= NUM_OTP_SLOTS || FLASH->sec_regs == nullptr) {
    return OtpWriteFailCorrupt;
  }

  len = strlen(value);
  if (len >= OTP_SLOT_SIZE) {
    return OtpWriteFailCorrupt;
  }

  existing_val = otp_get_slot(index);
  if (existing_val == nullptr) {
    return OtpWriteFailCorrupt;
  }

  for (size_t i = 0U; i < OTP_SLOT_SIZE; i++) {
    if ((uint8_t)existing_val[i] != FLASH_ERASE_VAL) {
      return OtpWriteFailAlreadyWritten;
    }
  }

  for (size_t i = 0U; i <= len; i++) {
    if (pbl_flash_sec_reg_write(FLASH, prv_slot_addr(index) + i, (uint8_t)value[i]) != 0) {
      return OtpWriteFailCorrupt;
    }
  }

  existing_val = otp_get_slot(index);
  if ((existing_val == nullptr) || (memcmp(existing_val, value, len + 1) != 0)) {
    return OtpWriteFailCorrupt;
  }

  return OtpWriteSuccess;
}

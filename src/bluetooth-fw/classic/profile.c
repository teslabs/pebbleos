/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#include "internal.h"

#include <stdio.h>
#include <string.h>

static uint8_t fcs(const uint8_t *p, unsigned n) {
  unsigned crc = 0xff;
  while (n--) {
    crc ^= *p++;
    for (unsigned i = 0; i < 8; ++i)
      crc = (crc >> 1) ^ ((crc & 1) ? 0xe0 : 0);
  }
  return 0xff - crc;
}

static void frame(BtClassicHost *s, unsigned dlci, unsigned control, bool response,
                  const uint8_t *data, unsigned length, unsigned credits) {
  uint8_t p[140];
  if (length > 127 || !s->rfcomm_cid)
    return;
  p[0] = (dlci << 2) | (response ? 3 : 1);
  p[1] = control | (credits ? 0x10 : 0);
  p[2] = (length << 1) | 1;
  unsigned offset = 3;
  if (credits)
    p[offset++] = credits;
  if (length)
    memcpy(p + offset, data, length);
  offset += length;
  p[offset++] = fcs(p, (control & ~0x10) == 0xef ? 2 : 3);
  bt_classic_l2cap_send(s, s->rfcomm_cid, p, offset);
}

static void mcc(BtClassicHost *s, uint8_t type, const uint8_t *data, unsigned length) {
  uint8_t p[32] = {type, (length << 1) | 1};
  if (length > sizeof(p) - 2)
    return;
  memcpy(p + 2, data, length);
  frame(s, 0, 0xef, false, p, length + 2, 0);
}

static void at_command(BtClassicHost *s, const char *text) {
  unsigned length = strlen(text);
  if (!s->rfcomm_open || s->at_pending || length >= sizeof(s->at_tx))
    return;
  memcpy(s->at_tx, text, length);
  s->at_tx_length = length;
  s->at_pending = true;
  s->status.busy = true;
  s->profile_deadline = s->now + 10000;
}

static void slc_next(BtClassicHost *s) {
  static const char *const commands[] = {
    "AT+BRSF=0\r", "AT+CIND=?\r", "AT+CIND?\r", "AT+CMER=3,0,0,1\r"
  };
  if (s->slc_step < 4)
    at_command(s, commands[s->slc_step++]);
  else {
    s->status.ready = true;
    snprintf(s->status.detail, sizeof(s->status.detail), "Ready to call");
  }
}

static void indicator(BtClassicHost *s, unsigned index, unsigned value) {
  if (index == s->indicator_call && value <= 1)
    s->status.call = value;
  if (index == s->indicator_setup && value <= 3) {
    s->status.call_setup = value;
    s->status.incoming = value == 1;
  }
}

static void line(BtClassicHost *s, const char *text) {
  if (!strcmp(text, "OK")) {
    if (!s->at_pending)
      return;
    s->at_pending = s->status.busy = false;
    if (!s->status.ready)
      slc_next(s);
  } else if (!strcmp(text, "ERROR") || !strncmp(text, "+CME ERROR", 10)) {
    s->at_pending = s->status.busy = false;
    bt_classic_error(s, "Phone rejected command");
  } else if (!strcmp(text, "RING"))
    s->status.incoming = true;
  else if (!strncmp(text, "+CIEV:", 6)) {
    unsigned index, value;
    if (sscanf(text + 6, " %u , %u", &index, &value) == 2)
      indicator(s, index, value);
  } else if (!strncmp(text, "+CIND:", 6)) {
    if (s->slc_step == 2) {
      unsigned index = 0;
      const char *p = text + 6;
      while ((p = strchr(p, '"'))) {
        const char *end = strchr(++p, '"');
        if (!end)
          break;
        ++index;
        if (end - p == 4 && !memcmp(p, "call", 4))
          s->indicator_call = index;
        if (end - p == 9 && !memcmp(p, "callsetup", 9))
          s->indicator_setup = index;
        p = end + 1;
      }
    } else if (s->slc_step == 3) {
      const char *p = text + 6;
      unsigned index = 1;
      while (*p) {
        while (*p == ' ')
          ++p;
        if (*p < '0' || *p > '9')
          break;
        indicator(s, index++, *p++ - '0');
        while (*p == ' ')
          ++p;
        if (*p++ != ',')
          break;
      }
    }
  }
}

void bt_classic_profile_reset(BtClassicHost *s) {
  s->rfcomm_cid = s->rfcomm_mtu = s->rfcomm_credits = 0;
  s->dlci = s->rx_credits = s->slc_step = 0;
  s->rfcomm_open = s->credit_mode = s->modem_ready = s->at_pending = false;
  s->status.ready = s->status.busy = false;
  s->at_tx_length = s->at_line_length = 0;
  s->indicator_call = s->indicator_setup = 0;
}

void bt_classic_profile_poll(BtClassicHost *s) {
  if (!s->rfcomm_open)
    return;
  if (s->at_pending && (int32_t)(s->now - s->profile_deadline) >= 0) {
    s->at_pending = s->status.busy = false;
    s->at_tx_length = 0;
    bt_classic_error(s, "Phone command timed out");
  }
  if (s->at_tx_length && (!s->credit_mode || s->rfcomm_credits)) {
    unsigned length = s->at_tx_length;
    if (length > s->rfcomm_mtu)
      length = s->rfcomm_mtu;
    frame(s, s->dlci, 0xef, false, s->at_tx, length, 0);
    memmove(s->at_tx, s->at_tx + length, s->at_tx_length - length);
    s->at_tx_length -= length;
    if (s->credit_mode)
      --s->rfcomm_credits;
  }
}

void bt_classic_rfcomm(BtClassicHost *s, BtClassicChannel *ch, const uint8_t *p, size_t n) {
  if (n < 4 || !(p[0] & 1))
    return;
  unsigned dlci = p[0] >> 2, control = p[1] & ~0x10, length = p[2] >> 1, offset = 3;
  if (!(p[2] & 1)) {
    if (n < 5)
      return;
    length |= (unsigned)p[3] << 7;
    ++offset;
  }
  bool credit = control == 0xef && dlci && s->credit_mode && (p[1] & 0x10);
  if (n != offset + length + 1 + credit || p[n - 1] != fcs(p, control == 0xef ? 2 : offset)) {
    bt_classic_error(s, "Invalid RFCOMM frame");
    return;
  }
  if (s->rfcomm_cid && ch->remote != s->rfcomm_cid)
    return;
  s->rfcomm_cid = ch->remote;
  if (control == 0x2f) {
    if (dlci == 0 || (dlci == s->dlci && s->rfcomm_mtu)) {
      frame(s, dlci, 0x73, true, NULL, 0, 0);
      if (dlci) {
        s->rfcomm_open = true;
        uint8_t modem[] = {(dlci << 2) | 3, 0x8d};
        mcc(s, 0xe3, modem, sizeof(modem));
        slc_next(s);
      }
    } else
      frame(s, dlci, 0x1f, true, NULL, 0, 0);
    return;
  }
  if (control == 0x43) {
    frame(s, dlci, 0x73, true, NULL, 0, 0);
    if (!dlci || dlci == s->dlci)
      bt_classic_profile_reset(s);
    return;
  }
  if (control != 0xef)
    return;
  if (!dlci) {
    if (length < 2)
      return;
    const uint8_t *m = p + offset;
    unsigned count = m[1] >> 1;
    if (!(m[1] & 1) || count + 2 != length)
      return;
    if (!(m[0] & 2)) {
      if ((m[0] & 0xfc) == 0xe0)
        s->modem_ready = true;
      return;
    }
    const uint8_t *value = m + 2;
    if ((m[0] & 0xfc) == 0x80 && count == 8) {
      if (value[0] != 2 || s->rfcomm_open) {
        frame(s, value[0], 0x1f, true, NULL, 0, 0);
        return;
      }
      uint8_t answer[8];
      memcpy(answer, value, 8);
      unsigned mtu = value[4] | (unsigned)value[5] << 8;
      if (!mtu || ch->mtu < 16)
        return;
      if (mtu > 127)
        mtu = 127;
      if (mtu > ch->mtu - 5u)
        mtu = ch->mtu - 5;
      s->dlci = value[0];
      s->rfcomm_mtu = mtu;
      s->credit_mode = value[1] == 0xf0;
      s->rfcomm_credits = s->credit_mode ? value[7] & 7 : 0;
      s->rx_credits = 7;
      answer[1] = s->credit_mode ? 0xe0 : 0;
      answer[4] = mtu;
      answer[5] = 0;
      answer[7] = s->credit_mode ? 7 : 0;
      mcc(s, 0x81, answer, 8);
    } else if ((m[0] & 0xfc) == 0xe0 && count >= 2 && (value[0] >> 2) == s->dlci) {
      mcc(s, 0xe1, value, count);
      s->modem_ready = true;
    } else if ((m[0] & 0xfc) == 0x20) {
      mcc(s, m[0] & ~2, value, count); // Test command.
    } else {
      uint8_t rejected = m[0];
      mcc(s, 0x11, &rejected, 1);
    }
    return;
  }
  if (dlci != s->dlci || !s->rfcomm_open)
    return;
  if (credit) {
    unsigned add = p[offset++];
    s->rfcomm_credits = s->rfcomm_credits + add > 255 ? 255 : s->rfcomm_credits + add;
  }
  if (length && s->credit_mode) {
    if (!s->rx_credits) {
      bt_classic_error(s, "RFCOMM peer exceeded credits");
      return;
    }
    --s->rx_credits;
  }
  for (unsigned i = 0; i < length; ++i) {
    char c = p[offset + i];
    if (c == '\r' || c == '\n') {
      if (s->at_line_length) {
        s->at_line[s->at_line_length] = 0;
        line(s, s->at_line);
        s->at_line_length = 0;
      }
    } else if (s->at_line_length + 1 < sizeof(s->at_line))
      s->at_line[s->at_line_length++] = c;
    else {
      s->at_line_length = 0;
      bt_classic_error(s, "HFP response too long");
    }
  }
  if (s->credit_mode && s->rx_credits <= 3) {
    frame(s, s->dlci, 0xef, false, NULL, 0, 7 - s->rx_credits);
    s->rx_credits = 7;
  }
}

bool bt_classic_valid_number(const char *number) {
  if (!number || !*number)
    return false;
  unsigned digits = 0;
  for (unsigned i = 0; number[i]; ++i) {
    if (i >= BT_CLASSIC_NUMBER_SIZE - 1)
      return false;
    char c = number[i];
    if (c >= '0' && c <= '9')
      ++digits;
    else if (c != '*' && c != '#' && !(c == '+' && i == 0))
      return false;
  }
  return digits != 0;
}

bool bt_classic_dial(BtClassicHost *s, const char *number) {
  if (!s->status.ready || s->at_pending || s->status.call || s->status.call_setup ||
      s->status.incoming || !bt_classic_valid_number(number))
    return false;
  char command[40];
  snprintf(command, sizeof(command), "ATD%s;\r", number);
  at_command(s, command);
  snprintf(s->status.detail, sizeof(s->status.detail), "Dial requested");
  return true;
}

bool bt_classic_answer(BtClassicHost *s) {
  if (!s->status.ready || s->at_pending || !s->status.incoming)
    return false;
  at_command(s, "ATA\r");
  return true;
}

bool bt_classic_hangup(BtClassicHost *s) {
  if (!s->status.ready || s->at_pending ||
      !(s->status.call || s->status.call_setup || s->status.incoming))
    return false;
  at_command(s, "AT+CHUP\r");
  return true;
}

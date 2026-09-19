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

static bool frame(BtClassicHost *s, unsigned dlci, unsigned control, bool response,
                  const uint8_t *data, unsigned length, unsigned credits) {
  uint8_t p[140];
  if (length > 127 || !s->rfcomm_cid)
    return false;
  p[0] = (dlci << 2) | ((response != s->rfcomm_initiator) ? 3 : 1);
  p[1] = control | (credits ? 0x10 : 0);
  p[2] = (length << 1) | 1;
  unsigned offset = 3;
  if (credits)
    p[offset++] = credits;
  if (length)
    memcpy(p + offset, data, length);
  offset += length;
  p[offset++] = fcs(p, (control & ~0x10) == 0xef ? 2 : 3);
  return bt_classic_l2cap_send(s, s->rfcomm_cid, p, offset);
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
  static const char *const commands[] = {"AT+BRSF=54\r",      "AT+CIND=?\r", "AT+CIND?\r",
                                         "AT+CMER=3,0,0,1\r", "AT+CHLD=?\r", "AT+CLIP=1\r",
                                         "AT+CCWA=1\r"};
  while (s->slc_step < sizeof(commands) / sizeof(commands[0])) {
    unsigned step = s->slc_step++;
    if ((step == 4 || step == 6) && !(s->ag_features & 1))
      continue;
    at_command(s, commands[step]);
    return;
  }
  if ((s->ag_features & 1) && (!s->indicator_held || (s->status.hold_support & 6) != 6)) {
    bt_classic_error(s, "Missing call hold capabilities");
    bt_classic_disconnect_peer(s);
    return;
  }
  s->status.ready = true;
  s->speaker_gain_dirty = true;
  s->speaker_gain_next = s->status.speaker_gain;
  s->calls_dirty = true;
  snprintf(s->status.detail, sizeof(s->status.detail), "Ready to call");
}

void bt_classic_channel_ready(BtClassicHost *s, BtClassicChannel *ch) {
  if (s->revoking)
    return;
  if (ch->psm == 1 && ch->outgoing && s->connect_stage == BtClassicConnectSdp) {
    bt_classic_sdp_start(s, ch);
  } else if (ch->psm == 3) {
    if (ch->outgoing && s->connect_stage == BtClassicConnectRfcomm) {
      s->rfcomm_initiator = true;
      s->rfcomm_cid = ch->remote;
      s->dlci = s->server_channel << 1;
      s->connect_stage = BtClassicConnectMux;
      frame(s, 0, 0x3f, false, NULL, 0, 0);
    } else if (!ch->outgoing && s->connect_stage != BtClassicConnectIdle) {
      // The phone won the race while our SDP discovery was in progress.
      s->connect_stage = BtClassicConnectSlc;
    }
  }
}

static void indicator(BtClassicHost *s, unsigned index, unsigned value) {
  if (!index)
    return;
  if ((index == s->indicator_call && value <= 1) || (index == s->indicator_setup && value <= 3) ||
      (index == s->indicator_held && value <= 2))
    s->calls_dirty = true;
  if (index == s->indicator_call && value <= 1)
    s->status.call = value;
  if (index == s->indicator_setup && value <= 3) {
    s->status.call_setup = value;
    s->status.incoming = value == 1;
    s->status.waiting = value == 1 && s->status.call;
    if (value != 1)
      s->status.waiting_number[0] = 0;
  }
  if (index == s->indicator_held && value <= 2) {
    s->status.call_held = value;
    // An exchange may leave callheld unchanged; do not retain the old caller identity.
    s->status.caller_number[0] = 0;
  }
  if (index == s->indicator_call && value == 0) {
    s->status.waiting = false;
    s->status.call_held = 0;
    s->status.waiting_number[0] = 0;
  }
  if ((index == s->indicator_call || index == s->indicator_setup) && !s->status.call &&
      !s->status.call_setup)
    s->status.caller_number[0] = 0;
}

static bool caller_id(char destination[BT_CLASSIC_NUMBER_SIZE], const char *text,
                      unsigned validity_field) {
  while (*text == ' ')
    ++text;
  if (*text++ != '"')
    return false;
  const char *end = strchr(text, '"');
  if (!end)
    return false;
  const char *p = end + 1;
  while (*p == ' ')
    ++p;
  if (*p++ != ',')
    return false;
  while (*p == ' ')
    ++p;
  if (*p < '0' || *p > '9')
    return false;
  unsigned type = 0;
  while (*p >= '0' && *p <= '9') {
    type = type * 10 + *p++ - '0';
    if (type > 255)
      return false;
  }
  while (*p == ' ')
    ++p;
  if (*p && *p != ',')
    return false;
  // Honor an optional CLI validity field; do not display withheld identities.
  bool withheld = false;
  bool voice_class = validity_field != 4;
  for (unsigned field = 2; *p == ',' && field <= validity_field; ++field) {
    ++p;
    while (*p == ' ')
      ++p;
    const char *start = p;
    if (*p == '"') {
      p = strchr(p + 1, '"');
      if (!p)
        return false;
      ++p;
    } else {
      while (*p && *p != ',')
        ++p;
    }
    const char *stop = p;
    while (stop > start && stop[-1] == ' ')
      --stop;
    if (validity_field == 4 && field == 2) {
      if (stop - start != 1 || *start != '1')
        return false;
      voice_class = true;
    }
    if (field == validity_field && stop != start) {
      if (stop - start != 1 || *start < '0' || *start > '2')
        return false;
      withheld = *start != '0';
    }
    while (*p == ' ')
      ++p;
    if (*p && *p != ',')
      return false;
  }
  char number[BT_CLASSIC_NUMBER_SIZE] = {0};
  if (!voice_class)
    return false;
  unsigned prefix = type == 145 && *text != '+' && end != text;
  size_t length = end - text;
  if (length + prefix >= sizeof(number))
    return false;
  if (prefix)
    number[0] = '+';
  memcpy(number + prefix, text, length);
  if (length && !bt_classic_valid_number(number))
    return false;
  if (withheld)
    memset(number, 0, sizeof(number));
  memcpy(destination, number, sizeof(number));
  return true;
}

static void speaker_gain(BtClassicHost *s, const char *text) {
  while (*text == ' ')
    ++text;
  if (*text < '0' || *text > '9')
    return;
  unsigned gain = 0;
  do {
    gain = gain * 10 + *text++ - '0';
    if (gain > 15)
      return;
  } while (*text >= '0' && *text <= '9');
  while (*text == ' ')
    ++text;
  if (*text)
    return;
  s->status.speaker_gain = gain;
}

static void supported_hold(BtClassicHost *s, const char *text) {
  if (s->slc_step != 5 || s->status.ready)
    return;
  while (*text == ' ')
    ++text;
  if (*text++ != '(')
    return;
  unsigned mask = 0;
  for (unsigned count = 0; count < 16; ++count) {
    while (*text == ' ')
      ++text;
    if (*text < '0' || *text > '4')
      return;
    unsigned action = *text++ - '0';
    if (*text == 'x' && (action == 1 || action == 2))
      ++text;
    else
      mask |= 1u << action;
    while (*text == ' ')
      ++text;
    if (*text == ')') {
      ++text;
      while (*text == ' ')
        ++text;
      if (!*text)
        s->status.hold_support = mask & 15;
      return;
    }
    if (*text++ != ',')
      return;
  }
}

static bool unsigned_field(const char **text, unsigned maximum, unsigned *value) {
  const char *p = *text;
  while (*p == ' ')
    ++p;
  if (*p < '0' || *p > '9')
    return false;
  unsigned result = 0;
  do {
    unsigned digit = *p++ - '0';
    if (digit > maximum || result > (maximum - digit) / 10)
      return false;
    result = result * 10 + digit;
  } while (*p >= '0' && *p <= '9');
  while (*p == ' ')
    ++p;
  *text = p;
  *value = result;
  return true;
}

static void current_call(BtClassicHost *s, const char *p) {
  if (!s->calls_query || !s->at_pending || s->at_tx_length)
    return;
  unsigned fields[5];
  const unsigned limits[] = {255, 1, 6, 2, 1};
  for (unsigned i = 0; i < 5; ++i) {
    if (!unsigned_field(&p, limits[i], &fields[i]) || (i < 4 && *p++ != ',')) {
      s->calls_invalid = true;
      return;
    }
  }
  BtClassicCall call = {.index = fields[0], .state = fields[2]};
  if (!call.index || (*p && (*p++ != ',' || !caller_id(call.number, p, 255)))) {
    s->calls_invalid = true;
    return;
  }
  if (fields[3] != 0)
    return; // Only voice calls participate in the watch's call display.
  for (unsigned i = 0; i < s->call_count; ++i) {
    if (s->calls[i].index == call.index) {
      s->calls_invalid = true;
      return;
    }
  }
  if (s->call_count == BT_CLASSIC_MAX_CALLS) {
    s->calls_invalid = true;
    return;
  }
  s->calls[s->call_count++] = call;
}

static void publish_calls(BtClassicHost *s) {
  if (s->calls_invalid || s->calls_dirty)
    return;
  const BtClassicCall *display = NULL, *waiting = NULL;
  unsigned priority = 0;
  for (unsigned i = 0; i < s->call_count; ++i) {
    const BtClassicCall *call = &s->calls[i];
    if (call->state == 5) {
      waiting = call;
      continue;
    }
    unsigned next = call->state == 0 ? 3 : call->state == 1 ? 1 : 2;
    if (next > priority) {
      display = call;
      priority = next;
    }
  }
  snprintf(s->status.caller_number, sizeof(s->status.caller_number), "%s",
           display ? display->number : "");
  if (s->status.waiting)
    snprintf(s->status.waiting_number, sizeof(s->status.waiting_number), "%s",
             waiting ? waiting->number : "");
}

static void line(BtClassicHost *s, const char *text) {
  if (!strcmp(text, "OK")) {
    if (!s->at_pending || s->at_tx_length)
      return;
    if (s->slc_step == 2 && (!s->indicator_call || !s->indicator_setup)) {
      bt_classic_error(s, "Missing call indicators");
      bt_classic_disconnect_peer(s);
      return;
    }
    s->at_pending = s->status.busy = false;
    if (s->calls_query) {
      publish_calls(s);
      s->calls_query = false;
    }
    if (s->calls_after_ack) {
      s->calls_dirty = true;
      s->calls_after_ack = false;
    }
    if (!s->status.ready)
      slc_next(s);
  } else if (!strcmp(text, "ERROR") || !strncmp(text, "+CME ERROR", 10)) {
    s->at_pending = s->status.busy = false;
    s->calls_query = s->calls_after_ack = false;
    bt_classic_error(s, "Phone rejected command");
    if (!s->status.ready)
      bt_classic_disconnect_peer(s);
  } else if (!strcmp(text, "RING"))
    s->status.incoming = true;
  else if (!strncmp(text, "+CLIP:", 6)) {
    caller_id(s->status.caller_number, text + 6, 5);
  } else if (!strncmp(text, "+CCWA:", 6) && s->status.call && (s->ag_features & 1)) {
    if (caller_id(s->status.waiting_number, text + 6, 4)) {
      s->status.waiting = true;
      s->calls_dirty = true;
    }
  } else if (!strncmp(text, "+CLCC:", 6)) {
    current_call(s, text + 6);
  } else if (!strncmp(text, "+CHLD:", 6)) {
    supported_hold(s, text + 6);
  } else if (!strncmp(text, "+BRSF:", 6) && s->slc_step == 1) {
    unsigned features;
    const char *p = text + 6;
    if (unsigned_field(&p, UINT32_MAX, &features) && !*p)
      s->ag_features = features;
  } else if (!strncmp(text, "+VGS:", 5)) {
    speaker_gain(s, text + 5);
  } else if (!strncmp(text, "+CIEV:", 6)) {
    unsigned index, value;
    const char *p = text + 6;
    if (unsigned_field(&p, 255, &index) && *p++ == ',' && unsigned_field(&p, 3, &value) && !*p)
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
        if (end - p == 8 && !memcmp(p, "callheld", 8))
          s->indicator_held = index;
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
  s->rfcomm_initiator = false;
  s->at_discard = false;
  s->status.ready = s->status.busy = false;
  s->status.audio_pending = false;
  s->status.call = s->status.incoming = false;
  s->status.call_setup = s->status.call_held = 0;
  s->status.waiting = false;
  s->status.hold_support = 0;
  s->status.waiting_number[0] = 0;
  s->ag_features = 0;
  s->calls_dirty = s->calls_query = s->calls_invalid = s->calls_after_ack = false;
  s->call_count = 0;
  s->status.caller_number[0] = 0;
  s->at_tx_length = s->at_line_length = 0;
  s->indicator_call = s->indicator_setup = s->indicator_held = 0;
  s->speaker_gain_dirty = false;
}

void bt_classic_profile_poll(BtClassicHost *s) {
  if (!s->rfcomm_open)
    return;
  if (s->at_pending && (int32_t)(s->now - s->profile_deadline) >= 0) {
    s->at_pending = s->status.busy = false;
    s->at_tx_length = 0;
    bt_classic_error(s, "Phone command timed out");
    // A late response cannot be associated safely with another AT command.
    bt_classic_disconnect_peer(s);
    return;
  }
  if (s->status.ready && !s->at_pending && s->speaker_gain_dirty) {
    s->status.speaker_gain = s->speaker_gain_next;
    char command[16];
    snprintf(command, sizeof(command), "AT+VGS=%u\r", s->status.speaker_gain);
    at_command(s, command);
    s->speaker_gain_dirty = false;
  }
  if (s->status.ready && !s->at_pending && s->calls_dirty && (s->ag_features & (1u << 6))) {
    s->calls_dirty = s->calls_invalid = false;
    s->calls_query = true;
    s->call_count = 0;
    at_command(s, "AT+CLCC\r");
  }
  if (s->at_tx_length && (!s->credit_mode || s->rfcomm_credits)) {
    unsigned length = s->at_tx_length;
    if (length > s->rfcomm_mtu)
      length = s->rfcomm_mtu;
    if (!frame(s, s->dlci, 0xef, false, s->at_tx, length, 0))
      return;
    memmove(s->at_tx, s->at_tx + length, s->at_tx_length - length);
    s->at_tx_length -= length;
    if (s->credit_mode)
      --s->rfcomm_credits;
  }
}

void bt_classic_rfcomm(BtClassicHost *s, BtClassicChannel *ch, const uint8_t *p, size_t n) {
  if (s->revoking || n < 4 || !(p[0] & 1))
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
  if (control == 0x63 && s->rfcomm_initiator) { // UA.
    if (!dlci && s->connect_stage == BtClassicConnectMux) {
      unsigned mtu = ch->mtu > 132 ? 127 : ch->mtu - 5;
      uint8_t pn[] = {s->dlci, 0xf0, 7, 0, mtu, 0, 0, 7};
      s->rfcomm_mtu = mtu;
      s->rx_credits = 7;
      s->connect_stage = BtClassicConnectPn;
      mcc(s, 0x83, pn, sizeof(pn));
    } else if (dlci == s->dlci && s->connect_stage == BtClassicConnectDlc) {
      s->connect_stage = BtClassicConnectSlc;
      s->rfcomm_open = true;
      uint8_t modem[] = {(dlci << 2) | 3, 0x8d};
      mcc(s, 0xe3, modem, sizeof(modem));
      slc_next(s);
    }
    return;
  }
  if (control == 0x0f && s->rfcomm_initiator) { // DM.
    bt_classic_disconnect_peer(s);
    return;
  }
  if (control == 0x2f) {
    if (dlci == 0 || (dlci == s->dlci && s->rfcomm_mtu)) {
      if (!frame(s, dlci, 0x73, true, NULL, 0, 0))
        return;
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
    if (!dlci || dlci == s->dlci) {
      s->reconnect_suppressed |= s->status.ready;
      bt_classic_profile_reset(s);
    }
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
      const uint8_t *value = m + 2;
      if (m[0] == 0x81 && count == 8 && s->connect_stage == BtClassicConnectPn &&
          value[0] == s->dlci) {
        unsigned mtu = value[4] | (unsigned)value[5] << 8;
        if (!mtu || mtu > s->rfcomm_mtu || (value[1] != 0xe0 && value[1] != 0)) {
          bt_classic_disconnect_peer(s);
          return;
        }
        s->rfcomm_mtu = mtu;
        s->credit_mode = value[1] == 0xe0;
        s->rfcomm_credits = s->credit_mode ? value[7] & 7 : 0;
        s->connect_stage = BtClassicConnectDlc;
        frame(s, s->dlci, 0x3f, false, NULL, 0, 0);
      }
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
      if (s->at_line_length && !s->at_discard) {
        s->at_line[s->at_line_length] = 0;
        line(s, s->at_line);
        if (s->revoking)
          return;
      }
      s->at_line_length = 0;
      s->at_discard = false;
    } else if (s->at_discard) {
      continue;
    } else if ((c == '\t' || (uint8_t)c >= 32) && c != 127 &&
               s->at_line_length + 1 < sizeof(s->at_line))
      s->at_line[s->at_line_length++] = c;
    else {
      s->at_line_length = 0;
      s->at_discard = true;
      bt_classic_error(s, "Invalid HFP response");
    }
  }
  if (s->credit_mode && s->rx_credits <= 3) {
    if (frame(s, s->dlci, 0xef, false, NULL, 0, 7 - s->rx_credits))
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
  s->calls_after_ack = true;
  snprintf(s->status.detail, sizeof(s->status.detail), "Dial requested");
  return true;
}

bool bt_classic_answer(BtClassicHost *s) {
  if (s->status.waiting)
    return bt_classic_call_hold(s, 2);
  if (!s->status.ready || s->at_pending || !s->status.incoming)
    return false;
  at_command(s, "ATA\r");
  s->calls_after_ack = true;
  return true;
}

bool bt_classic_hangup(BtClassicHost *s) {
  if (!s->status.ready || s->at_pending ||
      !(s->status.call || s->status.call_setup || s->status.incoming))
    return false;
  at_command(s, "AT+CHUP\r");
  s->calls_after_ack = true;
  return true;
}

bool bt_classic_set_speaker_gain(BtClassicHost *s, unsigned gain) {
  if (!s->status.ready || gain > 15)
    return false;
  if (gain != s->status.speaker_gain || s->speaker_gain_dirty) {
    s->speaker_gain_next = gain;
    s->status.speaker_gain = gain;
    s->speaker_gain_dirty = true;
  }
  return true;
}

bool bt_classic_call_hold(BtClassicHost *s, unsigned action) {
  if (!s->status.ready || s->at_pending || action > 3 || !(s->status.hold_support & (1u << action)))
    return false;
  if (action == 3 ? s->status.call_held != 1 || s->status.waiting
                  : !s->status.waiting && !s->status.call_held && !(action == 2 && s->status.call))
    return false;
  char command[16];
  snprintf(command, sizeof(command), "AT+CHLD=%u\r", action);
  at_command(s, command);
  s->calls_after_ack = true;
  return true;
}

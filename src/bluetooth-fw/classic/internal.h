/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "host.h"

// Internal protocol boundaries, still independent of controller and OS APIs.
void bt_classic_error(BtClassicHost *host, const char *reason);
void bt_classic_disconnect_peer(BtClassicHost *host);
bool bt_classic_l2cap_send(BtClassicHost *host, uint16_t cid, const uint8_t *data, size_t size);
void bt_classic_sdp(BtClassicHost *host, BtClassicChannel *channel, const uint8_t *data,
                    size_t size);
void bt_classic_rfcomm(BtClassicHost *host, BtClassicChannel *channel, const uint8_t *data,
                       size_t size);
void bt_classic_profile_poll(BtClassicHost *host);
void bt_classic_profile_reset(BtClassicHost *host);

void bt_classic_l2cap_receive(BtClassicHost *host, const uint8_t *packet, unsigned length);

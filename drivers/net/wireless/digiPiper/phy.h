/*
 * Linux device driver for Digi's WiWave WLAN card
 *
 * Copyright © 2008  Digi International, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef phy_h_
#define phy_h_

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/module.h>
#include <net/mac80211.h>


#include "pipermain.h"
#include "mac.h"

void phy_set_plcp(unsigned char *frame, unsigned length, struct ieee80211_rate *rate,
                  const struct ieee80211_rate *max_rate, int aes_len);
void phy_process_plcp(struct piper_priv *piper, struct rx_frame_hdr *hdr,
		struct ieee80211_rx_status *status, unsigned int *length);
unsigned int phy_determine_rssi(struct rx_frame_hdr *hdr);

#endif

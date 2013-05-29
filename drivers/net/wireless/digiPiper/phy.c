/*
 * Linux device driver for Digi's WiWave WLAN card
 *
 * Copyright © 2008  Digi International, Inc
 *
 * Author: Andres Salomon <dilinger@debian.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/module.h>
#include <net/mac80211.h>

#include "pipermain.h"
#include "mac.h"
#include "airoha.h"
#include "phy.h"

#define PHY_DEBUG	(0)

#if PHY_DEBUG
static int dlevel = DVERBOSE;
#define dprintk(level, fmt, arg...)	if (level >= dlevel)			\
					printk(KERN_ERR PIPER_DRIVER_NAME	\
					    ": %s - " fmt, __func__, ##arg)
#else
#define dprintk(level, fmt, arg...)	do {} while (0)
#endif

#define NUMBER_OF_WORD32(x)     ((x + 3) >> 2)

static int is_ofdm_rate(int rate)
{
	return (rate % 3) == 0;
}

void phy_set_plcp(unsigned char *frame, unsigned length, struct ieee80211_rate *rate,
                  const struct ieee80211_rate *max_rate, int aes_len)
{
	int ofdm = is_ofdm_rate(rate->bitrate);
	int plcp_len = length + FCS_LEN + aes_len;
	struct tx_frame_hdr *hdr;

	if (ofdm) {
		/* OFDM header */
		struct ofdm_hdr *ofdm;

		ofdm = (struct ofdm_hdr *) &frame[sizeof(struct tx_frame_hdr)];
		memset(ofdm, 0, sizeof(*ofdm));
		if (rate->bitrate > max_rate->bitrate) {
		    /*
		     * If the hardware cannot support the selected rate, then
		     * set the highest rate that it can support.
		     */
		    ofdm->rate = max_rate->hw_value;
		} else {
		    ofdm->rate = rate ->hw_value;
		}
		ofdm->length = cpu_to_le16(plcp_len);
	} else {
		/* PSK/CCK header */
		struct psk_cck_hdr *pskcck;
		int us_len;

		pskcck = (struct psk_cck_hdr *) &frame[sizeof(struct tx_frame_hdr)];
		pskcck->signal = rate->bitrate;
		pskcck->service = PLCP_SERVICE_LOCKED;

		/* convert length from bytes to usecs */
		switch (rate->bitrate) {
		case 10:
			us_len = plcp_len * 8;
			break;
		case 20:
			us_len = plcp_len * 4;
			break;
		case 55:
			us_len = (16 * plcp_len + 10) / 11;
			break;
		case 110:
			us_len = (8 * plcp_len + 10) / 11;

			/* set length extension bit if needed */
			dprintk(DALL, "us_len = %d, plcp_len = %d, (11 * us_len) = %d, \
 				(11 * us_len) / 8 = %d\n", us_len, plcp_len,
				(11 * us_len), (11 * us_len) / 8);

			if ((11 * us_len) / 8 > plcp_len) {
				pskcck->service |= PLCP_SERVICE_LENEXT;
				dprintk(DALL, "Set PLCP_SERVICE_LENEXT, \
				    pskcck->service = 0x%4.4X\n", pskcck->service);
			} else {
				dprintk(DALL, "Did not set PLCP_SERVICE_LENEXT, \
				  pskcck->service = 0x%4.4X\n", pskcck->service);
			}
			break;
		default:
			digi_dbg("rate = %p, rate->bitrate%d\n", rate, rate->bitrate);
			WARN_ON(1);
			us_len = 0;
		}

		pskcck->length = cpu_to_le16(us_len);

		dprintk(DALL, "pskcck .length = %d, signal = %d, service = %d\n",
			pskcck->length, pskcck->signal, pskcck->service);
		dprintk(DALL, "rate->bitrate=%x (@%dM), pckcck->length=%d\n",
			rate->bitrate, rate->bitrate/10, pskcck->length);
	}

	hdr = (struct tx_frame_hdr *) frame;
	hdr->pad = 0;
	hdr->length = NUMBER_OF_WORD32((length + aes_len + TX_HEADER_LENGTH));
	hdr->modulation_type = ofdm ? MOD_TYPE_OFDM : MOD_TYPE_PSKCCK;

	dprintk(DVVERBOSE, "frame hdr .length = %d, .modulation_type = %d\n",
		hdr->length, hdr->modulation_type);

	dprintk(DVERBOSE, "TX: %d byte %s packet @ %dmbit\n", length,
		ofdm ? "OFDM" : "PSK/CCK", rate->bitrate/10);
}
EXPORT_SYMBOL_GPL(phy_set_plcp);


static int get_signal(struct rx_frame_hdr *hdr, enum ieee80211_band rf_band, int transceiver)
{
    int gain;
    int signal;

    if (transceiver == RF_AIROHA_2236) {
        const u8 lnaTable_al2236[] =
        {
            0, 0, 20, 36
        };

        // Map high gain values to dbm
        const signed char gainTable_al2236[] =
        {
           -85, -85, -88, -88, -92
        };
        // Convert received signal strength to dbm
        gain = lnaTable_al2236[hdr->rssi_low_noise_amp] + 2*hdr->rssi_variable_gain_attenuator;
        if (gain > 92)
            signal = -96;
        else if (gain > 87)
            signal = gainTable_al2236[gain - 88];
        else
            signal = 4 - gain;
    } else {
        static const u8 lnaTable_al7230_24ghz[] =
        {
            0, 0, 18, 42
        };
        static const u8 lnaTable_al7230_50ghz[] =
        {
            0, 0, 17, 37
        };
        /* Convert received signal strength to dbm for RF_AIROHA_7230 */
        if (rf_band == IEEE80211_BAND_2GHZ) {
            gain = lnaTable_al7230_24ghz[hdr->rssi_low_noise_amp] + 2*hdr->rssi_variable_gain_attenuator;
            signal = 2 - gain;
        } else {
            gain = lnaTable_al7230_50ghz[hdr->rssi_low_noise_amp] + 2*hdr->rssi_variable_gain_attenuator;
            signal = -5 - gain;
        }
    }

    return signal;
}




/* FIXME, following limtis should depend on the platform */
#define PIPER_MAX_SIGNAL_DBM		(-30)
#define PIPER_MIN_SIGNAL_DBM		(-90)
#if 0
/*
 * Latest version of mac80211 doesn't seem to be interested in link quality????
 */
static int calculate_link_quality(int signal)
{
	int quality;

	if (signal < PIPER_MIN_SIGNAL_DBM)
		quality = 0;
	else if (signal > PIPER_MAX_SIGNAL_DBM)
		quality = 100;
	else
		quality = (signal - PIPER_MIN_SIGNAL_DBM) * 100 /
			  (PIPER_MAX_SIGNAL_DBM - PIPER_MIN_SIGNAL_DBM);

	dprintk(DVERBOSE, "signal: %d, quality: %d/100\n", signal, quality);

	return quality;
}
#endif

unsigned int phy_determine_rssi(struct rx_frame_hdr *hdr)
{
	return (hdr->rssi_low_noise_amp << 5) | hdr->rssi_variable_gain_attenuator;
}
EXPORT_SYMBOL_GPL(phy_determine_rssi);



void phy_process_plcp(struct piper_priv *piper, struct rx_frame_hdr *hdr,
		struct ieee80211_rx_status *status, unsigned int *length)
{
	unsigned rate, i;
	struct digi_rf_ops *rf = piper->rf;

	memset(status, 0, sizeof(*status));
	status->band = piper->rf->getBand(piper->channel);
	status->signal = get_signal(hdr, status->band, piper->pdata->rf_transceiver);
	status->antenna = hdr->antenna;
	status->freq = piper->rf->getFrequency(piper->channel);

	if (hdr->modulation_type == MOD_TYPE_OFDM) {
		/* OFDM */
		struct ofdm_hdr *ofdm = &hdr->mod.ofdm;
		const int ofdm_rate[] = {
			480, 240, 120, 60, 540, 360, 180, 90
		};

		rate = ofdm_rate[ofdm->rate & 0x7];
		*length = le16_to_cpu(ofdm->length);
		dprintk(DVVERBOSE, "%d byte OFDM packet @ %dmbit\n",
			*length, rate/10);
	} else {
		/* PSK/CCK */
		struct psk_cck_hdr *pskcck = &hdr->mod.psk;

		rate = pskcck->signal;

		*length = le16_to_cpu(pskcck->length);
		switch (rate) {
		case 10:
			*length /= 8;
			break;
		case 20:
			*length /= 4;
			break;
		case 55:
			*length = (11 * (*length)) / 16;
			break;
		case 110:
			*length = (11 * (*length)) / 8;
			if (pskcck->service & PLCP_SERVICE_LENEXT)
				(*length)--;
			break;
		default:
			/* WARN_ON(1); This happens to often for us to generate that long error message */
			*length = 0;
		}

		dprintk(DVVERBOSE, "%d byte PSK/CCK packet @ %dmbit\n",
			*length, rate/10);
	}

	/* match rate with the list of bitrates that we supplied the stack */
	for (i = 0; i < rf->bands[status->band].n_bitrates; i++) {
		if (rf->bands[status->band].bitrates[i].bitrate == rate)
			break;
	}

	if (i != rf->bands[status->band].n_bitrates)
		status->rate_idx = i;
	else {
	    *length = 0;
		digi_dbg(PIPER_DRIVER_NAME
				": couldn't find bitrate index for %d?\n",
				rate);
		status->flag |= RX_FLAG_FAILED_PLCP_CRC;
		return;
	}
}
EXPORT_SYMBOL_GPL(phy_process_plcp);

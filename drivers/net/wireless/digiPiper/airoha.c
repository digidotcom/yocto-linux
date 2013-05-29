/*
 * Ubec AH7230 radio support.
 *
 * Copyright © 2009  Digi International, Inc
 *
 * Author: Contact support@digi.com for information about this software.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <net/mac80211.h>

#include "pipermain.h"
#include "mac.h"
#include "net/cfg80211.h"
#include "airoha.h"
#include "airohaCalibration.h"

/*
 * Number of us to change channels.  I counted the number of udelays once
 * and it was about 2030, plus the 1 us delays for each register write.
 * So probably about 2200 in reality, I'm saying 2500 to be safe.
 */
#define CHANNEL_CHANGE_TIME	(2500)

/*
 * Maximum possible receive signal strength in dbm.  Most of the
 * values will be negative.
 */
#define MAX_SIGNAL_IN_DBM	(5)

#define read_reg(reg)		priv->ac->rd_reg(priv,reg)
#define write_reg(reg,val,op)	priv->ac->wr_reg(priv,reg,val,op)
#define mac_set_tx_power(x)	al7230_set_txpwr(hw,x)


static void InitializeRF(struct ieee80211_hw *hw, int band_selection);
static int al7230_set_txpwr(struct ieee80211_hw *hw, uint8_t val);

static const struct {
	unsigned int integer;
	unsigned int fraction;
	unsigned int address4;
	unsigned int tracking;
} freqTableAiroha_7230[] = {
	{ 0, 0, 0, 0 },					// 0

	// 2.4 GHz band (802.11b/g)
	{ 0x00379, 0x13333, 0x7FD78, TRACK_BG_BAND },	// B-1   (2412 MHz)   1
	{ 0x00379, 0x1B333, 0x7FD78, TRACK_BG_BAND },	// B-2   (2417 MHz)   2
	{ 0x00379, 0x03333, 0x7FD78, TRACK_BG_BAND },	// B-3   (2422 MHz)   3
	{ 0x00379, 0x0B333, 0x7FD78, TRACK_BG_BAND },	// B-4   (2427 MHz)   4
	{ 0x0037A, 0x13333, 0x7FD78, TRACK_BG_BAND },	// B-5   (2432 MHz)   5
	{ 0x0037A, 0x1B333, 0x7FD78, TRACK_BG_BAND },	// B-6   (2437 MHz)   6
	{ 0x0037A, 0x03333, 0x7FD78, TRACK_BG_BAND },	// B-7   (2442 MHz)   7
	{ 0x0037A, 0x0B333, 0x7FD78, TRACK_BG_BAND },	// B-8   (2447 MHz)   8
	{ 0x0037B, 0x13333, 0x7FD78, TRACK_BG_BAND },	// B-9   (2452 MHz)   9
	{ 0x0037B, 0x1B333, 0x7FD78, TRACK_BG_BAND },	// B-10  (2457 MHz)  10
	{ 0x0037B, 0x03333, 0x7FD78, TRACK_BG_BAND },	// B-11  (2462 MHz)  11
	{ 0x0037B, 0x0B333, 0x7FD78, TRACK_BG_BAND },	// B-12  (2467 MHz)  12
	{ 0x0037C, 0x13333, 0x7FD78, TRACK_BG_BAND },	// B-13  (2472 MHz)  13
	{ 0x0037C, 0x06666, 0x7FD78, TRACK_BG_BAND },	// B-14  (2484 MHz)  14

	{ 0, 0, 0, 0 },					// reserved for future b/g expansion 15
	{ 0, 0, 0, 0 },					// reserved for future b/g expansion 16

	// Extended 5 GHz bands (802.11a)
	{ 0x0FF54, 0x00000, 0x67F78, TRACK_5150_5350_A_BAND }, // A-8   (5040 MHz)  21 tracking?
	{ 0x0FF54, 0x0AAAA, 0x77F78, TRACK_5150_5350_A_BAND }, // A-12  (5060 MHz)  22 tracking?
	{ 0x0FF55, 0x15555, 0x77F78, TRACK_5150_5350_A_BAND }, // A-16  (5080 MHz)  23 tracking?
	{ 0x0FF56, 0x05555, 0x77F78, TRACK_5150_5350_A_BAND }, // A-34  (5170 MHz)  24
	{ 0x0FF56, 0x0AAAA, 0x77F78, TRACK_5150_5350_A_BAND }, // A-36  (5180 MHz)  25
	{ 0x0FF57, 0x10000, 0x77F78, TRACK_5150_5350_A_BAND }, // A-38  (5190 MHz)  26
	{ 0x0FF57, 0x15555, 0x77F78, TRACK_5150_5350_A_BAND }, // A-40  (5200 MHz)  27
	{ 0x0FF57, 0x1AAAA, 0x77F78, TRACK_5150_5350_A_BAND }, // A-42  (5210 MHz)  28
	{ 0x0FF57, 0x00000, 0x67F78, TRACK_5150_5350_A_BAND }, // A-44  (5220 MHz)  29
	{ 0x0FF57, 0x05555, 0x77F78, TRACK_5150_5350_A_BAND }, // A-46  (5230 MHz)  30
	{ 0x0FF57, 0x0AAAA, 0x77F78, TRACK_5150_5350_A_BAND }, // A-48  (5240 MHz)  31

	{ 0x0FF58, 0x15555, 0x77F78, TRACK_5150_5350_A_BAND }, // A-52  (5260 MHz)  32
	{ 0x0FF58, 0x00000, 0x67F78, TRACK_5150_5350_A_BAND }, // A-56  (5280 MHz)  33
	{ 0x0FF58, 0x0AAAA, 0x77F78, TRACK_5150_5350_A_BAND }, // A-60  (5300 MHz)  34
	{ 0x0FF59, 0x15555, 0x77F78, TRACK_5150_5350_A_BAND }, // A-64  (5320 MHz)  35

	{ 0x0FF5C, 0x15555, 0x77F78, TRACK_5470_5725_A_BAND }, // A-100 (5500 MHz)  36
	{ 0x0FF5C, 0x00000, 0x67F78, TRACK_5470_5725_A_BAND }, // A-104 (5520 MHz)  37
	{ 0x0FF5C, 0x0AAAA, 0x77F78, TRACK_5470_5725_A_BAND }, // A-108 (5540 MHz)  38
	{ 0x0FF5D, 0x15555, 0x77F78, TRACK_5470_5725_A_BAND }, // A-112 (5560 MHz)  39
	{ 0x0FF5D, 0x00000, 0x67F78, TRACK_5470_5725_A_BAND }, // A-116 (5580 MHz)  40
	{ 0x0FF5D, 0x0AAAA, 0x77F78, TRACK_5470_5725_A_BAND }, // A-120 (5600 MHz)  41
	{ 0x0FF5E, 0x15555, 0x77F78, TRACK_5470_5725_A_BAND }, // A-124 (5620 MHz)  42
	{ 0x0FF5E, 0x00000, 0x67F78, TRACK_5470_5725_A_BAND }, // A-128 (5640 MHz)  43
	{ 0x0FF5E, 0x0AAAA, 0x77F78, TRACK_5470_5725_A_BAND }, // A-132 (5660 MHz)  44
	{ 0x0FF5F, 0x15555, 0x77F78, TRACK_5470_5725_A_BAND }, // A-136 (5680 MHz)  45
	{ 0x0FF5F, 0x00000, 0x67F78, TRACK_5470_5725_A_BAND }, // A-140 (5700 MHz)  46

	{ 0x0FF60, 0x18000, 0x77F78, TRACK_5725_5825_A_BAND }, // A-149 (5745 MHz)  47
	{ 0x0FF60, 0x02AAA, 0x77F78, TRACK_5725_5825_A_BAND }, // A-153 (5765 MHz)  48
	{ 0x0FF60, 0x0D555, 0x77F78, TRACK_5725_5825_A_BAND }, // A-157 (5785 MHz)  49
	{ 0x0FF61, 0x18000, 0x77F78, TRACK_5725_5825_A_BAND }, // A-161 (5805 MHz)  50
	{ 0x0FF61, 0x02AAA, 0x77F78, TRACK_5725_5825_A_BAND }, // A-165 (5825 MHz)  51
};

#define CHAN4G(idx, _freq)					\
	.band			= IEEE80211_BAND_2GHZ,		\
	.center_freq		= (_freq),			\
	.hw_value		= idx,				\
	.max_antenna_gain	= 0,				\
	.max_power		= 12

static struct ieee80211_channel al7230_bg_channels[] = {
	{ CHAN4G(1, 2412) },
	{ CHAN4G(2, 2417) },
	{ CHAN4G(3, 2422) },
	{ CHAN4G(4, 2427) },
	{ CHAN4G(5, 2432) },
	{ CHAN4G(6, 2437) },
	{ CHAN4G(7, 2442) },
	{ CHAN4G(8, 2447) },
	{ CHAN4G(9, 2452) },
	{ CHAN4G(10, 2457) },
	{ CHAN4G(11, 2462) },
	{ CHAN4G(12, 2467) },
	{ CHAN4G(13, 2472) },
	{ CHAN4G(14, 2484) },
};

static const struct ieee80211_rate al7230_bg_rates[] = {
	/* psk/cck rates */
	{
		.bitrate = 10,
		.flags = IEEE80211_RATE_SHORT_PREAMBLE,
	},
	{
		.bitrate = 20,
		.flags = IEEE80211_RATE_SHORT_PREAMBLE,
	},
	{
		.bitrate = 55,
		.flags = IEEE80211_RATE_SHORT_PREAMBLE,
	},
	{
		.bitrate = 110,
		.flags = IEEE80211_RATE_SHORT_PREAMBLE,
	},
	/* ofdm rates */
	{
		.bitrate = 60,
		.hw_value = 0xb,
	},
	{
		.bitrate = 90,
		.hw_value = 0xf,
	},
	{
		.bitrate = 120,
		.hw_value = 0xa,
	},
	{
		.bitrate = 180,
		.hw_value = 0xe,
	},
	{
		.bitrate = 240,
		.hw_value = 0x9,
	},
	{
		.bitrate = 360,
		.hw_value = 0xd,
	},
	{
		.bitrate = 480,
		.hw_value = 0x8,
	},
	{
		.bitrate = 540,
		.hw_value = 0xc,
	},
};

#define MAX_BG_RATE_INDEX       ((sizeof(al7230_bg_rates) / sizeof(struct ieee80211_rate)) - 1)


#define CHAN5G(idx, frequency)					\
	.band			= IEEE80211_BAND_5GHZ,		\
	.center_freq		= frequency,			\
	.max_antenna_gain	= 0,				\
	.max_power		= 8,				\
	.hw_value		= idx

static struct ieee80211_channel al7230_a_channels[] = {
	{ CHAN5G(17, 5040) },
	{ CHAN5G(18, 5060) },
	{ CHAN5G(19, 5080) },
	{ CHAN5G(20, 5170) },
	{ CHAN5G(21, 5180) },
	{ CHAN5G(22, 5190) },
	{ CHAN5G(23, 5200) },
	{ CHAN5G(24, 5210) },
	{ CHAN5G(25, 5220) },
	{ CHAN5G(26, 5230) },
	{ CHAN5G(27, 5240) },
	{ CHAN5G(28, 5260) },
	{ CHAN5G(29, 5280) },
	{ CHAN5G(30, 5300) },
	{ CHAN5G(31, 5320) },
	{ CHAN5G(32, 5500) },
	{ CHAN5G(33, 5520) },
	{ CHAN5G(34, 5540) },
	{ CHAN5G(35, 5560) },
	{ CHAN5G(36, 5580) },
	{ CHAN5G(37, 5600) },
	{ CHAN5G(38, 5620) },
	{ CHAN5G(39, 5640) },
	{ CHAN5G(40, 5660) },
	{ CHAN5G(41, 5680) },
	{ CHAN5G(42, 5700) },
	{ CHAN5G(43, 5745) },
	{ CHAN5G(44, 5765) },
	{ CHAN5G(45, 5785) },
	{ CHAN5G(46, 5805) },
	{ CHAN5G(47, 5825) }
};

static const struct ieee80211_rate al7230_a_rates[] = {
	/* ofdm rates */
	{
		.bitrate = 60,
		.hw_value = 0xb,
	},
	{
		.bitrate = 90,
		.hw_value = 0xf,
	},
	{
		.bitrate = 120,
		.hw_value = 0xa,
	},
	{
		.bitrate = 180,
		.hw_value = 0xe,
	},
	{
		.bitrate = 240,
		.hw_value = 0x9,
	},
	{
		.bitrate = 360,
		.hw_value = 0xd,
	},
	{
		.bitrate = 480,
		.hw_value = 0x8,
	},
	{
		.bitrate = 540,
		.hw_value = 0xc,
	},
};

#define _54MBPS_A_RATE_INDEX        ((sizeof(al7230_a_rates) / sizeof(struct ieee80211_rate)) - 1)
#define _48MBPS_A_RATE_INDEX        ((sizeof(al7230_a_rates) / sizeof(struct ieee80211_rate)) - 2)
#define _36MBPS_A_RATE_INDEX        ((sizeof(al7230_a_rates) / sizeof(struct ieee80211_rate)) - 3)
#define _24MBPS_A_RATE_INDEX        ((sizeof(al7230_a_rates) / sizeof(struct ieee80211_rate)) - 4)
#define _18MBPS_A_RATE_INDEX        ((sizeof(al7230_a_rates) / sizeof(struct ieee80211_rate)) - 5)

static enum ieee80211_band getBand(int channelIndex)
{
	enum ieee80211_band result;

	if (channelIndex >= BAND_A_OFFSET) {
		result = IEEE80211_BAND_5GHZ;
	} else {
		result = IEEE80211_BAND_2GHZ;
	}

	return result;
}

static int getFrequency(int channelIndex)
{
	int result;

	if (getBand(channelIndex) == IEEE80211_BAND_5GHZ) {
		result = al7230_a_channels[channelIndex - BAND_A_OFFSET].center_freq;
	} else {
		result = al7230_bg_channels[channelIndex - 1].center_freq;
	}

	return result;
}

static int write_rf(struct ieee80211_hw *hw, unsigned char reg, unsigned int val)
{
	struct piper_priv *priv = hw->priv;
	int err;

	err = write_reg(BB_SPI_DATA, val << 4 | reg, op_write);
	udelay(3);		/* Mike Schaffner says to allow 2 us or more between all writes */
	return err;
}


/*
 * This function is called to set the value of Airoha register
 * 0xc.  This register must be set to different values depending
 * on the H/W revision of the board due to changes in the board
 * design.
 */
static void set_hw_specific_parameters(struct ieee80211_hw *hw,
											unsigned int band,
										    unsigned int hw_revision,
										    unsigned int hw_platform)
{
    (void) hw_platform;
#ifdef CONFIG_MACH_CCW9P9215JS
		switch (hw_revision) {
			case WCD_HW_REV_PROTOTYPE:
			case WCD_HW_REV_PILOT:
			case WCD_HW_REV_A:
			default:
				if (band == IEEE80211_BAND_2GHZ) {
					write_rf(hw, 0xc, 0x31);
				} else {
					write_rf(hw, 0xc, 0x00143 );
				}
				break;
		}
#else
		switch (hw_revision) {
			case WCD_HW_REV_PROTOTYPE:
			case WCD_HW_REV_PILOT:
				if (band == IEEE80211_BAND_2GHZ) {
					write_rf(hw, 0xc, 0xa3);
				} else {
					write_rf(hw, 0xc, 0x00143 );
				}
			break;

			case WCD_HW_REV_A:
			default:
				if (band == IEEE80211_BAND_2GHZ) {
					write_rf(hw, 0xc, 0x70);
				} else {
					write_rf(hw, 0xc, 0x00143 );
				}
				break;
		}
#endif
}

static int al7230_rf_set_chan_private(struct ieee80211_hw *hw, int channelIndex, bool enable_rx)
{
	struct piper_priv *priv = hw->priv;
	static int rf_band;
#ifdef WANT_DEBUG
	const char *channelLookup[] = {
		"invalid 0",
		"B-1",
		"B-2",
		"B-3",
		"B-4",
		"B-5",
		"B-6",
		"B-7",
		"B-8",
		"B-9",
		"B-10",
		"B-11",
		"B-12",
		"B-13",
		"B-14",
		"invalid 15",
		"invalid 16",
		"A-8",
		"A-12",
		"A-16",
		"A-34",
		"A-36",
		"A-38",
		"A-40",
		"A-42",
		"A-44",
		"A-46",
		"A-48",
		"A-52",
		"A-56",
		"A-60",
		"A-64",
		"A-100",
		"A-104",
		"A-108",
		"A-112",
		"A-116",
		"A-120",
		"A-124",
		"A-128",
		"A-132",
		"A-136",
		"A-140",
		"A-149",
		"A-153",
		"A-157",
		"A-161",
		"A-165"
	};
printk(KERN_ERR "Setting channel %s\n", channelLookup[channelIndex]);
#endif
	if (channelIndex >= BAND_A_OFFSET)
		rf_band = IEEE80211_BAND_5GHZ;
	else
		rf_band = IEEE80211_BAND_2GHZ;
	/* Disable the rx processing path */
	write_reg(BB_GENERAL_CTL, ~BB_GENERAL_CTL_RX_EN, op_and);

    write_reg(BB_OUTPUT_CONTROL, 0xfffff33f, op_and);
    write_reg(BB_OUTPUT_CONTROL, 0x00000880, op_or);

	if (priv->pdata->rf_transceiver == RF_AIROHA_2236) {
/* TODO, when using this transceiver, resolve this commented code */
#ifdef BUILD_THIS_CODE_SECTION
		write_reg(BB_GENERAL_STAT, BB_GENERAL_STAT_B_EN, op_or);

		if (macParams.band == WLN_BAND_B) {
			/* turn off OFDM */
			write_reg(BB_GENERAL_STAT, ~BB_GENERAL_STAT_A_EN, op_and);
		} else {
			/* turn on OFDM */
			write_reg(BB_GENERAL_STAT, BB_GENERAL_STAT_A_EN, op_or);
		}
		/* set the 802.11b/g frequency band specific tracking constant */
		write_reg(BB_TRACK_CONTROL, 0xff00ffff, op_and);

		write_reg(BB_TRACK_CONTROL, TRACK_BG_BAND, op_or);

		/* perform chip and frequency-band specific RF initialization */
		InitializeRF(hw, rf_band);

		mac_set_tx_power(priv->tx_power);

		write_rf(hw, 0, freqTableAiroha_2236[channelIndex].integer);
		write_rf(hw, 1, freqTableAiroha_2236[channelIndex].fraction);

		/* critical delay for correct calibration */
		udelay(150);

		/*
		 * TXON, PAON and RXON should all be low before Calibration
		 * TXON and PAON will be low as long as no frames are written to the TX
                 * DATA fifo.
		 * RXON will be low as long as the receive path is not enabled (bit 0 of
                 * GEN CTL register is 0).
		 */

		/* calibrate RF transceiver */

		/* TXDCOC->active; RCK->disable */
		write_rf(hw, 15, 0x00D87);
		udelay(50);
		/* TXDCOC->disable; RCK->enable */
		write_rf(hw, 15, 0x00787);
		udelay(50);
		/* TXDCOC->disable; RCK->disable */
		write_rf(hw, 15, 0x00587);
		udelay(50);

		/* configure the baseband processing engine */
		write_reg(BB_GENERAL_CTL, ~BB_GENERAL_CTL_GEN_5GEN, op_and);

		/*Re-enable the rx processing path */
		write_reg(BB_GENERAL_CTL, BB_GENERAL_CTL_RX_EN, op_or);
#endif
	} else if (priv->pdata->rf_transceiver == RF_AIROHA_7230) {
		/* enable the frequency-band specific PA */
		if (rf_band == IEEE80211_BAND_2GHZ) {
			//HW_GEN_CONTROL &= ~GEN_PA_ON;
			write_reg(BB_GENERAL_STAT, BB_GENERAL_STAT_A_EN | BB_GENERAL_STAT_B_EN,
				  op_or);

			/* set the 802.11b/g frequency band specific tracking constant */
			write_reg(BB_TRACK_CONTROL, 0xff00ffff, op_and);

			write_reg(BB_TRACK_CONTROL, TRACK_BG_BAND, op_or);

		} else {
			//HW_GEN_CONTROL |= GEN_PA_ON;

			// turn off PSK/CCK
			write_reg(BB_GENERAL_STAT, ~BB_GENERAL_STAT_B_EN, op_and);

			// turn on OFDM
			write_reg(BB_GENERAL_STAT, BB_GENERAL_STAT_A_EN, op_or);

			/* Set the 802.11a frequency sub-band specific tracking constant */
			/* All 8 supported 802.11a channels are in this 802.11a frequency sub-band */
			write_reg(BB_TRACK_CONTROL, 0xff00ffff, op_and);

			write_reg(BB_TRACK_CONTROL, freqTableAiroha_7230[channelIndex].tracking,
				  op_or);
		}

		/* perform chip and frequency-band specific RF initialization */
		InitializeRF(hw, rf_band);

		mac_set_tx_power(priv->tx_power);

		/* Set the channel frequency */
		write_rf(hw, 0, freqTableAiroha_7230[channelIndex].integer);
		udelay(150);				/* Mike Schaffner says this is needed here */
		write_rf(hw, 1, freqTableAiroha_7230[channelIndex].fraction);
		udelay(150);				/* Mike Schaffner says this is needed here */
		write_rf(hw, 4, freqTableAiroha_7230[channelIndex].address4);
		udelay(150);				/* Mike Schaffner says this is needed here */


		// Select the frequency band: 5Ghz or 2.4Ghz
		if (rf_band == IEEE80211_BAND_5GHZ) {
			/* calibrate RF transceiver */

			/* TXDCOC->active; RCK->disable */
			write_rf(hw, 15, 0x9ABA8);
			udelay(50);

			/* TXDCOC->disable; RCK->enable */
			write_rf(hw, 15, 0x3ABA8);
			udelay(50);

			/* TXDCOC->disable; RCK->disable */
			write_rf(hw, 15, 0x12BAC);
			udelay(50);

			/* configure the baseband processing engine */
			/*
			 * This bit always as to be turned off when we are using
			 * the Airoha chip, even though it's named the 5G EN bit.
			 * It has to do with how they hooked up the Airoha.
			 */
			write_reg(BB_GENERAL_CTL, ~BB_GENERAL_CTL_GEN_5GEN, op_and);
		} else {
			/* calibrate RF transceiver */

			/* TXDCOC->active; RCK->disable */
			write_rf(hw, 15, 0x9ABA8);
			udelay(50);

			/* TXDCOC->disable; RCK->enable */
			write_rf(hw, 15, 0x3ABA8);
			udelay(50);

			/* TXDCOC->disable; RCK->disable */
			write_rf(hw, 15, 0x1ABA8);
			udelay(50);

			/* configure the baseband processing engine */
			write_reg(BB_GENERAL_CTL, ~BB_GENERAL_CTL_GEN_5GEN, op_and);
			/*
			 * No short preambles allowed for ODFM.
			 */
			write_reg(BB_GENERAL_CTL, ~BB_GENERAL_CTL_SH_PRE, op_and);
		}

		/*Re-enable the rx processing path */
		if (enable_rx)
			write_reg(BB_GENERAL_CTL, BB_GENERAL_CTL_RX_EN, op_or);

		/* re-enable transmitter */
        write_reg(BB_OUTPUT_CONTROL, 0xfffff33f, op_and);
	} else {
		printk(KERN_WARNING PIPER_DRIVER_NAME ": undefined rf transceiver!\n");
		return -EINVAL;
	}

    digiWifiCalibrationRestartCalibration(priv);
    /*
     * This is a patch for a problem which should be corrected in
     * hardware on new units.  We are rewriting the MAC address
     * because on units without the H/W patch the address can
     * be corrupted when we change channels.
     */
    piper_set_macaddr(priv);

	return 0;
}

static int al7230_rf_set_chan(struct ieee80211_hw *hw, int channelIndex)
{
	return al7230_rf_set_chan_private(hw, channelIndex, true);
}

static int al7230_rf_set_chan_no_rx(struct ieee80211_hw *hw, int channelIndex)
{
	return al7230_rf_set_chan_private(hw, channelIndex, false);
}



static int al7230_set_txpwr(struct ieee80211_hw *hw, uint8_t value)
{
	struct piper_priv *priv = hw->priv;

	if (priv->pdata->rf_transceiver == RF_AIROHA_2236) {
		const unsigned char powerTable_2236[] = {
			4, 10, 10, 18, 22, 22, 28, 28,
			33, 33, 36, 38, 40, 43, 45, 47
		};
		write_rf(hw, 9, 0x05440 | powerTable_2236[value & 0xf]);
	} else if (priv->pdata->rf_transceiver == RF_AIROHA_7230) {
		const unsigned char powerTable_7230[] = {
			0x14, 0x14, 0x14, 0x18, 0x18, 0x1c, 0x1c, 0x20,
			0x20, 0x24, 0x24, 0x29, 0x29, 0x2c, 0x2c, 0x30
		};
		int correctedPowerIndex = digiWifiCalibrationPowerIndex(priv);

		if (correctedPowerIndex != -1) {
		    write_rf(hw, 11, 0x08040 | correctedPowerIndex);
		} else {
		    write_rf(hw, 11, 0x08040 | powerTable_7230[value & 0xf]);
		}
	} else {
		printk(KERN_WARNING PIPER_DRIVER_NAME
		       ": undefined rf transceiver!\n");
		return -EINVAL;
	}
	return 0;
}

static void al7230_set_power_index(struct ieee80211_hw *hw, unsigned int value)
{
	write_rf(hw, 11, 0x08040 | value);
}

static void InitializeRF(struct ieee80211_hw *hw, int band_selection)
{
	struct piper_priv *priv = hw->priv;

	if (priv->pdata->rf_transceiver == RF_AIROHA_2236) {
		digi_dbg("**** transceiver == RF_AIROHA_2236\n");
		/* Initial settings for 20 MHz reference frequency, 802.11b/g */

		/* CH_integer: Frequency register 0 */
		write_rf(hw, 0, 0x01f79 );

		/* CH_fraction: Frequency register 1 */
		write_rf(hw, 1, 0x03333 );

		/*Config 1 = default value */
		write_rf(hw, 2, 0x00B80 );

		/*Config 2 = default value */
		write_rf(hw, 3, 0x00E7F );

		/*Config 3 = default value */
		write_rf(hw, 4, 0x0905A );

		/*Config 4 = default value */
		write_rf(hw, 5, 0x0F4DC );

		/*Config 5 = default value */
		write_rf(hw, 6, 0x0805B );

		/*Config 6 = Crystal frequency /2 to pll reference divider */
		write_rf(hw, 7, 0x0116C );

		/*Config 7 = RSSI = default value */
		write_rf(hw, 8, 0x05B68 );

		/* TX gain control for LA2236 */
		write_rf(hw, 9, 0x05460 );   // sit at the middle

		/* RX Gain = digi specific value: AGC adjustment is done over the GC1-GC7
		IC pins interface. AGC MAX GAIN value is configured in the FPGA BB register
		instead of the RF register here below */
		write_rf(hw, 10, 0x001BB );

		/* TX Gain = digi specific vaue: TX GAIN set using the register */
		write_rf(hw, 11, 0x000f9 );

		/* PA current = default value */
		write_rf(hw, 12, 0x039D8 );

		/* Config 8 = default value  */
		write_rf(hw, 13, 0x08000 );

		/* Config 9 = default value */
		write_rf(hw, 14, 0x00000 );

		/* Config 10 = default value  */
		write_rf(hw, 15, 0x00587 );

		//mac_set_tx_power (macParams.tx_power);

		/* Calibration procedure */
		write_reg(BB_OUTPUT_CONTROL, 0x00000300, op_or);
		udelay(150);

		/* TXDCOC->active; RCK->disable */
		write_rf(hw, 15, 0x00D87 );
		udelay(50);

		/* TXDCOC->disable; RCK->enable */
		write_rf(hw, 15, 0x00787 );
		udelay(50);

		/* TXDCOC->disable; RCK->disable */
		write_rf(hw, 15, 0x00587 );
		udelay(50);
	} else if (priv->pdata->rf_transceiver == RF_AIROHA_7230) {
		switch (band_selection) {
		case IEEE80211_BAND_2GHZ:
			/* Initial settings for 20 MHz reference frequency, 802.11b/g */
			write_reg(BB_OUTPUT_CONTROL, 0xfffffcff, op_and);
			write_reg(BB_OUTPUT_CONTROL, 0x00000200, op_or);
			udelay(150);

			/* Frequency register 0 */
			write_rf(hw, 0, 0x00379 );

			/* Frequency register 1 */
			write_rf(hw, 1, 0x13333 );
			udelay(10);

			/*Config 1 = default value */
			write_rf(hw, 2, 0x841FF );

			/*Config 2 = default value */
			write_rf(hw, 3, 0x3FDFA );

			/*Config 3 = default value */
			write_rf(hw, 4, 0x7FD78 );

			/*Config 4 = default value */
			write_rf(hw, 5, 0x802BF );

			/*Config 5 = default value */
			write_rf(hw, 6, 0x56AF3 );

			/*Config 6 = Crystal frequency /2 to pll reference divider */
			write_rf(hw, 7, 0xCE000 );

			/*Config 7 = RSSI = default value */
			write_rf(hw, 8, 0x6EBC0 );

			/* Filter BW  = default value */
			write_rf(hw, 9, 0x221BB );

			/* RX Gain = digi specific value: AGC adjustment is done over the GC1-GC7
			IC pins interface. AGC MAX GAIN value is configured in the FPGA BB register
			instead of the RF register here below */
			write_rf(hw, 10, 0xE0040 );

			/* TX Gain = digi specific vaue: TX GAIN set using the register */
			// write_rf(hw, 11, 0x08070);
			mac_set_tx_power (priv->tx_power);  //Digi value

			/* PA current = default value */
			set_hw_specific_parameters(hw, IEEE80211_BAND_2GHZ, priv->rf->hw_revision, priv->rf->hw_platform);

			/* Config 8 = default value  */
			write_rf(hw, 13, 0xFFFFF );

			/* Config 9 = default value */
			write_rf(hw, 14, 0x00000 );

			/* Config 10 = default value  */
			write_rf(hw, 15, 0x1ABA8 );

			/* Calibration procedure */
			write_reg(BB_OUTPUT_CONTROL, 0x00000300, op_or);

			udelay(150);

			/* Calibration procedure */

			/* TXDCOC->active; RCK->disable */
			write_rf(hw, 15, 0x9ABA8 );
			udelay(50);

			/* TXDCOC->disable; RCK->enable */
			write_rf(hw, 15, 0x3ABA8 );
			udelay(50);

			/* TXDCOC->disable; RCK->disable */
			write_rf(hw, 15, 0x1ABA8 );
			udelay(50);

			write_reg(BB_GENERAL_CTL, ~BB_GENERAL_CTL_MAX_GAIN_MASK, op_and);
			write_reg(BB_GENERAL_CTL, BB_GENERAL_CTL_DEFAULT_MAX_GAIN_BG, op_or);
			break;

		case IEEE80211_BAND_5GHZ:
			/* Initial settings for 20 MHz reference frequency, 802.11a */
			write_reg(BB_OUTPUT_CONTROL, 0xfffffcff, op_and);
			write_reg(BB_OUTPUT_CONTROL, 0x00000200, op_or);
			udelay(150);

			/* Frequency register 0 */
			write_rf(hw, 0, 0x0FF56 );

			/* Frequency register 1 */
			write_rf(hw, 1, 0x0AAAA );

			udelay(10);

			/*Config 1 = default value */
			write_rf(hw, 2, 0x451FE );

			/*Config 2 = default value */
			write_rf(hw, 3, 0x5FDFA );

			/*Config 3 = default value */
			write_rf(hw, 4, 0x67f78 );

			/*Config 4 = default value */
			write_rf(hw, 5, 0x853FF );

			/*Config 5 = default value */
			write_rf(hw, 6, 0x56AF3 );

			/*Config 6 = Crystal frequency /2 to pll reference divider */
			write_rf(hw, 7, 0xCE000 );

			/*Config 7 = RSSI = default value */
			write_rf(hw, 8, 0x6EBC0 );

			/* Filter BW  = default value */
			write_rf(hw, 9, 0x221BB );

			/* RX Gain = digi value */
			write_rf(hw, 10, 0xE0600 );

			/* TX Gain = digi specific vaue: TX GAIN set using the register */
			// write_rf(hw, 11, 0x08070 );
			mac_set_tx_power (priv->tx_power);  //Digi value

			/* PA current = default value */
			set_hw_specific_parameters(hw, IEEE80211_BAND_5GHZ, priv->rf->hw_revision, priv->rf->hw_platform);

			/* Config 8 = default value  */
			write_rf(hw, 13, 0xFFFFF );

			    /* Config 9 = default value */
			write_rf(hw, 14, 0x00000 );

			/* Config 10 = default value  */
			write_rf(hw, 15, 0x12BAC );

			/* Calibration procedure */
			write_reg(BB_OUTPUT_CONTROL, 0x00000300, op_or);

			udelay(150);

			/* Calibration procedure */

			/* TXDCOC->active; RCK->disable */
			write_rf(hw, 15, 0x9ABA8 );
			udelay(50);

			/* TXDCOC->disable; RCK->enable */
			write_rf(hw, 15, 0x3ABA8 );
			udelay(50);

			/* TXDCOC->disable; RCK->disable */
			write_rf(hw, 15, 0x12BAC );
			udelay(50);
			write_reg(BB_GENERAL_CTL, ~BB_GENERAL_CTL_MAX_GAIN_MASK, op_and);
			write_reg(BB_GENERAL_CTL, BB_GENERAL_CTL_DEFAULT_MAX_GAIN_A, op_or);
			break;
		}
	} else {
		printk(KERN_WARNING PIPER_DRIVER_NAME
		       ": undefined rf transceiver!\n");
	}
}

static int al7230_rf_stop(struct ieee80211_hw *hw)
{
	return 0;
}

static const unsigned int max_rate_index[] = {
        0,                         // not used
        MAX_BG_RATE_INDEX,         // B-1   (2412 MHz)   1
        MAX_BG_RATE_INDEX,         // B-2   (2417 MHz)   2
        MAX_BG_RATE_INDEX,         // B-3   (2422 MHz)   3
        MAX_BG_RATE_INDEX,         // B-4   (2427 MHz)   4
        MAX_BG_RATE_INDEX,         // B-5   (2432 MHz)   5
        MAX_BG_RATE_INDEX,         // B-6   (2437 MHz)   6
        MAX_BG_RATE_INDEX,         // B-7   (2442 MHz)   7
        MAX_BG_RATE_INDEX,         // B-8   (2447 MHz)   8
        MAX_BG_RATE_INDEX,         // B-9   (2452 MHz)   9
        MAX_BG_RATE_INDEX,         // B-10  (2457 MHz)  10
        MAX_BG_RATE_INDEX,         // B-11  (2462 MHz)  11
        MAX_BG_RATE_INDEX,         // B-12  (2467 MHz)  12
        MAX_BG_RATE_INDEX,         // B-13  (2472 MHz)  13
        MAX_BG_RATE_INDEX,         // B-14  (2484 MHz)  14
        0,                         // reserved for future b/g expansion 15
        0,                         // reserved for future b/g expansion 16
        _18MBPS_A_RATE_INDEX,      // L-184 (4920 MHz)  17
        _18MBPS_A_RATE_INDEX,      // L-188 (4940 MHz)  18
        _18MBPS_A_RATE_INDEX,      // L-192 (4960 MHz)  19
        _18MBPS_A_RATE_INDEX,      // L-196 (4980 MHz)  20
        _24MBPS_A_RATE_INDEX,      // A-8   (5040 MHz)  21
        _24MBPS_A_RATE_INDEX,      // A-12  (5060 MHz)  22
        _24MBPS_A_RATE_INDEX,      // A-16  (5080 MHz)  23
        _24MBPS_A_RATE_INDEX,      // A-34  (5170 MHz)  24
        _24MBPS_A_RATE_INDEX,      // A-36  (5180 MHz)  25
        _48MBPS_A_RATE_INDEX,      // A-38  (5190 MHz)  26
        _48MBPS_A_RATE_INDEX,      // A-40  (5200 MHz)  27
        _48MBPS_A_RATE_INDEX,      // A-42  (5210 MHz)  28
        _48MBPS_A_RATE_INDEX,      // A-44  (5220 MHz)  29
        _48MBPS_A_RATE_INDEX,      // A-46  (5230 MHz)  30
        _48MBPS_A_RATE_INDEX,      // A-48  (5240 MHz)  31
        _48MBPS_A_RATE_INDEX,      // A-52  (5260 MHz)  32
        _48MBPS_A_RATE_INDEX,      // A-56  (5280 MHz)  33
        _48MBPS_A_RATE_INDEX,      // A-60  (5300 MHz)  34
        _48MBPS_A_RATE_INDEX,      // A-64  (5320 MHz)  35
        _48MBPS_A_RATE_INDEX,      // A-100 (5500 MHz)  36
        _48MBPS_A_RATE_INDEX,      // A-104 (5520 MHz)  37
        _48MBPS_A_RATE_INDEX,      // A-108 (5540 MHz)  38
        _48MBPS_A_RATE_INDEX,      // A-112 (5560 MHz)  39
        _48MBPS_A_RATE_INDEX,      // A-116 (5580 MHz)  40
        _48MBPS_A_RATE_INDEX,      // A-120 (5600 MHz)  41
        _48MBPS_A_RATE_INDEX,      // A-124 (5620 MHz)  42
        _54MBPS_A_RATE_INDEX,      // A-128 (5640 MHz)  43
        _54MBPS_A_RATE_INDEX,      // A-132 (5660 MHz)  44
        _54MBPS_A_RATE_INDEX,      // A-136 (5680 MHz)  45
        _54MBPS_A_RATE_INDEX,      // A-140 (5700 MHz)  46
        _54MBPS_A_RATE_INDEX,      // A-149 (5745 MHz)  47
        _54MBPS_A_RATE_INDEX,      // A-153 (5765 MHz)  48
        _54MBPS_A_RATE_INDEX,      // A-157 (5785 MHz)  49
        _54MBPS_A_RATE_INDEX,      // A-161 (5805 MHz)  50
        _54MBPS_A_RATE_INDEX,      // A-165 (5825 MHz)  51
};


static void getOfdmBrs(int channelIndex, u64 brsBitMask, unsigned int *ofdm, unsigned int *psk)
{
	/*
	 * brsBitMask is a bit mask into the al7230_bg_rates array.  Bit 0 refers
	 * to the first entry in the array, bit 1 the second, and so on.  The first
	 * 4 bits/array entries refer to the PSK bit rates we support, the next 8
	 * bits/array entries refer to the OFDM rates we support.  So the PSK BRS
	 * mask is bits 0-3, the OFDM bit mask is bits 4-11.
	 */

	if (getBand(channelIndex) == IEEE80211_BAND_2GHZ)
	{
		*psk = brsBitMask & 0xf;
		*ofdm = (brsBitMask & 0xff0) >> 4;
	}
	else
	{
	    unsigned int max_rate_mask = 0xff;

#ifdef CONFIG_MACH_CCW9P9215JS
        /*
         * Set BRS mask on wi9p to limit rates that we will send ACK's at.
         */
	    max_rate_mask = max_rate_mask >> (7 - max_rate_index[channelIndex]);
#endif
		*psk = 0;
		*ofdm = (brsBitMask & max_rate_mask);
	}
}

static struct ieee80211_supported_band al7230_bands[] = {
	{
        	.band = IEEE80211_BAND_2GHZ,
        	.n_channels = ARRAY_SIZE(al7230_bg_channels),
        	.n_bitrates = ARRAY_SIZE(al7230_bg_rates),
        	.channels = (struct ieee80211_channel *) al7230_bg_channels,
        	.bitrates = (struct ieee80211_rate *) al7230_bg_rates,
	},
	{
        	.band = IEEE80211_BAND_5GHZ,
        	.n_channels = ARRAY_SIZE(al7230_a_channels),
        	.n_bitrates = ARRAY_SIZE(al7230_a_rates),
        	.channels = (struct ieee80211_channel *) al7230_a_channels,
        	.bitrates = (struct ieee80211_rate *) al7230_a_rates,
	},
};

static const struct ieee80211_rate *getRate(unsigned int rateIndex)
{
    return &al7230_bg_rates[rateIndex];
}


/*
 * This routine can power up or power down the airoha transceiver.
 * When the transceiver is powered back up, you must delay 1 ms and
 * then call the set channel routine to make it operational again.
 */
static void power_on(struct ieee80211_hw *hw, bool want_power_on)
{
    if (want_power_on) {
		write_rf(hw, 15, 0x1ABA8 ); /* this is actually for 2 Ghz */
    } else {
        write_rf(hw, 15, 0x1ABAE );
    }
}

static void al7230_set_hw_info(struct ieee80211_hw *hw, int channel,
							   u16 hw_platform_code)
{
    struct piper_priv *priv = hw->priv;

	priv->rf->hw_revision = hw_platform_code & WCD_HW_REV_MASK;
	priv->rf->hw_platform = (hw_platform_code & WCD_PLATFORM_MASK);

	set_hw_specific_parameters(hw, getBand(channel), priv->rf->hw_revision, priv->rf->hw_platform);
}

/*
 * This routine returns the highest transmit rate that the hardware
 * can support and still generate a high quality signal.
 */
const struct ieee80211_rate *al7230_rf_get_max_rate(unsigned int hw_platform, unsigned int hw_revision, unsigned int channel_index)
{
    const struct ieee80211_rate *result = NULL;

    (void) hw_revision;

    if (hw_platform == WCD_CCW9P_PLATFORM) {
        /*
         * If a Wi9p, then look up the maximum rate from the table
         * above.
         */
		if (getBand(channel_index) == IEEE80211_BAND_2GHZ) {
		    result = &al7230_bg_rates[max_rate_index[channel_index]];
		} else {
		    result = &al7230_a_rates[max_rate_index[channel_index]];
		}
    } else {
        /*
         * For any other platform, use the maximum rate advertised
         * for the channel.
         */
		if (getBand(channel_index) == IEEE80211_BAND_2GHZ) {
		    result = &al7230_bg_rates[MAX_BG_RATE_INDEX];
		} else {
		    result = &al7230_a_rates[_54MBPS_A_RATE_INDEX];
		}
	}

    return result;
}


struct digi_rf_ops al7230_rf_ops = {
	.name			= "Airoha 7230",
	.init			= InitializeRF,
	.stop			= al7230_rf_stop,
	.set_chan		= al7230_rf_set_chan,
	.set_chan_no_rx	= al7230_rf_set_chan_no_rx,
	.set_pwr		= al7230_set_txpwr,
	.set_pwr_index		= al7230_set_power_index,
	.set_hw_info    = al7230_set_hw_info,
	.channelChangeTime	= CHANNEL_CHANGE_TIME,
	.maxSignal		= MAX_SIGNAL_IN_DBM,
	.getOfdmBrs		= getOfdmBrs,
	.getBand		= getBand,
	.getFrequency		= getFrequency,
	.getRate		= getRate,
	.bands			= al7230_bands,
	.power_on       = power_on,
	.n_bands		= ARRAY_SIZE(al7230_bands),
    .hw_revision    = WCD_HW_REV_A,
    .hw_platform    = WCD_CCW9P_PLATFORM,
    .getMaxRate     = al7230_rf_get_max_rate,
};
EXPORT_SYMBOL_GPL(al7230_rf_ops);

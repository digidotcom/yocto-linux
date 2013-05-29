/*
 * piper.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <net/mac80211.h>
#include <linux/usb.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <asm/gpio.h>
#include <linux/timer.h>

#include "pipermain.h"
#include "mac.h"
#include "phy.h"
#include "airoha.h"
#include "airohaCalibration.h"
#include "piperDsp.h"
#include "piperMacAssist.h"
#include "digiPs.h"

#define WANT_AIROHA_CALIBRATION     (1)
#define WANT_DEBUG_COMMANDS			(1)


static void piper_clear_irq_mask(struct piper_priv *piperp, unsigned int bits)
{
	piperp->ac->wr_reg(piperp, BB_IRQ_MASK, ~bits, op_and);
}

static void piper_set_irq_mask(struct piper_priv *piperp, unsigned int bits)
{
	piperp->ac->wr_reg(piperp, BB_IRQ_MASK, bits, op_or);
}

/* Generate a random number */
static int local_rand(void)
{
	static unsigned long next = 1;

	/* RAND_MAX assumed to be 32767 */
	next = next * 1103515245 + 12345;
	return((unsigned)(next/65536) % 32768);
}

/*
 * Load the MAC Assist firmware into the chip. This is done by setting a bit
 * in the control register to enable MAC Assist firmware download, and then
 * writing the firmware into the data FIFO.
 */
void piper_load_mac_firmware(struct piper_priv *piperp)
{
	unsigned int i;

	printk(KERN_DEBUG PIPER_DRIVER_NAME ": loading MAC Assist firmware\n");

	/* Zero out MAC assist SRAM (put into known state before enabling MAC assist) */
	for (i = 0; i < 0x100; i += 4)
		piperp->ac->wr_reg(piperp, i, 0, op_write);

	/* Enable download the MAC Assist program RAM */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, BB_GENERAL_CTL_FW_LOAD_ENABLE, op_or);

	/* load MAC Assist data */
	for (i = 0; i < piper_macassist_data_len; i++)
		piperp->ac->wr_reg(piperp, BB_DATA_FIFO, piper_wifi_macassist_ucode[i],
				   op_write);

	/* disable MAC Assist download */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~BB_GENERAL_CTL_FW_LOAD_ENABLE, op_and);
}

/*
 * Load the DSP firmware into the chip.  This is done by setting a bit
 * in the control register to enable DSP firmware download, and then
 * writing the firmware into the data FIFO.
 */
void piper_load_dsp_firmware(struct piper_priv *piperp)
{
	unsigned int i;

	printk(KERN_DEBUG PIPER_DRIVER_NAME ": loading DSP firmware\n");

	/* Enable load of DSP firmware */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, BB_GENERAL_CTL_DSP_LOAD_ENABLE, op_or);

	/* load DSP data */
	for (i = 0; i < piper_dsp_data_len; i++)
		piperp->ac->wr_reg(piperp, BB_DATA_FIFO, piper_wifi_dsp_ucode[i],
				   op_write);

	/* Disable load of DSP firmware */
	udelay(10);
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~BB_GENERAL_CTL_DSP_LOAD_ENABLE, op_and);

	/* Let her rip */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, BB_GENERAL_CTL_MAC_ASSIST_ENABLE, op_or);
}


/*
 * This routine corrects a bug in the Piper chip where internal clocks would
 * be out of sync with each other and cause the chip to generate noise spikes.
 * This problem should be fixed in the next chip (Chopper).
 *
 * I'm not sure exactly what this code is doing.  It comes straight from the
 * guy who designed the chip.
 */
int piper_spike_suppression(struct piper_priv *piperp, bool retry)
{
	int timeout1 = 300, timeout2 = 30000;
	int ret = 0;

	/*
	 * Initial timing measurement to avoid spike
	 * The new "magic" value is 0x63 at address 0xA62.  Bit-0 indicates the
	 * timing measurement is complete.  Bit-1 indicates that a second timing
	 * measurment was performed.  The upper nibble is the timing measurement
	 * value. This code should eliminate the possibility of spikes at the
	 * beginning of all PSK/CCK frames and eliminate the spikes at the end of
	 * all PSK (1M, 2M) frames.
	 */

	/* reset the timing value */
	piperp->ac->wr_reg(piperp, MAC_STATUS, 0xffff00ff, op_and);

	while ((piperp->ac->rd_reg(piperp, MAC_STATUS) & 0x0000ff00) != 0x00006300) {

		/* reset the timing value */
		piperp->ac->wr_reg(piperp, MAC_STATUS, 0xffff00ff, op_and);

		/* issue WiFi soft reset */
		piperp->ac->wr_reg(piperp, BB_GENERAL_STAT, 0x40000000, op_write);

		/* Set TX_ON Low */
		piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0xffffff3f, op_and);
		piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0x00000080, op_or);

		/* Set PA_2G Low */
		piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0xfffff0ff, op_and);
		piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0x00000a00, op_or);

		/* Set RX_ON low  */
		piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0xcfffffff, op_and);
		piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0x20000000, op_or);

		/* start the WiFi mac & dsp */
		piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x37720820, op_write);
		timeout1 = 500;

		/* Wait for timing measurement to finish */
		while ((piperp->ac->rd_reg(piperp, MAC_STATUS) & 0x00000100) != 0x00000100) {
			udelay(2);
			timeout1--;
			if (!timeout1)
				break;
		}

		timeout2--;
		if (!timeout2) {
			ret = -EIO;
			break;
		}

		if (!retry) {
			ret = -EIO;
			break;
		}
	}

	/* Set TX_ON/RXHP_ON and RX to normal wifi, restore the reset value to HW_OUT_CTRL */
	piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0x1, op_write);

	return ret;
}
EXPORT_SYMBOL_GPL(piper_spike_suppression);

void piper_reset_mac(struct piper_priv *piperp)
{
	int i;

	/* set the TX-hold bit */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x37720080, op_write);

	/* clear the TX-FIFO memory */
	for (i = 0; i < 448; i++)
		piperp->ac->wr_reg(piperp, BB_DATA_FIFO, 0, op_write);

	/* reset the TX-FIFO */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x377200C0, op_write);

	/* release the TX-hold and reset */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x37720000, op_write);

/*	iowrite32(ioread32(piperp->vbase + MAC_STATUS) & ~0x40000000,
		  piperp->vbase + BB_GENERAL_STAT);*/
	mdelay(1);
}

/*
 * Load the MAC address into the chip. Use the value stored in the
 * environment, if there is one, otherwise use the default value.
 */
void piper_set_macaddr(struct piper_priv *piperp)
{
	/* Default MAC Addr used if the nvram parameters are corrupted */
	u8 mac[6] = {0x00, 0x04, 0xf3, 0x11, 0x43, 0x35};
	u8 *pmac = piperp->pdata->macaddr;
	int i;
    static bool firstTime = true;

	for (i = 0; i < 6; i++) {
		if (*(pmac + i) != 0xff)
			break;
		if (i == 5) {
			/* There is a problem with the parameters, use default */
			if (firstTime) {
    			printk(KERN_INFO PIPER_DRIVER_NAME
    				": invalid mac address, using default\n");
    		}
			memcpy(piperp->pdata->macaddr, mac, sizeof(piperp->pdata->macaddr));
		}
	}

	firstTime = false;
	memcpy(piperp->hw->wiphy->perm_addr, piperp->pdata->macaddr,
	       sizeof(piperp->hw->wiphy->perm_addr));

	/* configure ethernet address */
	piperp->ac->wr_reg(piperp, MAC_STA_ID0, *(pmac + 3) | *(pmac + 2) << 8 |
			   *(pmac + 1) << 16 | *(pmac + 0) << 24, op_write);
	piperp->ac->wr_reg(piperp, MAC_STA_ID1, *(pmac + 5) << 16 | *(pmac + 4) << 24,
			   op_write);
}
EXPORT_SYMBOL_GPL(piper_set_macaddr);


/* Configure the H/W with the antenna settings */
static int piper_set_antenna(struct piper_priv *piperp, enum antenna_select sel)
{
	if (sel == ANTENNA_BOTH) {
		piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
				   BB_GENERAL_CTL_ANT_DIV, op_or);
		piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
				   ~BB_GENERAL_CTL_ANT_SEL, op_and);
	} else {
		piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
				   ~BB_GENERAL_CTL_ANT_DIV, op_and);
		/* select the antenna if !diversity */
		if (sel == ANTENNA_1)
			piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
					   ~BB_GENERAL_CTL_ANT_SEL, op_and);
		else
			piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
					   BB_GENERAL_CTL_ANT_SEL, op_or);
	}

	/* select which antenna to transmit on */
	piperp->ac->wr_reg(piperp, BB_RSSI, ~BB_RSSI_ANT_MASK, op_and);
	if (sel == ANTENNA_BOTH)
		piperp->ac->wr_reg(piperp, BB_RSSI, BB_RSSI_ANT_DIV_MAP, op_or);
	else
		piperp->ac->wr_reg(piperp, BB_RSSI, BB_RSSI_ANT_NO_DIV_MAP, op_or);

	return 0;
}

/*
 * Compute a beacon backoff time as described in section 11.1.2.2 of 802.11 spec.
 *
 */
static u16 get_next_beacon_backoff(void)
{
#define MAX_BEACON_BACKOFF      (2 * ASLOT_TIME * DEFAULT_CW_MIN)

	/*
	* We shift the result of local_rand() by 4 bits because the notes
	* for the algorithm say that we shouldn't rely on the last few
	* bits being random.  Other than that, we just take the random
	* value and make sure it is less than MAX_BEACON_BACKOFF.
	*/
	return (local_rand() >> 4) % MAX_BEACON_BACKOFF;
}

static int load_beacon(struct piper_priv *digi, unsigned char *buffer,
                        unsigned int length)
{
    return digi->ac->wr_fifo(digi, BEACON_FIFO, buffer, length);
}

static int piper_init_rx_tx(struct piper_priv *piperp)
{
	tasklet_init(&piperp->rx_tasklet, piper_rx_tasklet, (unsigned long)piperp);
	tasklet_disable(&piperp->rx_tasklet);
	piperp->expectingAck = false;

	spin_lock_init(&piperp->tx_tasklet_lock);
	spin_lock_init(&piperp->tx_queue_lock);
	piperp->tx_tasklet_running = false;
    memset(&piperp->tx_queue, 0, sizeof(piperp->tx_queue));
    piperp->tx_queue_head = 0;
    piperp->tx_queue_tail = 0;
    piperp->tx_queue_count = 0;
	tasklet_init(&piperp->tx_tasklet, piper_tx_tasklet, (unsigned long)piperp);
	tasklet_disable(&piperp->tx_tasklet);

	return 0;
}

static void piper_free_rx_tx(struct piper_priv *piperp)
{
	tasklet_disable(&piperp->rx_tasklet);
	tasklet_kill(&piperp->rx_tasklet);
	tasklet_disable(&piperp->tx_tasklet);
	tasklet_kill(&piperp->tx_tasklet);
	piper_empty_tx_queue(piperp);
}

/*
 * This function sets the tracking control according to a channel's
 * frequency.
 */
static int piper_set_tracking_constant(struct piper_priv *piperp, unsigned megahertz)
{
	piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, ~TRACK_CONSTANT_MASK, op_and);
	if (megahertz < 4920)
	{
		piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, TRACK_BG_BAND, op_or);
	}
	else if (megahertz <= 4980)
	{
		piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, TRACK_4920_4980_A_BAND, op_or);
	}
	else if (megahertz <= 5350)
	{
		piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, TRACK_5150_5350_A_BAND, op_or);
	}
	else if (megahertz <= 5725)
	{
		piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, TRACK_5470_5725_A_BAND, op_or);
	}
	else
	{
		piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, TRACK_5725_5825_A_BAND, op_or);
	}

	return 0;
}

/*
 * This function is called to set the value of the B_TX_GAIN field of the
 * HW_CONF1 mac register. This register must be set to different values depending
 * on the H/W revision of the board due to changes in the board design.
 */
static unsigned int get_b_tx_gain(struct piper_priv *piperp)
{
	unsigned int tx_gain = 0;

#ifdef CONFIG_MACH_CCW9P9215JS
	tx_gain = TRACK_TX_B_GAIN_NORMAL;
#else
	u16 hw_revision = piperp->pdata->wcd.header.hw_platform & WCD_HW_REV_MASK;

	switch (hw_revision) {
		case WCD_HW_REV_PROTOTYPE:
		case WCD_HW_REV_PILOT:
			tx_gain = 0xc0000000;
			break;
		case WCD_HW_REV_A:
		default:
			tx_gain = 0x90000000;
			break;
	}
#endif

	return tx_gain;
}


static int piper_init_hw(struct piper_priv *piperp, enum ieee80211_band band)
{
	int ret = 0;

	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, BB_GENERAL_CTL_INIT, op_write);

	/* Initialize baseband general control register for the specific transceiver */
	if (piperp->pdata->rf_transceiver == RF_AIROHA_7230) {
		if (band == IEEE80211_BAND_2GHZ) {
			piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, GEN_INIT_AIROHA_24GHZ, op_write);
			piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, 0xff00ffff, op_and);
			piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, TRACK_BG_BAND, op_or);
			digi_dbg("piper_init_hw Initialized for band B / BG\n");
		} else {
			piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, GEN_INIT_AIROHA_50GHZ, op_write);
			piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, 0xff00ffff, op_and);
			piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, TRACK_5150_5350_A_BAND, op_or);
			digi_dbg("piper_init_hw Initialized for band A\n");
		}
		piperp->ac->wr_reg(piperp, BB_CONF_2, 0x09325ad4, op_write);
		/* Initialize the SPI word length */
		piperp->ac->wr_reg(piperp, BB_SPI_CTRL, SPI_INIT_AIROHA, op_write);
	} else if (piperp->pdata->rf_transceiver == RF_AIROHA_2236) {
		piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, GEN_INIT_AIROHA_24GHZ, op_write);
		piperp->ac->wr_reg(piperp, BB_CONF_2, 0x09325ad4, op_write);
		piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, 0xff00ffff, op_and);
		piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, TRACK_BG_BAND, op_or);
		/* Initialize the SPI word length */
		piperp->ac->wr_reg(piperp, BB_SPI_CTRL, SPI_INIT_AIROHA2236, op_write);
	} else {
		printk(KERN_WARNING PIPER_DRIVER_NAME ": undefined rf transceiver!\n");
		return -EINVAL;
	}

	/*
	 *Clear the Intretupt Mask Register before enabling external intretupts.
	 * Also clear out any status bits in the Intretupt Status Register.
	 */
	piperp->ac->wr_reg(piperp, BB_IRQ_MASK, 0, op_write);
	piperp->ac->wr_reg(piperp, BB_IRQ_STAT, 0xff, op_write);

	/*
	 * If this firmware supports additional MAC addresses.
	 */
	if (((piperp->ac->rd_reg(piperp, MAC_STATUS) >> 16) & 0xff) >= 8) {
		/* Disable additional addresses to start with */
		piperp->ac->wr_reg(piperp, MAC_CTL, ~MAC_CTL_MAC_FLTR, op_and);
		piperp->ac->wr_reg(piperp, MAC_STA2_ID0, 0, op_write);
		piperp->ac->wr_reg(piperp, MAC_STA2_ID1, 0, op_write);
		piperp->ac->wr_reg(piperp, MAC_STA3_ID0, 0, op_write);
		piperp->ac->wr_reg(piperp, MAC_STA3_ID1, 0, op_write);
	}
	/* TODO:  Set this register programatically */
	piperp->ac->wr_reg(piperp, MAC_DTIM_PERIOD, 0x0, op_write);

	/*
	 * Note that antenna diversity will be set by hw_start, which is the
	 * caller of this function.
	 */

	/* reset RX and TX FIFOs */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, BB_GENERAL_CTL_RXFIFORST
			 | BB_GENERAL_CTL_TXFIFORST, op_or);
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~(BB_GENERAL_CTL_RXFIFORST
						 | BB_GENERAL_CTL_TXFIFORST), op_and);

	piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, 0xC043002C, op_write);
	piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, ~TRACK_TX_B_GAIN_MASK, op_and);
	piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, get_b_tx_gain(piperp), op_or);

	/* Initialize RF transceiver */
	piperp->rf->init(piperp->hw, band);
	piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0x04000001, op_or);
	piperp->ac->wr_reg(piperp, MAC_CFP_ATIM, 0x0, op_write);
	piperp->ac->wr_reg(piperp, BB_GENERAL_STAT, ~(BB_GENERAL_STAT_DC_DIS
						  | BB_GENERAL_STAT_SPRD_DIS), op_and);
	piperp->ac->wr_reg(piperp, BB_GENERAL_STAT, ~(BB_GENERAL_STAT_SRC_DIS
						  | BB_GENERAL_STAT_DLL_DIS), op_and);

	piperp->ac->wr_reg(piperp, MAC_SSID_LEN, (MAC_OFDM_BRS_MASK | MAC_PSK_BRS_MASK),
			   op_write);

	/*
	 * Set BSSID to the broadcast address so that we receive all packets.  The stack
	 * will set a real BSSID when it's ready.
	 */
	piperp->ac->wr_reg(piperp, MAC_BSS_ID0, 0xffffffff, op_write);
	piperp->ac->wr_reg(piperp, MAC_BSS_ID1, 0xffffffff, op_write);

	piperp->ps.poweredDown = false;

#if WANT_AIROHA_CALIBRATION
	digi_dbg("Calling digiWifiInitCalibration()\n");
	digiWifiInitCalibration(piperp);
#endif

	return ret;
}

static int piper_deinit_hw(struct piper_priv *piperp)
{
	int ret = 0;
#if WANT_AIROHA_CALIBRATION
	digi_dbg("Calling digiWifiDeInitCalibration()\n");
	digiWifiDeInitCalibration(piperp);
#endif

	return ret;
}


static void adjust_max_agc(struct piper_priv *piperp, unsigned int rssi, _80211HeaderType *header)
{
#define LOWEST_MAXAGC_AL2236        0x76
#define HIGHEST_MAXAGC_AL2236       0x7B
#define HIGHEST_MAXAGC_AL7230_24GHZ       0x7c
#define LOWEST_MAXAGC_AL7230_24GHZ        0x76
#define HIGHEST_MAXAGC_AL7230_50GHZ       0x79
#define LOWEST_MAXAGC_AL7230_50GHZ        0x73
#define RSSI_AVG_COUNT  8

    unsigned char maxgain = 0;
    static unsigned char lowest = 0, highest = 0;
    static int k=0, j=0, i =0, tempRssi=0;
	static unsigned int savedRSSI[RSSI_AVG_COUNT]; /****/

	savedRSSI[k % RSSI_AVG_COUNT] = rssi;
	if (   (piperp->pdata->rf_transceiver == RF_AIROHA_2236)
		|| (piperp->pdata->rf_transceiver == RF_AIROHA_7230)) {

	    if (piperp->pdata->rf_transceiver == RF_AIROHA_2236)
	    {
	        lowest = LOWEST_MAXAGC_AL2236;
	        highest = HIGHEST_MAXAGC_AL2236;
	    }
	    else
	    {

	        if (piperp->rf->getBand(piperp->channel) == IEEE80211_BAND_5GHZ) {
	            highest = HIGHEST_MAXAGC_AL7230_50GHZ;
	            lowest = LOWEST_MAXAGC_AL7230_50GHZ;
	        }
	        else {
	            highest = HIGHEST_MAXAGC_AL7230_24GHZ;
	            lowest = LOWEST_MAXAGC_AL7230_24GHZ;
	        }
	    }

	    if (piperp->areWeAssociated)
	    {

	        if (   (piperp->if_type == NL80211_IFTYPE_ADHOC)
	        	|| (piperp->if_type == NL80211_IFTYPE_MESH_POINT))
	        {
	            //Monitor the receive signal strength from Ad-hoc network
	            if (memcmp (piperp->bssid, header->addr3, sizeof(piperp->bssid)) == 0)
	            {
	                /* we don't do avareging on all the signals here because it may come from different
	                 * unit in that Ad-hoc network. Instead, we do avareging on the signals with higher rssi
	                 */

	                if ((rssi + 4) > lowest)
	                {
	                    k++;
	                    tempRssi += rssi;

	                    if (k >= RSSI_AVG_COUNT)
	                    {
	                        maxgain = (((tempRssi/k) + 4) > highest)? highest : ((tempRssi/k) + 4) ;
	                        k = 0;
	                        tempRssi = 0;
	                        i =0;
	                    }
	                }
	                else
	                {
	                    i++;
	                    if (i >= (RSSI_AVG_COUNT*4))
	                    {
	                        maxgain = lowest;
	                        i = 0;
	                    }

	                }
	            }
	        }
	        else
	        {
	            //Monitor the receive signal strength from the frames we received from the associated AP
	            if (memcmp (piperp->bssid, header->addr2, sizeof(piperp->bssid)) == 0)
	            {
	                //averaging all the signals because they come from the same AP
	                k++;
	                tempRssi += rssi;

	                if (k >= RSSI_AVG_COUNT*2)
	                {
	                    if (((tempRssi/k) + 4) > lowest)
	                        maxgain = (((tempRssi/k) + 4) > highest)? highest : ((tempRssi/k) + 4) ;
	                    else
	                        maxgain = lowest;

	                    k = 0;
	                    tempRssi = 0;
	                }
	            }
	        }
	        j = 0;
	    }
	    else
	    {
	        j++;
	        if (j >= (RSSI_AVG_COUNT*4))
	        {
	            maxgain = highest;
	            j = 0;
	        }
	        k = 0;
	        tempRssi = 0;
	    }

	    if(   (maxgain != 0)
	       && (maxgain != ((piperp->ac->rd_reg(piperp, BB_GENERAL_CTL) & BB_GENERAL_CTL_MAX_GAIN_MASK) >> 16)))
	    {
	    	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~BB_GENERAL_CTL_MAX_GAIN_MASK, op_and);
	    	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, (maxgain << 16) & BB_GENERAL_CTL_MAX_GAIN_MASK, op_or);
	    }
	}
}


/* Make sure all keys are disabled when we start */
static void piper_init_keys(struct piper_priv *piperp)
{
	unsigned int i;

	for (i = 0; i < PIPER_MAX_KEYS; i++)
		piperp->key[i].valid = false;

	piperp->aes_key_count = 0;
}

static void tx_timer_timeout(unsigned long arg)
{
	struct piper_priv *piperp = (struct piper_priv *) arg;

	tasklet_hi_schedule(&piperp->tx_tasklet);
}

/* sysfs entries to get/set antenna mode */
static ssize_t show_antenna_sel(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct piper_priv *piperp = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", piperp->antenna == ANTENNA_BOTH ? "diversity" :
		       piperp->antenna == ANTENNA_1 ? "primary" : "secondary");
}

static ssize_t store_antenna_sel(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct piper_priv *piperp = dev_get_drvdata(dev);
	enum antenna_select ant;
	size_t len = count;
	int ret;

	ant = piperp->antenna;

	if (buf[count - 1] == '\n')
		len--;

	/* TODO check also string length */
	if (!strncmp("diversity", buf, len))
		ant = ANTENNA_BOTH;
	else if (!strncmp("primary", buf, len))
		ant = ANTENNA_1;
	else if (!strncmp("secondary", buf, len))
		ant = ANTENNA_2;

	if (ant != piperp->antenna) {
		if ((ret = piperp->set_antenna(piperp, ant)) != 0) {
			printk(KERN_WARNING PIPER_DRIVER_NAME
			       ": error setting antenna to %d (err: %d)\n", ant, ret);
		} else
			piperp->antenna = ant;
	}

	return count;
}
static DEVICE_ATTR(antenna_sel, S_IWUSR | S_IRUGO, show_antenna_sel, store_antenna_sel);

static ssize_t show_power_duty(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct piper_priv *piperp = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", piperp->power_duty);
}

static ssize_t store_power_duty(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
#define MINIMUM_DUTY_CYCLE	(33)
#define LIMIT_LINEAL_DUTY_CYCLE	(75)
	struct piper_priv *piperp = dev_get_drvdata(dev);
	int pw_duty;
	ssize_t ret = -EINVAL;

	ret = sscanf(buf, "%d\n", &pw_duty);
	if (ret > 0) {
		if (pw_duty < MINIMUM_DUTY_CYCLE) {
			piperp->power_duty = MINIMUM_DUTY_CYCLE;
		} else if (pw_duty > LIMIT_LINEAL_DUTY_CYCLE && pw_duty < 100) {
			piperp->power_duty = LIMIT_LINEAL_DUTY_CYCLE;
		} else if (pw_duty > 100) {
			pw_duty = 100;
			piperp->power_duty = 100;
		} else if (pw_duty == 100 ||
			(pw_duty >= MINIMUM_DUTY_CYCLE && pw_duty <= LIMIT_LINEAL_DUTY_CYCLE)) {
			piperp->power_duty = pw_duty;
		}

		if (piperp->areWeAssociated) {
			if ((pw_duty == 100) && (piperp->ps.mode == PS_MODE_LOW_POWER)) {
				/*
				 * Turn duty cycling off if 100% duty cycle was selected and
				 * duty cycling was on.
				 */
				piper_ps_set(piperp, false);
			} else if ((pw_duty != 100) && (piperp->ps.mode != PS_MODE_LOW_POWER)) {
				/*
				 * Turn duty cycling on if a valid duty cycle was set and
				 * duty cycling was off.
				 */
				piper_ps_set(piperp, true);
			}
		}
	}


	return ret < 0 ? ret : count;
}
static DEVICE_ATTR(power_duty, S_IWUSR | S_IRUGO, show_power_duty, store_power_duty);

#if WANT_DEBUG_COMMANDS

static ssize_t show_debug_cmd(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct piper_priv *piperp = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n", piperp->debug_cmd);
}

static ssize_t store_debug_cmd(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct piper_priv *piperp = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;

	if (strlen(buf) < sizeof(piperp->debug_cmd))
	{
		if (strstr(buf, "cal_dump") != NULL) {
		    digiWifiCalibrationDumpNvram(piperp);
		    ret = 1;
		} else if (strstr(buf, "dump") != NULL) {
			digiWifiDumpRegisters(piperp, MAIN_REGS | MAC_REGS);
			ret = 1;
		} else if (strstr(buf, "ps_state") != NULL) {
			printk(KERN_ERR "rxTaskletRunning = %d, allowTransmits = %d, stopped_tx_queues = %d\n",
					piperp->ps.rxTaskletRunning, piperp->ps.allowTransmits, piperp->ps.stopped_tx_queues);
			ret = 1;
		} else if (strstr(buf, "rssi_dump") != NULL) {
		    spinlock_t lock;
		    unsigned long flags;
            unsigned int rssi;

	        spin_lock_init(&lock);
	        spin_lock_irqsave(&piperp->ps.lock, flags);
		    piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~BB_GENERAL_CTL_RX_EN, op_and);
		    udelay(15);
		    rssi = piperp->ac->rd_reg(piperp, BB_RSSI);
		    printk(KERN_ERR "\n**rssi = 0x%8.8X\n", rssi);
			digiWifiDumpRegisters(piperp, MAIN_REGS | MAC_REGS);
			ret = 1;
			spin_unlock_irqrestore(&lock, flags);
		} else {
			strcpy(piperp->debug_cmd, buf);
			piperp->debug_cmd[strlen(buf)-1] = 0;		/* truncate the \n */
		}
		ret = count;
	}

	return ret < 0 ? ret : count;
}
static DEVICE_ATTR(debug_cmd, S_IWUSR | S_IRUGO, show_debug_cmd, store_debug_cmd);

#endif

#ifdef CONFIG_PM

static int piper_suspend(struct platform_device *dev, pm_message_t state)
{
	struct piper_priv *piperp = platform_get_drvdata(dev);
	unsigned long flags;

	/* TODO, use in future the ps.lock instead of fully disabling interrupts here */
	piperp->power_save_was_on_when_suspended = (piperp->ps.mode == PS_MODE_LOW_POWER);
	if (piperp->power_save_was_on_when_suspended)
		piper_ps_set(piperp, false);
	mdelay(10);
	piper_sendNullDataFrame(piperp, true);
	ssleep(1);

	local_irq_save(flags);
	/*
	 * Save power save state and then make sure power save is turned off.
	 */
	piper_MacEnterSleepMode(piperp, true);
	local_irq_restore(flags);

	return 0;
}

static int piper_resume(struct platform_device *dev)
{
	struct piper_priv *piperp = platform_get_drvdata(dev);
	unsigned long flags;

	if (piperp->pdata->early_resume)
		piperp->pdata->early_resume(piperp);

	/* TODO, use in future the ps.lock instead of fully disabling interrupts here */
	local_irq_save(flags);
	piper_MacEnterActiveMode(piperp, true);
	if (piperp->tx_tasklet_running) {
	    tasklet_hi_schedule(&piperp->tx_tasklet);
	} else {
	    ieee80211_wake_queues(piperp->hw);
	}
	local_irq_restore(flags);

	/*
	 * Restore power save if it was on before
	 */
	if (piperp->power_save_was_on_when_suspended) {
		piper_ps_set(piperp, true);
	} else {
		piper_sendNullDataFrame(piperp, false);
	}

	return 0;
}
#else
#define piper_suspend	NULL
#define piper_resume	NULL
#endif

static int __devinit piper_probe(struct platform_device* pdev)
{
	struct piper_pdata *pdata = pdev->dev.platform_data;
	struct piper_priv *piperp;
	int ret = 0;

	if (!pdata) {
		return -EINVAL;
	}

	ret = piper_alloc_hw(&piperp, sizeof(*piperp));
	if (ret) {
		printk(KERN_ERR PIPER_DRIVER_NAME ": failed to alloc piper_priv\n");
		return ret;
	}

	piperp->ac = kzalloc(sizeof(struct access_ops), GFP_KERNEL);
	if (!piperp->ac){
		printk(KERN_ERR PIPER_DRIVER_NAME ": failed to alloc memory for ac struct\n");
		ret = -ENOMEM;
		goto error_alloc;
	}

	piperp->drv_name = PIPER_DRIVER_NAME;
	dev_set_drvdata(&pdev->dev, piperp);
	piperp->pdata = pdata;
	pdata->piperp = piperp;
	spin_lock_init(&piperp->ac->reg_lock);
	spin_lock_init(&piperp->aesLock);

	piperp->vbase = ioremap(pdev->resource[0].start,
				pdev[0].resource->end - pdev->resource[0].start);

	if (!piperp->vbase) {
		printk(KERN_ERR PIPER_DRIVER_NAME ": ioremap base %x, len %x error\n",
		       pdev->resource[0].start,	pdev[0].resource->end - pdev->resource[0].start);
		ret = -ENOMEM;
		goto error_remap;
	}

	piperp->pstats.tx_start_count = 0;
	piperp->pstats.tx_complete_count = 0;

	/*
	 * Platform initialization. This will initialize the hardware, including the load
	 * of the mac and dsp firmware into the piper chip
	 */
	if (pdata->init) {
		if ((ret = pdata->init(piperp)) != 0) {
			printk(KERN_ERR PIPER_DRIVER_NAME
				": platform init() returned error (%d)\n", ret);
			goto error_init;
		}
	}

	piper_ps_init(piperp);
	init_timer(&piperp->tx_timer);
	piperp->tx_timer.function = tx_timer_timeout;
	piperp->tx_timer.data = (unsigned long) piperp;
	piper_init_rx_tx(piperp);
	piper_init_keys(piperp);

	piperp->init_hw = piper_init_hw;
	piperp->deinit_hw = piper_deinit_hw;
	piperp->set_irq_mask_bit = piper_set_irq_mask;
	piperp->clear_irq_mask_bit = piper_clear_irq_mask;
	piperp->load_beacon = load_beacon;
	piperp->rand = local_rand;
	piperp->get_next_beacon_backoff = get_next_beacon_backoff;
	piperp->set_antenna = piper_set_antenna;
	piperp->set_tracking_constant = piper_set_tracking_constant;
	piperp->antenna = ANTENNA_1;
	piperp->adjust_max_agc = adjust_max_agc;

	/*
	 * Set the default duty cycle value.  Note that duty cycling
	 * is disabled reguardless of what this variable is set to until
	 * the user types "iwconfig wlan0 power on".  I just love the
	 * "power on" syntax to turn *down* the power.
	 */
	piperp->power_duty = 100;

	/* TODO this should be read earlier and actions should be taken
	 * based on different revisions at driver initialization or runtime */
	piperp->version = piperp->ac->rd_reg(piperp, BB_VERSION);

	piperp->irq = pdev->resource[1].start;
	piperp->tx_cts = false;
	piperp->beacon.loaded = false;
	piperp->beacon.enabled = false;
	piperp->beacon.weSentLastOne = false;

	ret = request_irq(piperp->irq, piper_irq_handler,
			  IRQF_TRIGGER_HIGH, PIPER_DRIVER_NAME,	piperp);
	if (ret) {
		printk(KERN_ERR PIPER_DRIVER_NAME ": unable to request irq %d (%d)",
			piperp->irq, ret);
		goto retor_irq;
	}

	disable_irq(piperp->irq);

	ret = piper_register_hw(piperp, &pdev->dev, &al7230_rf_ops);
	if (ret) {
		printk(KERN_ERR PIPER_DRIVER_NAME ": failed to register priv, ret=%d\n", ret);
		goto error_reg_hw;
	}

	if (pdata->late_init)
		pdata->late_init(piperp);

	ret = device_create_file(&pdev->dev, &dev_attr_antenna_sel);
	if (ret) {
		printk(KERN_ERR PIPER_DRIVER_NAME ": failed to create sysfs file\n");
		goto error_sysfs;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_power_duty);
	if (ret) {
		printk(KERN_ERR PIPER_DRIVER_NAME ": failed to create sysfs file\n");
		goto error_sysfs;
	}

	strcpy(piperp->debug_cmd, "off");
#if WANT_DEBUG_COMMANDS
	ret = device_create_file(&pdev->dev, &dev_attr_debug_cmd);
	if (ret) {
		printk(KERN_ERR PIPER_DRIVER_NAME ": failed to create sysfs file\n");
		goto error_sysfs;
	}
#endif

	printk(KERN_INFO PIPER_DRIVER_NAME ": driver loaded (fw ver = 0x%08x)\n",
		piperp->version);

	return 0;

error_sysfs:
	piper_unregister_hw(piperp);
error_reg_hw:
	piper_ps_deinit(piperp);
	piper_free_rx_tx(piperp);
retor_irq:
	free_irq(piperp->irq, piperp);
error_init:
	iounmap(piperp->vbase);
	piperp->vbase = NULL;
error_remap:
	release_resource(pdev->resource);
error_alloc:
	piper_free_hw(piperp);
	return ret;
}

static int __devexit piper_remove(struct platform_device *pdev)
{
	struct piper_priv *piperp = dev_get_drvdata(&pdev->dev);

	printk(KERN_DEBUG PIPER_DRIVER_NAME " %s\n", __func__);

	device_remove_file(&pdev->dev, &dev_attr_antenna_sel);
	device_remove_file(&pdev->dev, &dev_attr_power_duty);
#if WANT_DEBUG_COMMANDS
	device_remove_file(&pdev->dev, &dev_attr_debug_cmd);
#endif

	piper_ps_deinit(piperp);
	piper_unregister_hw(piperp);
	disable_irq(piperp->irq);
	piper_clear_irq_mask(piperp, 0xffffffff);
	free_irq(piperp->irq, piperp);
	piper_free_rx_tx(piperp);
	release_resource(pdev->resource);
	piper_free_hw(piperp);

	return 0;
}

/* describes the driver */
static struct platform_driver piper_driver = {
	.probe		= piper_probe,
	.remove		= __devexit_p(piper_remove),
	.suspend 	= piper_suspend,
	.resume		= piper_resume,
	.driver 	= {
		.name  = PIPER_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init piper_init_module(void)
{
	return platform_driver_register(&piper_driver);
}

static void __exit piper_exit_module(void)
{
	platform_driver_unregister(&piper_driver);
}

module_init(piper_init_module);
module_exit(piper_exit_module);

MODULE_DESCRIPTION("Digi Piper WLAN Driver");
MODULE_AUTHOR("Contact support@digi.com for questions on this code");
MODULE_VERSION(DRV_VERS);
MODULE_LICENSE("GPL");

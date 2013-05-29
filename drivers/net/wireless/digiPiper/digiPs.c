/*
 * digiPs.c
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/*
 * This file contains the routines that are related to transmitting
 * frames.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>

#include "pipermain.h"
#include "mac.h"
#include "phy.h"
#include "airoha.h"
#include "digiPs.h"

/*
 * Macro converts milliseconds to HZ.
 *
 * TODO:  Look for standard Linux version of this.
 */
#define MILLS_TO_JIFFIES(x)		((((x)*HZ) + 500) / 1000)




#define MINIMUM_SLEEP_PERIOD	(20)

/*
 * Amount of time before shutdown deadline to start the shutdown process.  This
 * should allow enough time to get one last frame out, which will be the
 * null-data frame notifying the AP that we are shutting down.
 */
#define PS_TRANSMITTER_SHUTDOWN_MS 	(10)

/*
 * Amount of time we will wake up before the next beacon.  We try to wake up before
 * the next beacon so that we don't miss it.
 */
#define PS_WAKE_BEFORE_BEACON_MS	(20)

/*
 * Minimum amount of time we we keep awake for.
 */
#define PS_MINUMUM_POWER_UP_PERIOD_MS	(10)

/*
 * Minimum amount of time we will sleep.  If we will end up sleeping
 * for less than this, then don't go to sleep.
 */
#define PS_MINIMUM_SLEEP_TIME (10)

/*
 * Length of one tick of the transmit clean up timer in ms.  This is
 * the minimum amount of time we will sleep for.
 */
#define PS_TRANSMIT_TIMER_TICK_MS	((1000/HZ) ? (1000/HZ) : 1)

/*
 * Length of time we will wait past the expected arrival of a beacon before assuming
 * that we missed it.
 */
#define PS_BEACON_TIMEOUT_MS		(40)


/*
 * Minimum beacon interval we will support for duty cycling.  There is so much overhead
 * in duty cycling that it doesn't make sense to do it for short beacon intervals.
 */
#define PS_MINIMUM_BEACON_INT		(100)

/*
 * Amount of time we will pause duty cycling for after receiving an event that suggests
 * wpa_supplicant is attempting to reassociate with an AP.
 */
#define PS_SCAN_DELAY				(5000)


// Powersave register index
#define	INDX_GEN_CONTROL    0	// General control
#define	INDX_GEN_STATUS	    1	// General status
#define	INDX_RSSI_AES		2	// RSSI and AES status
#define	INDX_INTR_MASK	    3	// Interrupt mask
#define INDX_SPI_CTRL       4	// RF SPI control
#define INDX_CONF1          5	// Configuration 1
#define INDX_CONF2          6	// Configuration 2
#define	INDX_AES_MODE		7	// ARS mode
#define INDX_OUT_CTRL       8	// Output control
#define INDX_MAC_CONTROL    9	// MAC control
#define INDX_STAID_1		10  // first part of stations ID
#define INDX_STAID_2		11  // 2nd part of station ID
#define INDX_BSSID_1		12  // 1st part of BSS ID
#define INDX_BSSID_2		13  // 2nd part of BSS ID
#define INDX_BRS_SSID		14  // BRS mask and SSID length
#define INDX_BACKOFF		15	// backoff
#define INDX_DTIM_LISTEN	16  // DTIM perido and listen interval
#define INDX_BEACON_INT		17	// beacon interval
#define INDX_MAC_CTL		18	// MAC control register
#define INDX_BEACON_MASK	19	// beacon mask and backoff
#define INDX_TOTAL          20

static u32 savedRegs[INDX_TOTAL];	// Used to save registers for sleep mode


/*
 * TODO: Delete this.
 */
struct ps_stats {
	unsigned int modeStart;
	unsigned int cycleStart;
	unsigned int receivedBeacons;
	unsigned int missedBeacons;
	unsigned int jiffiesOn;
	unsigned int jiffiesOff;
} stats;



static void ps_free_frame(struct ieee80211_hw *hw, struct sk_buff *skb)
{
	struct piper_priv *piperp = hw->priv;

	if (skb) {
		dev_kfree_skb(skb);
		piperp->ps.frames_pending--;
	}
}





#define ACK_SIZE		(14)	/* length of ACK in bytes */

// Length (in usecs) of a MAC frame of bytes at rate (in 500kbps units)
// not including SIFS and PLCP preamble/header
#define	CCK_DURATION(bytes, rate)		((16*(bytes)+(rate)-1)/(rate))

#define USE_SHORTPREAMBLE(is_1_Mbit, use_short_preamble)	((!is_1_Mbit) & use_short_preamble)

// Length (in usecs) of SIFS and PLCP preamble/header.
#define	PRE_LEN(is_1_Mbit, use_short_preamble)			(USE_SHORTPREAMBLE(is_1_Mbit, use_short_preamble) ? 106 : 202)

// Duration (in usecs) of an OFDM frame at rate (in 500kbps units)
// including SIFS and PLCP preamble/header
#define	OFDM_DURATION(bytes, rate)	(36 + 4*((4*(bytes)+(rate)+10)/(rate)))

static unsigned int getRateIndex(struct piper_priv *piperp)
{
	unsigned int rates = piperp->ac->rd_reg(piperp, MAC_SSID_LEN);
	unsigned int result = AIROHA_LOWEST_OFDM_RATE_INDEX;

	if (piperp->rf->getBand(piperp->channel) == IEEE80211_BAND_2GHZ) {
		if ((rates & MAC_PSK_BRS_MASK) != 0) {
			result = AIROHA_LOWEST_PSK_RATE_INDEX;
		}
	}

	return result;
}

static int getAckDuration(struct piper_priv *piperp)
{
	bool is_1_Mbit = (getRateIndex(piperp) == AIROHA_LOWEST_PSK_RATE_INDEX);
	int duration = 0;

	if (is_1_Mbit) {
		duration = CCK_DURATION(ACK_SIZE, 1);
	} else {
		duration = OFDM_DURATION(ACK_SIZE, 6);
	}

	duration += PRE_LEN(is_1_Mbit, piperp->use_short_preamble);

	return duration;
}


/*
 * This function is used to notify the AP about the current state of
 * power save.  One of the bits in the 802.11 header field is set to
 * indicate the status of power save.  This bit is actually set appropriately
 * for all frames sent, we just send a null data frame just to make
 * sure something is sent to the AP in a reasonable amount of time.
 */
/*
 * Possible values for is_power_management_on argument
 */
#define POWERING_DOWN		(true)
#define POWERED_UP			(false)
void piper_sendNullDataFrame(struct piper_priv *piperp, bool is_power_management_on)
{
	struct sk_buff *skb = NULL;
	_80211HeaderType *header;
	struct ieee80211_tx_info *tx_info;

	if ((piperp->bssid[0] == 0) && (piperp->bssid[1] == 0)
	    && (piperp->bssid[2] == 0) && (piperp->bssid[3] == 0)
	    && (piperp->bssid[4] == 0) && (piperp->bssid[5] == 0)) {
		goto piper_sendNullDataFrame_Exit;
	}

	skb =
	    __dev_alloc_skb(sizeof(_80211HeaderType) +
			    piperp->hw->extra_tx_headroom, GFP_ATOMIC);
	if (skb == NULL)
		goto piper_sendNullDataFrame_Exit;

	tx_info = (struct ieee80211_tx_info *) skb->cb;

	skb_reserve(skb, piperp->hw->extra_tx_headroom);
	header = (_80211HeaderType *) skb_put(skb, sizeof(_80211HeaderType));
	memset(header, 0, sizeof(*header));
	header->fc.type = TYPE_NULL_DATA;
	header->fc.pwrMgt = is_power_management_on;
	header->duration = getAckDuration(piperp);
	memcpy(header->addr1, piperp->bssid, sizeof(header->addr1));
	memcpy(header->addr2, piperp->pdata->macaddr, sizeof(header->addr2));
	memcpy(header->addr3, piperp->bssid, sizeof(header->addr3));

	tx_info->flags = IEEE80211_TX_CTL_ASSIGN_SEQ | IEEE80211_TX_CTL_FIRST_FRAGMENT;
	tx_info->band = piperp->rf->getBand(piperp->channel);
	tx_info->antenna_sel_tx = 1;	/* actually this is ignored for now */
	tx_info->control.rates[0].idx = 0;
	tx_info->control.rates[0].count = 2;	/* no retries.  Don't tie us up waiting for an ACK */
	tx_info->control.rates[0].flags = 0;
	tx_info->control.rates[1].idx = -1;
	tx_info->control.rts_cts_rate_idx = -1;
	piperp->ps.frames_pending++;

	if (piper_hw_tx_private(piperp->hw, skb, ps_free_frame) != 0) {
		/* printk(KERN_ERR
		       "piper_hw_tx() failed unexpectedly when sending null data frame.\n"); */
		ps_free_frame(piperp->hw, skb);
	}

piper_sendNullDataFrame_Exit:
	return;
}




#define RESET_PIPER		(1)				/* must be set to 1 (0 case is only for debugging)*/
#define PS_DONT_FORCE		(false)		/* set force to this value if we want to be safe */
#define PS_FORCE_POWER_DOWN	(true)		/* set force to this value to shut down the H/W reguardless*/
/*
 * This routine shuts down Piper and the Airoha transceiver.  First we check to
 * make sure the driver and H/W are idle.  Then we save the state of the H/W.
 * Then we shut down the Airoha and place piper into reset.
 */
int piper_MacEnterSleepMode(struct piper_priv *piperp, bool force)
{
	/*
	 * Interrupts are already disabled when we get here.
	 */

	if (piperp->ps.poweredDown)
		return 0;

	savedRegs[INDX_INTR_MASK] = piperp->ac->rd_reg(piperp, BB_IRQ_MASK);
	piperp->ac->wr_reg(piperp, BB_IRQ_MASK, 0, op_write);

    if (!force) {
    	if (   (piperp->ps.rxTaskletRunning)
    	    || ((piperp->ac->rd_reg(piperp, BB_RSSI) & BB_RSSI_EAS_BUSY))
    		|| (  (piperp->ac->rd_reg(piperp, BB_GENERAL_CTL)
    	     	& BB_GENERAL_CTL_TX_FIFO_EMPTY) == 0)
			|| (piperp->tx_tasklet_running)
    		|| (  (piperp->ac->rd_reg(piperp, BB_GENERAL_STAT)
    			& BB_GENERAL_STAT_RX_FIFO_EMPTY) == 0)
    		|| (piperp->ac->rd_reg(piperp, BB_IRQ_STAT) & savedRegs[INDX_INTR_MASK])) {

			piperp->ac->wr_reg(piperp, BB_IRQ_MASK, savedRegs[INDX_INTR_MASK], op_write);
    		return -1;
		}
    }

	disable_irq(piperp->irq);

	savedRegs[INDX_GEN_CONTROL] = piperp->ac->rd_reg(piperp, BB_GENERAL_CTL);
	savedRegs[INDX_GEN_STATUS] = piperp->ac->rd_reg(piperp, BB_GENERAL_STAT);
	savedRegs[INDX_RSSI_AES] = piperp->ac->rd_reg(piperp, BB_RSSI) & ~BB_RSSI_EAS_BUSY;
	savedRegs[INDX_SPI_CTRL] = piperp->ac->rd_reg(piperp, BB_SPI_CTRL);
	savedRegs[INDX_CONF1] = piperp->ac->rd_reg(piperp, BB_TRACK_CONTROL);
	savedRegs[INDX_CONF2] = piperp->ac->rd_reg(piperp, BB_CONF_2);
	savedRegs[INDX_OUT_CTRL] = piperp->ac->rd_reg(piperp, BB_OUTPUT_CONTROL);
	savedRegs[INDX_MAC_CONTROL] = piperp->ac->rd_reg(piperp, MAC_CTL);

	savedRegs[INDX_STAID_1]		= piperp->ac->rd_reg(piperp, MAC_STA_ID0);
	savedRegs[INDX_STAID_2]		= piperp->ac->rd_reg(piperp, MAC_STA_ID1);
	savedRegs[INDX_BSSID_1]		= piperp->ac->rd_reg(piperp, MAC_BSS_ID0);
	savedRegs[INDX_BSSID_2]		= piperp->ac->rd_reg(piperp, MAC_BSS_ID1);
	savedRegs[INDX_BRS_SSID]	= piperp->ac->rd_reg(piperp, MAC_SSID_LEN);
	savedRegs[INDX_BACKOFF]		= piperp->ac->rd_reg(piperp, MAC_BACKOFF);
	savedRegs[INDX_DTIM_LISTEN]	= piperp->ac->rd_reg(piperp, MAC_DTIM_PERIOD);
	savedRegs[INDX_BEACON_INT]	= piperp->ac->rd_reg(piperp, MAC_CFP_ATIM);
	savedRegs[INDX_MAC_CTL]		= piperp->ac->rd_reg(piperp, MAC_CTL);
	savedRegs[INDX_BEACON_MASK]	= piperp->ac->rd_reg(piperp, MAC_BEACON_FILT);

	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~BB_GENERAL_CTL_RX_EN, op_and);	//disable receiving
	piperp->ac->wr_reg(piperp, MAC_CTL, 0, op_write);
	piperp->ac->wr_reg(piperp, BB_IRQ_MASK, 0, op_write);

#if RESET_PIPER
    // Power down the airoha transceiver
	piperp->rf->power_on(piperp->hw, false);
	udelay(10);
	// hold the transceiver in reset mode
	if (piperp->pdata->reset)
		piperp->pdata->reset(piperp, 1);
#endif
	stats.jiffiesOn += jiffies - stats.cycleStart;
	stats.cycleStart = jiffies;
	piperp->ps.poweredDown = true;

	return 0;
}


#define PS_NO_SPIKE_SUPPRESSION		(false)		/* want_spike_suppression = don't want spike suppression*/
#define PS_WANT_SPIKE_SUPPRESSION	(true)		/* want_spike_suppression = do spike suppression*/
/*
 * Turn the H/W back on after it was shutdown with piper_MacEnterSleepMode.
 *
 *	1. Power up the hardware.
 *  2. Perform the spike suppression routine if desired.  The spike suppression
 *     routine resyncs the clocks in Piper in order to prevent us from generating
 *     noise spikes at 1 and 2 MHz.  Unfortunately it takes an indeterminate amount
 *     of time, so we normally don't do it and just make sure we do not send at
 *	   at those data rates while duty cycling.
 * 3.  Set the channel.  The Airoha was shut off so we have to set the channel
 *     again.
 * 4.  Restore the state of piper registers.
 * 5.  Zero out and reset the transmitter FIFO.  Mike Schaffner claims this should
 *     not be necessary, but we seem to run into trouble when we don't do it.
 * 6.  Restore the interrupts.
 */
void piper_MacEnterActiveMode(struct piper_priv *piperp, bool want_spike_suppression)
{
	int i;
// #define WANT_DEBUG
#ifdef WANT_DEBUG
	static unsigned int run = 0;
#endif

#if RESET_PIPER

	if (piperp->pdata->reset) {
#ifdef WANT_DEBUG
		if (piperp->ac->rd_reg(piperp, BB_GENERAL_CTL) & BB_GENERAL_CTL_TX_FIFO_FULL) {
			printk(KERN_ERR "**** While in reset, run = %d\n", run);
			digiWifiDumpRegisters(piperp, MAIN_REGS);
			while(1);
		}
		if (piperp->ac->rd_reg(piperp, BB_RSSI) & BB_RSSI_EAS_BUSY) {
			printk(KERN_ERR "**** While in reset AES busy set, run = %d\n", run);
			digiWifiDumpRegisters(piperp, MAIN_REGS);
			while(1);
		}
#endif
	    piperp->pdata->reset(piperp, 0);
	    udelay(10);

#ifdef WANT_DEBUG
		if (piperp->ac->rd_reg(piperp, BB_GENERAL_CTL) & BB_GENERAL_CTL_TX_FIFO_FULL) {
			printk(KERN_ERR "**** After reset, run = %d\n", run);
			digiWifiDumpRegisters(piperp, MAIN_REGS);
			while(1);
		}
		if (piperp->ac->rd_reg(piperp, BB_RSSI) & BB_RSSI_EAS_BUSY) {
			printk(KERN_ERR "**** After reset AES busy set, run = %d\n", run);
			digiWifiDumpRegisters(piperp, MAIN_REGS);
			while(1);
		}
		run++;
#endif
	    piperp->rf->power_on(piperp->hw, true);
	    mdelay(1);
	    piper_spike_suppression(piperp, want_spike_suppression);
	}
#endif

	piperp->ac->wr_reg(piperp, BB_IRQ_STAT, 0xff, op_write);


#if RESET_PIPER
	piperp->rf->set_chan_no_rx(piperp->hw, piperp->channel);
#endif

      // store the registers back

	piperp->ac->wr_reg(piperp, BB_GENERAL_STAT, 0x30000000, op_write);
	piperp->ac->wr_reg(piperp, BB_RSSI, savedRegs[INDX_RSSI_AES] & ~BB_RSSI_EAS_BUSY, op_write);

//	piperp->ac->wr_reg(piperp, BB_IRQ_MASK, savedRegs[INDX_INTR_MASK], op_write);
	piperp->ac->wr_reg(piperp, BB_SPI_CTRL, savedRegs[INDX_SPI_CTRL], op_write);
	piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, savedRegs[INDX_CONF1], op_write);
	piperp->ac->wr_reg(piperp, BB_CONF_2, savedRegs[INDX_CONF2], op_write);
	piperp->ac->wr_reg(piperp, BB_AES_CTL, 0, op_write);
	piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, savedRegs[INDX_OUT_CTRL], op_write);
	piperp->ac->wr_reg(piperp, MAC_CTL, savedRegs[INDX_MAC_CONTROL], op_write);

	piperp->ac->wr_reg(piperp, MAC_STA_ID0,     savedRegs[INDX_STAID_1],          op_write);
	piperp->ac->wr_reg(piperp, MAC_STA_ID1,     savedRegs[INDX_STAID_2],          op_write);
	piperp->ac->wr_reg(piperp, MAC_BSS_ID0,     savedRegs[INDX_BSSID_1],          op_write);
	piperp->ac->wr_reg(piperp, MAC_BSS_ID1,     savedRegs[INDX_BSSID_2],          op_write);
	piperp->ac->wr_reg(piperp, MAC_SSID_LEN,    savedRegs[INDX_BRS_SSID],       op_write);
	piperp->ac->wr_reg(piperp, MAC_BACKOFF,     savedRegs[INDX_BACKOFF],          op_write);
	piperp->ac->wr_reg(piperp, MAC_DTIM_PERIOD, savedRegs[INDX_DTIM_LISTEN],  op_write);
	piperp->ac->wr_reg(piperp, MAC_CFP_ATIM,    savedRegs[INDX_BEACON_INT], op_write);
	piperp->ac->wr_reg(piperp, MAC_CTL,         savedRegs[INDX_MAC_CTL],          op_write);
	piperp->ac->wr_reg(piperp, MAC_BEACON_FILT, savedRegs[INDX_BEACON_MASK],  op_write);

//****
	// set bit-11 in the general control register to a 1 to start the processors
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
			   BB_GENERAL_CTL_MAC_ASSIST_ENABLE, op_or);

	// set the TX-hold bit
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x37720080, op_write);

	// clear the TX-FIFO memory
	for (i = 0; i < 448; i++)
		piperp->ac->wr_reg(piperp, BB_DATA_FIFO, 0, op_write);

	// clear RX-FIFO memory
	for (i = 0; i < 512; i++)
		piperp->ac->rd_reg(piperp, BB_DATA_FIFO);

	// reset the TX-FIFO and RX-FIFO
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x377200E0, op_write);


	// release the TX-hold and reset
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x37720000, op_write);


//***
	/*
	 * Reset the interrupt mask.  We could have been receiving a frame when we
	 * powered down.  This could cause us to store the wrong mask, so we want
	 * to make sure we enabe the RX interrupts.  However, we should not have the
	 * TX interrupts enabled when we come out of power save mode.
	 */
	udelay(50);
	piperp->ac->wr_reg(piperp, BB_IRQ_STAT, 0xff, op_write);
	piperp->clear_irq_mask_bit(piperp, 0xffffffff);
	piperp->set_irq_mask_bit(piperp, BB_IRQ_MASK_RX_OVERRUN | BB_IRQ_MASK_RX_FIFO);
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
	             (savedRegs[INDX_GEN_CONTROL] | 0x37000000 |
	              BB_GENERAL_CTL_RX_EN), op_write);

	stats.jiffiesOff += jiffies - stats.cycleStart;
	stats.cycleStart = jiffies;

	/* TODO, this is a temporary workaround and should be better analyzed in future.
	 * The problem is that the general power save code is not synchronized with the
	 * dynamic PW and this is causing that, the following line, unbalances the
	 * piper wireless irq */
	if (piperp->ps.poweredDown != false)
		enable_irq(piperp->irq);

	piperp->ps.poweredDown = false;
}




/*
 * So what crazy thing are we doing here?  Well, Piper has a bug where it
 * can send noise spikes at 1 Mbps and 2 Mbps if it is powered up without
 * running a special spike suppression routine.  The spike suppression code
 * takes an average of 30 ms, and I have timed it taking as long as 300 ms.
 * This is not something you want to use for duty cycling.  The solution is
 * to avoid sending at those two rates.  After the transmit routine determines
 * the rate mac80211 specified, it will call us and we will decide whether
 * we like that rate.  If it is one of our bad rates, then we will bump it
 * up to a good rate.
 */
struct ieee80211_rate *piper_ps_check_rate(struct piper_priv *piperp,
					   struct ieee80211_rate *rate)
{
#define BAD_RATE_1MBPS		(10)
#define BAD_RATE_2MBPS		(20)
	if ((piperp->ps.mode == PS_MODE_LOW_POWER) && (rate != NULL)) {
		if ((rate->bitrate == BAD_RATE_1MBPS)
		    || (rate->bitrate == BAD_RATE_2MBPS)) {
			rate =
			    (struct ieee80211_rate *) piperp->rf->
			    getRate(AIROHA_55_MBPS_RATE_INDEX);
		}
	}

	return rate;
}

EXPORT_SYMBOL_GPL(piper_ps_check_rate);



/*
 * This routine restarts the transmitter after powering back
 * up, or failing to power down.
 *
 * 	1) Clear the power management bit so that frames will be
 *     sent indicating that we are poweredup.
 *  2) Set the flag to allow transmits again.
 *  3) If we stopped the mac80211 transmitter queue, then start
 *     it back up again.
 *  4) Notify the AP that we are awake by sending a null-data frame
 *     with the power management bit clear.
 */
static void ps_resume_transmits(struct piper_priv *piperp)
{
	piperp->ps.power_management = POWERED_UP;
 	piperp->ps.allowTransmits = true;
	piperp->ps.stopped_tx_queues = false;
 	piper_sendNullDataFrame(piperp, POWERED_UP);
	ieee80211_wake_queues(piperp->hw);
}






/*
 * This routine sets an event timer.  The ps_state_machine routine
 * will be executed when the event timer expires.
 */
static void ps_set_timer_event(struct piper_priv *piperp,
								enum piper_ps_event next_event,
								unsigned int timeout_ms)
{
	unsigned int delay_in_jiffies = MILLS_TO_JIFFIES(timeout_ms);

	if (delay_in_jiffies == 0)
		delay_in_jiffies++;

	del_timer_sync(&piperp->ps.timer);
	piperp->ps.this_event = next_event;
	piperp->ps.timer.expires = delay_in_jiffies + jiffies;
	add_timer(&piperp->ps.timer);
}


/*
 * This routine cancels an event timer set by ps_set_timer_event.
 */
static void ps_cancel_timer_event(struct piper_priv *piperp)
{
	del_timer_sync(&piperp->ps.timer);
}


#define WANT_STATE_MACHINE_DEBUG		(0)
#if WANT_STATE_MACHINE_DEBUG

struct event_record {
	enum piper_ps_event event;
	enum piper_ps_state state;
};

#define MAX_EVENTS_RECORDED		(15)
static unsigned int debug_events_index = 0;
static struct event_record debug_events[MAX_EVENTS_RECORDED];

void debug_track_event(enum piper_ps_event event, enum piper_ps_state state)
{
	debug_events[debug_events_index].event = event;
	debug_events[debug_events_index].state = state;
	if (debug_events_index == MAX_EVENTS_RECORDED) {
		unsigned int i;

		printk(KERN_ERR);
		for (i = 0; i < MAX_EVENTS_RECORDED; i++) {
			printk("(%d,%d)", debug_events[i].event,
					debug_events[i].state);
		}
		printk("\n");
		debug_events_index = 0;
	} else {
		debug_events_index++;
	}
}


#else
	#define debug_track_event(event, state)
#endif

/*
 * This is the entry point into the duty cycle state machine.  It may be called
 * by:
 *
 *		1) piper_ps_handle_beacon when a beacon frame is received.
 *		2) the receiver task when we receive the ACK for the last pending frame
 *		   when we are waiting to power down.
 *      3) by ps_timer for many timer events.
 */
static void ps_state_machine(struct piper_priv *piperp, enum piper_ps_event event)
{
	unsigned long flags;

	debug_track_event(event, piperp->ps.state);

	spin_lock_irqsave(&piperp->ps.lock, flags);


	switch (event) {
		case PS_EVENT_BEACON_RECEIVED:
			/*
			 * We just received a beacon.  This is really the driving event in this state
			 * machine.  Everything else is synchronized from it.
			 *
			 * We know we are powered up because we just received the beacon.  The normal
			 * control path is to set a timer which will expire when we need to start
			 * preparations for powering down again.
			 */
			ps_cancel_timer_event(piperp);				/* cancel missed beacon timer*/
			stats.receivedBeacons++;
			if (   (!piperp->areWeAssociated)
			    || (piperp->ps.beacon_int <  PS_MINIMUM_BEACON_INT)
			    || (piperp->ps.scan_timer != 0)) {
				/*
				 * Don't duty cyle while we are trying to associate.
				 */
				piperp->ps.state = PS_STATE_WAIT_FOR_BEACON;
				if (piperp->ps.scan_timer) {
					piperp->ps.scan_timer--;
				}
				break;
			}
			if (piperp->ps.state == PS_STATE_WAIT_FOR_BEACON) {
				int timeout;
				/*
				 * Calculate amount of time to sleep.
				 */
				piperp->ps.sleep_time = (piperp->ps.beacon_int * (100 - piperp->power_duty)) / 100;

				/*
				 * Now figure out how long we have before it's time to go to sleep.  We
				 * have to wake up at least PS_WAKE_BEFORE_BEACON_MS before we expect to
				 * receive the next beacon, and we need to start powering down at least
				 * PS_TRANSMITTER_SHUTDOWN_MS ahead of time.
				 */
				timeout = piperp->ps.beacon_int - (piperp->ps.sleep_time + PS_TRANSMITTER_SHUTDOWN_MS + PS_WAKE_BEFORE_BEACON_MS);
				if ((timeout > 0) && (piperp->ps.sleep_time > PS_MINIMUM_SLEEP_TIME)) {
					/*
					 * If there is enough time left that it makes sense to duty
					 * cycle, then program the timer and advance to the next state.
					 */
					piperp->ps.state = PS_STATE_WAIT_FOR_STOP_TRANSMIT_EVENT;
					ps_set_timer_event(piperp, PS_EVENT_STOP_TRANSMIT_TIMER_EXPIRED, timeout);

					break;
				}
			} else {
				 if (   (piperp->ps.state == PS_STATE_WAIT_FOR_TRANSMITTER_DONE)
				 	 || (piperp->ps.state == PS_STATE_WAIT_FOR_TRANSMITTER_DONE_EVENT)) {
					ps_resume_transmits(piperp);
				}
				digi_dbg("*** Beacon event in state %d.\n", piperp->ps.state);
			}
			/*
			 * We will come here if we were in the wrong state when we received the
			 * beacon, or if the duty cycle is too short.
			 */
			piperp->ps.state = PS_STATE_WAIT_FOR_BEACON;
			ps_set_timer_event(piperp, PS_EVENT_MISSED_BEACON,
						  piperp->ps.beacon_int + PS_BEACON_TIMEOUT_MS);
			break;
		case PS_EVENT_STOP_TRANSMIT_TIMER_EXPIRED:
			/*
			 * This event is hit when it's time to start powering down.  Unfortunately, this is
			 * not a simple thing to do.  The first things to do are:
			 *
			 *  1. Set the power save on flag.  This will cause any transmit frame to be
			 * 	   sent with the power management bit set.
			 *  2. Stop the mac80211 layer from sending us anymore frames.
			 *  3. Signal the AP that we are powering down by sending a null-data frame with the
			 * 	   power management bit set.
			 */
			 if (piperp->ps.state == PS_STATE_WAIT_FOR_STOP_TRANSMIT_EVENT) {
			 	if (piperp->ps.scan_timer) {
			 		/*
			 		 * Don't shut down if we are scanning.
			 		 */
					piperp->ps.state = PS_STATE_WAIT_FOR_BEACON;
					break;
				}
			 	piperp->ps.power_management = POWERING_DOWN;
			 	piperp->ps.allowTransmits = false;
			 	ieee80211_stop_queues(piperp->hw);
			 	piperp->ps.stopped_tx_queues = true;
			 	piper_sendNullDataFrame(piperp, POWERING_DOWN);
			 	piperp->ps.state = PS_STATE_WAIT_FOR_TRANSMITTER_DONE;
				ps_set_timer_event(piperp, PS_EVENT_TRANSMITTER_DONE_TIMER_TICK,
							PS_TRANSMIT_TIMER_TICK_MS);
			} else {
				/*
				 * This should never happen (famous last words)
				 */
				digi_dbg("** stop tx event, state = %d.\n", piperp->ps.state);
				piperp->ps.state = PS_STATE_WAIT_FOR_BEACON;
				ps_set_timer_event(piperp, PS_EVENT_MISSED_BEACON,
							piperp->ps.beacon_int + PS_BEACON_TIMEOUT_MS);
			}
			break;
		case PS_EVENT_TRANSMITTER_DONE:
			/*
			 * This event is triggered when the receive task finishes processing the ACK
			 * from the null-data frame sent by the PS_EVENT_STOP_TRANSMIT_TIMER_EXPIRED event.
			 * We try to power down now.
			 */
			 if (piperp->ps.scan_timer == 0) {
				 if (   (piperp->ps.state == PS_STATE_WAIT_FOR_TRANSMITTER_DONE)
				 	 || (piperp->ps.state == PS_STATE_WAIT_FOR_TRANSMITTER_DONE_EVENT)) {
					ps_cancel_timer_event(piperp);				/* cancel transmitter done timeout timer*/
				 	if (piper_MacEnterSleepMode(piperp, PS_DONT_FORCE) == 0) {
				 		piperp->ps.state = PS_STATE_WAIT_FOR_WAKEUP_ALARM;
				 		/*
				 		 * Note that the value PS_EVENT_TRANSMITTER_DONE_TIMER_TICK is
				 		 * updated as necessary by the PS_EVENT_TRANSMITTER_DONE_TIMER_TICK
				 		 * event to take into account the amount of time it took for the
				 		 * transmitter to finish sending the last frame.
				 		 */
				 		ps_set_timer_event(piperp, PS_EVENT_WAKEUP, piperp->ps.sleep_time);
				 		break;
					}
				 } else {
#ifdef WANT_DEBUG
		printk(KERN_ERR "couldn't sleep, rxt=%d, AES busy = %d, txfifo=%d, txt=%d, rxfifo=%d\n",
				(piperp->ps.rxTaskletRunning),((piperp->ac->rd_reg(piperp, BB_RSSI) & BB_RSSI_EAS_BUSY) != 0),
				(  (piperp->ac->rd_reg(piperp, BB_GENERAL_CTL)& BB_GENERAL_CTL_TX_FIFO_EMPTY) == 0),
				(piperp->tx_tasklet_running),
    	     	(  (piperp->ac->rd_reg(piperp, BB_GENERAL_STAT)
    			& BB_GENERAL_STAT_RX_FIFO_EMPTY) == 0));
#endif
				 	digi_dbg("** PS_EVENT_TRANSMITTER_DONE event, but state == %d.\n", piperp->ps.state);
				}
			}
			 /*
			  * If we fall through to here, then either we are in the wrong state, or we were
			  * not able to power down the H/W.
			  */
			ps_resume_transmits(piperp);
			piperp->ps.state = PS_STATE_WAIT_FOR_BEACON;
			ps_set_timer_event(piperp, PS_EVENT_MISSED_BEACON,
						 piperp->ps.beacon_int + PS_BEACON_TIMEOUT_MS);
			break;
		case PS_EVENT_TRANSMITTER_DONE_TIMER_TICK:
			/*
			 * This event is triggered periodically while we are waiting for the
			 * transmitter to finish sending that last packet.  We decrement
			 * piperp->ps.sleep_time (which is used by the PS_EVENT_TRANSMITTER_DONE
			 * event).  If piperp->ps.sleep_time is still larger than our minimum
			 * required sleep time, then we just restart the timer.
			 */
			if (piperp->ps.state == PS_STATE_WAIT_FOR_TRANSMITTER_DONE) {
				piperp->ps.sleep_time -= PS_TRANSMIT_TIMER_TICK_MS;
				if (piperp->ps.sleep_time >= PS_MINIMUM_SLEEP_TIME) {
				 	piperp->ps.state = PS_STATE_WAIT_FOR_TRANSMITTER_DONE;
					ps_set_timer_event(piperp, PS_EVENT_TRANSMITTER_DONE_TIMER_TICK,
								PS_TRANSMIT_TIMER_TICK_MS);
				} else {
					/*
					 * Transmitter did not shut down in time.  Resume normal operations
					 * and stay awake until the next beacon.
					 */
					ps_resume_transmits(piperp);
				 	piperp->ps.state = PS_STATE_WAIT_FOR_BEACON;
					ps_set_timer_event(piperp, PS_EVENT_MISSED_BEACON,
								 piperp->ps.sleep_time + PS_WAKE_BEFORE_BEACON_MS
								 + PS_BEACON_TIMEOUT_MS);
				}
			} else if (piperp->ps.state == PS_STATE_WAIT_FOR_TRANSMITTER_DONE_EVENT) {
				piperp->ps.sleep_time -= PS_TRANSMIT_TIMER_TICK_MS;
				/*
				 * The piper_ps_rx_task_exiting routine sets this state just before it
				 * releases the lock on ps.state and calls this event.  If we ever
				 * come here, then the timer tick occurred just between the time
				 * piper_ps_rx_task_exiting released the lock and ps_state_machine
				 * reset it.  Since the tx done event is in progress, we should ignore
				 * this tick.
				 */
				break;
			} else {
				digi_dbg("** done tick in state %d.\n", piperp->ps.state);
			}
			break;
		case PS_EVENT_WAKEUP:
			/*
			 * This event is called when we have powered down and it is time
			 * to power back up again.
			 *
			 *	1) Power up the H/W.
			 *	2) Resume normal operations
			 *  3) Update our state.
			 *  4) Set a timeout for receiving the next beacom frame.
			 */
			if (piperp->ps.state == PS_STATE_WAIT_FOR_WAKEUP_ALARM) {
				piper_MacEnterActiveMode(piperp, PS_NO_SPIKE_SUPPRESSION);
				ps_resume_transmits(piperp);
			 	piperp->ps.state = PS_STATE_WAIT_FOR_BEACON;
				ps_set_timer_event(piperp, PS_EVENT_MISSED_BEACON,
							 PS_BEACON_TIMEOUT_MS + PS_WAKE_BEFORE_BEACON_MS);
			} else {
				digi_dbg("** wake event in state %d.\n", piperp->ps.state);
			}
			break;
		case PS_EVENT_MISSED_BEACON:
			/*
			 * This event is called when we miss a beacon.  For now just update
			 * our statistics.
			 */
		 	piperp->ps.state = PS_STATE_WAIT_FOR_BEACON;
		 	if ((piperp->areWeAssociated)  && (piperp->ps.scan_timer == 0)) {
				stats.missedBeacons++;
				ps_set_timer_event(piperp, PS_EVENT_MISSED_BEACON, piperp->ps.beacon_int);
			}
			break;
		default:
			digi_dbg("**** ps_state_machine received unknown event %d.\n", event);
			break;
	}
	spin_unlock_irqrestore(&piperp->ps.lock, flags);

}

/*
 * This routine is called by the receiver task when it exits.  We use it to generate
 * the PS_EVENT_TRANSMITTER_DONE event.  The event signifies that the transmitter has
 * sent our null-data frame and is idle.  We determine this by checking frames_pending,
 * while will be nonzero if a null-data frame is waiting to be sent, and the machine's
 * state.
 */
void piper_ps_rx_task_exiting(struct piper_priv *piperp)
{
	unsigned long flags;

	spin_lock_irqsave(&piperp->ps.lock, flags);

	if (   (piperp->ps.frames_pending == 0)
		&& (piperp->ps.state == PS_STATE_WAIT_FOR_TRANSMITTER_DONE)) {
		/*
		 * We have a race condition between the transmitter done tick and
		 * this routine.  So this routine changes the state to
		 * PS_STATE_WAIT_FOR_TRANSMITTER_DONE_EVENT before it releases the
		 * lock so that we don't get confused.
		 */
		piperp->ps.state = PS_STATE_WAIT_FOR_TRANSMITTER_DONE_EVENT;
		spin_unlock_irqrestore(&piperp->ps.lock, flags);
		ps_state_machine(piperp, PS_EVENT_TRANSMITTER_DONE);
	} else {
		spin_unlock_irqrestore(&piperp->ps.lock, flags);
	}
}

/*
 * Called when the event timer expires.  Call the state machine to handle
 * the event.
 */
static void ps_timer(unsigned long context)
{
	struct piper_priv *piperp = (struct piper_priv *) context;

	ps_state_machine(piperp, piperp->ps.this_event);
}



/*
 * This routine is called when we receive a beacon.  We extract the beacon interval
 * in case it has changed and then call the state machine.
 */
static void piper_ps_handle_beacon(struct piper_priv *piperp, struct sk_buff *skb)
{
#define BEACON_INT_LSB					(8)
#define BEACON_INT_MSB					(9)
	u32 beacon_int;
	_80211HeaderType *header = (_80211HeaderType *) skb->data;
	bool fromOurAp = piperp->areWeAssociated
					&& (memcmp(piperp->bssid, header->addr3, sizeof (header->addr3)) == 0);

	/*
	 * mac80211 does not inform us when the beacon interval changes, so we have
	 * to read this information from the beacon ourselves.
	 */

	if (fromOurAp) {
		beacon_int = skb->data[sizeof(_80211HeaderType) + BEACON_INT_LSB];
		beacon_int |= (skb->data[sizeof(_80211HeaderType) + BEACON_INT_MSB] << 8);
		piperp->ps.beacon_int = beacon_int;

		if (piperp->ps.mode == PS_MODE_LOW_POWER) {
			ps_state_machine(piperp, PS_EVENT_BEACON_RECEIVED);
		}
	}
}



/*
 * This routine is called when mac80211 starts doing things that might indicate it
 * is attempting to scan or reassociate.  Things like changing the channel or
 * disassociating.  When we receive an event like that, we stop duty cycling for
 * a while since it may interfere with attempts to reassociate with an access point.
 */
void piper_ps_scan_event(struct piper_priv *piperp)
{
	(void) piperp;
#if 0
	/*
	 * It appears that pausing duty cycling during association events may actually
	 * worsen performance I suspect that either the AP or mac80211 is measuring
	 * our throughput and adjusting the load accordingly, and that momentary changes
	 * in performance caused by pausing duty cyling interfere with this.
	 *
	 * TODO:  Consider removing this code.  I left it in for now in case we decide
	 *        to try it again, but if we're not going to use it, it just makes the
	 *        driver more confusing and should be removed.
	 */
	if (piperp->ps.beacon_int != 0) {
		piperp->ps.scan_timer = PS_SCAN_DELAY / piperp->ps.beacon_int;
	} else {
		piperp->ps.scan_timer = PS_SCAN_DELAY / 100;
	}
#endif
}



/*
 * This routine is called so we can process incoming frames.  We do the
 * handshaking to receive buffered frames in PS mode here.
 */
void piper_ps_process_receive_frame(struct piper_priv *piperp, struct sk_buff *skb)
{
	_80211HeaderType *header = (_80211HeaderType *) skb->data;

	if (header->fc.type == TYPE_BEACON) {
		piper_ps_handle_beacon(piperp, skb);
	} else if (   (header->fc.type == TYPE_ASSOC_RESP)
			   || (header->fc.type == TYPE_REASSOC_RESP)
			   || (header->fc.type == TYPE_PROBE_RESP)
			   || (header->fc.type == TYPE_DISASSOC)
			   || (header->fc.type == TYPE_DEAUTH)
			   || (header->fc.type == TYPE_ACTION)) {
		piper_ps_scan_event(piperp);
	}
}

EXPORT_SYMBOL_GPL(piper_ps_process_receive_frame);



/*
 * This function turns power save mode on or off.
 */
void piper_ps_set(struct piper_priv *piperp, bool powerSaveOn)
{
#define MAX_SHUTDOWN_TIMEOUT		(100)
	unsigned long flags;

	spin_lock_irqsave(&piperp->ps.lock, flags);

	piper_ps_scan_event(piperp);
	if (powerSaveOn) {
		if (piperp->ps.beacon_int >= PS_MINIMUM_BEACON_INT) {
			if (piperp->ps.mode != PS_MODE_LOW_POWER) {
			    piperp->ps.aid = 0;
			    piperp->ps.mode = PS_MODE_LOW_POWER;
			    piperp->ps.state= PS_STATE_WAIT_FOR_BEACON;
			    piperp->ps.power_management = POWERED_UP;
			    piperp->ps.poweredDown = false;
			    piperp->ps.allowTransmits = true;
			    piperp->ps.stopped_tx_queues = false;
			    stats.receivedBeacons = 0;
			    stats.missedBeacons = 0;
				stats.modeStart = jiffies;
				stats.cycleStart = jiffies;
				stats.jiffiesOff = 0;
				stats.jiffiesOn = 0;
				piper_sendNullDataFrame(piperp, POWERED_UP);
				/*
				 * Will start it the next time we receive a beacon.
				 */
			}
		} else {
			printk(KERN_ERR
			       "\nUnable to set power save mode because the beacon \n"
			       "interval set on this access point less than 100ms.\n");
		}
	} else {
		ps_cancel_timer_event(piperp);
		if (piperp->ps.mode == PS_MODE_LOW_POWER) {
			piperp->ps.mode = PS_MODE_FULL_POWER;		/* stop duty cycle timer */
			if (piperp->ps.poweredDown) {
				/*
				 * If we were powered down, then power up and do the spike suppression.
				 */
				piper_MacEnterActiveMode(piperp, PS_WANT_SPIKE_SUPPRESSION);
			} else {
				unsigned int timeout = 50;
				int result;
				/*
				 * If we branch here, then we were already powered up.  You would think
				 * that we would be all set, but it's not that easy.  Piper has a bug in
				 * it where we have to run a special spike suppression routine when we
				 * power it up.  However, this routine takes an average of 30 ms to run,
				 * and I've see it take as long as 300 ms.  This is not acceptable when
				 * we are duty cycling every 100 ms.  To get around this, we do NOT do
				 * the spike suppression while duty cycling.  Instead, we simply avoid
				 * transmitting at those rates which would cause spikes.  Now, however,
				 * we are ending duty cycling and returning to normal operations so we
				 * have to do the spike suppression.  Since we are powered up, the first
				 * thing to do is to power down.
				 */
				if (piperp->ps.state != PS_STATE_WAIT_FOR_STOP_TRANSMIT_EVENT) {
					/*
					 * If we come here, then we did not happen to be trying to power down
					 * just as we got the command from mac80211, so we have to start
					 * the procedure.  This is the normal case.
					 *
					 * 1. Set the power save on flag.  This will cause frames to be
					 *    transmitted with the power management bit on.  The reason
					 *    for doing that is to tell the AP to stop sending us frames.
					 * 2. Stop the mac80211 layer from sending us more frames by stopping
					 *    the transmit queues.
					 * 3. Send a null-data frame to the AP with the power management bit
					 *    set.  This should cause it to stop sending us frames.
					 */
				 	piperp->ps.power_management = POWERING_DOWN;
				 	piperp->ps.allowTransmits = false;
				 	ieee80211_stop_queues(piperp->hw);
				 	piperp->ps.stopped_tx_queues = true;
				 	piper_sendNullDataFrame(piperp, POWERING_DOWN);
				}
				/*
				 * Now wait for that last frame to go and and then shut down.
				 */
				 result = -1;
				 for (timeout = 0; (timeout < MAX_SHUTDOWN_TIMEOUT) && (result != 0); timeout++) {
					spin_unlock_irqrestore(&piperp->ps.lock, flags);
					mdelay(10);
					spin_lock_irqsave(&piperp->ps.lock, flags);
					result = piper_MacEnterSleepMode(piperp, PS_DONT_FORCE);
				}
				if (result != 0) {
					/*
					 * This is bad.  For some reason we are not able to power down.  We
					 * will try to force it now, but this may end up putting the driver
					 * or H/W into a bad state.  However, we can't sit in the loop above
					 * forever either.
					 */
#ifdef WANT_DEBUG
					printk(KERN_ERR "Forcing Piper to power down\n");
					printk(KERN_ERR "BB_RSSI_EAS_BUSY = %d\n", piperp->ac->rd_reg(piperp, BB_RSSI) & BB_RSSI_EAS_BUSY);
					printk(KERN_ERR "BB_GENERAL_CTL_TX_FIFO_EMPTY = %d\n",
					    piperp->ac->rd_reg(piperp, BB_GENERAL_CTL) & BB_GENERAL_CTL_TX_FIFO_EMPTY);
					printk(KERN_ERR "BB_GENERAL_STAT_RX_FIFO_EMPTY = %d\n",
						piperp->ac->rd_reg(piperp, BB_GENERAL_STAT) & BB_GENERAL_STAT_RX_FIFO_EMPTY);
					digiWifiDumpRegisters(piperp, MAIN_REGS | MAC_REGS);
#endif
					piper_MacEnterSleepMode(piperp, PS_FORCE_POWER_DOWN);
				}
				/*
				 * Wait a moment and then power the H/W back up and execute the spike suppression
				 * routine.
				 */
				spin_unlock_irqrestore(&piperp->ps.lock, flags);
				mdelay(30);
				spin_lock_irqsave(&piperp->ps.lock, flags);
				piper_MacEnterActiveMode(piperp, PS_WANT_SPIKE_SUPPRESSION);
				ps_resume_transmits(piperp);
			}
			stats.jiffiesOn += jiffies - stats.cycleStart;
#define WANT_STATS		(0)
#if WANT_STATS
			if ((piperp->ps.beacon_int != 0)
			    && ((jiffies - stats.modeStart) != 0)) {
				printk(KERN_ERR
				       "jiffiesOff = %u, jiffiesOn = %u, total time = %lu\n",
				       stats.jiffiesOff, stats.jiffiesOn,
				       (jiffies - stats.modeStart));
				printk(KERN_ERR
				       "Powered down %ld percent of the time.\n",
				       (stats.jiffiesOff * 100) / (jiffies - stats.modeStart));
				printk(KERN_ERR
				       "Received %u of %lu beacons while in powersave mode.\n",
				       stats.receivedBeacons,
				       (jiffies -
					stats.modeStart) /
				       MILLS_TO_JIFFIES(piperp->ps.beacon_int));
				printk(KERN_ERR "received %d beacons, missed %d\n",
						stats.receivedBeacons, stats.missedBeacons);
				printk(KERN_ERR "allowTransmits = %d, stopped_tx_queues = %d, q_count = %d\n",
 						 	piperp->ps.allowTransmits, piperp->ps.stopped_tx_queues,
 						 	piperp->tx_queue_count);
				if ((stats.receivedBeacons + stats.missedBeacons) != 0)
					printk(KERN_ERR "%d%% beacons were missed\n",
						(100 * stats.missedBeacons) / (stats.receivedBeacons + stats.missedBeacons));
			}
#endif
		}
	    piperp->ps.aid = 0;
	    piperp->ps.state= PS_STATE_WAIT_FOR_BEACON;
	    piperp->ps.power_management = POWERED_UP;
	    piperp->ps.poweredDown = false;
	    piperp->ps.allowTransmits = true;
	    piperp->ps.stopped_tx_queues = false;
		ps_resume_transmits(piperp);
		piper_sendNullDataFrame(piperp, POWERED_UP);
	}

	spin_unlock_irqrestore(&piperp->ps.lock, flags);
}

EXPORT_SYMBOL_GPL(piper_ps_set);



/*
 * Called when driver is loaded.  Initialize our context.
 */
void piper_ps_init(struct piper_priv *piperp)
{
	memset(&piperp->ps, 0, sizeof(piperp->ps));
    piperp->ps.beacon_int = 100;
    piperp->ps.aid = 0;
	init_timer(&piperp->ps.timer);
	piperp->ps.timer.function = ps_timer;
	piperp->ps.timer.data = (unsigned long) piperp;
    piperp->ps.mode = PS_MODE_FULL_POWER;
    piperp->ps.state= PS_STATE_WAIT_FOR_BEACON;
	spin_lock_init(&piperp->ps.lock);
    piperp->ps.power_management = POWERED_UP;
    piperp->ps.poweredDown = false;
    piperp->ps.rxTaskletRunning;
    piperp->ps.allowTransmits = true;
    piperp->ps.stopped_tx_queues = false;
    piperp->ps.frames_pending = 0;
}

EXPORT_SYMBOL_GPL(piper_ps_init);


/*
 * Called when driver is unloaded.  Make sure the PS
 * timer is shut down.
 */
void piper_ps_deinit(struct piper_priv *piperp)
{
	piper_ps_set(piperp, true);
	piperp->ps.mode = PS_MODE_FULL_POWER;
	del_timer_sync(&piperp->ps.timer);
}

EXPORT_SYMBOL_GPL(piper_ps_deinit);

/*
 * digiMac80211.c
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/*
 * This file contains the routines that interface with the mac80211
 * library.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/etherdevice.h>
#include <net/mac80211.h>
#include <crypto/aes.h>
#include "pipermain.h"
#include "mac.h"
#include "phy.h"
#include "digiPs.h"

#define MAC_DEBUG	(1)

#if MAC_DEBUG
static int dlevel = DWARNING;
#define dprintk(level, fmt, arg...)	if (level >= dlevel)			\
					printk(KERN_ERR PIPER_DRIVER_NAME	\
					    ": %s - " fmt, __func__, ##arg)
#else
#define dprintk(level, fmt, arg...)	do {} while (0)
#endif

/*
 * This constant determines how many times per second the led_timer_fn
 * function will be called.  (HZ >> 3) means 8 times a second.
 */
#define LED_TIMER_RATE		(HZ >> 3)
#define LED_MAX_COUNT			(15)

/*
 * This function is called from a timer to blink an LED in order to
 * indicate what are current state is.
 */
static void led_timer_fn(unsigned long context)
{
	struct piper_priv *piperp = (struct piper_priv *) context;
	static unsigned int count = 0;

	if(!piperp->pdata->set_led)
		return;

	switch (piperp->led_state) {
		case led_shutdown:
			/* Turn LED off if we are shut down */
			piperp->pdata->set_led(piperp, STATUS_LED, 0);
			break;
		case led_adhoc:
			/* Blink LED slowly in ad-hoc mode */
			piperp->pdata->set_led(piperp, STATUS_LED, (count & 8) ? 0 : 1);
			break;
		case led_not_associated:
			/* Blink LED rapidly when not associated with an AP */
			piperp->pdata->set_led(piperp, STATUS_LED, (count & 1) ? 0 : 1);
			break;
		case led_associated:
			/* LED steadily on when associated */
			piperp->pdata->set_led(piperp, STATUS_LED, 1);
			break;
		default:
			/* Oops.  How did we get here? */
			break;
	}
	count++;
	if (count > LED_MAX_COUNT) {
		count = 0;
	}

	piperp->led_timer.expires += LED_TIMER_RATE;
	add_timer(&piperp->led_timer);
}

/*
 * This function sets the current LED state.
 */
static int piper_set_status_led(struct ieee80211_hw *hw, enum led_states state)
{
	struct piper_priv *piperp = (struct piper_priv *)hw->priv;

	piperp->led_state = state;

	if(!piperp->pdata->set_led)
		return -ENOSYS;

	if (state == led_shutdown)
		piperp->pdata->set_led(piperp, STATUS_LED, 0);

	return 0;
}

/*
 * This routine is called to enable IBSS support whenever we receive
 * configuration commands from mac80211 related to IBSS support.  The
 * routine examines the configuration settings to determine if IBSS
 * support should be enabled and, if so, turns on automatic beacon
 * generation.
 *
 * TODO: This code may need to be moved into piper.c since other
 *       H/W does not implement automatic generate of beacons.
 */
static void piper_enable_ibss(struct piper_priv *piperp, enum nl80211_iftype iftype)
{
	dprintk(DVVERBOSE, "\n");

	if (((iftype == NL80211_IFTYPE_ADHOC) || (iftype == NL80211_IFTYPE_MESH_POINT))
	    && (piperp->beacon.loaded) && (piperp->beacon.enabled)
	    && ((piperp->ac->rd_reg(piperp, MAC_CFP_ATIM) & MAC_BEACON_INTERVAL_MASK) != 0)) {
		/*
		 * If we come here, then we are running in IBSS mode, beacons are enabled,
		 * and we have the information we need, so start sending beacons.
		 */
		/* TODO: Handle non-zero ATIM period.  mac80211 currently has no way to
		 * tell us what the ATIM period is, but someday they might fix that.*/

		u32 reg = piperp->ac->rd_reg(piperp, MAC_BEACON_FILT) & ~MAC_BEACON_BACKOFF_MASK;
		piperp->ac->wr_reg(piperp, MAC_BEACON_FILT,
				  reg | piperp->get_next_beacon_backoff(), op_write);
		/* enable beacon interrupts*/
		piperp->set_irq_mask_bit(piperp, BB_IRQ_MASK_TBTT);
		piperp->ac->wr_reg(piperp,
				  MAC_CTL, MAC_CTL_BEACON_TX | MAC_CTL_IBSS, op_or);
		dprintk(DVERBOSE, "IBSS turned ON\n");
		piper_set_status_led(piperp->hw, led_adhoc);
	} else {
		/*
		 * If we come here, then either we are not suppose to transmit beacons,
		 * or we do not yet have all the information we need to transmit
		 * beacons.  Make sure the automatic beacon function is disabled.
		 */
                 /* shut down beacons */
		piperp->ac->wr_reg(piperp, MAC_CTL,
				  ~(MAC_CTL_BEACON_TX | MAC_CTL_IBSS), op_and);
		piperp->set_irq_mask_bit(piperp, BB_IRQ_MASK_TBTT);
		dprintk(DVERBOSE, "IBSS turned OFF\n");
	}
}

/*
 * Set the transmit power level.  The real work is done in the
 * transceiver code.
 */
static int piper_set_tx_power(struct ieee80211_hw *hw, int power)
{
	struct piper_priv *digi = hw->priv;
	int err;
	int oldTxPower = digi->tx_power;

	dprintk(DVVERBOSE, "\n");

	if (power == digi->tx_power)
		return 0;

	digi->tx_power = power;
	err = digi->rf->set_pwr(hw, power);
	if (err)
		digi->tx_power = oldTxPower;

	return err;
}

/*
 * Utility routine that sets a sequence number for a data packet.
 */
static void assign_seq_number(struct sk_buff *skb, bool increment)
{
#define SEQUENCE_NUMBER_MASK        (0xfff0)
	static u16 seq_number = 0;
	_80211HeaderType *header = (_80211HeaderType *)skb->data;

	dprintk(DVVERBOSE, "\n");

	if (skb->len >= sizeof(*header)) {
		u16 seq_field;

		/*
		 * TODO:  memcpy's are here because I am concerned we may get
		 *        an unaligned frame.  Is this a real possibility?  Or
		 *        am I just wasting CPU time?
		 */
		memcpy(&seq_field, &header->squ.sq16, sizeof(header->squ.sq16));
		seq_field &= ~SEQUENCE_NUMBER_MASK;
		seq_field |= (SEQUENCE_NUMBER_MASK & (seq_number << 4));
		memcpy(&header->squ.sq16, &seq_field, sizeof(header->squ.sq16));
		if (increment)
			seq_number++;
	}
}

#define GETU32(pt) (((u32)(pt)[0] << 24) ^ ((u32)(pt)[1] << 16) ^ ((u32)(pt)[2] <<  8) ^ ((u32)(pt)[3]))
/* Get 16 bits at byte pointer */
#define	GET16(bp)		((bp)[0] | ((bp)[1] << 8))
/* Get 32 bits at byte pointer */
#define	GET32(bp)		((bp)[0] | ((bp)[1] << 8) | ((bp)[2] << 16) | ((bp)[3] << 24))
/* Store 16 bits at byte pointer */
#define	SET16(bp, data)		{ (bp)[0] = (data); \
				(bp)[1] = (data) >> 8; }
/* Store 32 bits at byte pointer */
#define	SET32(bp, data)		{ (bp)[0] = (data); \
				(bp)[1] = (data) >> 8;  \
				(bp)[2] = (data) >> 16; \
				(bp)[3] = (data) >> 24; }

static inline void dw_inc_48(u48* n)
{
	(*n)++;
	*n &= ((u64) 1 << 48) - 1;
}

/*
 * This function prepares a blob of data we will have to send to the AES
 * H/W encryption engine.  The data consists of the AES initialization
 * vector and 2 16 byte headers.
 *
 * Returns true if successful, or false if something goes wrong
 */
bool piper_prepare_aes_datablob(struct piper_priv *piperp, unsigned int keyIndex,
				u8 *aesBlob, u8 *frame, u32 length, bool isTransmit)
{
	_80211HeaderType *header = (_80211HeaderType *)frame;
	u8 *body = &frame[sizeof(*header)];
	int dlen = length - (_80211_HEADER_LENGTH + PIPER_EXTIV_SIZE);

	dprintk(DVVERBOSE, "\n");

	if (keyIndex >= PIPER_MAX_KEYS) {
		dprintk(DWARNING, "encryption key index %d is out of range.\n",
			keyIndex);
		return false;
	}

	if (piperp->key[keyIndex].valid == false)
		return false;

	/* Set up CCM initial block for MIC IV */
	memset(aesBlob, 0, AES_BLOB_LENGTH);
	aesBlob[0] = 0x59;
	aesBlob[1] = 0;
	memcpy (&aesBlob[2], header->addr2, ETH_ALEN);
	aesBlob[8]  = body[7];
	aesBlob[9]  = body[6];
	aesBlob[10] = body[5];
	aesBlob[11] = body[4];
	aesBlob[12] = body[1];
	aesBlob[13] = body[0];
	aesBlob[14] = dlen >> 8;
	aesBlob[15] = dlen;

	/* Set up MIC header blocks */
#define AES_HEADER_0_OFFSET (16)
#define AES_HEADER_1_OFFSET (32)
	aesBlob[AES_HEADER_0_OFFSET+0] = 0;
	aesBlob[AES_HEADER_0_OFFSET+1] = 22;
	aesBlob[AES_HEADER_0_OFFSET+2] = frame[0] & 0xcf;
	aesBlob[AES_HEADER_0_OFFSET+3] = frame[1] & 0xd7;
	/*
	 * This memcpy writes data into the last 12 bytes of the first header
	 * and the first 6 bytes of the 2nd header.  I did it as one memcpy
	 * call for efficiency.
	 */
	memcpy(&aesBlob[AES_HEADER_0_OFFSET+4], header->addr1, 3*ETH_ALEN);
	aesBlob[AES_HEADER_1_OFFSET+6] = header->squ.sq.frag;
	aesBlob[AES_HEADER_1_OFFSET+7] = 0;
	memset (&aesBlob[AES_HEADER_1_OFFSET+8], 0, 8);	/* clear vector location in data */

	return true;
}

/*
 * mac80211 calls this routine to transmit a frame.  We set up
 * up the information the trasmit tasklet will need, and then
 * schedule the tasklet.
 */
int piper_hw_tx_private(struct ieee80211_hw *hw, struct sk_buff *skb, tx_skb_return_cb_t skb_return_cb)
{
	struct piper_priv *piperp = hw->priv;
	struct ieee80211_tx_info *txInfo = IEEE80211_SKB_CB(skb);
	unsigned long flags;

	dprintk(DVVERBOSE, "\n");
	/*
	 * Our H/W can only transmit a single packet at a time.  mac80211
	 * already maintains a queue of packets, so there is no reason
	 * for us to set up another one.  We stop the mac80211 queue everytime
	 * we get a transmit request and restart it when the transmit
	 * operation completes.
	 */
	ieee80211_stop_queues(hw);

	if (txInfo->flags & IEEE80211_TX_CTL_ASSIGN_SEQ) {
		assign_seq_number(skb,
				  !!(txInfo->flags & IEEE80211_TX_CTL_FIRST_FRAGMENT));
	}

	piperp->use_hw_aes = false;
	if (txInfo->control.hw_key != NULL) {
		/*
		 * We've been passed an encryption key, so mac80211 must want us
		 * to encrypt the packet with our fancy H/W encryptor.  Let's get
		 * set up for that now.
		 */
		piperp->tx_aes_key = txInfo->control.hw_key->hw_key_idx;
		piperp->use_hw_aes =
			  piper_prepare_aes_datablob(piperp,
						     txInfo->control.hw_key->hw_key_idx,
						     (u8 *)piperp->tx_aes_blob, skb->data,
						     skb->len, true);
	}
	piper_ps_set_header_flag(piperp, ((_80211HeaderType *) skb->data));		/* set power management bit as appropriate*/

	/*
	 * Add space at the start of the frame for the H/W level transmit header.
	 * We can't generate the header now. It must be generated everytime we
	 * transmit because the transmit rate changes when we do retries.
	 */
	skb_push(skb, TX_HEADER_LENGTH);

	txInfo->flags &= ~(IEEE80211_TX_STAT_TX_FILTERED |
			   IEEE80211_TX_STAT_ACK |
			   IEEE80211_TX_STAT_AMPDU |
			   IEEE80211_TX_STAT_AMPDU_NO_BACK);

	if (piper_tx_enqueue(piperp, skb, skb_return_cb) == -1) {
		skb_pull(skb, TX_HEADER_LENGTH);		/* undo the skb_push above */
		return -EBUSY;
	}

	spin_lock_irqsave(&piperp->tx_tasklet_lock, flags);
	if (!piperp->tx_tasklet_running) {
		piperp->tx_tasklet_running = true;
		tasklet_hi_schedule(&piperp->tx_tasklet);
	}

	spin_unlock_irqrestore(&piperp->tx_tasklet_lock, flags);

	piperp->pstats.tx_start_count++;

	return 0;
}
EXPORT_SYMBOL_GPL(piper_hw_tx_private);


int piper_hw_tx(struct ieee80211_hw *hw, struct sk_buff *skb)
{
	return piper_hw_tx_private(hw, skb, ieee80211_tx_status_irqsafe);
}


/*
 * mac80211 calls this routine to initialize the H/W.
 */
static int piper_hw_start(struct ieee80211_hw *hw)
{
	struct piper_priv *piperp = hw->priv;
	int ret = 0;

	dprintk(DVVERBOSE, "\n");
	piperp->if_type = __NL80211_IFTYPE_AFTER_LAST;

	/* Initialize the MAC H/W */
	if ((ret = piperp->init_hw(piperp, IEEE80211_BAND_2GHZ)) != 0) {
		dprintk(DERROR, "unable to initialize piper HW (%d)\n", ret);
		return ret;
	}

	/*
	 * Initialize the antenna with the default setting defined in the
	 * probe function. This can be changed, currently, through a sysfs
	 * entry in the device directory
	 */
	if ((ret = piperp->set_antenna(piperp, piperp->antenna)) != 0) {
		dprintk(DERROR, "piper_set_antenna_div() failed (%d)\n", ret);
		return ret;
	}

	/* set status led to link off */
	piper_set_status_led(hw, led_shutdown);

	/* Get the tasklets ready to roll */
	piperp->tx_result = TX_NOT_DONE;
	tasklet_enable(&piperp->rx_tasklet);
	tasklet_enable(&piperp->tx_tasklet);

	/*
	 * Enable receive interrupts, but leave the transmit interrupts
	 * and beacon interrupts off for now.
	 */
	piperp->clear_irq_mask_bit(piperp, 0xffffffff);
	piperp->set_irq_mask_bit(piperp,
				 BB_IRQ_MASK_RX_OVERRUN | BB_IRQ_MASK_RX_FIFO);
	enable_irq(piperp->irq);

	memset(piperp->bssid, 0, ETH_ALEN);

	return 0;
}

/*
 * mac80211 calls this routine to shut us down.
 */
static void piper_hw_stop(struct ieee80211_hw *hw)
{
	struct piper_priv *piperp = hw->priv;

	dprintk(DVVERBOSE, "\n");

	/* Initialize the MAC H/W */
	if (piperp->deinit_hw)
		piperp->deinit_hw(piperp);

	/* set status led to link off */
	piper_set_status_led(hw, led_shutdown);

	/* turn off phy */
	piperp->rf->stop(hw);

	/* Disable interrupts before turning off */
	disable_irq(piperp->irq);

	/* turn off MAX_GAIN, ADC clocks, and so on */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~BB_GENERAL_CTL_RESET, op_and);

	/* turn off MAC control/mac filt/aes key */
	piperp->ac->wr_reg(piperp, MAC_CTL, 0, op_write);

	/* turn off interrupts */
	tasklet_disable(&piperp->rx_tasklet);
	tasklet_disable(&piperp->tx_tasklet);
	piperp->clear_irq_mask_bit(piperp, 0xffffffff);
}

/*
 * mac80211 calls this routine to really start the H/W.
 * The device type is also set here.
 */
static int piper_hw_add_intf(struct ieee80211_hw *hw,
		struct ieee80211_vif *vif)
{
	struct piper_priv *piperp = hw->priv;

	dprintk(DVVERBOSE, "if type: %x\n", vif->type);

	/* __NL80211_IFTYPE_AFTER_LAST means no mode selected */
	if (piperp->if_type != __NL80211_IFTYPE_AFTER_LAST) {
		dprintk(DERROR, "unsupported interface type %x, expected %x\n",
			vif->type, __NL80211_IFTYPE_AFTER_LAST);
		return -EOPNOTSUPP;
	}

	switch (vif->type) {
	case NL80211_IFTYPE_ADHOC:
		piper_set_status_led(piperp->hw, led_adhoc);
		piperp->if_type = vif->type;
		break;

	case NL80211_IFTYPE_STATION:
		piper_set_status_led(hw, led_not_associated);
		piperp->if_type = vif->type;
		break;

	case NL80211_IFTYPE_MESH_POINT:
		piperp->if_type = vif->type;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

/*
 * mac80211 calls this function to shut down us down.
 */
static void piper_hw_rm_intf(struct ieee80211_hw *hw,
		struct ieee80211_vif *vif)
{
	struct piper_priv *digi = hw->priv;

	dprintk(DVVERBOSE, "\n");
	digi->if_type = __NL80211_IFTYPE_AFTER_LAST;
}

/*
 * mac80211 calls this function to pass us configuration information.
 */
static int piper_config(struct ieee80211_hw *hw, u32 changed)
{
	struct piper_priv *piperp = hw->priv;
	struct ieee80211_conf *conf = &hw->conf;
	u32 tempval;
	int err;

	dprintk(DVVERBOSE, "\n");

	if (changed & IEEE80211_CONF_CHANGE_PS) {
		/*
		 * Enable power save mode if bit set in flags, and if we are in station
		 * mode.  Power save is not supported in ad-hoc/mesh mode.
		 */
		piper_ps_scan_event(piperp);
		piper_ps_set(piperp, (	(conf->flags & IEEE80211_CONF_PS)
							  && (piperp->if_type == NL80211_IFTYPE_STATION)
							  && (piperp->areWeAssociated)));
	}

	/* Adjust the listen interval */
	if (IEEE80211_CONF_CHANGE_LISTEN_INTERVAL & changed) {
		tempval = piperp->ac->rd_reg(piperp, MAC_DTIM_PERIOD) & ~MAC_LISTEN_INTERVAL_MASK;
		tempval |= conf->listen_interval;
		piperp->ac->wr_reg(piperp, MAC_DTIM_PERIOD, tempval, op_write);
	}
	/* Adjust the power level */
	if ((err = piper_set_tx_power(hw, conf->power_level)) != 0) {
		dprintk(DERROR, "unable to set tx power to %d\n",
		      conf->power_level);
		return err;
	}

	/* Set channel */
	if (conf->channel->hw_value != piperp->channel) {
		piper_ps_scan_event(piperp);
		if ((err = piperp->rf->set_chan(hw, conf->channel->hw_value)) !=0) {
			dprintk(DERROR, "unable to set ch to %d\n",
				conf->channel->hw_value);
			return err;
		}
		piperp->channel = conf->channel->hw_value;
	}

	return 0;
}


/*
 * mac80211 wants to change our frame filtering settings.  We don't
 * actually support this.
 */
static void piper_hw_config_filter(struct ieee80211_hw *hw,
		unsigned int changed_flags, unsigned int *total_flags,
		u64 multicast)
{
	dprintk(DVVERBOSE, "\n");

	/* we don't support filtering so clear all flags; however, we also
	 * can't pass failed FCS/PLCP frames, so don't clear those. */
	*total_flags &= (FIF_FCSFAIL | FIF_PLCPFAIL);
}

/*
 * There are 1024 TU's (time units) to a second, and HZ jiffies to a
 * second.  This macro converts TUs to jiffies.
 */
#define TU_TO_JIFFIES(x)		(((x*HZ) + 512) / 1024)


/*
 * mac80211 calls this routine when something about our BSS has changed.
 * Usually, this routine only gets called when we associate, or disassociate.
 */
static void piper_hw_bss_changed(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			   struct ieee80211_bss_conf *conf, u32 changed)
{
	struct piper_priv *piperp = hw->priv;
	u32 reg;

	dprintk(DVVERBOSE, " changed = 0x%08x\n", changed);

	if (changed & BSS_CHANGED_ASSOC) {
		piper_ps_scan_event(piperp);
		/* Our association status has changed */
		if (piperp->if_type == NL80211_IFTYPE_STATION) {
			piper_set_status_led(hw, conf->assoc ? led_associated : led_not_associated);
		}
		piperp->areWeAssociated = conf->assoc;
		piperp->ps.aid = conf->aid;

		digi_dbg(" AP %s\n", conf->assoc ?
			"associated" : "disassociated");
	}
	if (changed & BSS_CHANGED_ERP_CTS_PROT) {
		piperp->tx_cts = conf->use_cts_prot;
	}
	if (changed & BSS_CHANGED_ERP_PREAMBLE) {
#define WANT_SHORT_PREAMBLE_SUPPORT     (1)
/* TODO: Determine if short preambles really hurt us, or if I'm just seeing things. */
#if WANT_SHORT_PREAMBLE_SUPPORT
		if (conf->use_short_preamble) {
			piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
					   BB_GENERAL_CTL_SH_PRE, op_or);
		} else {
			piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
					   ~BB_GENERAL_CTL_SH_PRE, op_and);
		}
		piperp->use_short_preamble = conf->use_short_preamble;
#else
		piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, BB_GENERAL_CTL_SH_PRE, op_or);
#endif
	}

	if (changed & BSS_CHANGED_BASIC_RATES) {
		/*
		 * The list of transmit rates has changed.  Update the
		 * rates we will receive at to match those the AP will
		 * transmit at.  This should improve our receive performance
		 * since we won't listen to junk at the wrong rate.
		 */
		unsigned int ofdm = 0, psk = 0;

		reg = piperp->ac->rd_reg(piperp, MAC_SSID_LEN) &
				         ~(MAC_OFDM_BRS_MASK | MAC_PSK_BRS_MASK);

		piperp->rf->getOfdmBrs(piperp->channel, conf->basic_rates, &ofdm, &psk);
		reg |= ofdm << MAC_OFDM_BRS_SHIFT;
		reg |= psk << MAC_PSK_BRS_SHIFT;
		piperp->ac->wr_reg(piperp, MAC_SSID_LEN, reg, op_write);

		dprintk(DVERBOSE, "BRS mask set to 0x%8.8X\n", reg);

	}

	/* Adjust the beacon interval*/
	if (BSS_CHANGED_BEACON_INT & changed) {
		reg = piperp->ac->rd_reg(piperp, MAC_CFP_ATIM) & ~MAC_BEACON_INTERVAL_MASK;
		reg |= conf->beacon_int << MAC_BEACON_INTERVAL_SHIFT;
		piperp->ac->wr_reg(piperp, MAC_CFP_ATIM, reg, op_write);
		piperp->ps.beacon_int = conf->beacon_int;
	}
	if (BSS_CHANGED_BSSID & changed
	   && !is_zero_ether_addr(conf->bssid)
	   && !is_multicast_ether_addr(conf->bssid)) {
		unsigned int bssid[2];

		piper_ps_scan_event(piperp);
		switch (vif->type) {
		case NL80211_IFTYPE_STATION:
		case NL80211_IFTYPE_ADHOC:
			dprintk(DVERBOSE, "BSSID: %2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X\n",
				conf->bssid[0], conf->bssid[1], conf->bssid[2],
				conf->bssid[3],	conf->bssid[4], conf->bssid[5]);

			bssid[0] = conf->bssid[3] | conf->bssid[2] << 8 |
				  conf->bssid[1] << 16 | conf->bssid[0] << 24;
			bssid[1] = conf->bssid[5] << 16 | conf->bssid[4] << 24;

			if ((bssid[0] == 0) && (bssid[1] == 0)) {
				/*
				* If we come here, then the MAC layer is telling us to set a 0
				* SSID.  In this case, we really want to set the SSID to the
				* broadcast address so that we receive broadcasts.
				*/
				bssid[0] = 0xffffffff;
				bssid[1] = 0xffffffff;
			}
			piperp->ac->wr_reg(piperp, MAC_BSS_ID0, bssid[0], op_write);
			piperp->ac->wr_reg(piperp, MAC_BSS_ID1, bssid[1], op_write);
			memcpy(piperp->bssid, conf->bssid, ETH_ALEN);
			break;
		default:
			break;
		}
	}
	if (   (BSS_CHANGED_BEACON & changed)
	    && (piperp->if_type == NL80211_IFTYPE_ADHOC)) {
		struct sk_buff *beacon = ieee80211_beacon_get(hw, vif);
		struct ieee80211_rate rate;

		if (!beacon)
			return;

		rate.bitrate = 10;	/* beacons always sent at 1 Megabit*/
		skb_push(beacon, TX_HEADER_LENGTH);
		phy_set_plcp(beacon->data, beacon->len - TX_HEADER_LENGTH, &rate,
		            piperp->rf->getMaxRate(piperp->rf->hw_platform,
		                piperp->rf->hw_revision, piperp->channel), 0);
		piperp->ac->wr_reg(piperp, MAC_CTL, ~MAC_CTL_BEACON_TX, op_and);
		piperp->load_beacon(piperp, beacon->data, beacon->len);

		dev_kfree_skb(beacon);          /* we are responsible for freeing this buffer*/
	}
	if (   (BSS_CHANGED_BEACON_ENABLED & changed)
	    && (piperp->if_type == NL80211_IFTYPE_ADHOC)) {
		piperp->beacon.enabled = conf->enable_beacon;
		/*
		 * This function looks at piperp->beacon.enabled and will enable
		 * or disable the automatic beacon as necessary.
		 */
		piper_enable_ibss(piperp, vif->type);
	}
	/* Save new DTIM period */
	reg = piperp->ac->rd_reg(piperp, MAC_DTIM_PERIOD) & ~MAC_DTIM_PERIOD_MASK;
	reg |= conf->dtim_period << MAC_DTIM_PERIOD_SHIFT;
	piperp->ac->wr_reg(piperp, MAC_DTIM_PERIOD, reg, op_write);
}

/*
 * Use the SSL library routines to expand the AES key.
 */
static int piper_expand_aes_key(struct ieee80211_key_conf *key,
				u32 *expandedKey)
{
	struct crypto_aes_ctx aes;
	int ret;

	dprintk(DVVERBOSE, "\n");

	if (key->keylen != AES_KEYSIZE_128)
		return -EOPNOTSUPP;

	ret = crypto_aes_expand_key(&aes, key->key, key->keylen);
	if (ret)
		return -EOPNOTSUPP;

	memcpy(expandedKey, aes.key_enc, EXPANDED_KEY_LENGTH);

	return 0;
}

/*
 * mac80211 calls this routine to set a new encryption key, or to
 * retire an old one.  We support H/W AES encryption/decryption, so
 * save the AES related keys.
 */
static int piper_hw_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
			    struct ieee80211_vif *vif, struct ieee80211_sta *sta,
			    struct ieee80211_key_conf *key)
{
	struct piper_priv *piperp = hw->priv;
	int ret = 0;

	dprintk(DVVERBOSE, "\n");

	if ((key->alg != ALG_CCMP) || (key->keyidx >= PIPER_MAX_KEYS)) {
		/*
		 * If we come here, then mac80211 was trying to set a key for some
		 * algorithm other than AES, or trying to set a key index greater
		 * than 3.  We only support AES, and only 4 keys.
		 */
		ret = -EOPNOTSUPP;
		goto set_key_error;
	}
	key->hw_key_idx = key->keyidx;

	if (cmd == SET_KEY) {
		ret = piper_expand_aes_key(key, piperp->key[key->keyidx].expandedKey);
		if (ret)
			goto set_key_error;

		if (!piperp->key[key->keyidx].valid)
			piperp->aes_key_count++;
		piperp->key[key->keyidx].txPn = 0;
		piperp->key[key->keyidx].rxPn = 0;
		piperp->key[key->keyidx].valid = (ret == 0);
		key->flags |= IEEE80211_KEY_FLAG_GENERATE_IV;
	} else {
		/* disable key */
		if (piperp->key[key->keyidx].valid)
			piperp->aes_key_count--;
		piperp->key[key->keyidx].valid = false;
	}

	if (piperp->aes_key_count > 0)
		piperp->ac->wr_reg(piperp, MAC_CTL, ~MAC_CTL_AES_DISABLE, op_and);
	else
		piperp->ac->wr_reg(piperp, MAC_CTL, MAC_CTL_AES_DISABLE, op_or);

set_key_error:
	if (ret)
		dprintk(DVERBOSE, "unable to set AES key\n");

	return ret;
}

/*
 * mac80211 calls this routine to determine if we transmitted the
 * last beacon.  Unfortunately, we can't tell for sure.  We give
 * mac80211 our best guess.
 */
static int piper_hw_tx_last_beacon(struct ieee80211_hw *hw)
{
	struct piper_priv *piperp = hw->priv;

	dprintk(DVVERBOSE, "\n");
	return piperp->beacon.weSentLastOne ? 1 : 0;
}


static int piper_get_stats(struct ieee80211_hw *hw,
			   struct ieee80211_low_level_stats *stats)
{
	struct piper_priv *piperp = hw->priv;

	dprintk(DVVERBOSE, "\n");
	if (stats)
		memcpy(stats, &piperp->pstats.ll_stats, sizeof(piperp->pstats.ll_stats));

	return 0;
}

const struct ieee80211_ops hw_ops = {
	.tx			= piper_hw_tx,
	.start 			= piper_hw_start,
	.stop			= piper_hw_stop,
	.add_interface		= piper_hw_add_intf,
	.remove_interface	= piper_hw_rm_intf,
	.config			= piper_config,
	.configure_filter 	= piper_hw_config_filter,
	.bss_info_changed 	= piper_hw_bss_changed,
	.tx_last_beacon		= piper_hw_tx_last_beacon,
	.set_key		= piper_hw_set_key,
	.get_stats 		= piper_get_stats,
};

/*
 * This routine is called by the probe routine to allocate the
 * data structure we need to communicate with mac80211.
 */
int piper_alloc_hw(struct piper_priv **priv, size_t priv_sz)
{
	struct piper_priv *piperp;
	struct ieee80211_hw *hw;

	hw = ieee80211_alloc_hw(priv_sz, &hw_ops);
	if (!hw)
		return -ENOMEM;

	hw->flags |= IEEE80211_HW_RX_INCLUDES_FCS
		  | IEEE80211_HW_SIGNAL_DBM
		  | IEEE80211_HW_SPECTRUM_MGMT
		  | IEEE80211_HW_REPORTS_TX_ACK_STATUS
		  | IEEE80211_HW_BEACON_FILTER
#if !WANT_SHORT_PREAMBLE_SUPPORT
		  | IEEE80211_HW_2GHZ_SHORT_SLOT_INCAPABLE
#endif
	             /* | IEEE80211_HW_SPECTRUM_MGMT TODO:  Turn this on when we are ready*/;

	hw->queues = 1;
	hw->extra_tx_headroom = 4 + sizeof(struct ofdm_hdr);
	piperp = hw->priv;
	*priv = piperp;
	piperp->areWeAssociated = false;
	memset(&piperp->pstats.ll_stats, 0, sizeof(piperp->pstats.ll_stats));
	piperp->hw = hw;

	return 0;
}
EXPORT_SYMBOL_GPL(piper_alloc_hw);

/*
 * This routine is called by the remove function to free the memory
 * allocated by piper_alloc_hw.
 */
void piper_free_hw(struct piper_priv *piperp)
{
	ieee80211_free_hw(piperp->hw);
}
EXPORT_SYMBOL_GPL(piper_free_hw);

/*
 * This routine is called by the probe routine to register
 * with mac80211.
 */
int piper_register_hw(struct piper_priv *priv, struct device *dev,
		      struct digi_rf_ops *rf)
{
	struct ieee80211_hw *hw = priv->hw;
	u8 macaddr[8];
	u32 temp;
	int i, ret;

	dprintk(DVVERBOSE, "\n");

	priv->rf = rf;
	for (i = 0; i < rf->n_bands; i++) {
		enum ieee80211_band b = rf->bands[i].band;
		hw->wiphy->bands[b] = &rf->bands[i];
	}
	hw->wiphy->interface_modes =   BIT(NL80211_IFTYPE_ADHOC)
	                             | BIT(NL80211_IFTYPE_STATION)
/* TODO: Enable this             | BIT(NL80211_IFTYPE_MESH_POINT) */
	                             ;
	hw->channel_change_time = rf->channelChangeTime;
	hw->vif_data_size = 0;
	hw->sta_data_size = 0;
	hw->max_rates = IEEE80211_TX_MAX_RATES;
	hw->max_rate_tries = 5;             /* completely arbitrary, and apparently ignored by the rate algorithm */
	hw->max_signal = rf->maxSignal;
	hw->max_listen_interval = 10;       /* I don't think APs will work with values larger than 4 actually */

	SET_IEEE80211_DEV(hw, dev);

	temp = cpu_to_be32(priv->ac->rd_reg(priv, MAC_STA_ID0));
	memcpy(macaddr, &temp, sizeof(temp));
	temp = cpu_to_be32(priv->ac->rd_reg(priv, MAC_STA_ID1));
	memcpy(&macaddr[4], &temp, sizeof(temp));
	SET_IEEE80211_PERM_ADDR(hw, macaddr);
	hw->rate_control_algorithm = NULL;

	if ((ret = ieee80211_register_hw(hw)) != 0) {
		dprintk(DERROR, "unable to register ieee80211 hw, ret = %d\n", ret);
		goto error;
	}

	printk(KERN_INFO PIPER_DRIVER_NAME ": registered as %s\n",
	      	wiphy_name(hw->wiphy));

	init_timer(&priv->led_timer);
	priv->led_state = led_shutdown;
	priv->led_timer.function = led_timer_fn;
	priv->led_timer.data = (unsigned long) priv;
	priv->led_timer.expires = jiffies + LED_TIMER_RATE;
	add_timer(&priv->led_timer);

error:
	return ret;
}
EXPORT_SYMBOL_GPL(piper_register_hw);


void piper_unregister_hw(struct piper_priv *piperp)
{
	dprintk(DVVERBOSE, "\n");
	del_timer_sync(&piperp->led_timer);
	piper_set_status_led(piperp->hw, led_shutdown);
	ieee80211_unregister_hw(piperp->hw);
}
EXPORT_SYMBOL_GPL(piper_unregister_hw);


MODULE_DESCRIPTION("Digi Piper WLAN core");
MODULE_AUTHOR("contact support@digi.com for information about this code");
MODULE_LICENSE("GPL");

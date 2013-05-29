/*
 * digiTx.c
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
#include <linux/jiffies.h>
#include <linux/timer.h>

#include "pipermain.h"
#include "mac.h"
#include "phy.h"
#include "digiPs.h"

#define FRAME_CONTROL_FIELD_OFFSET      (sizeof(struct tx_frame_hdr) + sizeof(struct psk_cck_hdr))
#define AES_TIMEOUT			(200)

#define TX_DEBUG			(1)

#if TX_DEBUG
//static int dlevel = DWARNING;
#define dprintk(level, fmt, arg...)	if (level >= dlevel)			\
					printk(KERN_ERR PIPER_DRIVER_NAME	\
					    ": %s - " fmt, __func__, ##arg)
#else
#define dprintk(level, fmt, arg...)	do {} while (0)
#endif

/*
 * Adds an entry into the tx queue.
 */
int piper_tx_enqueue(struct piper_priv *piperp, struct sk_buff *skb, tx_skb_return_cb_t skb_return_cb)
{
	unsigned long flags;
	int result = -1;

	spin_lock_irqsave(&piperp->tx_queue_lock, flags);
	if (NEXT_TX_QUEUE_INDEX(piperp->tx_queue_head) != piperp->tx_queue_tail) {
		piperp->tx_queue[piperp->tx_queue_head].skb = skb;
		piperp->tx_queue[piperp->tx_queue_head].skb_return_cb = skb_return_cb;
		piperp->tx_queue_head = NEXT_TX_QUEUE_INDEX(piperp->tx_queue_head);
		piperp->tx_queue_count++;
		result = 0;
	}
	spin_unlock_irqrestore(&piperp->tx_queue_lock, flags);

	return result;
}
EXPORT_SYMBOL_GPL(piper_tx_enqueue);


/*
 * Returns the skb for the current element.
 */
struct sk_buff *piper_tx_getqueue(struct piper_priv *piperp)
{
	if (piperp->tx_queue_head == piperp->tx_queue_tail) {
		return NULL;
	} else {
		return piperp->tx_queue[piperp->tx_queue_tail].skb;
	}
}
EXPORT_SYMBOL_GPL(piper_tx_getqueue);

/*
 * Returns the skb buffer call back for the current element.
 */
static inline tx_skb_return_cb_t piper_tx_getqueue_return_fn(struct piper_priv *piperp)
{
	return piperp->tx_queue[piperp->tx_queue_tail].skb_return_cb;
}


/*
 * Called to advance the queue tail.
 */
static inline void piper_tx_queue_next(struct piper_priv *piperp)
{
	unsigned long flags;

	spin_lock_irqsave(&piperp->tx_queue_lock, flags);
	piperp->pstats.tx_retry_index = 0;
	piperp->pstats.tx_total_tetries = 0;
	memset(piperp->pstats.tx_retry_count, 0, sizeof(piperp->pstats.tx_retry_count));
	if (piperp->tx_queue_head != piperp->tx_queue_tail) {
		piperp->tx_queue[piperp->tx_queue_tail].skb = NULL;
		piperp->tx_queue[piperp->tx_queue_tail].skb_return_cb = NULL;
		piperp->tx_queue_tail = NEXT_TX_QUEUE_INDEX(piperp->tx_queue_tail);
		piperp->tx_queue_count--;
	}
	spin_unlock_irqrestore(&piperp->tx_queue_lock, flags);
}

/*
 * Called when we unload to clear any remaining queue entries.
 */
void piper_empty_tx_queue(struct piper_priv *piperp)
{
	while (piper_tx_getqueue(piperp)) {
		struct ieee80211_tx_info *info = IEEE80211_SKB_CB(piper_tx_getqueue(piperp));

		ieee80211_tx_info_clear_status(info);
		piper_tx_getqueue_return_fn(piperp)(piperp->hw, piper_tx_getqueue(piperp));
		piper_tx_queue_next(piperp);
	}
}
EXPORT_SYMBOL_GPL(piper_empty_tx_queue);


bool piper_tx_queue_half_full(struct piper_priv *piperp)
{
	return (piperp->tx_queue_count >= (PIPER_TX_QUEUE_SIZE >> 1));
}
EXPORT_SYMBOL_GPL(piper_tx_queue_half_full);



/*
 * This routine writes a frame using H/W AES encryption.
 *
 * Arguments
 *      digi            context
 *      buffer          pointer to start of frame
 *      length          number of bytes in frame
 *
 * Return Values
 *      0           success
 *      !0          transmit failed
 */
static int piper_write_aes(struct piper_priv *piperp, unsigned char *buffer,
			   unsigned int length)
{
	int result;
	int timeout = AES_TIMEOUT;
	unsigned long spinLockFlags;

	/*
	 * Step 1: Wait for AES to become ready.
	 */
	spin_lock_irqsave(&piperp->aesLock, spinLockFlags);
	while (piperp->ac->rd_reg(piperp, BB_RSSI) & BB_RSSI_EAS_BUSY) {
		timeout--;
		if (timeout == 0) {
			/*
			 * If we come here, then AES busy appears to be stuck high.  It should only be
			 * high for a maximum of about 80 us when it is encrypting a transmit frame.
			 * Our timeout value is high enough to guarantee that the engine has had enough
			 * time to complete the transmit.  Apparently there is data stuck in the FIFO
			 * from either a previous transmit or receive.
			 */
			digi_dbg("write AES, AES busy stuck on\n");
			digiWifiDumpRegisters(piperp, MAIN_REGS | MAC_REGS);
			/*
			 * We recover by simply continuing on.  Step 3 writes to the AES control
			 * register.  This will reset the AES engine and clear the error condition.
			 */
			break;
		}
		udelay(1);
	}

	/*
	 * Step 2: Write the unencrypted part of the frame into the normal
	 *         data FIFO.
	 */
	piperp->ac->wr_fifo(piperp, BB_DATA_FIFO, buffer,
		      _80211_HEADER_LENGTH + TX_HEADER_LENGTH + PIPER_EXTIV_SIZE);

	/*
	 * Step 3: Write to the AES control register.  Writing to it puts
	 *         AES H/W engine into transmit mode.  We also make sure
	 *         the AES mode is set correctly.
	 */
	piperp->ac->wr_reg(piperp, BB_AES_CTL, 0, op_write);

	/*
	 * Step 4: Write the expanded AES key into the AES FIFO.
	 */
	piperp->ac->wr_fifo(piperp, BB_AES_FIFO,
		      (unsigned char *)piperp->key[piperp->tx_aes_key].expandedKey,
		      EXPANDED_KEY_LENGTH);

	/*
	 * Step 5: Write the AES IV and headers into the AES FIFO.
	 */
	piperp->ac->wr_fifo(piperp, BB_AES_FIFO, (unsigned char *)piperp->tx_aes_blob,
			    AES_BLOB_LENGTH);

	/*
	 * Step 6: Now, finally, write the part of the frame that needs to
	 *         be encrypted into the AES FIFO.
	 */
	result =
	    piperp->ac->wr_fifo(piperp, BB_AES_FIFO,
			  &buffer[_80211_HEADER_LENGTH + TX_HEADER_LENGTH + PIPER_EXTIV_SIZE],
			  length - (_80211_HEADER_LENGTH + TX_HEADER_LENGTH +
				    PIPER_EXTIV_SIZE));

	spin_unlock_irqrestore(&piperp->aesLock, spinLockFlags);

	return result;
}

/*
 * Determine what bit rate the next retry should be sent at.
 *
 * The mac80211 library passes us an array of tx bit rates.  Each entry
 * has a rate index and a limit (max number of retries at that rate).
 * We use the rate index to build the H/W transmit header.  The limit
 * is decremented each time we retry.  When it reaches zero, we try the
 * next rate in the array.
 */
static struct ieee80211_rate *get_tx_rate(struct piper_priv *piperp, struct ieee80211_tx_info *info)
{
	struct ieee80211_rate *ret = NULL;

	if (piperp->pstats.tx_retry_count[piperp->pstats.tx_retry_index] >=
	    info->control.rates[piperp->pstats.tx_retry_index].count) {
		piperp->pstats.tx_retry_index++;
	}

	if (piperp->pstats.tx_retry_index >= IEEE80211_TX_MAX_RATES) {
	    return NULL;        /* don't go beyond the end of the rates array */
	}

	if (piperp->pstats.tx_retry_index == 0) {
		ret = ieee80211_get_tx_rate(piperp->hw, info);
	} else {
		ret = ieee80211_get_alt_retry_rate(piperp->hw, info, piperp->pstats.tx_retry_index - 1);
	}

	if (ret != NULL) {
		if (piperp->calibrationTxRate) {
			ret = piperp->calibrationTxRate;
		}
	}

	ret = piper_ps_check_rate(piperp, ret);

	return ret;
}

/*
 * This function returns a value for the contention window in microseconds.  We
 * start with the contention window at CW_MIN and double it everytime we have to
 * retry.
 */
static u16 piper_get_cw(struct piper_priv *piperp, bool isFirstTime)
{
	static u16 cw = DEFAULT_CW_MIN;

	if (isFirstTime) {
		cw = DEFAULT_CW_MIN;
	} else {
		cw <<= 1;
		if (cw > DEFAULT_CW_MAX) {
			cw = DEFAULT_CW_MAX;
		}
	}
	return (cw + (10 * (piperp->rand() & (cw - 1)))) & 0xffff;
}

/*
 * This function will prepend an RTS or CTS to self frame ahead of the current
 * TX frame.  This is done if the wantRts or wantCts flag is set.  The mac80211
 * library determines if either of these flags is set.
 *
 * The RTS or CTS message is written into the transmit FIFO ahead of the
 * data frame.  Note that RTS and CTS messages are always sent in the clear
 * so we do not have to worry about encryption.
 *
 * Our caller, the master transmit routine, is responsible for setting the
 * transmit hold bit before calling us and clearing it after the data frame
 * has been written into the FIFO.  This ensures that the RTS/CTS frame is
 * not transmitted until after the data frame is ready to go.
 *
 * Also note that if we are unable to send the RTS/CTS frame, then the H/W
 * is smart enough to also about the data frame.  So we will not send
 * the data frame without the RTS/CTS frame.
 */
static void handle_rts_cts(struct piper_priv *piperp,
			   struct ieee80211_tx_info *txInfo, unsigned int frameType)
{
	piperp->tx_rts = false;

	if (frameType == TYPE_DATA) {
		unsigned int header[2];
		struct ieee80211_rate *rate = NULL;
		bool wantCts = (!!(txInfo->control.rates[piperp->pstats.tx_retry_index].flags
				   & IEEE80211_TX_RC_USE_CTS_PROTECT)
				| piperp->tx_cts);
		bool wantRts = !!(txInfo->control.rates[piperp->pstats.tx_retry_index].flags
				  & IEEE80211_TX_RC_USE_RTS_CTS);

		if ((wantRts) || (wantCts)) {
			/*
			 * If we are sending an RTS or a CTS, then get the rate information.
			 */
			if (piperp->calibrationTxRate) {
				rate = piperp->calibrationTxRate;
			} else {
				rate = ieee80211_get_rts_cts_rate(piperp->hw, txInfo);
			}
			if (rate == NULL) {
				digi_dbg
				    ("ieee80211_get_rts_cts_rate(digi->hw, txInfo) returned NULL!\n");
			}
		}
		if ((wantRts) && (rate)) {
			/*
			 * We're sending an RTS, so load it into the FIFO.
			 */
			struct ieee80211_rts rtsFrame;

			ieee80211_rts_get(piperp->hw, txInfo->control.vif,
					  piper_tx_getqueue(piperp)->data + TX_HEADER_LENGTH,
					  piper_tx_getqueue(piperp)->len - TX_HEADER_LENGTH, txInfo,
					  &rtsFrame);
			/*
			 * If we come here, then we need to send an RTS frame ahead of the
			 * current data frame.
			 */
			phy_set_plcp((unsigned char *)header, sizeof(struct ieee80211_rts), rate,
				     piperp->rf->getMaxRate(piperp->rf->hw_platform,
				                piperp->rf->hw_revision, piperp->channel), 0);
			piperp->ac->wr_fifo(piperp, BB_DATA_FIFO, (unsigned char *)header,
				      TX_HEADER_LENGTH);
			piperp->ac->wr_fifo(piperp, BB_DATA_FIFO, (unsigned char *)&rtsFrame,
				      sizeof(rtsFrame));
			if (piperp->pstats.tx_total_tetries != 0) {
				piperp->pstats.ll_stats.dot11RTSFailureCount++;
			}
			piperp->tx_rts = true;
		} else if ((wantCts) && (rate)) {
			/*
			 * We're sending a CTS, so load it into the FIFO.
			 */
			struct ieee80211_cts ctsFrame;

			ieee80211_ctstoself_get(piperp->hw, txInfo->control.vif,
						piper_tx_getqueue(piperp)->data + TX_HEADER_LENGTH,
						piper_tx_getqueue(piperp)->len - TX_HEADER_LENGTH,
						txInfo, &ctsFrame);
			/*
			 * At the time this code was written, the mac80211 library had
			 * a bug in the ieee80211_ctstoself_get which caused it to copy
			 * the wrong MAC address into the cts frame.  So we copy the
			 * right one (ours) in now.
			 */
			memcpy(piperp->ctsFrame.ra, piperp->hw->wiphy->perm_addr, ETH_ALEN);

			/*
			 * If we come here, then we need to send a CTS to self frame ahead of the
			 * current data frame.
			 */
			phy_set_plcp((unsigned char *)header, sizeof(struct ieee80211_cts), rate,
				     piperp->rf->getMaxRate(piperp->rf->hw_platform,
				                piperp->rf->hw_revision, piperp->channel), 0);
			piperp->ac->wr_fifo(piperp, BB_DATA_FIFO, (unsigned char *)header,
				      TX_HEADER_LENGTH);
			piperp->ac->wr_fifo(piperp, BB_DATA_FIFO, (unsigned char *)&ctsFrame,
				      sizeof(ctsFrame));
		}
	}
}

/*
 * This routine is called to report the result of a transmit operation to
 * mac80211.  It is used for both successful transmissions and failures.
 * It sends the result to the stack, removes the current tx frame from the
 * queue, and then wakes
 * up the transmit queue.
 */
void packet_tx_done(struct piper_priv *piperp, tx_result_t result,
			   int signal_strength)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(piper_tx_getqueue(piperp));
	int i;
	struct sk_buff *skb;
	unsigned long flags;

#define WANT_TRANSMIT_RESULT 	(0)
#if WANT_TRANSMIT_RESULT
	const char *resultText[] =
	{
		"RECEIVED_ACK",
		"TX_COMPLETE",
		"OUT_OF_RETRIES",
		"TX_NOT_DONE"
	};
#endif
	del_timer_sync(&piperp->tx_timer);
        piperp->expectingAck = false;

#if WANT_TRANSMIT_RESULT
	printk(KERN_ERR "Transmit result %s\n", resultText[result]);
#endif
	if (piperp->tx_calib_cb)
		piperp->tx_calib_cb(piperp);

	if (piper_tx_getqueue(piperp) != NULL) {
		skb_pull(piper_tx_getqueue(piperp), TX_HEADER_LENGTH);

		ieee80211_tx_info_clear_status(info);

		/* prepare statistics and pass them to the stack */
		for (i = 0; i < IEEE80211_TX_MAX_RATES; i++) {
			info->status.rates[i].count = piperp->pstats.tx_retry_count[i];
			if (info->status.rates[i].count == 0)
				info->status.rates[i].idx = -1;
		}

		info->status.ack_signal = signal_strength;
		info->flags |= (result == RECEIVED_ACK) ? IEEE80211_TX_STAT_ACK : 0;
		piperp->pstats.tx_complete_count++;
		if (piperp->tx_rts)
			piperp->pstats.ll_stats.dot11RTSSuccessCount++;

		skb = piper_tx_getqueue(piperp);
		piper_tx_getqueue_return_fn(piperp)(piperp->hw, skb);

		piper_tx_queue_next(piperp);

		if (piper_tx_getqueue(piperp) == NULL) {
			spin_lock_irqsave(&piperp->tx_tasklet_lock, flags);
			piperp->tx_tasklet_running = false;
			spin_unlock_irqrestore(&piperp->tx_tasklet_lock, flags);
			if (piperp->ps.allowTransmits) {
				ieee80211_wake_queues(piperp->hw);
			} else {
				/*
				 * Do not wake up the mac80211 Tx queue if we are trying to power
				 * down.  Make sure we set the stopped_tx_queues flag so that we
				 * know to restart the queues.
				 */
				piperp->ps.stopped_tx_queues = true;
			}
		} else {
			if (result == OUT_OF_RETRIES) {
				/*
				 * If we come here, then we are being called from the middle of the
				 * transmit routine and we have to reschedule the transmit task to
				 * start dequeueing the next frame.
				 */
				tasklet_hi_schedule(&piperp->tx_tasklet);
			}
		}
	} else {
		digi_dbg("packet_tx_done called with empty queue\n");
	}

	piperp->tx_result = TX_NOT_DONE;
}
EXPORT_SYMBOL_GPL(packet_tx_done);

/*
 * This function is the entry point for the transmit tasklet.  It
 * is called to transmit frames.  It will first be called to transmit
 * the frame and then to retry if the original transmit fails.  So
 * it does both the first transmit and the subsequent retries.
 *
 * Arguments
 *      context     context information
 */
void piper_tx_tasklet(unsigned long context)
{
	struct piper_priv *piperp = (struct piper_priv *)context;
	frameControlFieldType_t *fc;
	int err;

	piperp->expectingAck = false;
	del_timer_sync(&piperp->tx_timer);
	if ((piperp->tx_result == RECEIVED_ACK) || (piperp->tx_result == TX_COMPLETE)) {
		/*
		 * We will come here if the receiver task received an ACK, or if we got
		 * a tx fifo empty interrupt.  In these cases the receiver thread or ISR
		 * schedule the tx tasklet to handle the event rather than calling
		 * packet_tx_done directly.
		 */
		packet_tx_done(piperp, piperp->tx_result, piperp->tx_signal_strength);
	}


	/*
	 * Clear flags here to cover ACK case.  We do not clear the flags in the ACK
	 * routine since it is possible to receive an ACK after we have started the
	 * next packet.  The appropriate interrupts will be reenabled if we decide
	 * to retransmit.
	 */
	piperp->clear_irq_mask_bit(piperp,
				BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT |
				BB_IRQ_MASK_TX_ABORT);

	if (piper_tx_getqueue(piperp) != NULL) {
		struct ieee80211_tx_info *txInfo = IEEE80211_SKB_CB(piper_tx_getqueue(piperp));
		struct ieee80211_rate *txRate = get_tx_rate(piperp, txInfo);

		if (txRate != NULL) {
			fc = (frameControlFieldType_t *)
					&piper_tx_getqueue(piperp)->data[FRAME_CONTROL_FIELD_OFFSET];

			/* set the retry bit if this is not the first try */
			if (piperp->pstats.tx_retry_count[0] != 0)
				fc->retry = 1;

			piperp->ac->wr_reg(piperp, MAC_BACKOFF,
					   piper_get_cw(piperp,
					   (piperp->pstats.tx_retry_count[0] == 0)),
					   op_write);

			/*
			 * Build the H/W transmit header.  The transmit header is rebuilt on each
			 * retry because it has the TX rate information which may change for
			 * retries.
			 */
			phy_set_plcp(piper_tx_getqueue(piperp)->data,
				     piper_tx_getqueue(piperp)->len - TX_HEADER_LENGTH,
				     txRate, piperp->rf->getMaxRate(piperp->rf->hw_platform,
				                piperp->rf->hw_revision, piperp->channel),
				     piperp->use_hw_aes ? 8 : 0);

			/*
			 * Pause the transmitter so that we don't start transmitting before we
			 * are ready.
			 */
			piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, BB_GENERAL_CTL_TX_HOLD, op_or);

			handle_rts_cts(piperp, txInfo, fc->type);

			if (piperp->use_hw_aes == true && txInfo->control.hw_key != NULL)  {
				err =
				    piper_write_aes(piperp, piper_tx_getqueue(piperp)->data,
						    piper_tx_getqueue(piperp)->len);
			} else {
				err =
				    piperp->ac->wr_fifo(piperp, BB_DATA_FIFO, piper_tx_getqueue(piperp)->data,
						piper_tx_getqueue(piperp)->len);
			}

			/* Clear any pending TX interrupts */
			piperp->ac->wr_reg(piperp, BB_IRQ_STAT,
					  BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT |
					  BB_IRQ_MASK_TX_ABORT, op_write);

			/*
			 * Now start the transmitter.
			 */
			piperp->expectingAck = ((txInfo->flags & IEEE80211_TX_CTL_NO_ACK) == 0);
			piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~BB_GENERAL_CTL_TX_HOLD,
					  op_and);

			/*
			 * Set interrupt flags.  Use the timeout interrupt if we expect
			 * an ACK.  Use the FIFO empty interrupt if we do not expect an ACK.
			 */
			if (txInfo->flags & IEEE80211_TX_CTL_NO_ACK) {
				piperp->set_irq_mask_bit(piperp,
							 BB_IRQ_MASK_TX_FIFO_EMPTY |
							 BB_IRQ_MASK_TX_ABORT);
			} else {
				/*
				 * We set up a timer to fire in 1/4 second.  We should not need it, but somehow
				 * we seem to miss a timeout interrupt occasionally.  Perhaps we encounter a receive
				 * overrun which causes the H/W to discard the ACK packet without generating
				 * a timeout.
				 */
				piperp->tx_timer.expires = jiffies + (HZ >> 2);
				add_timer(&piperp->tx_timer);

				/*
				 * Also set the IRQ mask to listen for timeouts and TX aborts.  We will receive
				 * an ACK (which is handled by the RX routine) if the TX is successful.
				 */
				piperp->set_irq_mask_bit(piperp,
							 BB_IRQ_MASK_TIMEOUT |
							 BB_IRQ_MASK_TX_ABORT);
			}
			if ((piperp->pstats.tx_total_tetries != 0) &&
			    ((txInfo->flags & IEEE80211_TX_CTL_NO_ACK) == 0)) {
				piperp->pstats.ll_stats.dot11ACKFailureCount++;
			}
			piperp->pstats.tx_retry_count[piperp->pstats.tx_retry_index]++;
			piperp->pstats.tx_total_tetries++;
		} else {
			packet_tx_done(piperp, OUT_OF_RETRIES, 0);
		}
	} else {
		long unsigned int flags;

		spin_lock_irqsave(&piperp->tx_tasklet_lock, flags);
		piperp->tx_tasklet_running = false;
		spin_unlock_irqrestore(&piperp->tx_tasklet_lock, flags);
	}
}
EXPORT_SYMBOL_GPL(piper_tx_tasklet);




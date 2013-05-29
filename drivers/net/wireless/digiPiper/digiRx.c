/*
 * digiRx.c
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
#include "digiPs.h"

#define WANT_RECEIVE_COUNT_SCROLL	(0)
#define AES_TIMEOUT			(200)
#define RX_DEBUG			(1)

#if RX_DEBUG
static int dlevel = DWARNING;
#define dprintk(level, fmt, arg...)	if (level >= dlevel)			\
					printk(KERN_ERR PIPER_DRIVER_NAME	\
					    ": %s - " fmt, __func__, ##arg)
#else
#define dprintk(level, fmt, arg...)	do {} while (0)
#endif


/*
 * This routine is called to flush the receive and transmit FIFOs.  It is used
 * for error recovery when we detect corrupted data in the FIFO's.  It should be
 * called with the AES lock set.
 */
static void reset_fifo(struct piper_priv *piperp)
{
	unsigned int i;

	piperp->ac->rd_reg(piperp, BB_AES_CTL);
	piperp->ac->wr_reg(piperp, BB_AES_CTL, 0, op_write);
	// clear the TX-FIFO memory
	for (i = 0; i < 448; i++)
		piperp->ac->wr_reg(piperp, BB_DATA_FIFO, 0, op_write);

	// clear RX-FIFO memory
	for (i = 0; i < 512; i++)
		piperp->ac->rd_reg(piperp, BB_DATA_FIFO);
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, BB_GENERAL_CTL_RXFIFORST, op_or);
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~BB_GENERAL_CTL_RXFIFORST, op_and);
}

/*
 * This routine is called to receive a frame.  The hardware header has
 * already been read from the FIFO.  We need to read out the frame.  If
 * the frame was encrypted with AES and we have the correct key, then
 * we use the AES H/W encryption engine to decrypt the frame.  We also
 * set up a ieee80211_rx_status structure with the appropriate info.
 *
 * Arguments
 *      digi                context information
 *      skb                 empty buffer to receive packet into
 *      length              number of bytes in FIFO
 *      fr_ctrl_field   buffer to copy frame control header into
 *      status              status structure we must write status into
 *
 * Returns
 *      true            frame was received
 *      false           an encryption error was detected
 */
static bool receive_packet(struct piper_priv *piperp, struct sk_buff *skb, int length,
			  frameControlFieldType_t * fr_ctrl_field,
			  struct ieee80211_rx_status *status)
{
	_80211HeaderType *header;
	bool result = true;
	int originalLength = length;
	int headerlen;
#if WANT_RECEIVE_COUNT_SCROLL
	static int packetCount = 0;
#endif

	headerlen = _80211_HEADER_LENGTH;
	if (length <  _80211_HEADER_LENGTH) {
		/*
		 * If we branch here, then there is not enough data to make a
		 * complete header.  This is possible if this is a control frame.
		 * Adjust our length so that we do not read too much data from
		 * the FIFO.
		 */
		headerlen = length;
	}

	/*
	 * Read the frame header.  This includes the frame control fields
	 * as well as the 802.11 header.
	 */
	header = (_80211HeaderType *) skb_put(skb, headerlen);
	length -= headerlen;
	piperp->ac->rd_fifo(piperp, BB_DATA_FIFO, (uint8_t *) header, headerlen);
	memcpy(fr_ctrl_field, &header->fc, sizeof(fr_ctrl_field));

 	if (((u32)(skb->tail)) & 0x3) {
		/* align data */
		skb_reserve(skb, 4 - ((u32)(skb->tail) & 0x3));
 	}

	if (header->fc.protected) {
		/*
		 * If we branch here, then the frame is encrypted.  We need
		 * to figure out if we should try to decrypt it.
		 */
		unsigned char *rsnHeader;
		unsigned int aesDataBlob[AES_BLOB_LENGTH / sizeof(unsigned int)];
		unsigned int keyIndex;

		rsnHeader = skb_put(skb, PIPER_EXTIV_SIZE);

		/*
		 * Step 1: Read the rest of the unencrypted data, which should
		 *         consist of the extiv fields.
		 */
		piperp->ac->rd_fifo(piperp, BB_DATA_FIFO, rsnHeader, PIPER_EXTIV_SIZE);
		length -= PIPER_EXTIV_SIZE;
		keyIndex = rsnHeader[3] >> 6;

		if (piper_prepare_aes_datablob(piperp, keyIndex, (u8 *) aesDataBlob,
					       (unsigned char *)header, originalLength - 12,
					       false)) {
			/*
			 * If we come here, then we have the correct encryption key for
			 * the frame and will now try to decrypt it.
			 */
			unsigned int timeout = AES_TIMEOUT;
			unsigned long flags;

			spin_lock_irqsave(&piperp->aesLock, flags);

			/*
			 * Step 2: Wait for AES to become ready.
			 */
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
					dprintk(DWARNING, "1st AES busy never became ready\n");
					digiWifiDumpRegisters(piperp, MAIN_REGS | MAC_REGS);
					/*
					 * Recover by flushing the FIFO and returning in error.
					 */
					reset_fifo(piperp);
#if 0
					/*
					 * TODO: Figure out why this code snippet doesn't work.  I would think
					 * that if we reset the fifo, we should just return in error since we will
					 * have discarded the frame.  However, when we do that the system hangs
					 * (after a while).  This doesn't make sense.
					 */
					spin_unlock_irqrestore(&piperp->aesLock, flags);
					result = false;
					goto receive_packet_exit;
#else
					break;
#endif
				}
				udelay(1);
			}

			/*
			 * Step 3: Set the AES mode, and then read from the AES control
			 *         register to put the AES engine into receive mode.
			 */
			piperp->ac->rd_reg(piperp, BB_AES_CTL);

			/*
			 * Step 4: Write the expanded AES key into the AES FIFO.
			 */
			piperp->ac->wr_fifo(piperp, BB_AES_FIFO,
				      (unsigned char *)piperp->key[keyIndex].expandedKey,
				      EXPANDED_KEY_LENGTH);

			/*
			 * Step 5: Write the AES IV and headers into the AES FIFO.
			 */
			piperp->ac->wr_fifo(piperp, BB_AES_FIFO, (unsigned char *)aesDataBlob,
				      AES_BLOB_LENGTH);

			/*
			 * Step 6: Now, finally, read the unencrypted frame from the
			 *         AES FIFO.  Adjust the length so that we don't try
			 *         to read the MIC or the ICV which the AES engine will
			 *         process for us.
			 */
			length -= MIC_SIZE + ICV_SIZE;
			piperp->ac->rd_fifo(piperp, BB_AES_FIFO, skb_put(skb, length), length);
			/*
			 * mac80211 seems to expect there to be a MIC even if the packet
			 * has already been decrypted.  It will drop off what it thinks
			 * are the extra MIC bytes, so add some extra bytes that it
			 * can drop off without losing any data.
			 */
			skb_put(skb, MIC_SIZE);	/* add fake MIC */

			/*
			 * Step 7: Wait for AES to become ready.
			 */
			while (piperp->ac->rd_reg(piperp, BB_RSSI) & BB_RSSI_EAS_BUSY) {
				timeout--;
				if (timeout == 0) {
					dprintk(DWARNING, "2nd AES busy never became ready\n");
					digiWifiDumpRegisters(piperp, MAIN_REGS | MAC_REGS);
				}
				udelay(1);
			}
			result = ((piperp->ac->rd_reg(piperp, BB_RSSI) & BB_RSSI_EAS_MIC) != 0);
			timeout = 500;
			while ((piperp->ac->rd_reg(piperp, BB_RSSI) & BB_RSSI_EAS_FIFO_EMPTY) == 0) {
				timeout--;
				piperp->ac->rd_reg(piperp, BB_AES_FIFO);
				udelay(1);
			}
#ifdef WANT_DEBUG
			if (piperp->ac->rd_reg(piperp, BB_RSSI) & BB_RSSI_EAS_BUSY) {
				digi_dbg("AES busy set at end of rx\n");
			}
#endif
			spin_unlock_irqrestore(&piperp->aesLock, flags);

			/*  pad an extra 8 bytes for the MIC which the H/W strips */
			skb_put(skb, 8);
			if (result) {
				status->flag |= RX_FLAG_DECRYPTED;
			} else {
				digi_dbg("Error decrypting packet\n");
			}
		} else {
			/*
			 * If we branch here, then we are not able to decrypt the
			 * packet possibly because we don't have the key, or because
			 * the packet was encrypted using TKIP.  Read the rest of the
			 * encrypted data.  mac80211 will have to decrypt it in software.
			 */
			piperp->ac->rd_fifo(piperp, BB_DATA_FIFO, skb_put(skb, length),
					    length);
		}
	} else {
		/*
		 * Frame is not encrypted, so just read it.
		 */
		piperp->ac->rd_fifo(piperp, BB_DATA_FIFO, skb_put(skb, length), length);
	}

#if WANT_RECEIVE_COUNT_SCROLL
	if (((++packetCount) & 1023) == 0) {
		printk(KERN_ERR "\n%d recd, tx_start_count = %d, tx_complete_count = %d.\n",
			packetCount, piperp->pstats.tx_start_count,
			piperp->pstats.tx_complete_count);
	}
#endif
#if 0
receive_packet_exit:
#endif
	return result;
}

/*
 * This routine is called when we receive an ACK.  This should be after
 * we have transmitted a packet.  We need to tell the upper layer we
 * have a packet by calling ieee80211_tx_status_irqsafe with status
 * information.  The transmit routine also disables queuing whenever we
 * transmit since we can only transmit one packet at a time, so we need
 * to reenable to transmit queue too.
 */
static inline void handle_ack(struct piper_priv *piperp, int signal_strength)
{
	if (piper_tx_getqueue(piperp) && piperp->expectingAck) {
		struct ieee80211_tx_info *txInfo = IEEE80211_SKB_CB(piper_tx_getqueue(piperp));
		if ((txInfo->flags & IEEE80211_TX_CTL_NO_ACK) == 0) {
			piperp->clear_irq_mask_bit(piperp,
						   BB_IRQ_MASK_TX_FIFO_EMPTY |
						   BB_IRQ_MASK_TIMEOUT |
						   BB_IRQ_MASK_TX_ABORT);
			piperp->tx_signal_strength = signal_strength;
			piperp->tx_result = RECEIVED_ACK;
			tasklet_hi_schedule(&piperp->tx_tasklet);
		}
	}
}


/*
 * This is the entry point for the receive tasklet.  It is executed
 * to process receive packets.  It allocates an SKB and receives
 * the packet into it.
 *
 * We may be called from the receive ISR if WANT_TO_RECEIVE_FRAMES_IN_ISR
 * is set.
 */
void piper_rx_tasklet(unsigned long context)
{
	struct piper_priv *piperp = (struct piper_priv *)context;

	/*
	 * This while loop will keep executing as long as the H/W indicates there
	 * are more frames in the FIFO to be received.
	 */

	piperp->ps.rxTaskletRunning = true;

	while (((piperp->ac->rd_reg(piperp, BB_GENERAL_STAT) &
			BB_GENERAL_STAT_RX_FIFO_EMPTY) == 0)
			&& (!piperp->ps.poweredDown)) {
		struct sk_buff *skb = NULL;
		struct ieee80211_rx_status status = { 0 };
		struct rx_frame_hdr header;
		unsigned int length = 0;
		frameControlFieldType_t fr_ctrl_field;

		/*
		 * Read and process the H/W header.  This header is created by
		 * the hardware is is not part of the frame.
		 */
		piperp->ac->rd_fifo(piperp, BB_DATA_FIFO, (u8 *)&header, sizeof(header));
		phy_process_plcp(piperp, &header, &status, &length);
		if ((length == 0) || (length > (RX_FIFO_SIZE - 48))) {	/* 48 bytes for padding and related stuff */
			unsigned long flags;

			dprintk(DERROR, "bogus frame length (%d)\n", length);
			dprintk(DERROR, "0x%08x 0x%08x\n", *(u32 *)&header, *(((u32 *)&header) + 1));
			spin_lock_irqsave(&piperp->aesLock, flags);
			reset_fifo(piperp);
			spin_unlock_irqrestore(&piperp->aesLock, flags);
			continue;
		}

		if (length != 0) {

			skb = __dev_alloc_skb(RX_FIFO_SIZE + 100, GFP_ATOMIC);
			if (skb == NULL) {
				/* Oops.  Out of memory.  Exit the tasklet */
				dprintk(DERROR, "__dev_alloc_skb failed\n");
				break;
			}

			if (receive_packet(piperp, skb, length, &fr_ctrl_field, &status)) {

				if (length >= _80211_HEADER_LENGTH)
				{
					/*
					 * If using the Airoha transceiver, then we want to monitor
					 * the receive signal strength and continuously adjust the
					 * receive amplifier so that we get the best possible signal
					 * to noise ratio.
					 */
					unsigned int rssi = phy_determine_rssi(&header);

					piperp->adjust_max_agc(piperp, rssi, (_80211HeaderType *) skb->data);
				}

				if (fr_ctrl_field.type == TYPE_ACK)
					handle_ack(piperp, status.signal);

				if ((fr_ctrl_field.type == TYPE_ACK)
				    || (fr_ctrl_field.type == TYPE_RTS)
				    || (fr_ctrl_field.type == TYPE_CTS)) {
					/*
					 * Don't pass up RTS, CTS, or ACK frames.  They just
					 * confuse the stack
					 */
					dev_kfree_skb(skb);
				} else {
					if (fr_ctrl_field.type == TYPE_BEACON) {
						piperp->beacon.weSentLastOne = false;
					}

				piper_ps_process_receive_frame(piperp, skb);
				memcpy(IEEE80211_SKB_RXCB(skb), &status, sizeof(status));
#if WANT_TO_RECEIVE_FRAMES_IN_ISR
					ieee80211_rx_irqsafe(piperp->hw, skb);
#else
					ieee80211_rx(piperp->hw, skb);
#endif
				}
			} else {
				/* Frame failed MIC, so discard it */
				dprintk(DWARNING, "dropping bad frame\n");
				dev_kfree_skb(skb);
			}
		}
	}

	piperp->ps.rxTaskletRunning = false;
	piperp->set_irq_mask_bit(piperp, BB_IRQ_MASK_RX_FIFO);
	piper_ps_rx_task_exiting(piperp);
}
EXPORT_SYMBOL_GPL(piper_rx_tasklet);




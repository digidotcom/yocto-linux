/*
 * digiIsr.c
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/*
 * This file contains the routines that are related to processing interrupts
 * from the MAC.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>

#include "pipermain.h"
#include "mac.h"
#include "phy.h"

#define IRQ_DEBUG	(0)
#if IRQ_DEBUG
static int dlevel = DWARNING;
#define dprintk(level, fmt, arg...)	if (level >= dlevel)			\
					printk(KERN_ERR PIPER_DRIVER_NAME	\
					    ": %s - " fmt, __func__, ##arg)
#else
#define dprintk(level, fmt, arg...)	do {} while (0)
#endif

/*
 * This routine handles interrupts from the MAC.
 */
irqreturn_t piper_irq_handler(int irq, void *dev_id)
{
	struct piper_priv *piperp = dev_id;
	u32 status;

	/* Acknowledge pending interrupts */
	status = piperp->ac->rd_reg(piperp, BB_IRQ_STAT);
	status &= piperp->ac->rd_reg(piperp, BB_IRQ_MASK);
	piperp->ac->wr_reg(piperp, BB_IRQ_STAT, status, op_write);

	if (status & BB_IRQ_MASK_RX_FIFO) {
		/*
		 * This interrupt indicates we have a frame in the FIFO.
		 * Set up to receive the packet.  Disable further interrupts
		 * until the receive is complete.
		 */
		piperp->clear_irq_mask_bit(piperp, BB_IRQ_MASK_RX_FIFO);
		/*
		 * Call the receive routine directly inside the irq handler
		 * or in the tasklet, depending on configuration.
		 */
#if WANT_TO_RECEIVE_FRAMES_IN_ISR
		piper_rx_tasklet((unsigned long) piperp);
#else
		tasklet_hi_schedule(&piperp->rx_tasklet);
#endif
	}

	if (status & BB_IRQ_MASK_TX_FIFO_EMPTY) {
		/*
		 * Transmit complete interrupt.  This IRQ is only unmasked if we are
		 * not expecting the packet to be ACKed.  This will be the case for
		 * broadcasts.  In this case, tell mac80211 the transmit occurred and
		 * restart the tx queue.
		 */
		if (piper_tx_getqueue(piperp) != NULL) {
			piperp->tx_signal_strength = 0;
			piperp->tx_result = TX_COMPLETE;
			tasklet_hi_schedule(&piperp->tx_tasklet);
		} else {
			dprintk(DWARNING, "BB_IRQ_MASK_TX_FIFO_EMPTY and null packet?\n");
		}
		piperp->clear_irq_mask_bit(piperp, BB_IRQ_MASK_TX_FIFO_EMPTY |
					BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
	}

	if (status & BB_IRQ_MASK_TIMEOUT) {
		/* AP did not ACK our TX packet */
		if (piper_tx_getqueue(piperp) != NULL) {
			/* Update retry counter */
			tasklet_hi_schedule(&piperp->tx_tasklet);
		} else {
			dprintk(DWARNING, "BB_IRQ_MASK_TIMEOUT and null packet?\n");
		}
		piperp->clear_irq_mask_bit(piperp, BB_IRQ_MASK_TX_FIFO_EMPTY |
					BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
	}

	if (unlikely(status & BB_IRQ_MASK_TX_ABORT)) {
		dprintk(DWARNING, "TX abort\n");

		/* Could not transmit a packet because the media was busy */
		if (piper_tx_getqueue(piperp) != NULL) {
			tasklet_hi_schedule(&piperp->tx_tasklet);
		} else {
			dprintk(DWARNING, "BB_IRQ_MASK_TX_ABORT and null packet?\n");
		}
		piperp->clear_irq_mask_bit(piperp, BB_IRQ_MASK_TX_FIFO_EMPTY |
					BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
	}

	if (status & BB_IRQ_MASK_TBTT) {
		/*
		 * This interrupt occurs at the start of a beacon period.  The only thing
		 * we need to do is to write a new beacon backoff value.
		 */
		u32 reg = piperp->ac->rd_reg(piperp, MAC_BEACON_FILT) & ~MAC_BEACON_BACKOFF_MASK;
		piperp->ac->wr_reg(piperp, MAC_BEACON_FILT,
				reg | piperp->get_next_beacon_backoff(), op_write);
		/*
		 * TODO:
		 * Improve the way we keep track of whether or not we sent the last
		 * beacon.  What we are doing now is to assume that we did until and
		 * unless we receive a beacon.  What we should do is look for either
		 * a beacon or a TX end interrupt.  However, since mac80211 doesn't
		 * tell us what the ATIM window is, we have to assume it is zero,
		 * which means we could be transmitting a frame at the same
		 * time we are sending the beacon, so there isn't really any easy
		 * way for us to do this.  In fact, even if there was an ATIM
		 * window, we could have started a transmit just before we get this
		 * interrupt, so I'm not sure how we are really suppose to keep
		 * track of this.
		 */
		/* assume we sent last beacon unless we receive one */
		piperp->beacon.weSentLastOne = true;
	}

	if (status & BB_IRQ_MASK_ATIM) {
		/*
		 * This interrupt should not occur since we are not using it.  When in
		 * IBSS mode, the beacon period starts at the TBTT interrupt and ends
		 * at this interrupt.  We are not suppose to send packets between the
		 * two interrupts.  However, mac80211 does not seem to provide a way
		 * for us to find out how long the ATIM period is, so we have to assume
		 * that there isn't one.
		 *
		 * If we were supporting this interrupt we would have to synchronize
		 * with the transmit routine so that transmit is paused during this
		 * time.
		 */
		dprintk(DWARNING, "BB_IRQ_MASK_ATIM irq (0x%08x)\n", status);
		piperp->clear_irq_mask_bit(piperp, BB_IRQ_MASK_ATIM);
	}

	if (unlikely(status & BB_IRQ_MASK_RX_OVERRUN))
		piperp->pstats.rx_overruns++;

	return IRQ_HANDLED;
}
EXPORT_SYMBOL_GPL(piper_irq_handler);




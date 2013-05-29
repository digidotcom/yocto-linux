/*
 * drivers/net/ns9xxx-eth.c
 *
 * Copyright (C) 2007,2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/* open issues
 *  - use phy irq
 *  - VLAN
 *  - clk_enable only at open time?
 *  - PM for hibernate
 */
#include <linux/clk.h>
#include <linux/crc32.h>
#include <linux/etherdevice.h>
#include <linux/mii.h>
#include <linux/ns9xxx-eth.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <asm/gpio.h>

#define DRIVER_NAME "ns9xxx-eth"

#define ETH_EGCR1	0x0000
#define ETH_EGCR1_ERX		(1 << 31)
#define ETH_EGCR1_ERXDMA	(1 << 30)
#define ETH_EGCR1_ERXSHT	(1 << 28)
#define ETH_EGCR1_ETX		(1 << 23)
#define ETH_EGCR1_ETXDMA	(1 << 22)
#define ETH_EGCR1_ERXINIT	(1 << 19)
#define ETH_EGCR1_PM		(3 << 14)
#define ETH_EGCR1_PM_MII		0
#define ETH_EGCR1_MACHRST	(1 <<  9)
#define ETH_EGCR1_ITXA		(1 <<  8)
#define ETH_EGCR1_MB1		(1 << 21)

#define ETH_EGCR2	0x0004
/* TCLER is different comparing the ns9360 vs ns921x */
/* FIXME TCLER should be used differently per processor */
#define ETH_EGCR2_TCLER_NS921X	(1 <<  7)
#define ETH_EGCR2_TCLER_NS9360	(1 <<  3)
#define ETH_EGCR2_TCLER		(1 <<  7)
#define ETH_EGCR2_TKICK		(1 <<  3)
#define ETH_EGCR2_AUTOZ		(1 <<  2)
#define ETH_EGCR2_CLRCNT	(1 <<  1)
#define ETH_EGCR2_STEN		(1 <<  0)

#define ETH_EGSR	0x0008
#define ETH_EGSR_RXINIT		(1 << 20)

#define ETH_TS		0x0018
#define ETH_TS_OK		( 1 << 15)
#define ETH_TS_BR		( 1 << 14)
#define ETH_TS_MC		( 1 << 13)
#define ETH_TS_AL		( 1 << 12)
#define ETH_TS_AED		( 1 << 11)
#define ETH_TS_AEC		( 1 << 10)
#define ETH_TS_AUR		( 1 <<  9)
#define ETH_TS_AJ		( 1 <<  8)
#define ETH_TS_DEF		( 1 <<  6)
#define ETH_TS_CRC		( 1 <<  5)
#define ETH_TS_COLC		(15 <<  0)

#define ETH_MAC1	0x0400
#define ETH_MAC1_SRST		(1 << 15)
#define ETH_MAC1_RXEN		(1 <<  0)

#define ETH_MAC2	0x0404
#define ETH_MAC2_PADEN		(1 <<  5)
#define ETH_MAC2_CRCEN		(1 <<  4)
#define ETH_MAC2_FULLD		(1 <<  0)

#define ETH_B2BIPG	0x0408

#define ETH_EPSR	0x0418
#define ETH_EPSR_SPEED_MASK	(1 << 8)
#define ETH_EPSR_SPEED_100	(1 << 8)
#define ETH_EPSR_SPEED_10	(0 << 8)


#define ETH_MIIMCFG	0x0420
#define ETH_MIIMCFG_RMIIM	(1 << 15)
#define ETH_MIIMCFG_CLKS	(7 <<  2)
#define ETH_MIIMCFG_CLKS_DIV40	(7 <<  2)

#define ETH_MIIMCMD	0x0424
#define ETH_MIIMCMD_READ	(1 <<  0)
#define ETH_MIIMADDR	0x0428
#define ETH_MIIMWD	0x042c
#define ETH_MIIMRD	0x0430
#define ETH_MIIMIR	0x0434
#define ETH_MIIMIR_LF		(1 <<  3)
#define ETH_MIIMIR_BUSY		(1 <<  0)

#define ETH_SA1		0x0440
#define ETH_SA2		0x0444
#define ETH_SA3		0x0448

#define ETH_SAF		0x0500
#define ETH_SAF_PRO		(1 <<  3)
#define ETH_SAF_PRM		(1 <<  2)
#define ETH_SAF_PRA		(1 <<  1)
#define ETH_SAF_BROAD		(1 <<  0)

#define ETH_HT1		0x0504
#define ETH_HT2		0x0508

#define ETH_STAT_TR64	0x0680
#define ETH_STAT_TR127	0x0684
#define ETH_STAT_TR255	0x0688
#define ETH_STAT_TR511	0x068C
#define ETH_STAT_TR1K	0x0690
#define ETH_STAT_TRMAX	0x0694
#define ETH_STAT_TRMGV	0x0698
#define ETH_STAT_RBYT	0x069C
#define ETH_STAT_RPKT	0x06A0
#define ETH_STAT_RFCS	0x06A4
#define ETH_STAT_RMCA	0x06A8
#define ETH_STAT_RBCA	0x06AC
#define ETH_STAT_RXCF	0x06B0
#define ETH_STAT_RXPF	0x06B4
#define ETH_STAT_RXUO	0x06B8
#define ETH_STAT_RALN	0x06BC
#define ETH_STAT_RFLR	0x06C0
#define ETH_STAT_RCDE	0x06C4
#define ETH_STAT_RCSE	0x06C8
#define ETH_STAT_RUND	0x06CC
#define ETH_STAT_ROVR	0x06D0
#define ETH_STAT_RFRG	0x06D4
#define ETH_STAT_RJBR	0x06D8
#define ETH_STAT_TBYT	0x06E0
#define ETH_STAT_TPKT	0x06E4
#define ETH_STAT_TMCA	0x06E8
#define ETH_STAT_TBCA	0x06EC
#define ETH_STAT_TDFR	0x06F4
#define ETH_STAT_TEDF	0x06F8
#define ETH_STAT_TSCL	0x06FC
#define ETH_STAT_TMCL	0x0700
#define ETH_STAT_TLCL	0x0704
#define ETH_STAT_TXCL	0x0708
#define ETH_STAT_TNCL	0x070C
#define ETH_STAT_TJBR	0x0718
#define ETH_STAT_TFCS	0x071C
#define ETH_STAT_TOVR	0x0724
#define ETH_STAT_TUND	0x0728
#define ETH_STAT_TFRG	0x072C

#define ETH_RXABDP	0x0a00
#define ETH_RXBBDP	0x0a04
#define ETH_RXCBDP	0x0a08
#define ETH_RXDBDP	0x0a0c

#define ETH_IS		0x0a10
#define ETH_IS_RXOVFLDATA	(1 << 25)
#define ETH_IS_RXOVFLSTAT	(1 << 24)
#define ETH_IS_RXDONEA		(1 << 22)
#define ETH_IS_RXDONEB		(1 << 21)
#define ETH_IS_RXDONEC		(1 << 20)
#define ETH_IS_RXDONED		(1 << 19)
#define ETH_IS_RXNOBUF		(1 << 18)
#define ETH_IS_RXBUFFUL		(1 << 17)
#define ETH_IS_RXBR		(1 << 16)
#define ETH_IS_TXDONE		(1 <<  2)
#define ETH_IS_TXERR		(1 <<  1)
#define ETH_IS_TXIDLE		(1 <<  0)

#define ETH_IE		0x0a14
#define ETH_IE_RXOVFLDATA	(1 << 25)
#define ETH_IE_RXOVFLSTAT	(1 << 24)
#define ETH_IE_RXDONEA		(1 << 22)
#define ETH_IE_RXDONEB		(1 << 21)
#define ETH_IE_RXDONEC		(1 << 20)
#define ETH_IE_RXDONED		(1 << 19)
#define ETH_IE_RXNOBUF		(1 << 18)
#define ETH_IE_RXBUFFUL		(1 << 17)
#define ETH_IE_RXBR		(1 << 16)
#define ETH_IE_TXDONE		(1 <<  2)
#define ETH_IE_TXERR		(1 <<  1)
#define ETH_IE_TXIDLE		(1 <<  0)

#define	ETH_RX_IRQS		(ETH_IE_RXOVFLDATA | \
				 ETH_IE_RXDONEA | \
				 ETH_IE_RXDONEB | \
				 ETH_IE_RXDONEC | \
				 ETH_IE_RXDONED | \
				 ETH_IE_RXNOBUF | \
				 ETH_IE_RXBUFFUL)

#define	ETH_TX_IRQS		(ETH_IE_TXDONE | ETH_IE_TXERR)

#define ETH_TXBDP	0x0a18

#define ETH_TRBDP	0x0a1c

#define ETH_RXFREE	0x0a3c
#define ETH_RXFREE_A		(1 <<  0)

#define ETH_TXBDR	0x1000

/* hardware limits sets from ethernet controller */
#define	HW_RX_RINGS		(4)
#define	MAX_ETH_FRAME_LEN	(1522)

/* software limits sets by driver */
#define TOTAL_NR_TXDESC		(64)
#define NR_RXDESC_PER_RING	(45)
#define TOTAL_NR_RXDESC		(HW_RX_RINGS*NR_RXDESC_PER_RING) /* total number of descriptors */

/* masks for DMA descriptors handling */
#define DMADESC_WRAP		(1 << 15)
#define DMADESC_INTR		(1 << 14)
#define DMATXDESC_LAST		(1 << 13)
#define DMARXDESC_EN		(1 << 13)
#define DMADESC_FULL		(1 << 12)

#ifdef CONFIG_GPIO_ETH_ACTIVITY_LED
 #define ACTIVITYLED_TOGGLE_TIMEOUT	2	/* in jiffies */
 #define ACTIVITYLED_OFF_TIMEOUT	20	/* in jiffies */
 unsigned int rxtx_activity = 0;
 struct timer_list activityled_timer;
#endif

union ns9xxx_dma_desc {
	struct {
		u32 source;
		u16 len;
		u16 reserved;
		u32 dest;
		u16 status;
		u16 flags;
	};
	u32 data[4];
};

struct ns9xxx_eth_priv {
	unsigned char __iomem *membase;
	resource_size_t mapbase;
	spinlock_t lock;
	struct net_device_stats stats;
	struct clk *clk;
	unsigned int irqtx;
	unsigned int irqrx;
#ifdef CONFIG_GPIO_ETH_ACTIVITY_LED
	unsigned int activityled;
#endif

	struct net_device *ndev;	/* Parent NET device */

	/* phy management */
	int lastlink;
	int lastduplex;
	int lastspeed;
	struct mii_bus *mdiobus;
	struct phy_device *phy;

	/* rx stuff */
	struct sk_buff **rxskb;
	union ns9xxx_dma_desc *rxdesc;
	dma_addr_t rxdeschandle;

	/* tx stuff */
	struct sk_buff **txskb;
	u16 txbusy;
	u16 txfree;

	/* work to recover from rx stall condition that can happen at 100 Mbps HD */
	struct delayed_work recover_from_rx_stall;
};

static inline u32 ethread32(struct net_device *dev, unsigned int offset)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	u32 ret = ioread32(priv->membase + offset);

	dev_vdbg(&dev->dev, "read  0x%p -> 0x%08x\n",
			priv->membase + offset, ret);

	return ret;
}

static inline void ethwrite32(struct net_device *dev,
		u32 value, unsigned int offset)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);

	dev_vdbg(&dev->dev, "write 0x%p <- 0x%08x\n",
			priv->membase + offset, value);
	iowrite32(value, priv->membase + offset);
}

static inline void ethupdate32(struct net_device *dev,
			       u32 and, u32 or, unsigned int offset)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	u32 reg;

	reg = ioread32(priv->membase + offset) & and;
	iowrite32(reg | or, priv->membase + offset);
	dev_vdbg(&dev->dev, "update 0x%p <- 0x%08x\n",
		 priv->membase + offset, reg | or);
}

#ifdef CONFIG_GPIO_ETH_ACTIVITY_LED
static void toggle_activityled(void)
{
	/* set activity flag */
	rxtx_activity = 1;
	/* run timer if not already running */
	if(!timer_pending(&activityled_timer))
		mod_timer(&activityled_timer, jiffies + ACTIVITYLED_TOGGLE_TIMEOUT);
}

static void activityled_timer_fn(unsigned long gpio)
{
	static int cnt = 0;

	if(rxtx_activity) {
		/* toggle RX/TX Ethernet activity LED */
		gpio_set_value(gpio, !gpio_get_value(gpio));
		mod_timer(&activityled_timer, jiffies + ACTIVITYLED_TOGGLE_TIMEOUT);
		cnt = 0;
		rxtx_activity = 0;
	} else {
		if(cnt++ < ACTIVITYLED_OFF_TIMEOUT / ACTIVITYLED_TOGGLE_TIMEOUT)
			mod_timer(&activityled_timer, jiffies + ACTIVITYLED_TOGGLE_TIMEOUT);
		else {
			gpio_set_value(gpio, 1); /* switch LED off  */
			cnt = 0;
		}
	}

}
#endif

static int ns9xxx_eth_miibus_pollbusy(struct mii_bus *bus)
{
	unsigned int timeout = 0x3000;
	struct net_device *dev = bus->priv;

	while (timeout--) {
		u32 miimir;

		miimir = ethread32(dev, ETH_MIIMIR);

		if (!(miimir & ETH_MIIMIR_BUSY))
			break;

		cpu_relax();
	}

	return timeout ? 0 : -EBUSY;
}

static int ns9xxx_eth_mdiobus_read(struct mii_bus *bus, int phyid, int regnum)
{
	struct net_device *dev = bus->priv;
	int ret;

	if ((phyid & ~0x1f) || (regnum & ~0x1f))
		return -EINVAL;

	ret = ns9xxx_eth_miibus_pollbusy(bus);
	if (ret)
		goto out;

	ethwrite32(dev, phyid << 8 | regnum, ETH_MIIMADDR);
	ethwrite32(dev, 0, ETH_MIIMCMD);
	ethwrite32(dev, ETH_MIIMCMD_READ, ETH_MIIMCMD);

	ret = ns9xxx_eth_miibus_pollbusy(bus);
	if (ret)
		goto out;

	ret = ethread32(dev, ETH_MIIMRD) & 0xffff;

out:
	dev_vdbg(bus->parent, "%s,  phyid = %d, regnum = %d -> %04x\n",
			__func__, phyid, regnum, ret);

	return ret;
}

static int ns9xxx_eth_mdiobus_write(struct mii_bus *bus,
		int phyid, int regnum, u16 val)
{
	struct net_device *dev = bus->priv;
	int ret;

	if ((phyid & ~0x1f) || (regnum & ~0x1f))
		return -EINVAL;

	ret = ns9xxx_eth_miibus_pollbusy(bus);
	if (ret)
		goto out;

	ethwrite32(dev, phyid << 8 | regnum, ETH_MIIMADDR);
	ethwrite32(dev, val, ETH_MIIMWD);

	ret = ns9xxx_eth_miibus_pollbusy(bus);

out:
	dev_vdbg(bus->parent, "%s: phyid = %d, regnum = %d, val = %04hx -> %04x\n",
			__func__, phyid, regnum, val, ret);

	return ret;
}

static int ns9xxx_eth_mdiobus_reset(struct mii_bus *bus)
{
	struct net_device *dev = bus->priv;

	dev_dbg(bus->parent, "%s\n", __func__);

	ethwrite32(dev, ETH_MIIMCFG_RMIIM, ETH_MIIMCFG);

	/* TODO: currently the biggest divider (40) is used.  This could be
	 * tuned depending on the PHY.  phylib doesn't provide the needed
	 * information, though :-( */
	ethwrite32(dev, ETH_MIIMCFG_CLKS_DIV40, ETH_MIIMCFG);

	return 0;
}

static inline int ns9xxx_eth_create_skbuff(struct net_device* dev, const int descr)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	struct sk_buff *skb;
	int ret;

	skb = dev_alloc_skb(MAX_ETH_FRAME_LEN);
	if (likely(skb)) {
		priv->rxskb[descr] = skb;
		skb->dev = dev;

		priv->rxdesc[descr].source = dma_map_single(&dev->dev, skb->data,
				skb->len, DMA_FROM_DEVICE);

		priv->rxdesc[descr].len = MAX_ETH_FRAME_LEN;
		priv->rxdesc[descr].flags = DMADESC_INTR | DMARXDESC_EN |
			(descr == (NR_RXDESC_PER_RING - 1) ? DMADESC_WRAP : 0);
		ret = 0;
	} else {
		printk(KERN_ERR "%s: out of memory\n", __func__);
		priv->rxdesc[descr].flags = 0;
		ret = -ENOMEM;
	}

	return ret;
}

static inline void ns9xxx_eth_rx_process_ring(struct net_device* dev, unsigned int ring)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	union ns9xxx_dma_desc *desc;
	unsigned int i, endring;

	endring = (ring + 1) * NR_RXDESC_PER_RING;
	desc = &priv->rxdesc[ring * NR_RXDESC_PER_RING];

	for (i = ring * NR_RXDESC_PER_RING; i < endring; i++, desc++) {
		if (desc->flags & DMADESC_FULL) {
			struct sk_buff *skb;
			skb = dev_alloc_skb(MAX_ETH_FRAME_LEN);
			if (likely(skb)) {
				skb_reserve(skb, 2);	/* 16 byte IP header align */
				memcpy(skb_put(skb, desc->len), (unsigned char *)priv->rxskb[i]->data,
				desc->len);
				skb->protocol = eth_type_trans(skb, dev);
				priv->stats.rx_packets++;
				priv->stats.rx_bytes += desc->len;
				dev->last_rx = jiffies;
				netif_rx(skb);
			} else {
				printk(KERN_ERR "%s: out of memory, dropping packet\n", __func__);
				dev->stats.rx_dropped++;
			}
			desc->len = MAX_ETH_FRAME_LEN;
			desc->flags = DMADESC_INTR | DMARXDESC_EN | (i == (NR_RXDESC_PER_RING - 1) ? DMADESC_WRAP : 0);
		}
	}
}

static irqreturn_t ns9xxx_eth_rx_int(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	unsigned long flags;
	u32 is, rxdonemask, ring;

	spin_lock_irqsave(&priv->lock, flags);

	is = ethread32(dev, ETH_IS);
	/* Acknowledge interrupts */
	ethwrite32(dev, is & ETH_RX_IRQS, ETH_IS);
	dev_vdbg(&dev->dev, "%s: ETH_IS=%08x\n", __func__, is);

	if (is & ETH_IS_RXOVFLDATA) {
		if (!(is & (ETH_IS_RXDONEA | ETH_IS_RXDONEB |
		    ETH_IS_RXDONEC | ETH_IS_RXDONEA))) {
			/* The ETH_IS_RXOVFLDATA bit is set, then the receiver front
			 * end has apparently locked up. We schedule a work that resets
			 * the interface. We check the DONE bits to try to empty the
			 * receive rings of packets before the receiver reset. Note
			 * that once we get into this lockup state, the ETH_IS_RXOVFLDATA
			 * interrupt will happen continously until we reset the receiver.
			 */
			ethupdate32(dev, ~(ETH_EGCR1_ERX | ETH_EGCR1_ERXDMA), 0, ETH_EGCR1);
			ethupdate32(dev, ~ETH_MAC1_RXEN, 0, ETH_MAC1);
			ethupdate32(dev, ~ETH_IE_RXOVFLDATA, 0, ETH_IE);

			schedule_delayed_work(&priv->recover_from_rx_stall, 0);
		}
	}

	if (is & (ETH_IS_RXNOBUF | ETH_IS_RXBUFFUL)) {
		priv->stats.rx_dropped++;
	}

	for (ring = 0, rxdonemask = ETH_IS_RXDONEA; ring < HW_RX_RINGS; ring++) {
		if (is & rxdonemask) {
			ns9xxx_eth_rx_process_ring(dev, ring);
			ethwrite32(dev, 1 << ring, ETH_RXFREE);
		}
		rxdonemask >>= 1;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

#ifdef CONFIG_GPIO_ETH_ACTIVITY_LED
	toggle_activityled();
#endif
	return IRQ_HANDLED;
}

static void ns9xxx_eth_recover_from_rx_stall(struct work_struct *work)
{
	struct ns9xxx_eth_priv *priv =
	    container_of(work, struct ns9xxx_eth_priv,
			 recover_from_rx_stall.work);

	struct net_device *dev = priv->ndev;
	unsigned long flags;
	int i, timeout = 20;

	spin_lock_irqsave(&priv->lock, flags);

	for (i = 0; i < HW_RX_RINGS; i++) {
		ethwrite32(dev, priv->rxdeschandle + (i * NR_RXDESC_PER_RING),
			   ETH_RXABDP + (i * 4));
	}

	ethupdate32(dev, 0xffffffff, ETH_EGSR_RXINIT, ETH_EGSR);
	ethupdate32(dev, 0xffffffff, ETH_EGCR1_ERX, ETH_EGCR1);
	ethupdate32(dev, 0xffffffff, ETH_EGCR1_ERXINIT, ETH_EGCR1);

	while (!(ethread32(dev, ETH_EGSR) & ETH_EGSR_RXINIT) && timeout--)
		udelay(1);

	ethupdate32(dev, ~ETH_EGCR1_ERXINIT, 0, ETH_EGCR1);

	/* Re-enable the overflow interrupt */
	ethupdate32(dev, 0xffffffff, ETH_IE_RXOVFLDATA, ETH_IE);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static inline int ns9xxx_eth_num_txbusy(struct net_device *dev)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);

	return (TOTAL_NR_TXDESC + priv->txfree - priv->txbusy) %
		TOTAL_NR_TXDESC;
}

static inline int ns9xxx_eth_num_txfree(struct net_device *dev)
{
	return TOTAL_NR_TXDESC - ns9xxx_eth_num_txbusy(dev);
}

static void ns9xxx_eth_read_txdesc(struct net_device *dev,
		union ns9xxx_dma_desc *txbuffer)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	int i;

	for (i = 0; i < 4; ++i)
		txbuffer->data[i] = ethread32(dev,
				ETH_TXBDR + 16 * priv->txbusy + 4 * i);
}

static void ns9xxx_eth_start_tx_dma(struct net_device *dev)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	u32 egcr2, ie;
	u32 start;

	dev_vdbg(&dev->dev, "%s\n", __func__);

	dev->trans_start = jiffies;

	/* XXX: really kick TCLER? */

	egcr2 = ethread32(dev, ETH_EGCR2);
	if (egcr2 & (ETH_EGCR2_TCLER | ETH_EGCR2_TKICK)) {
		egcr2 &= ~(ETH_EGCR2_TCLER | ETH_EGCR2_TKICK);
		ethwrite32(dev, egcr2, ETH_EGCR2);
	}

	start = 4 * priv->txbusy;

	ie = ethread32(dev, ETH_IE);
	if ((ie & ETH_TX_IRQS) == 0) {
		u32 egcr1 = ethread32(dev, ETH_EGCR1);
		ethwrite32(dev, start, ETH_TXBDP);

		ethwrite32(dev, ie | ETH_TX_IRQS, ETH_IE);
		ethwrite32(dev, egcr1 | ETH_EGCR1_ETXDMA, ETH_EGCR1);
	} else
		ethwrite32(dev, start, ETH_TRBDP);

	egcr2 |= ETH_EGCR2_TCLER | ETH_EGCR2_TKICK;
	ethwrite32(dev, egcr2, ETH_EGCR2);
}

static irqreturn_t ns9xxx_eth_tx_int(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	u32 is;
	struct sk_buff *skb;
	union ns9xxx_dma_desc txbuffer;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	is = ethread32(dev, ETH_IS);
	dev_vdbg(&dev->dev, "%s: ETH_IS=%08x\n", __func__, is);

	/* ack */
	ethwrite32(dev, is & (ETH_IS_TXDONE | ETH_IS_TXERR), ETH_IS);

	while (1) {
		if (!ns9xxx_eth_num_txbusy(dev))
			break;

		skb = priv->txskb[priv->txbusy];

		ns9xxx_eth_read_txdesc(dev, &txbuffer);

		dma_unmap_single(&dev->dev, txbuffer.source,
				skb->len, DMA_TO_DEVICE);

		priv->txbusy = (priv->txbusy + 1) % TOTAL_NR_TXDESC;

		if (txbuffer.status & ETH_TS_OK) {
			priv->stats.tx_packets++;
			priv->stats.tx_bytes += skb->len;
		} else {
			priv->stats.tx_errors++;

			/* XXX: fill in tx_aborted_errors etc. */
			/* kick TCLER? */
		}

		dev_kfree_skb_irq(skb);

		if (txbuffer.flags & DMADESC_FULL)
			break;
	}

	if (ns9xxx_eth_num_txbusy(dev) &&
			(is & ETH_IS_TXIDLE))
		ns9xxx_eth_start_tx_dma(dev);

	spin_unlock_irqrestore(&priv->lock, flags);

#ifdef CONFIG_GPIO_ETH_ACTIVITY_LED
	toggle_activityled();
#endif

	return IRQ_HANDLED;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void ns9xxx_eth_netpoll(struct net_device *dev)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	unsigned long flags;

	local_irq_save(flags);
	ns9xxx_eth_rx_int(priv->irqrx, dev);
	ns9xxx_eth_tx_int(priv->irqtx, dev);
	local_irq_restore(flags);
}
#endif

static void ns9xxx_eth_adjust_link(struct net_device *dev)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	struct phy_device *phydev = priv->phy;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	if ((phydev->link != priv->lastlink) ||
	    (priv->lastspeed != phydev->speed) ||
	    (priv->lastduplex != phydev->duplex)){

		if (phydev->link) {
			u32 mac2 = ethread32(dev, ETH_MAC2);
			u32 epsr = ethread32(dev, ETH_EPSR);

			/* Adjsut speed */
			epsr &= ~ETH_EPSR_SPEED_MASK;
			epsr |= (phydev->speed == SPEED_100) ?
				ETH_EPSR_SPEED_100 : 0;
			ethwrite32(dev, epsr, ETH_EPSR);
			priv->lastspeed = phydev->speed;

			/* Adjsut duplex */
			mac2 &= ~ETH_MAC2_FULLD;
			mac2 |= phydev->duplex ? ETH_MAC2_FULLD : 0;
			ethwrite32(dev, mac2, ETH_MAC2);
			ethwrite32(dev, phydev->duplex ? 0x15 : 0x12,
				   ETH_B2BIPG);
			priv->lastduplex = phydev->duplex;

			dev_info(&dev->dev, "link up (%d/%s)\n", phydev->speed,
				 (phydev->duplex == DUPLEX_FULL) ?
				 "full" : "half");
		} else {
			/* link down */
			priv->lastspeed = 0;
			priv->lastduplex = -1;
			dev_info(&dev->dev, "link down\n");
		}
		priv->lastlink = phydev->link;
	}

	spin_unlock_irqrestore(&priv->lock, flags);
}

static int ns9xxx_eth_hwinit(struct net_device *dev)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	u32 egcr1, ie;
	int timeout, i;

	dev_dbg(&dev->dev, "%s\n", __func__);

	/* disable everything */
	ethwrite32(dev, ETH_EGCR1_PM_MII, ETH_EGCR1);
	ethwrite32(dev, 0, ETH_IE);

	/* ack any pending irq */
	ethwrite32(dev, ethread32(dev, ETH_IS), ETH_IS);

	/* program station address */
	ethwrite32(dev, dev->dev_addr[0] | dev->dev_addr[1] << 8, ETH_SA3);
	ethwrite32(dev, dev->dev_addr[2] | dev->dev_addr[3] << 8, ETH_SA2);
	ethwrite32(dev, dev->dev_addr[4] | dev->dev_addr[5] << 8, ETH_SA1);

	for (i = 0; i <  HW_RX_RINGS; i++) {
		ethwrite32(dev, priv->rxdeschandle + (i * NR_RXDESC_PER_RING),
			   ETH_RXABDP + (i * 4));
	}

	ethwrite32(dev, ETH_SAF_BROAD, ETH_SAF);

	egcr1 = ETH_EGCR1_ERX | ETH_EGCR1_ERXDMA | ETH_EGCR1_ETX |
		ETH_EGCR1_PM_MII | ETH_EGCR1_ITXA | ETH_EGCR1_MB1;
	ethwrite32(dev, egcr1 | ETH_EGCR1_ERXINIT, ETH_EGCR1);

	timeout = 6;
	while (!(ethread32(dev, ETH_EGSR) & ETH_EGSR_RXINIT) && timeout--)
		udelay(1);

	if (!timeout)
		return -EBUSY;

	ethwrite32(dev, ETH_EGSR_RXINIT, ETH_EGSR);
	ethwrite32(dev, egcr1, ETH_EGCR1);

	ethwrite32(dev, ETH_MAC1_RXEN, ETH_MAC1);
	ethwrite32(dev, ETH_MAC2_CRCEN | ETH_MAC2_PADEN, ETH_MAC2);
	ethwrite32(dev, 0x12, ETH_B2BIPG);

	/* clear and enable statistics */
	ethupdate32(dev, 0xffffffff, ETH_EGCR2_CLRCNT, ETH_EGCR2);
	ethupdate32(dev, ~ETH_EGCR2_CLRCNT, 0, ETH_EGCR2);
	ethupdate32(dev, 0xffffffff, ETH_EGCR2_AUTOZ | ETH_EGCR2_STEN, ETH_EGCR2);

	ie = ethread32(dev, ETH_IE);
	ethwrite32(dev, ie | ETH_RX_IRQS, ETH_IE);

	ethwrite32(dev, 0xf, ETH_RXFREE);
	return 0;
}

static void ns9xxx_eth_hwdisable(struct net_device *dev)
{
	dev_dbg(&dev->dev, "%s\n", __func__);

	ethwrite32(dev, 0, ETH_EGCR1);
	ethwrite32(dev, 0, ETH_IE);
	ethwrite32(dev, 0, ETH_MAC1);
	ethwrite32(dev, 0, ETH_MAC2);
}

static int ns9xxx_eth_open(struct net_device *dev)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	struct resource *res;
	int ret = -ENOMEM;
	int i;

	dev_dbg(&dev->dev, "%s\n", __func__);

	res = request_mem_region(priv->mapbase, 0x2800, DRIVER_NAME);
	if (!res) {
		dev_dbg(&dev->dev, "%s: err_request_mem\n", __func__);
		goto err_request_mem;
	}

	phy_start(priv->phy);

	priv->txfree = 1;
	priv->txbusy = 1;

	priv->rxdesc = dma_alloc_coherent(&dev->dev,
			sizeof(*priv->rxdesc) * TOTAL_NR_RXDESC,
			&priv->rxdeschandle, GFP_KERNEL);
	if (!priv->rxdesc) {
		dev_dbg(&dev->dev, "%s: err_alloc_rxdesc\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_rxdesc;
	}

	priv->rxskb = kmalloc(sizeof(*priv->rxskb) * TOTAL_NR_RXDESC,
			GFP_KERNEL);
	if (!priv->rxskb) {
		dev_dbg(&dev->dev, "%s: err_alloc_rxskb\n", __func__);
		goto err_alloc_rxskb;
	}

	for (i = 0; i < TOTAL_NR_RXDESC; ++i) {
		ret = ns9xxx_eth_create_skbuff(dev, i);

		if (ret) {
			dev_dbg(&dev->dev, "%s: err_setup_rxskb (i = %d)\n",
					__func__, i);
			goto err_setup_rxskb;
		}
	}

	priv->txskb = kmalloc(sizeof(*priv->txskb) * TOTAL_NR_TXDESC,
			GFP_KERNEL);
	if (!priv->txskb) {
		dev_dbg(&dev->dev, "%s: err_alloc_txskb\n", __func__);
		goto err_alloc_txskb;
	}

	ret = ns9xxx_eth_hwinit(dev);
	if (ret) {
		dev_dbg(&dev->dev, "%s: err_hwinit -> %d\n", __func__, ret);
		goto err_hwinit;
	}

	ret = request_irq(priv->irqtx, ns9xxx_eth_tx_int, 0, DRIVER_NAME, dev);
	if (ret) {
		dev_dbg(&dev->dev, "%s: err_request_irq_tx -> %d\n",
				__func__, ret);
		goto err_request_irq_tx;
	}

	ret = request_irq(priv->irqrx, ns9xxx_eth_rx_int, 0, DRIVER_NAME, dev);
	if (ret) {
		dev_dbg(&dev->dev, "%s: err_request_irq_rx -> %d\n",
				__func__, ret);

		free_irq(priv->irqtx, dev);
err_request_irq_tx:

		ns9xxx_eth_hwdisable(dev);
err_hwinit:

err_setup_rxskb:
		for (i = 0; priv->rxskb[i]; ++i) {
			dma_unmap_single(&dev->dev, priv->rxdesc[i].source,
					priv->rxskb[i]->len, DMA_FROM_DEVICE);
			kfree_skb(priv->rxskb[i]);
		}

		kfree(priv->txskb);
err_alloc_txskb:

		kfree(priv->rxskb);
err_alloc_rxskb:

		dma_free_coherent(&dev->dev,
				sizeof(*priv->rxdesc) * TOTAL_NR_RXDESC,
				priv->rxdesc, priv->rxdeschandle);
err_alloc_rxdesc:

		release_mem_region(priv->mapbase, 0x2800);
err_request_mem:

		return ret;
	}

	netif_start_queue(dev);

	return 0;
}

static int ns9xxx_eth_stop(struct net_device *dev)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);

	dev_dbg(&dev->dev, "%s\n", __func__);

	free_irq(priv->irqrx, dev);
	free_irq(priv->irqtx, dev);

	cancel_delayed_work(&priv->recover_from_rx_stall);

	ns9xxx_eth_hwdisable(dev);
	kfree(priv->txskb);
	kfree(priv->rxskb);
	dma_free_coherent(&dev->dev,
			sizeof(*priv->rxdesc) * TOTAL_NR_RXDESC,
			priv->rxdesc, priv->rxdeschandle);

	release_mem_region(priv->mapbase, 0x2800);

	return 0;
}

static int ns9xxx_eth_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	unsigned long flags;
	int ret;

	if (!netif_running(dev))
		return -EINVAL;

	if (!priv->phy)
		return -EINVAL;

	spin_lock_irqsave(&priv->lock, flags);
	ret = phy_mii_ioctl(priv->phy, if_mii(ifr), cmd);
	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static void ns9xxx_eth_add_txdesc(struct net_device *dev,
		union ns9xxx_dma_desc *txbuffer)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	int i;

	dev_vdbg(&dev->dev, "%s: txfree=%hu, txbusy=%hu\n",
			__func__, priv->txfree, priv->txbusy);

	for (i = 0; i < 4; ++i)
		ethwrite32(dev, txbuffer->data[i],
				ETH_TXBDR + 16 * priv->txfree + 4 * i);

	priv->txfree = (priv->txfree + 1) % TOTAL_NR_TXDESC;
}

static int ns9xxx_eth_hard_start_xmit(struct sk_buff *skb,
		struct net_device *dev)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	union ns9xxx_dma_desc txbuffer = {};
	unsigned long flags;
	int ret;

	/* do I need to lock already here? */
	spin_lock_irqsave(&priv->lock, flags);

	dev_vdbg(&dev->dev, "%s(skb=%p): skb->data=%p\n",
			__func__, skb, skb->data);

	if (likely(ns9xxx_eth_num_txfree(dev) > 0)) {
		txbuffer.source = dma_map_single(&dev->dev, skb->data,
				skb->len, DMA_TO_DEVICE);
		txbuffer.len = skb->len;
		txbuffer.flags = DMATXDESC_LAST | DMADESC_FULL;

		if (unlikely(priv->txfree == TOTAL_NR_TXDESC - 1))
			txbuffer.flags |= DMADESC_WRAP;

		priv->txskb[priv->txfree] = skb;
		ns9xxx_eth_add_txdesc(dev, &txbuffer);

		if (ns9xxx_eth_num_txbusy(dev) == 1)
			ns9xxx_eth_start_tx_dma(dev);

		ret = NETDEV_TX_OK;
	} else {
		netif_stop_queue(dev);
		ret = NETDEV_TX_BUSY;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static void ns9xxx_eth_tx_timeout(struct net_device *dev)
{
	/* XXX */
	dev_warn(&dev->dev, "%s unimplemented\n", __func__);
}

static struct net_device_stats *ns9xxx_eth_get_stats(struct net_device *dev)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);

	priv->stats.rx_length_errors += ethread32(dev, ETH_STAT_RFLR);
	priv->stats.rx_over_errors += ethread32(dev, ETH_STAT_ROVR);
	priv->stats.rx_crc_errors += ethread32(dev, ETH_STAT_RFCS);
	priv->stats.rx_frame_errors += ethread32(dev, ETH_STAT_RALN);
	priv->stats.rx_errors += ethread32(dev, ETH_STAT_RCDE) +
				 ethread32(dev, ETH_STAT_RCSE) +
				 ethread32(dev, ETH_STAT_RUND) +
				 ethread32(dev, ETH_STAT_RFRG) +
				 ethread32(dev, ETH_STAT_RJBR);
	priv->stats.multicast += ethread32(dev, ETH_STAT_RMCA);
	priv->stats.tx_aborted_errors += ethread32(dev, ETH_STAT_TXCL);
	priv->stats.collisions += ethread32(dev, ETH_STAT_TNCL);

	return &priv->stats;
}

static void ns9xxx_set_multicast_list(struct net_device *dev)
{
	u32 saf = ETH_SAF_BROAD;

	dev_dbg(&dev->dev, "%s\n", __func__);

	/* TODO: use the hash tables to improve multicast reception */
	if (dev->flags & IFF_PROMISC)
		/* get everything */
		saf |= ETH_SAF_PRO;

	else if (dev->flags & IFF_ALLMULTI)
		/* get all multicast traffic */
		saf |= ETH_SAF_PRM;

	else if (!netdev_mc_empty(dev)) {
		struct netdev_hw_addr *ha;
		u32 ht[2] = {0, 0};

		netdev_for_each_mc_addr(ha, dev) {
			/*
			 * The HRM of ns9360 and ns9215 state that the upper 6
			 * bits are used to calculate the bit in the hash table,
			 * but the sample code (and the NET+OS driver) uses bits
			 * 28:23 ...
			 */
			u32 crc = ether_crc(ETH_ALEN, ha->addr) >> 23;
			crc &= 0x3f;

			ht[crc & 0x20 ? 1 : 0] |= 1 << (crc & 0x1f);
		}

		saf |= ETH_SAF_PRA;

		ethwrite32(dev, ht[0], ETH_HT1);
		ethwrite32(dev, ht[1], ETH_HT2);
	}

	ethwrite32(dev, saf, ETH_SAF);
}

static int ns9xxx_eth_mdiobus_init(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	struct plat_ns9xxx_eth *pdata = pdev->dev.platform_data;
	int i;
	char phyid[MII_BUS_ID_SIZE + 3];
	int ret = -ENOMEM;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	priv->mdiobus = mdiobus_alloc();
	if (priv->mdiobus == NULL)
		goto err_out;

	priv->mdiobus->name = DRIVER_NAME "-mii";
	snprintf(priv->mdiobus->id, MII_BUS_ID_SIZE, "0");
	priv->mdiobus->priv = dev;
	priv->mdiobus->read = ns9xxx_eth_mdiobus_read;
	priv->mdiobus->write = ns9xxx_eth_mdiobus_write;
	priv->mdiobus->reset = ns9xxx_eth_mdiobus_reset;
	priv->mdiobus->phy_mask = pdata->phy_mask;
	priv->mdiobus->parent = &pdev->dev;
	priv->mdiobus->irq = kmalloc(sizeof(*priv->mdiobus->irq) * PHY_MAX_ADDR,
			GFP_KERNEL);

	if (!priv->mdiobus->irq) {
		dev_dbg(&pdev->dev, "%s: err_alloc_irq\n", __func__);
		goto err_alloc_irq;
	}

	for (i = 0; i < PHY_MAX_ADDR; ++i)
		priv->mdiobus->irq[i] = PHY_POLL;

	ret = mdiobus_register(priv->mdiobus);
	if (ret) {
		dev_dbg(&pdev->dev, "%s: err_mdiobus_register -> %d\n",
				__func__, ret);
		goto err_mdiobus_register;
	}

	for (i = 0; i < PHY_MAX_ADDR; ++i)
		if (priv->mdiobus->phy_map[i])
			break;

	if (i >= PHY_MAX_ADDR) {
		dev_dbg(&pdev->dev, "%s: no phy found\n", __func__);
		ret = -ENODEV;
		goto err_find_phy;
	}

	dev_dbg(&pdev->dev, "%s: using phy at address %d\n", __func__, i);

	snprintf(phyid, sizeof(phyid), PHY_ID_FMT,
			priv->mdiobus->id, i);

	priv->phy = phy_connect(dev, phyid, &ns9xxx_eth_adjust_link,
			0, PHY_INTERFACE_MODE_MII);

	if (IS_ERR(priv->phy)) {
		ret = PTR_ERR(priv->phy);
		dev_dbg(&pdev->dev, "%s: err_phy_connect -> %d\n",
				__func__, ret);
err_find_phy:

		mdiobus_unregister(priv->mdiobus);
err_mdiobus_register:

		kfree(priv->mdiobus->irq);
err_alloc_irq:
		mdiobus_free(priv->mdiobus);
err_out:
		return ret;
	}

	priv->phy->supported &= SUPPORTED_10baseT_Half |
		SUPPORTED_10baseT_Full |
		SUPPORTED_100baseT_Half |
		SUPPORTED_100baseT_Full |
		SUPPORTED_Autoneg |
		SUPPORTED_MII;

	priv->phy->advertising = priv->phy->supported;

	return 0;
}

static int ns9xxx_eth_mdiobus_disable(struct net_device *dev)
{
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);

	dev_dbg(&dev->dev, "%s\n", __func__);

	phy_disconnect(priv->phy);
	mdiobus_unregister(priv->mdiobus);
	kfree(priv->mdiobus->irq);
	mdiobus_free(priv->mdiobus);

	return 0;
}

static const struct net_device_ops ns9xxx_netdev_ops = {
	.ndo_open               = ns9xxx_eth_open,
	.ndo_stop               = ns9xxx_eth_stop,
	.ndo_start_xmit         = ns9xxx_eth_hard_start_xmit,
	.ndo_tx_timeout         = ns9xxx_eth_tx_timeout,
	.ndo_set_multicast_list = ns9xxx_set_multicast_list,
	.ndo_do_ioctl		= ns9xxx_eth_ioctl,
	.ndo_get_stats		= ns9xxx_eth_get_stats,
	.ndo_change_mtu         = eth_change_mtu,
	.ndo_validate_addr      = eth_validate_addr,
	.ndo_set_mac_address    = eth_mac_addr,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller    = ns9xxx_eth_netpoll,
#endif
};

static __devinit int ns9xxx_eth_pdrv_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct ns9xxx_eth_priv *priv;

	struct resource *mem;
	struct plat_ns9xxx_eth *pdata = pdev->dev.platform_data;
	unsigned char __iomem *membase;
	u32 sa;
	int ret = -ENODEV;
	int i;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (!pdata)
		goto err_pdata;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_dbg(&pdev->dev, "%s: err_get_mem\n", __func__);
		goto err_get_mem;
	}

	ret = -ENOMEM;

	membase = ioremap(mem->start, 0x2800);
	if (!membase) {
		dev_dbg(&pdev->dev, "%s: err_ioremap\n", __func__);
		goto err_ioremap;
	}

	dev = alloc_etherdev(sizeof(*priv));
	if (!dev) {
		dev_dbg(&pdev->dev, "%s: err_alloc_etherdev\n", __func__);
		goto err_alloc_etherdev;
	}

	dev->dev.coherent_dma_mask = (u32)-1;

	SET_NETDEV_DEV(dev, &pdev->dev);
	platform_set_drvdata(pdev, dev);

	dev->netdev_ops = &ns9xxx_netdev_ops;

	/* TODO: implement VLAN */
	dev->features = 0;

	priv = netdev_priv(dev);
	priv->ndev = dev;

	spin_lock_init(&priv->lock);
	INIT_DELAYED_WORK(&priv->recover_from_rx_stall, ns9xxx_eth_recover_from_rx_stall);

	priv->membase = membase;
	priv->mapbase = mem->start;
	priv->irqrx = pdata->irqrx;
	priv->irqtx = pdata->irqtx;
#ifdef CONFIG_GPIO_ETH_ACTIVITY_LED
	priv->activityled = pdata->activityled;
	/* init kernel timer for toggling
	 * the activity LED */
	activityled_timer.data = (unsigned long)priv->activityled;
	activityled_timer.function = activityled_timer_fn;
	activityled_timer.expires = jiffies + ACTIVITYLED_TOGGLE_TIMEOUT;
	init_timer(&activityled_timer);
	add_timer(&activityled_timer);
#endif

	priv->clk = clk_get(&pdev->dev, DRIVER_NAME);
	if (IS_ERR(priv->clk)) {
		ret = PTR_ERR(priv->clk);
		dev_dbg(&pdev->dev, "%s: err_clk_get -> %d\n", __func__, ret);
		goto err_clk_get;
	}

	ret = clk_enable(priv->clk);
	if (ret) {
		dev_dbg(&pdev->dev, "%s: err_clk_enable -> %d\n",
				__func__, ret);
		goto err_clk_enable;
	}

	sa = ethread32(dev, ETH_SA1);
	if (sa == 0) {
		dev_warn(&pdev->dev, "Warning: Using default mac address "
				"00:04:f3:ff:ff:fa\n");
		memcpy(dev->dev_addr, "\x00\x04\xf3\xff\xff\xfa", ETH_ALEN);
	} else {
		/* assume the bootloader has provided a valid mac address */
		dev->dev_addr[4] = sa;
		dev->dev_addr[5] = sa >> 8;
		sa = ethread32(dev, ETH_SA2);
		dev->dev_addr[2] = sa;
		dev->dev_addr[3] = sa >> 8;
		sa = ethread32(dev, ETH_SA3);
		dev->dev_addr[0] = sa;
		dev->dev_addr[1] = sa >> 8;
		dev_dbg(&pdev->dev, "mac address: %pM\n", dev->dev_addr);
	}

	ethwrite32(dev, ETH_EGCR1_PM_MII | ETH_EGCR1_MACHRST, ETH_EGCR1);
	udelay(5);
	ethwrite32(dev, ETH_EGCR1_PM_MII | ETH_EGCR1_ETX, ETH_EGCR1);

	ethwrite32(dev, 0, ETH_MAC1);

	dev_dbg(&pdev->dev, "%s: clear tx DMA descs\n", __func__);
	for (i = 0; i < 4 * TOTAL_NR_TXDESC; ++i)
		ethwrite32(dev, 0, ETH_TXBDR + 4 * i);

	ret = ns9xxx_eth_mdiobus_init(pdev);
	if (ret) {
		dev_dbg(&pdev->dev, "%s: err_mdiobus_init -> %d\n",
				__func__, ret);
		goto err_mdiobus_init;
	}

	/* Make the device wakeup capable, but disabled by default */
	device_init_wakeup(&pdev->dev, 1);
	device_set_wakeup_enable(&pdev->dev, 0);

	ret = register_netdev(dev);
	if (ret) {
		dev_dbg(&pdev->dev, "%s: err_register_netdev -> %d\n",
				__func__, ret);

		ns9xxx_eth_mdiobus_disable(dev);
err_mdiobus_init:

		clk_disable(priv->clk);
err_clk_enable:

		clk_put(priv->clk);
err_clk_get:

		platform_set_drvdata(pdev, NULL);
		free_netdev(dev);
err_alloc_etherdev:

		iounmap(membase);
err_ioremap:
err_get_mem:
err_pdata:
		return ret;
	}

	/* When using nfsroot, there are RPC requests which can be sent
	 * before the link is up (long autonegotiation...). Then the
	 * RPC is not completed until the autonegotiation timeouts (which
	 * adds long time to the boot process)
	 * Following line tries to workaround that situation by starting
	 * the phy here, so we have some worked completed in advance to
	 * the driver open call */
	phy_start(priv->phy);

	dev_info(&pdev->dev, "%s at MMIO %p\n", dev->name, membase);
	dev_dbg(&pdev->dev, "&priv = %p\n", &priv);
	dev_dbg(&pdev->dev, "&priv->txfree = %p\n", &priv->txfree);
	dev_dbg(&pdev->dev, "&priv->txbusy = %p\n", &priv->txbusy);

	return 0;
}

static __devexit int ns9xxx_eth_pdrv_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	unregister_netdev(dev);

	ns9xxx_eth_mdiobus_disable(dev);

	clk_disable(priv->clk);

	clk_put(priv->clk);

	platform_set_drvdata(pdev, NULL);

	iounmap(priv->membase);

	free_netdev(dev);

#ifdef CONFIG_GPIO_ETH_ACTIVITY_LED
	del_timer(&activityled_timer);
#endif

	return 0;
}

#if defined(CONFIG_PM)
static int ns9xxx_eth_pdrv_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	int ret;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (state.event != PM_EVENT_SUSPEND)
		/* XXX: implement full state saving */
		return -EBUSY;

	netif_device_detach(dev);

	if (device_may_wakeup(&pdev->dev)) {

		ret = enable_irq_wake(priv->irqrx);
		if (ret) {
			dev_dbg(&pdev->dev, "%s: err_enable_irq_wake -> %d\n",
					__func__, ret);
			goto err_enable_irq_wake;
		}

	} else {
		int mii_bmcr;

		dev_dbg(&pdev->dev, "%s: !device_may_wakeup\n", __func__);
		/* Put Ethernet PHY in power down mode */
		mii_bmcr = ns9xxx_eth_mdiobus_read(priv->mdiobus,
						   priv->phy->addr,
						   MII_BMCR);
		mii_bmcr &= ~BMCR_RESET;
		mii_bmcr |= BMCR_PDOWN;
		ns9xxx_eth_mdiobus_write(priv->mdiobus, priv->phy->addr,
					 MII_BMCR, mii_bmcr);
		clk_disable(priv->clk);
	}

	return 0;

err_enable_irq_wake:
	netif_device_attach(dev);

	return ret;
}

static int ns9xxx_eth_pdrv_resume(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct ns9xxx_eth_priv *priv = netdev_priv(dev);
	int ret;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(priv->irqrx);
	else {
		int mii_bmcr;
		ret = clk_enable(priv->clk);

		/* Wake up Ethernet PHY */
		mii_bmcr = ns9xxx_eth_mdiobus_read(priv->mdiobus,
						   priv->phy->addr,
						   MII_BMCR);
		mii_bmcr &= ~(BMCR_RESET | BMCR_PDOWN);
		ns9xxx_eth_mdiobus_write(priv->mdiobus, priv->phy->addr,
					MII_BMCR, mii_bmcr);
		if (ret) {
			dev_dbg(&pdev->dev, "%s: err_clk_enable -> %d",
					__func__, ret);
			return ret;
		}
	}

	netif_device_attach(dev);

	return 0;
}
#endif

static struct platform_driver ns9xxx_eth_pdriver = {
	.probe = ns9xxx_eth_pdrv_probe,
	.remove = __devexit_p(ns9xxx_eth_pdrv_remove),
#if defined(CONFIG_PM)
	.suspend = ns9xxx_eth_pdrv_suspend,
	.resume = ns9xxx_eth_pdrv_resume,
#endif
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ns9xxx_eth_init(void)
{
	int ret;

	/* catch compiler bugs, endian issues etc pp */
	BUG_ON(offsetof(union ns9xxx_dma_desc, data) != 0x0);
	BUG_ON(offsetof(union ns9xxx_dma_desc, source) != 0x0);
	BUG_ON(offsetof(union ns9xxx_dma_desc, len) != 0x4);
	BUG_ON(offsetof(union ns9xxx_dma_desc, dest) != 0x8);
	BUG_ON(offsetof(union ns9xxx_dma_desc, flags) != 0xe);
	BUG_ON(offsetof(union ns9xxx_dma_desc, status) != 0xc);

	ret = platform_driver_register(&ns9xxx_eth_pdriver);
	if (ret) {
		pr_debug("%s: err_pdrv_register\n", __func__);

		return ret;
	}
	pr_info("Digi NS9XXX Ethernet driver\n");

	return 0;
}

static void __exit ns9xxx_eth_exit(void)
{
	platform_driver_unregister(&ns9xxx_eth_pdriver);
}

module_init(ns9xxx_eth_init);
module_exit(ns9xxx_eth_exit);

MODULE_AUTHOR("Uwe Kleine-Koenig");
MODULE_DESCRIPTION("Digi NS9XXX ethernet driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);

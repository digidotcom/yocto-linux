/*
 * linux/drivers/spi/spi_ns921x.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 * (C) Copyright 2009, Emerald Electronics Design, LLC,
 *		Mark Litwack <mlitwack@employees.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>

#include <mach/gpio.h>
#include <mach/spi.h>

/* registers */
#define	DMA_IRQSTATUS			(0x0000)
#define	DMA_RXCTL			(0x0004)
#define	DMA_RXBUF			(0x0008)
#define	DMA_RXIRQCFG			(0x000c)
#define	DMA_TXCTL			(0x0018)
#define	DMA_TXBUF			(0x001c)
#define	DMA_TXIRQCFG			(0x0020)

#define SPI_CONFIG			(0x1000)
#define SPI_CLOCK			(0x1010)
#define SPI_IRQENABLE			(0x1020)
#define SPI_IRQSTATUS			(0x1024)

/* DMA config bit fields */
#define DMA_ENABLECHANNEL		(1 << 31)
/* DMA interrupt/FIFO status flags */
#define DMA_IRQSTAT_RXNCIP	(1 << 31)
#define DMA_IRQSTAT_RXECIP	(1 << 30)
#define DMA_IRQSTAT_RXNRIP	(1 << 29)
#define DMA_IRQSTAT_RXCAIP	(1 << 28)
#define DMA_IRQSTAT_RXPCIP	(1 << 27)
#define DMA_IRQSTAT_TXNCIP	(1 << 24)
#define DMA_IRQSTAT_TXECIP	(1 << 23)
#define DMA_IRQSTAT_TXNRIP	(1 << 22)
#define DMA_IRQSTAT_TXCAIP	(1 << 21)
/* DMA interrupt configuration flags */
#define DMA_IRQCFG_NCIE		(1 << 24)
#define DMA_IRQCFG_ECIE		(1 << 23)
#define DMA_IRQCFG_NRIE		(1 << 22)
#define DMA_IRQCFG_CAIE		(1 << 21)
#define DMA_IRQCFG_PCIE		(1 << 20)
#define DMA_IRQCFG_BLENSTAT	(0xFFFF)
/* SPI config bit fields */
#define SPI_CONFIG_MASTER		(1 << 0)
#define SPI_CONFIG_SLAVE		(1 << 1)
#define SPI_CONFIG_BITORDRMSB		(1 << 2)
#define SPI_CONFIG_RXBYTE		(1 << 3)
#define SPI_CONFIG_MODE0		(0x00)
#define SPI_CONFIG_MODE1		(0x10)
#define SPI_CONFIG_MODE2		(0x20)
#define SPI_CONFIG_MODE3		(0x30)
#define SPI_CLOCK_ENABLE		(1 << 16)
#define SPI_IRQENABLE_RXIDLE		(1 << 0)
#define SPI_IRQENABLE_TXIDLE		(1 << 1)
#define SPI_IRQSTATUS_RXNRIP		(1 << 29)
#define SPI_IRQSTATUS_TXNRIP		(1 << 22)

/* config masks */
#define SPI_CONFIG_MODE_MASK		(0x30)
#define SPI_CONFIG_DISCARD_MASK		(0x700)
#define SPI_CLOCK_DIVISOR_MASK		(0x3FF)

/* config shifts */
#define SPI_CONFIG_MODE_SHIFT		(4)
#define SPI_CONFIG_DISCARD_SHIFT	(8)
#define SPI_CLOCK_DIVISOR_SHIFT		(0)

#define SPI_IDLE_LEVEL			0xff
#define MAX_RATE			33330000
#define	MAX_BUFFER_SIZE			65532

#define	SPI_CS_ACTIVE		1
#define	SPI_CS_INACTIVE		0

#define DRIVER_NAME			"spi_ns921x"

union ns9xxx_dma_desc {
	struct {
		u32 source;
		u16 len;
		u16 reserved1;
		u32 reserved2;

		u16 status;
#define DMADESC_WRAP	(1 << 15)
#define DMADESC_INTR	(1 << 14)
#define DMATXDESC_LAST	(1 << 13)
#define DMADESC_FULL	(1 << 12)
		u16 flags;
	};
	u32 data[4];
};

struct spi_ns921x {
	void __iomem		*ioaddr;	/* SPI DMA base address */
	struct resource		*mem;
	struct clk		*clk;

//	struct workqueue_struct	*workqueue;
	struct work_struct	work;
	spinlock_t		lock;
	struct list_head	queue;
	struct spi_transfer	*current_transfer;
	unsigned long		remaining_bytes;
	unsigned		dma_xfer_len;
	u32			flags;
	wait_queue_head_t	waitq;

	int			irq;
	unsigned int		speed;

/*	union ns9xxx_dma_desc	*rxdesc;
	union ns9xxx_dma_desc	*txdesc;
	dma_addr_t		rxdesc_real;
	dma_addr_t		txdesc_real;
*/
	void			*buf_tx;
	void			*buf_rx;
	dma_addr_t		buf_tx_dma;
	dma_addr_t		buf_rx_dma;
	union ns9xxx_dma_desc	*desc_tx;
	union ns9xxx_dma_desc	*desc_rx;
	dma_addr_t		desc_tx_dma;
	dma_addr_t		desc_rx_dma;

	struct spi_ns9xxx_data	*pdata;
	//struct tasklet_struct   xmit_tasklet;
};

#define BUFFER_SIZE		PAGE_SIZE
#define INVALID_DMA_ADDRESS	0xffffffff
#define SPI_RX_IDONE		0x00000001
#define SPI_TX_IDONE		0x00000002

static void spi_ns921x_chipsel(struct spi_device *spi, int active)
{
unsigned int cspol = spi->mode & SPI_CS_HIGH ? 1 : 0;
void (* func)(struct spi_device *spi, int active) = spi->controller_data;
struct spi_ns921x *info = spi_master_get_devdata(spi->master);

	/* use special function to toggle CS? */
	if( func != NULL ) {
		/* Reconfigure CS line as GPIO input */
		if( SPI_CS_ACTIVE == active )
			gpio_configure_ns921x(info->pdata->gpios[SPI_EN_GPIO_OFFSET],
						NS921X_GPIO_INPUT,
						NS921X_GPIO_DONT_INVERT,
						NS921X_GPIO_FUNC_3,
						NS921X_GPIO_ENABLE_PULLUP );

		/* Call the special function */
		func( spi, cspol == active);

		/* Reconfigure CS line as SPI_EN */
		if( SPI_CS_INACTIVE == active )
			gpio_configure_ns921x(info->pdata->gpios[SPI_EN_GPIO_OFFSET],
						0,
						0,
						info->pdata->gpio_funcs[SPI_EN_GPIO_OFFSET],
						0);
	}
	else {
		/* Use a GPIO to toggle CS? */
		if( info->pdata->gpio_funcs[SPI_EN_GPIO_OFFSET] == NS921X_GPIO_FUNC_GPIO )
			gpio_set_value( info->pdata->gpios[SPI_EN_GPIO_OFFSET], cspol == active );
		/* otherwise CS will be automatically controlled by the spi engine */
	}
}

/*
 * Configure the controller for the passed SPI-device. Additionally it will
 * use the configuration values of the SPI-transfer if it's different than NULL
 */
static int spi_ns921x_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct spi_ns921x *info = spi_master_get_devdata(spi->master);
	unsigned int div,reg;
	unsigned int bpw;
	unsigned int hz;

	pr_debug("Setup for next xfer called\n");

	/* If the bits per word is zero, then use the default value of the SPI-device */
	bpw = t ? t->bits_per_word : spi->bits_per_word;
	if (!bpw)
		bpw = spi->bits_per_word;

	/* We only support eight bits per word */
	if (bpw != 8) {
		pr_err("Invalid bits-per-word (%d)\n", bpw);
		return -EINVAL;
	}

	/*
	 * If the clock rate of the xfer is zero, then use the already configured
	 * rate of this SPI-device
	 */
	hz  = t ? t->speed_hz : spi->max_speed_hz;
	if (!hz)
		hz = spi->max_speed_hz;

	if( hz > MAX_RATE ) {
		pr_err("Invalid speed (%d). Max is %d\n", hz, spi->max_speed_hz );
		return -EINVAL;
	}

	/* calculate divisor */
	div = (clk_get_rate(info->clk) / hz + 1);
	if(div > SPI_CLOCK_DIVISOR_MASK) {
		pr_err("Invalid speed (%d). Min is %ld\n",
			hz,
			clk_get_rate(info->clk)/(SPI_CLOCK_DIVISOR_MASK-1));
		return -EINVAL;
	}

	/* configure clock divisor (3 steps, see hardware reference) */
	/* step1: unset SPI_CLOCK_ENABLE, don't touch other bits */
	reg = ioread32(info->ioaddr + SPI_CLOCK);
	iowrite32(reg & ~SPI_CLOCK_ENABLE, info->ioaddr + SPI_CLOCK);

	reg = div & (SPI_CLOCK_DIVISOR_MASK << SPI_CLOCK_DIVISOR_SHIFT);
	iowrite32(reg, info->ioaddr + SPI_CLOCK);

	/* step3: set SPI_CLOCK_ENABLE, don't touch other bits */
	iowrite32(reg | SPI_CLOCK_ENABLE, info->ioaddr + SPI_CLOCK);

	return 0;
}

static int spi_ns921x_setup(struct spi_device *spi)
{
	struct spi_ns921x *info = spi_master_get_devdata(spi->master);
	unsigned int reg;
	int ret;

	if (spi->chip_select > spi->master->num_chipselect) {
		dev_dbg(&spi->dev,
				"setup: invalid chipselect %u (%u defined)\n",
				spi->chip_select, spi->master->num_chipselect);
		return -EINVAL;
	}

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	/* configure SPI controller */
	reg = SPI_CONFIG_MASTER;

	if (!(spi->mode & SPI_LSB_FIRST))
		reg |= SPI_CONFIG_BITORDRMSB;

	if (spi->mode & SPI_CPOL) {
		if (spi->mode & SPI_CPHA)
			reg |= SPI_CONFIG_MODE3;
		else
			reg |= SPI_CONFIG_MODE2;
	} else if (spi->mode & SPI_CPHA)
		reg |= SPI_CONFIG_MODE1;
	else
		reg |= SPI_CONFIG_MODE0;

	iowrite32(reg, info->ioaddr + SPI_CONFIG);

	ret = spi_ns921x_setupxfer(spi, NULL);
	if (!ret)
		pr_debug("Mode %d, %u bpw, %d HZ\n",
			     spi->mode, spi->bits_per_word,
			     spi->max_speed_hz);

	return ret;
}

static void spi_ns921x_map_xfer(struct spi_device *spi,
		struct spi_transfer *xfer)
{
	xfer->tx_dma = xfer->rx_dma = INVALID_DMA_ADDRESS;
	/* TODO: Check this DMA mapping
	if (!(xfer->len & (L1_CACHE_BYTES - 1))) {
		if (xfer->tx_buf && !((unsigned long)xfer->tx_buf &
					(L1_CACHE_BYTES - 1)))
			xfer->tx_dma = dma_map_single(&spi->dev, (void *)xfer->tx_buf,
						xfer->len, DMA_TO_DEVICE);
		if (xfer->rx_buf && !((unsigned long)xfer->rx_buf &
					(L1_CACHE_BYTES - 1)))
			xfer->rx_dma = dma_map_single(&spi->dev, xfer->rx_buf,
						xfer->len, DMA_FROM_DEVICE);
	}
	*/
}

static void spi_ns921x_next_xfer(struct spi_master *master,
		struct spi_message *msg)
{
	struct spi_ns921x *info = spi_master_get_devdata(master);
	struct spi_transfer *xfer;
	dma_addr_t tx_dma, rx_dma;
	u32 len;
	int err;

	xfer = info->current_transfer;
	if (!xfer || info->remaining_bytes == 0) {
		if (xfer)
			xfer = list_entry(xfer->transfer_list.next,
					struct spi_transfer, transfer_list);
		else
			xfer = list_entry(msg->transfers.next,
					struct spi_transfer, transfer_list);
		info->remaining_bytes = xfer->len;
		info->current_transfer = xfer;
	}

	len = info->remaining_bytes;

	/* Set the requested configuration for this new SPI-transfer */
	err = spi_ns921x_setupxfer(msg->spi, xfer);
	if (err) {
		pr_err("Setting up the next SPI-transfer\n");
		list_del(&msg->queue);
		msg->status = err;

		spin_unlock(&info->lock);
		msg->complete(msg->context);
		spin_lock(&info->lock);

		info->current_transfer = NULL;
		return;
	}

	tx_dma = xfer->tx_dma;
	rx_dma = xfer->rx_dma;

	/* Some drivers (like mmc_spi) set rx_dma=0 if they
	 * don't need any return data.  If they do this, we
	 * put the unwanted rx data in rx_dma and hope they
	 * didn't ask to send more than BUFFER_SIZE (4096)
	 * bytes.  This is at least better than trashing
	 * page 0.
	 *
	 * - mwl
	 */
	if (!rx_dma || rx_dma == INVALID_DMA_ADDRESS) {
		rx_dma = info->buf_rx_dma;
		if (len > BUFFER_SIZE)
			len = BUFFER_SIZE;
	}
	if (tx_dma == INVALID_DMA_ADDRESS) {
		if (xfer->tx_buf) {
			tx_dma = info->buf_tx_dma;
			if (len > BUFFER_SIZE)
				len = BUFFER_SIZE;
			memcpy(info->buf_tx, xfer->tx_buf +
					xfer->len - info->remaining_bytes, len);
			dma_sync_single_for_device(&master->dev,
					info->buf_tx_dma, len, DMA_TO_DEVICE);
		} else {
			/* Send undefined data; rx_dma is handy */
			tx_dma = rx_dma;
		}
	}

	/* The IO hub DMA controller doesn't cope well with
	 * non-aligned rx buffers (and possibly tx too).  On
	 * rx, at least, it causes the last byte read by the
	 * peripheral on the *previous* transfer to be
	 * delivered again if the current transfer is
	 * unaligned.
	 *
	 * For now, just warn about it so someone can fix
	 * the offending caller when it occurs.  It might
	 * not be a problem on tx.
	 *
	 * - mwl
	 */

	/* Remove the warnings if using mmc_spi */
#if !(defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE))
	if (rx_dma & 0x3) {
		dev_err(&master->dev, "unaligned rx_dma (=0x%08x, len=%u)\n",
			rx_dma, len);
	}
	if (tx_dma & 0x3) {
		dev_err(&master->dev, "unaligned tx_dma (=0x%08x, len=%u)\n",
			tx_dma, len);
	}
#endif
	/* Setup the DMA channels */
	/* reset dma controller */
	iowrite32(0, info->ioaddr + DMA_RXCTL);
	iowrite32(0, info->ioaddr + DMA_TXCTL);
	/* set DMA descriptor */
	memset(info->desc_tx, 0, sizeof(*info->desc_tx));
	info->desc_tx->source = (u32)(tx_dma);
	info->desc_tx->len = len;
	info->desc_tx->flags =
		DMADESC_WRAP | DMADESC_INTR | DMATXDESC_LAST | DMADESC_FULL;

	memset(info->desc_rx, 0, sizeof(*info->desc_rx));
	info->desc_rx->source = (u32)(rx_dma);
	info->desc_rx->len = len;
	info->desc_rx->flags = DMADESC_WRAP;

	info->dma_xfer_len = len;

	/* configure dma channels */
	iowrite32( DMA_IRQCFG_NCIE | DMA_IRQCFG_ECIE |
		DMA_IRQCFG_NRIE | DMA_IRQCFG_CAIE | DMA_IRQCFG_PCIE,
		info->ioaddr + DMA_RXIRQCFG );
	iowrite32((u32)info->desc_tx_dma, info->ioaddr + DMA_TXBUF);
	iowrite32( DMA_IRQCFG_NCIE | DMA_IRQCFG_ECIE |
		DMA_IRQCFG_NRIE | DMA_IRQCFG_CAIE,
		info->ioaddr + DMA_TXIRQCFG );
	iowrite32((u32)info->desc_rx_dma, info->ioaddr + DMA_RXBUF);

	/* fire up dma channels */
	iowrite32(DMA_ENABLECHANNEL, info->ioaddr + DMA_RXCTL);
	iowrite32(DMA_ENABLECHANNEL, info->ioaddr + DMA_TXCTL);
}

static void spi_ns921x_next_message(struct spi_master *master)
{
	struct spi_ns921x *info = spi_master_get_devdata(master);
	struct spi_message *msg;

	BUG_ON(info->current_transfer);

	msg = list_entry(info->queue.next, struct spi_message, queue);

	spi_ns921x_chipsel(msg->spi, SPI_CS_ACTIVE);
	spi_ns921x_next_xfer(master, msg);
}

static int spi_ns921x_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct spi_ns921x *info = spi_master_get_devdata(spi->master);
	struct spi_transfer *xfer;
	unsigned long flags;
	if (unlikely(list_empty(&msg->transfers)
			|| !spi->max_speed_hz))
		return -EINVAL;

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (!(xfer->tx_buf || xfer->rx_buf)) {
			dev_dbg(&spi->dev, "missing rx or tx buf\n");
			return -EINVAL;
		}
	}

	/* scrub dcache "early" */
	if (!msg->is_dma_mapped) {
		list_for_each_entry(xfer, &msg->transfers, transfer_list)
			spi_ns921x_map_xfer(spi, xfer);
	}

	msg->status = -EINPROGRESS;
	msg->actual_length = 0;

	spin_lock_irqsave(&info->lock, flags);
	list_add_tail(&msg->queue, &info->queue);
	if (!info->current_transfer)
		spi_ns921x_next_message(spi->master);
	spin_unlock_irqrestore(&info->lock, flags);

	return 0;
}

static void spi_ns921x_irq_trigger_next(struct spi_master *master)
{
	struct spi_ns921x *info = spi_master_get_devdata(master);
	struct spi_message *msg;
	struct spi_transfer *xfer;
	u32 len, ctrl;

	BUG_ON((info->flags & (SPI_TX_IDONE|SPI_RX_IDONE)) != (SPI_TX_IDONE|SPI_RX_IDONE));

	info->flags &= ~(SPI_TX_IDONE|SPI_RX_IDONE);
	xfer = info->current_transfer;
	msg = list_entry(info->queue.next, struct spi_message, queue);

	len = info->dma_xfer_len;
	if (len > BUFFER_SIZE)
		len = BUFFER_SIZE;
	/*
	 * If the rx buffer wasn't aligned, we used a bounce
	 * buffer for the transfer. Copy the data back and
	 * make the bounce buffer ready for re-use.
	 */
	if (xfer->rx_buf && xfer->rx_dma == INVALID_DMA_ADDRESS) {
		dma_sync_single_for_cpu(&master->dev, info->buf_rx_dma,
					len, DMA_FROM_DEVICE);
		memcpy(xfer->rx_buf, info->buf_rx + xfer->len -
				info->remaining_bytes, len);
	}

	info->remaining_bytes -= len;

	if (info->remaining_bytes == 0) {
		msg->actual_length += xfer->len;

		if (!msg->is_dma_mapped) {
			if (xfer->tx_dma != INVALID_DMA_ADDRESS)
				dma_unmap_single(&master->dev,
						 xfer->tx_dma,
						 xfer->len,
						 DMA_TO_DEVICE);
			if (xfer->rx_dma != INVALID_DMA_ADDRESS)
				dma_unmap_single(&master->dev,
						 xfer->rx_dma,
						 xfer->len,
						 DMA_FROM_DEVICE);
		}

		/* REVISIT: udelay in irq is unfriendly */
		if (xfer->delay_usecs)
			udelay(xfer->delay_usecs);

		if (msg->transfers.prev == &xfer->transfer_list) {

			/* Message complete */
			spi_ns921x_chipsel(msg->spi, SPI_CS_INACTIVE);

			list_del(&msg->queue);
			msg->status = 0;

			spin_unlock(&info->lock);
			msg->complete(msg->context);
			spin_lock(&info->lock);

			info->current_transfer = NULL;

			/* continue; complete() may have queued requests */
			if (list_empty(&info->queue)) {
				ctrl = ioread32(info->ioaddr +
						DMA_RXCTL);
				ctrl &= ~DMA_ENABLECHANNEL;
				iowrite32(ctrl, info->ioaddr +
						DMA_RXCTL);
				ctrl = ioread32(info->ioaddr +
						DMA_TXCTL);
				ctrl &= ~DMA_ENABLECHANNEL;
				iowrite32(ctrl, info->ioaddr +
						DMA_TXCTL);
			} else
				spi_ns921x_next_message(master);
		} else {
			if (xfer->cs_change) {
				spi_ns921x_chipsel(msg->spi, SPI_CS_INACTIVE);
				udelay(1);
				spi_ns921x_chipsel(msg->spi, SPI_CS_ACTIVE);
			}

			spi_ns921x_next_xfer(master, msg);
		}
	} else {
		/* There are still pending data in the current transfer */
		spi_ns921x_next_xfer(master, msg);
	}
}

static irqreturn_t spi_ns921x_irq(int irq, void *dev_id)
{
	struct spi_master *master = dev_id;
	struct spi_ns921x *info = spi_master_get_devdata(master);
	u32 status, dlen, rx_status;

	spin_lock(&info->lock);

	/* get status and mask all pending interrupts */
	status = ioread32(info->ioaddr + DMA_IRQSTATUS);
	iowrite32(status, info->ioaddr + DMA_IRQSTATUS);

	/* TX interrupt error conditions */
	if (status & (DMA_IRQSTAT_TXECIP | DMA_IRQSTAT_TXCAIP )) {
		dev_err(&master->dev, "tx dma failure! status = 0x%08x\n",
				status);
		/* retrigger the xfer */
		spi_ns921x_next_xfer(master, list_entry(info->queue.next,
					struct spi_message, queue));
	}
	/* RX interrupt error conditions */
	rx_status = ioread32(info->ioaddr + DMA_RXIRQCFG);
	dlen = rx_status & DMA_IRQCFG_BLENSTAT;

	if (info->dma_xfer_len != dlen)
		/* ?? what to do... ?? */
		dev_err(&master->dev, "incomplete dma xfer"
				"(%d/%d bytes transfered) \n",
				dlen, info->dma_xfer_len);

	if (status & (DMA_IRQSTAT_RXECIP | DMA_IRQSTAT_RXCAIP |
				DMA_IRQSTAT_RXPCIP)) {
		dev_err(&master->dev, "rx dma failure! status = 0x%08x\n",
				status);

		/* retrigger the xfer */
		spi_ns921x_next_xfer(master, list_entry(info->queue.next,
					struct spi_message, queue));
	}
	/* TX normal completion */
	if (status & DMA_IRQSTAT_TXNCIP) {
		info->flags |= SPI_TX_IDONE;
		if (info->flags & SPI_RX_IDONE)
			spi_ns921x_irq_trigger_next(master);
	}
	/* RX normal completion */
	if (status & DMA_IRQSTAT_RXNCIP) {
		info->flags |= SPI_RX_IDONE;
		if (info->flags & SPI_TX_IDONE)
			spi_ns921x_irq_trigger_next(master);
	}
	spin_unlock(&info->lock);

	return IRQ_HANDLED;
}

static __devinit int spi_ns921x_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct spi_ns921x *info;
	int ret;
	int gpio_cs, gpio_cs_func;

	master = spi_alloc_master(&pdev->dev, sizeof(*info));
	if (!master) {
		pr_debug(DRIVER_NAME ": cannot allocate spi master\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, master);

	info = spi_master_get_devdata(master);
	info->pdata = pdev->dev.platform_data;
	info->irq = platform_get_irq(pdev, 0);
	if (info->irq < 0) {
		pr_debug(DRIVER_NAME ": no interrupt available\n");
		ret = -ENOENT;
		goto err_res;
	}

	info->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!info->mem) {
		pr_debug(DRIVER_NAME ": memory not available\n");
		ret = -ENOENT;
		goto err_res;
	}

	if (!request_mem_region(info->mem->start,
			info->mem->end - info->mem->start + 1,
			DRIVER_NAME)) {
		pr_debug(DRIVER_NAME ": memory already mapped\n");
		ret = -EIO;
		goto err_res;
	}

	info->ioaddr = ioremap(info->mem->start,
			info->mem->end - info->mem->start + 1);
	if (!info->ioaddr) {
		pr_debug(DRIVER_NAME ":  unable to remap IO memory\n");
		ret = -EIO;
		goto err_remap;
	}

	master->bus_num = pdev->id;
	/* hardware controlled cs */
	master->num_chipselect = 1;
	/* the spi->mode bits understood by this driver: */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST;

	/* If CS is GPIO controlled, configure it as output */
	gpio_cs = info->pdata->gpios[SPI_EN_GPIO_OFFSET];
	gpio_cs_func = info->pdata->gpio_funcs[SPI_EN_GPIO_OFFSET];
	if( NS921X_GPIO_FUNC_GPIO == gpio_cs_func )
		gpio_direction_output( gpio_cs,
				gpio_get_value( gpio_cs ) );

	master->setup = spi_ns921x_setup;
	master->transfer = spi_ns921x_transfer;

	/* prepare waitq and spinlock */
	spin_lock_init(&info->lock);
	INIT_LIST_HEAD(&info->queue);
	init_waitqueue_head(&info->waitq);

	/* register and map memory for dma-descriptors */
	info->buf_tx = dma_alloc_coherent(&pdev->dev, BUFFER_SIZE,
					&info->buf_tx_dma, GFP_KERNEL);
	if (!info->buf_tx) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "%s: err_alloc_buf_tx\n", __func__);
		goto err_alloc_buf_tx;
	}

	info->buf_rx = dma_alloc_coherent(&pdev->dev, BUFFER_SIZE,
					&info->buf_rx_dma, GFP_KERNEL);
	if (!info->buf_rx) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "%s: err_alloc_buf_rx\n", __func__);
		goto err_alloc_buf_rx;
	}

	info->desc_tx = dma_alloc_coherent(&pdev->dev,
			sizeof(info->desc_tx), &info->desc_tx_dma, GFP_DMA);
	if (!info->desc_tx) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "%s: err_alloc_desc_tx\n", __func__);
		goto err_alloc_desc_tx;
	}

	info->desc_rx = dma_alloc_coherent(&pdev->dev,
			sizeof(info->desc_rx), &info->desc_rx_dma, GFP_DMA);
	if (!info->desc_rx) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "%s: err_alloc_desc_rx\n", __func__);
		goto err_alloc_desc_rx;
	}

	info->clk = clk_get(&pdev->dev, DRIVER_NAME);
	if (IS_ERR(info->clk)) {
		pr_debug(DRIVER_NAME ": no clock available\n");
		ret = PTR_ERR(info->clk);
		goto err_clk;
	}

	ret = clk_enable(info->clk);
	if (ret) {
		pr_debug(DRIVER_NAME ": cannot enable clock\n");
		goto err_clk;
	}

	/* reset dma controller */
	iowrite32(0, info->ioaddr + DMA_RXCTL);
	iowrite32(0, info->ioaddr + DMA_TXCTL);

	/* register interrupt service routine */
	ret = request_irq(info->irq, spi_ns921x_irq, 0,
			pdev->name, master);
	if (ret) {
		pr_debug(DRIVER_NAME ": cannot register SPI interrupt\n");
		goto err_irq;
	}

	dev_info(&pdev->dev, "NS921x SPI controller at 0x%p (irq: %d)\n",
			info->ioaddr, info->irq );

	ret = spi_register_master(master);
	if (ret) {
		pr_debug(DRIVER_NAME ": cannot register spi master\n");
		goto err_reg;
	}

	return 0;

err_reg:
	//destroy_workqueue(info->workqueue);
//err_workqueue:
	free_irq(info->irq, info);
err_irq:
	clk_disable(info->clk);
err_clk:
	dma_free_coherent(&pdev->dev, sizeof(info->desc_rx),
			info->desc_rx, info->desc_rx_dma);
err_alloc_desc_rx:
	dma_free_coherent(&pdev->dev, sizeof(info->desc_tx),
			info->desc_tx, info->desc_tx_dma);
err_alloc_desc_tx:
	dma_free_coherent(&pdev->dev, BUFFER_SIZE,
			info->buf_rx, info->buf_rx_dma);
err_alloc_buf_rx:
	dma_free_coherent(&pdev->dev, BUFFER_SIZE,
			info->buf_tx, info->buf_tx_dma);
err_alloc_buf_tx:
	iounmap(info->ioaddr);

err_remap:
	release_mem_region(info->mem->start,
			info->mem->end - info->mem->start + 1);
err_res:
	spi_master_put(master);

	return ret;
}

static __devexit int spi_ns921x_remove(struct platform_device *pdev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct spi_ns921x *info = spi_master_get_devdata(master);

	spi_unregister_master(master);
//	destroy_workqueue(info->workqueue);
	free_irq(info->irq, info);
	clk_disable(info->clk);
	dma_free_coherent(&pdev->dev, sizeof(info->desc_rx),
			info->desc_rx, info->desc_rx_dma);
	dma_free_coherent(&pdev->dev, sizeof(info->desc_tx),
			info->desc_tx, info->desc_tx_dma);
	dma_free_coherent(&pdev->dev, BUFFER_SIZE,
			info->buf_rx, info->buf_rx_dma);
	dma_free_coherent(&pdev->dev, BUFFER_SIZE,
			info->buf_tx, info->buf_tx_dma);
	iounmap(info->ioaddr);
	release_mem_region(info->mem->start,
			info->mem->end - info->mem->start + 1);
	spi_master_put(master);

	return 0;
}

static struct platform_driver spi_ns9xxx_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= spi_ns921x_probe,
	.remove		= __devexit_p(spi_ns921x_remove),
};

static __init int spi_ns921x_init(void)
{
	return platform_driver_register(&spi_ns9xxx_driver);
}

static __exit void spi_ns921x_exit(void)
{
	platform_driver_unregister(&spi_ns9xxx_driver);
}

module_init(spi_ns921x_init);
module_exit(spi_ns921x_exit);

MODULE_AUTHOR("Digi International Inc.");
MODULE_DESCRIPTION("Digi ns921x SPI Controller driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);

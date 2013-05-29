/*
 * linux/drivers/spi/spi_ns9360.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
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
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>

#include <asm/dma.h>
#include <mach/gpio.h>
#include <mach/spi.h>

#define DRIVER_NAME	"spi_ns9360"

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

struct spi_ns9360 {
	void __iomem		*ioaddr;
	void __iomem		*ioaddr_bbus;
	struct resource		*mem;
	struct resource		*mem_bbus;
	struct clk		*clk;
	struct spi_ns9xxx_data	*pdata;
	unsigned		irq_tx;
	unsigned		irq_rx;

	spinlock_t		lock;
	struct list_head	queue;
	struct spi_transfer	*current_transfer;
	unsigned long		remaining_bytes;
	unsigned		dma_xfer_len;
	u32			flags;

	void			*buf_tx;
	void			*buf_rx;
	dma_addr_t		buf_tx_dma;
	dma_addr_t		buf_rx_dma;
	union ns9xxx_dma_desc	*desc_tx;
	union ns9xxx_dma_desc	*desc_rx;
	dma_addr_t		desc_tx_dma;
	dma_addr_t		desc_rx_dma;
};

#define BUFFER_SIZE		PAGE_SIZE
#define INVALID_DMA_ADDRESS	0xffffffff
#define SPI_RX_IDONE		0x00000001
#define SPI_TX_IDONE		0x00000002
#define	SPI_DEFAULT_BPS		1000000
#define	SPI_CS_ACTIVE		1
#define	SPI_CS_INACTIVE		0

#define BBUS_DMA_RXBUFP	0x00
#define BBUS_DMA_TXBUFP	0x20
#define BBUS_DMA_RXCTRL	0x10
#define BBUS_DMA_TXCTRL	0x30
#define BBUS_DMA_CTRL_CE	(1 << 31)
#define BBUS_DMA_CTRL_MODE_FBR	(0x01 << 26)
#define BBUS_DMA_CTRL_MODE_FBW	(0x00 << 26)
#define BBUS_DMA_CTRL_RESET	(1 << 18)
#define BBUS_DMA_CTRL_BTE1	0

#define BBUS_DMA_RXSTAT	0x14
#define BBUS_DMA_TXSTAT	0x34
#define BBUS_DMA_STAT_NCIP	(1 << 31)
#define BBUS_DMA_STAT_ECIP	(1 << 30)
#define BBUS_DMA_STAT_CAIP	(1 << 28)
#define BBUS_DMA_STAT_PCIP	(1 << 27)
#define BBUS_DMA_STAT_NCIE	(1 << 24)
#define BBUS_DMA_STAT_ECIE	(1 << 23)
#define BBUS_DMA_STAT_NRIE	(1 << 22)
#define BBUS_DMA_STAT_CAIE	(1 << 21)
#define BBUS_DMA_STAT_PCIE	(1 << 20)
#define BBUS_DMA_STAT_BLEN_MA	(0x0000FFFF)

#define SPI_CTRLA	0x00
#define SPI_CTRLA_CE		(1 << 31)
#define SPI_CTRLA_WLS8		(3 << 24)
#define SPI_CTRLA_ERXDMA	(1 << 8)
#define SPI_CTRLA_ETXDMA	(1 << 0)

#define SPI_CTRLB	0x04
#define SPI_CTRLB_MODE_SPIM	(1 << 21)
#define SPI_CTRLB_BITORDR	(1 << 19)
#define SPI_BITRATE	0x0C
#define SPI_BITRATE_EBIT	(1 << 31)
#define SPI_BITRATE_TMODE	(1 << 30)
#define SPI_BITRATE_CLKMUX	(0x01 << 24)
#define SPI_BITRATE_TXCINV	(1 << 23)
#define SPI_BITRATE_RXCINV	(1 << 22)
#define SPI_BITRATE_SPCPOL_LOW	(1 << 21)
#define SER_BITRATE_N_MA	(0x00007FFF)

#define	SPI_DMA_TX_STAT_OPT \
	(BBUS_DMA_STAT_NCIE | BBUS_DMA_STAT_ECIE | BBUS_DMA_STAT_NRIE | \
	 BBUS_DMA_STAT_CAIE | BBUS_DMA_STAT_PCIE)
#define	SPI_DMA_RX_STAT_OPT \
	(BBUS_DMA_STAT_NCIE | BBUS_DMA_STAT_ECIE | BBUS_DMA_STAT_NRIE | \
	 BBUS_DMA_STAT_CAIE | BBUS_DMA_STAT_PCIE)
#define	SPI_DMA_TX_CTRL_OPT \
	(BBUS_DMA_CTRL_MODE_FBR | BBUS_DMA_CTRL_BTE1 | BBUS_DMA_CTRL_CE)
#define	SPI_DMA_RX_CTRL_OPT \
	(BBUS_DMA_CTRL_MODE_FBW | BBUS_DMA_CTRL_BTE1 | BBUS_DMA_CTRL_CE)

static void spi_ns9360_chipsel(struct spi_device *spi, int active)
{
unsigned int cspol = spi->mode & SPI_CS_HIGH ? 1 : 0;
void (* func)(struct spi_device *spi, int active) = spi->controller_data;
struct spi_ns9360 *info = spi_master_get_devdata(spi->master);

	/* use special function to toggle CS? */
	if( func )
	{
		/* Reconfigure CS line as GPIO input */
		if( SPI_CS_ACTIVE == active )
			gpio_configure_ns9360(info->pdata->gpios[SPI_EN_GPIO_OFFSET],
						NS9360_GPIO_INPUT,
						NS9360_GPIO_DONT_INVERT,
						NS9360_GPIO_FUNC_3);

		/* Call the special function */
		func( spi, cspol == active);

		/* Reconfigure CS line as SPI_EN */
		if( SPI_CS_INACTIVE == active )
			gpio_configure_ns9360(info->pdata->gpios[SPI_EN_GPIO_OFFSET],
						0,
						0,
						info->pdata->gpio_funcs[SPI_EN_GPIO_OFFSET]);
	}
	else {
		/* Use a GPIO to toggle CS? */
		if( info->pdata->gpio_funcs[SPI_EN_GPIO_OFFSET] == NS9360_GPIO_FUNC_GPIO )
			gpio_set_value( info->pdata->gpios[SPI_EN_GPIO_OFFSET], cspol == active );
		/* otherwise CS will be automatically controlled by the spi engine */
	}
}

/*
 * Configure the controller for the passed SPI-device. Additionally it will
 * use the configuration values of the SPI-transfer if it's different than NULL
 */
static int spi_ns9360_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct spi_ns9360 *info = spi_master_get_devdata(spi->master);
	unsigned int bpw;
	unsigned int hz;
	u32 brate;
	u32 ctrl;

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
	if(!spi->max_speed_hz)
		spi->max_speed_hz = SPI_DEFAULT_BPS;
	hz  = t ? t->speed_hz : spi->max_speed_hz;
	if (!hz)
		hz = spi->max_speed_hz;

	if( hz > (clk_get_rate(info->clk) / 4) ) {
		pr_err("Invalid speed (%d)\n", hz );
		return -EINVAL;
	}
	/* reset previous configuration */
	iowrite32(0x00, info->ioaddr + SPI_CTRLA);

	/* configure bitrate register */
	brate = clk_get_rate(info->clk) / hz / 2 - 1;
	if ((brate & SER_BITRATE_N_MA) != brate) {
		dev_err(&spi->dev, "invalid bitrate %d\n", hz);
		return -EINVAL;
	}

	brate |= SPI_BITRATE_EBIT | SPI_BITRATE_CLKMUX | SPI_BITRATE_TMODE;

	if ((spi->mode & SPI_MODE_3) == SPI_MODE_0)
		brate |= SPI_BITRATE_SPCPOL_LOW;
	else if ((spi->mode & SPI_MODE_3) == SPI_MODE_1)
		brate |= SPI_BITRATE_TXCINV | SPI_BITRATE_RXCINV;
	else if ((spi->mode & SPI_MODE_3) == SPI_MODE_2)
		brate |= SPI_BITRATE_SPCPOL_LOW | SPI_BITRATE_TXCINV |
			SPI_BITRATE_RXCINV;

	iowrite32(brate, info->ioaddr + SPI_BITRATE);

	/* configure bitorder and master operation */
	ctrl = SPI_CTRLB_MODE_SPIM;

	if (!(spi->mode & SPI_LSB_FIRST))
		ctrl |= SPI_CTRLB_BITORDR;

	iowrite32(ctrl, info->ioaddr + SPI_CTRLB);

	/* enable this channel */
	ctrl = SPI_CTRLA_CE | SPI_CTRLA_WLS8;
	iowrite32(ctrl, info->ioaddr + SPI_CTRLA);

	return 0;
}

static int spi_ns9360_setup(struct spi_device *spi)
{
	struct spi_ns9360 *info = spi_master_get_devdata(spi->master);
	int ret;

	if (spi->chip_select > spi->master->num_chipselect) {
		dev_dbg(&spi->dev,
				"setup: invalid chipselect %u (%u defined)\n",
				spi->chip_select, spi->master->num_chipselect);
		return -EINVAL;
	}

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	if ((spi->mode & SPI_CS_HIGH) && !info->pdata) {
		/* FIXME: switch inverter on CS-pin */
	}

	/* Configure per-transfer settings (speed/bpw/mode) */
	ret = spi_ns9360_setupxfer(spi, NULL);

	return ret;
}

static void spi_ns9360_map_xfer(struct spi_device *spi,
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

static void spi_ns9360_next_xfer(struct spi_master *master,
		struct spi_message *msg)
{
	struct spi_ns9360 *info = spi_master_get_devdata(master);
	struct spi_transfer *xfer;
	dma_addr_t tx_dma, rx_dma;
	u32 len, ctrl;
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
	err = spi_ns9360_setupxfer(msg->spi, xfer);
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
	/* setup the DMA channels */
	iowrite32(BBUS_DMA_CTRL_RESET, info->ioaddr_bbus + BBUS_DMA_RXCTRL);
	iowrite32(BBUS_DMA_CTRL_RESET, info->ioaddr_bbus + BBUS_DMA_TXCTRL);

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
	iowrite32(SPI_DMA_TX_STAT_OPT, info->ioaddr_bbus + BBUS_DMA_TXSTAT);
	iowrite32(SPI_DMA_RX_STAT_OPT, info->ioaddr_bbus + BBUS_DMA_RXSTAT);
	iowrite32((u32)info->desc_tx_dma, info->ioaddr_bbus + BBUS_DMA_TXBUFP);
	iowrite32((u32)info->desc_rx_dma, info->ioaddr_bbus + BBUS_DMA_RXBUFP);
	iowrite32(SPI_DMA_TX_CTRL_OPT, info->ioaddr_bbus + BBUS_DMA_TXCTRL);
	iowrite32(SPI_DMA_RX_CTRL_OPT, info->ioaddr_bbus + BBUS_DMA_RXCTRL);

	/* fire up dma channels */
	ctrl = ioread32(info->ioaddr + SPI_CTRLA);
	ctrl |= SPI_CTRLA_ETXDMA | SPI_CTRLA_ERXDMA;
	iowrite32(ctrl, info->ioaddr + SPI_CTRLA);
}

static void spi_ns9360_next_message(struct spi_master *master)
{
	struct spi_ns9360 *info = spi_master_get_devdata(master);
	struct spi_message *msg;

	BUG_ON(info->current_transfer);

	msg = list_entry(info->queue.next, struct spi_message, queue);

	spi_ns9360_chipsel(msg->spi, SPI_CS_ACTIVE);
	spi_ns9360_next_xfer(master, msg);
}

static int spi_ns9360_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct spi_ns9360 *info = spi_master_get_devdata(spi->master);
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
			spi_ns9360_map_xfer(spi, xfer);
	}

	msg->status = -EINPROGRESS;
	msg->actual_length = 0;

	spin_lock_irqsave(&info->lock, flags);
	list_add_tail(&msg->queue, &info->queue);
	if (!info->current_transfer)
		spi_ns9360_next_message(spi->master);
	spin_unlock_irqrestore(&info->lock, flags);

	return 0;
}

static void spi_ns9360_irq_trigger_next(struct spi_master *master)
{
	struct spi_ns9360 *info = spi_master_get_devdata(master);
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
			spi_ns9360_chipsel(msg->spi, SPI_CS_INACTIVE);

			list_del(&msg->queue);
			msg->status = 0;

			spin_unlock(&info->lock);
			msg->complete(msg->context);
			spin_lock(&info->lock);

			info->current_transfer = NULL;

			/* continue; complete() may have queued requests */
			if (list_empty(&info->queue)) {
				ctrl = ioread32(info->ioaddr_bbus +
						BBUS_DMA_RXCTRL);
				ctrl &= ~BBUS_DMA_CTRL_CE;
				iowrite32(ctrl, info->ioaddr_bbus +
						BBUS_DMA_RXCTRL);
				ctrl = ioread32(info->ioaddr_bbus +
						BBUS_DMA_TXCTRL);
				ctrl &= ~BBUS_DMA_CTRL_CE;
				iowrite32(ctrl, info->ioaddr_bbus +
						BBUS_DMA_TXCTRL);
			} else
				spi_ns9360_next_message(master);
		} else {
			if (xfer->cs_change) {
				spi_ns9360_chipsel(msg->spi, SPI_CS_INACTIVE);
				udelay(1);
				spi_ns9360_chipsel(msg->spi, SPI_CS_ACTIVE);
			}

			spi_ns9360_next_xfer(master, msg);
		}
	} else {
		/* There are still pending data in the current transfer */
		spi_ns9360_next_xfer(master, msg);
	}
}

static irqreturn_t spi_ns9360_irq_tx(int irq, void *dev_id)
{
	struct spi_master *master = dev_id;
	struct spi_ns9360 *info = spi_master_get_devdata(master);
	u32 status;

	spin_lock(&info->lock);

	/* get status and mask all pending interrupts */
	status = ioread32(info->ioaddr_bbus + BBUS_DMA_TXSTAT);
	iowrite32(status, info->ioaddr_bbus + BBUS_DMA_TXSTAT);

	if (status & (BBUS_DMA_STAT_ECIP | BBUS_DMA_STAT_CAIP |
				BBUS_DMA_STAT_PCIP)) {
		dev_err(&master->dev, "tx dma failure! status = 0x%08x\n",
				status);

		/* retrigger the xfer */
		spi_ns9360_next_xfer(master, list_entry(info->queue.next,
					struct spi_message, queue));
	}

	if (status & BBUS_DMA_STAT_NCIP) {
		info->flags |= SPI_TX_IDONE;
		if (info->flags & SPI_RX_IDONE)
			spi_ns9360_irq_trigger_next(master);
	}

	spin_unlock(&info->lock);

	return IRQ_HANDLED;
}

static irqreturn_t spi_ns9360_irq_rx(int irq, void *dev_id)
{
	struct spi_master *master = dev_id;
	struct spi_ns9360 *info = spi_master_get_devdata(master);
	u32 status, dlen;

	spin_lock(&info->lock);

	/* get status and mask all pending interrupts */
	status = ioread32(info->ioaddr_bbus + BBUS_DMA_RXSTAT);
	iowrite32(status, info->ioaddr_bbus + BBUS_DMA_RXSTAT);

	dlen = status & BBUS_DMA_STAT_BLEN_MA;
	if (info->dma_xfer_len != dlen)
		/* ?? what to do... ?? */
		dev_err(&master->dev, "incomplete dma xfer"
				"(%d/%d bytes transfered) \n",
				dlen, info->dma_xfer_len);

	if (status & (BBUS_DMA_STAT_ECIP | BBUS_DMA_STAT_CAIP |
				BBUS_DMA_STAT_PCIP)) {
		dev_err(&master->dev, "rx dma failure! status = 0x%08x\n",
				status);

		/* retrigger the xfer */
		spi_ns9360_next_xfer(master, list_entry(info->queue.next,
					struct spi_message, queue));
	}

	if (status & BBUS_DMA_STAT_NCIP) {
		info->flags |= SPI_RX_IDONE;
		if (info->flags & SPI_TX_IDONE)
			spi_ns9360_irq_trigger_next(master);
	}

	spin_unlock(&info->lock);

	return IRQ_HANDLED;
}

static int __devinit spi_ns9360_probe(struct platform_device *pdev)
{
	int ret;
	struct spi_master *master;
	struct spi_ns9360 *info;
	int gpio_cs, gpio_cs_func;

	master = spi_alloc_master(&pdev->dev, sizeof(*info));
	if (!master) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "%s: err_alloc_master\n", __func__);
		goto err_alloc_master;
	}

	platform_set_drvdata(pdev, master);

	info = spi_master_get_devdata(master);
	info->pdata = pdev->dev.platform_data;

	info->irq_rx = platform_get_irq(pdev, 0);
	if (info->irq_rx < 0) {
		ret = -ENOENT;
		dev_dbg(&pdev->dev, "%s: err_get_irq_rx\n", __func__);
		goto err_get_irq_rx;
	}

	info->irq_tx = platform_get_irq(pdev, 1);
	if (info->irq_tx < 0) {
		ret = -ENOENT;
		dev_dbg(&pdev->dev, "%s: err_get_irq_tx\n", __func__);
		goto err_get_irq_tx;
	}

	info->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!info->mem) {
		ret = -ENOENT;
		dev_dbg(&pdev->dev, "%s: err_get_mem\n", __func__);
		goto err_get_mem;
	}

	if (!request_mem_region(info->mem->start,
			info->mem->end - info->mem->start + 1,
			pdev->name)) {
		ret = -ENXIO;
		dev_dbg(&pdev->dev, "%s: err_req_mem\n", __func__);
		goto err_req_mem;
	}

	info->ioaddr = ioremap(info->mem->start,
			info->mem->end - info->mem->start + 1);
	if (!info->ioaddr) {
		ret = -ENXIO;
		dev_dbg(&pdev->dev, "%s: err_map_mem\n", __func__);
		goto err_map_mem;
	}

	info->mem_bbus = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!info->mem_bbus) {
		ret = -ENOENT;
		dev_dbg(&pdev->dev, "%s: err_get_mem_bbus\n", __func__);
		goto err_get_mem_bbus;
	}

	if (!request_mem_region(info->mem_bbus->start,
			info->mem_bbus->end - info->mem_bbus->start + 1,
			pdev->name)) {
		ret = -ENXIO;
		dev_dbg(&pdev->dev, "%s: err_req_mem_bbus\n", __func__);
		goto err_req_mem_bbus;
	}

	info->ioaddr_bbus = ioremap(info->mem_bbus->start,
			info->mem_bbus->end - info->mem_bbus->start + 1);
	if (!info->ioaddr_bbus) {
		ret = -ENXIO;
		dev_dbg(&pdev->dev, "%s: err_map_mem_bbus\n", __func__);
		goto err_map_mem_bbus;
	}

	master->bus_num = pdev->id;
	/* hardware controlled cs */
	master->num_chipselect = 1;
	/* the spi->mode bits understood by this driver: */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST;

	/* If CS is GPIO controlled, configure it as output */
	gpio_cs = info->pdata->gpios[SPI_EN_GPIO_OFFSET];
		gpio_cs_func = info->pdata->gpio_funcs[SPI_EN_GPIO_OFFSET];
		if( NS9360_GPIO_FUNC_GPIO == gpio_cs_func )
			gpio_direction_output( gpio_cs,
					gpio_get_value( gpio_cs ) );

	master->setup = spi_ns9360_setup;
	master->transfer = spi_ns9360_transfer;

	spin_lock_init(&info->lock);
	INIT_LIST_HEAD(&info->queue);

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
		ret = PTR_ERR(info->clk);
		dev_dbg(&pdev->dev, "%s: err_clk_get\n", __func__);
		goto err_clk_get;
	}

	ret = clk_enable(info->clk);
	if (ret) {
		dev_dbg(&pdev->dev, "%s: err_clk_enable\n", __func__);
		goto err_clk_enable;
	}

	ret = request_irq(info->irq_rx, spi_ns9360_irq_rx, 0,
			pdev->name, master);
	if (ret) {
		dev_dbg(&pdev->dev, "%s: err_req_irq_rx\n", __func__);
		goto err_req_irq_rx;
	}

	ret = request_irq(info->irq_tx, spi_ns9360_irq_tx, 0,
			pdev->name, master);
	if (ret) {
		dev_dbg(&pdev->dev, "%s: err_req_irq_tx\n", __func__);
		goto err_req_irq_tx;
	}

	ret = spi_register_master(master);
	if (ret) {
		dev_dbg(&pdev->dev, "%s: err_reg_reg_master\n", __func__);
		goto err_reg_master;
	}

	dev_info(&pdev->dev, "NS9360 SPI controller at 0x%p (irq: %d/%d)\n",
			info->ioaddr, info->irq_rx, info->irq_tx);

	return 0;

err_reg_master:
	free_irq(info->irq_tx, master);
err_req_irq_tx:
	free_irq(info->irq_rx, master);
err_req_irq_rx:
	clk_disable(info->clk);
err_clk_enable:
	clk_put(info->clk);
err_clk_get:
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
	iounmap(info->ioaddr_bbus);
err_map_mem_bbus:
	release_mem_region(info->mem_bbus->start,
			info->mem_bbus->end - info->mem_bbus->start + 1);
err_req_mem_bbus:
err_get_mem_bbus:
	iounmap(info->ioaddr);
err_map_mem:
	release_mem_region(info->mem->start,
			info->mem->end - info->mem->start + 1);
err_req_mem:
err_get_mem:
err_get_irq_tx:
err_get_irq_rx:
	spi_master_put(master);
err_alloc_master:

	return ret;
}

static int __devexit spi_ns9360_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct spi_ns9360 *info = spi_master_get_devdata(master);

	free_irq(info->irq_tx, master);
	free_irq(info->irq_rx, master);

	clk_disable(info->clk);
	clk_put(info->clk);

	dma_free_coherent(&pdev->dev, sizeof(info->desc_rx),
			info->desc_rx, info->desc_rx_dma);
	dma_free_coherent(&pdev->dev, sizeof(info->desc_tx),
			info->desc_tx, info->desc_tx_dma);
	dma_free_coherent(&pdev->dev, BUFFER_SIZE,
			info->buf_rx, info->buf_rx_dma);
	dma_free_coherent(&pdev->dev, BUFFER_SIZE,
			info->buf_tx, info->buf_tx_dma);

	iounmap(info->ioaddr_bbus);
	release_mem_region(info->mem_bbus->start,
			info->mem_bbus->end - info->mem_bbus->start + 1);

	iounmap(info->ioaddr);
	release_mem_region(info->mem->start,
			info->mem->end - info->mem->start + 1);

	spi_master_put(master);

	return 0;
}

static struct platform_driver spi_ns9360_driver = {
	.probe		= spi_ns9360_probe,
	.remove		= __devexit_p(spi_ns9360_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init spi_ns9360_init(void)
{
	return platform_driver_register(&spi_ns9360_driver);
}

static void __exit spi_ns9360_exit(void)
{
	platform_driver_unregister(&spi_ns9360_driver);
}

module_init(spi_ns9360_init);
module_exit(spi_ns9360_exit);

MODULE_DESCRIPTION("NS9360 SPI Driver");
MODULE_AUTHOR("Digi International Inc.");
MODULE_LICENSE("GPL v2");

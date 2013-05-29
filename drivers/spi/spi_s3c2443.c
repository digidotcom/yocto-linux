/* -*- linux-c -*-
 *
 * linux/drivers/spi/spi_s3c2443.c
 *
 * Copyright (c) 2008 Digi International
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  !Revision:   $Revision: 1.1.1.1 $
 *  !Author:     Luis Galdos
 *  !Descr:      High Speed SPI driver for the S3C2443
 *  !References: Based on the driver of the Samsung SMDK for the S3C2443
 *
 */

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <asm/io.h>
#include <asm/dma.h>
#include <mach/hardware.h>
#include <asm/delay.h>

#include <mach/gpio.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>
#include <plat/regs-spi.h>
#include <mach/dma.h>
#include <plat/spi.h>
#include <mach/dma.h>
#include <mach/regs-s3c2443-clock.h>
#include <plat/regs-dma.h>


/* This macro selects the virtual DMA-channel to use */
#define SPI_TX_CHANNEL				DMACH_SPI0
#define SPI_RX_CHANNEL				DMACH_SPI0_RX

#define S3C2443_SPI_DMA_BUFFER			(PAGE_SIZE)

/* Supported modes */
#define S3C2443_SPI_SUPPORTED_MODES		(SPI_CPOL | SPI_CPHA)

/* DMA transfer unit (byte). */
#define S3C24XX_DMA_XFER_BYTE			1
#define S3C24XX_DMA_XFER_HWORD			2
#define S3C24XX_DMA_XFER_WORD			4

/* Used for setting the CS on the different SPI-operations */
#define SPI_CS_ACTIVE				1
#define SPI_CS_INACTIVE				0

#define printk_err(fmt, args...)                printk(KERN_ERR "[ ERROR ] spi-s3c2443: " fmt, ## args)
#define printk_info(fmt, args...)               printk(KERN_INFO "spi-s3c2443: " fmt, ## args)

#if 0
#define S3C2443_SPI_DEBUG
#endif

#ifdef S3C2443_SPI_DEBUG
#  define printk_debug(fmt, args...)		printk(KERN_DEBUG "spi-s3c2443: %s() " fmt, __FUNCTION__ , ## args)
#else
#  define printk_debug(fmt, args...)
#endif


enum s3c2443_xfer_t {
	S3C2443_DMA_TX = 0,
	S3C2443_DMA_RX = 1,
};


struct s3c2443_spi {
	void __iomem		*regs;
	int			 irq;
	struct clk		*clk;
	struct resource		*ioarea;
	struct spi_master	*master;
	struct device		*dev;
	struct s3c2443_spi_info *pdata;

	struct list_head xmit_queue;
	spinlock_t xmit_lock;
	struct spi_transfer *current_xfer;
	long remaining_bytes;
	unsigned long dma_xfer_len;
	struct tasklet_struct xmit_tasklet;
	void *tx_buf;
	void *rx_buf;
	dma_addr_t tx_dma;
	dma_addr_t rx_dma;

	unsigned int dma_ch;
	struct s3c2410_dma_client dmach;
	unsigned int dma_ch_rx;
	struct s3c2410_dma_client dmach_rx;
};


/* Internal functions */
static void s3c2443_spi_next_message(struct spi_master *master);
static void s3c2443_spi_next_xfer(struct spi_master *master, struct spi_message *msg);


static inline struct s3c2443_spi *spi_to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static inline struct s3c2443_spi *master_to_hw(struct spi_master *master)
{
	return spi_master_get_devdata(master);
}


static void s3c2443_spi_chipsel(struct spi_device *spi, int active)
{
	/* unsigned int cspol; */
	struct s3c2443_spi *hw;
	void (* cs_callback)(struct spi_device *_spi, int _act);
	unsigned long slavecfg;
	struct s3c2443_spi_info *info;
	struct s3c24xx_spi_gpio *cs;

	hw = spi_to_hw(spi);
	if (!hw) {
		printk_err("HW null pointer found!\n");
		return;
	}

	/*
	 * If the SPI-device has a pointer function for controlling the
	 * chip select, then we will call its passed function. Otherwise the
	 * CS will be controlled by the SPI-hardware
	 */
	if (spi->controller_data) {
		info = hw->pdata;
		cs = info->cs;

		printk_debug("Calling the controller data\n");

		/*
		 * Since the SPI-device will control its own CS disable ours by
		 * configuring it as input, then we have enabled the pull-up, right?
		 */
		if (active == SPI_CS_ACTIVE)
			s3c2410_gpio_cfgpin(cs->nr, S3C2410_GPIO_INPUT);

		cs_callback = spi->controller_data;
		cs_callback(spi, !active);

		/* Now reenable our CS, but before disable the last CS-activation! */
		if (active == SPI_CS_INACTIVE) {
			writel(S3C2443_SPI0_SLAVE_SIG_INACT,
			       hw->regs + S3C2443_SPI0_SLAVE_SEL);
			s3c2410_gpio_cfgpin(cs->nr, cs->cfg);
		}
	}

	/*
	 * Even an external interrupt is being used, we MUST enable the slave
	 * register for being able to shift the data to the bus. Otherwise the
	 * FIFO will not send the data to the bus, and an error from the DMA-engine
	 * will be generated: "... timeout ... load buffer ..."
	 */
	printk_debug("Using the HW-CS (%i)\n", active);
	slavecfg = S3C2443_SPI0_SLAVE_SIG_INACT;
	if (active == SPI_CS_ACTIVE)
		slavecfg = S3C2443_SPI0_SLAVE_SIG_ACT;

	writel(slavecfg, hw->regs + S3C2443_SPI0_SLAVE_SEL);
}

/*
 * Please reset the SPI-controller before starting a transfer. Otherwise the
 * configuration of the last transfer will interfere with the new transfer
 */
static void s3c2443_spi_sw_reset(struct s3c2443_spi *spi)
{
	unsigned long cfg;

	cfg = readl(spi->regs + S3C2443_SPI0_CH_CFG);
        writel(cfg | S3C2443_SPI0_CH_SW_RST, spi->regs + S3C2443_SPI0_CH_CFG);
	writel(cfg, spi->regs + S3C2443_SPI0_CH_CFG);
}


static int s3c2443_spi_hw_init(struct s3c2443_spi *hw)
{
	unsigned long misccr, clkcfg;
	struct s3c2443_spi_info *info;
	int cnt;
	struct s3c24xx_spi_gpio *cs;

	info = hw->pdata;

        writel(readl(S3C2443_SCLKCON) | S3C2443_SCLKCON_HSSPICLK, S3C2443_SCLKCON);
        writel(readl(S3C2443_PCLKCON) | S3C2443_PCLKCON_HSSPI, S3C2443_PCLKCON);

	if (info->input_clk == S3C2443_HSSPI_INCLK_PCLK)
		clkcfg = S3C2443_SPI0_CLKSEL_PCLK;
	else
		clkcfg = S3C2443_SPI0_CLKSEL_EPLL;
	writel(clkcfg, hw->regs + S3C2443_SPI0_CLK_CFG);

	misccr = __raw_readl(S3C24XX_MISCCR);
	__raw_writel(misccr | (1 << 31), S3C24XX_MISCCR);

        /* HSSPI software reset */
	s3c2443_spi_sw_reset(hw);

	/* Configure the chip selects depending on the passed platform data */
        s3c2410_gpio_cfgpin(info->miso.nr, info->miso.cfg);
	s3c2410_gpio_cfgpin(info->mosi.nr, info->mosi.cfg);
	s3c2410_gpio_cfgpin(info->clk.nr, info->clk.cfg);
	for (cnt = 0; cnt < info->num_chipselect; cnt++) {

		/*
		 * We need to activate the pullup for being able to reconfigure the
		 * CS as an input IO
		 */
		cs = info->cs + cnt;
		s3c2410_gpio_cfgpin(cs->nr, cs->cfg);
		s3c2410_gpio_pullup(cs->nr, 0);
	}

	return 0;
}

/*
 * Configure the controller for the passed SPI-device. Additionally it will
 * use the configuration values of the SPI-transfer if it's different than NULL
 */
static int s3c2443_spi_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct s3c2443_spi *hw = spi_to_hw(spi);
	unsigned int bpw;
	unsigned int hz, max_hz, min_hz;
	unsigned int div;
	unsigned long clkreg, chcfg, mode;

	printk_debug("Setup for next xfer called\n");

	/* If the bits per word is zero, then use the default value of the SPI-device */
	bpw = t ? t->bits_per_word : spi->bits_per_word;
	if (!bpw)
		bpw = spi->bits_per_word;

	/* We only support eigth bits per word */
	if (bpw != 8) {
		printk_err("Invalid bits-per-word (%d)\n", bpw);
		return -EINVAL;
	}

	/*
	 * If the clock rate of the xfer is zero, then use the already configured
	 * rate of this SPI-device
	 */
	hz  = t ? t->speed_hz : spi->max_speed_hz;
	if (!hz)
		hz = spi->max_speed_hz;

	/* The below calculation is coming from the SMDK */
	max_hz = clk_get_rate(hw->clk) / 2;
	min_hz = max_hz / 256;
	if (hz > max_hz || hz < min_hz) {
		printk_err("Speed %d Hz out of range (Min %i | Max %i)\n",
			   hz, min_hz, max_hz);
		return -EINVAL;
	}

	div = clk_get_rate(hw->clk) / hz;
	div /= 2;
	div = (div) ? (div - 1) : (0);
	if (div > S3C2443_SPI0_CLK_PRE_MASK) {
		printk_err("Invalid clock divider %d (%i Hz)\n", div, hz);
		return -EINVAL;
	}

	/* Only modify the prescaler */
	printk_debug("Setting pre-scaler to %d (%d Hz)\n", div, hz);
	clkreg = readl(hw->regs + S3C2443_SPI0_CLK_CFG) & ~(S3C2443_SPI0_CLK_PRE_MASK);
	writel(clkreg | div, hw->regs + S3C2443_SPI0_CLK_CFG);

	/*
	 * For some reasons the clock configuration isn't working correctly by
	 * the FIRST execution. For this reason let's make a sanity check and
	 * if required reset the hardware and reconfigure the clock again. This seems
	 * to solve the problem.
	 * Luis Galdos
	 */
	clkreg = readl(hw->regs + S3C2443_SPI0_CLK_CFG);
	if ((clkreg & S3C2443_SPI0_CLK_PRE_MASK) != div) {
		printk_debug("Need to reconfigure the clock\n");
		s3c2443_spi_hw_init(hw);
		s3c2443_spi_sw_reset(hw);
		writel(clkreg | div, hw->regs + S3C2443_SPI0_CLK_CFG);
	}

	/* Set the correct mode for the passed SPI-transfer */
	mode = spi->mode & 0x03;
	switch (mode) {
	case SPI_MODE_2:
		/* Sampled by falling edges with a high inactive clock (MODE_2) */
		chcfg = S3C2443_SPI0_CH_FALLING | S3C2443_SPI0_CH_FORMAT_A;
		break;

	case SPI_MODE_1:
		/* Sampled by falling edges with a low inactive clock (MODE_1) */
		chcfg = S3C2443_SPI0_CH_RISING | S3C2443_SPI0_CH_FORMAT_B;
		break;

	case SPI_MODE_3:
		/* Sampled by rising edges with a high inactive clock (MODE_3) */
		chcfg = S3C2443_SPI0_CH_FALLING | S3C2443_SPI0_CH_FORMAT_B;
		break;

	case SPI_MODE_0:
	default:
		/* Sampled by rising edges with a low inactive clock (MODE_0) */
		chcfg = S3C2443_SPI0_CH_RISING | S3C2443_SPI0_CH_FORMAT_A;
		break;
	}

	printk_debug("Configuring the mode %lu\n", mode);
	writel(chcfg , hw->regs + S3C2443_SPI0_CH_CFG);
	return 0;
}

/*
 * Configure the master-controller. This function is only called when the higher
 * layer wants to MODIFY the current configuration of the controller.
 */
static int s3c2443_spi_setup(struct spi_device *spi)
{
	int ret;

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	if (spi->mode & ~S3C2443_SPI_SUPPORTED_MODES) {
		printk_debug("Unsupported mode bits %x\n",
			     spi->mode & ~S3C2443_SPI_SUPPORTED_MODES);
		return -EINVAL;
	}

	ret = s3c2443_spi_setupxfer(spi, NULL);
	if (!ret)
		printk_debug("Mode %d, %u bpw, %d HZ\n",
			     spi->mode, spi->bits_per_word,
			     spi->max_speed_hz);

	return ret;
}

/*
 * Tasklet function for checking the message queue. If there is another transfer
 * pending, then this function will start the transfer, otherwise it calls the
 * complete function of the SPI-message and reset the internal data for the
 * next available message of the internal message queue and start it too
 */
static void s3c2443_spi_next_tasklet(unsigned long data)
{
	struct s3c2443_spi *hw;
	struct spi_transfer *xfer;
	struct spi_message *msg;
	unsigned long stat;
	int timeout, err;

	hw = (struct s3c2443_spi *)data;
	if (!hw)
		return;

	xfer = hw->current_xfer;
	msg = list_entry(hw->xmit_queue.next, struct spi_message, queue);
	if (!xfer || !msg)
		return;

	/*
	 * If we used the own internal DMA-buffer for the DMA-transfer then copy
	 * the data to the higher SPI-message. If not, then cause we have
	 * passed the transfer-buffer to the DMA-controller
	 */
	if (!msg->is_dma_mapped && xfer->rx_buf && hw->dma_xfer_len) {
		printk_debug("Copying %lu bytes to SPI-buffer %p\n",
			     hw->dma_xfer_len, xfer->rx_buf);
		memcpy(xfer->rx_buf, hw->rx_buf, hw->dma_xfer_len);
	}

	/*
	 * We have a problem at this place, then we don't know if
	 * the shift-register is really empty. For this reason we
	 * use the below loop. Please note that by high configured
	 * clocks the shift register will be already empty at this point
	 * and by low clocks the usleep is not really relevant
	 * compared with the complete transmission time. The problem is
	 * that udelay will cause a higher CPU-load, but we don't
	 * want to sleep before calling the tasklet, and at this place
	 * is no more possible to take a siesta.
	 * Luis Galdos
	 */
	err = (hw->remaining_bytes < 0) ? (hw->remaining_bytes) : 0;
	if (!err && xfer->tx_buf) {
		stat = readl(hw->regs + S3C2443_SPI0_STATUS);
		timeout = (err) ? 0 : 0xffff;
		while (!(stat & S3C2443_SPI0_STUS_TX_DONE) && timeout) {
			udelay(2);
			stat = readl(hw->regs + S3C2443_SPI0_STATUS);
			timeout--;
		}

		if (!timeout)
			err = -ETIME;
	}

	/*
	 * If NO more data is remaining then update the internal variables of the
	 * SPI-message and call the complete-function. The errors are passed to us
	 * with an negative value of remaining_bytes
	 */
	if (hw->remaining_bytes == 0 || err) {

		/* Disable the IRQs and FIFOs, then we are done */
		writel(0x00, hw->regs + S3C2443_SPI0_CH_CFG);

		msg->actual_length += hw->dma_xfer_len;

		if (xfer->delay_usecs)
			udelay(xfer->delay_usecs);

		/* Check if the current SPI-transfer was the last of the message */
		if (msg->transfers.prev == &xfer->transfer_list || err) {
			msg->status = err;

			/* Disable the slave selection (chip select inactive) */
			s3c2443_spi_chipsel(msg->spi, SPI_CS_INACTIVE);

			stat = readl(hw->regs + S3C2443_SPI0_STATUS);

			printk_debug("Msg complete %i | stat 0x08%x)\n",
				     err, (unsigned int)stat);

			list_del(&msg->queue);
			msg->complete(msg->context);

			/*
			 * Now, check for the next message of the internal xmit-queue.
			 * But first lock the code segment, then probably somebody is
			 * trying to start a new transfer at this moment
			 */
			spin_lock(&hw->xmit_lock);
			hw->current_xfer = NULL;
			if (!list_empty(&hw->xmit_queue))
				s3c2443_spi_next_message(hw->master);
			spin_unlock(&hw->xmit_lock);

		} else {
			printk_debug("Calling the next XFER of the msg %p\n", msg);

			/* Toggle the CS if requested */
			if (xfer->cs_change) {
                                s3c2443_spi_chipsel(msg->spi, SPI_CS_INACTIVE);
                                udelay(1);
                                s3c2443_spi_chipsel(msg->spi, SPI_CS_ACTIVE);
                        }

			s3c2443_spi_next_xfer(hw->master, msg);
		}

	} else {
		/*
		 * Start the next write operation if there is remaining data of the
		 * last SPI-transfer
		 */
		printk_debug("Need to send more data (%lu)\n", hw->remaining_bytes);
		s3c2443_spi_next_xfer(hw->master, msg);
	}

}

/*
 * The DMA-subsytem calls this function from an interrupt context. For this
 * reason we only update the remaining-bytes counter and check if can
 * schedule the tasklet for the SPI-complete. BUT the called will be scheduled
 * only if the FIFO has no more data to send, otherwise we will modify the interrupt
 * registers for receiving an interrupt from the TX-FIFO later.
 */
static void s3c2443_spi_dma_callback(struct s3c2410_dma_chan *ch,
				       void *_hw, int size,
				       enum s3c2410_dma_buffresult result)
{
	struct s3c2443_spi *hw;
	unsigned long mode, inten, stat;
	int is_tx;

	hw = (struct s3c2443_spi *)_hw;
	if (!hw) {
		printk_err("Got a NULL pointer from the DMA-callback!\n");
		return;
	}

	/* Check if the callback corresponds to the RX or TX-channel */
	is_tx = (ch->client == &hw->dmach) ? (1) : (0);
	printk_debug("%s DMA callback | Ch %i\n", (is_tx) ? "TX" : "RX", ch->number);

	/* If the transfer has a RX-channel running then need to wait for it */
	if (is_tx && hw->current_xfer->rx_buf && result == S3C2410_RES_OK) {
		printk_debug("Need to wait for the RX-DMA\n");
		return;
	}

	/*
	 * If an error is detected pass the info to the tasklet too (place the error
	 * message into the status of the message).
	 * By aborts only set the number of remaining bytes to zero, so that the
	 * tasklet will complete the message
	 */
	switch (result) {
	case S3C2410_RES_OK:
		printk_debug("Result OK\n");
		hw->remaining_bytes -= size;
		hw->dma_xfer_len += size;
		break;
	case S3C2410_RES_ERR:
		printk_err("Result ERROR\n");
		hw->dma_xfer_len = 0;
		hw->remaining_bytes = -ERESTART;
		break;
	case S3C2410_RES_ABORT:
		printk_debug("Result ABORT\n");
		hw->dma_xfer_len += size;
		hw->remaining_bytes = -ECONNABORTED;
		break;
	}

	/*
	 * At this point, normally the TX-FIFO has still data to transfer to the
	 * bus. That means for us, we can enable the interrupt for the TX-FIFO
	 * which informs us about the TX-done. So, only enable
	 * the level IRQ, so that the IRQ-handler will call the tasklet later.
	 * BTW, we only enable the interrupt when we are transferring data using
	 * the DMA-controller, otherwise the DMA-subsystem calls this function when
	 * all the bytes were already read
	 */
	if (likely(result == S3C2410_RES_OK && hw->current_xfer->tx_buf)) {
		stat = readl(hw->regs + S3C2443_SPI0_STATUS);

		printk_debug("Need to wait for the TX-FIFO | stat 0x%x\n",
		       (unsigned int)stat);

		/* Disable the DMA-mode for the TX-FIFO */
		mode = readl(hw->regs + S3C2443_SPI0_MODE_CFG);
		mode &= ~S3C2443_SPI0_MODE_TXDMA_ON;
		mode |= (1 << 5);
		writel(mode, hw->regs + S3C2443_SPI0_MODE_CFG);

		/* Enable the ready interrupt */
		inten = readl(hw->regs + S3C2443_SPI0_INT_EN);
		inten |= S3C2443_SPI0_INT_TX_UNDERRUN_EN |
			S3C2443_SPI0_INT_TX_FIFORDY_EN;
		writel(inten , hw->regs + S3C2443_SPI0_INT_EN);
	} else {
		printk_debug("Calling the tasklet from the DMA-callback\n");
		tasklet_schedule(&hw->xmit_tasklet);
	}
}

/*
 * IMPORTANT: If the data transfer is smaller than four bytes, then the SPI-controller
 * must be configured without the burst-mode! Otherwise the channel will run
 * amok!
 * Luis Galdos
 */
static int s3c2443_spi_dma_init(unsigned int dma_ch, enum s3c2443_xfer_t mode,
				int length)
{
	int xmode;

	printk_debug("DMA init for next %s\n", (mode == S3C2443_DMA_TX) ? "TX" : "RX");

	if (mode == S3C2443_DMA_TX) {
		/*
		 * For TX-transfers configure the destination with fixed address and
		 * inside the APB-bus. The source of transfers will be the
		 * system memory
		 */
                s3c2410_dma_devconfig(dma_ch, S3C2410_DMASRC_MEM,
				      S3C2443_SPI0_TX_DATA_PA);
        } else if (mode == S3C2443_DMA_RX) {
		/*
		 * By RX-transfer the source is the RX-FIFO of the SPI-controller.
		 * The controller is part of the APB and should not increment
		 * the address
		 */
                s3c2410_dma_devconfig(dma_ch, S3C2410_DMASRC_HW,
				      S3C2443_SPI0_RX_DATA_PA);
	} else {
		printk_err("Invalid DMA transfer mode (%i)\n", mode);
		return -EINVAL;
	}

	/*
	 * Select the correct transfer mode depending on the number of bytes
	 * to transfer. Unfortunately the DMA-controller can transfer only data
	 * quantums depending on the number of the data size to be transferred
	 * (see register: DCON, field DSZ)
	 * Luis Galdos
	 */
	if (!(length & 0x3))
		xmode = S3C24XX_DMA_XFER_WORD;
	else
		xmode = S3C24XX_DMA_XFER_BYTE;

	/* @XXX: Don't use the burst mode, then it's not working with this driver */
	s3c2410_dma_config(dma_ch,
			   xmode);

	s3c2410_dma_setflags(dma_ch, S3C2410_DMAF_AUTOSTART);
        return 0;
}

/*
 * This function is used to handle the next transfer of a SPI-message.
 * If the configuration of the transfer failed, then this function will
 * set the error code and will schedule the tasklet for completing the
 * SPI-message correctly.
 */
static void s3c2443_spi_next_xfer(struct spi_master *master,
				  struct spi_message *msg)
{
	struct spi_transfer *xfer;
	struct s3c2443_spi *hw;
	unsigned long len;
	dma_addr_t rx_dma, tx_dma;
	int err;
	unsigned long clkcfg, chcfg, modecfg, spi_packet;

	hw = master_to_hw(master);

	/* Always reset the controller first */
	s3c2443_spi_sw_reset(hw);

	/*
	 * Check if we are going to start sending a complete new message, or if
	 * the passed message was already used for sending a SPI-transfer
	 */
	xfer = hw->current_xfer;
	if (!xfer || hw->remaining_bytes == 0) {
		if (xfer)
			xfer = list_entry(xfer->transfer_list.next,
					  struct spi_transfer, transfer_list);
		else
			xfer = list_entry(msg->transfers.next,
					  struct spi_transfer, transfer_list);

		hw->current_xfer = xfer;
		hw->remaining_bytes = xfer->len;
		hw->dma_xfer_len = 0;
	}

	/* Set the requested configuration for this new SPI-transfer */
	err = s3c2443_spi_setupxfer(msg->spi, xfer);
	if (err) {
		printk_err("Setting up the next SPI-transfer\n");
		hw->remaining_bytes = err;
		goto exit_err;
	}

	len = hw->remaining_bytes;

        /* 2. Enable the output clock (but don't modify the clock prescaler) */
	clkcfg = readl(hw->regs + S3C2443_SPI0_CLK_CFG);
	clkcfg |= S3C2443_SPI0_ENCLK_ENABLE;
	writel(clkcfg, hw->regs + S3C2443_SPI0_CLK_CFG);

	/*
	 * Configure the transfer mode depending on the number of bytes to send
	 * @XXX: For some unknown reasons we can't enable the burst mode. Please be
	 * aware then the burst mode for the FIFOs will work ONLY if the
	 * DMA-channel is configured for the burst mode too (so far so good).
	 * Luis Galdos
	 */
	modecfg = S3C2443_SPI0_MODE_CH_TSZ_BYTE;
	if (!(len & 0x3))
		modecfg = S3C2443_SPI0_MODE_CH_TSZ_WORD;

	/*
	 * Enable the DMA-modes depending on the transfer type
	 */
	if (xfer->tx_buf)
		modecfg |= S3C2443_SPI0_MODE_TXDMA_ON;
	if (xfer->rx_buf)
		modecfg	|= S3C2443_SPI0_MODE_RXDMA_ON;

        writel(modecfg, hw->regs + S3C2443_SPI0_MODE_CFG);

        /*
	 * 4. Set SPI INT_EN register
	 * Disable all the interrupts then we only use the DMA-controller for
	 * transferring the data from the RAM to the FIFO
	 */
        writel(0x00, hw->regs + S3C2443_SPI0_INT_EN);

	/* Clear the pending register */
        writel(0x1f, hw->regs + S3C2443_SPI0_PENDING_CLR);

        /*
         * If the passed SPI-buffer isn't DMA-mapped, then use the internal
         * DMA-buffers for sending the data to the DMA-layer. If we are using
	 * the internal buffer, then check that we can only send the maximal
	 * number of bytes. In the tasklet the transfer of the rest data can
	 * be triggered.
         */
        tx_dma = xfer->tx_dma;
        rx_dma = xfer->rx_dma;
        if (!msg->is_dma_mapped) {

		printk_debug("SPI-message not DMA mapped\n");

                rx_dma = hw->rx_dma;
                if (len > S3C2443_SPI_DMA_BUFFER)
                        len = S3C2443_SPI_DMA_BUFFER;

                if (xfer->tx_buf) {
                        tx_dma = hw->tx_dma;

                        if (len > S3C2443_SPI_DMA_BUFFER)
                                len = S3C2443_SPI_DMA_BUFFER;

			/* IMPORTANT: Copy the TX-data from the correct place! */
                        memcpy(hw->tx_buf,
                               xfer->tx_buf + hw->dma_xfer_len, len);
                }
        }

        /*
	 * 5. Set Packet Count configuration register
	 * For some unknown reasons the transfer of only ONE byte isn't
	 * working correctly. The data byte transferred from the DMA-controller
	 * isn't flushed from the TX-FIFO to the bus. But disabling the trailling
	 * count and reconfiguring the TX-FIFO helps to fix this problem.
	 * (Luis Galdos)
	 */
	spi_packet = len | S3C2443_SPI0_PACKET_CNT_EN;
	if (unlikely(len == 1)) {
		spi_packet = 0;
		s3c2443_spi_sw_reset(hw);
		s3c2443_spi_setupxfer(msg->spi, xfer);
	}
        writel(spi_packet, hw->regs + S3C2443_SPI0_PACKET_CNT);

	printk_debug("%s : %lu bytes | Rx %p | Tx %p\n", (xfer->tx_buf) ? "TX" : "RX",
		     len, xfer->rx_buf, xfer->tx_buf);

	s3c2443_spi_chipsel(msg->spi, SPI_CS_ACTIVE);

	/*
	 * Prepare the DMA-channel with the correct direction, and callback-function
	 * before sending the data to the DMA-subsystem
	 * If the TX-channel is being used, then only enable the TX-callback, otherwise
	 * use the RX-channel for the callback
	 */
	s3c2410_dma_set_buffdone_fn(SPI_TX_CHANNEL, NULL);
	s3c2410_dma_set_buffdone_fn(SPI_RX_CHANNEL, NULL);
	if (xfer->tx_buf)
		s3c2410_dma_set_buffdone_fn(SPI_TX_CHANNEL, s3c2443_spi_dma_callback);
	if (xfer->rx_buf)
		s3c2410_dma_set_buffdone_fn(SPI_RX_CHANNEL, s3c2443_spi_dma_callback);


	if (xfer->rx_buf) {
		s3c2443_spi_dma_init(hw->dma_ch_rx, S3C2443_DMA_RX, len);
		err = s3c2410_dma_enqueue(hw->dma_ch_rx, hw, rx_dma, len);
		if (err) {
			printk_err("Starting the RX DMA-channel, %i\n", err);
			s3c2410_dma_ctrl(hw->dma_ch_rx, S3C2410_DMAOP_FLUSH);
			hw->remaining_bytes = -EBUSY;
			goto exit_err;
		}
	}

	if (xfer->tx_buf) {
		s3c2443_spi_dma_init(hw->dma_ch, S3C2443_DMA_TX, len);
		err = s3c2410_dma_enqueue(hw->dma_ch, hw, tx_dma, len);
		if (err) {
			printk_err("Starting the TX DMA-channel, %i\n", err);
			hw->remaining_bytes = -EBUSY;
			goto exit_err;
		}
	}

	/* Init the FIFOs */
        chcfg = readl(hw->regs + S3C2443_SPI0_CH_CFG);
        chcfg &= ~(S3C2443_SPI0_CH_TXCH_ON | S3C2443_SPI0_CH_RXCH_ON);
        if (xfer->tx_buf)
                chcfg |= S3C2443_SPI0_CH_TXCH_ON;
        if (xfer->rx_buf)
                chcfg |= S3C2443_SPI0_CH_RXCH_ON;

        writel(chcfg, hw->regs + S3C2443_SPI0_CH_CFG);

	return;

	/* By errors only schedule the tasklet */
 exit_err:
	tasklet_schedule(&hw->xmit_tasklet);
}

/* Get the head of the message queue and start the xmit */
static void s3c2443_spi_next_message(struct spi_master *master)
{
	struct s3c2443_spi *hw;
	struct spi_message *msg;

	hw = master_to_hw(master);

	msg = list_entry(hw->xmit_queue.next, struct spi_message, queue);

	printk_debug("Starting to handle a new message %p\n", msg);

	/* And start to send the new message now */
	/* s3c2443_spi_chipsel(msg->spi, SPI_CS_ACTIVE); */
	s3c2443_spi_next_xfer(master, msg);
}

/*
 * This function will be called from the external SPI-layer for sending data
 * to the bus. It will return inmediately, if another message is being
 * transmitted, otherwise it will start sending the data to the bus
 */
static int s3c2443_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct s3c2443_spi *hw = spi_to_hw(spi);
	struct spi_transfer *xfer;

	if (unlikely(list_empty(&msg->transfers)))
		return -EINVAL;

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (!xfer->tx_buf && !xfer->rx_buf) {
			printk_err("Missing rx/tx buffer.\n");
			return -EINVAL;
		}
	}

	msg->status = -EINPROGRESS;
	msg->actual_length = 0;

	/*
	 * Add the new arrived message to the internal queue and start the
	 * transfer if the xmit-queue is IDLE
	 */
	spin_lock(&hw->xmit_lock);
	list_add_tail(&msg->queue, &hw->xmit_queue);
	if (!hw->current_xfer)
		s3c2443_spi_next_message(spi->master);
	spin_unlock(&hw->xmit_lock);

	return 0;
}

/*
 * Normally the IRQs are not used for transferring data to the FIFOs, we use only
 * the DMA-controller for this purpose.
 *
 * @FIXME: For some reasons we are not clearing all the interrupts correctly. With
 * the activated debug prints you will see what I'm talking about.
 * (Luis Galdos)
 */
static irqreturn_t s3c2443_spi_irq(int irq, void *_hw)
{
	struct s3c2443_spi *hw = _hw;
	unsigned int spsta = readl(hw->regs + S3C2443_SPI0_STATUS);

	spin_lock(&hw->xmit_lock);

	/* Disable all the interrupts and clear the pending errors */
	writel(0x00, hw->regs + S3C2443_SPI0_INT_EN);
	writel(spsta, hw->regs + S3C2443_SPI0_STATUS);
	writel(0x1f, hw->regs + S3C2443_SPI0_PENDING_CLR);

	printk_debug("IRQ stat 0x%08x\n", spsta);

	/*
	 * If we detect an error than force to the complete of the SPI-message by
	 * setting the number of remaining bytes to zero
	 */
	if (spsta &
	    (S3C2443_SPI0_STUS_RX_OVERRUN_ERR | S3C2443_SPI0_STUS_RX_UNDERRUN_ERR)) {
		hw->remaining_bytes = 0;
		printk_err("RX error (0x%08x)\n", spsta);
	}

	if (spsta &
	    (S3C2443_SPI0_STUS_TX_OVERRUN_ERR | S3C2443_SPI0_STUS_TX_UNDERRUN_ERR)) {
		hw->remaining_bytes = 0;
		printk_err("TX error (0x%08x)\n", spsta);
	}

	/* Force the complete of the current transfer */
	tasklet_schedule(&hw->xmit_tasklet);

	spin_unlock(&hw->xmit_lock);
	return IRQ_HANDLED;
}

static int __devinit s3c2443_spi_probe(struct platform_device *pdev)
{
	struct s3c2443_spi_info *pdata;
	struct s3c2443_spi *hw;
	struct spi_master *master;
	struct resource *res;
	int err = 0;

	printk_info("Probing device with the ID %i\n", pdev->id);

	master = spi_alloc_master(&pdev->dev, sizeof(struct s3c2443_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_exit;
	}

	hw = spi_master_get_devdata(master);
	memset(hw, 0, sizeof(struct s3c2443_spi));

	hw->master = spi_master_get(master);
	hw->pdata = pdev->dev.platform_data;
	hw->dev = &pdev->dev;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_put_spi;
	}

	/* Sanity checks for the passed platform data */
	hw->pdata = pdata;
	if (hw->pdata->num_chipselect > 1) {
		printk_err("Only supports one chip select (%i passed)\n",
			   hw->pdata->num_chipselect);
		goto err_put_spi;
	}

	platform_set_drvdata(pdev, hw);

	/* find and map our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk_err("Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_put_spi;
	}

	hw->ioarea = request_mem_region(res->start, (res->end - res->start)+1,
					pdev->name);
	if (hw->ioarea == NULL) {
		printk_err("Cannot reserve region\n");
		err = -ENXIO;
		goto err_put_spi;
	}

	hw->regs = ioremap(res->start, (res->end - res->start) + 1);
	if (hw->regs == NULL) {
		printk_err("Cannot map IO\n");
		err = -ENXIO;
		goto err_release_iomem;
	}

	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq <= 0) {
		printk_err("No IRQ specified? Aborting.\n");
		err = -ENOENT;
		goto err_unmap_iomem;
	}

	err = request_irq(hw->irq, s3c2443_spi_irq, 0, pdev->name, hw);
	if (err) {
		printk_err("Cannot claim IRQ\n");
		goto err_unmap_iomem;
	}

	/* Use the passed input clock */
	if (pdata->input_clk == S3C2443_HSSPI_INCLK_PCLK)
		hw->clk = clk_get(&pdev->dev, "pclk");
	else if (pdata->input_clk == S3C2443_HSSPI_INCLK_EPLL)
		hw->clk = clk_get(&pdev->dev, "epll");
	else {
		printk_err("Invalid input clock passed (%i)\n", pdata->input_clk);
		goto err_free_irq;
	}

	if (IS_ERR(hw->clk)) {
		printk_err("No clock for device\n");
		err = PTR_ERR(hw->clk);
		goto err_free_irq;
	}

	/* @FIXME: For the moment, permanently enable the clock */
	clk_enable(hw->clk);

	printk_info("Input clock frequency: %lu Hz\n", clk_get_rate(hw->clk));

	/* Init the internal data */
	spin_lock_init(&hw->xmit_lock);
	INIT_LIST_HEAD(&hw->xmit_queue);

	/* Create the DMA-pool for DMA-unmapped message */
	hw->tx_buf = dma_alloc_coherent(&pdev->dev, S3C2443_SPI_DMA_BUFFER,
					&hw->tx_dma, GFP_DMA);
	if (!hw->tx_buf) {
		printk_err("Couldn't get the TX DMA-memory\n");
		err = -ENOMEM;
		goto err_put_clk;
	}

	hw->rx_buf = dma_alloc_coherent(&pdev->dev, S3C2443_SPI_DMA_BUFFER,
					&hw->rx_dma, GFP_DMA);
	if (!hw->rx_buf) {
		printk_err("Couldn't get the RX DMA-memory\n");
		err = -ENOMEM;
		goto err_free_txdma;
	}

	/* We are safe, init the internal data for receiving the transfer requests */
	tasklet_init(&hw->xmit_tasklet, s3c2443_spi_next_tasklet,
		     (unsigned long)hw);

	/* Request the channel for the TX-transfers */
	hw->dmach.name = (char *)pdev->name;
	err = s3c2410_dma_request(SPI_TX_CHANNEL, &hw->dmach, hw);
	if (err < 0) {
		printk_err("TX DMA channel %i request failed\n.", SPI_TX_CHANNEL);
		goto err_free_rxdma;
	}
	hw->dma_ch = SPI_TX_CHANNEL;
	printk_debug("Got the TX DMA-channel %i (%i)\n", hw->dma_ch, SPI_TX_CHANNEL);

	/* Request the channel for the RX-transfers */
	hw->dmach_rx.name = (char *)pdev->name;
	err = s3c2410_dma_request(SPI_RX_CHANNEL, &hw->dmach_rx, hw);
	if (err < 0) {
		printk_err("RX DMA channel %i request failed\n.", SPI_RX_CHANNEL);
		goto err_free_chtx;
	}
	hw->dma_ch_rx = SPI_RX_CHANNEL;
	printk_debug("Got the RX DMA-channel %i (%i)\n", hw->dma_ch_rx, SPI_RX_CHANNEL);

	/* Set the callback function for the DMA-channel */
	s3c2410_dma_set_buffdone_fn(SPI_TX_CHANNEL, NULL);
	s3c2410_dma_set_opfn(SPI_TX_CHANNEL, NULL);
	s3c2410_dma_set_buffdone_fn(SPI_RX_CHANNEL, NULL);
	s3c2410_dma_set_opfn(SPI_RX_CHANNEL, NULL);

	/* Setup and register the SPI master */
	master->num_chipselect = pdata->num_chipselect;
	/* the spi->mode bits understood by this driver: */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST;
	master->setup          = s3c2443_spi_setup;
	master->transfer       = s3c2443_spi_transfer;
	master->bus_num	       = pdata->bus_num;
	err = spi_register_master(hw->master);
	if (err) {
		printk_err("Failed to register the SPI master\n");
		goto err_free_chrx;
	}

	/* Init the hardware (reset, enable clock, etc.) */
	s3c2443_spi_hw_init(hw);

	return 0;

 err_free_chrx:
	s3c2410_dma_free(SPI_RX_CHANNEL, &hw->dmach);

 err_free_chtx:
	s3c2410_dma_free(SPI_TX_CHANNEL, &hw->dmach);

 err_free_rxdma:
	dma_free_coherent(&pdev->dev, S3C2443_SPI_DMA_BUFFER, hw->rx_buf, hw->rx_dma);

 err_free_txdma:
	dma_free_coherent(&pdev->dev, S3C2443_SPI_DMA_BUFFER, hw->tx_buf, hw->tx_dma);

 err_put_clk:
	clk_disable(hw->clk);
	clk_put(hw->clk);

 err_free_irq:
	free_irq(hw->irq, hw);

 err_unmap_iomem:
	iounmap(hw->regs);

 err_release_iomem:
	release_resource(hw->ioarea);
	kfree(hw->ioarea);

 err_put_spi:
	spi_master_put(hw->master);;

 err_exit:
	platform_set_drvdata(pdev, NULL);
	return err;
}

static int __devexit s3c2443_spi_remove(struct platform_device *pdev)
{
	struct s3c2443_spi *hw = platform_get_drvdata(pdev);

	spi_unregister_master(hw->master);


	tasklet_kill(&hw->xmit_tasklet);

	clk_disable(hw->clk);
	clk_put(hw->clk);

	free_irq(hw->irq, hw);
	iounmap(hw->regs);

	release_resource(hw->ioarea);
	kfree(hw->ioarea);

	/* Free the allocated DMA-buffers */
	dma_free_coherent(&pdev->dev, S3C2443_SPI_DMA_BUFFER, hw->tx_buf, hw->tx_dma);
	dma_free_coherent(&pdev->dev, S3C2443_SPI_DMA_BUFFER, hw->rx_buf, hw->rx_dma);

	/* Free the DMA-channels */
	s3c2410_dma_free(SPI_TX_CHANNEL, &hw->dmach);
	s3c2410_dma_free(SPI_RX_CHANNEL, &hw->dmach_rx);

	/* Free the SPI-master */
	spi_master_put(hw->master);

	platform_set_drvdata(pdev, NULL);

	return 0;
}


#ifdef CONFIG_PM
static int s3c2443_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct s3c2443_spi *hw = platform_get_drvdata(pdev);

	clk_disable(hw->clk);
	return 0;
}

static int s3c2443_spi_resume(struct platform_device *pdev)
{
	struct s3c2443_spi *hw;
	int retval;

	hw = platform_get_drvdata(pdev);

        /* @FIXME: Why do we need to free the DMA-channels? */
	s3c2410_dma_free(SPI_TX_CHANNEL, &hw->dmach);
	s3c2410_dma_free(SPI_RX_CHANNEL, &hw->dmach_rx);

	retval = s3c2410_dma_request(SPI_TX_CHANNEL, &hw->dmach, hw);
	if (retval < 0)
		goto exit_resume;

	retval = s3c2410_dma_request(SPI_RX_CHANNEL, &hw->dmach_rx, hw);
	if (retval < 0)
		goto exit_free_tx;

	clk_enable(hw->clk);
	return 0;

exit_free_tx:
	s3c2410_dma_free(SPI_TX_CHANNEL, &hw->dmach);

exit_resume:
	return retval;
}
#else
#define s3c2443_spi_suspend NULL
#define s3c2443_spi_resume  NULL
#endif

MODULE_ALIAS("platform:s3c2443-spi");

static struct platform_driver s3c2443_spi_driver = {
	.probe		= s3c2443_spi_probe,
	.remove		= __devexit_p(s3c2443_spi_remove),
	.suspend	= s3c2443_spi_suspend,
	.resume		= s3c2443_spi_resume,
	.driver		= {
		.name	= "spi-s3c2443",
		.owner	= THIS_MODULE,
	},
};

static int __init s3c2443_spi_init(void)
{
        return platform_driver_register(&s3c2443_spi_driver);
}

static void __exit s3c2443_spi_exit(void)
{
        platform_driver_unregister(&s3c2443_spi_driver);
}

module_init(s3c2443_spi_init);
module_exit(s3c2443_spi_exit);

MODULE_DESCRIPTION("S3C2443 SPI Driver");
MODULE_AUTHOR("Luis Galdos, <luis.galdos@digi.com>");
MODULE_LICENSE("GPL");

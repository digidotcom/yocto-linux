/* -*- linux-c -*-
 *
 *  linux/drivers/mmc/s3c-hsmmc.c - Samsung S3C24XX HS-MMC driver
 *
 * $Id: s3c-hsmmc.c,v 1.18 2007/06/07 07:14:57 scsuh Exp $
 *
 *  Copyright (C) 2006 Samsung Electronics, All Rights Reserved.
 *  by Seung-Chull, Suh <sc.suh@samsung.com>
 *
 *  This driver is made for High Speed MMC interface. This interface
 *  is adopted and implemented since s3c2443 was made.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Modified by Ryu,Euiyoul <steven.ryu@samsung.com>
 *  Modified by Seung-chull, Suh to support s3c6400
 *  Modified by Luis Galdos, Added PM-support
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/highmem.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/err.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>

#include <asm/dma.h>
#include <asm/dma-mapping.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/scatterlist.h>
#include <asm/sizes.h>
//#include <linux/amba/mmci.h>

#include <mach/dma.h>
#include <plat/hsmmc.h>
#include <mach/regs-hsmmc.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>
#include <mach/gpio-fns.h>
#include <mach/gpio-nrs.h>


#define printk_err(fmt, args...)                printk(KERN_ERR "[ ERROR ] hsmmc: " fmt, ## args)
#define printk_info(fmt, args...)               printk(KERN_INFO "hsmmc: " fmt, ## args)

#if 0
#define CONFIG_S3CMCI_DEBUG
#endif

#ifdef CONFIG_S3CMCI_DEBUG
#  define printk_debug(fmt, args...)		printk(KERN_DEBUG "hsmmc: %s() " fmt, __FUNCTION__ , ## args)
#else
#  define printk_debug(fmt, args...)
#endif

/* Enables the support for the scatterlists */
#define CONFIG_HSMMC_PSEUDO_SCATTERGATHER       (1)

#define DBG(x, ...)

#include "s3c-hsmmc.h"

#define DRIVER_NAME				"s3c-hsmmc"
#define PFX					DRIVER_NAME ": "

#define RESSIZE(ressource) (((ressource)->end - (ressource)->start)+1)

#ifdef CONFIG_HSMMC_PROC_DATA
struct s3c_hsmmc_host *global_host[3];
#endif

/*****************************************************************************\
 *                                                                           *
 * Low level functions                                                       *
 *                                                                           *
\*****************************************************************************/

static struct s3c_hsmmc_cfg s3c_hsmmc_platform = {
	.hwport = 0,
	.host_caps = (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED),
	.base = NULL,
	.ctrl3[0] = 0x80800000,
	.ctrl3[1] = 0x80800000,
	.set_gpio = NULL,
};

/* s3c_hsmmc_get_platdata
 *
 * get the platform data associated with the given device, or return
 * the default if there is none
 */

static struct s3c_hsmmc_cfg *s3c_hsmmc_get_platdata (struct device *dev)
{
	if (dev->platform_data != NULL)
		return (struct s3c_hsmmc_cfg *)dev->platform_data;

	return &s3c_hsmmc_platform;
}

static void s3c_hsmmc_reset (struct s3c_hsmmc_host *host, u8 mask)
{
	unsigned long timeout;

	s3c_hsmmc_writeb(mask, S3C2410_HSMMC_SWRST);

	if (mask & S3C_HSMMC_RESET_ALL)
		host->clock = (uint)-1;

	/* Wait max 100 ms */
	timeout = 100;

	/* hw clears the bit when it's done */
	while (s3c_hsmmc_readb(S3C2410_HSMMC_SWRST) & mask) {
		if (timeout == 0) {
			printk("%s: Reset 0x%x never completed. \n",
				mmc_hostname(host->mmc), (int)mask);
			return;
		}
		timeout--;
		mdelay(1);
	}

}

static void s3c_hsmmc_ios_init (struct s3c_hsmmc_host *host)
{
	u32 intmask;

	s3c_hsmmc_reset(host, S3C_HSMMC_RESET_ALL);

	intmask = S3C_HSMMC_INT_BUS_POWER | S3C_HSMMC_INT_DATA_END_BIT |
		S3C_HSMMC_INT_DATA_CRC | S3C_HSMMC_INT_DATA_TIMEOUT | S3C_HSMMC_INT_INDEX |
		S3C_HSMMC_INT_END_BIT | S3C_HSMMC_INT_CRC | S3C_HSMMC_INT_TIMEOUT |
		S3C_HSMMC_INT_CARD_REMOVE | S3C_HSMMC_INT_CARD_INSERT |
		S3C_HSMMC_INT_DATA_AVAIL | S3C_HSMMC_INT_SPACE_AVAIL |
		S3C_HSMMC_INT_DATA_END | S3C_HSMMC_INT_RESPONSE;

#ifdef CONFIG_HSMMC_PSEUDO_SCATTERGATHER
	intmask |= S3C_HSMMC_INT_DMA_END;
#endif
	s3c_hsmmc_writel(intmask, S3C2410_HSMMC_NORINTSTSEN);
	s3c_hsmmc_writel(intmask, S3C2410_HSMMC_NORINTSIGEN);
}

/*****************************************************************************\
 *                                                                           *
 * Tasklets                                                                  *
 *                                                                           *
\*****************************************************************************/

static void s3c_hsmmc_tasklet_card (ulong param)
{
	struct s3c_hsmmc_host *host;
	unsigned long iflags;

	host = (struct s3c_hsmmc_host*)param;
	spin_lock_irqsave( &host->lock, iflags);

	if (!(s3c_hsmmc_readl(S3C2410_HSMMC_PRNSTS) & S3C_HSMMC_CARD_PRESENT)) {
		if (host->mrq) {
			printk(KERN_ERR "%s: Card removed during transfer!\n",
				mmc_hostname(host->mmc));
			printk(KERN_ERR "%s: Resetting controller.\n",
				mmc_hostname(host->mmc));

			s3c_hsmmc_reset(host, S3C_HSMMC_RESET_CMD);
			s3c_hsmmc_reset(host, S3C_HSMMC_RESET_DATA);

			host->mrq->cmd->error = -EILSEQ;
			tasklet_schedule(&host->finish_tasklet);
		}
	}

	spin_unlock_irqrestore( &host->lock, iflags);

	mmc_detect_change(host->mmc, msecs_to_jiffies(500));
}

static void s3c_hsmmc_activate_led(struct s3c_hsmmc_host *host)
{
	unsigned int ctrl;
	struct s3c_hsmmc_cfg *cfg = host->plat_data;

	if (cfg->gpio_led == S3C2410_GPJ(13)) {
		ctrl = s3c_hsmmc_readl(S3C2410_HSMMC_HOSTCTL);
		ctrl &= ~S3C_HSMMC_CTRL_LED;
		s3c_hsmmc_writel(ctrl, S3C2410_HSMMC_HOSTCTL);
	} else if (cfg->gpio_led)
		s3c2410_gpio_setpin(cfg->gpio_led, 1);


#if 0	// for 6400
	s3c_gpio_cfgpin(S3C_GPJ13, S3C_GPJ13_SD0LED);
#endif
}

static void s3c_hsmmc_deactivate_led(struct s3c_hsmmc_host *host)
{
	unsigned int ctrl;
	struct s3c_hsmmc_cfg *cfg = host->plat_data;
	
	if (cfg->gpio_led == S3C2410_GPJ(13)) {
		ctrl = s3c_hsmmc_readl(S3C2410_HSMMC_HOSTCTL);
		ctrl |= S3C_HSMMC_CTRL_LED;
		s3c_hsmmc_writel(ctrl, S3C2410_HSMMC_HOSTCTL);
	} else if (cfg->gpio_led)
		s3c2410_gpio_setpin(cfg->gpio_led, 0);


#if 0	// for 6400
	s3c_gpio_cfgpin(S3C_GPJ13, S3C_GPJ13_INP);
#endif
}

/*****************************************************************************\
 *                                                                           *
 * Core functions                                                            *
 *                                                                           *
\*****************************************************************************/

#ifdef CONFIG_HSMMC_PSEUDO_SCATTERGATHER
static inline uint s3c_hsmmc_build_dma_table (struct s3c_hsmmc_host *host,
						struct mmc_data *data)
{
	uint i, j = 0, sub_num = 0;
	dma_addr_t addr;
	uint size, length, end;
	int boundary, xor_bit;
	struct scatterlist * sg = data->sg;

	/* build dma table except last one */
	for (i=0; i<(host->sg_len-1); i++) {
		addr = sg[i].dma_address;
		size = sg[i].length;
		DBG("%d - addr: %08x, size: %08x\n", i, addr, size);

		for (; (j<CONFIG_S3C_HSMMC_MAX_HW_SEGS*4) && size; j++) {
			end = addr + size;
			xor_bit = min(7+(2+8+2), fls(addr^end) -1);

			DBG("%08x %08x %08x %d\n", addr, size, end, xor_bit);

			host->dblk[j].dma_address = addr;

			length = (end & ~((1<<xor_bit)-1)) - addr;
			boundary = xor_bit - (2+8+2);
			DBG("length: %x, boundary: %d\n", length, boundary);

			if (length < S3C_HSMMC_MALLOC_SIZE) {
				boundary = 0;

				if ((addr+length) & (S3C_HSMMC_MALLOC_SIZE-1)) {
					void *dest;

					DBG("#########error fixing: %08x, %x\n", addr, length);
					dest = host->sub_block[sub_num] + S3C_HSMMC_MALLOC_SIZE - length;
					if (data->flags & MMC_DATA_WRITE) { /* write */
						memcpy(dest, phys_to_virt(addr), length);
					}

					host->dblk[j].original = phys_to_virt(addr);
					host->dblk[j].dma_address = dma_map_single(NULL, dest, length, host->dma_dir);
					sub_num++;
				}
			}

			host->dblk[j].length = length;
			host->dblk[j].boundary = boundary;
			DBG("   %d: %08x, %08x %x\n",
						j, addr, length, boundary);
			addr += length;
			size -= length;
		}
	}

	/* the last one */
	host->dblk[j].dma_address = sg[i].dma_address;
	host->dblk[j].length = sg[i].length;
	host->dblk[j].boundary = 0x7;

 	return (j+1);
}
#endif

static inline void s3c_hsmmc_prepare_data(struct s3c_hsmmc_host *host,
					  struct mmc_command *cmd)
{
	u32 reg;
	struct mmc_data *data = cmd->data;

	/* If no data to send, only enable the CMD complete interrupt */
	if (data == NULL) {
		reg = s3c_hsmmc_readl(S3C2410_HSMMC_NORINTSTSEN);
		reg |= S3C_HSMMC_NIS_CMDCMP;
		s3c_hsmmc_writel(reg, S3C2410_HSMMC_NORINTSTSEN);
		return;
	}

#ifdef CONFIG_HSMMC_PROC_DATA
	{
		u32 total_size;
		int bit;

		total_size = data->blksz * data->blocks;
		bit = fls(total_size) - 13;
		if (bit < 0) bit = 0;
		if (data->flags & MMC_DATA_WRITE) {	/* write */
			host->tx_pkt[bit]++;
		} else {	/* read */
			host->rx_pkt[bit]++;
		}
	}
#endif
	
	reg = s3c_hsmmc_readl(S3C2410_HSMMC_NORINTSTSEN) & ~S3C_HSMMC_NIS_CMDCMP;
	reg |= S3C_HSMMC_NIS_TRSCMP;
	s3c_hsmmc_writel(reg, S3C2410_HSMMC_NORINTSTSEN);

	host->dma_dir = (data->flags & MMC_DATA_READ)
		? DMA_FROM_DEVICE : DMA_TO_DEVICE;


	
#ifdef CONFIG_HSMMC_PSEUDO_SCATTERGATHER
	host->sg_len = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
				  host->dma_dir);

	reg = s3c_hsmmc_readl(S3C2410_HSMMC_NORINTSTSEN);
	if (host->sg_len == 1) {
		reg &= ~S3C_HSMMC_NIS_DMA;
	} else {
		reg |= S3C_HSMMC_NIS_DMA;
	}
	s3c_hsmmc_writel(reg, S3C2410_HSMMC_NORINTSTSEN);

	DBG("data->sg_len: %d\n", data->sg_len);
	host->dma_blk = s3c_hsmmc_build_dma_table(host, data);
	host->next_blk = 0;
#else
	dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
		   host->dma_dir);
#endif
}

static inline void s3c_hsmmc_set_transfer_mode (struct s3c_hsmmc_host *host,
					struct mmc_data *data)
{
	u16 mode;

	mode = S3C_HSMMC_TRNS_DMA;

	if (data->stop)
		mode |= S3C_HSMMC_TRNS_ACMD12;

	/* Set the configuration for the number of blocks to transfer */
	if (data->blocks > 1)
		mode |= (S3C_HSMMC_TRNS_MULTI | S3C_HSMMC_TRNS_BLK_CNT_EN);

	/* Set the transfer direction */
	if (data->flags & MMC_DATA_READ)
		mode |= S3C_HSMMC_TRNS_READ;

	printk_debug("Setting the transfer mode to 0x%08x\n", mode);
	s3c_hsmmc_writew(mode, S3C2410_HSMMC_TRNMOD);
}

static inline void s3c_hsmmc_send_register (struct s3c_hsmmc_host *host)
{
	struct mmc_command *cmd = host->cmd;
	struct mmc_data *data = cmd->data;

	u32 cmd_val;

	if (data) {
#ifdef CONFIG_HSMMC_PSEUDO_SCATTERGATHER
		struct s3c_hsmmc_dma_blk *dblk;

		dblk = &host->dblk[0];

		s3c_hsmmc_writew(S3C_HSMMC_MAKE_BLKSZ(dblk->boundary, data->blksz),
				 S3C2410_HSMMC_BLKSIZE);
		
		s3c_hsmmc_writel(dblk->dma_address, S3C2410_HSMMC_SYSAD);
#else
		s3c_hsmmc_writew(S3C_HSMMC_MAKE_BLKSZ(0x7, data->blksz),
				 S3C2410_HSMMC_BLKSIZE);
		
		s3c_hsmmc_writel(sg_dma_address(data->sg), S3C2410_HSMMC_SYSAD);
#endif
		
		s3c_hsmmc_writew(data->blocks, S3C2410_HSMMC_BLKCNT);
		s3c_hsmmc_set_transfer_mode(host, data);
	}

	s3c_hsmmc_writel(cmd->arg, S3C2410_HSMMC_ARGUMENT);

	cmd_val = (cmd->opcode << 8);
	if (cmd_val == (12<<8))
		cmd_val |= (3 << 6);

	if (cmd->flags & MMC_RSP_136)		/* Long RSP */
		cmd_val |= S3C_HSMMC_CMD_RESP_LONG;
	else if (cmd->flags & MMC_RSP_BUSY)	/* R1B */
		cmd_val |= S3C_HSMMC_CMD_RESP_SHORT_BUSY;
	else if (cmd->flags & MMC_RSP_PRESENT)	/* Normal RSP */
		cmd_val |= S3C_HSMMC_CMD_RESP_SHORT;

	if (cmd->flags & MMC_RSP_OPCODE)
		cmd_val |= S3C_HSMMC_CMD_INDEX;

	if (cmd->flags & MMC_RSP_CRC)
		cmd_val |= S3C_HSMMC_CMD_CRC;

	if (data)
		cmd_val |= S3C_HSMMC_CMD_DATA;

	s3c_hsmmc_writew(cmd_val, S3C2410_HSMMC_CMDREG);
}


/* Send a command to the HSMMC-controller */
static inline void s3c_hsmmc_send_command(struct s3c_hsmmc_host *host,
					  struct mmc_command *cmd)
{
	u32 mask;
	ulong timeout;

	/* Wait max 10 ms */
	timeout = 10;

	mask = S3C_HSMMC_CMD_INHIBIT;
	if ((cmd->data != NULL) || (cmd->flags & MMC_RSP_BUSY))
		mask |= S3C_HSMMC_DATA_INHIBIT;

	while (s3c_hsmmc_readl(S3C2410_HSMMC_PRNSTS) & mask) {
		if (timeout == 0) {
			printk_err("%s: Controller never released "
				   "inhibit bit(s).\n", mmc_hostname(host->mmc));
			cmd->error = -EILSEQ;
			tasklet_schedule(&host->finish_tasklet);
			return;
		}
		timeout--;
		mdelay(1);
	}

	mod_timer(&host->timer, jiffies + 4 * HZ);

	host->cmd = cmd;

	s3c_hsmmc_prepare_data(host, cmd);
	s3c_hsmmc_send_register(host);
}


static void s3c_hsmmc_finish_data(struct s3c_hsmmc_host *host)
{
	struct mmc_data *data;
	u16 blocks;

	BUG_ON(!host->data);

	data = host->data;
	host->data = NULL;

	dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
			(data->flags & MMC_DATA_READ)
			? DMA_FROM_DEVICE : DMA_TO_DEVICE);

	/*
	 * Controller doesn't count down when in single block mode.
	 */
	if ((data->blocks == 1) && (data->error == 0))
		blocks = 0;
	else {
		blocks = s3c_hsmmc_readw(S3C2410_HSMMC_BLKCNT);
	}
	data->bytes_xfered = data->blksz * (data->blocks - blocks);

	if ((data->error == 0) && blocks) {
		printk_err("%s: Controller signalled completion even "
			   "though there were blocks left. : %d\n",
			   mmc_hostname(host->mmc), blocks);
		data->error = -EILSEQ;
	}

	printk_debug("Ending data transfer (%d bytes)\n", data->bytes_xfered);

	tasklet_schedule(&host->finish_tasklet);
}

static void s3c_hsmmc_finish_command (struct s3c_hsmmc_host *host)
{
	int i;
	unsigned int resp;

	BUG_ON(host->cmd == NULL);

	if (host->cmd->flags & MMC_RSP_PRESENT) {
		if (host->cmd->flags & MMC_RSP_136) {
			/* CRC is stripped so we need to do some shifting. */
			for (i = 0; i < 4; i++) {
				resp = s3c_hsmmc_readl(S3C2410_HSMMC_RSPREG0 +
						       (3 - i) * 4);
				host->cmd->resp[i] = resp << 8;
				if (i != 3) {
					resp = s3c_hsmmc_readb(S3C2410_HSMMC_RSPREG0 +
							       ((3 - i) * 4) - 1);
					host->cmd->resp[i] |= resp;
				}
			}
		} else {
			resp = s3c_hsmmc_readl(S3C2410_HSMMC_RSPREG0);
			printk_debug("Got a RSP: 0x%08x\n", resp);
			host->cmd->resp[0] = resp;
		}
	}

	host->cmd->error = 0;

	printk_debug("Ending cmd (%u)\n", host->cmd->opcode);

	if (host->cmd->data)
		host->data = host->cmd->data;
	else
		tasklet_schedule(&host->finish_tasklet);

	host->cmd = NULL;
}

static void s3c_hsmmc_tasklet_finish (unsigned long param)
{
	struct s3c_hsmmc_host *host;
	unsigned long iflags;
	struct mmc_request *mrq;

	host = (struct s3c_hsmmc_host*)param;

	spin_lock_irqsave(&host->lock, iflags);

	del_timer(&host->timer);

	mrq = host->mrq;

	/*
	 * The controller needs a reset of internal state machines
	 * upon error conditions.
	 */
	if ((mrq->cmd->error != 0) ||
		(mrq->data && (mrq->data->error != 0)) ) {
		s3c_hsmmc_reset(host, S3C_HSMMC_RESET_CMD);
		s3c_hsmmc_reset(host, S3C_HSMMC_RESET_DATA);
	}

	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;

	s3c_hsmmc_deactivate_led(host);

	mmiowb();
	spin_unlock_irqrestore(&host->lock, iflags);

	mmc_request_done(host->mmc, mrq);
}

/*****************************************************************************
 *                                                                           *
 * Interrupt handling                                                        *
 *                                                                           *
 *****************************************************************************/

static void s3c_hsmmc_cmd_irq (struct s3c_hsmmc_host *host, u32 intmask)
{
	if (intmask & S3C_HSMMC_INT_TIMEOUT)
		host->cmd->error = -ETIMEDOUT;
	else if (intmask & S3C_HSMMC_INT_CRC)
		host->cmd->error = -EILSEQ;
	else if (intmask & (S3C_HSMMC_INT_END_BIT | S3C_HSMMC_INT_INDEX))
		host->cmd->error = -EILSEQ;
	else
		host->cmd->error = -EILSEQ;

	tasklet_schedule(&host->finish_tasklet);
}

static void s3c_hsmmc_data_irq (struct s3c_hsmmc_host *host, u32 intmask)
{
	if (!host->data) {
		/*
		 * A data end interrupt is sent together with the response
		 * for the stop command.
		 */
		if (intmask & S3C_HSMMC_INT_DATA_END)
			return;

		printk_err("%s: Got data interrupt even though no "
			   "data operation was in progress.\n",
			   mmc_hostname(host->mmc));
		return;
	}

	if (intmask & S3C_HSMMC_INT_DATA_TIMEOUT)
		host->data->error = -ETIMEDOUT;
	else if (intmask & S3C_HSMMC_INT_DATA_CRC)
		host->data->error = -EILSEQ;
	else if (intmask & S3C_HSMMC_INT_DATA_END_BIT)
		host->data->error = -EILSEQ;

	if (host->data->error != 0)
		s3c_hsmmc_finish_data(host);
}


/*****************************************************************************\
 *                                                                           *
 * Interrupt handling                                                        *
 *                                                                           *
\*****************************************************************************/

/*
 * ISR for SDI Interface IRQ
 * Communication between driver and ISR works as follows:
 *   host->mrq 			points to current request
 *   host->complete_what	tells the ISR when the request is considered done
 *     COMPLETION_CMDSENT	  when the command was sent
 *     COMPLETION_RSPFIN          when a response was received
 *     COMPLETION_XFERFINISH	  when the data transfer is finished
 *     COMPLETION_XFERFINISH_RSPFIN both of the above.
 *   host->complete_request	is the completion-object the driver waits for
 *
 * 1) Driver sets up host->mrq and host->complete_what
 * 2) Driver prepares the transfer
 * 3) Driver enables interrupts
 * 4) Driver starts transfer
 * 5) Driver waits for host->complete_rquest
 * 6) ISR checks for request status (errors and success)
 * 6) ISR sets host->mrq->cmd->error and host->mrq->data->error
 * 7) ISR completes host->complete_request
 * 8) ISR disables interrupts
 * 9) Driver wakes up and takes care of the request
 */

static irqreturn_t s3c_hsmmc_irq (int irq, void *dev_id)
{
	irqreturn_t result = 0;
	struct s3c_hsmmc_host *host = dev_id;
	struct mmc_request *mrq;
	u32 intsts;
#ifdef CONFIG_HSMMC_S3C_IRQ_WORKAROUND
	uint i, org_irq_sts;
#endif
	spin_lock(&host->lock);

	mrq = host->mrq;

	intsts = s3c_hsmmc_readw(S3C2410_HSMMC_NORINTSTS);

	/* Sometimes, hsmmc does not update its status bit immediately
	 * when it generates irqs. by scsuh.
	 */
#ifdef CONFIG_HSMMC_S3C_IRQ_WORKAROUND
	for (i=0; i<0x1000; i++) {
		if ((intsts = s3c_hsmmc_readw(S3C2410_HSMMC_NORINTSTS)))
			break;
	}
#endif

	if (unlikely(!intsts)) {
		result = IRQ_NONE;
		goto out;
	}
	intsts = s3c_hsmmc_readl(S3C2410_HSMMC_NORINTSTS);
#ifdef CONFIG_HSMMC_S3C_IRQ_WORKAROUND
	org_irq_sts = intsts;
#endif

	printk_debug("IRQ : Status 0x%08x\n", intsts);
#if 0
	printk(PFX "got interrupt = 0x%08x\n", intsts);
	printk(PFX "got mask = 0x%08x\n", s3c_hsmmc_readl(S3C2410_HSMMC_NORINTSTSEN));
	printk(PFX "got signal = 0x%08x\n", s3c_hsmmc_readl(S3C2410_HSMMC_NORINTSIGEN));
#endif

	if (unlikely(intsts & S3C_HSMMC_INT_CARD_CHANGE)) {
		u32 reg16;

		if (intsts & S3C_HSMMC_INT_CARD_INSERT)
			printk_debug("card inserted.\n");
		else if (intsts & S3C_HSMMC_INT_CARD_REMOVE)
			printk_debug("card removed.\n");

		reg16 = s3c_hsmmc_readw(S3C2410_HSMMC_NORINTSTSEN);
		s3c_hsmmc_writew(reg16 & ~S3C_HSMMC_INT_CARD_CHANGE,
							S3C2410_HSMMC_NORINTSTSEN);
		s3c_hsmmc_writew(S3C_HSMMC_INT_CARD_CHANGE, S3C2410_HSMMC_NORINTSTS);
		s3c_hsmmc_writew(reg16, S3C2410_HSMMC_NORINTSTSEN);

		intsts &= ~S3C_HSMMC_INT_CARD_CHANGE;

		tasklet_schedule(&host->card_tasklet);
		goto insert;
	}

	if (likely(!(intsts & S3C_HSMMC_NIS_ERR))) {
		s3c_hsmmc_writel(intsts, S3C2410_HSMMC_NORINTSTS);

		if (intsts & S3C_HSMMC_NIS_CMDCMP) {
			printk_debug("command done\n");
			s3c_hsmmc_finish_command(host);
		}

		if (intsts & S3C_HSMMC_NIS_TRSCMP) {
			printk_debug("transfer done\n\n");
			s3c_hsmmc_finish_command(host);
			s3c_hsmmc_finish_data(host);
			intsts &= ~S3C_HSMMC_NIS_DMA;
		}

#ifdef CONFIG_HSMMC_PSEUDO_SCATTERGATHER
		if (intsts & S3C_HSMMC_NIS_DMA) {
			struct s3c_hsmmc_dma_blk *dblk;

			dblk = &host->dblk[host->next_blk];
			if (dblk->original) {
				/* on read */
				if (host->dma_dir == DMA_FROM_DEVICE) {
					memcpy(dblk->original,
					       phys_to_virt(dblk->dma_address),
					       dblk->length);
				}
				dma_unmap_single(NULL, dblk->dma_address, dblk->length,
						 host->dma_dir);
				dblk->original = 0;
			}

			host->next_blk++;
			dblk = &host->dblk[host->next_blk];

			if (host->next_blk == (host->dma_blk-1)) {
				u32 reg;

				reg = s3c_hsmmc_readl(S3C2410_HSMMC_NORINTSTSEN);
				reg &= ~S3C_HSMMC_NIS_DMA;
				s3c_hsmmc_writel(reg, S3C2410_HSMMC_NORINTSTSEN);
			}

			/* We do not handle DMA boundaries, so set it to max (512 KiB) */
			s3c_hsmmc_writew((dblk->boundary<<12 | 0x200),
					 S3C2410_HSMMC_BLKSIZE);
			s3c_hsmmc_writel(dblk->dma_address, S3C2410_HSMMC_SYSAD);
		}
#endif
	} else {
		unsigned int interr;
		interr = s3c_hsmmc_readl(S3C2410_HSMMC_ERRINTSTS) & 0xffff;

		/* Timeout errors are printed in the tasklet, or? (Luis Galdos) */
		if (interr != S3C_HSMMC_EIS_CMDTIMEOUT)
			printk_err("Error detected : 0x%04x\n", interr);
		
		if (intsts & S3C_HSMMC_INT_CMD_MASK) {
			s3c_hsmmc_writel(intsts & S3C_HSMMC_INT_CMD_MASK,
					 S3C2410_HSMMC_NORINTSTS);
			s3c_hsmmc_cmd_irq(host, intsts & S3C_HSMMC_INT_CMD_MASK);
		}

		if (intsts & S3C_HSMMC_INT_DATA_MASK) {
			s3c_hsmmc_writel(intsts & S3C_HSMMC_INT_DATA_MASK,
					 S3C2410_HSMMC_NORINTSTS);
			s3c_hsmmc_finish_command(host);
			s3c_hsmmc_data_irq(host, intsts & S3C_HSMMC_INT_DATA_MASK);
		}

		intsts &= ~(S3C_HSMMC_INT_CMD_MASK | S3C_HSMMC_INT_DATA_MASK);
	}

	/* XXX: fix later by scsuh */
#if 0
	if (intsts & S3C_HSMMC_INT_BUS_POWER) {
		printk_err("%s: Card is consuming too much power!\n",
			mmc_hostname(host->mmc));
		s3c_hsmmc_writel(S3C_HSMMC_INT_BUS_POWER, S3C2410_HSMMC_NORINTSTS);
	}

	intsts &= S3C_HSMMC_INT_BUS_POWER;

	if (intsts) {
		printk_err("%s: Unexpected interrupt 0x%08x.\n",
			mmc_hostname(host->mmc), intsts);

		s3c_hsmmc_writel(intsts, S3C2410_HSMMC_NORINTSTS);
	}
#endif

#ifdef CONFIG_HSMMC_S3C_IRQ_WORKAROUND
	for (i=0; i<0x1000; i++) {
		if (org_irq_sts != s3c_hsmmc_readl(S3C2410_HSMMC_NORINTSTS))
			break;
	}
#endif

insert:
	result = IRQ_HANDLED;
	mmiowb();

out:
	spin_unlock(&host->lock);

	return result;
}

static void s3c_hsmmc_check_status (unsigned long data)
{
        struct s3c_hsmmc_host *host = (struct s3c_hsmmc_host *)data;

	s3c_hsmmc_irq(0, host);
}

static void s3c_hsmmc_request (struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct s3c_hsmmc_host *host = mmc_priv(mmc);
	unsigned long flags;

	printk_debug("[CMD%d] arg:0x%08x flags:0x%02x retries:%u\n",
		     mrq->cmd->opcode, mrq->cmd->arg,
		     mrq->cmd->flags, mrq->cmd->retries);

	spin_lock_irqsave(&host->lock, flags);

	WARN_ON(host->mrq != NULL);

	s3c_hsmmc_activate_led(host);

	host->mrq = mrq;

	if (likely(s3c_hsmmc_readl(S3C2410_HSMMC_PRNSTS) & S3C_HSMMC_CARD_PRESENT)) {
		s3c_hsmmc_send_command(host, mrq->cmd);
	} else {
		printk_debug("No card present? Aborting a command request.\n");
		host->mrq->cmd->error = -ENOMEDIUM;
		tasklet_schedule(&host->finish_tasklet);
	}

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

/* return 0: OK
 * return -1: error
 */
static int set_bus_width (struct s3c_hsmmc_host *host, uint width)
{
	u8 reg;

	reg = s3c_hsmmc_readb(S3C2410_HSMMC_HOSTCTL);

	switch (width) {
	case MMC_BUS_WIDTH_1:
		reg &= ~(S3C_HSMMC_CTRL_4BIT | S3C_HSMMC_CTRL_8BIT);
		break;
	case MMC_BUS_WIDTH_4:
		reg |= S3C_HSMMC_CTRL_4BIT;
		break;
	case MMC_BUS_WIDTH_8:
		reg |= S3C_HSMMC_CTRL_8BIT;
		break;
	default:
		printk_err("Invalid bus width %u\n", width);
		return -EINVAL;
	}

	s3c_hsmmc_writeb(reg, S3C2410_HSMMC_HOSTCTL);

	printk_debug("HOSTCTL 0x%02x\n", s3c_hsmmc_readb(S3C2410_HSMMC_HOSTCTL));

	return 0;
}

static void hsmmc_set_clock (struct s3c_hsmmc_host *host, ulong clock)
{
	uint i, j;
	u32 val = 0, tmp_clk = 0, clk_src = 0;
	ulong timeout;
	u16 div = -1;
	u8 ctrl;

	/* if we already set, just out. */
	if (clock == host->clock) {
		printk("%p:host->clock0 : %d\n", host->base, host->clock);
		return;
	}

	/* before setting clock, clkcon must be disabled. */
	s3c_hsmmc_writew(0, S3C2410_HSMMC_CLKCON);

	s3c_hsmmc_writeb(S3C_HSMMC_TIMEOUT_MAX, S3C2410_HSMMC_TIMEOUTCON);

	/* change the edge type according to frequency */
	ctrl = s3c_hsmmc_readb(S3C2410_HSMMC_HOSTCTL);
	if (clock > 25000000)
		ctrl |= S3C_HSMMC_CTRL_HIGHSPEED;
	else
		ctrl &= ~S3C_HSMMC_CTRL_HIGHSPEED;
	s3c_hsmmc_writeb(ctrl, S3C2410_HSMMC_HOSTCTL);

	if (clock == 0) {
		return;
	}

	/* calculate optimal clock. by scsuh */
	for (i=0; i < 3; i++) {
		tmp_clk = clk_get_rate(host->clk[i]);
		if (tmp_clk <= clock) {
			if (tmp_clk >= val) {
				val = tmp_clk;
				div = 0;
				clk_src = i+1;
			}
		}

		for (j=0x1; j<=0x80; j <<= 1) {
			tmp_clk = clk_get_rate(host->clk[i]) / (j<<1);
			if ((val < tmp_clk) && (tmp_clk <= clock)) {
				val = tmp_clk;
				div = j;
				clk_src = i+1;
				break;
			}
		}
	}

	/* clk_src = 0x00; */
	printk_debug("val: %d, div: %x, clk_src: %d\n", val, div, clk_src);

	s3c_hsmmc_writel(0x0000c100 | (clk_src << 4), S3C2410_HSMMC_CONTROL2);
	if (clock > 25000000)
		/* 0x00008080 6400, 0x00800080 2443 */
		s3c_hsmmc_writel(host->ctrl3[1], S3C2410_HSMMC_CONTROL3);
	else
		/* 0x80808080 6400, 0x00800080 2443 */
		s3c_hsmmc_writel(host->ctrl3[0], S3C2410_HSMMC_CONTROL3);

	s3c_hsmmc_writew(((div<<8) | S3C_HSMMC_CLOCK_INT_EN), S3C2410_HSMMC_CLKCON);

	timeout = 10;
	while (!((val = s3c_hsmmc_readw(S3C2410_HSMMC_CLKCON))
				& S3C_HSMMC_CLOCK_INT_STABLE)) {
		if (!timeout) {
			printk_err("Clock stabilization: %08x | Div %u\n", val, div);
			return;
		}
		timeout--;
		mdelay(1);
	}

	s3c_hsmmc_writew(val | S3C_HSMMC_CLOCK_CARD_EN, S3C2410_HSMMC_CLKCON);
	timeout = 10;
	while (!((val = s3c_hsmmc_readw(S3C2410_HSMMC_CLKCON))
				& S3C_HSMMC_CLOCK_EXT_STABLE)) {
		if (!timeout) {
			printk_err("Clock stabilization: %08x | Div %u\n", val, div);
			return;
		}
		timeout--;
		mdelay(1);
	}

//out:
//	host->clock = clock;
//	printk("%p: host->clock1 : %d\n", host->base, host->clock);
}


static void s3c_hsmmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct s3c_hsmmc_host *host = mmc_priv(mmc);
	struct s3c_hsmmc_cfg *cfg = host->plat_data;

	unsigned long iflags;

	spin_lock_irqsave(&host->lock, iflags);


	switch (ios->power_mode) {
	case MMC_POWER_ON:
	case MMC_POWER_UP:
		s3c2410_gpio_cfgpin(S3C2410_GPL(0), S3C2443_GPL0_SD0DAT0);

		if (cfg->host_caps & (MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA)) {
			s3c2410_gpio_cfgpin(S3C2410_GPL(1), S3C2443_GPL1_SD0DAT1);
			s3c2410_gpio_cfgpin(S3C2410_GPL(2), S3C2443_GPL2_SD0DAT2);
			s3c2410_gpio_cfgpin(S3C2410_GPL(3), S3C2443_GPL3_SD0DAT3);
		}

		if (cfg->host_caps & MMC_CAP_8_BIT_DATA) {
			s3c2410_gpio_cfgpin(S3C2410_GPL(4), S3C2443_GPL4_SD0DAT4);
			s3c2410_gpio_cfgpin(S3C2410_GPL(5), S3C2443_GPL5_SD0DAT5);
			s3c2410_gpio_cfgpin(S3C2410_GPL(6), S3C2443_GPL6_SD0DAT6);
			s3c2410_gpio_cfgpin(S3C2410_GPL(7), S3C2443_GPL7_SD0DAT7);
		}

		s3c2410_gpio_cfgpin(S3C2410_GPL(8), S3C2443_GPL8_SD0CMD);
		s3c2410_gpio_cfgpin(S3C2410_GPL(9), S3C2443_GPL9_SD0CLK);

		/* Check for the dedicated GPIOs of the HSMMC-controller */
		if (cfg->gpio_led == S3C2410_GPJ(13))
			s3c2410_gpio_cfgpin(S3C2410_GPJ(13), S3C2440_GPJ13_SD0LED);
		
		if (cfg->gpio_detect == S3C2410_GPJ(14))
			s3c2410_gpio_cfgpin(S3C2410_GPJ(14), S3C2440_GPJ14_SD0CD);

		if (cfg->gpio_wprotect == S3C2410_GPJ(15))
			s3c2410_gpio_cfgpin(S3C2410_GPJ(15), S3C2440_GPJ15_SD0WP);

		break;
	default:
		break;
	}

	/*
	 * Reset the chip on each power off.
	 * Should clear out any weird states.
	 */
	if (ios->power_mode == MMC_POWER_OFF) {
		s3c_hsmmc_writew(0, S3C2410_HSMMC_NORINTSIGEN);
		s3c_hsmmc_ios_init(host);
	}

	if (host->plat_data->set_gpio)
		host->plat_data->set_gpio();

	printk_debug("Clock: %d | Bus width %d\n", ios->clock, ios->bus_width);
	hsmmc_set_clock(host, ios->clock);
	set_bus_width(host, ios->bus_width);

	printk_debug("CTRL2 0x%08x | CTRL3 0x%08x | CLKCON 0x%04x\n",
		     s3c_hsmmc_readl(S3C2410_HSMMC_CONTROL2),
		     s3c_hsmmc_readl(S3C2410_HSMMC_CONTROL3),
		     s3c_hsmmc_readw(S3C2410_HSMMC_CLKCON));

	if (ios->power_mode == MMC_POWER_OFF)
		s3c_hsmmc_writeb(S3C_HSMMC_POWER_OFF, S3C2410_HSMMC_PWRCON);
	else
		s3c_hsmmc_writeb(S3C_HSMMC_POWER_ON_ALL, S3C2410_HSMMC_PWRCON);

	udelay(1000);
	spin_unlock_irqrestore(&host->lock, iflags);
}



/*
 * If no GPIO was configured for the write protect, then assume that the card
 * is NOT write protected
 */
static int s3c_hsmmc_get_ro(struct mmc_host *mmc)
{
	struct s3c_hsmmc_host *host = mmc_priv(mmc);
	struct s3c_hsmmc_cfg *cfg = host->plat_data;
	unsigned int retval;
	
	/* Depending on the configured GPIO get the RO-value */
	if (cfg->gpio_wprotect == S3C2410_GPJ(15)) {
		retval = s3c_hsmmc_readl(S3C2410_HSMMC_PRNSTS);
		retval &= S3C_HSMMC_WRITE_PROTECT; 
	} else if (cfg->gpio_wprotect) {
		retval = s3c2410_gpio_getpin(cfg->gpio_wprotect);
		retval = cfg->wprotect_invert ? !retval : retval;
	} else
		retval = 0;
			
	return retval;
}


static struct mmc_host_ops s3c_hsmmc_ops = {
	.request	= s3c_hsmmc_request,
	.set_ios	= s3c_hsmmc_set_ios,
	.get_ro		= s3c_hsmmc_get_ro,
};



static int s3c_hsmmc_probe (struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct s3c_hsmmc_host *host;
	struct s3c_hsmmc_cfg *plat_data;
	uint i;
	int ret;

	mmc = mmc_alloc_host(sizeof(struct s3c_hsmmc_host), &pdev->dev);
	if (!mmc)
		ret = -ENOMEM;

	plat_data = s3c_hsmmc_get_platdata(&pdev->dev);

	host = mmc_priv(mmc);

	host->mmc = mmc;
	host->plat_data = plat_data;
	platform_set_drvdata(pdev, mmc);

	host->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!host->mem) {
		printk_err("Failed to get io memory region resouce.\n");
		ret = -ENOENT;
		goto probe_free_host;
	}

	host->mem = request_mem_region(host->mem->start,
				       RESSIZE(host->mem), pdev->name);
	if (!host->mem) {
		printk_err("Failed to request io memory region.\n");
		ret = -ENOENT;
		goto probe_free_host;
	}

	host->base = ioremap(host->mem->start, RESSIZE(host->mem));
	if (!host->base) {
		printk_err("Failed to ioremap() the region %p\n", host->mem);
		ret = -EINVAL;
		goto err_free_mem_region;
	}
	
	host->irq = platform_get_irq(pdev, 0);
	if (host->irq == 0) {
		printk("failed to get interrupt resouce.\n");
		ret = -EINVAL;
		goto untasklet;
	}

	host->flags |= S3C_HSMMC_USE_DMA;

	/* every platform has different ctrl3 value. by scsuh */
	host->ctrl3[0] = plat_data->ctrl3[0];
	host->ctrl3[1] = plat_data->ctrl3[1];

#ifdef CONFIG_HSMMC_PSEUDO_SCATTERGATHER
	for (i=0; i<S3C_HSMMC_MAX_SUB_BUF; i++)
		host->sub_block[i] = kmalloc(S3C_HSMMC_MALLOC_SIZE, GFP_KERNEL);
#endif

	s3c_hsmmc_reset(host, S3C_HSMMC_RESET_ALL);

#ifdef CONFIG_ARCH_S3C6400
	clk_enable(clk_get(&pdev->dev, "sclk_48m"));
#endif

	/* register 3 clock source if exist */
	for (i=0; i<3; i++) {
		host->clk[i] = clk_get(&pdev->dev, plat_data->clk_name[i]);
		if (IS_ERR(host->clk[i])) {
			ret = PTR_ERR(host->clk[i]);
			host->clk[i] = ERR_PTR(-ENOENT);
		}

		if (clk_enable(host->clk[i])) {
			host->clk[i] = ERR_PTR(-ENOENT);
		}
	}

	mmc->ops = &s3c_hsmmc_ops;
	mmc->ocr_avail = MMC_VDD_32_33|MMC_VDD_33_34;
	mmc->f_min = 400 * 1000; /* at least 400kHz */

	/* you must make sure that our hsmmc block can support
	 * up to 52MHz. by scsuh
	 */
	mmc->f_max = 100 * MHZ;
	mmc->caps = plat_data->host_caps;
	printk_debug("mmc->caps: %08x\n", (unsigned int)mmc->caps);

	spin_lock_init(&host->lock);

	/*
	 * Maximum number of segments. Hardware cannot do scatter lists.
	 * XXX: must modify later. by scsuh
	 */
#ifdef CONFIG_HSMMC_PSEUDO_SCATTERGATHER
	mmc->max_hw_segs = CONFIG_S3C_HSMMC_MAX_HW_SEGS;
	mmc->max_phys_segs = CONFIG_S3C_HSMMC_MAX_HW_SEGS;
#else
	mmc->max_hw_segs = 1;
#endif

	/*
	 * Maximum number of sectors in one transfer. Limited by DMA boundary
	 * size (512KiB), which means (512 KiB/512=) 1024 entries.
	 */
	mmc->max_blk_count = 1024;

	/*
	 * Maximum segment size. Could be one segment with the maximum number
	 * of sectors.
	 */
	mmc->max_seg_size = mmc->max_blk_count * 512;
	mmc->max_req_size = mmc->max_blk_count * 512;

	init_timer(&host->timer);
        host->timer.data = (unsigned long)host;
        host->timer.function = s3c_hsmmc_check_status;
        host->timer.expires = jiffies + HZ;

	/*
	 * Init tasklets.
	 */
	tasklet_init(&host->card_tasklet,
		s3c_hsmmc_tasklet_card, (unsigned long)host);
	tasklet_init(&host->finish_tasklet,
		s3c_hsmmc_tasklet_finish, (unsigned long)host);

	ret = request_irq(host->irq, s3c_hsmmc_irq, 0, DRIVER_NAME, host);
	if (ret)
		goto untasklet;

	s3c_hsmmc_ios_init(host);

	mmc_add_host(mmc);

#ifdef CONFIG_HSMMC_PROC_DATA
	global_host[plat_data->hwport] = host;
#endif

	printk(KERN_INFO "%s.%d: at 0x%p with irq %d. clk src:",
			pdev->name, pdev->id, host->base, host->irq);
	for (i=0; i<3; i++) {
		if (!IS_ERR(host->clk[i]))
			printk(" %s", plat_data->clk_name[i]);
	}
	printk("\n");

	return 0;

 untasklet:
	tasklet_kill(&host->card_tasklet);
	tasklet_kill(&host->finish_tasklet);

	for (i=0; i<3; i++) {
		clk_disable(host->clk[i]);
		clk_put(host->clk[i]);
	}

 err_free_mem_region:
	release_mem_region(host->mem->start, RESSIZE(host->mem));
	
 probe_free_host:
	mmc_free_host(mmc);
	platform_set_drvdata(pdev, NULL);
	
	return ret;
}

static int s3c_hsmmc_remove(struct platform_device *dev)
{
	struct mmc_host *mmc  = platform_get_drvdata(dev);
	struct s3c_hsmmc_host *host = mmc_priv(mmc);
	int i;

	printk_debug("Removing the MMC %p | S3C host %p\n", mmc, host);
	
	mmc_remove_host(mmc);

	/* Reset the MMC controller */
	s3c_hsmmc_reset(host, S3C_HSMMC_RESET_ALL);

#ifdef CONFIG_ARCH_S3C6400
	clk_disable(clk_get(&dev->dev, "sclk_48m"));
#endif

	/* Free only the requested clocks */
	for (i =0; i< 3; i++) {
		if (!PTR_ERR(host->clk[i])) {
			clk_disable(host->clk[i]);
			clk_put(host->clk[i]);
		}
	}

	/* Free the IRQ and mapped memory */
	free_irq(host->irq, host);
	iounmap(host->base);
	release_mem_region(host->mem->start, RESSIZE(host->mem));
	
	del_timer_sync(&host->timer);

	tasklet_kill(&host->card_tasklet);
	tasklet_kill(&host->finish_tasklet);

#ifdef CONFIG_HSMMC_PSEUDO_SCATTERGATHER
	kfree(host->sub_block);
#endif
	mmc_free_host(mmc);

	return 0;
}

/* This is for the power management support */
#ifdef CONFIG_PM
static int s3c_hsmmc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mmc_host *mmc;
	struct s3c_hsmmc_host *host;
	int cnt, retval;

	mmc = platform_get_drvdata(pdev);
	host = mmc_priv(mmc);

	/* Check for all the three available clocks to disable */
	for (cnt = 0; cnt < 3; cnt++) {
		if (!PTR_ERR(host->clk[cnt]))
			clk_disable(host->clk[cnt]);
	}

	retval = mmc_suspend_host(mmc);

	return retval;
}

static int s3c_hsmmc_resume(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct s3c_hsmmc_host *host;
	int cnt, retval;

	mmc = platform_get_drvdata(pdev);
	host = mmc_priv(mmc);
	
	/* Reenable the clocks first */
	for (cnt = 0; cnt < 3; cnt++) {
		if (!PTR_ERR(host->clk[cnt]))
			clk_enable(host->clk[cnt]);
	}

	s3c_hsmmc_ios_init(host);

	 /*
	  * By unsafe resumes we MUST check the card state at this point, then the
	  * higher MMC-layer is probably transferring some kind of data to the
	  * block device that doesn't exist any more.
	  */
#if defined(CONFIG_MMC_UNSAFE_RESUME)
	if (s3c_hsmmc_readl(S3C2410_HSMMC_PRNSTS) & S3C_HSMMC_CARD_PRESENT)
		retval = mmc_resume_host(mmc);
	else {
		retval = 0;
		mmc_detect_change(mmc, msecs_to_jiffies(500));
	}
#else
	retval = mmc_resume_host(mmc);	
#endif /* CONFIG_MMC_UNSAFE_RESUME */
	
	return retval;
}
#else
#define s3c_hsmmc_suspend				NULL
#define s3c_hsmmc_resume				NULL
#endif

static struct platform_driver s3c_hsmmc_driver = {
	.probe          = s3c_hsmmc_probe,
	.remove         = s3c_hsmmc_remove,
	.suspend	= s3c_hsmmc_suspend,
	.resume		= s3c_hsmmc_resume,
	.driver		= {
		.name	= "s3c-hsmmc",
		.owner	= THIS_MODULE,
	},
};

static int __init s3c_hsmmc_drv_init(void)
{
	return platform_driver_register(&s3c_hsmmc_driver);
}

static void __exit s3c_hsmmc_drv_exit(void)
{
	platform_driver_unregister(&s3c_hsmmc_driver);
}

#ifdef CONFIG_HSMMC_PROC_DATA
static struct proc_dir_entry *evb_resource_dump;


static int mem_proc_read (char *buffer, char **buffer_location, off_t offset,
			  int buffer_length, int *zero, void *ptr)
{
	return 0;
}


static int mem_proc_write(struct file *file, const char *buffer,
			  unsigned long count, void *data)
{
	uint i, range = 0;
	char flag = 'l';
	uint port = 0;

#if 0
	printk("buffers: %s\n", buffer);
#endif
	sscanf(buffer, "%d %c", &port, &flag);

	switch (flag) {
	case 'c':
		printk("clean %d port\n", port);
		for (i=0; i<7; i++) {
			global_host[port]->rx_pkt[i] = 0;
			global_host[port]->tx_pkt[i] = 0;
		}
//		break;

	case 'l':
		printk("\t\t\trx\ttx\n");
		for (i=0; i<7; i++) {
			printk("0x%05x ~ 0x%05x\t%-7d\t%-7d\n",
			       range, 0x1000 << i,
			       global_host[port]->rx_pkt[i],
			       global_host[port]->tx_pkt[i]);
			range = 0x1000 << i;

		}
		break;
	}

	return count;
}

int __init mem_proc_scsuh (void)
{
	evb_resource_dump = create_proc_entry("mmc", 0666, &proc_root);
	evb_resource_dump->read_proc = mem_proc_read;
	evb_resource_dump->write_proc = mem_proc_write;
	evb_resource_dump->nlink = 1;
	return 0;
}

module_init(mem_proc_scsuh);
#endif

module_init(s3c_hsmmc_drv_init);
module_exit(s3c_hsmmc_drv_exit);


MODULE_DESCRIPTION("S3C SD HOST I/F 1.0 Driver");
MODULE_LICENSE("GPL");

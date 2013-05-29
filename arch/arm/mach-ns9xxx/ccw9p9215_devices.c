/*
 * arch/arm/mach-ns9xxx/cc9p9215_devices.c
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <mach/fim-ns921x.h>
#include <mach/hardware.h>

#include <mach/regs-sys-common.h>
#include <mach/regs-sys-ns921x.h>
#include <mach/regs-mem.h>

#include <asm/leds.h>

#include "pipermain.h"
#include "mac.h"
#include "airoha.h"

/*
 * Pick Digi's internal FIM board
 * Use internal board, defined to 1
 * Use newer boards, defined to 0
 */
#if defined(CONFIG_DIGI_PIPER_WIFI)

/* Low level functions to access piper chip */
static u32 read_reg(struct piper_priv *piperp, u8 reg)
{
	return ioread32(piperp->vbase + reg);
}

static int write_reg(struct piper_priv *piperp, u8 reg, u32 val, reg_op_t op)
{
	unsigned long flags;

	spin_lock_irqsave(&piperp->ac->reg_lock, flags);
	switch (op) {
	case op_write:
		iowrite32(val, piperp->vbase + reg);
		break;
	case op_or:
		iowrite32(val | ioread32(piperp->vbase + reg), piperp->vbase + reg);
		break;
	case op_and:
		iowrite32(val & ioread32(piperp->vbase + reg), piperp->vbase + reg);
		break;
	default:
		printk(KERN_WARNING PIPER_DRIVER_NAME
		       ": Invalid write register operation (%d)\n", op);
		WARN_ON(1);
		break;
	}
	spin_unlock_irqrestore(&piperp->ac->reg_lock, flags);

	return 0;
}

/*
 * This macro waits for the AES busy bit to clear if we are writing to the
 * AES FIFO.
 */
#define wait_for_aes_ready()	while ((addr == BB_AES_FIFO)			\
				      && ((ioread32(piperp->vbase + BB_RSSI)	\
					    & BB_RSSI_EAS_FIFO_FULL) != 0)) {	\
					udelay(1);				\
				}

static int write_fifo(struct piper_priv *piperp, u8 addr, u8 *buf, int len)
{
	int wordIndex;
	int wordLength = len / sizeof(unsigned int);
	unsigned long flags;
	bool loadingBeacon = (addr == BEACON_FIFO);

	spin_lock_irqsave(&piperp->ac->reg_lock, flags);

	if (loadingBeacon) {
		/*
		 * If we are loading a new beacon, then adjust the address to point
		 * to the data FIFO, and set the beacon enable bit which tells piper
		 * to put this data into the beacon buffer.
		 */
		addr = BB_DATA_FIFO;
		iowrite32(ioread32(piperp->vbase + BB_GENERAL_CTL) | BB_GENERAL_CTL_BEACON_EN,
			  piperp->vbase + BB_GENERAL_CTL);
	}

	if (((unsigned)(buf) & 0x3) == 0) {
		/*
		 * We come here if the data is 32-bit aligned.  We can dispense
		 * with memcpys
		 */
		if (wordLength == 1) {
			/*
			 * Only 1 word of data, so just one write.
			 */
			unsigned int *word = (unsigned int *)buf;

			wait_for_aes_ready();
			iowrite32(cpu_to_be32(*word), piperp->vbase + addr);
			len -= 4;
		} else {
			/*
			 * More than one word of data, so set up a for loop.
			 */
			for (wordIndex = 0; wordIndex < wordLength; wordIndex++) {
				unsigned int *word = (unsigned int *)buf;

				wait_for_aes_ready();
				iowrite32(cpu_to_be32(word[wordIndex]), piperp->vbase + addr);
				len -= 4;
			}
		}
	} else {
		/*
		 * Ugh!  Data is not 32-bit aligned.  We have to memcpy it!
		 */
		for (wordIndex = 0; wordIndex < wordLength; wordIndex++) {
			unsigned int word;

			memcpy(&word, &buf[wordIndex * sizeof(unsigned int)],
			       sizeof(unsigned int));

			wait_for_aes_ready();
			iowrite32(cpu_to_be32(word), piperp->vbase + addr);
			len -= 4;
		}
	}

	if (len) {
		/*
		 * Double Ugh!  There was left over data at the end.  We have to write
		 * the leftover data into the upper bytes of the last word, making
		 * sure the unused bytes are set to zero.
		 */
		unsigned int word;

		memcpy(&word, &buf[wordLength * sizeof(unsigned int)], sizeof(unsigned int));
		word = cpu_to_be32(word);
		switch (len) {
		case 1:
			word &= 0xff000000;
			break;
		case 2:
			word &= 0xffff0000;
			break;
		case 3:
			word &= 0xffffff00;
			break;
		default:
			printk(KERN_WARNING PIPER_DRIVER_NAME
			       ": len = %d at end of piper_write\n", len);
			break;
		}
		wait_for_aes_ready();
		iowrite32(word, piperp->vbase + addr);
	}

	if (loadingBeacon) {
		/*
		 * If we just loaded a beacon, then don't forget to turn off the
		 * load beacon bit.
		 */
		iowrite32(ioread32(piperp->vbase + BB_GENERAL_CTL) & ~BB_GENERAL_CTL_BEACON_EN,
			  piperp->vbase + BB_GENERAL_CTL);
		piperp->beacon.loaded = true;
	}

	spin_unlock_irqrestore(&piperp->ac->reg_lock, flags);

	return 0;
}

/*
 * This routine waits for the empty flag to clear when we are reading from
 * the AES FIFO.
 */
#define wait_for_aes_not_empty()							\
	while ((addr == BB_AES_FIFO)							\
	      && ((ioread32(piperp->vbase + BB_RSSI) & BB_RSSI_EAS_FIFO_EMPTY) != 0)) {	\
		udelay(1);								\
		if (--timeout == 0) {							\
			timeout = 10000;						\
		}									\
	}

static int read_fifo(struct piper_priv *piperp, u8 addr, u8 *buf, int len)
{
	int wordIndex;
	unsigned long flags;
	int timeout = 10000;

	spin_lock_irqsave(&piperp->ac->reg_lock, flags);

	/*
	 * We can only read 32-bit words, so round the length up to an even multiple of
	 * 4 if necessary.
	 */
	len += 3;
	len &= 0xfffffc;

	if ((len == 4) && ((((unsigned)buf) & 0x3) == 0)) {
		unsigned *word = (unsigned *)buf;

		wait_for_aes_not_empty();
		*word = be32_to_cpu(ioread32(piperp->vbase + addr));
	} else if ((((unsigned)buf) & 0x3) == 0) {
		unsigned *word = (unsigned *)buf;

		for (wordIndex = 0; wordIndex < (len / sizeof(unsigned)); wordIndex++) {
			wait_for_aes_not_empty();
			word[wordIndex] = be32_to_cpu(ioread32(piperp->vbase + addr));
		}
	} else {
		/*
		 * If we come here, then the buffer is not aligned and we have to
		 * memcpy the data.
		 */
		for (wordIndex = 0; wordIndex < (len / sizeof(unsigned)); wordIndex++) {
			unsigned word;

			wait_for_aes_not_empty();
			word = be32_to_cpu(ioread32(piperp->vbase + addr));
			memcpy(&buf[wordIndex * sizeof(unsigned)], &word, sizeof(word));
		}
	}
	spin_unlock_irqrestore(&piperp->ac->reg_lock, flags);

	return 0;
}

/* Initialize piper hardware, mac and dsp firmwares and mac address */
static int piper_init_chip_hw(struct piper_priv *piperp)
{
	int ret;

	piper_load_mac_firmware(piperp);
	piper_load_dsp_firmware(piperp);

	ret = piper_spike_suppression(piperp, true);
	if (ret) {
		printk(KERN_WARNING PIPER_DRIVER_NAME
		       ": spike suppresion error\n");
		return ret;
	}
	piper_reset_mac(piperp);
	piper_set_macaddr(piperp);

	return ret;
}

static void ccw9p9215_piper_set_led(struct piper_priv *piperp, enum wireless_led led, int val)
{
	if(led == STATUS_LED)
		leds_event(val ? led_green_on : led_green_off);
}

static void ccw9p9215_piper_reset(struct piper_priv *piperp, int reset)
{
	gpio_set_value(piperp->pdata->rst_gpio, !reset);
}

static int ccw9p9215_piper_init(struct piper_priv *piperp)
{
	ccw9p9215_piper_reset(piperp, 1);
	mdelay(1);
	ccw9p9215_piper_reset(piperp, 0);
	mdelay(1);

	/* Initialize functions to access register */
	piperp->ac->wr_reg = write_reg;
	piperp->ac->rd_reg = read_reg;
	piperp->ac->wr_fifo = write_fifo;
	piperp->ac->rd_fifo = read_fifo;

	mdelay(1);

	return piper_init_chip_hw(piperp);
}

static int ccw9p9215_piper_late_init(struct piper_priv *piperp)
{
	/* Configure irq gpio line */
	gpio_configure_ns921x(piperp->pdata->irq_gpio, NS921X_GPIO_INPUT,
			      NS921X_GPIO_DONT_INVERT, NS921X_GPIO_FUNC_2,
			      NS921X_GPIO_ENABLE_PULLUP);

	return 0;
}

static struct resource piper_resources[] = {
	{
		.start = 0x70000000,
		.end   = 0x70000000 + 0x100,
		.flags = IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS9XXX_EXT0,
		.flags	= IORESOURCE_IRQ,
	}
};

/* describes the device */
static struct platform_device piper_device = {
	.id		= 0,
	.name		= PIPER_DRIVER_NAME,
	.num_resources	= ARRAY_SIZE(piper_resources),
	.resource	= piper_resources,
};

void __init ns9xxx_add_device_ccw9p9215_wifi(struct piper_pdata *pdata)
{
	int ret;

	if (!pdata)
		return;

	if (pdata->rst_gpio >= 0) {
		ret = gpio_request(pdata->rst_gpio, PIPER_DRIVER_NAME "-reset-gpio");
		if (ret != 0)
			printk(KERN_WARNING PIPER_DRIVER_NAME
			       ": failed to request reset gpio %d\n", pdata->rst_gpio);
		else {
			/* Configure reset line and hold the chip in reset */
			gpio_direction_output(pdata->rst_gpio, 0);
			pdata->reset = ccw9p9215_piper_reset;
		}
	}

	if (pdata->irq_gpio >= 0) {
		ret = gpio_request(pdata->irq_gpio, PIPER_DRIVER_NAME "-irq-gpio");
		if (ret != 0)
			printk(KERN_WARNING PIPER_DRIVER_NAME
			       ": failed to request irq gpio %d\n", pdata->irq_gpio);
	}

	/* Configure the memory controller (CS3) with the appropriate settings */
	/* 32 bit bus width */
	writel(MEM_SMC_PB_1 | MEM_SMC_MW_32, MEM_SMC(3));
	/* Static Memory Write Enable Delay x */
	writel(0, MEM_SMWED(3));
	/* Static Memory Output Enable Delay x */
	writel(2, MEM_SMOED(3));
	/* Static Memory Read Delay x */
	writel(8, MEM_SMRD(3));
	/* Static Memory Page Mode Read Delay 0 */
	writel(0, MEM_SMPMRD(3));
	/* Static Memory Write Delay */
	writel(4, MEM_SMWD(3));
	/* Static Memory Turn Round Delay x */
	writel(2, MEM_SWT(3));
	/* Enable the CS0 access */
	writel(readl(SYS_SMCSSMM(3)) | SYS_SMCSSMM_CSEx_EN, SYS_SMCSSMM(3));

	pdata->rf_transceiver = RF_AIROHA_7230;
	pdata->init = ccw9p9215_piper_init;
	pdata->late_init = ccw9p9215_piper_late_init;
	pdata->set_led = ccw9p9215_piper_set_led;
	piper_device.dev.platform_data = pdata;

	platform_device_register(&piper_device);
}

#else
void __init ns9xxx_add_device_ccw9p9215_wifi(struct piper_pdata *pdata) {}
#endif


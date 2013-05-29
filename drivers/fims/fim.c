/* -*- linux-c -*-
 *
 * drivers/fim.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 *  !Revision:   $Revision: 1.45 $
 *  !Author:     Silvano Najera, Luis Galdos
 *  !Descr:
 *  !References:
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/blkdev.h>
#include <linux/spinlock.h>
#include <linux/moduleparam.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <asm/signal.h>
#include <linux/kfifo.h>
#include <linux/sysfs.h>

/* Arch-dependent header files */
#include <mach/regs-sys-ns921x.h>
#include <mach/regs-iohub-ns921x.h>
#include <mach/irqs.h>

/* Include the structure for the FIM-firmware */
#include <mach/fim-ns921x.h>
#include <mach/fim-firmware.h>

#include "fim_reg.h"
#include "dma.h"

#define DRIVER_VERSION			"v0.2"
#define DRIVER_AUTHOR			"Silvano Najera, Luis Galdos"
#define DRIVER_DESC			"FIMs driver"

/* Module information */
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

#define DRIVER_NAME			"fim"
#define FIM_DRIVER_NAME			"fims"
#define FIM_BUS_TYPE_NAME		"fim-bus"

#define IRQ_NS921X_PIC			IRQ_NS921X_PIC0
#define	FIM0_SHIFT			6

#if 0
#define FIM_CORE_DEBUG
#endif

#define printk_err(fmt, args...)	printk(KERN_ERR "[ ERROR ] fims: " fmt, ## args)
#define printk_info(fmt, args...)	printk(KERN_INFO "fims: " fmt, ## args)
#define printk_warn(fmt, args...)	printk(KERN_DEBUG "fims: " fmt, ## args)

#ifdef FIM_CORE_DEBUG
#  define printk_debug(fmt, args...)	printk(KERN_DEBUG "fims: %s() " fmt, __FUNCTION__ , ## args)
#else
#  define printk_debug(fmt, args...)
#endif

#define PIC_IS_LOCKED(pic)		(atomic_read(&pic->requested))
#define PIC_LOCK_REQUEST(pic)		do { atomic_set(&pic->requested, 1); } while (0)
#define PIC_FREE_REQUEST(pic)		do { atomic_set(&pic->requested, 0); } while (0)

#define PIC_TX_LOCK(pic)		do { atomic_set(&pic->tx_tasked, 1); } while (0)
#define PIC_TX_FREE(pic)		do { atomic_set(&pic->tx_tasked, 0); } while (0)
#define PIC_TX_IS_FREE(pic)		(!atomic_read(&pic->tx_tasked))

/* Use the same function for all the binary attributes */
#define FIM_SYSFS_ATTR(_name) \
{ \
    .attr =  { \
	    .name = __stringify(_name), \
	    .mode = S_IRUGO, \
	    .owner = THIS_MODULE, \
    }, \
    .read =  fim_sysfs_attr_read, \
}

struct fims_t {
	struct pic_t pics[FIM_NR_PICS];
	struct bus_type *bus_type;
	struct device *bus_dev;
	struct device_driver *driver;
};

static struct fims_t *the_fims;

static int pic_config_output_clock_divisor(struct pic_t *pic,
					   struct fim_program_t *program);
static int pic_send_interrupt(struct pic_t *pic, unsigned int code);

static int pic_download_firmware(struct pic_t *pic, const unsigned char *buffer);
static int pic_stop_and_reset(struct pic_t *pic);
static int pic_start_at_zero(struct pic_t *pic);

static int fim_remove(struct device *dev);
static int fim_probe(struct device *dev);
static irqreturn_t pic_irq(int irq, void *tpic);
static int pic_dma_init(struct pic_t *pic, struct fim_dma_cfg_t *cfg);
static void pic_dma_stop(struct pic_t *pic);
static int pic_is_running(struct pic_t *pic);

/* Interrupt handlers */
static void isr_from_pic(struct pic_t *pic, int irqnr);
static void isr_dma_tx(struct pic_t *pic, int irqnr);
static void isr_dma_rx(struct pic_t *pic, int irqnr);
static int fim_start_tx_dma(struct pic_t *pic);

/*
 * Be aware with this function, we have max. one PAGE available for writing into
 * the return buffer
 * When the module is unregistered it's NOT required to reset the attributes
 */
static ssize_t fim_sysfs_attr_read(struct file *filp, struct kobject *kobj,
				   struct bin_attribute *attr, char *buffer,
				   loff_t off, size_t count)
{
	int size, cnt;
	struct device *dev;
	struct pic_t *pic;
	struct attribute *bin;
	struct iohub_dma_desc_t *desc;
	unsigned int ctrlreg;

	dev = container_of(kobj, struct device, kobj);
	bin = &attr->attr;
	pic = dev_get_drvdata(dev);
	if (!pic)
		return 0;

	/* Check for the correct attribute */
	if (!strcmp(bin->name, "irq"))
		size = snprintf(buffer, PAGE_SIZE, "%i", pic->irq);

	else if (pic->driver && !strcmp(bin->name, "module"))
		size = snprintf(buffer, PAGE_SIZE, "%s", pic->driver->driver.name);

	else if (!strcmp(bin->name, "running"))
		size = snprintf(buffer, PAGE_SIZE, "%i", pic_is_running(pic));

	else if (pic->driver && !strcmp(bin->name, "version"))
		size = snprintf(buffer, PAGE_SIZE, "@TODO: Version of the firmware");

	else if (pic->driver && !strcmp(bin->name, "fwname"))
		size = snprintf(buffer, PAGE_SIZE, "%s", pic->fw_name);

	else if (pic->driver && !strcmp(bin->name, "fwlength"))
		size = snprintf(buffer, PAGE_SIZE, "%i", pic->fw_length);

	else if (pic->driver && !strcmp(bin->name, "txdma")) {
		ctrlreg = readl(pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);
		size = snprintf(buffer, PAGE_SIZE, "CTRL 0x%08x\n", ctrlreg);
		ctrlreg = readl(pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);
		for (cnt=0; cnt < pic->tx_fifo.length; cnt++) {
			desc = fim_dma_get_by_index(&pic->tx_fifo, cnt);
			size += snprintf(buffer + size,
					PAGE_SIZE - size,
					"[ TX %02i ] 0x%04x | %i\n",
					cnt, desc->control, desc->length);
		}
	}

	else if (pic->driver && !strcmp(bin->name, "rxdma")) {
		ctrlreg = readl(pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);
		size = snprintf(buffer, PAGE_SIZE, "CTRL 0x%08x\n", ctrlreg);
		for (cnt=0; cnt < pic->rx_fifo.length; cnt++) {
			desc = fim_dma_get_by_index(&pic->rx_fifo, cnt);
			size += snprintf(buffer + size,
					PAGE_SIZE - size,
					"[ RX %02i ] 0x%04x | %i\n",
					cnt, desc->control, desc->length);
		}
	}

	else
		size = 0;

	/* Only allows a single read for this binary attributes */
	if (off || count < size)
		return 0;

	return size;
}

/* These are the non-default binary attributes for the FIMs */
static struct bin_attribute fim_sysfs_attrs[] = {
        FIM_SYSFS_ATTR(irq),
	FIM_SYSFS_ATTR(running),
	FIM_SYSFS_ATTR(module),
	FIM_SYSFS_ATTR(version),
	FIM_SYSFS_ATTR(fwname),
	FIM_SYSFS_ATTR(fwlength),
	FIM_SYSFS_ATTR(txdma),
	FIM_SYSFS_ATTR(rxdma),
};

/*
 * This tasklet will search for full receive DMA-buffers
 * Only if the PIC has an associated driver the corresponding callback will be started
 */
static void pic_rx_tasklet_func(unsigned long data)
{
	struct pic_t *pic;
	struct iohub_dma_desc_t *desc;
	struct fim_driver *driver;
	struct fim_buffer_t buffer;
	struct iohub_dma_fifo_t *fifo;

	/* If no driver is attached, then the DMA-channel is normally disabled */
	pic = (struct pic_t *)data;
	if (!pic || !pic->driver)
		return;

	driver = pic->driver;
	fifo = &pic->rx_fifo;

	/* The pointer to DMA-first will be updated in the locked segment! */
	spin_lock(&pic->rx_lock);
	desc = fifo->dma_first;

	/*
	 * If a driver is waiting for the data then create a callback buffer,
	 * otherwise only restore the DMA-buffer descriptor
	 */
	while (desc->control & IOHUB_DMA_DESC_CTRL_FULL) {
		/*
		 * If the driver doesn't have a RX-callback then only free
		 * the DMA-descriptors
		 */
		if (driver->dma_rx_isr) {
			buffer.data = phys_to_virt(desc->src);
			buffer.length = desc->length;
			pic->driver->dma_rx_isr(driver, pic->irq, &buffer);
		}

		/* And free the DMA-descriptor for a next transfer */
		desc->length = pic->dma_cfg.rxsz;
		desc->control &= ~(IOHUB_DMA_DESC_CTRL_FULL |
				   IOHUB_DMA_DESC_CTRL_LAST);
		desc = fim_dma_get_next(fifo, desc);
	}

	/* Update the buffer index for the next tasklet */
	fifo->dma_first = desc;

	/* Check for previously detected errors */
	if (fifo->rx_error) {

		if (pic->driver->dma_error_isr)
			pic->driver->dma_error_isr(pic->driver, fifo->rx_error, 0);

		/* fim_dma_reset_fifo(fifo); */
		fifo->rx_error = 0;
	}

	spin_unlock(&pic->rx_lock);
}

inline static struct pic_t *get_pic_by_index(int index)
{
	if (index < FIM_MIN_PIC_INDEX || index > FIM_MAX_PIC_INDEX)
		return NULL;


	return &the_fims->pics[index];
}

static struct pic_t *get_pic_from_driver(struct fim_driver *driver)
{
	if (!driver)
		return NULL;

	return get_pic_by_index(driver->picnr);
}

int fim_dump_registers(struct fim_driver *driver)
{
	struct pic_t *pic;

	if (!(pic = get_pic_from_driver(driver))) {
		printk(KERN_ERR "fim_driver structure has not been initialized\n");
		return -ENODEV;
	}

	printk(KERN_ERR "IOHUB_IFS_REG = 0x%8.8X\n", readl(pic->iohub_addr + IOHUB_IFS_REG));
	printk(KERN_ERR "IOHUB_RX_DMA_CTRL_REG = 0x%8.8X\n", readl(pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG));
	printk(KERN_ERR "IOHUB_RX_DMA_BUFPTR_REG = 0x%8.8X\n", readl(pic->iohub_addr + IOHUB_RX_DMA_BUFPTR_REG));
	printk(KERN_ERR "IOHUB_RX_DMA_ICTRL_REG = 0x%8.8X\n", readl(pic->iohub_addr + IOHUB_RX_DMA_ICTRL_REG));
	printk(KERN_ERR "IOHUB_RX_DIR_REG = 0x%8.8X\n", readl(pic->iohub_addr + IOHUB_RX_DIR_REG));
	printk(KERN_ERR "IOHUB_RX_DIR_FIFO_REG = 0x%8.8X\n", readl(pic->iohub_addr + IOHUB_RX_DIR_FIFO_REG));
	printk(KERN_ERR "IOHUB_TX_DMA_CTRL_REG = 0x%8.8X\n", readl(pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG));
	printk(KERN_ERR "IOHUB_TX_DMA_BUFPTR_REG = 0x%8.8X\n", readl(pic->iohub_addr + IOHUB_TX_DMA_BUFPTR_REG));
	printk(KERN_ERR "IOHUB_TX_ICTRL_REG = 0x%8.8X\n", readl(pic->iohub_addr + IOHUB_TX_ICTRL_REG));
	printk(KERN_ERR "IOHUB_TX_FIFO_REG = 0x%8.8X\n", readl(pic->iohub_addr + IOHUB_TX_FIFO_REG));
	printk(KERN_ERR "IOHUB_TX_DIR_REG = 0x%8.8X\n", readl(pic->iohub_addr + IOHUB_TX_DIR_REG));

	return 0;
}

static int pic_get_ctrl_reg(struct pic_t *pic, int reg, unsigned int *val)
{
	if (NS92XX_FIM_CTRL_REG_CHECK(reg))
		return -EINVAL;

	*val = readl(pic->reg_addr + NS92XX_FIM_CTRL_REG(reg));
	return 0;
}

static void pic_set_ctrl_reg(struct pic_t *pic, int reg, unsigned int val)
{
	if (NS92XX_FIM_CTRL_REG_CHECK(reg))
		return;

	writel(val, pic->reg_addr + NS92XX_FIM_CTRL_REG(reg));
}

static int pic_is_running(struct pic_t *pic)
{
	unsigned int regval;

	regval = readl(pic->reg_addr + NS92XX_FIM_GEN_CTRL_REG);
	if (regval & NS92XX_FIM_GEN_CTRL_PROGMEM)
		return 1;
	else
		return 0;
}

int fim_is_running(struct fim_driver *driver)
{
	struct pic_t *pic;

	if (!(pic = get_pic_from_driver(driver)))
		return -ENODEV;

	return pic_is_running(pic);
}

static void fim_pic_release(struct device *dev)
{
	/* @TODO: Nothing to do here? */
}

/*
 * Returns the number of available bytes that can be theoretically requested
 * for a TX-transfer with more than one buffer descriptor
 */
int fim_tx_buffers_room(struct fim_driver *driver)
{
	int retval, cnt;
	struct pic_dma_desc_t *pic_desc;
	struct pic_t *pic;

	if (!(pic = get_pic_by_index(driver->picnr)))
		return -ENODEV;

	retval = 0;
	for (cnt=0; cnt < pic->tx_fifo.length; cnt++) {
		pic_desc = pic->tx_desc + cnt;
		if (atomic_read(&pic_desc->tasked))
			continue;

		retval += pic_desc->length;
	}

	return retval;
}

int fim_tx_buffers_level(struct fim_driver *driver)
{
	int retval, cnt;
	struct pic_dma_desc_t *pic_desc;
	struct pic_t *pic;

	if (!(pic = get_pic_by_index(driver->picnr)))
		return -ENODEV;

	retval = 0;
	for (cnt=0; cnt < pic->tx_fifo.length; cnt++) {
		pic_desc = pic->tx_desc + cnt;
		if (atomic_read(&pic_desc->tasked))
			retval += pic_desc->length;
	}

	return retval;
}

int fim_number_pics(void)
{
	return FIM_NR_PICS;
}

/*
 * Allocate a new FIM-buffer
 * @TODO: Bind this buffer to an internal list of the driver, so that we can free
 * all the buffers if the driver is being unloaded
 */
struct fim_buffer_t *fim_alloc_buffer(struct fim_driver *driver, int length,
				      unsigned int gfp_flags)
{
	struct fim_buffer_t *retval;

	if (!driver || length < 0)
		return NULL;

	if (!(retval = kmalloc(sizeof(struct fim_buffer_t) + length , gfp_flags)))
		return NULL;

	retval->sent = 0;
	retval->length = length;
	retval->data = (void *)retval + sizeof(struct fim_buffer_t);
	retval->private = NULL;
	return retval;
}


/*
 * Free an already requested FIM-buffer
 * @TODO: Use the internal list for removing this buffer
 */
void fim_free_buffer(struct fim_driver *driver, struct fim_buffer_t *buffer)
{
	if (!driver || !buffer)
		return;

	kfree(buffer);
}

/*
 * Be sure that you protect this function correctly.
 * Use the spinlock tx_lock for this purpose.
 */
inline static void pic_reset_tx_fifo(struct pic_t *pic)
{
	int cnt;
	struct iohub_dma_desc_t *desc;
	struct pic_dma_desc_t *pic_desc;

	fim_dma_reset_fifo(&pic->tx_fifo);
	for(cnt=0; cnt < pic->tx_fifo.length; cnt++) {
		desc = fim_dma_get_by_index(&pic->tx_fifo, cnt);
		desc->control &= ~IOHUB_DMA_DESC_CTRL_FULL;
		desc->length = 0;
		pic_desc = pic->tx_desc + cnt;
		atomic_set(&pic_desc->tasked, 0);
	}
}

/*
 * Check if there are enough buffers for the requested memory size
 * In success then will try to restart the DMA-channel
 * The return value is different then ZERO by errors.
 * This function can be called from an interrupt context too.
 */
int fim_send_buffer(struct fim_driver *driver, const struct fim_buffer_t *bufdesc)
{
	struct pic_t *pic;
	int cnt, len, last_len, pos;
	struct pic_dma_desc_t *pic_desc;
	struct pic_dma_desc_t *pic_descs[IOHUB_MAX_DMA_BUFFERS] = { NULL }; /* @XXX */
	struct iohub_dma_desc_t *desc;
	int backup_len;
	int count, retval;
	unsigned char *buffer;

	if (!bufdesc || !driver || (bufdesc->length <= 0) || !bufdesc->data ||
	    !bufdesc->private)
		return -EINVAL;

	if (!(pic = get_pic_from_driver(driver)))
		return -ENODEV;

	/*
	 * Lock this section so that the PIC can be unregistered now
	 */
	spin_lock(&pic->tx_lock);
	if (!pic->driver) {
		retval = -ENODEV;
		goto exit_ret;
	}

	/* Check for the available buffers */
	len = count = bufdesc->length;
	buffer = bufdesc->data;
	pos = 0;
	for (cnt=0; cnt < pic->tx_fifo.length; cnt++) {
		pic_desc = pic->tx_desc + cnt;
		if (atomic_read(&pic_desc->tasked))
			continue;

		atomic_set(&pic_desc->tasked, 1);
		pic_descs[pos++] = pic_desc;

		last_len = len;
		len -= (int)pic_desc->length;
		if (len <= 0) {
			/* Try to get the correct number of FIFO descriptors */
			if(!(desc = fim_dma_alloc(&pic->tx_fifo, pos))) {
				printk_err(" Couldn't get the FIFO descriptors.\n");
				break;
			}

			/* Copy the transfer data to the corresponding DMA-buffers */
			backup_len = pic_desc->length;
			pic_desc->length = last_len;
			for (cnt=0; pic_descs[cnt] && cnt < pos; cnt++) {
				len = pic_descs[cnt]->length;
				memcpy(phys_to_virt(pic_descs[cnt]->src), buffer, len);
				desc->src = pic_descs[cnt]->src;
				desc->reserved = 0;
				desc->length = len;
				buffer += len;
				desc = fim_dma_get_next(&pic->tx_fifo, desc);
			}
			pic_desc->length = backup_len;

			/*
			 * Save the private data into the first PIC-DMA descriptor
			 * This private data will be used inside the TX-callback function
			 * @BUG: We must save the requested length for the TX-callback,
			 * then the DMA-controller set the length to zero.
			 */
			pic_descs[0]->private = bufdesc->private;
			pic_descs[0]->total_length = bufdesc->length;
			fim_start_tx_dma(pic);
			retval = 0;
			goto exit_ret;
		}
	}

	printk_debug("No free TX-buffers available (%i). Aborting.\n", bufdesc->length);
	for (cnt=0; pic_descs[cnt] && cnt < pic->tx_fifo.length; cnt++)
		atomic_dec(&pic_descs[cnt]->tasked);
	retval = -ENOMEM;

 exit_ret:
	spin_unlock(&pic->tx_lock);
	return retval;
}

/* @TODO: Please test this function before using it */
void fim_flush_rx(struct fim_driver *driver)
{
	unsigned int cnt;
	struct pic_t *pic;
	struct iohub_dma_desc_t *desc;
	struct iohub_dma_fifo_t *fifo;

	if (!(pic = get_pic_by_index(driver->picnr)))
		return;

	writel(0x00, pic->iohub_addr + IOHUB_RX_DMA_ICTRL_REG);
	writel(IOHUB_RX_DMA_CTRL_CA, pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);
	for (cnt=0; cnt < pic->rx_fifo.length; cnt++) {
		desc = fim_dma_get_by_index(&pic->rx_fifo, cnt);
		desc->control = IOHUB_DMA_DESC_CTRL_INT;
	}

	writel(IOHUB_ICTRL_RXNCIE | IOHUB_ICTRL_RXNRIE | IOHUB_ICTRL_RXPCIE,
	       pic->iohub_addr + IOHUB_RX_DMA_ICTRL_REG);
	writel(pic->rx_fifo.phys_descs, pic->iohub_addr + IOHUB_RX_DMA_BUFPTR_REG);
	writel(IOHUB_RX_DMA_CTRL_CE, pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);

	/* Reset the internal fifo too */
	fifo = &pic->rx_fifo;
	fim_dma_reset_fifo(fifo);
}

/*
 * Drain the TX-buffers by first aborting the DMA-channel. The DMA-descriptors
 * will be reseted inside the interrupt routine
 */
void fim_flush_tx(struct fim_driver *driver)
{
	struct pic_t *pic;

	if (!(pic = get_pic_by_index(driver->picnr)))
		return;

	pic_reset_tx_fifo(pic);

	writel(IOHUB_RX_DMA_CTRL_CA, pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);
}

/*
 * This function is responsible for restarting the DMA-channel with the correct index
 * If the DMA-channel is transferring data it will returns inmediately, but the
 * callback of the DMA-TX will recall this function and send the requested data buffers
 */
static int fim_start_tx_dma(struct pic_t *pic)
{
	unsigned long txctrl;
	struct iohub_dma_desc_t *desc = NULL;
	struct iohub_dma_fifo_t *fifo;
	int retval = 0;
	unsigned int channel_index, fifo_index;

	if (fim_dma_is_empty(&pic->tx_fifo)) {
		printk(KERN_DEBUG "Nothing to do, the TX-FIFO is empty.\n");
		return 0;
	}

	/* @XXX: Need to protect this section */
	if (atomic_read(&pic->tx_tasked)) {
		printk_debug("The DMA-Controller seems to be tasked.\n");
		return 0;
	}

	/* Lock the next DMA-transfer and get the internal data */
	atomic_set(&pic->tx_tasked, 1);

	spin_lock(&pic->tx_lock);
	fifo = &pic->tx_fifo;
	fifo->dma_last = fifo->dma_next;

	/*
	 * Write the control bit fields for the new TX-transfer
	 * The close-buffer interrupt will be enabled only for the last descriptor!
	 */
	for (desc=fifo->dma_first; ; desc=fim_dma_get_next(fifo, desc)) {
		if (desc->length <= 0) {
			printk_err("Invalid FIFO descriptor 0x%p length, %u\n",
			       desc, desc->length);
			retval = -EINVAL;
			goto exit_unlock;
		}

		/* IMPORTANT: Enable the interrupt only for the last buffer! */
		desc->control = IOHUB_DMA_DESC_CTRL_FULL;
		if (desc == fifo->last)
			desc->control |= IOHUB_DMA_DESC_CTRL_WRAP;
		if (desc == fifo->dma_last) {
			desc->control |= (IOHUB_DMA_DESC_CTRL_INT |
					  IOHUB_DMA_DESC_CTRL_LAST);
			break;
		}
	}

	/* Get the index of the first FIFO descriptor for the DMA-controller */
	fifo_index = fim_dma_get_index(&pic->tx_fifo, fifo->dma_first);
	txctrl = readl(pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);
	txctrl |= IOHUB_TX_DMA_CTRL_INDEXEN | IOHUB_TX_DMA_CTRL_INDEX(fifo_index);
	writel(txctrl, pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);

	/*
	 * If the TX-FIFO and the DMA-Channel have different index then we have
	 * an error, then normally they must have the same value.
	 * @FIXME: Workaround for this problem, or remove the paranoic sanity check
	 */
	channel_index = readl(pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG) & 0x3FF;
	if (fifo_index && (fifo_index != channel_index)) {
		printk_warn("FIFO index 0x%X mismatch DMA index 0x%X\n",
		       fifo_index, channel_index);
	} else {
		printk_debug("DMA index %i for next DMA buffer\n", channel_index);
	}

	/*
	 * Check if an abort interrupt was executed in the time we have
	 * configured the DMA-descriptors. In that skip the channel restart
	 */
	if (!atomic_read(&pic->tx_aborted)) {
		writel(txctrl & ~IOHUB_TX_DMA_CTRL_CE,
		       pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);
		writel(txctrl | IOHUB_TX_DMA_CTRL_CE,
		       pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);
	} else
		atomic_set(&pic->tx_tasked, 0);

 exit_unlock:
	spin_unlock(&pic->tx_lock);
	return retval;
}

/* DONT poll with this function. We need another function for this purpose. */
int fim_get_exp_reg(struct fim_driver *driver, int nr, unsigned int *value)
{
	struct pic_t *pic;

	if (!driver)
		return -EINVAL;

	if (!(pic = get_pic_by_index(driver->picnr)))
		return -EINVAL;

	*value = readl(pic->reg_addr + NS92XX_FIM_EXP_REG(nr));

	return 0;
}

/* Called when the PIC interrupts the ARM-processor */
static void isr_from_pic(struct pic_t *pic, int irqnr)
{
	unsigned int status;
	struct fim_driver *driver;
	unsigned int rx_fifo;
	unsigned int timeout;

	status = readl(pic->reg_addr + NS92XX_FIM_GEN_CTRL_REG);
	rx_fifo = readl(pic->iohub_addr + IOHUB_RX_DIR_FIFO_REG);

	driver = pic->driver;
	if (driver && driver->fim_isr)
		driver->fim_isr(driver, pic->irq, status & 0xFF, rx_fifo);

	/* @TEST */
	writel(status, pic->reg_addr + NS92XX_FIM_CTRL7_REG);

	writel(status | NS92XX_FIM_GEN_CTRL_INTACKWR,
	       pic->reg_addr + NS92XX_FIM_GEN_CTRL_REG);

	timeout = 0xFFFF;
	do {
		timeout--;
		status = readl(pic->reg_addr + NS92XX_FIM_GEN_CTRL_REG);
	} while (timeout && (status & NS92XX_FIM_GEN_CTRL_INTFROMPIC));

	/* @XXX: Should we stop the PIC for avoiding more timeout errors? */
	if (!timeout) {
		printk_err("FIM %i interrupt handshake timeout.\n", pic->index);
		return;
	}

	writel(status & ~NS92XX_FIM_GEN_CTRL_INTACKWR, pic->reg_addr +
	       NS92XX_FIM_GEN_CTRL_REG);
}

static void isr_dma_tx(struct pic_t *pic, int irqnr)
{
	struct iohub_dma_fifo_t *fifo;
	unsigned int ifs, txctrl, ictrl;
	int cnt;
	struct iohub_dma_desc_t *desc;
	struct pic_dma_desc_t *pic_desc;
	struct fim_driver *driver;
	struct fim_buffer_t buffer= { 0, NULL, NULL };

	fifo = &pic->tx_fifo;
	ifs = readl(pic->iohub_addr + IOHUB_IFS_REG);
	if (ifs & IOHUB_IFS_TXNRIP) {
		/*
		 * From the Net+OS driver seems to be that the DMA controller is
		 * setting the TXNRIP status bit although no error happened. But for
		 * being sure that no error happened, we check the full control bit
		 * of the last buffer descriptor: If this bit is high then the
		 * last buffer was not transferred, ERROR!
		 */
		txctrl = readl(pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG) - 1;
		desc = fifo->dma_last;
		if(desc->control & IOHUB_DMA_DESC_CTRL_FULL) {
			ictrl = readl(pic->iohub_addr + IOHUB_TX_ICTRL_REG);
			printk_err("TXNRIP: ICTRL 0x%08x | CTRL 0x%08x | DESC 0x%p\n",
			       ictrl, txctrl, desc);
		}
	}

	/*
	 * The channel abort is normally being called for flushing the TX-buffers.
	 */
	if (ifs & IOHUB_IFS_TXCAIP) {

		atomic_set(&pic->tx_aborted, 1);
		spin_lock(&pic->tx_lock);
		printk_debug("TXCAIP: Freeing the TX-buffers\n");
		pic_reset_tx_fifo(pic);

		/* Reset the index of the TX-channel to zero */
		writel(IOHUB_RX_DMA_CTRL_CA | IOHUB_TX_DMA_CTRL_INDEXEN,
		       pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);
		writel(0x00, pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);

		/* Reset the atomic variables for enabling the TX-requests */
		atomic_set(&pic->tx_aborted, 0);
		atomic_set(&pic->tx_tasked, 0);
		spin_unlock(&pic->tx_lock);
		return;
	}

	/*
	 * If no NC interrupt pending then return immediately.
	 * @FIXME: Is possible that a driver is waiting for the TXNCIP on this case?
	 */
	if(!(ifs & IOHUB_IFS_TXNCIP))
		return;

	/* Now free the allocated PIC-DMA descriptors */
	desc = fifo->dma_first;
	driver = pic->driver;
	do {
		for(cnt=0; cnt < pic->tx_fifo.length; cnt++) {
			pic_desc = pic->tx_desc + cnt;
			if(pic_desc->src == desc->src) {
				if(!buffer.data) {
					buffer.data = phys_to_virt(desc->src);
					buffer.private = pic_desc->private;
				}

				/*
				 * @BUG: The DMA-channel seems to reset the buffer length
				 * otherwise use: buffer.length += desc->length;
				 */
				buffer.length = pic_desc->total_length;
				desc->src = 0;
				atomic_set(&pic_desc->tasked, 0);
				break;
			}
		}
		desc = fim_dma_get_next(fifo, desc);
	} while (desc != fim_dma_get_next(fifo, fifo->dma_last));

	/* Now give the control to the driver's callback function */
	if (driver->dma_tx_isr)
		driver->dma_tx_isr(driver, pic->irq, &buffer);

	/* Free the DMA-controller */
	fifo->dma_first = fim_dma_get_next(fifo, fifo->dma_last);
	atomic_set(&pic->tx_tasked, 0);

	/* Check if another descriptors are waiting in the FIFO */
	if (fifo->dma_next != fifo->dma_last)
		fim_start_tx_dma(pic);
}

/* Only check that no errors ocurred by the last buffer descriptor */
static void isr_dma_rx(struct pic_t *pic, int irqnr)
{
	unsigned int ifs;
	struct iohub_dma_fifo_t *fifo;
	unsigned int rxctrl, ictrl;
	unsigned long error;
	int verbose;

	/* Get the selected verbosity */
	verbose = (pic->driver) ? (pic->driver->verbose) : (0);

	/* If we have an error, then always restart the DMA-channel */
	ifs = readl(pic->iohub_addr + IOHUB_IFS_REG);
	error = 0;

	if (ifs & IOHUB_IFS_RXNRIP) {
		ictrl = readl(pic->iohub_addr + IOHUB_RX_DMA_ICTRL_REG);
		rxctrl = readl(pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);

		if (verbose && printk_ratelimit())
			printk_warn("RXNRIP: ictrl 0x%08x | rxctrl 0x%08x\n", ictrl, rxctrl);
		error |= IOHUB_IFS_RXNRIP;
	}

	if (!(ifs & IOHUB_IFS_RXNCIP)) {

		if (verbose && printk_ratelimit())
			printk_warn("RXNCIP: Unexpected state (ifs: %x)\n", ifs);
		error |= IOHUB_IFS_RXNCIP;
	}

	/* If we have a failure, then reset the DMA-controller and -FIFO */
	if (error) {
		fifo = &pic->rx_fifo;
		writel(fifo->phys_descs, pic->iohub_addr + IOHUB_RX_DMA_BUFPTR_REG);
		writel(0x00, pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);
		writel(ifs, pic->iohub_addr + IOHUB_IFS_REG);
		writel(IOHUB_RX_DMA_CTRL_CE, pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);

		/* This is for the RX tasklet that will inform the FIM-driver about this failure */
		fifo->rx_error = error;
	}

	/*
	 * @IMPORTANT: The API of Net+OS is restarting the RX-channel after each
	 * transfer, but we don't know exactly why. Since our API is working
	 * without the restart, we don't touch the RX-channel.
	 */
#if 0
	rxctrl = readl(pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);
	rxctrl &= ~IOHUB_RX_DMA_CTRL_CE;
	writel(rxctrl, pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);
	writel(rxctrl | IOHUB_RX_DMA_CTRL_CE, pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);
#endif

	/* Schedule the task which will free the RX-DMA buffer descriptors */
	tasklet_hi_schedule(&pic->rx_tasklet);
}

/* This is the main ISR for the PIC-interrupts */
static irqreturn_t pic_irq(int irq, void *tpic)
{
	unsigned int ifs;
	struct pic_t *pic = (struct pic_t *)tpic;
	unsigned long flags;

	spin_lock_irqsave(&pic->lock, flags);
	ifs = readl(pic->iohub_addr + IOHUB_IFS_REG);

	if (ifs & IOHUB_IFS_MODIP)
		isr_from_pic(pic, irq);
	if (ifs & IOHUB_IFS_DMA_TX)
		isr_dma_tx(pic, irq);
	if (ifs & IOHUB_IFS_DMA_RX)
		isr_dma_rx(pic, irq);

	writel(ifs, pic->iohub_addr + IOHUB_IFS_REG);
	spin_unlock_irqrestore(&pic->lock, flags);
	return IRQ_HANDLED;
}

int fim_enable_irq(struct fim_driver *driver)
{
	struct pic_t *pic;

	if (!driver)
		return -EINVAL;

	if (!(pic = get_pic_from_driver(driver)))
		return -EINVAL;

	/* IRQ enable if required */
	if (!atomic_read(&pic->irq_enabled)) {
		printk_debug("Enabling the PIC IRQ %i\n", pic->irq);
		enable_irq(pic->irq);
		atomic_set(&pic->irq_enabled, 1);
	}
	return 0;
}

int fim_disable_irq(struct fim_driver *driver)
{
	struct pic_t *pic;

	if (!driver)
		return -EINVAL;

	if (!(pic = get_pic_from_driver(driver)))
		return -EINVAL;

	/* Disable the IRQ */
	if (atomic_read(&pic->irq_enabled)) {
		printk_debug("Disabling the PIC IRQ %i\n", pic->irq);
		disable_irq(pic->irq);
		atomic_set(&pic->irq_enabled, 0);
	}
	return 0;
}

/*
 * Function for downloading the FIM-firmware
 * IMPORTANT: This function will automatically stop the FIM if it's running,
 * additionally it will (try) to restore the current state of the DMA-channels
 */
int fim_download_firmware(struct fim_driver *driver)
{
	struct pic_t *pic;
	int retval;
	const struct firmware *fw;
	const unsigned char *fwbuf;
	unsigned int txctrl, rxctrl;

	if (!(pic = get_pic_from_driver(driver)))
		return -EINVAL;

	/* Stop the FIM first */
	if (pic_is_running(pic)) {
		retval = pic_stop_and_reset(pic);
		if (retval) {
			printk_err("Couldn't stop the PIC %i\n", pic->index);
			return retval;
		}
	}

	/*
	 * Now download the firmware using the firmware-subsystem or from
	 * the passed array with the firmware code
	 */
	if (driver->fw_name) {
		printk_debug("Requesting the firmware '%s'\n", driver->fw_name);
		snprintf(pic->fw_name, FIM_MAX_FIRMWARE_NAME, "%s", driver->fw_name);
		retval = request_firmware(&fw, driver->fw_name, pic->dev);
		if (retval) {
			printk_err("request_firmware() failed, %i\n", retval);
			goto exit_download;
		}
		fwbuf = fw->data;
	} else if (driver->fw_code) {
		printk_debug("Using the built-in firmware code\n");
		fwbuf = driver->fw_code;
	} else {
		printk_err("No code and no firmware name passed? Aborting\n");
		retval = -EINVAL;
		goto exit_download;
	}

	/*
	 * Since the firmware download should be executed on the fly, disable
	 * the DMA-channels first, otherwise the channels will run like a
	 * spree killer
	 */
	spin_lock(&pic->tx_lock);
	txctrl = readl(pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);
	rxctrl = readl(pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);
	writel(txctrl & ~IOHUB_TX_DMA_CTRL_CE, pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);
	writel(rxctrl & ~IOHUB_RX_DMA_CTRL_CE, pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);
	retval = pic_download_firmware(pic, fwbuf);
	writel(txctrl, pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);
	writel(rxctrl, pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);
	spin_unlock(&pic->tx_lock);

	/* Release the obtained data from the firmware-subsystem */
	if (driver->fw_name)
		release_firmware(fw);

	if (retval)
		printk_err("Firmware install in the PIC %i failed.\n", pic->index);

 exit_download:
	return retval;
}

int fim_register_driver(struct fim_driver *driver)
{
	struct pic_t *pic;
	int retval;
	const struct firmware *fw;
	const unsigned char *fwbuf;

	/* Sanity checks */
	if (!driver || !driver->driver.name)
		return -EINVAL;

	if (!(pic = get_pic_by_index(driver->picnr)))
		return -ENODEV;

	if (pic->driver || PIC_IS_LOCKED(pic)) {
		printk_err("PIC %i already requested.\n", pic->index);
		return -EBUSY;
	}

        /* Lock the PIC request */
	PIC_LOCK_REQUEST(pic);

	/* Stop the PIC before any other action */
	retval = pic_stop_and_reset(pic);
	if (retval) {
		printk_err("Couldn't stop and reset the FIM.\n");
		goto exit_free_pic;
	}

	/* Try to get the firmware from the user space */
	if (driver->fw_name) {
		printk_debug("Requesting the firmware '%s'\n", driver->fw_name);
		snprintf(pic->fw_name, FIM_MAX_FIRMWARE_NAME, "%s", driver->fw_name);
		retval = request_firmware(&fw, driver->fw_name, pic->dev);
		if (retval) {
			printk_err("request_firmware() failed, %i\n", retval);
			goto exit_free_pic;
		}
		fwbuf = fw->data;
	} else if (driver->fw_code) {
		printk_debug("Using the built-in firmware code\n");
		fwbuf = driver->fw_code;
	} else {
		printk_err("%s: Neither code nor firmware passed. Aborting request.\n",
			   driver->driver.name);
		goto exit_free_pic;
	}

	retval = pic_download_firmware(pic, fwbuf);

	/* Release the obtained data from the firmware-subsystem */
	if (driver->fw_name)
		release_firmware(fw);

	if (retval) {
		printk_err("%s: Firmware install by the FIM%i failed.\n",
			   driver->driver.name, pic->index);
		goto exit_free_pic;
	}

	/* Start the PIC at zero */
	retval = pic_start_at_zero(pic);
	if (retval) {
		printk_err("%s: FIM%i couldn't be started at zero.\n",
			   driver->driver.name, pic->index);
		goto exit_free_pic;
	}

	/*
	 * The DMA channels will be controlled by the CPU. Pass the driver-config
	 * values to the init-function
	 */
	retval = pic_dma_init(pic, driver->dma_cfg);
	if (retval) {
		printk_err("%s: DMA channel init failed for the FIM%i\n",
			   driver->driver.name, pic->index);
		goto goto_stop_pic;
	}

	/*
	 * Enable the tasklet before requesting the IRQ, then in some cases the
	 * FIM sends an interrupt before its initialization through the FIM-driver.
	 */
	pic->driver = driver;
	driver->dev = pic->dev;
	tasklet_init(&pic->rx_tasklet, pic_rx_tasklet_func, (unsigned long)pic);

	/* Request the IRQ for this FIM */
	retval = request_irq(pic->irq, pic_irq, 0, driver->driver.name, pic);
	if (retval) {
		printk_err("%s: Couldn't request the IRQ %i\n",
			   driver->driver.name, pic->irq);
		goto exit_kill_tasklet;
	}

	/* Disable the IRQ and done */
	disable_irq(pic->irq);
	atomic_set(&pic->irq_enabled, 0);
	printk_info("FIM%i registered for driver '%s' [IRQ %i]\n",
		    pic->index, driver->driver.name, pic->irq);

	return 0;

 exit_kill_tasklet:
	tasklet_kill(&pic->rx_tasklet);

 goto_stop_pic:
	pic_stop_and_reset(pic);

 exit_free_pic:
	PIC_FREE_REQUEST(pic);

	return retval;
}

int fim_unregister_driver(struct fim_driver *driver)
{
	int ret;
	struct pic_t *pic;

	if (!driver)
		return -EINVAL;

	if (!(pic = get_pic_from_driver(driver)))
		return -EINVAL;

	if (!pic->driver || !PIC_IS_LOCKED(pic))
		return -EINVAL;

	/* Stop the PIC but wait for another running operations first */
	spin_lock(&pic->tx_lock);
	ret = pic_stop_and_reset(pic);
	if (ret)
		printk_err("Couldn't stop the PIC %i\n", pic->index);

	fim_disable_irq(pic->driver);

	/* Free the requested IRQ */
	printk_debug("Freeing the IRQ %i\n", pic->irq);
	free_irq(pic->irq, pic);

	/* Free the DMA-resources */
	tasklet_kill(&pic->rx_tasklet);
	pic_dma_stop(pic);

	/* And free the driver field */
	pic->driver = NULL;
	PIC_FREE_REQUEST(pic);
	spin_unlock(&pic->tx_lock);
	return 0;
}

/*
 * Return a PIC-pointer for low level operations
 * The driver structure contains the number of the pointer
 */
struct pic_t *fim_request_pic(int picnr)
{
	struct pic_t *pic;

	if (!(pic = get_pic_by_index(picnr)))
		return NULL;

	if (pic->driver || PIC_IS_LOCKED(pic))
		return NULL;

	PIC_LOCK_REQUEST(pic);
	writel(0x00, pic->reg_addr + NS92XX_FIM_GEN_CTRL_REG);
	printk_info("PIC %i successful requested\n", pic->index);
	return pic;
}

/*
 * Free the PIC that was requested with the above function fim_request_pic()
 */
void fim_free_pic(struct pic_t *pic)
{
	if (!pic)
		return;

	if (!(get_pic_by_index(pic->index)))
		return;

	PIC_FREE_REQUEST(pic);
}

/*
 * This function can be used for downloading a firmware to the PIC.
 * Please note that this function will reset the complete IOHUB-module, included the
 * DMA-configuration registers. That's important when the FIM-driver
 * is using the index of the DMA-channel, then this will have the index zero after
 * the download.
 */
static int pic_download_firmware(struct pic_t *pic, const unsigned char *buffer)
{
	int mode, ret;
	unsigned int status;
	struct fim_program_t *program = (struct fim_program_t *)buffer;
	int offset;

	if (!program || !pic)
		return -EINVAL;

	if (!(FORMAT_TYPE_VALID(program->format)))
		return -EINVAL;

	printk_debug("Starting the download of the PIC firmware.\n");

	/* Check if the PIC is running, before starting the firmware update */
	if (pic_is_running(pic)) {
		printk_err("The PIC %i is running. Aborting the download.\n",
		       pic->index);
		return -EBUSY;
	}

	/* Check if the firmware has the correct header */
	if (!(PROCESSOR_TYPE_VALID(program->processor))) {
		printk_err("Invalid processor type. Aborting firmware download.\n");
		return -EINVAL;
	}

	/* Enable the clock to IO processor and reset the module */
	status = readl(SYS_CLOCK);
	writel(status | (1 << (pic->index + FIM0_SHIFT)), SYS_CLOCK);
	status = readl(SYS_RESET);
	writel(status & ~(1 << (pic->index + FIM0_SHIFT)), SYS_RESET);
	writel(status | (1 << (pic->index + FIM0_SHIFT)), SYS_RESET);

	/* Configure the output clock */
	ret = pic_config_output_clock_divisor(pic, program);
	if (ret) {
		printk_err("Couldn't set the clock output divisor.\n");
		return ret;
	}

	switch (program->hw_mode) {
	case FIM_HW_ASSIST_MODE_NONE:
		mode = 0x00;
		break;
	case FIM_HW_ASSIST_MODE_GENERIC:
		mode = 0x01;
		break;
	case FIM_HW_ASSIST_MODE_CAN:
		mode = 0x02;
		break;
	default:
		printk_err("Invalid HWA mode %i\n", program->hw_mode);
		return -EINVAL;
	}

	status = readl(pic->hwa_addr + NS92XX_FIM_HWA_GEN_CONF_REG);
	writel(mode | status, pic->hwa_addr + NS92XX_FIM_HWA_GEN_CONF_REG);

	/* Update the HW assist config registers */
	for (offset = 0; offset < FIM_NUM_HWA_CONF_REGS; offset++) {
		status = program->hwa_cfg[offset];
		writel(status, pic->hwa_addr + NS92XX_FIM_HWA_SIGNAL(offset));
	}

	/* Check for the maximal supported number of instructions */
	if (program->length > FIM_NS9215_MAX_INSTRUCTIONS) {
		printk_err("Invalid firmware length %i (too long)\n", program->length);
		return -EINVAL;
	}

	/* Start programming the PIC (the program size is in 16bit-words) */
	printk_debug("Starting to program the firmware (%i Bytes)\n",
		     program->length);
	for (offset = 0; offset < program->length; offset++)
		writel(program->data[offset] & NS92XX_FIM_INSTRUCTION_MASK,
		       pic->instr_addr + 4*offset);

	/* Save the firmware length into the internal structure for the sysfs */
	pic->fw_length = program->length;
	return 0;
}

static int pic_start_at_zero(struct pic_t *pic)
{
	unsigned int regval;

	if (!pic)
		return -EINVAL;
	if (pic->index < 0)
		return -ENODEV;

	printk_debug("Starting the PIC %i at zero.\n", pic->index);

	regval = readl(pic->reg_addr + NS92XX_FIM_GEN_CTRL_REG);
	writel(regval | NS92XX_FIM_GEN_CTRL_START_PIC, pic->reg_addr +
	       NS92XX_FIM_GEN_CTRL_REG);

	/* Check if the PIC is really running */
	if (!pic_is_running(pic)) {
		printk_err("Unable to start the PIC %i\n", pic->index);
		return -EAGAIN;
	}

	return 0;
}

static int pic_stop_and_reset(struct pic_t *pic)
{
	unsigned int regval;
	int retval;

	if (!pic)
		return -EINVAL;

	if (pic_is_running(pic))
		printk_debug("The PIC %i is running, need to stop it.\n", pic->index);

	regval = readl(pic->reg_addr + NS92XX_FIM_GEN_CTRL_REG);

	/* Clear the interrupt flags too! */
	regval &= ~NS92XX_FIM_INT_MASK(0xff);

	writel(regval & NS92XX_FIM_GEN_CTRL_STOP_PIC,
	       pic->reg_addr + NS92XX_FIM_GEN_CTRL_REG);

	retval = 0;
	if (pic_is_running(pic)) {
		printk_err("Unabled to stop the PIC %i\n", pic->index);
		retval = -EAGAIN;
	}

	/* Reset the HWA generial register too */
	writel(0x00, pic->hwa_addr + NS92XX_FIM_HWA_GEN_CONF_REG);

	return retval;
}

/*
 * This function stops the FIM and check if the DMA-channel still has a buffer
 * for the FIM.
 */
int fim_send_reset(struct fim_driver *driver)
{
	struct pic_t *pic;
	int retval, cnt, fifo_reset;
	struct iohub_dma_fifo_t *fifo;
	struct iohub_dma_desc_t *desc;

	if (!(pic = get_pic_by_index(driver->picnr)))
		return -EINVAL;

	retval = pic_stop_and_reset(pic);
	if (retval)
		return retval;

	/*
	 * For avoiding errors by the restart of the FIM, check if a DMA buffer
	 * is waiting for being transmitted. In that case, reset the buffer and
	 * reset the index of the DMA-controller.
	 */
	fifo = &pic->tx_fifo;
	fifo_reset = 0;
	for (desc = fifo->first, cnt = 0; cnt < fifo->length; cnt++) {

		/*
		 * If we are stopping the FIM but have some DMA-buffers pending, then
		 * we need to reset the DMA-channel before the next restart.
		 */
		if (!fifo_reset && (desc->control & IOHUB_DMA_DESC_CTRL_FULL))
			fifo_reset = 1;

		desc->control = 0;
		desc = fim_dma_get_next(fifo, desc);
	}

	if (fifo_reset) {
		unsigned long txctrl;

		printk_warn("Stopping FIM %i with a full DMA buffer!\n",
			    pic->index);
		fim_dma_reset_fifo(fifo);
		txctrl = IOHUB_TX_DMA_CTRL_INDEXEN | IOHUB_TX_DMA_CTRL_INDEX(0);
		writel(txctrl, pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);
	}

	/*
	 * Reset the RX-channel too.
	 * IMPORTANT: Set the wrap control bit by the last DMA-buffer, otherwise
	 * the FIFO will overflow!
	 */
	fifo = &pic->rx_fifo;
	for (desc = fifo->first, cnt = 0; cnt < fifo->length; cnt++) {
		desc->control = IOHUB_DMA_DESC_CTRL_INT;
		desc = fim_dma_get_next(fifo, desc);
	}
	fifo->last->control |= IOHUB_DMA_DESC_CTRL_WRAP;
	fim_dma_reset_fifo(fifo);

	/* Free the internal resources */
	atomic_set(&pic->tx_tasked, 0);
	atomic_set(&pic->tx_aborted, 0);

	fim_send_stop(driver);
	fim_send_start(driver);

	return retval;
}

int fim_send_start(struct fim_driver *driver)
{
	ulong reg;

	struct pic_t *pic;
	if (!(pic = get_pic_by_index(driver->picnr)))
		return -EINVAL;

	/* Connect the clock to the FIM */
	reg = readl(SYS_CLOCK);
	writel(reg | (1 << (pic->index + FIM0_SHIFT)), SYS_CLOCK);

	reg = readl(SYS_RESET);
	writel(reg | (1 << (pic->index + FIM0_SHIFT)), SYS_RESET);

	return pic_start_at_zero(pic);
}

int fim_send_stop(struct fim_driver *driver)
{
	struct pic_t *pic;
	int ret;
	ulong reg;

	if (!(pic = get_pic_by_index(driver->picnr)))
		return -EINVAL;

	reg = readl(pic->reg_addr + NS92XX_FIM_GEN_CTRL_REG);

	/* Clear the interrupt flags too! */
	reg &= ~NS92XX_FIM_INT_MASK(0xff);

	writel(reg & ~NS92XX_FIM_GEN_CTRL_START_PIC,
	       pic->reg_addr + NS92XX_FIM_GEN_CTRL_REG);

	ret = 0;
	if (pic_is_running(pic)) {
		printk_err("Unabled to stop the PIC %i\n", pic->index);
		ret = -EAGAIN;
	}

	/* Disable the clock and reset the module */
	reg = readl(SYS_CLOCK);
	writel(reg & ~(1 << (pic->index + FIM0_SHIFT)), SYS_CLOCK);
	reg = readl(SYS_RESET);
	writel(reg & ~(1 << (pic->index + FIM0_SHIFT)), SYS_RESET);

	return ret;
}

int fim_send_interrupt2(struct fim_driver *driver, unsigned int code)
{
	struct pic_t *pic;
	if (!driver || !(pic = get_pic_by_index(driver->picnr)))
		return -EINVAL;

	return pic_send_interrupt(pic, code);
}

/*
 * This function provides the access to the control registers of the PICs
 * reg : Number of the control register (from 0 to 15)
 * val : Value to write into the control register
 */
void fim_set_ctrl_reg(struct fim_driver *driver, int reg, unsigned int val)
{
	struct pic_t *pic;

	if (!(pic = get_pic_from_driver(driver)))
		return;

	writel(val, pic->reg_addr + NS92XX_FIM_CTRL_REG(reg));
}

/* Provides the read access to the control registers of the PICs */
int fim_get_ctrl_reg(struct fim_driver *driver, int reg, unsigned int *val)
{
	struct pic_t *pic;

	if (!(pic = get_pic_from_driver(driver)) || !val)
		return -EINVAL;

	if (NS92XX_FIM_CTRL_REG_CHECK(reg))
		return -EINVAL;

	*val = readl(pic->reg_addr + NS92XX_FIM_CTRL_REG(reg));
	return 0;
}

int fim_get_stat_reg(struct fim_driver *driver, int reg, unsigned int *val)
{
	struct pic_t *pic;

	if (!(pic = get_pic_from_driver(driver)) || !val)
		return -EINVAL;

	if (NS92XX_FIM_STAT_REG_CHECK(reg))
		return -EINVAL;

	*val = readl(pic->reg_addr + NS92XX_FIM_STAT_REG(reg));
	return 0;
}

/* Interrupt the PIC sending the interrput with the number `code' */
static int pic_send_interrupt(struct pic_t *pic, u32 code)
{
	unsigned int stopcnt;
	u32 status;

	if (!pic || !code || (code & ~0x7f))
		return -EINVAL;

	if (!pic_is_running(pic)) {
		printk_err("The PIC %i isn't running. Aborting the IRQ request.\n",
			   pic->index);
		return -EAGAIN;
	}

	code = NS92XX_FIM_INT_MASK(code);
	status = readl(pic->reg_addr + NS92XX_FIM_GEN_CTRL_REG);
	writel(status | code, pic->reg_addr + NS92XX_FIM_GEN_CTRL_REG);

	/* This loop is perhaps problematic, exit with a timeout */
	stopcnt = 0xFFFF;
	do {
		status = readl(pic->reg_addr  + NS92XX_FIM_GEN_CTRL_REG);
		stopcnt--;
	} while (!(status & NS92XX_FIM_GEN_CTRL_INTACKRD) && stopcnt);

	if (!stopcnt) {
		printk_err("Timeout waiting for RDACK from the PIC %i\n", pic->index);
		return -EINVAL;
	}

	/* Reset the interrupt bits for the PIC acknowledge */
	status &= ~NS92XX_FIM_GEN_CTRL_INTTOPIC;
	writel(status, pic->reg_addr + NS92XX_FIM_GEN_CTRL_REG);

	stopcnt = 0xFFFF;
	do {
		status = readl(pic->reg_addr + NS92XX_FIM_GEN_CTRL_REG);
		stopcnt--;
	} while ((status & NS92XX_FIM_GEN_CTRL_INTACKRD) && stopcnt);

	if (!stopcnt) {
		printk_err("Timeout by the second RDACK from the PIC %i\n", pic->index);
		return -EINVAL;
	}

	return 0;
}

/* Set the HWA PIC clock (see PIC module specification, page 19) */
static int pic_config_output_clock_divisor(struct pic_t *pic,
					   struct fim_program_t *program)
{
	int div;
	int clkd;
	unsigned int val;

	if (!pic || !program) return -EINVAL;

	div = program->clkdiv;
	switch (div) {
	case FIM_CLK_DIV_2:
		clkd = FIM_HWA_GEN_CFG_CLKSEL_DIVIDE_BY_2;
		break;

	case FIM_CLK_DIV_4:
		clkd = FIM_HWA_GEN_CFG_CLKSEL_DIVIDE_BY_4;
		break;

	case FIM_CLK_DIV_8:
		clkd = FIM_HWA_GEN_CFG_CLKSEL_DIVIDE_BY_8;
		break;

	case FIM_CLK_DIV_16:
		clkd = FIM_HWA_GEN_CFG_CLKSEL_DIVIDE_BY_16;
		break;

	case FIM_CLK_DIV_32:
		clkd = FIM_HWA_GEN_CFG_CLKSEL_DIVIDE_BY_32;
		break;

	case FIM_CLK_DIV_64:
		clkd = FIM_HWA_GEN_CFG_CLKSEL_DIVIDE_BY_64;
		break;

	case FIM_CLK_DIV_128:
		clkd = FIM_HWA_GEN_CFG_CLKSEL_DIVIDE_BY_128;
		break;

	case FIM_CLK_DIV_256:
		clkd = FIM_HWA_GEN_CFG_CLKSEL_DIVIDE_BY_256;
		break;

	default:
		return -EINVAL;
	}

	val = readl(pic->hwa_addr + NS92XX_FIM_HWA_GEN_CONF_REG);
	writel(val | clkd, pic->hwa_addr + NS92XX_FIM_HWA_GEN_CONF_REG);
	return 0;
}

static int fim_probe(struct device *dev)
{
	int i, cnt, retval;
	struct pic_t *pic;

	if (!dev)
		return -EINVAL;

	printk_debug("Starting the FIM-probe of the device `%s'\n", dev_name(dev));

	/* Use the get_pic_by_index function for verifying that we have a FIM attached */
	pic = dev_get_drvdata(dev);
	pic = get_pic_by_index(pic->index);
	if (!pic) {
		printk_err("Invalid PIC index %i\n", pic->index);
		return -ENODEV;
	}

	i = pic->index;
	pic->dev = dev;
	pic->reg_addr = ioremap(NS92XX_FIM_REG_BASE_PA + i*NS92XX_FIM_REG_OFFSET,
				NS92XX_FIM_REG_SIZE);
	pic->instr_addr = ioremap(NS92XX_FIM_INSTR_BASE_PA + i*NS92XX_FIM_INSTR_OFFSET,
				  NS92XX_FIM_INSTR_SIZE);
	pic->hwa_addr = ioremap(NS92XX_FIM_HWA_BASE_PA + i*NS92XX_FIM_HWA_OFFSET,
				NS92XX_FIM_HWA_SIZE);
	pic->iohub_addr = ioremap(NS92XX_FIM_IOHUB_BASE_PA + i*NS92XX_FIM_IOHUB_OFFSET,
				  NS92XX_FIM_IOHUB_SIZE);

	/* @FIXME: Unmap the already registered memory addresses */
	if (!pic->reg_addr || !pic->instr_addr || !pic->hwa_addr || !pic->iohub_addr) {
		printk_err("ioremap() failed by the PIC %i\n", pic->index);
		retval = -EAGAIN;
		goto exit_unmap;
	}

	spin_lock_init(&pic->lock);

	/* Create the file attributes under the sysfs */
	retval = 0;
	for (cnt=0; cnt < ARRAY_SIZE(fim_sysfs_attrs); cnt++) {
		retval = sysfs_create_bin_file(&dev->kobj, &fim_sysfs_attrs[cnt]);
		if (retval) {
			while (cnt)
				sysfs_remove_bin_file(&dev->kobj,
						      &fim_sysfs_attrs[--cnt]);
			goto exit_unmap;
		}
	}

	/* Set the function pointers for providing low level access to the PICs */
	pic->is_running = pic_is_running;
	pic->start_at_zero = pic_start_at_zero;
	pic->stop_and_reset = pic_stop_and_reset;
	pic->download_firmware = pic_download_firmware;
	pic->get_ctrl_reg = pic_get_ctrl_reg;
	pic->set_ctrl_reg = pic_set_ctrl_reg;
	pic->send_interrupt = pic_send_interrupt;
	pic->ack_interrupt = isr_from_pic;
	return 0;

 exit_unmap:
	if (pic->reg_addr) iounmap(pic->reg_addr);
	if (pic->instr_addr) iounmap(pic->instr_addr);
	if (pic->hwa_addr) iounmap(pic->hwa_addr);
	if (pic->iohub_addr) iounmap(pic->iohub_addr);
	pic->reg_addr = pic->instr_addr = pic->hwa_addr = pic->iohub_addr = NULL;

	return retval;
}

/* This function will be called for each PIC-device (but not for the parent device) */
static int fim_remove(struct device *dev)
{
	int retval, cnt;
	struct pic_t *pic = dev_get_drvdata(dev);
	if (!pic)
		return -EINVAL;

	printk_info("Starting to remove the PIC %s\n", dev_name(dev));

	if (pic->driver) {
		retval = fim_unregister_driver(pic->driver);
		if (retval) {
			printk_err("Can unregister the PIC %i, %i\n", pic->index,
				   retval);
		}
		pic->driver = NULL;
	}

	/* Remove the created sysfs attributes */
	for (cnt=0; cnt < ARRAY_SIZE(fim_sysfs_attrs); cnt++)
		sysfs_remove_bin_file(&dev->kobj, &fim_sysfs_attrs[cnt]);

	/* Free the mapped pages of the IO memory */
	if (pic->reg_addr) iounmap(pic->reg_addr);
	if (pic->instr_addr) iounmap(pic->instr_addr);
	if (pic->hwa_addr) iounmap(pic->hwa_addr);
	if (pic->iohub_addr) iounmap(pic->iohub_addr);
	pic->dev = NULL;
	return 0;
}

/* This function returns a non-zero value if the device correspond to our fim-bus */
static int fim_bus_match(struct device *dev, struct device_driver *driver)
{
	printk_debug("@TODO: Create a match mechanism for the FIMs.\n");
	return 1;
}

struct bus_type fim_bus_type = {
	.name = FIM_BUS_TYPE_NAME,
	.match = fim_bus_match,
};

/*
 * Bus parent that will be registered without calling the probe function
 * the sys file is under: /sys/devices/fims/
 */
struct device fim_bus_dev = {
	.release = fim_pic_release
};

static struct device_driver fims_driver = {
	.probe =  fim_probe,
	.remove = fim_remove,
	.name = FIM_DRIVER_NAME,
	.owner = THIS_MODULE,
	.bus = &fim_bus_type,
};

/*
 * These are the two PIC devices that we currently have in the NS9215
 * With the help of the `bus type' the probe function will be called when
 * the devices are registered
 * Sys path: /sys/bus/fim-bus/devices/fim[01..]
 */
static struct device fim_pics[] = {
	{
		.release = fim_pic_release,
		.bus = &fim_bus_type,
		.parent = &fim_bus_dev,
	},
	{
		.release = fim_pic_release,
		.bus = &fim_bus_type,
		.parent = &fim_bus_dev,
	}
};

/*
 * If no configuration for the DMA-buffer descriptors is passed (cfg = NULL),
 * then use the default values. These are defined under the header file 'fim.h'
 */
inline static int pic_dma_check_config(struct pic_t *pic, struct fim_dma_cfg_t *cfg)
{
	if (cfg) {
		if (cfg->rxnr <= 0 || cfg->rxnr > IOHUB_MAX_DMA_BUFFERS ||
		    cfg->txnr <= 0 || cfg->txnr > IOHUB_MAX_DMA_BUFFERS) {
			printk_err("Invalid number of RX/TX-DMA buffers (%i | %i)\n",
				   cfg->rxnr, cfg->txnr);
			return -EINVAL;
		}

		if (cfg->rxsz > IOHUB_MAX_DMA_LENGTH || cfg->rxsz <= 0 ||
		    cfg->txsz > IOHUB_MAX_DMA_LENGTH || cfg->txsz <= 0) {
			printk_err("Invalid DMA-buffer length (%i | %i)\n",
				   cfg->rxsz, cfg->txsz);
			return -EINVAL;
		}
	}

	pic->dma_cfg.rxnr = (!cfg) ? PIC_DMA_RX_BUFFERS : cfg->rxnr;
	pic->dma_cfg.txnr = (!cfg) ? PIC_DMA_TX_BUFFERS : cfg->txnr;
	pic->dma_cfg.rxsz = (!cfg) ? PIC_DMA_BUFFER_SIZE : cfg->rxsz;
	pic->dma_cfg.txsz = (!cfg) ? PIC_DMA_BUFFER_SIZE : cfg->txsz;
	return 0;
}

/*
 * This function starts the DMA-buffers and -fifos for the passed PIC
 * rxs   : Number of RX-buffer descriptors to use
 * txs   : Number of TX-buffer descriptors to create
 * rxlen : Length of each RX-DMA buffer
 * txlen : Length of each TX-DMA buffer
 * @IMPORTANT: No sanity check will be executed in this function
 */
static int pic_dma_init(struct pic_t *pic, struct fim_dma_cfg_t *cfg)
{
	int retval;
	struct iohub_dma_desc_t *desc;
	struct pic_dma_desc_t *pic_desc;
	int cnt;
	dma_addr_t phys_buffers;

	if (!pic || !pic->dev)
		return -EINVAL;

	/* Sanity checks for the passed DMA-descriptors configuration */
 	if ((retval = pic_dma_check_config(pic, cfg)))
		return retval;

	/* Get the DMA-memory for the RX- and TX-buffers */
	cfg = &pic->dma_cfg;
	pic->dma_size = (cfg->rxnr * cfg->rxsz) + (cfg->txnr * cfg->txsz);
	pic->dma_virt = dma_alloc_coherent(NULL, pic->dma_size,
					   &pic->dma_phys, GFP_KERNEL);
	if (!pic->dma_virt)
		return -ENOMEM;

	/*
	 * Get the DMA-memory for the RX-descriptors (FIFOs)
	 */
	pic->rx_fifo.descs = dma_alloc_coherent(NULL,
						cfg->rxnr * IOHUB_DMA_DESC_LENGTH,
						&pic->rx_fifo.phys_descs,
						GFP_KERNEL);
	if (!pic->rx_fifo.descs) {
		retval = -ENOMEM;
		goto free_dma_bufs;
	}

	fim_dma_init_fifo(&pic->rx_fifo, cfg->rxnr, pic->rx_fifo.descs);
	printk_debug("RX FIFO descriptors range is 0x%p to 0x%p [0x%08x]\n",
		     pic->rx_fifo.first, pic->rx_fifo.last, pic->rx_fifo.phys_descs);

	/* Setup the RX-descriptors with the corresponding DMA-buffers */
	phys_buffers = pic->dma_phys;
	desc = pic->rx_fifo.first;
	do {
		desc->length = cfg->rxsz;
		desc->src = phys_buffers;
		desc->control = IOHUB_DMA_DESC_CTRL_INT;
		printk_debug("[ RX %p ] 0x%p (0x%08x) | 0x%08x | 0x%04x\n",
			     desc, phys_to_virt(desc->src), desc->src, desc->length,
			     desc->control);
		phys_buffers += cfg->rxsz;
		desc = fim_dma_get_next(&pic->rx_fifo, desc);
	} while (desc != pic->rx_fifo.first);
	pic->rx_fifo.last->control |= IOHUB_DMA_DESC_CTRL_WRAP;

	/*
	 * Now create the DMA-memory for the TX-buffer descriptors
	 */
	pic->tx_fifo.descs = dma_alloc_coherent(NULL,
						cfg->txnr * IOHUB_DMA_DESC_LENGTH,
						&pic->tx_fifo.phys_descs,
						GFP_KERNEL);
	if (!pic->tx_fifo.descs) {
		retval = -ENOMEM;
		goto free_dma_rxfifo;
	}

	fim_dma_init_fifo(&pic->tx_fifo, cfg->txnr, pic->tx_fifo.descs);
	printk_debug("TX FIFO descriptors range is 0x%p to 0x%p [0x%08x]\n",
		     pic->tx_fifo.first, pic->tx_fifo.last, pic->tx_fifo.phys_descs);
	if(!(pic->tx_desc = kzalloc(cfg->txsz *
				    sizeof(struct pic_dma_desc_t), GFP_KERNEL))) {
		printk_err("Couldn't create the TX-dma descriptors\n");
		retval = -ENOMEM;
		goto free_dma_rxfifo;
	}

	/* Setup the TX-buffers and -descriptors */
	desc = pic->tx_fifo.first;
	cnt = 0;
	do {
		desc->length = cfg->txsz;
		desc->src = phys_buffers;
		desc->control = 0;
		pic_desc = pic->tx_desc + cnt++;
		pic_desc->src = phys_buffers;
		pic_desc->length = cfg->txsz;
		printk_debug("[ TX %p ] 0x%p (0x%08x) | 0x%08x | 0x%04x\n",
			     desc, phys_to_virt(desc->src), desc->src, desc->length,
			     desc->control);

		phys_buffers += cfg->txsz;
		desc = fim_dma_get_next(&pic->tx_fifo, desc);
	} while (desc != pic->tx_fifo.first);

	/* Set the last buffer descriptor with the WRAP bit */
	pic->tx_fifo.last->control = IOHUB_DMA_DESC_CTRL_WRAP;

	/*
	 * Enable the interrupts for the TX-DMA channel and set the buf desc pointer
	 * TXCAIP : used for flushing the TX-buffers
	 */
	spin_lock_init(&pic->tx_lock);
	writel(0x00, pic->iohub_addr + IOHUB_TX_ICTRL_REG);
	writel(IOHUB_TX_DMA_CTRL_CA, pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);
	writel(0x00, pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);
	writel(IOHUB_ICTRL_TXNCIE | IOHUB_ICTRL_TXNRIE | IOHUB_ICTRL_TXECIE |
	       IOHUB_IFS_TXCAIP, pic->iohub_addr + IOHUB_TX_ICTRL_REG);
	writel(pic->tx_fifo.phys_descs, pic->iohub_addr + IOHUB_TX_DMA_BUFPTR_REG);

	/* Enable the interrupts for the RX-DMA channel */
	spin_lock_init(&pic->rx_lock);
	writel(0x00, pic->iohub_addr + IOHUB_RX_DMA_ICTRL_REG);
	writel(IOHUB_RX_DMA_CTRL_CA, pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);
	writel(IOHUB_ICTRL_RXNCIE | IOHUB_ICTRL_RXNRIE | IOHUB_ICTRL_RXPCIE,
	       pic->iohub_addr + IOHUB_RX_DMA_ICTRL_REG);
	writel(pic->rx_fifo.phys_descs, pic->iohub_addr + IOHUB_RX_DMA_BUFPTR_REG);
	writel(IOHUB_RX_DMA_CTRL_CE, pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);

	return 0;

 free_dma_rxfifo:
	dma_free_coherent(NULL, pic->rx_fifo.length, pic->rx_fifo.descs,
			  pic->rx_fifo.phys_descs);
	pic->rx_fifo.descs = NULL;
	pic->rx_fifo.phys_descs = 0;

 free_dma_bufs:
	dma_free_coherent(NULL, pic->dma_size, pic->dma_virt, pic->dma_phys);
	pic->dma_virt = NULL;
	pic->dma_phys = 0;

	return retval;
}

static void pic_dma_stop(struct pic_t *pic)
{
	int cnt;
	struct pic_dma_desc_t *pic_desc;

	if (!pic || !pic->dev)
		return;

	printk_debug("Freeing the DMA resources of the PIC %i\n", pic->index);

	/* Disable all the interrupts and abort all the DMA-transfers */
	writel(0x00, pic->iohub_addr + IOHUB_TX_ICTRL_REG);
	writel(IOHUB_TX_DMA_CTRL_CA, pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);
	writel(0x00, pic->iohub_addr + IOHUB_TX_DMA_CTRL_REG);

	writel(0x00, pic->iohub_addr + IOHUB_RX_DMA_ICTRL_REG);
	writel(IOHUB_RX_DMA_CTRL_CA, pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);
	writel(0x00, pic->iohub_addr + IOHUB_RX_DMA_CTRL_REG);

	/* Reset the DMA-channel data */
	atomic_set(&pic->tx_tasked, 0);
	for (cnt=0; cnt < pic->tx_fifo.length; cnt++) {
		pic_desc = pic->tx_desc + cnt;
		atomic_set(&pic_desc->tasked, 0);
	}

	/* Free the internal TX-descriptors */
	if (pic->tx_desc) {
		kfree(pic->tx_desc);
		pic->tx_desc = NULL;
	}

	/* Free the FIFOs */
	if (pic->tx_fifo.descs && pic->tx_fifo.phys_descs) {
		dma_free_coherent(NULL, pic->tx_fifo.length, pic->tx_fifo.descs,
				  pic->tx_fifo.phys_descs);
		pic->tx_fifo.descs = NULL;
		pic->tx_fifo.phys_descs = 0;
	}

	if (pic->rx_fifo.descs && pic->rx_fifo.phys_descs) {
		dma_free_coherent(NULL, pic->rx_fifo.length, pic->rx_fifo.descs,
				  pic->rx_fifo.phys_descs);
		pic->rx_fifo.descs = NULL;
		pic->rx_fifo.phys_descs = 0;
	}

	/* Free the DMA-buffers */
	if (pic->dma_virt && pic->dma_phys) {
		dma_free_coherent(NULL, pic->dma_size, pic->dma_virt, pic->dma_phys);
		pic->dma_virt = NULL;
		pic->dma_phys = 0;
	}
}

int fim_dma_stop(struct fim_driver *fim)
{
	struct pic_t *pic;

	if (!(pic = get_pic_from_driver(fim)))
                return -ENODEV;

	pic_dma_stop(pic);
	return 0;
}

int fim_dma_start(struct fim_driver *fim, struct fim_dma_cfg_t *cfg)
{
	struct pic_t *pic;

	if (!(pic = get_pic_from_driver(fim)))
                return -ENODEV;

	/* Per default Use the already configured DMA configuration */
	if (!cfg)
		cfg = &pic->dma_cfg;

	return pic_dma_init(pic, cfg);
}

/*
 * @TODO: We wanted originally to separate the PICs and the virtual devices
 * initialization, but now we have a part of the PICs-init in this init-function
 */
static int __devinit fim_init_module(void)
{
	int ret, i;
	struct pic_t *pic;

	printk_info("Starting to register the FIMs module.\n");

	if (the_fims)
		return -EINVAL;
	if (!(the_fims = kzalloc(sizeof(struct fims_t), GFP_KERNEL))) {
		printk_err("kmalloc() failed.\n");
		return -ENOMEM;
	}

	/* Register the internal bus type */
	ret = bus_register(&fim_bus_type);
	if (ret) {
		printk_err("Registering the PICs bus, %i\n", ret);
		goto goto_free_mem;
	}

	/* Register the bus parent */
	dev_set_name(&fim_bus_dev, "fims");
	ret = device_register(&fim_bus_dev);
	if (ret) {
		printk_err( "Couldn't register the parent PIC device, %i\n", ret);
		goto goto_driver_unreg;
	}

	ret = driver_register(&fims_driver);
	if (ret) {
		printk_err("Couldn't register the FIMs driver, %i\n", ret);
		goto goto_bus_unreg;
	}

	/* Start registering the PIC devices */
	for (i=0; i <= FIM_MAX_PIC_INDEX; i++) {
		char fim_name[5];

		pic = &the_fims->pics[i];
		pic->index = i;
		pic->irq = IRQ_NS921X_PIC0 + i;
		sprintf(fim_name, "fim%d", i);
		dev_set_name(&fim_pics[i], fim_name);
		dev_set_drvdata(&fim_pics[i], pic);
		ret = device_register(&fim_pics[i]);
		if (ret) {
			printk_err("Registering the PIC device %i, %i\n", i, ret);
			while(i) device_unregister(&fim_pics[--i]);
			goto goto_parent_unreg;
		}
	}

	the_fims->driver = &fims_driver;
	the_fims->bus_dev = &fim_bus_dev;
	the_fims->bus_type = &fim_bus_type;
	printk(KERN_INFO DRIVER_DESC " " DRIVER_VERSION "\n");
	return 0;

 goto_parent_unreg:
	device_unregister(&fim_bus_dev);

 goto_driver_unreg:
	driver_unregister(&fims_driver);

 goto_bus_unreg:
	bus_unregister(&fim_bus_type);

 goto_free_mem:
	if(the_fims) kfree(the_fims);

	return ret;
}

static void __devexit fim_exit_module(void)
{
	int i;

	printk_info("Unregistering the FIMs module.\n");
	for (i=0; i <= FIM_MAX_PIC_INDEX; i++) {
		device_unregister(&fim_pics[i]);
	}

	device_unregister(&fim_bus_dev);
	driver_unregister(&fims_driver);

	bus_unregister(the_fims->bus_type);
	kfree(the_fims);
	the_fims = NULL;
}

module_init(fim_init_module);
module_exit(fim_exit_module);

EXPORT_SYMBOL(fim_register_driver);
EXPORT_SYMBOL(fim_unregister_driver);
EXPORT_SYMBOL(fim_get_exp_reg);
EXPORT_SYMBOL(fim_send_interrupt2);
EXPORT_SYMBOL(fim_enable_irq);
EXPORT_SYMBOL(fim_disable_irq);
EXPORT_SYMBOL(fim_send_buffer);
EXPORT_SYMBOL(fim_tx_buffers_room);
EXPORT_SYMBOL(fim_tx_buffers_level);
EXPORT_SYMBOL(fim_number_pics);
EXPORT_SYMBOL(fim_send_reset);
EXPORT_SYMBOL(fim_send_start);
EXPORT_SYMBOL(fim_send_stop);
EXPORT_SYMBOL(fim_flush_rx);
EXPORT_SYMBOL(fim_flush_tx);
EXPORT_SYMBOL(fim_alloc_buffer);
EXPORT_SYMBOL(fim_free_buffer);
EXPORT_SYMBOL(fim_set_ctrl_reg);
EXPORT_SYMBOL(fim_get_ctrl_reg);
EXPORT_SYMBOL(fim_get_stat_reg);
EXPORT_SYMBOL(fim_request_pic);
EXPORT_SYMBOL(fim_free_pic);
EXPORT_SYMBOL(fim_download_firmware);
EXPORT_SYMBOL(fim_is_running);
EXPORT_SYMBOL(fim_dma_stop);
EXPORT_SYMBOL(fim_dma_start);

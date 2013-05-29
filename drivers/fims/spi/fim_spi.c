/*
 * linux/drivers/spi/spi_fim.c
 *
 * Copyright (C) 2010 by Digi International Inc.
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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/stat.h>
#include <linux/kthread.h>
#include <mach/gpio.h>
#include <mach/spi.h>
#include <mach/fim-ns921x.h>
#include <linux/moduleparam.h>
#include "mach/fim-firmware.h"
#include "mach/fim-ns921x.h"
#include "../fim_reg.h"
#include "fim_spi.h"

#define DRIVER_NAME	 "fim-spi"

/*
 * Indexes for gpio_inuse array.
 */
#define MISO_GPIO_INDEX		(0)
#define MOSI_GPIO_INDEX		(1)
#define CLK_GPIO_INDEX		(2)
#define MASTER_CS_GPIO_INDEX	(3)
#define MAX_MASTER_GPIO		(4)

/*
 * Register addresses.
 */
#define MODE_REG		(0)
#define DELAY_LOOP_REG		(1)
#define CONTROL_FLAGS_REG	(9)
#define FIRMWARE_REVISION	(0)
#define ONE_BIT_CYCLES		(1)
#define BYTE_CYCLES		(2)
#define LOOP_CYCLES		(3)
#define LOOP_OVERHEAD		(4)
#define RESULT_REG		(5)

/*
 * Minimum SPI clock rate we can support.
 */
#define MIN_SPI_CLOCK_RATE	  (200000)

/*
 * q_status flags.
 */
#define Q_MESSAGE_AVAILABLE		(1)
#define Q_TRANSFER_COMPLETE		(2)
#define Q_TIME_TO_DIE			(4)
#define Q_DEAD				(8)

#define MAX_NAME_LENGTH			(40)

#define MAX_CS				(4)

struct spi_fim {
	struct spi_ns921x_fim   master_config;
	bool			gpio_inuse[MAX_MASTER_GPIO];
	bool			cs_inuse[MAX_CS];
	struct fim_driver	fim;
	spinlock_t		lock;
	struct list_head	message_queue;
	unsigned char		*rx_buffer;
	unsigned int		bytes_to_receive;
	wait_queue_head_t	wait_q;
	unsigned long int	q_status;
	unsigned long int	ahb_clock_rate;
	ulong			rx_err;
	ulong			tx_err;
	struct task_struct	*transfer_thread;
	char			driver_name[MAX_NAME_LENGTH];
	uint			dma_buffer_size;
	uint			number_dma_buffers;
	struct platform_device  *pdev;
	struct fim_dma_cfg_t	dma_cfg;
	unsigned int			max_speed;
};

#define NOT_SET		 (-2)
#define DISABLE_FEATURE (-1)

static int fims = 2;

static uint fim0_dma_buffer_size = 1024;
static uint fim0_number_dma_buffers = 16;
static int fim0_gpio_base=NOT_SET;
static int fim0_disable_master_cs=NOT_SET;
static int fim0_cs0=NOT_SET;
static int fim0_cs1=NOT_SET;
static int fim0_cs2=NOT_SET;
static int fim0_cs3=NOT_SET;
static int fim0_fim=NOT_SET;

static int fim1_load = 1;
static uint fim1_dma_buffer_size = 1024;
static uint fim1_number_dma_buffers = 16;
static int fim1_gpio_base=NOT_SET;
static int fim1_disable_master_cs=NOT_SET;
static int fim1_cs0=NOT_SET;
static int fim1_cs1=NOT_SET;
static int fim1_cs2=NOT_SET;
static int fim1_cs3=NOT_SET;
static int fim1_fim=NOT_SET;

module_param(fims, int, S_IRUGO);
module_param(fim0_dma_buffer_size, uint, S_IRUGO);
module_param(fim0_number_dma_buffers, uint, S_IRUGO);
module_param(fim0_gpio_base, int, S_IRUGO);
module_param(fim0_disable_master_cs, int, S_IRUGO);
module_param(fim0_cs0, int, S_IRUGO);
module_param(fim0_cs1, int, S_IRUGO);
module_param(fim0_cs2, int, S_IRUGO);
module_param(fim0_cs3, int, S_IRUGO);
module_param(fim0_fim, int, S_IRUGO);

module_param(fim1_load, int, S_IRUGO);
module_param(fim1_dma_buffer_size, uint, S_IRUGO);
module_param(fim1_number_dma_buffers, uint, S_IRUGO);
module_param(fim1_gpio_base, int, S_IRUGO);
module_param(fim1_disable_master_cs, int, S_IRUGO);
module_param(fim1_cs0, int, S_IRUGO);
module_param(fim1_cs1, int, S_IRUGO);
module_param(fim1_cs2, int, S_IRUGO);
module_param(fim1_cs3, int, S_IRUGO);
module_param(fim1_fim, int, S_IRUGO);

#define BUFFER_SIZE		PAGE_SIZE
#define INVALID_DMA_ADDRESS	0xffffffff
#define SPI_RX_IDONE		0x00000001
#define SPI_TX_IDONE		0x00000002

static unsigned int get_max_speed(struct spi_fim *info)
{
	unsigned int pic_clock_rate = info->ahb_clock_rate * 4;
	unsigned int cycles_per_bit = 0;

	fim_get_stat_reg(&info->fim, ONE_BIT_CYCLES, &cycles_per_bit);
	return (pic_clock_rate / cycles_per_bit);
}

/*
 * This routine is called with configuration parameters for a slave device.
 * We check the configuration parameters, but do not try to set them since
 * a transfer may be in progress when this function is called.
 *
 * It would seem that we should at least configure the GPIO pin here so that
 * we only have to do it once.  However, we configure it in assert_chip_select
 * instead for these reasons:
 *
 * 1. The Linux documentation states that this setup function may be called
 *	while a transfer is ongoing.  The user may be checking the configuration
 *	settings for a future transfer.  If we were to reconfigure the CS GPIO,
 *	which would include deasserting it, while a transfer was ongoing, that
 *	would interfere with the transfer.
 * 2. It is not clear who should own the GPIO pin.  Do we own it?  Or does
 *	the protocol driver own it?
 *
 * Return an error code if we don't like the configuration.
 */
static int spi_fim_setup(struct spi_device *spi)
{
	int result = 0;
	struct spi_fim *info = spi_master_get_devdata(spi->master);

	if (spi == NULL) {
		result = -EINVAL;
		printk(KERN_ERR "%s: spi_fim_setup spi_device parameter is NULL\n", info->driver_name);
		goto spi_fim_setup_quit;
	}

	if (spi->mode & (SPI_3WIRE | SPI_LOOP)) {
		printk(KERN_ERR "%s: spi_fim_setup unsupported SPI mode\n", info->driver_name);
		result = -EINVAL;
		goto spi_fim_setup_quit;
	}

	if ((spi->bits_per_word != 0) && (spi->bits_per_word != 8)) {
		printk(KERN_ERR "%s: spi_fim_setup unsupported word size\n", info->driver_name);
		result = -EINVAL;
		goto spi_fim_setup_quit;
	}

	if (spi->max_speed_hz > get_max_speed(info)) {
		spi->max_speed_hz = get_max_speed(info);
	}

	if (spi->max_speed_hz < MIN_SPI_CLOCK_RATE) {
		spi->max_speed_hz = MIN_SPI_CLOCK_RATE;
	}

	info->max_speed = spi->max_speed_hz;

	if (spi->chip_select >= MAX_CS) {
		printk(KERN_ERR "%s: spi_fim_setup chip select is invalid\n", info->driver_name);
		result = -EINVAL;
		goto spi_fim_setup_quit;
	}

spi_fim_setup_quit:
	return result;
}

/*
 * Called at the beginning of each SPI transfer to set the SPI mode.
 *
 * This function has the side effect of resetting the transfer rate back
 * to the maximum.  The set_speed function will be called seperately to
 * set the desired transfer speed.
 */
void configure_port(struct spi_fim *info, struct spi_device *dev)
{
#define FIM_SPI_MODE_MASK		   (3)
#define FIM_SPI_MODE_MSB_FIRST	  (4)
#define FIM_SPI_CONFIG_INTERRUPT	(1)
	unsigned int mode;

	mode = dev->mode & FIM_SPI_MODE_MASK;
	if ((dev->mode & SPI_LSB_FIRST) == 0)
		mode |= FIM_SPI_MODE_MSB_FIRST;

	fim_set_ctrl_reg(&info->fim, MODE_REG, mode);
	fim_set_ctrl_reg(&info->fim, CONTROL_FLAGS_REG, 0);	 /* clear all control flags*/
	fim_send_interrupt2(&info->fim, FIM_SPI_CONFIG_INTERRUPT);
}

/*
 * Compute the correct values for the delay loop counter given the user's desired speed
 * and the PIC clock rate.  Configure the FIM by writing the delay loop count into the
 * configuration registers.
 */
static void set_speed(struct spi_fim *info, unsigned int speed)
{
	unsigned int pic_clock_rate = info->ahb_clock_rate * 4;
	unsigned int fim_max_speed = 0;
	unsigned int cycles_per_bit = 0;
	unsigned int delay_loop_overhead = 0;
	unsigned int delay_loop_cycles = 0;
	unsigned int lsb = 0;

	fim_get_stat_reg(&info->fim, ONE_BIT_CYCLES, &cycles_per_bit);
	fim_get_stat_reg(&info->fim, LOOP_OVERHEAD, &delay_loop_overhead);
	fim_get_stat_reg(&info->fim, LOOP_CYCLES, &delay_loop_cycles);

	fim_max_speed = pic_clock_rate / cycles_per_bit;

	if (speed > info->max_speed) {
		speed = info->max_speed;
	}

	if (speed < MIN_SPI_CLOCK_RATE) {		   /* should never happen */
		speed = MIN_SPI_CLOCK_RATE;
	}

	if (speed < fim_max_speed) {
		/*
		 * If we come here, then the user selected a speed less than our top
		 * speed so we'll have to use delay loop counters.  The formula for
		 * converting a speed into a delay loop count is:
		 *
		 *	  count = (pic_rate/rate - cycles_per_bit - delay_loop_overhead) / delay_loop_cycles
		 *
		 * That formula has two problems.
		 *
		 *  1. It will give us a floating point value for a result, and the
		 *	 kernel doesn't do floating point.
		 *  2. When we round it, we want to make sure we always round up so
		 *	 that we never set a speed that is faster than the user requested.
		 *	 Larger delay_count values give slower speeds.
		 *
		 * To deal with these problems we:
		 *
		 *  1. Multiply the numerator by 100 before dividing by delay_loop_cycles.
		 *  2. Add 99 to the result to round up the delay count.
		 *  3. Divide back down by 100 to end up with the delay count we want
		 *	 to use which will never generate a speed greater than the user
		 *	 requested.
		 */
		unsigned long accumulator = pic_clock_rate / speed;

		if (accumulator > (cycles_per_bit + delay_loop_overhead)) {
			accumulator -= cycles_per_bit;
			accumulator -= delay_loop_overhead;
			accumulator *= 100;
			accumulator /= delay_loop_cycles;
			accumulator += 99;
			accumulator /= 100;
			if (accumulator > 0) {
				lsb = (unsigned int) (accumulator & 0xff);
			} else {
				lsb = 1;
			}
		} else {
			lsb = 1;
		}
	}

	fim_set_ctrl_reg(&info->fim, DELAY_LOOP_REG, lsb);
}

/*
 * This function asserts or deasserts a chip select.  The chip select is
 * a GPIO pin.
 *
 * 1.  Figure out whether we are setting the pin high or low.  Normally
 *	 CS is low active, so we would set it low to assert it.  However, if
 *	 the SPI_CS_HIGH bit is set in the mode flag, then CS is inverted.
 * 2.  Write the value to the pin.
 * 3.  Configure the pin.  The pin is configured after the value is set so
 *	 that the correct value will be output when the configuration
 *	 is set.
 *
 * We always reconfigure the pin whenever this function is called.  The logical
 * place to configure the GPIO pin would be in spi_fim_setup except that the
 * Linux SPI documentation states that the setup function may be called while
 * a SPI transfer is in progress, and setting the GPIO configuration and a
 * default value for the pin could cause errors in the transfer.
 */
static void assert_chip_select(struct spi_fim *info, struct spi_device *spi, bool do_assert)
{
	if ((spi->chip_select < MAX_CS) && (info->master_config.cs[spi->chip_select].enabled)) {
		int pin_value = do_assert ? 0 : 1;

		if (spi->mode & SPI_CS_HIGH) {
			pin_value = 1 - pin_value;
		}
		gpio_set_value(info->master_config.cs[spi->chip_select].gpio, pin_value);
		gpio_configure_ns921x_unlocked(info->master_config.cs[spi->chip_select].gpio,
					   NS921X_GPIO_OUTPUT,
					   NS921X_GPIO_DONT_INVERT,
					   NS921X_GPIO_FUNC_3,
					   NS921X_GPIO_DISABLE_PULLUP);
	}
}

/*
 * This routine is called by the transfer thread to process a SPI message.
 * A SPI message consists of a list of spi_transfer structures which each
 * have a pair of buffers to be transfered.  We go through the list and
 * process each transfer.
 *
 * Note that we send the buffers one at a time rather than just putting them
 * all onto the DMA ring at once.  The reason for this is that the Linux SPI
 * API allows the user to precisely control the CS line and the timing between
 * individual tranfers.  The only way we can support that is by transfering
 * the message one buffer at a time.
 *
 * Return 0 if all transfer succeeded
 * Return Q_TIME_TO_DIE if we are being shut down
 * Return Linux error code if some other problem occurred
 */
static int process_message(struct spi_fim *info, struct spi_message *message)
{
	int result = 0;
	struct spi_transfer *transfer = NULL;
	message->actual_length = 0;

	/*
	 * Some slave devices rely on a change in CS to begin the first transfer so
	 * let's make sure CS is deasserted before we start the transfer.  It should
	 * already be deasserted.
	 */
	assert_chip_select(info, message->spi, false);
	udelay(1);

	/*
	 * Now go through each set of buffers in the message and send them out.
	 */
	list_for_each_entry(transfer, &message->transfers, transfer_list) {
		struct fim_buffer_t fim_dma_buffer;

		if (transfer->tx_buf == NULL) {
			/*
			 * Linux SPI documentation says we must transmit zeroes if no transmit buffer
			 * is supplied.  We use the receive buffer for this and fill it with zeroes.
			 */
			memset(transfer->rx_buf, 0, transfer->len);
			fim_dma_buffer.data = transfer->rx_buf;
		} else {
			/*
			 * Else if the user is graciously supplying a transmit buffer, then use it.
			 */
			fim_dma_buffer.data = (unsigned char *) transfer->tx_buf;
		}
		info->rx_buffer = transfer->rx_buf;
		info->bytes_to_receive = transfer->len;
		fim_dma_buffer.length = transfer->len;
		fim_dma_buffer.private = info;
		fim_dma_buffer.sent = 0;

		spin_lock(&info->lock);
		info->q_status &= ~Q_TRANSFER_COMPLETE;
		spin_unlock(&info->lock);
		configure_port(info, message->spi);
		set_speed(info, transfer->speed_hz);
		assert_chip_select(info, message->spi, true);

		/*
		 * Start the DMA transfer.  The receive transfer will start also and call rx_isr
		 * when we have received a buffer of data.
		 */
		if (fim_send_buffer(&info->fim, &fim_dma_buffer) != 0) {
			message->status = -ENOMEM;
			goto process_message_quit;
		}

		/*
		 * Wait until rx_isr is called with that buffer of receive data.
		 */
		wait_event_interruptible(info->wait_q,
					 (info->q_status & (Q_TRANSFER_COMPLETE | Q_TIME_TO_DIE)));
		if (info->q_status & Q_TIME_TO_DIE) {
			message->status = -EIO;
			if ((info->q_status & Q_TRANSFER_COMPLETE) == 0) {
				fim_flush_tx(&info->fim);
				msleep(1);
				fim_flush_rx(&info->fim);
			}
			goto process_message_quit;
		}
		message->actual_length += transfer->len;

		/*
		 * Toggle CS if the user wants us to.
		 */
		if (transfer->cs_change)
			assert_chip_select(info, message->spi, false);

		/*
		 * Insert a delay if the user wants us to.
		 */
		if (transfer->delay_usecs > 1000) {
			msleep(transfer->delay_usecs / 1000);
			udelay(transfer->delay_usecs % 1000);
		} else if (transfer->delay_usecs > 0) {
			udelay(transfer->delay_usecs);
		}
	}
	message->status = 0;

process_message_quit:
	assert_chip_select(info, message->spi, false);	/* deassert CS after end of transfer*/
	message->complete(message->context);

	if (info->q_status & Q_TIME_TO_DIE) {
		result = Q_TIME_TO_DIE;
	}

	return result;
}

/*
 * This function is the entry point for the transfer thread.  This thread is
 * responsible for going through messages in the message queue and processing
 * them.
 *
 *  1. Wait for the next message to be put into the queue.
 *  2. Get that message and delete it from the queue.
 *  3. Process it.
 *
 * When the driver is unloaded, the unload will set the Q_TIME_TO_DIE flag and
 * wake us up.  When we see that, we flush any remaining messages in the queue
 * and terminate.
 */
static int transfer_thread_entry(void *data)
{
	struct spi_fim *info = (struct spi_fim *) data;
	struct spi_message *message = NULL;

	while((info->q_status & Q_TIME_TO_DIE) == 0) {
		int is_list_empty;

		/*
		 * Initialize q_status which spi_fim_transfer will set when it puts
		 * a new message onto the message queue.  Check to see if there is already
		 * a message in the queue.
		 */
		spin_lock(&info->lock);
		info->q_status &= ~Q_MESSAGE_AVAILABLE;
		is_list_empty = list_empty_careful(&info->message_queue);
		spin_unlock(&info->lock);

		if (is_list_empty) {
			/*
			 * If we come here, then the message queue was empty, so we wait until
			 * spi_fim_transfer wakes us up with a new message.
			 */
			wait_event_interruptible(info->wait_q,
						 (info->q_status & (Q_MESSAGE_AVAILABLE | Q_TIME_TO_DIE)));
			if (info->q_status & Q_TIME_TO_DIE) {
				goto transfer_thread_clean_up_and_quit;
			}
		}
		/*
		 * When we get here, the list will contain at least one message.  Pull it
		 * off of the list.
		 */
		spin_lock(&info->lock);
		message = list_first_entry(&info->message_queue, struct spi_message, queue);
		list_del(&message->queue);
		spin_unlock(&info->lock);

		/*
		 * And process the message.
		 */
		if (process_message(info, message) == Q_TIME_TO_DIE) {
			goto transfer_thread_clean_up_and_quit;
		}
	}

transfer_thread_clean_up_and_quit:
	/*
	 * Flush any left over messages.
	 */
	spin_lock(&info->lock);
	list_for_each_entry(message, &info->message_queue, queue) {
		spin_unlock(&info->lock);
		message->status = -EIO;			 /* have to put some error, this is best I can find */
		message->actual_length = 0;
		message->complete(message->context);
		spin_lock(&info->lock);
	}
	spin_unlock(&info->lock);
	info->q_status |= Q_DEAD;

	return 0;
}

/*
 * This is a callback function used by the Linux SPI driver API to start a transfer.
 */
static int spi_fim_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct spi_fim *info = spi_master_get_devdata(spi->master);
	struct spi_transfer *xfer;
	unsigned long flags;

	if (unlikely(list_empty(&msg->transfers) || !spi->max_speed_hz))
		return -EINVAL;

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (!(xfer->tx_buf || xfer->rx_buf)) {
			dev_dbg(&spi->dev, "%s:  SPI transfer request is missing rx or tx buf\n", __func__);
			return -EINVAL;
		}
		if (xfer->len > (info->dma_buffer_size * info->number_dma_buffers))
		{
				dev_dbg(&spi->dev, "%s:  SPI tranfer request buffer is too large\n", __func__);
				return -EINVAL;
		}
	}

	msg->status = -EINPROGRESS;
	msg->actual_length = 0;

	spin_lock_irqsave(&info->lock, flags);
	list_add_tail(&msg->queue, &info->message_queue);
	info->q_status |= Q_MESSAGE_AVAILABLE;
	spin_unlock_irqrestore(&info->lock, flags);
	wake_up_interruptible(&info->wait_q);

	return 0;
}

/*
 * This routine frees up any GPIO pins that we may have allocated.
 */
static void free_gpio_pins(struct spi_fim *info)
{
	struct spi_ns921x_fim *master_config = &info->master_config;
	int miso = master_config->gpio_base;
	int clk = master_config->gpio_base + 1;
	int mosi = master_config->gpio_base + 2;
	int cs = master_config->gpio_base + 3;
	int cs_index;

	master_config = &info->master_config;	/* avoid some typing*/

	if (info->gpio_inuse[MASTER_CS_GPIO_INDEX]) {
		gpio_free(cs);
		info->gpio_inuse[MASTER_CS_GPIO_INDEX] = false;
	}
	if (info->gpio_inuse[MISO_GPIO_INDEX]) {
		gpio_free(miso);
		info->gpio_inuse[MISO_GPIO_INDEX] = false;
	}
	if (info->gpio_inuse[MOSI_GPIO_INDEX]) {
		gpio_free(mosi);
		info->gpio_inuse[MOSI_GPIO_INDEX] = false;
	}
	if (info->gpio_inuse[CLK_GPIO_INDEX]) {
		gpio_free(clk);
		info->gpio_inuse[CLK_GPIO_INDEX] = false;
	}

	for (cs_index = 0; cs_index < MAX_CS; cs_index++)
	{
		if (info->cs_inuse[cs_index])
		{
			gpio_free(master_config->cs[cs_index].gpio);
			info->cs_inuse[cs_index] = false;
		}
	}
}

static int get_gpio_fn(int gpio, int fim)
{
#define BAD_CODE	(-1)
	unsigned int fn_code = BAD_CODE;

	if (fim == 0) {
		switch(gpio) {
			case 0:
			case 1:
			case 2:
			case 3:
				fn_code = 2;
				break;
			case 32:
			case 33:
			case 34:
			case 35:
				fn_code = 1;
				break;
#if defined(CONFIG_PROCESSOR_NS9215)
			case 68:
			case 69:
			case 70:
			case 71:
				fn_code = 0;
				break;
#endif
			default:
				fn_code = BAD_CODE;
				break;
		}
	} else if (fim == 1) {
		switch(gpio){
			case 26:
			case 27:
			case 28:
			case 29:
				fn_code = 2;
				break;
			case 40:
			case 41:
			case 42:
			case 43:
				fn_code = 1;
				break;
#if defined(CONFIG_PROCESSOR_NS9215)
			case 68:
			case 69:
			case 70:
			case 71:
				fn_code = 1;
				break;
#endif
			default:
				fn_code = BAD_CODE;
				break;
		}
	}

	return fn_code;
}


static int configure_gpio_pin(int gpio, int fim)
{
	int result = get_gpio_fn(gpio, fim);

	if (result != BAD_CODE) {
	gpio_configure_ns921x_unlocked(gpio,
					NS921X_GPIO_OUTPUT,
					NS921X_GPIO_DONT_INVERT,
					result,
					NS921X_GPIO_DISABLE_PULLUP);
		result = 0;
	}

	return result;
}

/*
 * This function gets ownership of the GPIO pins we need and then
 * configures them for the FIM.  The function returns in error if
 * a GPIO pin is not available.
 */
static int setup_gpio_pins(struct spi_fim *info)
{
	struct spi_ns921x_fim *master_config = &info->master_config;
	int retval = -1;
	int cs_index;
	int miso = master_config->gpio_base;
	int clk = master_config->gpio_base + 1;
	int mosi = master_config->gpio_base + 2;
	int cs = master_config->gpio_base + 3;

	/*
	 * Set up master CS if we are configured to use it.
	 */
	if (master_config->flags & SPI_NS921X_SUPPORT_MASTER_CS) {
		retval = gpio_request(cs, info->driver_name);
		if (retval != 0) {
			dev_err(&info->pdev->dev, "%s: GPIO pin %d for master CS is not available.\n", __func__, cs);
			goto setup_gpio_pins_terminate;
		}
		info->gpio_inuse[MASTER_CS_GPIO_INDEX] = true;
		retval = configure_gpio_pin(cs, master_config->fim_nr);
		if (retval == -1) {
			dev_err(&info->pdev->dev, "%s: Unable to configure GPIO pin %d.\n", __func__, cs);
			goto setup_gpio_pins_terminate;
		}
	}

	/*
	 * Set up MISO.
	 */
	retval = gpio_request(miso, info->driver_name);
	if (retval != 0) {
		dev_err(&info->pdev->dev, "%s: GPIO pin %d for MISO is not available.\n", __func__, miso);
		goto setup_gpio_pins_terminate;
	}
	info->gpio_inuse[MISO_GPIO_INDEX] = true;
	retval = configure_gpio_pin(miso, master_config->fim_nr);
	if (retval == -1) {
		dev_err(&info->pdev->dev, "%s: Unable to configure GPIO pin %d.\n", __func__, miso);
		goto setup_gpio_pins_terminate;
	}

	/*
	 * Set up MOSI.
	 */
	retval = gpio_request(mosi, info->driver_name);
	if (retval != 0) {
		dev_err(&info->pdev->dev, "%s: GPIO pin %d for MOSI is not available.\n", __func__, mosi);
		goto setup_gpio_pins_terminate;
	}
	info->gpio_inuse[MOSI_GPIO_INDEX] = true;
	retval = configure_gpio_pin(mosi, master_config->fim_nr);
	if (retval == -1) {
		dev_err(&info->pdev->dev, "%s: Unable to configure GPIO pin %d.\n", __func__, mosi);
		goto setup_gpio_pins_terminate;
	}

	/*
	 * Set up CLK.
	 */
	retval = gpio_request(clk, info->driver_name);
	if (retval != 0) {
		dev_err(&info->pdev->dev, "%s: GPIO pin %d for SPI CLK is not available.\n", __func__, clk);
		goto setup_gpio_pins_terminate;
	}
	info->gpio_inuse[CLK_GPIO_INDEX] = true;
	retval = configure_gpio_pin(clk, master_config->fim_nr);
	if (retval == -1) {
		dev_err(&info->pdev->dev, "%s: Unable to configure GPIO pin %d.\n", __func__, clk);
		goto setup_gpio_pins_terminate;
	}

	for (cs_index = 0; cs_index < MAX_CS; cs_index++)
	{
		if (master_config->cs[cs_index].enabled)
		{
			/*
			 * At this point we don't know whether the CS is high active or low active,
			 * so we don't know what to do with the pin yet.  IMHO this is a bug in the
			 * design of the Linux SPI API.  Anyway, we just get ownership of the pin
			 * for now.  We will configure it when we send the first message out, because
			 * that is when we are told what the polarity of the pin is.
			 */
			retval = gpio_request(master_config->cs[cs_index].gpio, info->driver_name);
			if (retval != 0) {
				printk(KERN_ERR "%s: setup_gpio_pins unable to allocate GPIO pin %d for chip select %d\n",
						info->driver_name, master_config->cs[cs_index].gpio, cs_index);
				goto setup_gpio_pins_terminate;
			}
			info->cs_inuse[cs_index] = true;
		}
		else
		{
			info->cs_inuse[cs_index] = false;
		}
	}

	retval = 0;

setup_gpio_pins_terminate:
	/*
	 * The caller will call free_gpio_pins() to clean up if we return an error.
	 */
	return retval;
}

/*
 * This routine is called when the DMA RX transfer completes.  Copy the DMA receive
 * buffer into the caller's buffer and then wake up the transfer thread.
 */
void rx_isr(struct fim_driver *fim, int not_used, struct fim_buffer_t *fim_buffer)
{
	struct spi_fim *info = container_of(fim, struct spi_fim, fim);

	(void) not_used;

	if (info->bytes_to_receive == 0) {
		dev_err(&info->pdev->dev, "%s:  rx_isr called when bytes_to_receive == 0\n", __func__);
		return;
	} else if (fim_buffer->length > info->bytes_to_receive) {
		dev_err(&info->pdev->dev, "%s:  received %d bytes, but only expected %d\n", __func__,
			fim_buffer->length, info->bytes_to_receive);
	}

	if (info->rx_buffer != NULL) {
		memcpy(info->rx_buffer, fim_buffer->data, fim_buffer->length);
		info->rx_buffer += fim_buffer->length;
	}

	if (fim_buffer->length <= info->bytes_to_receive) {
		info->bytes_to_receive -= fim_buffer->length;
	} else {
		info->bytes_to_receive = 0;
	}

	spin_lock(&info->lock);
	if (info->bytes_to_receive == 0) {
		info->q_status |= Q_TRANSFER_COMPLETE;
		info->rx_buffer = NULL;
		wake_up_interruptible(&info->wait_q);
	}
	spin_unlock(&info->lock);
}

/*
 * This callback is executed if a DMA error interrupt occurs.
 */
static void dma_error_isr(struct fim_driver *fim, ulong rx_err, ulong tx_err)
{
	struct spi_fim *info = container_of(fim, struct spi_fim, fim);

	dev_err(&info->pdev->dev, "%s:  rx_err = %lu, rx_err = %lu\n", __func__,
		rx_err, tx_err);

	info->rx_err = rx_err;
	info->tx_err = tx_err;
}

/*
 * This routine registers the driver with the FIM API.  We specify what firmware
 * to use and callbacks for the ISR handlers.
 */
static int register_with_fim_api(struct spi_fim *info, struct device *dev)
{
#define FIM_FIRMWARE_FILENAME   "fim_spi.bin"
#define EXPECTED_FIRMWARE_REV   (1)
	int result = 0;
	unsigned int firmware_rev = 0;

	info->fim.picnr = info->master_config.fim_nr;

	/*
	 * When loading as a module, download the firmware from a file.  If we are running
	 * as a kernel driver, then use the internal binary image built as part of the
	 * driver.
	 */
#if defined(MODULE)
	info->fim.fw_code = NULL;
	info->fim.fw_name = FIM_FIRMWARE_FILENAME;
#else
	info->fim.fw_code = fim_SPI_firmware;
	info->fim.fw_name = NULL;
#endif

	info->fim.driver.name = info->driver_name;
	info->fim.dev = dev;
	info->fim.fim_isr = NULL;
	info->fim.dma_tx_isr = NULL;
	info->fim.dma_rx_isr = rx_isr;
	info->fim.dma_error_isr = dma_error_isr;
	info->fim.driver_data = info;
	info->dma_cfg.rxnr = info->number_dma_buffers;
	info->dma_cfg.txnr = info->number_dma_buffers;
	info->dma_cfg.rxsz = info->dma_buffer_size;
	info->dma_cfg.txsz = info->dma_buffer_size;

	info->fim.dma_cfg = &info->dma_cfg;
	info->fim.verbose = 1;

	result = fim_register_driver(&info->fim);
	if (result != 0) {
		dev_err(&info->pdev->dev, "%s:  fim_register_driver returned %d\n", __func__, result);
		goto register_with_fim_api_register_failed;
	}

	msleep(100);

	fim_get_stat_reg(&info->fim, FIRMWARE_REVISION, &firmware_rev);
	if (firmware_rev != EXPECTED_FIRMWARE_REV)
	{
		dev_err(&info->pdev->dev, "%s:  fim_send_start got firmware rev %u, expected %u\n", __func__, firmware_rev, EXPECTED_FIRMWARE_REV);
		goto register_with_fim_api_register_failed;
	}
	fim_enable_irq(&info->fim);
	info->max_speed = get_max_speed(info);

register_with_fim_api_register_failed:
	return result;
}

#if defined(MODULE)
/*
 * Process module parameters here.  As a driver, we are handed a set of platform parameters.
 * If we are loaded as a module, then the user can specify command line parameters
 * to override the defaults.  The parameters are defined internally as static
 * variables set to NOT_SET.  The module loader will load values from the command
 * line to override the NOT_SET value if the user provides any.  We check for that
 * and update our configuration settings accordingly.
 */
static void handle_module_parameters(struct spi_fim *info)
{
	struct spi_ns921x_fim *master = &info->master_config;

	if (master->fim_nr == 0) {
	info->dma_buffer_size = fim0_dma_buffer_size;
	info->number_dma_buffers = fim0_number_dma_buffers;
		if (fim0_gpio_base != NOT_SET) {
			master->gpio_base = fim0_gpio_base;
		}
		if (fim0_disable_master_cs != NOT_SET) {
			master->flags &= ~SPI_NS921X_SUPPORT_MASTER_CS;
			master->flags |= fim0_disable_master_cs ? 0 : SPI_NS921X_SUPPORT_MASTER_CS;
		}
		if (fim0_cs0 != NOT_SET) {
			if (fim0_cs0 == DISABLE_FEATURE) {
				master->cs[0].enabled = false;
			} else {
				master->cs[0].enabled = true;
				master->cs[0].gpio = fim0_cs0;
			}
		}
		if (fim0_cs1 != NOT_SET) {
			if (fim0_cs1 == DISABLE_FEATURE) {
				master->cs[1].enabled = false;
			} else {
				master->cs[1].enabled = true;
				master->cs[1].gpio = fim0_cs1;
			}
		}
		if (fim0_cs2 != NOT_SET) {
			if (fim0_cs2 == DISABLE_FEATURE) {
				master->cs[2].enabled = false;
			} else {
				master->cs[2].enabled = true;
				master->cs[2].gpio = fim0_cs2;
			}
		}
		if (fim0_cs3 != NOT_SET) {
			if (fim0_cs3 == DISABLE_FEATURE) {
				master->cs[3].enabled = false;
			} else {
				master->cs[3].enabled = true;
				master->cs[3].gpio = fim0_cs3;
			}
		}
	}
	else {
		info->dma_buffer_size = fim1_dma_buffer_size;
		info->number_dma_buffers = fim1_number_dma_buffers;
		if (fim1_gpio_base != NOT_SET) {
			master->gpio_base = fim1_gpio_base;
		}
		if (fim1_disable_master_cs != NOT_SET) {
			master->flags &= ~SPI_NS921X_SUPPORT_MASTER_CS;
			master->flags |= fim1_disable_master_cs ? 0 : SPI_NS921X_SUPPORT_MASTER_CS;
		}
		if (fim1_cs0 != NOT_SET) {
			if (fim1_cs0 == DISABLE_FEATURE) {
				master->cs[0].enabled = false;
			} else {
				master->cs[0].enabled = true;
				master->cs[0].gpio = fim1_cs0;
			}
		}
		if (fim1_cs1 != NOT_SET) {
			if (fim1_cs1 == DISABLE_FEATURE) {
				master->cs[1].enabled = false;
			} else {
				master->cs[1].enabled = true;
				master->cs[1].gpio = fim1_cs1;
			}
		}
		if (fim1_cs2 != NOT_SET) {
			if (fim1_cs2 == DISABLE_FEATURE) {
				master->cs[2].enabled = false;
			} else {
				master->cs[2].enabled = true;
				master->cs[2].gpio = fim1_cs2;
			}
		}
		if (fim1_cs3 != NOT_SET) {
			if (fim1_cs3 == DISABLE_FEATURE) {
				master->cs[3].enabled = false;
			} else {
				master->cs[3].enabled = true;
				master->cs[3].gpio = fim1_cs3;
			}
		}
	}
}
#endif


/*
 * This routine is called to install the driver.  We initialize the resources
 * and memory that we need.
 */
static __devinit int spi_fim_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct spi_fim *info;
	struct spi_ns921x_fim *master_config;
	int ret = -ENOMEM;
	struct clk *ahb_clock;

	/*
	 * Allocate the spi_master structure as well as our private data
	 * structure.
	 */
	printk(KERN_INFO "FIM SPI device driver probe routine called\n");
	master = spi_alloc_master(&pdev->dev, sizeof(*info));
	if (!master) {
		printk(KERN_ERR "FIM SPI Probe routine cannot allocate spi master structure\n");
		return -ENOMEM;
	}

	/*
	 * Set up the spi_master structure as the private data in the
	 * platform_device structure.
	 */
	platform_set_drvdata(pdev, master);

	/* Save pointer to configuration settings setup in platform files */
	master_config = pdev->dev.platform_data;

#if defined(MODULE)
	/*
	 * User can control which FIM we start through the command line.  I don't really like the syntax
	 * of this parameter, but this is what the other FIM drivers do so we will do the same for
	 * consistency.
	 *
	 *	  fims=0	  load SPI driver on FIM0
	 *	  fims=1	  load SPI driver on FIM1
	 *	  fims=2	  load SPI driver on both FIMs.
	 */
	if ((master_config->fim_nr == 0) && ((fims != 0) && (fims != 2))) {
		printk(KERN_ERR "FIM SPI driver not loading for FIM0 because fims=%d on command line.\n", fims);
		goto probe_not_wanted;
	} else if ((master_config->fim_nr == 1) && ((fims != 1) && (fims != 2))) {
		printk(KERN_ERR "FIM SPI driver not loading for FIM1 because fims=%d on command line.\n", fims);
		goto probe_not_wanted;
	}
#endif
	/*
	 * Get a pointer to our real private data.
	 */
	info = spi_master_get_devdata(master);
	memset(info, 0, sizeof(*info));
	info->master_config = *master_config;
#if defined(MODULE)
	handle_module_parameters(info);
#else
	if (master_config->fim_nr == 0) {
		info->dma_buffer_size = fim0_dma_buffer_size;
		info->number_dma_buffers = fim0_number_dma_buffers;
	} else {
		info->dma_buffer_size = fim1_dma_buffer_size;
		info->number_dma_buffers = fim1_number_dma_buffers;
	}
#endif
	if ((master_config->fim_nr != 0) && (master_config->fim_nr != 1)) {
		printk(KERN_ERR "%s passed invalid fim number %d.\n", __func__, master_config->fim_nr);
		goto probe_bad_fim_number;
	}
	sprintf(info->driver_name, "%s-%d", DRIVER_NAME, master_config->fim_nr);
	info->pdev = pdev;		  /* save pointer to master device structure*/

	master->bus_num = pdev->id;
	/* hardware controlled cs */
	master->num_chipselect = MAX_CS;
	/* the spi->mode bits understood by this driver: */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST;

	master->setup = spi_fim_setup;
	master->transfer = spi_fim_transfer;

	/* prepare waitq and spinlock */
	spin_lock_init(&info->lock);
	INIT_LIST_HEAD(&info->message_queue);
	init_waitqueue_head(&info->wait_q);

	/*
	 * Get the AHB clock rate.  We need it to compute timing.
	 */
	ahb_clock = clk_get(NULL, "ahbclock");
	if (IS_ERR(ahb_clock)) {
		dev_err(&pdev->dev, "%s:  Unable to get AHB clock source.\n", __func__);
		ret = PTR_ERR(ahb_clock);
		goto probe_no_clock;
	}
	info->ahb_clock_rate = clk_get_rate(ahb_clock);
	clk_put(ahb_clock);				 /* we're done with it now */

	ret = register_with_fim_api(info, &pdev->dev);
	if (ret != 0) {
		dev_err(&pdev->dev, "%s:  Unable to register with FIM API.\n", __func__);
		goto probe_cant_register_with_fim_driver;
	}

	/*
	 * Configure the GPIOs that we will be using.
	 */
	if (setup_gpio_pins(info) != 0) {
		/*
		 * One or more of the GPIO pins wasn't available.  Report an
		 * out of resources error.
		 */
		ret = -ENOMEM;
		dev_err(&pdev->dev, "%s: One or more required GPIO pins are not available.\n",
								__func__);
		dev_err(&pdev->dev, "%s: Driver terminating.\n", __func__);
		goto probe_missing_gpio;
	}

	info->q_status = 0;
	info->transfer_thread = kthread_run(transfer_thread_entry, info, info->driver_name);
	if (info->transfer_thread == NULL) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "%s: kthread_run() failed to start transfer thread.\n", __func__);
		goto probe_cant_create_thread;
	}

	ret = spi_register_master(master);
	if (ret) {
		dev_err(&pdev->dev, "%s : cannot register spi master\n", __func__);
		goto probe_cant_register;
	}

	return 0;

probe_cant_register:
	spin_lock(&info->lock);
	info->q_status |= Q_TIME_TO_DIE;
	spin_unlock(&info->lock);
	wake_up_interruptible(&info->wait_q);
probe_cant_create_thread:
probe_missing_gpio:
	free_gpio_pins(info);
	fim_disable_irq(&info->fim);
	fim_unregister_driver(&info->fim);
probe_cant_register_with_fim_driver:
probe_no_clock:
probe_bad_fim_number:
#if defined(MODULE)
	probe_not_wanted:
#endif
	spi_master_put(master);

	return ret;
}

/*
 * This routine unloads the driver.
 */
static __devexit int spi_fim_remove(struct platform_device *pdev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct spi_fim *info = spi_master_get_devdata(master);
	int result;
	int i;

	printk(KERN_ERR "spi_fim_remove called\n");
	spin_lock(&info->lock);
	info->q_status |= Q_TIME_TO_DIE;
	spin_unlock(&info->lock);
	wake_up_interruptible(&info->wait_q);
	for (i = 0; i < 10; i++) {
		msleep(10);
		if (info->q_status & Q_DEAD)
			break;
	}
	if ((info->q_status & Q_DEAD) == 0) {
		dev_err(&pdev->dev, "%s: transfer thread never set Q_DEAD\n", __func__);
	}

	free_gpio_pins(info);
	fim_disable_irq(&info->fim);
	result = fim_unregister_driver(&info->fim);
	if (result != 0) {
		dev_err(&pdev->dev, "%s: fim_unregister_driver returned %d.\n", __func__, result);
	}
	spi_master_put(master);
	dev_err(&pdev->dev, "%s: driver has unloaded.\n", __func__);

	spi_unregister_master(master);
	spi_master_put(master);

	return 0;
}

static struct platform_driver spi_ns9xxx_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= spi_fim_probe,
	.remove		= __devexit_p(spi_fim_remove),
};

static __init int spi_fim_init(void)
{

	return platform_driver_register(&spi_ns9xxx_driver);
}

static __exit void spi_fim_exit(void)
{
	platform_driver_unregister(&spi_ns9xxx_driver);
}

module_init(spi_fim_init);
module_exit(spi_fim_exit);

MODULE_AUTHOR("Digi International Inc.");
MODULE_DESCRIPTION("Digi ns921x FIM SPI Controller driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);

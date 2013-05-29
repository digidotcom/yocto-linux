/* -*- linux-c -*-
 *
 * drivers/fims/fim_sdio.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 *  !Revision:   $Revision: 1.20 $
 *  !Author:     Luis Galdos
 *  !Descr:
 *  !References:
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/scatterlist.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/irq.h>

#include <asm/scatterlist.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <mach/gpio.h>
#include <mach/regs-sys-ns921x.h>
#include <mach/regs-iohub-ns921x.h>
#include <mach/regs-io-ns921x.h>
#include <asm/delay.h>


/* For registering the FIM-driver */
#include <mach/fim-ns921x.h>


/*
 * If the driver is going to be loaded as a built-in driver, then include the header
 * file with the firmware, otherwise set the name of the binary file that should
 * be read with the help of the firmware-subsystem
 */
#if !defined(MODULE)
# if defined(CONFIG_PROCESSOR_NS9215)
#  include "fim_sdio0.h"
#  include "fim_sdio1.h"
# elif defined(CONFIG_PROCESSOR_NS9210)
#  include "fim_sdio0_9210.h"
#  include "fim_sdio1_9210.h"
# else
#  error "FIM-SDIO: Unsupported processor type."
# endif /* CONFIG_PROCESSOR_NS9215 */
extern const unsigned char fim_sdio_firmware[];
#define FIM_SDIO_FW_FILE			(NULL)
#define FIM_SDIO_FW_CODE0			fim_sdio_firmware0
#define FIM_SDIO_FW_CODE1			fim_sdio_firmware1
#else
const unsigned char *fim_sdio_firmware = NULL;
# if defined(CONFIG_PROCESSOR_NS9215)
#  define FIM_SDIO_FW_FILE_PAT			"fim_sdio%i.bin"
# elif defined(CONFIG_PROCESSOR_NS9210)
#  define FIM_SDIO_FW_FILE_PAT			"fim_sdio%i_9210.bin"
# else
#  error "FIM-SDIO: Unsupported processor type."
# endif /* CONFIG_PROCESSOR_NS9215 */
#define FIM_SDIO_FW_LEN				sizeof(FIM_SDIO_FW_FILE_PAT) + 10
#define FIM_SDIO_FW_CODE			(NULL)
#endif

/* Driver informations */
#define DRIVER_VERSION				"0.4"
#define DRIVER_AUTHOR				"Digi International"
#define DRIVER_DESC				"FIM SDIO driver"
#define FIM_SDIO_DRIVER_NAME			"fim-sdio"
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

/* Module parameter for selection the FIMs */
NS921X_FIM_NUMBERS_PARAM(fims_number);
/* Other module parameters */
#if defined(MODULE)
#if defined(CONFIG_FIM_APPKIT_BOARD)
#if defined(CONFIG_MACH_CME9210) || defined(CONFIG_MACH_CME9210JS) || \
    defined(CONFIG_MACH_CWME9210) || defined(CONFIG_MACH_CWME9210JS)
static int cd0_gpio = 9;
static int cd0_gpio_func = NS921X_GPIO_FUNC_2;
static int wp0_gpio = 6;
static int cd1_gpio = 9;
static int cd1_gpio_func = NS921X_GPIO_FUNC_2;
static int wp1_gpio = 6;
#else
static int cd0_gpio = 101;
static int cd0_gpio_func = NS921X_GPIO_FUNC_2;
static int wp0_gpio = 100;
static int cd1_gpio = 101;
static int cd1_gpio_func = NS921X_GPIO_FUNC_2;
static int wp1_gpio = 100;
#endif /* defined(CONFIG_MACH_CME9210) || defined(CONFIG_MACH_CME9210JS) ||
	  defined(CONFIG_MACH_CWME9210) || defined(CONFIG_MACH_CWME9210JS) */
#else
static int cd0_gpio = FIM_GPIO_DONT_USE;
static unsigned int cd0_gpio_func = NS921X_GPIO_FUNC_GPIO;
static int wp0_gpio = FIM_GPIO_DONT_USE;
static int cd1_gpio = FIM_GPIO_DONT_USE;
static unsigned int cd1_gpio_func = NS921X_GPIO_FUNC_GPIO;
static int wp1_gpio = FIM_GPIO_DONT_USE;
#endif
module_param_named(cd0, cd0_gpio, int, 0644);
module_param_named(cd0_func, cd0_gpio_func, int, 0644);
module_param_named(wp0, wp0_gpio, int, 0644);
module_param_named(cd1, cd1_gpio, int, 0644);
module_param_named(cd1_func, cd1_gpio_func, int, 0644);
module_param_named(wp1, wp1_gpio, int, 0644);
#endif

/* Registers with status information */
#define FIM_SDIO_GPIOS_REG			0x02
#define FIM_SDIO_GPIOS_REG_CD			0x01
#define FIM_SDIO_GPIOS_REG_WP			0x02
#define FIM_SDIO_CARD_STATREG			0x00

/* Interrupts from the FIM to the driver */
#define FIM_SDIO_INTARM_CARD_DAT1		0x01
#define FIM_SDIO_INTARM_CARD_DETECTED		0x02


/* Macros for the SDIO-interface to the FIM-firmware */
#define SDIO_HOST_TX_HDR			0x40
#define SDIO_HOST_CMD_MASK			0x3f

/* User specified macros */
//#define FIM_SDIO_TIMEOUT_MS			500
#define FIM_SDIO_TIMEOUT_MS			1000
#define FIM_SDIO_TX_CMD_LEN			5
#define FIM_SDIO_MAX_RESP_LENGTH		17

/* Status bits from the PIC-firmware */
#define FIM_SDIO_RX_RSP				0x01
#define FIM_SDIO_RX_BLKRD				0x02
#define FIM_SDIO_RX_TIMEOUT			0x04

/*
 * Firmware specific control registers
 */
#define FIM_SDIO_CLKDIV_REG			0	/* Clock delay */

#define FIM_SDIO_CONTROL1_REG			1
# define FIM_SDIO_CONTROL1_INTR_EN		0x01	/* 0: disable, 1: enable */
# define FIM_SDIO_CONTROL1_BUS_WIDTH4		0x02	/* 0: 1-bit mode, 1: 4-bit mode */
# define FIM_SDIO_CONTROL1_HIGH_SPEED		0x04	/* 0: use clock set in CONTROL0, 1: ignore clock set in CONTROL0 */
# define FIM_SDIO_CONTROL1_DISABLE_CRC		0x08	/* 0: enable, 1: disable */
# define FIM_SDIO_CONTROL1_STOP			0x10	/* stop the current multiblock transaction */
# define FIM_SDIO_CONTROL1_START		0x80	/* start firmware running */

#define FIM_SDIO_CONTROL2_REG			2
# define FIM_SDIO_CONTROL2_RESP48		0x01	/* expected 48 bits response */
# define FIM_SDIO_CONTROL2_RESP136		0x02	/* expected 136 bits response */
# define FIM_SDIO_CONTROL2_BLK_READ		0x04	/* block read command */
# define FIM_SDIO_CONTROL2_BLK_WRITE		0x08	/* block write command */
# define FIM_SDIO_CONTROL2_CHECK_BUSY		0x10	/* wait until the card is not busy after current command respone */
# define FIM_SDIO_CONTROL2_SKIP_CRC		0x20	/* skip CRC for a command respone, use it for R2 and R3 */
# define FIM_SDIO_CONTROL2_WAIT_FOR_CMD		0x40	/* after executing the current command, FIM waits for a new command to start the clock */

#define FIM_SDIO_BLKSZ_LSB_REG			3	/* LSB of the block/data size configured in bytes */
#define FIM_SDIO_BLKSZ_MSB_REG			4	/* MSB of the block/data size configured in bytes */
#define FIM_SDIO_BLKCNT_LSB_REG			5	/* LSB of the block count (set this to 1 if it is single block transfer */
#define FIM_SDIO_BLKCNT_MSB_REG			6	/* MSB of the block count */

/*
 * Firmware specific status registers
 */
#define FIM_SDIO_FIMVER_REG			0	/* Firmware version number */
#define FIM_SDIO_PORTEXP1_REG			1
# define FIM_SDIO_PORTEXP1_RESP			0x01	/* received data is respone */
# define FIM_SDIO_PORTEXP1_READ			0x02	/* received data is read data */
# define FIM_SDIO_PORTEXP1_TIMEOUT		0x04	/* no response received */
# define FIM_SDIO_PORTEXP1_CRCERR_CMD		0x08	/* CRC error in command response */
# define FIM_SDIO_PORTEXP1_CRCERR_DATA		0x10	/* CRC error in read data */
#define FIM_SDIO_DATAWR_STATUS_REG		2	/* Data write status reg. Will be updated with the CRC status after each block completion */
# define FIM_SDIO_DATAWR_STATUS_NOT_RDY		0x00	/* Not ready / In progress */
# define FIM_SDIO_DATAWR_STATUS_SUCCESS		0x02	/* Success */
# define FIM_SDIO_DATAWR_STATUS_NEG_CRC		0x05	/* Negative CRC */
# define FIM_SDIO_DATAWR_STATUS_NO_CRC		0x07	/* No CRC status response */

/* Internal flags for the request function */
#define FIM_SDIO_FLAG_R2_RESPONSE		0x01

/* Macros for the DMA-configuraton */
#define FIM_SDIO_DMA_BUFFER_SIZE		PAGE_SIZE
#define FIM_SDIO_DMA_RX_BUFFERS			21
#define FIM_SDIO_DMA_TX_BUFFERS			10

#define FIM_SDIO_MAXBLKSIZE 		4095
#define FIM_SDIO_BUFSIZE 		FIM_SDIO_DMA_BUFFER_SIZE
#define FIM_SDIO_BLKATONCE 		FIM_SDIO_DMA_TX_BUFFERS

/* Used for the Card Detect timer */
#define FIM_SDIO_CD_POLLING_TIMER			(HZ / 2)

/*
 * The below macro force the use of the timer for polling the state of the card
 * detection line
 */
#if 0
#define FIM_SDIO_FORCE_CD_POLLING
#endif

#define printk_err(fmt, args...)                printk(KERN_ERR "[ ERROR ] fim-sdio: " fmt, ## args)
#define printk_info(fmt, args...)               printk(KERN_INFO "fim-sdio: " fmt, ## args)
#define printk_dbg(fmt, args...)                printk(KERN_DEBUG "fim-sdio: " fmt, ## args)

#if 0
#define FIM_SDIO_DEBUG
#endif

#ifdef FIM_SDIO_DEBUG
#  define printk_debug(fmt, args...)		printk(KERN_DEBUG "fim-sdio: %s() " fmt, __FUNCTION__ , ## args)
#else
#  define printk_debug(fmt, args...)
#endif

/*
 * GPIO configuration
 * Please note that the write protect GPIO must be under the index [4], then
 * the driver will read its status (the firmware doesn't support it)
 */
#define FIM_SDIO_D0_GPIO			0
#define FIM_SDIO_D1_GPIO			1
#define FIM_SDIO_D2_GPIO			2
#define FIM_SDIO_D3_GPIO			3
#define FIM_SDIO_WP_GPIO			4
#define FIM_SDIO_CD_GPIO			5
#define FIM_SDIO_CLK_GPIO			6
#define FIM_SDIO_CMD_GPIO			7
#define FIM_SDIO_MAX_GPIOS			8

/* Values for the block read state machine */
enum fim_blkrd_state {
	BLKRD_STATE_IDLE		= 0,
	BLKRD_STATE_WAIT_ACK		= 1, /* Waiting for the block read ACK */
	BLKRD_STATE_WAIT_DATA		= 2, /* Waiting for the block read data */
	BLKRD_STATE_WAIT_CRC		= 3, /* Waiting for the CRC */
	BLKRD_STATE_HAVE_DATA		= 4, /* Have block read data with the CRC */
	BLKRD_STATE_TIMEOUTED		= 5, /* Timeout response from the PIC */
	BLKRD_STATE_CRC_ERR		= 6, /* Compared CRC (PIC and card) differs */
};

/* Values for the command state machine */
enum fim_cmd_state {
	CMD_STATE_IDLE			= 0,
	CMD_STATE_WAIT_ACK		= 1, /* Waiting for the response ACK */
	CMD_STATE_WAIT_DATA		= 2, /* Waiting for the response data */
	CMD_STATE_HAVE_RSP		= 3, /* Have response data */
	CMD_STATE_TIMEOUTED		= 4, /* Timeout response from the PIC */
	CMD_STATE_CRC_ERR		= 5, /* Compared CRC (PIC and card) differs */
};

/*
 * Internal port structure for the FIM-SDIO host controller
 * wp_gpio : GPIO to use for reading the write protect line
 */
struct fim_sdio_t {
	struct fim_driver fim;
	unsigned int flags;
	struct device *device;
	int index;

	struct fim_gpio_t gpios[FIM_SDIO_MAX_GPIOS];
	struct fim_buffer_t *buf;
	struct mmc_command *mmc_cmd;
	struct mmc_host *mmc;
	struct mmc_request *mmc_req;

	enum fim_cmd_state cmd_state;
	enum fim_blkrd_state blkrd_state;

	int trans_blocks;
	int trans_blksz;
	int trans_sg;
	int trans_sg_offs;
	int reg;

	unsigned int tx_crc_timeout;

	int wp_gpio;
	struct clk *sys_clk;
	int cd_gpio;
	int cd_irq;
	int cd_value;
	struct timer_list cd_timer;
	struct timer_list debounce_timer;

	/* Members used for restarting the FIM */
	atomic_t fim_is_down;
	struct work_struct restart_work;

	/* Wait queue, and events */
	wait_queue_head_t wq;
	s8 tx_ready;
	s8 rx_ready;
};

/*
 * Transfer command structure for the card
* cmd   : Command to send to the card
 */
struct fim_sd_tx_cmd_t {
        unsigned char cmd[FIM_SDIO_TX_CMD_LEN];
}__attribute__((__packed__));


/*
 * Response receive structure from the Card
 * resp  : Card response, with a length of 5 or 17 as appropriate
*/
struct fim_sd_rx_resp_t {
        unsigned char resp[FIM_SDIO_MAX_RESP_LENGTH];
}__attribute__((__packed__));


/* Main structure for the available ports */
struct fim_sdios_t {
	int fims;
	struct fim_sdio_t *ports;
};


static struct fim_sdios_t *fim_sdios;

/* Internal functions */
inline static struct fim_sdio_t *get_port_from_mmc(struct mmc_host *mmc);
static void fim_sd_set_clock(struct fim_sdio_t *port, long int clockrate);
static struct fim_buffer_t *fim_sd_alloc_cmd(struct fim_sdio_t *port);
static int fim_sd_send_command(struct fim_sdio_t *port, struct mmc_command *cmd);

#define FIM_SD_COPY_DIR_TO_DMA	0
#define FIM_SD_COPY_DIR_TO_SG	1
static int fim_sd_copy_dma_sg(struct fim_sdio_t *port, struct mmc_data *data,
				     unsigned char *dma_buf, int dma_len, unsigned char dir);

static int cdtimer_disabled = 0;

/* Return the corresponding port structure */
inline static struct fim_sdio_t *get_port_from_mmc(struct mmc_host *mmc)
{
	if (!mmc)
		return NULL;

	return (struct fim_sdio_t *)mmc->private[0];
}

/* Workqueue for restarting a FIM */
static void fim_sd_restart_work_func(struct work_struct *work)
{
	struct fim_sdio_t *port;
	struct fim_driver *fim;
	int ret;
#if defined(MODULE)
	const char *fwcode = NULL;
	char fwname[FIM_SDIO_FW_LEN];
#else
	char *fwname = NULL;
	const char *fwcode;
#endif

	port = container_of(work, struct fim_sdio_t, restart_work);
	fim = &port->fim;

#if defined(MODULE)
	snprintf(fwname, FIM_SDIO_FW_LEN, FIM_SDIO_FW_FILE_PAT, fim->picnr);
#else
	fwcode = (fim->picnr == 0) ? FIM_SDIO_FW_CODE0 : FIM_SDIO_FW_CODE1;
#endif
	fim->fw_name = fwname;
	fim->fw_code = fwcode;

	printk_dbg("Going to restart the FIM%i\n", fim->picnr);
	fim_disable_irq(&port->fim);

	ret = fim_send_stop(&port->fim);
	if (ret) {
		printk_err("Couldn't stop the FIM%i\n", fim->picnr);
		return;
	}

	ret = fim_dma_stop(&port->fim);
	if (ret) {
		printk_err("Couldn't stop FIM%i DMA\n", fim->picnr);
		return;
	}

#if 0
	ret = fim_download_firmware(&port->fim);
	if (ret) {
		printk_err("FIM%i download failed\n", fim->picnr);
		return;
	}
#endif

	ret = fim_dma_start(&port->fim, NULL);
	if (ret) {
		printk_err("Couldn't start FIM%i DMA\n", fim->picnr);
		return;
	}

	ret = fim_send_start(&port->fim);
	if (ret) {
		printk_err("FIM%i start failed\n", fim->picnr);
		return;
        }

	/* Reset the internal values */
	printk_dbg("Re-enabling the IRQ of FIM%u\n", fim->picnr);
	atomic_set(&port->fim_is_down, 0);
	fim_set_ctrl_reg(&port->fim, FIM_SDIO_CONTROL1_REG, FIM_SDIO_CONTROL1_START);
	fim_enable_irq(&port->fim);
	port->mmc_cmd = NULL;
	mmc_detect_change(port->mmc, msecs_to_jiffies(100));
}

/* Check the status of the card detect line if no external IRQ supported */
static void fim_sd_cd_timer_func(unsigned long _port)
{
	struct fim_sdio_t *port;
	int val;

	/*
	 * The timer might still be running for a while even though the
	 * module has been unloaded. It looks like del_timer_sync() does
	 * not really guarantee that the timer is not running, because
	 * the timeout function reloads the timer every time.
	 * The function must be protected against this by using a boolean
	 * variable 'cdtimer_disabled' that is set upon timer deletion.
	 */
	if (!cdtimer_disabled) {
		port = (struct fim_sdio_t *)_port;
		val = gpio_get_value_ns921x(port->cd_gpio);

		if (val != port->cd_value) {
			port->cd_value = val;

			/* Interrupt TX and RX */
			port->tx_ready = -1;
			port->rx_ready = -1;
			if ( port->mmc_req && port->mmc_req->cmd ) {
				port->mmc_req->cmd->error = -ENOMEDIUM;
			}
			wake_up_interruptible(&port->wq);

			if (val) {
				atomic_set(&port->fim_is_down, 1);

				mmc_detect_change(port->mmc, 0);
			}
			else {
				schedule_work(&port->restart_work);
			}

		}

		mod_timer(&port->cd_timer, jiffies + FIM_SDIO_CD_POLLING_TIMER);
	}
}

/*
 * Configure the trigger edge of the CD interrupt depending on the current
 * state of the GPIO
 */
static inline int fim_sd_prepare_cd_irq(struct fim_sdio_t *port)
{
	int val;
	unsigned int type;

	val = gpio_get_value_ns921x(port->cd_gpio);
	printk_debug("CD interrupt received (val %i)\n", val);

	if (!val)
		type = IRQF_TRIGGER_RISING;
	else
		type = IRQF_TRIGGER_FALLING;

	return set_irq_type(port->cd_irq, type);
}

/*
 * Card detect debounce timer callback.
 * Enables the card detect interrupt, and sets its type.
 */
static void fim_sd_cd_debounce_func(unsigned long _port)
{
	struct fim_sdio_t *port;
	int ret;
	int val;

	port = (struct fim_sdio_t *)_port;

	/* Interrupt TX and RX */
	port->tx_ready = -1;
	port->rx_ready = -1;
	if ( port->mmc_req && port->mmc_req->cmd ) {
		port->mmc_req->cmd->error = -ENOMEDIUM;
	}
	wake_up_interruptible(&port->wq);

	val = gpio_get_value_ns921x(port->cd_gpio);
	if (val) {
		atomic_set(&port->fim_is_down, 1);

		mmc_detect_change(port->mmc, 0);
	}
	else {
		schedule_work(&port->restart_work);
	}

	/* Set new IRQ type */
	if ((ret = fim_sd_prepare_cd_irq(port))) {
		printk_err("Failed CD IRQ reconfiguration (%i)\n", ret);
	}

	/* Enabling IRQ */
	enable_irq(port->cd_irq);
}

/*
 * The external interrupt line only supports one edge detection! That means, we must
 * check the status of the line and reconfigure the interrupt trigger type.
 */
static irqreturn_t fim_sd_cd_irq(int irq, void *_port)
{
	struct fim_sdio_t *port;
	//int ret;

	port = (struct fim_sdio_t *)_port;

	/* Disable CD IRQ, and start debounce timer */
	disable_irq(port->cd_irq);
	mod_timer(&port->debounce_timer, jiffies + msecs_to_jiffies(500));

	return IRQ_HANDLED;
}

inline static int fim_sd_card_plugged(struct fim_sdio_t *port)
{
	struct fim_driver *fim;
	unsigned int val;

	fim = &port->fim;
	fim_get_stat_reg(fim, FIM_SDIO_CARD_STATREG, &val);
	return !val;
}

/*
 * Handler for the incoming FIM-interrupts. Available interrupts:
 * - Card detection
 * - SDIO interrupts from the cards
 */
static void fim_sd_isr(struct fim_driver *driver, int irq, unsigned char code,
		       unsigned int rx_fifo)
{
	struct fim_sdio_t *port;
	unsigned int ireg;

	port = driver->driver_data;
	if (!port || !port->mmc) {
		printk_dbg("Null pointer by unexpected IRQ 0x%02x\n", code);
		return;
	}

	switch (code) {

	case FIM_SDIO_INTARM_CARD_DETECTED:
		if (fim_sd_card_plugged(port)) {
			fim_get_ctrl_reg(&port->fim, FIM_SDIO_CONTROL1_REG, &ireg);
			fim_set_ctrl_reg(&port->fim, FIM_SDIO_CONTROL1_REG, (ireg & ~FIM_SDIO_CONTROL1_INTR_EN));
			printk_debug("SD card detected\n");
		} else {
			printk_debug("SD card removed\n");
		}
		mmc_detect_change(port->mmc, msecs_to_jiffies(100));
		break;

	case FIM_SDIO_INTARM_CARD_DAT1:
		printk_debug("SDIO IRQ\n");

		/*
		 * Normally we don't need this sanity check but the FIM is reporting
		 * some SDIO-interrupts where no SDIO-card was initialized.
		 */
		if (port->mmc->sdio_irq_thread)
			mmc_signal_sdio_irq(port->mmc);
		else
			printk_dbg("Premature SDIO IRQ received!\n");
		break;

	default:
		printk_err("Unknown IRQ %i | FIM %i | %x\n",
			   code, port->fim.picnr, rx_fifo);
		break;
	}
}

/*
 * This is the TX-callback that the API call after a DMA-package was closed
 * The fim buffer structure contains our internal private data
 * Free the allocated FIM-buffer that was used for sending the DMA-data
*/
static void fim_sd_tx_isr(struct fim_driver *driver, int irq,
			  struct fim_buffer_t *pdata)
{
	struct fim_buffer_t *buf;
	struct fim_sdio_t *port;

	port = (struct fim_sdio_t *)driver->driver_data;
	if (pdata->private) {
		buf = pdata->private;
		fim_free_buffer(&port->fim, buf);
	}

	/* Set transmit ready flag */
	port->tx_ready = 1;
	wake_up_interruptible(&port->wq);
}

/* @XXX: Remove this ugly code! */
inline static void fim_sd_parse_resp(struct mmc_command *cmd,
				     struct fim_sd_rx_resp_t *resp)
{
	unsigned char *ptr;
	ptr = (unsigned char *)cmd->resp;
	if (cmd->flags & MMC_RSP_136) {
		*ptr++ = resp->resp[4];
		*ptr++ = resp->resp[3];
		*ptr++ = resp->resp[2];
		*ptr++ = resp->resp[1];
		*ptr++ = resp->resp[8];
		*ptr++ = resp->resp[7];
		*ptr++ = resp->resp[6];
		*ptr++ = resp->resp[5];
		*ptr++ = resp->resp[12];
		*ptr++ = resp->resp[11];
		*ptr++ = resp->resp[10];
		*ptr++ = resp->resp[9];
		*ptr++ = resp->resp[16];
		*ptr++ = resp->resp[15];
		*ptr++ = resp->resp[14];
		*ptr++ = resp->resp[13];
	} else {
		*ptr++ = resp->resp[4];
		*ptr++ = resp->resp[3];
		*ptr++ = resp->resp[2];
		*ptr++ = resp->resp[1];
		*ptr++ = resp->resp[5];
		*ptr++ = resp->resp[0];
	}
}

static unsigned char fim_sd_calc_crc7(unsigned char *buffer, unsigned char length)
{
	int i, a;
	unsigned char crc, data;

	crc = 0;
	for ( a = 0; a < length; a++ ) {
		data = buffer[a];
		for ( i = 0; i < 8; i++ ) {
			crc <<= 1;

			if ( (data & 0x80) ^ (crc & 0x80) ) {
				crc ^= 0x09;
			}
			data <<= 1;
		}
	}
	crc = (crc << 1) | 1;

	return crc;
}

static int fim_sd_check_r2_crc(struct fim_sd_rx_resp_t *resp, int len)
{
	unsigned char crc;
	unsigned char *buffer;

	if ( len != 17 ) {
		printk_err("Invalid length (%d) for R2 response\n", len);
		return 0;
	}

	buffer = (unsigned char *)resp;

	crc = fim_sd_calc_crc7(&buffer[1], 15);

	if ( crc != buffer[16] ) {
		printk_err("Invalid R2 CRC: 0x%x\n", crc);
	}

	return 1;
}

/*
 * Called when a receive DMA-buffer was closed.
 * The data received from the PIC has different formats. Sometimes it
 * contains a response and sometimes data of a block read request.
 */
static void fim_sd_rx_isr(struct fim_driver *driver, int irq,
			  struct fim_buffer_t *pdata)
{
	struct fim_sdio_t *port;
	struct mmc_command *mmc_cmd;
	struct fim_sd_rx_resp_t *resp;
	int len;
	unsigned int stat;

	/* Get the correct port from the FIM-driver structure */
	len = pdata->length;
	port = (struct fim_sdio_t *)driver->driver_data;

	/*
	 * The timeout function can set the command structure to NULL, for this reason
	 * check here is we can handle the response correctly
	 */
	if ((mmc_cmd = port->mmc_cmd) == NULL) {
		printk_err("Timeouted command response?\n");
		goto exit_unlock;
	}

	/*
	 * Check the current state of the command and update it if required
	 * IMPORTANT: The buffer can contain response data or the data from a block
	 * read too, for this reason was implemented the state machine
	 */
	resp = (struct fim_sd_rx_resp_t *)pdata->data;

	fim_get_stat_reg(&port->fim, FIM_SDIO_PORTEXP1_REG, &stat);

	if ( stat & FIM_SDIO_PORTEXP1_READ ) {
		/* First comes data */
		if ( port->blkrd_state == BLKRD_STATE_WAIT_DATA ) {
			if ( stat & FIM_SDIO_PORTEXP1_CRCERR_DATA ) {
				/* We need to receive response in case of error also */
				port->blkrd_state = BLKRD_STATE_CRC_ERR;
			}
			else {
				fim_sd_copy_dma_sg(port, mmc_cmd->data,
						pdata->data, pdata->length, FIM_SD_COPY_DIR_TO_SG);

				/* Check if we have a multiple transfer read */
				port->trans_blocks -= len / port->trans_blksz;
				if (port->trans_blocks > 0) {
					printk_debug("Wait for next block %i\n", port->trans_blocks);
				}
				else {
					port->blkrd_state = BLKRD_STATE_HAVE_DATA;
				}
			}
		}
		/* After data, there comes the response */
		else if ( port->cmd_state == CMD_STATE_WAIT_DATA ) {
			/* If the response has R2 type, then check the inner CRC */
			/* Should not come data before an R2 response */
			if ( port->flags & FIM_SDIO_FLAG_R2_RESPONSE ) {
				if ( fim_sd_check_r2_crc(resp,len) ) {
					fim_sd_parse_resp(mmc_cmd, resp);
					port->cmd_state = CMD_STATE_HAVE_RSP;
				}
				else {
					port->cmd_state = CMD_STATE_CRC_ERR;
				}
			}
			else {
				fim_sd_parse_resp(mmc_cmd, resp);
				port->cmd_state = CMD_STATE_HAVE_RSP;
			}
		}
		/* Unexpected state */
		else {
			if (mmc_cmd->data && mmc_cmd->data->flags & MMC_DATA_READ)
				printk_err("Failed multi RX (CMD%u | PIC stat %x | State %x)\n",
					mmc_cmd->opcode, stat, port->blkrd_state);
			else
				printk_err("Unexpected RX stat (CMD%i | PIC stat %x | Leng %i)\n",
					mmc_cmd->opcode, stat, pdata->length);
		}
	}
	else if ( stat & FIM_SDIO_PORTEXP1_RESP ) {
		/* First comes response */
		if ( port->cmd_state == CMD_STATE_WAIT_DATA ) {
			/* If the response has R2 type, then check the inner CRC */
			if ( port->flags & FIM_SDIO_FLAG_R2_RESPONSE ) {
				if ( fim_sd_check_r2_crc(resp,len) ) {
					fim_sd_parse_resp(mmc_cmd, resp);
					port->cmd_state = CMD_STATE_HAVE_RSP;
				}
				else {
					port->cmd_state = CMD_STATE_CRC_ERR;
				}
			}
			else {
				fim_sd_parse_resp(mmc_cmd, resp);
				port->cmd_state = CMD_STATE_HAVE_RSP;
			}
		}
		/* After response, there comes the data */
		else if ( port->blkrd_state == BLKRD_STATE_WAIT_DATA ) {
			if ( stat & FIM_SDIO_PORTEXP1_CRCERR_DATA ) {
				port->blkrd_state = BLKRD_STATE_CRC_ERR;
			}
			else {
				fim_sd_copy_dma_sg(port, mmc_cmd->data,
						pdata->data, pdata->length, FIM_SD_COPY_DIR_TO_SG);

				/* Check if we have a multiple transfer read */
				port->trans_blocks -= len / port->trans_blksz;
				if (port->trans_blocks > 0) {
					printk_debug("Wait for next block %i\n", port->trans_blocks);
				}
				else {
					port->blkrd_state = BLKRD_STATE_HAVE_DATA;
				}
			}
		}
		/* Unexpected state */
		else {
			if (mmc_cmd->data && mmc_cmd->data->flags & MMC_DATA_READ)
				printk_err("Failed multi RX (CMD%u | PIC stat %x | State %x)\n",
					mmc_cmd->opcode, stat, port->blkrd_state);
			else
				printk_err("Unexpected RX stat (CMD%i | PIC stat %x | Leng %i)\n",
					mmc_cmd->opcode, stat, pdata->length);
		}
	}

	if ( port->cmd_state == CMD_STATE_HAVE_RSP &&
		port->blkrd_state == BLKRD_STATE_CRC_ERR ) {
		printk_err("CRC failure\n");
		mmc_cmd->error = -EILSEQ;
	}

	if ( port->cmd_state == CMD_STATE_CRC_ERR ) {
		mmc_cmd->error = -EILSEQ;
	}

	/*
	 * By errors set the two states machines to the end position for sending
	 * the error to the MMC-layer
	 */
	if (mmc_cmd->error) {
		port->cmd_state = CMD_STATE_HAVE_RSP;
		port->blkrd_state = BLKRD_STATE_HAVE_DATA;
	}

	/* Sign that we're ready */
	if ( port->cmd_state == CMD_STATE_HAVE_RSP &&
		port->blkrd_state == BLKRD_STATE_HAVE_DATA ) {
		port->rx_ready = 1;
		wake_up_interruptible(&port->wq);
	}

 exit_unlock:
	return;
}

/* Send a buffer over the FIM-API */
static int fim_sd_send_buffer(struct fim_sdio_t *port, struct fim_buffer_t *buf)
{
	struct fim_driver *fim;
	int ret;

	if (!buf || !port)
		return -EINVAL;

	fim = &port->fim;
	buf->private = buf;

	port->tx_ready = 0;

	ret = fim_send_buffer(fim, buf);
	if ( ret != 0 )
	{
		return ret;
	}

	/* Wait for sending completed */
	if ( wait_event_interruptible_timeout(port->wq, port->tx_ready, msecs_to_jiffies(1000)) <= 0 )
	{
		printk_err("DMA transmit timeout\n");
		return -ETIMEDOUT;
	}
	if ( port->tx_ready == -1 )
	{
		printk_dbg("DMA transmit interrupt\n");
		return -ENOMEDIUM;
	}

	return 0;
}

/* Returns a command buffer allocated from the FIM-API */
static struct fim_buffer_t *fim_sd_alloc_cmd(struct fim_sdio_t *port)
{
	struct fim_driver *fim;
	int length;

	if(!port)
		return NULL;

	fim = &port->fim;
	length = sizeof(struct fim_sd_tx_cmd_t);

	return fim_alloc_buffer(fim, length, GFP_KERNEL);
}

/* Returns a buffer allocated from the FIM-API */
static struct fim_buffer_t *fim_sd_alloc_buffer(struct fim_sdio_t *port, int length)
{
	struct fim_driver *fim;

	if (!port || length <= 0)
		return NULL;

	fim = &port->fim;
	return fim_alloc_buffer(fim, length, GFP_KERNEL);
}

static void fim_sd_free_buffer(struct fim_sdio_t *port, struct fim_buffer_t *buf)
{
	struct fim_driver *fim;

	if (!port || !buf)
		return;

	fim = &port->fim;
	fim_free_buffer(fim, buf);
}

/*
 * Copy the data from the MMC-layer (scatter list) and the DMA-buffer for the FIM-API
 * The "dir" argument gives the direction:
 *	FIM_SD_COPY_DIR_TO_DMA:		copying from SG to DMA buffer
 *	FIM_SD_COPY_DIR_TO_SG:		copying from DMA buffer to SG
 */
static int fim_sd_copy_dma_sg(struct fim_sdio_t *port, struct mmc_data *data,
				     unsigned char *dma_buf, int dma_len, unsigned char dir)
{
	struct scatterlist *sg;
	unsigned int sg_len;
	unsigned char *sg_buf_virt;
	unsigned int sg_buf_len;
	int len_back;

	sg_len = data->sg_len;
	len_back = dma_len;

	while ( len_back > 0 ) {
		int len;

		/* Check sg */
		if ( port->trans_sg >= sg_len ) {
			printk_err("DMA_to_SG was called for a non existing SG\n");
			return 1;
		}
		sg = &data->sg[port->trans_sg];

		sg_buf_len = sg->length - port->trans_sg_offs;
		sg_buf_virt = (unsigned char *)sg_virt(sg) + port->trans_sg_offs;

		if ( sg_buf_len > len_back ) {
			len = len_back;
			port->trans_sg_offs += len;
		}
		else {
			len = sg_buf_len;
			port->trans_sg_offs = 0;
			++(port->trans_sg);
		}

		if ( dir ) {
			memcpy(sg_buf_virt, dma_buf, len);
		}
		else {
			memcpy(dma_buf, sg_buf_virt, len);
		}
		dma_buf += len;

		len_back -= len;
	}

	data->bytes_xfered += dma_len;

	return 0;
}

inline static int fim_sd_get_transmit_crc_status(struct fim_sdio_t *port)
{
	unsigned int crc_status;
	//unsigned int ret;
	volatile unsigned int cnt = port->tx_crc_timeout;

	/* Wait for write CRC status */
	do {
		--cnt;
		/*if ( (ret = wait_event_interruptible_timeout(port->wq, 0, usecs_to_jiffies(10))) != 0 ) {
			printk_err("Problem with wait_queue: %d\n", ret);
			return -ETIMEDOUT;
		}*/

		fim_get_stat_reg(&port->fim, FIM_SDIO_DATAWR_STATUS_REG, &crc_status);
	} while ( (crc_status == 0) && (cnt != 0) );

	if ( crc_status == 0 ) {
		printk_err("Transmit CRC timeout\n");
		return -ETIMEDOUT;
	}
	else if ( crc_status != FIM_SDIO_DATAWR_STATUS_SUCCESS ) {
		printk_err("Transmit CRC error: %d\n", crc_status);
		return -EILSEQ;
	}

	return 0;
}

/* This function will send the command to the PIC using the TX-DMA buffers */
static int fim_sd_send_command(struct fim_sdio_t *port, struct mmc_command *cmd)
{
	struct mmc_data *data;
	struct fim_buffer_t *buf;
	struct fim_sd_tx_cmd_t *txcmd;
	unsigned int block_length, blocks;
	int retval, cur_len, data_len = 0, len = 0;
	unsigned char opctl = 0;
	unsigned int stat;
	int i;
	unsigned int control1_reg;

	/* @TODO: Send an error response to the MMC-core */
	if (!(buf = fim_sd_alloc_cmd(port))) {
		printk_err("No memory available for a new CMD?\n");
		cmd->error = -ENOMEM;
		return -ENOMEM;
	}

	/* Use the buffer data for the TX-command */
	txcmd = (struct fim_sd_tx_cmd_t *)buf->data;

	/* Clear abort flag */
	fim_get_ctrl_reg(&port->fim, FIM_SDIO_CONTROL1_REG, &control1_reg);
	if ( control1_reg & FIM_SDIO_CONTROL1_STOP )
	{
		control1_reg &= ~FIM_SDIO_CONTROL1_STOP;
		fim_set_ctrl_reg(&port->fim, FIM_SDIO_CONTROL1_REG, control1_reg);
	}

	/*
	 * Set the internal flags for the next response sequences
	 * Assume that we will wait for a command response (not block read).
	 * By block reads the flag will be modified inside the if-condition
	 */
	port->flags = 0;
	port->cmd_state = CMD_STATE_WAIT_DATA;
	port->blkrd_state = BLKRD_STATE_HAVE_DATA;
	if ((data = cmd->data) != NULL) {
		block_length = data->blksz;
		blocks = data->blocks;

		printk_debug("CMD%u: %s %i blks | %i blksz | SG len %u\n",
			     cmd->opcode,
			     (data->flags & MMC_DATA_READ) ? "RX" : "TX",
			     data->blocks, data->blksz,
			     data->sg_len);

		/* Reset the scatter list position */
		port->trans_sg     = 0;
		port->trans_sg_offs = 0;
		port->trans_blocks = blocks;
		port->trans_blksz  = block_length;

		/* Setup the FIM registers (number of blocks and block size) */
		fim_set_ctrl_reg(&port->fim, FIM_SDIO_BLKCNT_LSB_REG, blocks & 0xFF);
		fim_set_ctrl_reg(&port->fim, FIM_SDIO_BLKCNT_MSB_REG, (blocks >> 8) & 0xFF);
		fim_set_ctrl_reg(&port->fim, FIM_SDIO_BLKSZ_LSB_REG, block_length & 0xFF);
		fim_set_ctrl_reg(&port->fim, FIM_SDIO_BLKSZ_MSB_REG,
				 (block_length >> 8) & 0xFF);

		/* Check if the transfer request is for reading or writing */
		if (cmd->data->flags & MMC_DATA_READ) {
			opctl |= FIM_SDIO_CONTROL2_BLK_READ;
			port->blkrd_state = BLKRD_STATE_WAIT_DATA;
		} else {
			opctl |= FIM_SDIO_CONTROL2_BLK_WRITE;
		}
	} else {
		block_length = 0;
		blocks = 0;
	}

	/* Set the correct expected response length */
	if (cmd->flags & MMC_RSP_136)
		opctl |= FIM_SDIO_CONTROL2_RESP136;
	else if (cmd->flags & MMC_RSP_PRESENT)
		opctl |= FIM_SDIO_CONTROL2_RESP48;
	/* Anyway, there's no response for command */

	/* Disable CRC calculation if not needed, or if the command has an R2 type response, which
	   has an internal CRC. */
	if ( (cmd->opcode == 2) || (cmd->opcode == 9) || (cmd->opcode == 10) ) {
		port->flags |= FIM_SDIO_FLAG_R2_RESPONSE;
	}
	if (!(cmd->flags & MMC_RSP_CRC) || (port->flags & FIM_SDIO_FLAG_R2_RESPONSE) ) {
		printk_debug("CRC is disabled\n");
		opctl |= FIM_SDIO_CONTROL2_SKIP_CRC;
	}

	fim_set_ctrl_reg(&port->fim, FIM_SDIO_CONTROL2_REG, opctl);

	txcmd->cmd[0] = SDIO_HOST_TX_HDR | (cmd->opcode & SDIO_HOST_CMD_MASK);
	txcmd->cmd[1] = cmd->arg >> 24;
	txcmd->cmd[2] = cmd->arg >> 16;
	txcmd->cmd[3] = cmd->arg >> 8;
	txcmd->cmd[4] = cmd->arg;

	port->rx_ready = 0;

	/*
	 * Store the private data for the callback function
	 * If an error ocurrs when sending the buffer, the timeout function will
	 * send the error to the MMC-layer
	 */
	port->buf = buf;
	port->mmc_cmd = cmd;
	buf->private = port;
	if ((retval = fim_sd_send_buffer(port, buf))) {
		fim_sd_free_buffer(port, buf);
		cmd->error = retval;
		return retval;
	}

	if ( cmd->flags & MMC_RSP_PRESENT ) {
		/* Wait 10ms for command response (it's max. 64 SD CLK according to the spec) */
		i = 10;
		fim_get_stat_reg(&port->fim, FIM_SDIO_PORTEXP1_REG, &stat);
		while ( (i > 0) && (stat == 0) ) {
			if ( (retval = wait_event_interruptible_timeout(port->wq, port->rx_ready, msecs_to_jiffies(1))) < 0 ) {
				printk_err("Problem with wait_queue: %d\n", retval);
				return -ETIMEDOUT;
			}
			if ( port->rx_ready == -1 )
			{
				printk_dbg("STAT reg reading interrupt\n");
				return -ENOMEDIUM;
			}

			fim_get_stat_reg(&port->fim, FIM_SDIO_PORTEXP1_REG, &stat);

			--i;
		};

		if ( stat & FIM_SDIO_PORTEXP1_TIMEOUT ) {
			printk_dbg("Timeout from FIM: 0x%x, CMD%d\n", stat, port->mmc_cmd->opcode);
			port->mmc_cmd->error = -ETIMEDOUT;
			return -ETIMEDOUT;
		}
		else if ( stat & FIM_SDIO_PORTEXP1_CRCERR_CMD ) {
			printk_info("Response CRC error\n");
			port->mmc_cmd->error = -EILSEQ;
			return -EILSEQ;
		}
		else if ( stat == 0 ) {
			printk_err("No response from FIM, down\n");
			atomic_set(&port->fim_is_down, 1);
			port->mmc_cmd->error = -ENOMEDIUM;
			return -ETIMEDOUT;
		}
	}

	/*
	 * If we have a write command then fill a next buffer and send it
	 * @TODO: We need here an error handling, then otherwise we have started a
	 * WR-transfer but have no transfer data (perhaps not too critical?)
	 */
	if (data && data->flags & MMC_DATA_WRITE) {
		data_len = data->blksz * data->blocks;
		while ( len < data_len ) {
			cur_len = ((data_len - len) > FIM_SDIO_DMA_BUFFER_SIZE) ?
				FIM_SDIO_DMA_BUFFER_SIZE : data_len - len;

			if (!(buf = fim_sd_alloc_buffer(port, cur_len))) {
				printk_err("Buffer alloc BLKWR failed, %i\n", cur_len);
				cmd->error = -EIO;
				return -EIO;
			}

			buf->private = port;
			fim_sd_copy_dma_sg(port, data, buf->data, buf->length, FIM_SD_COPY_DIR_TO_DMA);
			if ((retval = fim_sd_send_buffer(port, buf))) {
				printk_err("Send BLKWR-buffer failed, %i\n", retval);
				fim_sd_free_buffer(port, buf);
				cmd->error = retval;
				return retval;
			}

			if ( (retval = fim_sd_get_transmit_crc_status(port)) != 0 ) {
				cmd->error = retval;

				len += cur_len;

				if ( len < data_len )
				{
					/* Set the abort flag, and send a sector buffer to abort the data transmission */
					fim_get_ctrl_reg(&port->fim, FIM_SDIO_CONTROL1_REG, &control1_reg);
					control1_reg |= FIM_SDIO_CONTROL1_STOP;
					fim_set_ctrl_reg(&port->fim, FIM_SDIO_CONTROL1_REG, control1_reg);

					if ((buf = fim_sd_alloc_buffer(port, data->blksz)) != 0) {
						buf->private = port;
						if (fim_sd_send_buffer(port, buf)) {
							fim_sd_free_buffer(port, buf);
						}
					}
				}

				return retval;
			}

			len += cur_len;
		}
	}

	/* If command doesn't have a response, then this is the end, RX ISR is not needed*/
	if ( !(cmd->flags & MMC_RSP_PRESENT) ) {
		return 0;
	}

	/* Set message RX timeout */
	retval = wait_event_interruptible_timeout(port->wq, port->rx_ready, msecs_to_jiffies(1000));
	if ( retval <= 0 ) {
		printk_err("Message RX timeout\n");
		port->mmc_cmd->error = -ETIMEDOUT;
		return -ETIMEDOUT;
	}
	if ( port->rx_ready == -1 )
	{
		printk_dbg("Message RX interrupt\n");
		return -ENOMEDIUM;
	}

	if ( port->mmc_cmd->error ) {
		return port->mmc_cmd->error;
	}
	else {
		return 0;
	}
}

/*
 * Called for processing three main request types:
 * command : Submit the command to the PIC and returns inmediately
 * stop    : Request for stopping an already started request?
 * data    : For data transfer
 */
static void fim_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct fim_sdio_t *port;

	if (!(port = get_port_from_mmc(mmc)))
		return;

	/* In the case that the FIM was shutdown return at this point */
	if (atomic_read(&port->fim_is_down)) {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	/*
	 * Wait if the timeout function is running or the RX-callback is active
	 */
	port->mmc_req = mrq;

	fim_sd_send_command(port, port->mmc_req->cmd);

	if ( port->mmc_req->stop ) {
		fim_sd_send_command(port, port->mmc_req->stop);
	}

	port->mmc_cmd = NULL;
	port->mmc_req = NULL;
	mmc_request_done(mmc, mrq);
}

/* Set the transfer clock using the pre-defined values */
static void fim_sd_set_clock(struct fim_sdio_t *port, long int clockrate)
{
	unsigned long clkdiv;
	unsigned long clk;

	if (clockrate) {
		clk = clk_get_rate(port->sys_clk) / 6;
		clkdiv  = clk /clockrate - 9;

		if ( clkdiv < 1 )
		{
			clkdiv = 1;
		}
		else if ( clkdiv > 0xff )
		{
			clkdiv = 0xff;
		}
		else
			printk_debug("Calculated divisor is 0x%02lx for %lu\n",
				     clkdiv, clockrate);
	} else {
		printk_debug("@TODO: Disable the clock (set to 0Hz)\n");
		clkdiv = 0;
		clockrate = -EINVAL;
	}

	/* Now write the value to the corresponding FIM-register */
	if (clockrate >= 0) {
		printk_debug("Setting the clock to %ld (%lx)\n",
			     clockrate, clkdiv);
		fim_set_ctrl_reg(&port->fim, FIM_SDIO_CLKDIV_REG, clkdiv);
	}
}

/*
 * Called by the core system for setting the available modes and clock speed
 *
 */
static void fim_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct fim_sdio_t *port;
	unsigned long ireg;

	if (!(port = get_port_from_mmc(mmc)))
		return;

	/*
	 * The FIM-board doesn't have a power switch for the card, but probably the
	 * next revision will include it, so that we can control the switch from here.
	 */
	ireg = 0;
	switch (ios->power_mode) {
	case MMC_POWER_OFF:
	case MMC_POWER_UP:
	case MMC_POWER_ON:
		break;
	}

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		break;
	case MMC_BUS_WIDTH_4:
		ireg |= FIM_SDIO_CONTROL1_BUS_WIDTH4;
		break;
	default:
		printk_err("Invalid bus width option = %d\n", ios->bus_width);
		break;
	}

	fim_set_ctrl_reg(&port->fim, FIM_SDIO_CONTROL1_REG, ireg);

	fim_sd_set_clock(port, ios->clock);
}

/*
 * Return the read only status of the plugged card
 * Since the FIM-firmware doesn't include this GPIO, we will read it at this point
 */
static int fim_sd_get_ro(struct mmc_host *mmc)
{
	struct fim_sdio_t *port;

	if (!(port = get_port_from_mmc(mmc))) {
		printk_err("No FIM-port by a registered MMC-host?\n");
		return -ENODEV;
	}

	if (FIM_GPIO_DONT_USE != port->wp_gpio)
		return gpio_get_value_ns921x(port->wp_gpio);
	else {
		/*
		 * There is no GPIO for the Write Protect functionality:
		 * Assume Write Protect is off.
		 */
		return 0;
	}
}

static void fim_sd_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct fim_sdio_t *port;
	unsigned int ireg;

	printk_debug("%s the SDIO IRQ\n", enable ? "Enabling" : "Disabling");
	if (!(port = get_port_from_mmc(mmc))) {
		printk_err("NULL port pointer found!\n");
		return;
	}

	fim_get_ctrl_reg(&port->fim, FIM_SDIO_CONTROL1_REG, &ireg);
	if (enable)
		ireg |= FIM_SDIO_CONTROL1_INTR_EN;
	else
		ireg &= ~FIM_SDIO_CONTROL1_INTR_EN;

	fim_set_ctrl_reg(&port->fim, FIM_SDIO_CONTROL1_REG, ireg);
}


/* Available driver host operations */
static const struct mmc_host_ops fim_sd_ops = {
	.request = fim_sd_request,
	.set_ios = fim_sd_set_ios,
	.get_ro  = fim_sd_get_ro,
	.enable_sdio_irq = fim_sd_enable_sdio_irq,
};


/* Called normally only when exiting the driver */
static int fim_sdio_unregister_port(struct fim_sdio_t *port)
{
	int cnt;

	if (!port || !port->reg)
		return -ENODEV;

	printk_info("Removing the FIM MMC host %i\n", port->index);
	mmc_remove_host(port->mmc);
	mmc_free_host(port->mmc);

	cdtimer_disabled = 1;

	/* Reset the main control register */
	fim_set_ctrl_reg(&port->fim, FIM_SDIO_CONTROL1_REG, 0);

	fim_unregister_driver(&port->fim);

	for (cnt=0; cnt < FIM_SDIO_MAX_GPIOS; cnt++) {
		if (port->gpios[cnt].nr == FIM_LAST_GPIO)
			break;
		else if (port->gpios[cnt].nr == FIM_GPIO_DONT_USE)
			continue;
		else {
			gpio_free(port->gpios[cnt].nr);
			printk_debug("Freeing GPIO %i\n", port->gpios[cnt].nr);
		}
	}

	if (port->sys_clk && !IS_ERR(port->sys_clk)) {
		clk_put(port->sys_clk);
		port->sys_clk = NULL;
	}

	if (port->cd_irq > 0) {
		free_irq(port->cd_irq, port);
		port->cd_irq = -1;
	}

	port->reg = 0;
	return 0;
}

/* Register the new FIM driver by the FIM-API */
static int fim_sdio_register_port(struct device *dev, struct fim_sdio_t *port,
				  struct fim_sdio_platform_data *pdata,
				  struct fim_gpio_t gpios[])
{
	int retval;
	int cnt;
	struct fim_dma_cfg_t dma_cfg;
	int picnr = pdata->fim_nr;
	unsigned long hcaps = pdata->host_caps;
	unsigned int fwver;

#if defined(MODULE)
	const char *fwcode = NULL;
	char fwname[FIM_SDIO_FW_LEN];
	snprintf(fwname, FIM_SDIO_FW_LEN, FIM_SDIO_FW_FILE_PAT, picnr);
#else
	char *fwname = NULL;
	const char *fwcode = (picnr == 0) ? FIM_SDIO_FW_CODE0 : FIM_SDIO_FW_CODE1;
#endif

	/* Specific DMA configuration for the SD-host driver */
	dma_cfg.rxnr = FIM_SDIO_DMA_RX_BUFFERS;
	dma_cfg.txnr = FIM_SDIO_DMA_TX_BUFFERS;
	dma_cfg.rxsz = FIM_SDIO_DMA_BUFFER_SIZE;
	dma_cfg.txsz = FIM_SDIO_DMA_BUFFER_SIZE;

	port->index = picnr;
	port->fim.picnr = picnr;
	port->fim.driver.name = FIM_SDIO_DRIVER_NAME;
	port->fim.driver_data = port;
	port->fim.fim_isr = fim_sd_isr;
	port->fim.dma_tx_isr = fim_sd_tx_isr;
	port->fim.dma_rx_isr = fim_sd_rx_isr;
	port->fim.driver_data = port;
	port->fim.dma_cfg = &dma_cfg;

	/* Check if have a firmware code for using to */
	port->fim.fw_name = fwname;
	port->fim.fw_code = fwcode;
	retval = fim_register_driver(&port->fim);
	if (retval) {
		printk_err("Couldn't register the FIM driver.\n");
		return retval;
	}

	/* Request the corresponding GPIOs (@XXX: Check the returned values) */
	for (cnt=0; cnt < FIM_SDIO_MAX_GPIOS; cnt++) {

		if (gpios[cnt].nr == FIM_LAST_GPIO)
			break;

		if (gpios[cnt].nr == FIM_GPIO_DONT_USE)
			continue;

		printk_debug("Requesting the GPIO %i (Function %i)\n",
			     gpios[cnt].nr, gpios[cnt].func);
		retval = gpio_request(gpios[cnt].nr, FIM_SDIO_DRIVER_NAME);
		if (!retval) {
			gpio_configure_ns921x_unlocked(gpios[cnt].nr,
						       NS921X_GPIO_INPUT,
						       NS921X_GPIO_DONT_INVERT,
						       gpios[cnt].func,
						       NS921X_GPIO_ENABLE_PULLUP);
		} else {
			/* Free the already requested GPIOs */
			printk_err("Couldn't request the GPIO %i\n", gpios[cnt].nr);
			while (cnt)
				gpio_free(gpios[--cnt].nr);

			goto exit_unreg_fim;
		}
	}

	/* This is the timer for checking the state of the card detect line */
	init_timer(&port->cd_timer);
	port->cd_timer.function = fim_sd_cd_timer_func;
	port->cd_timer.data = (unsigned long)port;

	/* Initialize card detect pin's debounce timer */
	init_timer(&port->debounce_timer);
	port->debounce_timer.function = fim_sd_cd_debounce_func;
	port->debounce_timer.data = (unsigned long)port;

	/* Init wait queue head */
	init_waitqueue_head(&port->wq);

	port->mmc = mmc_alloc_host(sizeof(struct fim_sdio_t *), port->fim.dev);
	if (!port->mmc) {
		printk_err("Alloc MMC host by the FIM %i failed.\n", picnr);
		retval = -ENOMEM;
		goto exit_free_gpios;
	}

        /* Get a reference to the SYS clock for setting the clock */
        if (IS_ERR(port->sys_clk = clk_get(port->fim.dev, "systemclock"))) {
                printk_err("Couldn't get the SYS clock.\n");
                goto exit_free_host;
        }

	/* These are the default values for this SD-host */
	port->mmc->ops = &fim_sd_ops;

	/* Supported physical properties of the FIM-host (see the PIC-firmware code) */
	port->mmc->f_min = pdata->min_clk;
	port->mmc->f_max = pdata->max_clk;
	port->mmc->ocr_avail = MMC_VDD_33_34 | MMC_VDD_32_33;
	port->mmc->caps = hcaps;

	/* Maximum number of blocks in one req */
	port->mmc->max_blk_count = FIM_SDIO_BLKATONCE;
	port->mmc->max_blk_size  = FIM_SDIO_MAXBLKSIZE;
	/* The maximum per SG entry depends on the buffer size */
	port->mmc->max_seg_size  = FIM_SDIO_BUFSIZE;
	port->mmc->max_req_size  = FIM_SDIO_BUFSIZE;
	port->mmc->max_phys_segs = FIM_SDIO_BLKATONCE;
	port->mmc->max_hw_segs   = FIM_SDIO_BLKATONCE;

	/* Save our port structure into the private pointer */
	port->mmc->private[0] = (unsigned long)port;
	retval = mmc_add_host(port->mmc);
	if (retval) {
		printk_err("Couldn't add the MMC host\n");
		goto exit_put_clk;
	}

	/* Set the transmit CRC timeout time (200 for 5MHz) */
	if ( pdata->max_clk > 5000000 ) {
		port->tx_crc_timeout = 200;
	}
	else {
		port->tx_crc_timeout = 5000000 * 200 / pdata->max_clk;
	}

	memcpy(port->gpios, gpios, sizeof(struct fim_gpio_t) * FIM_SDIO_MAX_GPIOS);
	port->reg = 1;
	dev_set_drvdata(dev, port);

	/* Fist disable the SDIO-IRQ */
	fim_sd_enable_sdio_irq(port->mmc, 0);

	/*
	 * If no interrupt line is available for the card detection, then use the timer
	 * for the polling.
	 */
	if (FIM_GPIO_DONT_USE != port->cd_gpio) {
		/* First check if we have an IRQ at this line */
		port->cd_irq = gpio_to_irq(port->cd_gpio);

#if defined(FIM_SDIO_FORCE_CD_POLLING)
		port->cd_irq = -1;
#endif

		if (port->cd_irq > 0) {
			int iret;

			iret = request_irq(port->cd_irq, fim_sd_cd_irq,
					IRQF_DISABLED | IRQF_TRIGGER_FALLING,
					"fim-sdio-cd", port);
			if (iret) {
				printk_err("Failed IRQ %i request (%i)\n", port->cd_irq, iret);
				port->cd_irq = -1;
			}
			else
				printk_info("Got IRQ %i for GPIO %i\n", port->cd_irq,
					port->cd_gpio);
		}

		/*
		* By errors (e.g. in the case that we couldn't request the IRQ) use the
		* polling function
		*/
		if (port->cd_irq < 0) {
			printk_dbg(KERN_DEBUG "Polling the Card Detect line (no IRQ)\n");
			port->cd_value = gpio_get_value_ns921x(port->cd_gpio);
			mod_timer(&port->cd_timer, jiffies + HZ / 2);
		} else {
			/*
			* Setup the card detect interrupt at this point, otherwise the first
			* event detection will not work when the system is booted with a
			* plugged card.
			*/
			if ((retval = fim_sd_prepare_cd_irq(port))) {
				printk_err("Failed card detect IRQ setup\n");
				goto exit_free_cdirq;
			}
		}
	}
	else {
		/*
		 * There is no GPIO for the Card Detect functionality:
		 * Assume the card is always plugged.
		 */
		port->cd_irq = -1;
		port->cd_value = 1;
	}

	INIT_WORK(&port->restart_work, fim_sd_restart_work_func);

	/* And enable the FIM-interrupt */
	fim_enable_irq(&port->fim);
	fim_set_ctrl_reg(&port->fim, FIM_SDIO_CONTROL1_REG, FIM_SDIO_CONTROL1_START);

	/* Print the firmware version */
	fim_get_stat_reg(&port->fim, FIM_SDIO_FIMVER_REG, &fwver);
	printk_dbg("FIM%d running [fw rev 0x%02x]\n", port->fim.picnr, fwver);
	return 0;

 exit_free_cdirq:
	free_irq(port->cd_irq, port);

 exit_put_clk:
	clk_put(port->sys_clk);

 exit_free_host:
	mmc_free_host(port->mmc);

 exit_free_gpios:
	for (cnt=0; gpios[cnt].nr < FIM_SDIO_MAX_GPIOS; cnt++) {
		if (gpios[cnt].nr == FIM_LAST_GPIO)
			break;
		if (gpios[cnt].nr != FIM_GPIO_DONT_USE)
			gpio_free(gpios[cnt].nr);
	}

 exit_unreg_fim:
	fim_unregister_driver(&port->fim);

	return retval;
}

static int __devinit fim_sdio_probe(struct platform_device *pdev)
{
	struct fim_sdio_t *port;
	struct fim_sdio_platform_data *pdata;
	struct fim_gpio_t gpios[FIM_SDIO_MAX_GPIOS];
	int retval;

	printk_debug("Probing a new device with ID %i\n", pdev->id);

	pdata = pdev->dev.platform_data;
	if (!pdata)
		return -ENXIO;

	if (fim_check_device_id(fims_number, pdata->fim_nr)) {
#if defined(MODULE)
		printk_dbg("Skipping FIM%i (not selected)\n", pdata->fim_nr);
#else
		printk_err("Invalid FIM number '%i' in platform data\n", pdata->fim_nr);
#endif
		return -ENODEV;
	}

	port = fim_sdios->ports + pdata->fim_nr;

#if defined(MODULE)
	/* Check CD and WP parameters */
	if (0 == pdata->fim_nr) {
		if (cd0_gpio == FIM_GPIO_DONT_USE)
			printk_info("Card Detect functionality disabled for FIM 0\n");
		else {
			if (!gpio_issocgpio(cd0_gpio)) {
				cd0_gpio = FIM_GPIO_DONT_USE;
				printk_info("Invalid argument 'cd0'\n");
			}
			else {
				if (cd0_gpio_func < NS921X_GPIO_FUNC_0 ||
				cd0_gpio_func > NS921X_GPIO_FUNC_4 ) {
					printk_info("Invalid argument 'cd0_func'\n");
					return -EINVAL;
				}
			}
		}

		if (wp0_gpio == FIM_GPIO_DONT_USE)
			printk_info("Write Protect functionality disabled for FIM 0\n");
		else {
			if (!gpio_issocgpio(wp0_gpio)) {
				wp0_gpio = FIM_GPIO_DONT_USE;
				printk_info("Invalid argument 'wp0'\n");
			}
		}
	}
	if (1 == pdata->fim_nr) {
		if (cd1_gpio == FIM_GPIO_DONT_USE)
			printk_info("Card Detect functionality disabled for FIM 1\n");
		else  {
			if (!gpio_issocgpio(cd1_gpio)) {
				cd1_gpio = FIM_GPIO_DONT_USE;
				printk_info("Invalid argument 'cd1'\n");
			}
			else {
				if (cd1_gpio_func < NS921X_GPIO_FUNC_0 ||
				cd1_gpio_func > NS921X_GPIO_FUNC_4 ) {
					printk_info("Invalid argument 'cd1_func'\n");
					return -EINVAL;
				}
			}
		}

		if (wp1_gpio == FIM_GPIO_DONT_USE)
			printk_info("Write Protect functionality disabled for FIM 1\n");
		else  {
			if (!gpio_issocgpio(wp1_gpio)) {
				wp1_gpio = FIM_GPIO_DONT_USE;
				printk_info("Invalid argument 'wp1'\n");
			}
		}
	}
#endif

	/* Get the GPIOs-table from the platform data structure */
	gpios[FIM_SDIO_D0_GPIO].nr   = pdata->d0_gpio_nr;
	gpios[FIM_SDIO_D0_GPIO].func = pdata->d0_gpio_func;
	gpios[FIM_SDIO_D1_GPIO].nr   = pdata->d1_gpio_nr;
	gpios[FIM_SDIO_D1_GPIO].func = pdata->d1_gpio_func;
	gpios[FIM_SDIO_D2_GPIO].nr   = pdata->d2_gpio_nr;
	gpios[FIM_SDIO_D2_GPIO].func = pdata->d2_gpio_func;
	gpios[FIM_SDIO_D3_GPIO].nr   = pdata->d3_gpio_nr;
	gpios[FIM_SDIO_D3_GPIO].func = pdata->d3_gpio_func;
	gpios[FIM_SDIO_CLK_GPIO].nr   = pdata->clk_gpio_nr;
	gpios[FIM_SDIO_CLK_GPIO].func = pdata->clk_gpio_func;
	gpios[FIM_SDIO_CMD_GPIO].nr   = pdata->cmd_gpio_nr;
	gpios[FIM_SDIO_CMD_GPIO].func = pdata->cmd_gpio_func;
#if defined(MODULE)
	/* Use GPIOs provided as arguments */
	if (0 == pdata->fim_nr) {
		gpios[FIM_SDIO_WP_GPIO].nr   = wp0_gpio;
		gpios[FIM_SDIO_WP_GPIO].func = NS921X_GPIO_FUNC_GPIO;
		gpios[FIM_SDIO_CD_GPIO].nr   = cd0_gpio;
		gpios[FIM_SDIO_CD_GPIO].func = cd0_gpio_func;
	}
	else if (1 == pdata->fim_nr) {
		gpios[FIM_SDIO_WP_GPIO].nr   = wp1_gpio;
		gpios[FIM_SDIO_WP_GPIO].func = NS921X_GPIO_FUNC_GPIO;
		gpios[FIM_SDIO_CD_GPIO].nr   = cd1_gpio;
		gpios[FIM_SDIO_CD_GPIO].func = cd1_gpio_func;
	}
#else
	/* Use platform data */
	gpios[FIM_SDIO_WP_GPIO].nr   = pdata->wp_gpio_nr;
	gpios[FIM_SDIO_WP_GPIO].func = pdata->wp_gpio_func;
	gpios[FIM_SDIO_CD_GPIO].nr   = pdata->cd_gpio_nr;
	gpios[FIM_SDIO_CD_GPIO].func = pdata->cd_gpio_func;
#endif
	port->wp_gpio = gpios[FIM_SDIO_WP_GPIO].nr;
	port->cd_gpio = gpios[FIM_SDIO_CD_GPIO].nr;
	retval = fim_sdio_register_port(&pdev->dev, port, pdata, gpios);

	return retval;
}

static int __devexit fim_sdio_remove(struct platform_device *pdev)
{
	struct fim_sdio_t *port;
	int retval;

	port = dev_get_drvdata(&pdev->dev);

	retval = fim_sdio_unregister_port(port);
	if (!retval)
		dev_set_drvdata(&pdev->dev, NULL);

	return retval;
}

static struct platform_driver fim_sdio_platform_driver = {
	.probe	= fim_sdio_probe,
	.remove	= __devexit_p(fim_sdio_remove),
	.driver	= {
		   .owner = THIS_MODULE,
		   .name  = FIM_SDIO_DRIVER_NAME,
	},
};

/*
 * This is the function that will be called when the module is loaded
 * into the kernel space
 */
static int __init fim_sdio_init(void)
{
	int retval;
	int nrpics;

	printk_debug("Starting the FIM SDIO driver.\n");

	/* Get the number of available FIMs */
	nrpics = fim_number_pics();

        /* Check for the passed number parameter */
	if (fim_check_numbers_param(fims_number)) {
		printk_err("Invalid number '%i' of FIMs to handle\n", fims_number);
		return -EINVAL;
	}

	fim_sdios = kzalloc(sizeof(struct fim_sdios_t) +
			    (nrpics * sizeof(struct fim_sdio_t)), GFP_KERNEL);
	if (!fim_sdios)
		return -ENOMEM;

	fim_sdios->fims = nrpics;
	fim_sdios->ports = (void *)fim_sdios + sizeof(struct fim_sdios_t);

	retval = platform_driver_register(&fim_sdio_platform_driver);
	if (retval)
		goto exit_free_ports;

	printk_info(DRIVER_DESC " v" DRIVER_VERSION "\n");
	return 0;

 exit_free_ports:
	kfree(fim_sdios);

	return retval;
}

/*
 * Free the requested resources (GPIOs, memory, drivers, etc.)
 * The following steps MUST be followed when unregistering the driver:
 * - First remove and free the MMC-host
 * - Unregister the FIM-driver (will free the DMA-channel)
 * - Free the GPIOs at last
 */
static void __exit fim_sdio_exit(void)
{
	printk_info("Removing the FIM SDIO driver\n");
	platform_driver_unregister(&fim_sdio_platform_driver);
	kfree(fim_sdios);
}

module_init(fim_sdio_init);
module_exit(fim_sdio_exit);

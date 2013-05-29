/* -*- linux-c -*-
 *
 * drivers/fims/fim_can.c
 *
 * Copyright (C) 2008-2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 *  !Revision:   $Revision: 1.2 $
 *  !Author:     Luis Galdos, Hector Oron
 *  !Descr:
 *  !References: Based on the virtual CAN driver (Copyright (c) 2002-2007
 *               Volkswagen Group Electronic Research)
 *               All rights reserved.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/semaphore.h>

/* Header files for the CAN-stack */
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/netlink.h>

/* For registering the FIM-driver */
#include <mach/fim-ns921x.h>
#include <mach/gpio.h>

/* If the driver is being to be loaded as a built-in driver, then include the header */
#if !defined(MODULE)
#include "fim_can.h"
extern const unsigned char fim_can_firmware[];
#define FIM_CAN_FIRMWARE_FILE			(NULL)
#define FIM_CAN_FIRMWARE_CODE			fim_can_firmware
#else
const unsigned char *fim_can_firmware = NULL;
#define FIM_CAN_FIRMWARE_FILE			"fim_can.bin"
#define FIM_CAN_FIRMWARE_CODE			(NULL)
#endif

/* Driver informations */
#define DRIVER_VERSION				"1.3"
#define DRIVER_AUTHOR				"Luis Galdos, Hector Oron"
#define DRIVER_DESC				"FIM CAN bus driver"
#define FIM_DRIVER_NAME				"fim-can"
#define FIM_DRIVER_FIRMWARE_NAME		"fim_can.bin"
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

/* Max number of socket buffers for loopback */
#define TX_ECHO_SKB_MAX	1

/* Module parameter for selection the FIMs */
NS921X_FIM_NUMBERS_PARAM(fims_number);

#define printk_err(fmt, args...)		printk(KERN_ERR "[ ERROR ] fim-can: " fmt, ## args)
#define printk_info(fmt, args...)		printk(KERN_INFO "fim-can: " fmt, ## args)
#define printk_dbg(fmt, args...)		printk(KERN_DEBUG "fim-can: " fmt, ## args)

#if 0
#define FIM_CAN_DEBUG
#endif

#ifdef FIM_CAN_DEBUG
#  define printk_debug(fmt, args...)		printk(KERN_DEBUG "fim-can: " fmt, ## args)
#else
#  define printk_debug(fmt, args...)		do { } while (0)
#endif

#define FIM_PRIVATE_BUFFER			((void *)0xffffffff)

/* Available commands for the PIC */
#define FIM_CAN_CMD_TX				0x00
#define FIM_CAN_CMD_SHUTDOWN			0x40
#define FIM_CAN_CMD_CONFIG			0x41
#define FIM_CAN_CMD_FILTER(i)			(0x80 + i)
#define FIM_CAN_ERR_FILTER(i)			(i > 0x87 || i < 0x80)
#define FIM_CAN_NR_FILTERS			(8)

/* Default timing values (see Net+OS driver: nacbus.c) */
#define FIM_CAN_DEFAULT_SJW			(7)
#define FIM_CAN_DEFAULT_PROPSEG			(8)
#define FIM_CAN_DEFAULT_PHASE1			(8)
#define FIM_CAN_DEFAULT_PHASE2			(8)
#define FIM_CAN_DEFAULT_BITRATE			500000

/* Maximal values from the Net+OS driver */
#define FIM_CAN_MAX_BITRATE	                (1000000)

#define CAN_BUS_RECESSIVE			(0xffffffff)
#define CAN_BUS_DOMINANT			(0x00000000)

/* Control Registers */
#define FIM_CAN_COMMAND_REG                     (0)

/*
 * Command codes that can be written into the command register.
 */
#define FIM_CAN_NORMAL_OPERATIONS               (0)
#define FIM_CAN_FLUSH_TX_Q_AND_DIE              (1)


/* Status register according to the FIM-firmware specification (page 9) */
#define FIM_CAN_BUSSTATE_REG			0
#define FIM_CAN_TXERRCNT_REG			1
#define FIM_CAN_RXERRCNT_REG			2
#define FIM_CAN_FATALERR_REG			3
#define FIM_CAN_REVISION_REG                    4

/*
 * Expected revision number of the firmware.
 */
#define FIM_CAN_EXPECTED_FIRMWARE_REV          (2)


/* Control bits of the bus state register */
#define FIM_CAN_BUSSTATE_ERRPAS			0x01
#define FIM_CAN_BUSSTATE_OFF			0x02

/* Control bits of the fatal error register */
#define FIM_CAN_FATAL_CONTROLLER		0x01
#define FIM_CAN_FATAL_CMDERR			0x40
#define FIM_CAN_FATAL_HW			0x41
#define FIM_CAN_FATAL_FILTER			0x42
#define FIM_CAN_FATAL_CONFIG			0x43

/* Interrupts from the FIM to the ARM (see can_defs.inc) */
#define FIM_CAN_INT_RESET			(0x01)
#define FIM_CAN_INT_CMD_ERR			(0x45)
#define FIM_CAN_INT_HW_FAILURE			(0x41)
#define FIM_CAN_INT_FILTER_ERR			(0x42)
#define FIM_CAN_INT_CFG_ERR			(0x43)

/*
 * Macros for the configuration of the DMA-channel
 */
#define FIM_CAN_DMA_BUFFER_SIZE			(256)
#define FIM_CAN_DMA_BUFFERS			(60)

/*
 * @XXX: Remove this ugly macro and use a structure for the PIC-message according
 * to the specification, page 8 (Transmit message)
 */
#define FIRST_DATA_BYTE_IN_MESSAGE		(5)

/*
 * Default number of times we will have FIM retry transmits.  Note that the spec
 * actually says we should retry forever.  However, this doesn't work out well
 * on real networks, so we put a maximum number of retries.  This is our default
 * which I picked out of thin air.
 */
#define DEFAULT_MAX_RETRIES         (8)

/*
 * GPIO configuration
 */
#define FIM_CAN_MAX_GPIOS			(2)

/* Structure for setting the ID-filters */
#define FIM_CAN_FILTER_SIZE			(8)
struct fim_can_filter_t {
	u8 code;
	u8 filter[FIM_CAN_FILTER_SIZE];
};

/*
 * Initializing some needed variables. This is important step
 * as CAN driver might not work upon this values
 */
static struct can_bittiming_const fim_bittiming_const = {
	.tseg1_min = 1,
	.tseg1_max = 200,
	.tseg2_min = 1,
	.tseg2_max = 200,
	.sjw_max = 40,
	.brp_min = 1,
	.brp_max = 2048,
	.brp_inc = 1,
};

/*
 * This structure provides the interface for configuring the timing parameters of
 * the CAN-controller.
 */
struct fim_can_timing_t {
	u8 code;
	u16 sjw;
	u16 cspl;
	u16 cp2pl;
	u16 csi;
	u16 cp2i;
	u16 csp;
} __attribute__ ((packed));

/*
 * Structure for sending the shutdown command to the FIM
 * This command is required before stopping the FIM, otherwise a transmitted
 * frame can be corrupted
 */
struct fim_can_shutdown_t {
	u8 code;
};

/*
 * Structure for sending user-data to the controller
 * @XXX: The maximal frame length is just an arbitrary value (can be modified)
 */
#define FIM_CAN_FRAME_LENGTH			128
struct fim_can_txframe {
	u16 crc;
	u32 bytepos;
	u32 bitcnt;
	u32 bitstuff;
	u32 last5;
	u32 stuffed;
	u32 len;
	u32 result;
	u8 buffer[FIM_CAN_FRAME_LENGTH];
};

/* @FIXME: Use the generic structure for the CAN-frames from <include/linux/can.h> */
struct fim_can_rxframe {
	u8 id[4];
	u8 ctrl;
	u8 data[8];
} __attribute__ ((packed));

struct fim_can_t {
	struct can_priv can;
	int index;
	struct fim_driver fim;
	struct semaphore sem;
	struct net_device *dev;
	int reg;
 	struct fim_gpio_t gpios[FIM_CAN_MAX_GPIOS];
	struct clk *cpu_clk;
	int opened;
	spinlock_t lock;
	unsigned char id;
	unsigned char max_retries;
};

/* Function prototypes */
static int fim_can_send_skb(struct fim_can_t *port, unsigned char *data, int len,
			    struct sk_buff *skb);
static int fim_can_send_buffer(struct fim_can_t *port, unsigned char *data, int len);
static int fim_can_restart_fim(struct fim_can_t *port);
static int fim_can_start_fim(struct fim_can_t *port);
static int fim_can_stop_fim(struct fim_can_t *port);

/* Debug function for dumping the CAN-frames for the FIM-firmware */
#if defined(FIM_CAN_DEBUG)
#define fim_can_dump_frame(f, m)		fim_can_do_dump_frame(f, m)
#else
#define fim_can_dump_frame(f, m)		do { } while (0)
#endif
inline static void fim_can_do_dump_frame(struct fim_can_txframe *frame, char *marke)
{
	int cnt;
	char buffer[512];
	unsigned int pos;
	int len;
	unsigned char *data;

	len = 3 + (frame->stuffed + 7) / 8;
	data = (unsigned char *)(frame->buffer);
	pos = 0;
	for (cnt = 0; cnt < len; cnt++)
		pos += snprintf(buffer + pos, sizeof(buffer), "0x%02x ", *(data + cnt));

	printk_debug("%s buffer : %s\n", marke, buffer);
	printk_debug("%s infos  :\n\
\tCRC           : 0x%x\n\
\tbytePos       : 0x%x\n\
\tbitCount      : 0x%x\n\
\tbitStuff      : 0x%x\n\
\tlast 5        : 0x%x\n\
\tstuffed count : 0x%x\n",
		     marke,
		     frame->crc,
		     frame->bytepos,
		     frame->bitcnt,
		     frame->bitstuff,
		     frame->last5,
		     frame->stuffed);
}

/* Debug function for dumping the CAN-timing parameters */
#if defined(FIM_CAN_DEBUG)
#define fim_can_dump_timing(cfg, s, p, y, b)	fim_can_do_dump_timing(cfg, s, p, y, b)
#else
#define fim_can_dump_timing(cfg, s, p, y, b)	do { } while (0)
#endif

/* Print the timing values for the controller */
inline static void fim_can_do_dump_timing(struct fim_can_timing_t *cfg,
					  unsigned int sjw, unsigned int sample_point,
					  unsigned int sync_period, unsigned int bitrate)
{
	printk_debug("SJW %u | Sample point %u | Sync period %u | Bitrate %u\n\
\tSJW   : 0x%04x\n\
\tCSPL  : 0x%04x\n\
\tCP2PL : 0x%04x\n\
\tCSI   : 0x%04x\n\
\tCP2I  : 0x%04x\n\
\tCSP   : 0x%04x\n\
",
		     sjw,
		     sample_point,
		     sync_period,
		     bitrate,
		     cfg->sjw,
		     cfg->cspl,
		     cfg->cp2pl,
		     cfg->csi,
		     cfg->cp2i,
		     cfg->csp);
}

inline static void fim_can_parse_filter(struct fim_can_filter_t *flt, canid_t id,
					canid_t mask)
{
	flt->filter[0] = (id >> 24);
	flt->filter[1] = (mask >> 24);
	flt->filter[2] = (id >> 16);
	flt->filter[3] = (mask >> 16);
	flt->filter[4] = (id >> 8);
	flt->filter[5] = (mask >> 8);
	flt->filter[6] = id;
	flt->filter[7] = mask;
}

/* Calculate the latency times for the passed bitrate */
static int fim_can_calculate_latency(u32 bitrate, u16 *rx, u16 *tx)
{
	int ret;

	if (bitrate <= 0 || bitrate > FIM_CAN_MAX_BITRATE)
		return -ERANGE;

	/*
	 * In the future we will probably need different latency times for the
	 * bitrates. Let us keep this function for this purpose.
	 */
#define FIM_CAN_RXIRQ_LATENCY		(1)
#define FIM_CAN_TXIRQ_LATENCY		(100)
	*rx = FIM_CAN_RXIRQ_LATENCY;
	*tx = FIM_CAN_TXIRQ_LATENCY;
	ret = 0;

	return ret;
}

static int fim_can_shutdown(struct fim_can_t *port)
{
	struct fim_buffer_t buf;
	struct fim_can_shutdown_t cmd;

	cmd.code = FIM_CAN_CMD_SHUTDOWN;

	buf.private = FIM_PRIVATE_BUFFER;
	buf.length = sizeof(struct fim_can_shutdown_t);
	buf.data = (unsigned char *)&cmd;

	return 0;
}

/* Configures all timing values */
static void fim_can_fill_timing(struct fim_can_t *port, struct fim_can_timing_t *tim,
				u16 sjw, u16 sample_point, u16 sync_period)
{
	unsigned int ten_per;
	u16 larx, latx;
	struct can_bittiming *bt;

	bt = &port->can.bittiming;
	printk_debug("New timing: %u bps | sjw %u | sample %u | sync %u | clock %u\n",
		     bt->bitrate,
		     sjw,
		     sample_point,
		     sync_period,
		     bt->clock);

	if (fim_can_calculate_latency(bt->bitrate, &larx, &latx)) {
		printk_err("Failed latency calculation for rate %u\n", bt->bitrate);
		return;
	}

	/* Calculate how many clocks are in the ten percent of the bit rate */
	ten_per = (clk_get_rate(port->cpu_clk) + (5 * bt->bitrate)) / (10 * bt->bitrate);
	printk_debug("Ten percent by %i Bps is aprox. %i\n", bt->bitrate, ten_per);

	tim->sjw = (sjw * ten_per + 5) / 10;
	tim->cspl = (sample_point * ten_per + 5) / 10;
	tim->cp2pl = (((100 - sample_point) * ten_per) + 5) / 10;
	tim->csi = larx;
	tim->cp2i = latx;
	tim->csp = ((sync_period * ten_per) + 5) / 10;

	printk_debug("sjw: %d \tcspl: %d \tcp2pl: %d \tcsi: %d \tcp2i: %d \tcsp: %d\n",
		     tim->sjw, tim->cspl, tim->cp2pl,
		     tim->csi, tim->cp2i, tim->csp);
}

/* Configure the filter for accepting all the incoming messages */
inline static void fim_can_filter_accept(struct fim_can_filter_t *filter)
{
	memset(filter->filter, 0x00, FIM_CAN_FILTER_SIZE);
}

/* Configure the filter for rejecting all the incoming messages */
inline static void fim_can_filter_reject(struct fim_can_filter_t *filter)
{
	memset(filter->filter, 0x20, FIM_CAN_FILTER_SIZE);
}

/*
 * This function inits the CAN-controller according to its specification.
 * Unfortunately the firmware expects the configuration data and the filters in
 * only ONE message. For this reason we need a separated function only for the
 * init-configuration
 */
static int fim_can_set_init_config(struct fim_can_t *port)
{
	struct fim_can_timing_t cfg;
	struct fim_can_filter_t filters[FIM_CAN_NR_FILTERS];
	int cnt, retval;
	unsigned char *ptr;
	unsigned char data[sizeof(struct fim_can_timing_t) + (FIM_CAN_NR_FILTERS * 8)];
	struct can_bittiming *bt;

	/* Init the timing parameters */
	memset(&cfg, 0, sizeof(cfg));
	cfg.code = FIM_CAN_CMD_CONFIG;
	bt = &port->can.bittiming;

	if (bt->bitrate == 0) {
		printk("Sorry, bitrate required for\n");
		return -EINVAL;
	}

	/* We must use the default bit rates for our FIM-firmware */
	if (!bt->sjw)
		bt->sjw = FIM_CAN_DEFAULT_SJW;
	if (!bt->prop_seg)
		bt->prop_seg = FIM_CAN_DEFAULT_PROPSEG;

	if (!bt->phase_seg1)
		bt->phase_seg1 = FIM_CAN_DEFAULT_PHASE1;

	if (!bt->phase_seg2)
		bt->phase_seg2 = FIM_CAN_DEFAULT_PHASE2;

	fim_can_fill_timing(port, &cfg,
			    bt->sjw,
			    bt->prop_seg + bt->phase_seg1 + 1,
			    bt->phase_seg2);

	fim_can_do_dump_timing(&cfg,
			       bt->sjw,
			       bt->prop_seg + bt->phase_seg1 + 1,
			       bt->phase_seg2,
			       bt->bitrate);

	/* Not init the filter with the default values */
	for (cnt = 0; cnt < FIM_CAN_NR_FILTERS; cnt++)
		fim_can_filter_accept(&filters[cnt]);

	/* Now setup the init message */
	memcpy(data, &cfg, sizeof(struct fim_can_timing_t));
	ptr = data + sizeof(struct fim_can_timing_t);
	for (cnt = 0; cnt < FIM_CAN_NR_FILTERS; cnt++)
		memcpy(ptr, filters[cnt].filter, FIM_CAN_FILTER_SIZE);

	retval = fim_can_send_buffer(port, data, sizeof(data));
	if (retval)
		printk_err("Couldn't send the init config.\n");

	return retval;
}

/*
 * Implement the interface for chaning the state of the CAN-controller. The user
 * have access to this function over the sysfs attributes:
 * echo 2 > sys/class/net/can0/can_restart : mode = 1 (CAN_MODE_START)
 *
 */
static int fim_can_set_mode(struct net_device *dev, enum can_mode mode)
{
	struct fim_can_t *port;
	struct fim_driver *fim;
	int retval;

	port = netdev_priv(dev);
	fim = &port->fim;

	printk_debug("Set mode command %x called (FIM %i)\n", mode, fim->picnr);

	if (!port->opened) {
		printk_err("CAN-port %i was not opened up now\n", fim->picnr);
		return -ERESTARTSYS;
	}

	/*
	 * The different modes can be set using the sysfs attributes:
	 * start : /sys/class/net/can0/can_restart
	 */
	switch (mode) {
	case CAN_MODE_START:
		printk_dbg("FIM%i: Starting the controller\n", fim->picnr);
		retval = fim_can_restart_fim(port);
		break;
	case CAN_MODE_STOP:
		printk_dbg("FIM%i: Stopping the controller\n", fim->picnr);
		retval = fim_can_stop_fim(port);
		break;
	default:
		retval = -EOPNOTSUPP;
		break;
	}

	return retval;
}

/*
 * Read the status registers of the FIM for obtaining the status information.
 */
static int fim_can_get_state(const struct net_device *dev, enum can_state *state)
{
	struct fim_can_t *port;
	struct fim_driver *fim;
	int retval;
	unsigned int bus, fatal;

	port = netdev_priv(dev);
	fim = &port->fim;
	printk_debug("Get state called (FIM %i)\n", fim->picnr);

	retval = fim_get_exp_reg(fim, FIM_CAN_BUSSTATE_REG, &bus);
	if (retval) {
		printk_err("Reading the bus state register (%i)\n", retval);
		goto exit_all;
	}

	retval = fim_get_exp_reg(fim, FIM_CAN_FATALERR_REG, &fatal);
	if (retval) {
		printk_err("Reading the fatal error register (%i)\n", retval);
		goto exit_all;
	}

	/* @XXX: Are we using the correct flag for informing about the error? */
	if (fatal) {
		*state = CAN_STATE_BUS_OFF;
		goto exit_all;
	}

	/* Check the current status of the BUS */
	if (port->can.state == CAN_STATE_STOPPED) {
		printk_debug("Controller was stopped\n");
		*state = CAN_STATE_STOPPED;
	} else if (bus & FIM_CAN_BUSSTATE_OFF) {
		printk_debug("Current Bus state seems to be OFF\n");
		*state = CAN_STATE_BUS_OFF;
	} else if (bus & FIM_CAN_BUSSTATE_ERRPAS) {
		printk_debug("Bus seems to be in the passive error state\n");
		*state = CAN_STATE_ERROR_PASSIVE;
	} else
		*state = CAN_STATE_ERROR_ACTIVE;

	retval = 0;

exit_all:
	return retval;
}

/* Return zero if the FIM was successful stopped */
static int fim_can_stop_fim(struct fim_can_t *port)
{
	int retval;
	struct fim_driver *fim;

	if (!port)
		return -ENODEV;

	/*
	 * We will disable the IRQ anyway (otherwise is too dangerous having the
	 * active interrupt)
	 */
	retval = 0;
	fim = &port->fim;
	if (fim_is_running(fim)) {
		retval = fim_send_stop(fim);
		printk_debug("%s: Disabling interrupt\n",__func__);
		fim_disable_irq(fim);
	}

	return retval;
}

/*
 * This function will download the firmware, start and reconfigure the CAN-controller
 * For starting the FIM we assume that it was already stopped!
 */
static int fim_can_start_fim(struct fim_can_t *port)
{
	struct fim_driver *fim;
	int retval;
        unsigned int revision;

	if (!port)
		return -ENODEV;

	fim = &port->fim;
	if (fim_is_running(fim)) {
		printk_debug("The FIM %i is already running\n", fim->picnr);
		return 0;
	}

	/*
	 * The function for downloading the firmware will stop the FIM if
	 * its still running
	 */
	retval = fim_download_firmware(fim);
	if (retval)
		return -EAGAIN;

        /*
         * Make sure command register is clear.
         */
        fim_set_ctrl_reg(&port->fim, FIM_CAN_COMMAND_REG, FIM_CAN_NORMAL_OPERATIONS);

	retval = fim_send_start(fim);
	if (retval)
		return -EAGAIN;

	printk_debug("Enable interrupt\n");
	fim_enable_irq(fim);

	/*
	 * Disable the interrupt if an error is detected, otherwise the system
	 * can hang up
	 */
	retval = fim_can_set_init_config(port);
	if (retval) {
		printk_debug("%s: Disabling interrupt\n",__func__);
		fim_disable_irq(fim);
		return retval;
	}

        /*
         * Verify the correct FIM firmware is installed.
         */
        msleep(5);                  /* make sure FIM has a chance to start running*/
	retval = fim_get_exp_reg(fim, FIM_CAN_REVISION_REG, &revision);
	if (retval) {
		printk_err("Error reading the revision register (%i)\n", retval);
		return -EINVAL;
	}

        if (FIM_CAN_EXPECTED_FIRMWARE_REV != revision) {
		printk_err("Incorrect FIM firware.\n");
		printk_err("Revision %d is required, but revision %d was found.\n", FIM_CAN_EXPECTED_FIRMWARE_REV, revision);
		return -EINVAL;
	}

	return retval;
}

/*
 * This function restart the FIM
 * It includes the reconfiguration of the CAN-port too (it will send the init
 * configuration to the port too)
 */
static int fim_can_restart_fim(struct fim_can_t *port)
{
	int retval;

	if (!port)
		return -ENODEV;

	/* First try to stop the FIM */
	retval = fim_can_stop_fim(port);
	if (retval)
		return retval;

	return fim_can_start_fim(port);
}

/*
 * This function is called when the interface is going up (ifconfig <iface> up)
 * Only enable the interrupt and restart the netdev queue.
 */
static int fim_can_open(struct net_device *dev)
{
	struct fim_can_t *port;
	struct fim_driver *fim;
	int retval;
	struct fim_dma_cfg_t dma_cfg;

	/* So, lets start the PIC with our default data */
	port = netdev_priv(dev);
	fim = &port->fim;
	printk_debug("Open function called (FIM %i)\n", fim->picnr);

	/* Always set the CAN-interface to the active state! */
	retval = fim_unregister_driver(fim);
	/* Specific DMA configuration for the CAN-controller */
	dma_cfg.rxnr = FIM_CAN_DMA_BUFFERS;
	dma_cfg.txnr = FIM_CAN_DMA_BUFFERS;
	dma_cfg.rxsz = FIM_CAN_DMA_BUFFER_SIZE;
	dma_cfg.txsz = FIM_CAN_DMA_BUFFER_SIZE;
	port->fim.dma_cfg = &dma_cfg;
	retval = fim_register_driver(&port->fim);
	if (retval) {
		printk_err("fim_can_open: Couldn't register the FIM %i CAN driver.\n", fim->picnr);
		goto fim_can_open_exit;
	}
	retval = fim_can_restart_fim(port);
	if (!retval) {
		netif_start_queue(dev);
		port->can.state = CAN_STATE_ERROR_ACTIVE;
		port->opened = 1;
	}

fim_can_open_exit:
	return retval;
}

/*
 * This function is called when the interface is going down (ifconfig <iface> down)
 * Disable the IRQ and the netif queue.
 */
static int fim_can_stop(struct net_device *dev)
{
	struct fim_can_t *port;
	struct fim_driver *fim;
	int retval = 0;

	port = netdev_priv(dev);
	fim = &port->fim;
	printk_debug("Close function called (FIM %i)\n", fim->picnr);

	/* Paranoic sanity check */
	if (!port->opened)
		return 0;

	netif_stop_queue(dev);
	close_candev(dev);
	port->can.state = CAN_STATE_STOPPED;
	port->opened = 0;

	/* Transmit a shutdown command to the FIM before stopping it */
	if (fim_is_running(fim)) {
		retval = fim_can_shutdown(port);
		if (retval) {
			printk_err("Unable to shutdown the FIM %i\n", fim->picnr);
			goto err_exit;
		}

		/*
		 * The below delay is arbitrary and should give the FIM some time
		 * for processing the last CAN-frame on the bus
		 */
		udelay(1000);

		/* Check if need to stop the FIM */
		printk_debug("%s: Disabling interrupt\n", __func__);
		fim_disable_irq(fim);
		fim_send_stop(fim);
	}

err_exit:
	return retval;
}

/*
 * This function will save the SKB-pointer inside the FIM-buffer. By this way
 * is possible to free the SKB inside the TX-callback
 */
static int fim_can_send_skb(struct fim_can_t *port, unsigned char *data, int len,
			    struct sk_buff *skb)
{
	struct fim_buffer_t buf;

	buf.private = skb;
	buf.length = len;
	buf.data = data;
	return fim_send_buffer(&port->fim, &buf);
}

/*
 * This function will mask the FIM-buffer so that in the TX-callback we can
 * know that it doesn't contain any important information for us
 */
static int fim_can_send_buffer(struct fim_can_t *port, unsigned char *data, int len)
{
	struct fim_buffer_t buf;

	buf.private = FIM_PRIVATE_BUFFER;
	buf.length = len;
	buf.data = data;
	return fim_send_buffer(&port->fim, &buf);
}

/*
 * Write one bit to the frame, update the CRC, and bit stuff if we have
 * 5 consecutive bits all of the same value.
 */
#define LOWEST_FIVE_BITS_MASK			(0x1f)
static void fim_can_write_stuffbit(struct fim_can_txframe *frame, unsigned bit)
{
	bit &= 1;

	frame->buffer[frame->bytepos] = (frame->buffer[frame->bytepos] << 1) | bit;
	frame->last5 = ((frame->last5 << 1) | bit) & LOWEST_FIVE_BITS_MASK;
	frame->bitcnt++;
	frame->stuffed++;

	if (frame->bitcnt == 8) {
		frame->bytepos += 1;
		frame->bitcnt = 0;
	}

	if (frame->bitstuff && ((frame->last5 == LOWEST_FIVE_BITS_MASK) ||
				(frame->last5 == 0)))
		fim_can_write_stuffbit(frame, ~frame->last5);
}

/*
 * Real CAN polynomial is 0xc599, but this value used used
 * do to the way the algorithm works.  The high bit is
 * xor'ed in the first line.
 */
#define CAN_CRC_POLYNOMIAL_LESS_HIGH_BIT	(0x4599)
inline static void fim_can_add_crc(struct fim_can_txframe *frame, unsigned short bit)
{
	unsigned short crcnxt;

	frame->crc <<= 1;
	crcnxt = ((frame->crc & 0x8000) != 0) ^ (bit);
	frame->crc &= 0x7ffe;
	if (crcnxt)
		frame->crc ^= CAN_CRC_POLYNOMIAL_LESS_HIGH_BIT;
}

inline static void fim_can_frame_writeone(struct fim_can_txframe *frame, u32 bit)
{
	fim_can_add_crc(frame, bit);
	fim_can_write_stuffbit(frame, bit);
}

/*
 * Write one or more bits to the frame.
 */
inline static void fim_can_frame_write(struct fim_can_txframe *frame, unsigned int data,
				       int bits)
{
	int idx;
	for (idx = bits - 1; idx >= 0; idx--)
		fim_can_frame_writeone(frame, (data & (1 << idx)) != 0);
}

inline static void fim_can_frame_stop_stuff(struct fim_can_txframe *frame)
{
	frame->bitstuff = 0;
}

inline static void fim_can_frame_start(struct fim_can_txframe *frame)
{
	frame->crc = 0;
	frame->bytepos = FIRST_DATA_BYTE_IN_MESSAGE;

	frame->last5 = CAN_BUS_RECESSIVE;
	frame->bitcnt = 0;
	frame->bitstuff = 1;
	frame->stuffed = 0;
	fim_can_frame_writeone(frame, CAN_BUS_DOMINANT);
}

inline static void fim_can_frame_stop(struct fim_can_txframe *frame)
{
	frame->buffer[frame->bytepos] <<= (8 - frame->bitcnt);
	frame->buffer[frame->bytepos] |= (0xFF >> frame->bitcnt);
}

/*
 * Function that checks if the CAN-controller registered an error
 * If an error is detected, then a error frame will be passed to the socket-layer
 * Return zero if no error was detected, otherwise an value different than zero
 */
#define TRANSMIT_FAILURE        (true)
#define NOT_TRANSMIT_FAILURE    (false)
static int fim_can_check_error(struct net_device *dev, bool is_transmit_failure)
{
	struct sk_buff *skb;
	struct fim_can_t *port;
	struct fim_driver *fim;
	struct can_frame *cf;
	unsigned int bus, fatal;
	struct net_device_stats *stats = &dev->stats;

	port = netdev_priv(dev);
	fim = &port->fim;
	fim_get_exp_reg(fim, FIM_CAN_BUSSTATE_REG, &bus);
	fim_get_exp_reg(fim, FIM_CAN_FATALERR_REG, &fatal);

	if ((!bus) && (!fatal) && (!is_transmit_failure))
		return 0;

	printk_debug("Calling the error handling function (Bus 0x%x)\n", bus);

	/* Stop the FIM if it's running */
        if (fatal) {
        	if (fim_is_running(fim)) {
        		printk_debug("%s: Disabling interrupt\n",__func__);
        		fim_disable_irq(fim);
        		fim_send_reset(fim);
        	}
        }

	skb = dev_alloc_skb(sizeof(struct can_frame));
	if (!skb) {
		printk_err("Couldn't create the ERROR frame\n");
		goto end_exit;
	}

	skb->dev = dev;
	skb->protocol = htons(ETH_P_CAN);
	cf = (struct can_frame *)skb_put(skb, sizeof(struct can_frame));
	memset(cf, 0x00, sizeof(struct can_frame));
	cf->can_id = CAN_ERR_FLAG;
	cf->can_dlc = CAN_ERR_DLC;

	/*
	 * These are the possible errors defined in the FIM-CAN specification
	 * The corresponding error flags are under: include/linux/can/error.h
	 */
	if (bus & FIM_CAN_BUSSTATE_OFF) {
		cf->can_id |= CAN_ERR_BUSOFF;
		can_bus_off(dev);
        }

	/* @XXX: This seems to be the correct flags for our controller, or? */
	if (bus & FIM_CAN_BUSSTATE_ERRPAS) {
		cf->can_id |= CAN_ERR_BUSERROR;
		cf->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;
		port->can.can_stats.error_passive++;
	}

        if (is_transmit_failure) {
            cf->can_id |= CAN_ERR_ACK;
        }

	/* Pass the error frame to the socket layer */
	netif_rx(skb);

	dev->last_rx = jiffies;
	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

end_exit:
        if (   (bus & FIM_CAN_BUSSTATE_OFF)
            || (fatal)
            || (is_transmit_failure)) {
 	    return 1;
 	} else {
 	    return 0;
 	}
}

static int fim_can_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct fim_can_t *port;
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct fim_can_txframe frame;
	u8 dlc, cnt;
	int retval;
	canid_t id;

	port = netdev_priv(dev);
	spin_lock(&port->lock);
	netif_stop_queue(dev);
	spin_unlock(&port->lock);

	cf = (struct can_frame *)skb->data;

	id = cf->can_id;
	dlc = cf->can_dlc;

	/*
	 * Check for the bus state before sending the data
	 * @FIXME: The controller fails if its state is the error passive
	 */
	if (fim_can_check_error(dev, NOT_TRANSMIT_FAILURE)) {
		printk_debug("Bus OFF or unknown error. Aborting Xmit %p\n", skb);
		stats->tx_dropped++;
		spin_lock(&port->lock);
		dev_kfree_skb(skb);
		netif_wake_queue(dev);
		spin_unlock(&port->lock);
		return NETDEV_TX_OK;
	}

	/* Reset our frame first */
	memset(&frame, 0x0, sizeof(struct fim_can_txframe));

	fim_can_frame_start(&frame);
	fim_can_dump_frame(&frame, "START");

	/* Check for the extended ID (18 bits length) */
	if (id & CAN_EFF_FLAG) {
                fim_can_frame_write(&frame, id >> 18, 11);
		fim_can_frame_write(&frame, 1, 2);          /* write IDE and RTS as recessive*/
		fim_can_frame_write(&frame, id, 18);        /* write rest of ID*/
		fim_can_frame_write(&frame, (id & CAN_RTR_FLAG) != 0, 1);
		fim_can_frame_write(&frame, 0, 2);          /* write R0 and R1*/
        } else {
                fim_can_frame_write(&frame, id, 11);
		fim_can_frame_write(&frame, (id & CAN_RTR_FLAG) != 0, 1);
		fim_can_frame_write(&frame, 0, 1);          /* write IDE dominant for standard format*/
                fim_can_frame_write(&frame, 1, 1);          /* write R0 recessive*/
        }
	fim_can_dump_frame(&frame, "ID");

	fim_can_dump_frame(&frame, "RTR");

	/* Write the control field (data length passed by the user) */
	fim_can_frame_write(&frame, dlc, 4);
	fim_can_dump_frame(&frame, "CONTROL");

	/*
	 * Now write the data bytes (maximal eight bytes)
	 * According to the CAN-specification, don't add the data bytes to the frame
	 * although the data length code is greter than zero
	 */
	for (cnt = 0; cnt < (dlc & 0x0F) && !(id & CAN_RTR_FLAG); cnt++)
		fim_can_frame_write(&frame, cf->data[cnt], 8);

	fim_can_dump_frame(&frame, "DATA");

	/* Write the CRC */
	fim_can_frame_write(&frame, frame.crc, 15);
	fim_can_dump_frame(&frame, "CRC");

	/* Stop the bit stuffing for this frame */
	fim_can_frame_stop_stuff(&frame);

	/* Write the CRC delimiter, CRC ACK and ACK delimiter */
	fim_can_frame_write(&frame, CAN_BUS_RECESSIVE, 3);
	fim_can_dump_frame(&frame, "CRC/ACK");

	fim_can_frame_stop(&frame);

	/*
	 * Set the first three byte of the frame as described in the specification
	 * (see page 8, transmit message structure)
	 */
	frame.buffer[0] = FIM_CAN_CMD_TX;
	frame.buffer[1] = ((id & CAN_EFF_FLAG) ? 29 : 11) + 1;
	frame.buffer[2] = frame.stuffed;
	frame.buffer[3] = port->id++;
	frame.buffer[4] = port->max_retries;
	fim_can_dump_frame(&frame, "END");

	printk_debug("Sending a new SKB %p\n", skb);
	retval = fim_can_send_skb(port, frame.buffer, 5 + (frame.stuffed + 7) / 8, skb);
	if (retval) {
		stats->tx_fifo_errors++;
		stats->tx_dropped++;
		printk_err("Couldn't send a frame with the ID 0x%X\n", id);
		goto exit_free_skb;
	}

	stats->tx_bytes += dlc;
	stats->tx_packets++;
	dev->trans_start = jiffies;

exit_free_skb:
	return NETDEV_TX_OK;
}

/*
 * Only errors are being generated by the firmware
 * If an error ocurrs the FIM must be resetted (it hangs up in a endless loop)
 */
static void fim_can_isr(struct fim_driver *driver, int irq, unsigned char code,
			unsigned int rx_fifo)
{
	struct net_device *dev;
	struct fim_can_t *port;

	printk_debug("FIM IRQ %x | %08x\n", code, rx_fifo);

	port = driver->driver_data;
	dev = port->dev;

	if (code && code != FIM_CAN_INT_RESET) {
		printk_debug("%s: Disabling interrupt\n",__func__);
		fim_disable_irq(driver);
		fim_send_stop(driver);
	}

	switch (code) {
	case FIM_CAN_INT_CMD_ERR:
		printk_err("Command error (FIM %i)\n", driver->picnr);
		break;
	case FIM_CAN_INT_HW_FAILURE:
		printk_err("HW failure (FIM %i). Please restart the controller.\n",
			   driver->picnr);
		break;
	case FIM_CAN_INT_FILTER_ERR:
		printk_err("Filter error (FIM %i)\n", driver->picnr);
		break;
	case FIM_CAN_INT_CFG_ERR:
		printk_err("Config error (FIM %i)\n", driver->picnr);
		break;
	default:
		break;
	}

	/*
	 * Check if we have a stop condition of the CAN-controller
	 * If we need to stop the controller, restart the netif-queue anyway, then
	 * otherwise we will have problems when freeing the SKBs
	 */
	if (code && code != FIM_CAN_INT_RESET) {
		if (netif_queue_stopped(dev))
			netif_wake_queue(dev);
	}
}

static void fim_can_tx_isr(struct fim_driver *driver, int irq,
			   struct fim_buffer_t *pdata)
{
	struct fim_can_t *port;
	struct fim_buffer_t *buf;
	struct net_device *dev;
	unsigned long flags;
	struct sk_buff *skb;

	port = driver->driver_data;
	buf = pdata->private;
	dev = port->dev;
	skb = pdata->private;

	spin_lock_irqsave(&port->lock, flags);
	printk_debug("TX callback | SKB %p | dev %p\n", skb, dev);

        /*
         * Older versions of this driver used to do an error check here.  This
         * new version relies on feature in the FIM code which sends a transmit
         * result message for each transmit frame that tells the driver whether
         * the frame was successfully transmitted or not.  This message is received
         * by fim_can_rx_isr and it uses that message to do error checking.
         */

	/*
	 * Check if we have a SKB to free
	 * IMPORTANT: Allways free the SKB, otherwise we will have a memory leak,
	 * although in the template CAN-drivers they are not freeing the SKB
	 */
	if (skb && skb != FIM_PRIVATE_BUFFER) {
		printk_debug("Freeing the   SKB %p\n", skb);
		dev_kfree_skb_irq(skb);
	}

	/*
	 * Although we had a failure, we must restart the TX-queue, otherwise
	 * we will have a kernel panic when freeing the SKBs
	 */
	if (dev && netif_queue_stopped(dev)) {
		printk_debug("Restarting the netif-queue\n");
		netif_wake_queue(dev);
	}

	spin_unlock_irqrestore(&port->lock, flags);
}

/*
 * This function is called when the FIM has a new CAN-frame for us
 * According to the configured filters we will receive only the filtered messages
 */
static void fim_can_rx_isr(struct fim_driver *driver, int irq,
			   struct fim_buffer_t *pdata)
{
	struct sk_buff *skb;
	struct fim_can_t *port;
	struct net_device *dev;
	struct can_frame *cf;
	struct net_device_stats *stats;
	struct fim_can_rxframe *rx;
	unsigned long flags;

	/* Set our internal data */
	rx = (struct fim_can_rxframe *)pdata->data;
	port = driver->driver_data;
	dev = port->dev;
	stats = &dev->stats;
	spin_lock_irqsave(&port->lock, flags);

        /*
         * The FIM will send a 3 byte message for each transmit frame that
         * tells us whether the transmit was successful or not.  See if
         * this is one of those frames.
         */
#define TRANSMIT_RESULT_MESSAGE_TYPE_CODE           (1)
        if ((pdata->length == 3) && (rx->id[2] == TRANSMIT_RESULT_MESSAGE_TYPE_CODE)) {
#define TRANSMIT_SUCCESS                            (0)

            if (rx->id[0] != TRANSMIT_SUCCESS) {
                /*
                 * If we get here, then the transmit has failed.  Tell the
                 * upper layer driver about it.
                 */
	        fim_can_check_error(dev, TRANSMIT_FAILURE);
	        printk_debug("Reported transmit failure to upper layer\n");
	    }
	    else {
	        printk_debug("transmit successful\n");
	        fim_can_check_error(dev, NOT_TRANSMIT_FAILURE);
	    }

            goto fim_can_rx_isr_exit;
        }

	printk_debug("New CAN-frame (%i bytes) | ID 0x%02x%02x%02x%02x\n",
		     pdata->length, rx->id[3], rx->id[2], rx->id[1], rx->id[0]);

	skb = dev_alloc_skb(sizeof(struct can_frame));
	if (!skb) {
		printk_err("No memory available? Dropping a CAN-frame!\n");
		goto fim_can_rx_isr_exit;
	}

	skb->dev = dev;
	skb->protocol = htons(ETH_P_CAN);

	cf = (struct can_frame *)skb_put(skb, sizeof(struct can_frame));
	memset(cf, 0x00, sizeof(struct can_frame));

	/* Start processing the arrived frame */

	/* Set the CAN-ID corresponding to the FIM-controller specification */
	if (rx->id[0] & 0x80)
		cf->can_id |= CAN_RTR_FLAG;

	if (rx->id[0] & 0x40)
		cf->can_id |= CAN_EFF_FLAG;

	/* Remove the control bits and set to the correct ID */
	rx->id[0] &= 0x1F;
	if (cf->can_id & CAN_EFF_FLAG)
		cf->can_id |= rx->id[3] + (rx->id[2] << 8) + (rx->id[1] << 16) +
		    (rx->id[0] << 24);
	else
		cf->can_id |= (rx->id[1] >> 2) + (rx->id[0] << 6);

	/* Copy the data */
	cf->can_dlc = rx->ctrl & 0x0F;
	memcpy(cf->data, rx->data, cf->can_dlc);

	/* Pass the data to the sockets-layer */
	netif_rx(skb);

	/* Update the device statics */
	dev->last_rx = jiffies;
	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

fim_can_rx_isr_exit:
	spin_unlock_irqrestore(&port->lock, flags);
}

static int unregister_fim_can(struct fim_can_t *port)
{
	struct fim_gpio_t gpios[FIM_CAN_MAX_GPIOS];
	int cnt;
	struct fim_driver *fim;

	if (!port || !port->reg)
		return -ENODEV;

	//unregister_netdev(port->dev);
	unregister_candev(port->dev);
	free_candev(port->dev);

	/* Activate the interrupt (@BUG in the IRQ-subsystem?) */
	fim = &port->fim;

	printk_debug("%s: Disabling interrupt\n", __func__);
	fim_disable_irq(fim);

	/* Stop the FIM first */
	if (!fim_is_running(fim))
		fim_send_stop(fim);

	printk_debug("Going to unregister the FIM %i (running %i)\n",
		    fim->picnr, fim_is_running(fim));
	fim_unregister_driver(fim);

	/* And free the GPIOs */
	memcpy(gpios, port->gpios, sizeof(struct fim_gpio_t) * FIM_CAN_MAX_GPIOS);
	for (cnt = 0; cnt < FIM_CAN_MAX_GPIOS; cnt++) {

		if (gpios[cnt].nr != FIM_GPIO_DONT_USE) {
			printk_debug("Freeing the GPIO %i\n", gpios[cnt].nr);
			gpio_free(gpios[cnt].nr);
		}
	}

	/* Free the obtained clock */
	if (!IS_ERR(port->cpu_clk))
		clk_put(port->cpu_clk);

	port->reg = 0;

	return 0;
}

static const struct net_device_ops fim_can_netdev_ops = {
	.ndo_open = fim_can_open,
	.ndo_stop = fim_can_stop,
	.ndo_start_xmit = fim_can_xmit,
};

/*
 * IMPORTANT: First register the FIM-driver, and at last the CAN-device, then
 * it will automatically start with the bit time configuration.
 */
static int register_fim_can(struct platform_device *pdev, int picnr, struct fim_gpio_t gpios[],
			    int bitrate)
{
	int retval, cnt;
	int func;
	struct net_device *dev;
	struct fim_can_t *port;
	struct fim_dma_cfg_t dma_cfg;

	/* Now create the net device */
	dev = alloc_candev(sizeof(struct fim_can_t), TX_ECHO_SKB_MAX);
	if (!dev) {
		printk_err("Couldn't alloc a new CAN device\n");
		return -ENOMEM;
	}

	/* Set our port structure as private data */
	port = netdev_priv(dev);
	port->dev = dev;
	/* Get a reference to the SYS clock for setting the baudrate */
	if (IS_ERR(port->cpu_clk = clk_get(&dev->dev, "systemclock"))) {
		printk_err("Couldn't get the SYS clock.\n");
		goto err_free_candev;
	}

	/* Request the corresponding GPIOs (@XXX: Check the returned values) */
	for (cnt=0; cnt < FIM_CAN_MAX_GPIOS; cnt++) {
		if (gpios[cnt].nr == FIM_LAST_GPIO)
			break;
		if (gpios[cnt].nr == FIM_GPIO_DONT_USE)
			continue;
		printk_debug("Requesting the GPIO %i (Function %i)\n",
			     gpios[cnt].nr, gpios[cnt].func);
		retval = gpio_request(gpios[cnt].nr, FIM_DRIVER_NAME);
		if (!retval) {
			func = gpios[cnt].func;
			gpio_configure_ns921x_unlocked(gpios[cnt].nr,
						       NS921X_GPIO_INPUT,
						       NS921X_GPIO_DONT_INVERT,
						       func,
						       NS921X_GPIO_ENABLE_PULLUP);
		} else {
			/* Free the already requested GPIOs */
			printk_err("Couldn't request the GPIO %i\n", gpios[cnt].nr);
			while (cnt)
				gpio_free(gpios[--cnt].nr);
			goto err_free_candev;
		}
	}

	/* Now try to register the FIM-driver */
	port->fim.picnr = picnr;
	port->fim.driver.name = FIM_DRIVER_NAME;
	port->fim.fim_isr = fim_can_isr;
	port->fim.dma_tx_isr = fim_can_tx_isr;
	port->fim.dma_rx_isr = fim_can_rx_isr;
	port->fim.driver_data = port;

	/* Specific DMA configuration for the CAN-controller */
	dma_cfg.rxnr = FIM_CAN_DMA_BUFFERS;
	dma_cfg.txnr = FIM_CAN_DMA_BUFFERS;
	dma_cfg.rxsz = FIM_CAN_DMA_BUFFER_SIZE;
	dma_cfg.txsz = FIM_CAN_DMA_BUFFER_SIZE;
	port->fim.dma_cfg = &dma_cfg;

	/* Check if have a firmware code for using to */
	port->fim.fw_name = FIM_CAN_FIRMWARE_FILE;
	port->fim.fw_code = FIM_CAN_FIRMWARE_CODE;
	retval = fim_register_driver(&port->fim);
	if (retval) {
		printk_err("Couldn't register the FIM %i CAN driver.\n", picnr);
		goto err_free_gpios;
	}

	port->id = 0;
	port->max_retries = DEFAULT_MAX_RETRIES;

	/* Stop the FIM then it will be started in the open-function */
	fim_send_stop(&port->fim);

	/* Configure the net device with the default values */
	dev->flags |= IFF_ECHO;
	dev->netdev_ops = &fim_can_netdev_ops;

	/* Special attributes for the CAN-stack */
	port->can.do_get_state = fim_can_get_state;
	port->can.do_set_mode = fim_can_set_mode;

	port->can.clock.freq = clk_get_rate(port->cpu_clk);
 	printk_debug("CPU clock is %luHz\n", clk_get_rate(port->cpu_clk));

	/*
	 * @TODO: Set the correct maximal BRP for the controller.
	 *   DEFAULT_MAX_BRP 64
	 *   DEFAULT_MAX_SJW 4
	 */
 	port->can.bittiming_const = &fim_bittiming_const;
 	if (!port->can.bittiming_const) {
 		printk_debug("bittiming_const is not initialized.\n");
 		goto err_unreg_fim;
 	}

	/*
	 * Only store the passed bitrate. The corresponding timing parameters
	 * will be setup in the function that sends the configuration to the FIM.
	 * (Luis Galdos)
	 */
	port->can.bittiming.bitrate    = bitrate;

	spin_lock_init(&port->lock);
	port->dev = dev;
	memcpy(port->gpios, gpios, sizeof(struct fim_gpio_t) * FIM_CAN_MAX_GPIOS);
	port->reg = 1;
	dev_set_drvdata(&pdev->dev, port);
	SET_NETDEV_DEV(dev, &pdev->dev);

	/* Now register the new CAN-net device */
	retval = register_candev(dev);
	if (retval) {
		printk_err("Registering the net device for the FIM %i\n", picnr);
		goto err_unreg_fim;
	}

 	return 0;

err_unreg_fim:
	fim_unregister_driver(&port->fim);

err_free_gpios:
	for (cnt = 0; cnt < FIM_CAN_MAX_GPIOS; cnt++) {

                if (gpios[cnt].nr != FIM_GPIO_DONT_USE) {
			printk_debug("Freeing the GPIO %i\n", gpios[cnt].nr);
			gpio_free(gpios[cnt].nr);
		}
	}

err_free_candev:
	free_candev(dev);

 	return -EINVAL;
}

static __devinit int fim_can_probe(struct platform_device *pdev)
{
	int nrpics;
	int retval;
	struct fim_gpio_t gpios[FIM_CAN_MAX_GPIOS];
	struct fim_can_platform_data *pdata;
	int bitrate;

	printk_debug("Starting the FIM CAN bus driver.\n");

	pdata = pdev->dev.platform_data;
	if (!pdata)
		return -ENODEV;

	/* If no bitrate passed, then use the default bitrate */
	bitrate = pdata->fim_can_bitrate ? pdata->fim_can_bitrate :
		FIM_CAN_DEFAULT_BITRATE;

	/* Sanity check for the passed bit rate */
	if (bitrate <= 0 || bitrate > FIM_CAN_MAX_BITRATE) {
		printk_err("Invalid bit rate %i (max. rate is %i)\n",
			   bitrate, FIM_CAN_MAX_BITRATE);
		return -EINVAL;
	}

	/* Get the number of available PICs from the FIM-core */
	nrpics = fim_number_pics();

	if (fim_check_device_id(fims_number, pdata->fim_nr)) {
#if defined(MODULE)
		printk_dbg("Skipping FIM%i (not selected)\n", pdata->fim_nr);
#else
		printk_err("Invalid FIM number '%i' in platform data\n", pdata->fim_nr);
#endif
		return -ENODEV;
	}

	/*
	 * Start with the registration of the CAN-ports
	 */
	gpios[0].nr	= pdata->rx_gpio_nr;
	gpios[0].func	= pdata->rx_gpio_func;
	gpios[1].nr	= pdata->tx_gpio_nr;
	gpios[1].func	= pdata->tx_gpio_func;
	printk_debug("%s: GPIO RX: %d | GPIO TX: %d \n", __func__,
		     gpios[0].nr, gpios[1].nr );

	/* XXX SDIO code approach */
	retval = register_fim_can(pdev, pdata->fim_nr, gpios, bitrate);

	return retval;
}

static __devexit int fim_can_remove(struct platform_device *pdev)
{
	struct fim_can_t *port;

	port = dev_get_drvdata(&pdev->dev);
	if (!port) {
		printk_err("remove: NULL pointer found!\n");
		return -ENODEV;
	}

	unregister_fim_can(port);

	return 0;
}

static struct platform_driver fim_can_platform_driver = {
	.probe	= fim_can_probe,
	.remove	= __devexit_p(fim_can_remove),
	.driver	= {
		   .owner = THIS_MODULE,
		   .name  = FIM_DRIVER_NAME,
	},
};

/*
 * This is the function that will be called when the module is loaded
 * into the kernel space
 */
static __init int fim_can_init(void)
{
	printk_info(DRIVER_DESC " " DRIVER_VERSION "\n");

        /* Check for the passed number parameter */
	if (fim_check_numbers_param(fims_number)) {
		printk_err("Invalid number '%i' of FIMs to handle\n", fims_number);
		return -EINVAL;
	}

	return platform_driver_register(&fim_can_platform_driver);
}

static __exit void fim_can_exit(void)
{
	printk_info("Removing the FIM CAN driver\n");
	platform_driver_unregister(&fim_can_platform_driver);
}

module_init(fim_can_init);
module_exit(fim_can_exit);

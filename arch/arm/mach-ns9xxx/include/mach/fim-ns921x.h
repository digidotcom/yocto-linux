/* -*- linux-c -*-
 * arch/arm/mach-ns9xxx/include/mach/fim-ns921x.h
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 *  !Revision:   $Revision: 1.25 $
 *  !Author:     Silvano Najera, Luis Galdos
 *  !Descr:      
 *  !References:
 */


#ifndef _NS921X_FIM_CORE_H
#define _NS921X_FIM_CORE_H


#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/kfifo.h>
#include <linux/list.h>
#include <linux/interrupt.h>

#include <mach/dma-ns921x.h>


#define FIM_MAX_FIRMWARE_NAME			32


/* For DMA handling... */
#define	FIM_DMA_NCIP				(0x1)
#define FIM_DMA_NRIP				(0x2)
#define FIM_DMA_ECIP				(0x4)
#define FIM_DMA_CAIP				(0x8)
#define FIM_DMA_CANCELLED			FIM_DMA_CAIP
#define FIM_DMA_FLUSHED				(0x10)
#define FIM_DMA_SUCCESS		         	(FIM_DMA_NCIP | FIM_DMA_NRIP)


#define FIM_MAX_PIC_INDEX			(1)
#define FIM_MIN_PIC_INDEX			(0)
#define FIM_NR_PICS				(FIM_MAX_PIC_INDEX-FIM_MIN_PIC_INDEX+1)


/* @XXX: Place this macros in another place? */
#define NS92XX_FIM_GEN_CTRL_STOP_PIC		~NS92XX_FIM_GEN_CTRL_PROGMEM
#define NS92XX_FIM_GEN_CTRL_START_PIC		NS92XX_FIM_GEN_CTRL_PROGMEM



/* Please note that the maximal DMA-buffer size is 64kB */
/* @FIXME: Check that the maximal size of the descriptors is littler than one page */
#define PIC_DMA_RX_BUFFERS			(10)
#define PIC_DMA_TX_BUFFERS			(10)
#define PIC_DMA_BUFFER_SIZE			(1 * PAGE_SIZE)

/*
 * Internal structure for handling with the DMA-buffer descriptors
 * p_desc : Physical descriptors address
 * v_desc : Virtual access address for the descriptors
 * v_buf  : Virtual address of the memory buffers
 * length : Configured length of this buffer
 * tasked : Used for locking the descriptor
 */
struct pic_dma_desc_t {
	dma_addr_t src;
	size_t length;
	atomic_t tasked;
	void *private;
	int total_length;
};


/*
 * Structure used by the FIM-API to configure the DMA-buffer and buffer-descriptors.
 * rxnr : Number of RX-DMA-buffers
 * rxsz : Size of each DMA-buffer (in Bytes)
 * txnr : Number of TX-DMA-buffers
 * txsz : Size for each TX-buffer (in Bytes)
 */
struct fim_dma_cfg_t {
	int rxnr;
	int rxsz;
	int txnr;
	int txsz;
};


/*
 * This structure should be used for transferring data with the API
 * length  : Date length to transfer
 * data    : Data buffer
 * private : The API will not touch this pointer
 * sent    : The external driver can use it for waking up sleeping processes
 */
struct fim_buffer_t {
	int length;
	unsigned char *data;
	void *private;
	int sent;
};


/* @TODO: We need perhaps another PIC-structure for the U-Boot */
struct pic_t {
	int irq;
	struct device *dev;
	struct fim_driver *driver;
	void __iomem *reg_addr;
	void __iomem *instr_addr;
	void __iomem *hwa_addr;
	void __iomem *iohub_addr;
	spinlock_t lock;
	int index;
	atomic_t irq_enabled;
	atomic_t requested;
	
	/* RX-DMA structures */
	struct iohub_dma_fifo_t rx_fifo;
	spinlock_t rx_lock;
	struct fim_dma_cfg_t dma_cfg;
	
	/* Variables for the DMA-memory buffers */
	dma_addr_t dma_phys;
	void __iomem *dma_virt;
	size_t dma_size;
	
	/* Data for the handling of the TX-DMA buffers */
	spinlock_t tx_lock;
	struct pic_dma_desc_t *tx_desc;
	struct iohub_dma_fifo_t tx_fifo;
	atomic_t tx_tasked;
	atomic_t tx_aborted;
	struct tasklet_struct rx_tasklet;
	
	/* Info data for the sysfs */
	char fw_name[FIM_MAX_FIRMWARE_NAME];
	int fw_length;

	/* Functions for a low level access to the PICs */
	int (* is_running)(struct pic_t *);
	int (* start_at_zero)(struct pic_t *);
	int (* stop_and_reset)(struct pic_t *);
	int (* download_firmware)(struct pic_t *, const unsigned char *);
	int (* get_ctrl_reg)(struct pic_t *, int , unsigned int *);
	void (* set_ctrl_reg)(struct pic_t *, int , unsigned int );
	int (* send_interrupt)(struct pic_t *, u32 );
	void (* ack_interrupt)(struct pic_t * , int );
};


/*
 * Structure with the GPIOs to use for the driver to be initialized
 * nr     : GPIO number
 * name   : Name to use for the GPIO
 * picval : Value to pass to the PIC-firmware
 * func   : Function to be configured for the GPIO
 */
struct fim_gpio_t {
	int nr;
	char *name;
	unsigned char picval;  /* Value to pass to firmware */
	unsigned int func;
};

#define FIM_LAST_GPIO			-2
#define FIM_GPIO_DONT_USE		-1

/*
 * Internal structure for allocating a FIM driver
 * picnr       : Number of the PIC to use for this driver
 * fw_code     : Firmware code that should be used as firmware
 * fw_name     : Name of the firmware to get over the firmware layer
 * driver      : Driver structure
 * dev         : Device that should be set by the FIM-API
 * driver_data : The API will not touch this member
 * fim_isr     : Called when the PIC generates an interrupt
 * dma_tx_isr  : TX-callback function. Called in interrupt context
 * dma_rx_isr  : RX-callback. Called inside the interrupt context
 * dma_cfg     : If NULL then the API will use the default config
 * verbose     : Used by the FIM-core for printing sys messages (debug, infos, etc.)
 */
struct fim_driver {
	int picnr;
	const unsigned char *fw_code;
	const char *fw_name;
	struct device_driver driver;
	struct device *dev;
	void (*fim_isr)(struct fim_driver *, int, unsigned char, unsigned int);
	void (*dma_tx_isr)(struct fim_driver *, int, struct fim_buffer_t *);
	void (*dma_rx_isr)(struct fim_driver *, int, struct fim_buffer_t *);
	void (*dma_error_isr)(struct fim_driver *, ulong rx_err, ulong tx_err);
	void *driver_data;
	struct fim_dma_cfg_t *dma_cfg;
	int verbose;
};



/*
 * Structure for the FIM-devices with UART-support
 * If a GPIO should not be used, then it's required to disable it by using the
 * above macro 'FIM_GPIO_DONT_USE'
 *
 * fim_nr  : Number of the FIM to use for the device
 * gpio_nr : GPIO to use for the interface line
 * fim_cfg : Currently not used 
 */
struct fim_serial_platform_data {
	int fim_nr;
	
	int rx_gpio_nr;
	unsigned int rx_gpio_func;
	unsigned int rx_fim_cfg;

	int tx_gpio_nr;
	unsigned int tx_gpio_func;
	unsigned int tx_fim_cfg;

	int cts_gpio_nr;
	unsigned int cts_gpio_func;
	unsigned int cts_fim_cfg;

	int rts_gpio_nr;
	unsigned int rts_gpio_func;
	unsigned int rts_fim_cfg;
};


/* Macro for the configuration of the GPIOs for the FIM-serial driver */
#define NS921X_FIM_SERIAL_GPIOS(rx, tx, rts, cts, func)	\
		.rx_gpio_nr = rx, \
		.rx_gpio_func = func, \
		.tx_gpio_nr = tx, \
		.tx_gpio_func = func, \
		.rts_gpio_nr = rts, \
		.rts_gpio_func = func, \
		.cts_gpio_nr = cts, \
		.cts_gpio_func = func


/*
 * Structure for the FIM-devices with SDIO-support
 * If a GPIO should not be used, then it's required to disable it by using the
 * macro 'FIM_GPIO_DONT_USE'
 *
 * fim_nr    : Number of the FIM to use for the device
 * host_caps : Specific host capabilities (see: linux/mmc/host.h)
 */
struct fim_sdio_platform_data {
	int fim_nr;
	unsigned int host_caps;         /* Host capabilities */
	unsigned int min_clk;		/* Minimal SDIO clock */
	unsigned int max_clk;		/* Maximal SDIO clock (max. is 5MHz) */
	
	int d0_gpio_nr;			/* data 0 */
	unsigned int d0_gpio_func;
	int d1_gpio_nr;			/* data 1 */
	unsigned int d1_gpio_func;
	int d2_gpio_nr;			/* data 2 */
	unsigned int d2_gpio_func;
	int d3_gpio_nr;			/* data 3 */
	unsigned int d3_gpio_func;
	int wp_gpio_nr;			/* write protect */
	unsigned int wp_gpio_func;
	int cd_gpio_nr;			/* card detect */
	unsigned int cd_gpio_func;
	int clk_gpio_nr;		/* clock */
	unsigned int clk_gpio_func;
	int cmd_gpio_nr;		/* command */
	unsigned int cmd_gpio_func;
};


/*
 * Use the below macro if all the GPIOs can be configured with the same function
 * number (this is the normal case)
 */
#define NS921X_FIM_SDIO_GPIOS(d0, d1, d2, d3, wp, cd, clk, cmd, func)	\
		.d0_gpio_nr = d0, \
		.d0_gpio_func = func, \
		.d1_gpio_nr = d1, \
		.d1_gpio_func = func, \
		.d2_gpio_nr = d2, \
		.d2_gpio_func = func, \
		.d3_gpio_nr = d3, \
		.d3_gpio_func = func, \
		.wp_gpio_nr = wp, \
		.wp_gpio_func = func, \
		.cd_gpio_nr = cd, \
		.cd_gpio_func = func, \
		.clk_gpio_nr = clk, \
		.clk_gpio_func = func, \
		.cmd_gpio_nr = cmd, \
		.cmd_gpio_func = func

/* 
 * The new FIM board doesn't connect all the lines to the FIM. The CMD
 * and CD are not connected to the FIM.
 */
#define NS921X_FIM_SDIO_GPIOS_FIM(d0, d1, d2, d3, clk, cmd, func)	\
		.d0_gpio_nr = d0, \
		.d0_gpio_func = func, \
		.d1_gpio_nr = d1, \
		.d1_gpio_func = func, \
		.d2_gpio_nr = d2, \
		.d2_gpio_func = func, \
		.d3_gpio_nr = d3, \
		.d3_gpio_func = func, \
		.clk_gpio_nr = clk, \
		.clk_gpio_func = func, \
		.cmd_gpio_nr = cmd, \
		.cmd_gpio_func = func


/*
 * Passing this value as a chip select number will tell the driver that
 * the slave device has no chip select.
 */
#define FIM_SPI_NO_CHIP_SELECT      (255)

/*
 * Number of different chip selects to support.
 */
#define FIM_SPI_MAX_CS              (4)

struct fim_spi_cs_list {
    bool enabled;
    int gpio;
};

/*
 * Structure for the FIM-devices with SPI support.
 */
struct spi_ns921x_fim {
    int fim_nr;                     /* FIM number to use (0 or 1) */
    unsigned flags;
#define SPI_NS921X_SUPPORT_MASTER_CS    (1)
    int gpio_base;                  /* first of 4 GPIO pins we will use*/
    struct fim_spi_cs_list cs[FIM_SPI_MAX_CS];
};



#define NS921X_FIM_SPI_CS_GPIOS(idx, is_enabled, gpio_pin) \
                .cs[idx].enabled = is_enabled, \
                .cs[idx].gpio = gpio_pin


/*
 * Structure for the FIM-devices with CAN-support
 * If a GPIO should not be used, then it's required to disable it by using the
 * above macro 'FIM_GPIO_DONT_USE'
 *
 * fim_nr  : Number of the FIM to use for the device
 * gpio_nr : GPIO to use for the interface line
 */
struct fim_can_platform_data {

	int fim_nr;
	int fim_can_bitrate;

	int rx_gpio_nr;
	unsigned int rx_gpio_func;
	int tx_gpio_nr;
	unsigned int tx_gpio_func;
};

/* Macro for the configuration of the GPIOs for the FIM CAN driver */
#define NS921X_FIM_CAN_GPIOS(rx, tx, func)	\
		.rx_gpio_nr = rx, \
		.rx_gpio_func = func, \
		.tx_gpio_nr = tx, \
		.tx_gpio_func = func

/*
 * Structure for the FIM-devices with USB support
 * If a GPIO should not be used, then it's required to disable it by using the
 * above macro 'FIM_GPIO_DONT_USE'
 *
 * fim_nr  : Number of the FIM to use for the device
 * gpio_nr : GPIO to use for the interface line
 */
struct fim_usb_platform_data {

	int fim_nr;

	int (*init)(struct device *);
	int (*exit)(struct device *);

	int vp_gpio_nr;
	unsigned int vp_gpio_func;
	int vm_gpio_nr;
	unsigned int vm_gpio_func;
	int rcv_gpio_nr;
	unsigned int rcv_gpio_func;
	int oe_l_gpio_nr;
	unsigned int oe_l_gpio_func;
	int enum_gpio_nr;
	unsigned int enum_gpio_func;
	int spnd_gpio_nr;
	unsigned int spnd_gpio_func;
};

/*
 * Macro for the configuration of the GPIOs for the FIM USB driver
 * IMPORTANT: The FIM-firmware is able to control the DP, DM, OE and RCV pins, but NOT
 * the lines for the enumeration (ENUM) and suspend (SPND). The 'func_out' defines the
 * function for the output GPIOs
 */
#define NS921X_FIM_USB_GPIOS(vp, vm, rcv, oe_l, enume, spnd, func, func_out)      \
		.vp_gpio_nr = vp, \
		.vp_gpio_func = func, \
		.vm_gpio_nr = vm, \
		.vm_gpio_func = func, \
		.rcv_gpio_nr = rcv, \
		.rcv_gpio_func = func, \
		.oe_l_gpio_nr = oe_l, \
		.oe_l_gpio_func = func, \
		.enum_gpio_nr = enume, \
		.enum_gpio_func = func_out, \
		.spnd_gpio_nr = spnd, \
		.spnd_gpio_func = func_out

/* Macros for building the FIM-drivers as loadable modules */
#if defined(MODULE)
# define NS921X_FIM_NUMBERS_PARAM(number)		\
	static int number = -1;				\
	module_param_named(fims, number, int, 0644);
#else
# define NS921X_FIM_NUMBERS_PARAM(number)		\
	static int number = FIM_NR_PICS;
#endif

/* Call the function for checking the FIM module parameter */
#if defined(MODULE)
inline int fim_check_numbers_param(int number)	    \
{						    \
	if (number < 0 || number > FIM_NR_PICS)	    \
		return -1;			    \
	else					    \
		return 0;			    \
}
inline int fim_check_device_id(int number, uint id) {    \
	int ret;					  \
	if (id < 0)				       \
		ret = 1;			       \
	else if (number == FIM_NR_PICS && id < number) \
		ret = 0;			       \
	else if (number < FIM_NR_PICS && id == number) \
		ret = 0;			       \
	else					       \
		ret = 1;			       \
	return ret;				       \
}
#else
# define fim_check_numbers_param(number)        (0)
# define fim_check_device_id(number, id)        (id < 0 || id >= number)
#endif

/* These are the functions of the FIM-API */
int fim_register_driver(struct fim_driver *driver);
int fim_unregister_driver(struct fim_driver *driver);
int fim_send_interrupt2(struct fim_driver *driver, unsigned int code);
int fim_get_exp_reg(struct fim_driver *driver, int nr, unsigned int *value);
int fim_enable_irq(struct fim_driver *driver);
int fim_disable_irq(struct fim_driver *driver);
int fim_send_buffer(struct fim_driver *driver, const struct fim_buffer_t *bufdesc);
int fim_tx_buffers_room(struct fim_driver *driver);
int fim_tx_buffers_level(struct fim_driver *driver);
int fim_send_reset(struct fim_driver *driver);
int fim_send_start(struct fim_driver *driver);
int fim_send_stop(struct fim_driver *driver);
void fim_flush_rx(struct fim_driver *driver);
void fim_flush_tx(struct fim_driver *driver);
struct fim_buffer_t *fim_alloc_buffer(struct fim_driver *driver, int length,
				      unsigned int gfp_flags);
void fim_free_buffer(struct fim_driver *driver, struct fim_buffer_t *buffer);
void fim_set_ctrl_reg(struct fim_driver *driver, int reg, unsigned int val);
void fim_set_exp_reg(struct fim_driver *driver, int reg, unsigned int val);
int fim_get_ctrl_reg(struct fim_driver *driver, int reg, unsigned int *val);
int fim_get_stat_reg(struct fim_driver *driver, int reg, unsigned int *val);
struct pic_t *fim_request_pic(int picnr);
void fim_free_pic(struct pic_t *pic);
void fim_print_fifo_status(struct fim_driver *driver);
int fim_number_pics(void);
int fim_download_firmware(struct fim_driver *driver);
int fim_is_running(struct fim_driver *driver);

int fim_dma_stop(struct fim_driver *fim);
int fim_dma_start(struct fim_driver *fim, struct fim_dma_cfg_t *cfg);
int fim_dump_registers(struct fim_driver *driver);

#endif /* ifndef _NS921X_FIM_CORE_H */




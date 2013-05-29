/*
 *  linux/drivers/mmc/s3c-hsmmc.h - Samsung S3C SDI Interface driver
 *
 * $Id: s3c-hsmmc.h,v 1.9 2007/06/07 07:14:57 scsuh Exp $
 *
 *  Copyright (C) 2004 Thomas Kleffel, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef MHZ
#define MHZ (1000*1000)
#endif

#define s3c_hsmmc_readl(x)	readl((host->base)+(x))
#define s3c_hsmmc_readw(x)	readw((host->base)+(x))
#define s3c_hsmmc_readb(x)	readb((host->base)+(x))

#define s3c_hsmmc_writel(v,x)	writel((v),(host->base)+(x))
#define s3c_hsmmc_writew(v,x)	writew((v),(host->base)+(x))
#define s3c_hsmmc_writeb(v,x)	writeb((v),(host->base)+(x))

#define S3C_HSMMC_CLOCK_ON	1
#define S3C_HSMMC_CLOCK_OFF	0

#define	SPEED_NORMAL	0
#define	SPEED_HIGH	1

/*
 * Controller registers
 */

//S3C_HSMMC_BLOCK_SIZE:		0x04
#define S3C_HSMMC_MAKE_BLKSZ(dma, blksz) (((dma & 0x7) << 12) | (blksz & 0xFFF))

//S3C_HSMMC_TRANSFER_MODE:	0x0C
#define S3C_HSMMC_TRNS_DMA		0x01
#define S3C_HSMMC_TRNS_BLK_CNT_EN	0x02
#define S3C_HSMMC_TRNS_ACMD12		0x04
#define S3C_HSMMC_TRNS_READ		0x10
#define S3C_HSMMC_TRNS_MULTI		0x20

//S3C_HSMMC_COMMAND		0x0E
#define S3C_HSMMC_CMD_RESP_MASK		0x03
#define S3C_HSMMC_CMD_CRC		0x08
#define S3C_HSMMC_CMD_INDEX		0x10
#define S3C_HSMMC_CMD_DATA		0x20

#define S3C_HSMMC_CMD_RESP_NONE		0x00
#define S3C_HSMMC_CMD_RESP_LONG		0x01
#define S3C_HSMMC_CMD_RESP_SHORT	0x02
#define S3C_HSMMC_CMD_RESP_SHORT_BUSY	0x03

#define S3C_HSMMC_MAKE_CMD(c, f) (((c & 0xff) << 8) | (f & 0xff))

//S3C_HSMMC_PRESENT_STATE	0x24
#define S3C_HSMMC_CMD_INHIBIT		0x00000001
#define S3C_HSMMC_DATA_INHIBIT		0x00000002
#define S3C_HSMMC_DOING_WRITE		0x00000100
#define S3C_HSMMC_DOING_READ		0x00000200
#define S3C_HSMMC_SPACE_AVAILABLE	0x00000400
#define S3C_HSMMC_DATA_AVAILABLE	0x00000800
#define S3C_HSMMC_CARD_PRESENT		0x00010000
#define S3C_HSMMC_WRITE_PROTECT		0x00080000

//S3C_HSMMC_HOST_CONTROL	0x28
#define S3C_HSMMC_CTRL_LED		(0x01)
#define S3C_HSMMC_CTRL_4BITBUS		(0x02)
#define S3C_HSMMC_CTRL_HIGHSPEED	(0x04)
#define S3C_HSMMC_CTRL_4BIT		(0x02)
#define S3C_HSMMC_CTRL_8BIT		(0x20)

//S3C_HSMMC_POWER_CONTROL	0x29
#define S3C_HSMMC_POWER_OFF		0x00
#define S3C_HSMMC_POWER_ON		0x01
#define S3C_HSMMC_POWER_180		0x0A
#define S3C_HSMMC_POWER_300		0x0C
#define S3C_HSMMC_POWER_330		0x0E
#define S3C_HSMMC_POWER_ON_ALL		0xFF

//S3C_HSMMC_CLOCK_CONTROL	0x2C
#define S3C_HSMMC_DIVIDER_SHIFT		8
#define S3C_HSMMC_CLOCK_EXT_STABLE	0x0008
#define S3C_HSMMC_CLOCK_CARD_EN		0x0004
#define S3C_HSMMC_CLOCK_INT_STABLE	0x0002
#define S3C_HSMMC_CLOCK_INT_EN		0x0001

//S3C_HSMMC_TIMEOUT_CONTROL	0x2E
#define S3C_HSMMC_TIMEOUT_MAX		0x0E

//S3C_HSMMC_SOFTWARE_RESET	0x2F
#define S3C_HSMMC_RESET_ALL		0x01
#define S3C_HSMMC_RESET_CMD		0x02
#define S3C_HSMMC_RESET_DATA		0x04

//S3C_HSMMC_INT_STATUS	0x30
#define S3C_HSMMC_NIS_ERR		0x00008000
#define S3C_HSMMC_NIS_CMDCMP		0x00000001
#define S3C_HSMMC_NIS_TRSCMP		0x00000002
#define S3C_HSMMC_NIS_DMA		0x00000008

//S3C_HSMMC_EIS_STATUS	0x32
#define S3C_HSMMC_EIS_CMDTIMEOUT	0x00000001
#define S3C_HSMMC_EIS_CMDERR		0x0000000E
#define S3C_HSMMC_EIS_DATATIMEOUT	0x00000010
#define S3C_HSMMC_EIS_DATAERR		0x00000060
#define S3C_HSMMC_EIS_CMD12ERR		0x00000100


//S3C_HSMMC_SIGNAL_ENABLE	0x38
#define S3C_HSMMC_INT_MASK_ALL		0x0000
#define S3C_HSMMC_INT_RESPONSE		0x00000001
#define S3C_HSMMC_INT_DATA_END		0x00000002
#define S3C_HSMMC_INT_DMA_END		0x00000008
#define S3C_HSMMC_INT_SPACE_AVAIL	0x00000010
#define S3C_HSMMC_INT_DATA_AVAIL	0x00000020
#define S3C_HSMMC_INT_CARD_INSERT	0x00000040
#define S3C_HSMMC_INT_CARD_REMOVE	0x00000080
#define S3C_HSMMC_INT_CARD_CHANGE	0x000000c0 /* oring of above two */
#define S3C_HSMMC_INT_CARD_INT		0x00000100
#define S3C_HSMMC_INT_TIMEOUT		0x00010000
#define S3C_HSMMC_INT_CRC		0x00020000
#define S3C_HSMMC_INT_END_BIT		0x00040000
#define S3C_HSMMC_INT_INDEX		0x00080000
#define S3C_HSMMC_INT_DATA_TIMEOUT	0x00100000
#define S3C_HSMMC_INT_DATA_CRC		0x00200000
#define S3C_HSMMC_INT_DATA_END_BIT	0x00400000
#define S3C_HSMMC_INT_BUS_POWER		0x00800000
#define S3C_HSMMC_INT_ACMD12ERR		0x01000000

#define S3C_HSMMC_INT_NORMAL_MASK	0x00007FFF
#define S3C_HSMMC_INT_ERROR_MASK	0xFFFF8000

#define S3C_HSMMC_INT_CMD_MASK	(S3C_HSMMC_INT_RESPONSE | \
				 S3C_HSMMC_INT_TIMEOUT | \
				 S3C_HSMMC_INT_CRC | \
				 S3C_HSMMC_INT_END_BIT | \
				 S3C_HSMMC_INT_INDEX | \
				 S3C_HSMMC_NIS_ERR \
				)
#define S3C_HSMMC_INT_DATA_MASK	(S3C_HSMMC_INT_DATA_END | \
				 S3C_HSMMC_INT_DMA_END | \
				 S3C_HSMMC_INT_DATA_AVAIL | \
				 S3C_HSMMC_INT_SPACE_AVAIL | \
				 S3C_HSMMC_INT_DATA_TIMEOUT | \
				 S3C_HSMMC_INT_DATA_CRC | \
				 S3C_HSMMC_INT_DATA_END_BIT \
				)

//S3C_HSMMC_CAPABILITIES	0x40
#define S3C_HSMMC_TIMEOUT_CLK_MASK	0x0000003F
#define S3C_HSMMC_TIMEOUT_CLK_SHIFT	0
#define S3C_HSMMC_TIMEOUT_CLK_UNIT	0x00000080
#define S3C_HSMMC_CLOCK_BASE_MASK	0x00003F00
#define S3C_HSMMC_CLOCK_BASE_SHIFT	8
#define S3C_HSMMC_MAX_BLOCK_MASK	0x00030000
#define S3C_HSMMC_MAX_BLOCK_SHIFT	16
#define S3C_HSMMC_CAN_DO_DMA		0x00400000
#define S3C_HSMMC_CAN_VDD_330		0x01000000
#define S3C_HSMMC_CAN_VDD_300		0x02000000
#define S3C_HSMMC_CAN_VDD_180		0x04000000

//S3C_HSMMC_HOST_VERSION	0xFE
#define S3C_HSMMC_VENDOR_VER_MASK	0xFF00
#define S3C_HSMMC_VENDOR_VER_SHIFT	8
#define S3C_HSMMC_SPEC_VER_MASK		0x00FF
#define S3C_HSMMC_SPEC_VER_SHIFT	0

#ifndef CONFIG_S3C_HSMMC_MAX_HW_SEGS
#define CONFIG_S3C_HSMMC_MAX_HW_SEGS	32
#endif

struct s3c_hsmmc_dma_blk {
	dma_addr_t	dma_address;	/* dma address			*/
	uint		length;		/* length			*/
	uint		boundary;	/* Host DMA Buffer Boundary	*/
	void		*original;
};

struct s3c_hsmmc_host {
	void __iomem		*base;
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data		*data;
	struct mmc_host		*mmc;
	struct clk		*clk[3];
	struct resource		*mem;

	struct timer_list	timer;

	struct s3c_hsmmc_cfg	*plat_data;

	int			irq;
	spinlock_t		lock;

	struct tasklet_struct	card_tasklet;	/* Tasklet structures	*/
	struct tasklet_struct	finish_tasklet;

	unsigned int		clock;		/* Current clock (MHz)	*/
	unsigned int		ctrl3[2];	/* 0: normal, 1: high	*/

#define S3C_HSMMC_USE_DMA	(1<<0)
	int			flags;		/* Host attributes	*/

	uint			dma_dir;
#ifdef CONFIG_HSMMC_PSEUDO_SCATTERGATHER
	uint			sg_len;		/* size of scatter list	*/
	uint			dma_blk;	/* total dmablk number	*/
	uint			next_blk;	/* next block to send	*/
	struct s3c_hsmmc_dma_blk dblk[CONFIG_S3C_HSMMC_MAX_HW_SEGS*4];

	/* when pseudo algo cannot deal with sglist */
#define S3C_HSMMC_MALLOC_SIZE	PAGE_SIZE
#define S3C_HSMMC_MAX_SUB_BUF	CONFIG_S3C_HSMMC_MAX_HW_SEGS
	void			*sub_block[S3C_HSMMC_MAX_SUB_BUF];
#endif

#ifdef CONFIG_HSMMC_PROC_DATA
	unsigned int		rx_pkt[7];	/* 0 ~ 0x1000		*/
						/* 0x1000 ~ 0x2000	*/
						/* 0x2000 ~ 0x3000	*/
						/* 0x3000 ~ 0x4000	*/
						/* 0x4000 ~ 0x8000	*/
						/* 0x8000 ~ 0x10000	*/
						/* 0x10000 ~		*/
	unsigned int		tx_pkt[7];	/* 0 ~ 0x1000		*/
						/* 0x1000 ~ 0x2000	*/
						/* 0x2000 ~ 0x3000	*/
						/* 0x3000 ~ 0x4000	*/
						/* 0x4000 ~ 0x8000	*/
						/* 0x8000 ~ 0x10000	*/
						/* 0x10000 ~		*/
#endif
};


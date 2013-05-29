/*
 * arch/arm/mach-ns9xxx/include/mach/dma-ns921x.h
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */


#ifndef __ASM_ARCH_NS921X_DMA_H
#define __ASM_ARCH_NS921X_DMA_H

#include <mach/hardware.h>

/* External DMA control nad status registers. Valid channels are 1 and 2 */
#define NS921X_DMA_BDP(x)	__REG(0xa0800000 + ((x-1) & 0x1) * 0x10)
#define NS921X_DMA_CR(x)	__REG(0xa0800004 + ((x-1) & 0x1) * 0x10)
#define NS921X_DMA_STIE(x)	__REG(0xa0800008 + ((x-1) & 0x1) * 0x10)
#define NS921X_DMA_PCS(x)	__REG(0xa080000c + ((x-1) & 0x1) * 0x10)

/* Control register masks */
#define NS921X_DMA_CR_CE	(0x1 << 31)
#define NS921X_DMA_CR_CA	(0x1 << 30)
#define NS921X_DMA_CR_CG	(0x1 << 29)
#define NS921X_DMA_CR_SW_MA	(0x3 << 27)
#define NS921X_DMA_CR_SW_8b	(0x0 << 27)
#define NS921X_DMA_CR_SW_16b	(0x1 << 27)
#define NS921X_DMA_CR_SW_32b	(0x2 << 27)
#define NS921X_DMA_CR_DW_MA	(0x3 << 25)
#define NS921X_DMA_CR_DW_8b	(0x0 << 25)
#define NS921X_DMA_CR_DW_16b	(0x1 << 25)
#define NS921X_DMA_CR_DW_32b	(0x2 << 25)
#define NS921X_DMA_CR_SB_MA	(0x3 << 23)
#define NS921X_DMA_CR_SB_1B	(0x0 << 23)
#define NS921X_DMA_CR_SB_4B	(0x1 << 23)
#define NS921X_DMA_CR_SB_16B	(0x2 << 23)
#define NS921X_DMA_CR_SB_32B	(0x3 << 23)
#define NS921X_DMA_CR_DB_MA	(0x3 << 21)
#define NS921X_DMA_CR_DB_1B	(0x0 << 21)
#define NS921X_DMA_CR_DB_4B	(0x1 << 21)
#define NS921X_DMA_CR_DB_16B	(0x2 << 21)
#define NS921X_DMA_CR_DB_32B	(0x3 << 21)
#define NS921X_DMA_CR_SINC_N	(0x1 << 20)
#define NS921X_DMA_CR_DINC_N	(0x1 << 19)
#define NS921X_DMA_CR_POL	(0x1 << 18)
#define NS921X_DMA_CR_MODE	(0x1 << 17)
#define NS921X_DMA_CR_MODE_FBW	(0x0 << 17)
#define NS921X_DMA_CR_MODE_FBR	(0x1 << 17)
#define NS921X_DMA_CR_RESET	(0x1 << 16)
#define NS921X_DMA_CR_STATE_MA	(0x3f << 10)
#define NS921X_DMA_CR_INDEX_MA	(0x3ff << 0)

/* Status and interrupt enable masks */
#define NS921X_DMA_STIE_NCIP	(0x1 << 31)
#define NS921X_DMA_STIE_ECIP	(0x1 << 30)
#define NS921X_DMA_STIE_NRIP	(0x1 << 29)
#define NS921X_DMA_STIE_CAIP	(0x1 << 28)
#define NS921X_DMA_STIE_PCIP	(0x1 << 27)
#define NS921X_DMA_STIE_NCIE	(0x1 << 24)
#define NS921X_DMA_STIE_ECIE	(0x1 << 23)
#define NS921X_DMA_STIE_NRIE	(0x1 << 22)
#define NS921X_DMA_STIE_CAIE	(0x1 << 21)
#define NS921X_DMA_STIE_PCIE	(0x1 << 20)
#define NS921X_DMA_STIE_IE_ALL	(0x1f << 20)
#define NS921X_DMA_STIE_WRAP	(0x1 << 19)
#define NS921X_DMA_STIE_DONE	(0x1 << 18)
#define NS921X_DMA_STIE_LAST	(0x1 << 17)
#define NS921X_DMA_STIE_FULL	(0x1 << 16)
#define NS921X_DMA_STIE_BLEN_MA	(0xffff << 0)

/* Peripheral CS register */
#define NS921X_DMA_PCS_SEL_MA	(0x3 << 0)
#define NS921X_DMA_PCS_CS0	(0x0 << 0)
#define NS921X_DMA_PCS_CS1	(0x1 << 0)
#define NS921X_DMA_PCS_CS2	(0x2 << 0)
#define NS921X_DMA_PCS_CS3	(0x3 << 0)

#define EXT_DMA_DESC_CTRL_WRAP	(0x1 << 15)
#define EXT_DMA_DESC_CTRL_INT	(0x1 << 14)
#define EXT_DMA_DESC_CTRL_LAST	(0x1 << 13)
#define EXT_DMA_DESC_CTRL_FULL	(0x1 << 12)
#define EXT_DMA_DESC_CTRL_ALL	(EXT_DMA_DESC_CTRL_FULL | \
				 EXT_DMA_DESC_CTRL_INT | \
				 EXT_DMA_DESC_CTRL_LAST | \
				 EXT_DMA_DESC_CTRL_WRAP)

struct ext_dma_desc_t {
	unsigned int src;
	unsigned int length;
	unsigned int dest;
	unsigned short status;
	unsigned short control;
}__attribute__((__packed__));


/* 
 * IO-HUB constans and macros
 * The maximal number of DMA-buffer descriptors comes from the NET+OS 
 * distribution (iop_private.h) 
 */
#define IOHUB_MAX_DMA_BUFFERS	(64)
#define IOHUB_MAX_DMA_LENGTH	(65535)

#define IOHUB_DMA_DESC_CTRL_WRAP	EXT_DMA_DESC_CTRL_WRAP
#define IOHUB_DMA_DESC_CTRL_INT		EXT_DMA_DESC_CTRL_INT
#define IOHUB_DMA_DESC_CTRL_LAST	EXT_DMA_DESC_CTRL_LAST
#define IOHUB_DMA_DESC_CTRL_FULL	EXT_DMA_DESC_CTRL_FULL
#define IOHUB_DMA_DESC_CTRL_ALL		EXT_DMA_DESC_CTRL_ALL 

struct iohub_dma_desc_t {
	unsigned int src;
	unsigned int length;
	unsigned int reserved;
	unsigned short status;
	unsigned short control;
}__attribute__((packed, aligned));


#define IOHUB_DMA_DESC_LENGTH                   sizeof(struct iohub_dma_desc_t)

/* This is the FIFO used for the DMA-transfers of the IOHUB (e.g. FIMs) */
struct iohub_dma_fifo_t {
	int length;
	struct iohub_dma_desc_t **descs;
	dma_addr_t phys_descs;
	struct iohub_dma_desc_t *first;
	struct iohub_dma_desc_t *last;
	struct iohub_dma_desc_t *dma_first;
	struct iohub_dma_desc_t *dma_last;
	struct iohub_dma_desc_t *dma_next;
	struct iohub_dma_desc_t *next_free;
	unsigned long rx_error, tx_error;
	unsigned long rx_error1, tx_error2;
}__attribute__((__packed__));

#endif /* ifndef __ASM_ARCH_NS912X_DMA_H */

/* linux/arch/arm/mach-s3c2410/include/mach/regs-cfata.h
 *
 * Copyright (c) 2009 Digi Internationa Inc.
 * http://www.digi.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/


#ifndef ___ASM_ARCH_REGS_CFATA_H
#define ___ASM_ARCH_REGS_CFATA_H

/* PCMCIA / ATA controller registers */
#define S3C2443_ATTR_BASE		(0x0000)
#define S3C2443_IO_BASE			(0x0800)
#define S3C2443_COMM_BASE		(0x1000)
#define S3C2443_SFR_BASE		(0x1800)
#define S3C2443_MUX_REG			(0x1800)
#define S3C2443_PCCARD_BASE		(0x1820)
#define S3C2443_PCCARD_CFG		(0x1820)
#define S3C2443_PCCARD_INT		(0x1824)
#define S3C2443_PCCARD_ATTR		(0x1828)
#define S3C2443_PCCARD_IO		(0x182C)
#define S3C2443_PCCARD_COMM		(0x1830)

#define S3C2443_ATA_BASE		(0x1900)
#define S3C2443_ATA_CONTROL		(0x1900)
#define S3C2443_ATA_STATUS		(0x1904)
#define S3C2443_ATA_COMMAND		(0x1908)
#define S3C2443_ATA_SWRST		(0x190c)
#define S3C2443_ATA_IRQ			(0x1910)
#define S3C2443_ATA_IRQ_MASK		(0x1914)
#define S3C2443_ATA_CFG			(0x1918)

#define S3C2443_ATA_PIO_TIME		(0x192c)
#define S3C2443_ATA_UDMA_TIME		(0x1930)

#define S3C2443_ATA_PIO_DTR		(0x1954)
#define S3C2443_ATA_PIO_FED		(0x1958)
#define S3C2443_ATA_PIO_SCR		(0x195c)
#define S3C2443_ATA_PIO_LLR		(0x1960)
#define S3C2443_ATA_PIO_LMR		(0x1964)
#define S3C2443_ATA_PIO_LHR		(0x1968)
#define S3C2443_ATA_PIO_DVR		(0x196c)
#define S3C2443_ATA_PIO_CSD		(0x1970)
#define S3C2443_ATA_PIO_DAD		(0x1974)
#define S3C2443_ATA_PIO_RDATA		(0x197c)
#define S3C2443_BUS_FIFO_STATUS		(0x1990)
#define S3C2443_ATA_FIFO_STATUS		(0x1994)


/* PCCARD / ATA register masks */

/* MUX_REG masks */
#define S3C2443_MUX_OUTPUT_DISABLE	(1 << 2)
#define S3C2443_MUX_OUTPUT_ENABLE	(0 << 2)
#define S3C2443_MUX_PWREN_PWOFF		(1 << 1)
#define S3C2443_MUX_PWREN_PWON		(0 << 1)
#define S3C2443_MUX_MODE_PCCARD		(0 << 0)
#define S3C2443_MUX_MODE_IDE		(1 << 0)

/* PCCARD_CFG */
#define S3C2443_PCC_CARD_RESET		(1 << 13)
#define S3C2443_PCC_INT_SEL		(1 << 12)
#define S3C2443_PCC_nWAIT_EN		(1 << 11)
#define S3C2443_PCC_DEVICE_ATT_8	(0 << 10)
#define S3C2443_PCC_DEVICE_ATT_16	(1 << 10)
#define S3C2443_PCC_DEVICE_COMM_8	(0 << 9)
#define S3C2443_PCC_DEVICE_COMM_16	(1 << 9)
#define S3C2443_PCC_DEVICE_IO_8		(0 << 8)
#define S3C2443_PCC_DEVICE_IO_16	(1 << 8)
#define S3C2443_PCC_NOCARD_ERR		(1 << 3)
#define S3C2443_PCC_nWAIT		(1 << 2)
#define S3C2443_PCC_nIREQ		(1 << 1)
#define S3C2443_PCC_nCD			(1 << 0)

/* PCCARD_INT */
#define S3C2443_PCC_INTMSK_ERR_N	(1 << 10)
#define S3C2443_PCC_INTMSK_IREQ		(1 << 9)
#define S3C2443_PCC_INTMSK_CD		(1 << 8)
#define S3C2443_PCC_INTMSK_ALL		(S3C2443_PCC_INTMSK_ERR_N | \
					 S3C2443_PCC_INTMSK_IREQ | \
					 S3C2443_PCC_INTMSK_CD)
#define S3C2443_PCC_INTSRC_ERR_N	(1 << 2)
#define S3C2443_PCC_INTSRC_IREQ		(1 << 1)
#define S3C2443_PCC_INTSRC_CD		(1 << 0)
#define S3C2443_PCC_INTSRC_ALL		(S3C2443_PCC_INTSRC_ERR_N | \
					 S3C2443_PCC_INTSRC_IREQ | \
					 S3C2443_PCC_INTSRC_CD)

/* PCCARD_ATTR */
#define S3C2443_PCC_HOLD_ATTR		(0x7f << 16)
#define S3C2443_PCC_CMND_ATTR		(0x7f << 8)
#define S3C2443_PCC_SETUP_ATTR		(0x7f << 0)

/* PCCARD_IO */
#define S3C2443_PCC_HOLD_IO		(0x7f << 16)
#define S3C2443_PCC_CMND_IO		(0x7f << 8)
#define S3C2443_PCC_SETUP_IO		(0x7f << 0)

/* PCCARD_COMM */
#define S3C2443_PCC_HOLD_COMM		(0x7f << 16)
#define S3C2443_PCC_CMND_COMM		(0x7f << 8)
#define S3C2443_PCC_SETUP_COMM		(0x7f << 0)

#endif /* ___ASM_ARCH_REGS_CFATA_H */

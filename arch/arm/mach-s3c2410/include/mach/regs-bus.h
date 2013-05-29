/* linux/include/asm-arm/arch-s3c2410/regs-bus.h
 *
 * Copyright (c) 2009 Digi International
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * S3C2443 bus matrix and EBI registers
 */

#ifndef __ASM_ARM_REGS_BUS
#define __ASM_ARM_REGS_BUS

#define S3C2443_BUSREG(x) ((x) + S3C2443_VA_EBI)

#define S3C2443_BPRIORITY0		S3C2443_BUSREG(0x00)
#define S3C2443_BPRIORITY1		S3C2443_BUSREG(0x04)

#define S3C2443_EBICON			S3C2443_BUSREG(0x08)

#define S3C2443_EBICON_FIXPRTYPE	(3<<0)
#define S3C2443_EBICON_PR_TYPE		(1<<2)
#define S3C2443_EBICON_BANK1_CFG	(1<<8)
#define S3C2443_EBICON_BANK2_CFG	(1<<9)
#define S3C2443_EBICON_BANK3_CFG	(1<<10)


#endif /* __ASM_ARM_REGS_BUS */

/*
 * arch/arm/mach-ns9xxx/include/mach/regs-bbu.h
 *
 * Copyright (C) 2006 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef __ASM_ARCH_REGS_BBU_H
#define __ASM_ARCH_REGS_BBU_H

#include <mach/hardware.h>

/* BBus Utility */

#define NS9360_BBU_MSR		__REG(0x90600000)

/* GPIO Configuration Registers block 1 */
/* NOTE: the HRM starts counting at 1 for the GPIO registers, here the start is
 * at 0 for each block.  That is, NS9360_BBU_GCONFb1(0) is GPIO Configuration
 * Register #1, NS9360_BBU_GCONFb2(0) is GPIO Configuration Register #8. */
#define NS9360_BBU_GCONFb1(x)	__REG2(0x90600010, (x))
#define NS9360_BBU_GCONFb2(x)	__REG2(0x90600100, (x))

#define __NS9360_BBU_GCONFx_SHIFT(m)		(((m) & 7) << 2)

#define NS9360_BBU_GCONFx_DIR(m)	__REGBIT_SHIFT(3, __NS9360_BBU_GCONFx_SHIFT(m))
#define NS9360_BBU_GCONFx_DIR_INPUT(m)	__REGVAL(NS9360_BBU_GCONFx_DIR(m), 0)
#define NS9360_BBU_GCONFx_DIR_OUTPUT(m)	__REGVAL(NS9360_BBU_GCONFx_DIR(m), 1)
#define NS9360_BBU_GCONFx_INV(m)	__REGBIT_SHIFT(2, __NS9360_BBU_GCONFx_SHIFT(m))
#define NS9360_BBU_GCONFx_INV_NO(m)		__REGVAL(NS9360_BBU_GCONFx_INV(m), 0)
#define NS9360_BBU_GCONFx_INV_YES(m)		__REGVAL(NS9360_BBU_GCONFx_INV(m), 1)
#define NS9360_BBU_GCONFx_FUNC(m)	__REGBITS_SHIFT(1, 0, __NS9360_BBU_GCONFx_SHIFT(m))
#define NS9360_BBU_GCONFx_FUNC_0(m)		__REGVAL(NS9360_BBU_GCONFx_FUNC(m), 0)
#define NS9360_BBU_GCONFx_FUNC_1(m)		__REGVAL(NS9360_BBU_GCONFx_FUNC(m), 1)
#define NS9360_BBU_GCONFx_FUNC_2(m)		__REGVAL(NS9360_BBU_GCONFx_FUNC(m), 2)
#define NS9360_BBU_GCONFx_FUNC_3(m)		__REGVAL(NS9360_BBU_GCONFx_FUNC(m), 3)

#define NS9360_BBU_GCTRL1	__REG(0x90600030)
#define NS9360_BBU_GCTRL2	__REG(0x90600034)
#define NS9360_BBU_GCTRL3	__REG(0x90600120)

#define NS9360_BBU_GSTAT1	__REG(0x90600040)
#define NS9360_BBU_GSTAT2	__REG(0x90600044)
#define NS9360_BBU_GSTAT3	__REG(0x90600130)

#define NS9360_BBUS_USB		__REG(0x90600070)

#define	NS9360_BBUS_DMA_ISTAT	__REG(0x90600060)
#define NS9360_BBUS_DMA_IEN	__REG(0x90600064)

#define	NS9360_BBUS_ISTAT	__REG(0xa0401000)
#define NS9360_BBUS_IEN		__REG(0xa0401004)
#define NS9360_BBUS_IEN_GLBL		(1 << 31)

#endif /* ifndef __ASM_ARCH_REGS_BBU_H */

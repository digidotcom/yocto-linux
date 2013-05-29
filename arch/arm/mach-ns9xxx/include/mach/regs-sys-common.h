/*
 * arch/arm/mach-ns9xxx/include/mach/regs-sys-common.h
 *
 * Copyright (C) 2007,2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __ASM_ARCH_REGSSYSCOMMON_H
#define __ASM_ARCH_REGSSYSCOMMON_H
#include <mach/hardware.h>

/* Interrupt Vector Address Register Level x */
#define SYS_IVA(x)	__REG2(0xa09000c4, (x))

/* Interrupt Configuration registers */
#define SYS_IC(x)	__REG2(0xa0900144, (x))
#define __SYS_IC_FIELDNUM(i)		(3 - ((i) & 3))
#define __SYS_IC_SHIFT(i)		(((i) & 3) << 3)
#define SYS_IC_IE(i)		__REGBIT(7 + __SYS_IC_SHIFT(i))
#define SYS_IC_IE_EN(i)		__REGVAL(SYS_IC_IE(i), 1)
#define SYS_IC_IE_DIS(i)	__REGVAL(SYS_IC_IE(i), 0)
#define SYS_IC_ISD(i)		__REGBITS_SHIFT(4, 0, __SYS_IC_SHIFT(i))

/* ISRADDR */
#define SYS_ISRADDR     __REG(0xa0900164)

/* Interrupt Status Active */
#define SYS_ISA		__REG(0xa0900168)

/* Interrupt Status Raw */
#define SYS_ISR		__REG(0xa090016c)

/* Active Interrupt Level ID Status register */
#define SYS_AILID	__REG(0xa090018c)

/* System Memory Chip Select x Dynamic Memory Base */
#define SYS_SMCSDMB(x)	__REG2(0xa09001d0, (x) << 1)

/* System Memory Chip Select x Dynamic Memory Mask */
#define SYS_SMCSDMM(x)	__REG2(0xa09001d4, (x) << 1)

/* System Memory Chip Select x Static Memory Base */
#define SYS_SMCSSMB(x)	__REG2(0xa09001f0, (x) << 1)

/* System Memory Chip Select x Static Memory Base: Chip select x base */
#define SYS_SMCSSMB_CSxB	__REGBITS(31, 12)

/* System Memory Chip Select x Static Memory Mask */
#define SYS_SMCSSMM(x)	__REG2(0xa09001f4, (x) << 1)

/* System Memory Chip Select x Static Memory Mask: Chip select x mask */
#define SYS_SMCSSMM_CSxM	__REGBITS(31, 12)

/* System Memory Chip Select x Static Memory Mask: Chip select x enable */
#define SYS_SMCSSMM_CSEx	__REGBIT(0)
#define SYS_SMCSSMM_CSEx_DIS		__REGVAL(SYS_SMCSSMM_CSEx, 0)
#define SYS_SMCSSMM_CSEx_EN		__REGVAL(SYS_SMCSSMM_CSEx, 1)

/* General purpose, user-defined ID register */
#define SYS_GENID	__REG(0xa0900210)


#define SYS_EIxCTRL(i)	__REG2(0xa0900214, (i))
#define SYS_EIxCTRL_CLEAR	__REGBIT(2)
#define SYS_EIxCTRL_PLTY	__REGBIT(1)
#define SYS_EIxCTRL_PLTY_HIGH		__REGVAL(SYS_EIxCTRL_PLTY, 0)
#define SYS_EIxCTRL_PLTY_LOW		__REGVAL(SYS_EIxCTRL_PLTY, 1)
#define SYS_EIxCTRL_TYPE	__REGBIT(0)
#define SYS_EIxCTRL_TYPE_LEVEL		__REGVAL(SYS_EIxCTRL_TYPE, 0)
#define SYS_EIxCTRL_TYPE_EDGE		__REGVAL(SYS_EIxCTRL_TYPE, 1)

#endif /* ifndef __ASM_ARCH_REGSSYSCOMMON_H */

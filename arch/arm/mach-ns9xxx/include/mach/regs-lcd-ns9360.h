/*
 * arch/arm/mach-ns9xxx/include/mach/regs-lcd-ns9360.h
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __ASM_ARCH_REGSLCDNS9360_H
#define __ASM_ARCH_REGSLCDNS9360_H

#define LCD_TIMING0	0x00
#define LCD_TIMING0_HBP(x)	(((x) & 0xFF) << 24)
#define LCD_TIMING0_HFP(x)	(((x) & 0xFF) << 16)
#define LCD_TIMING0_HSW(x)	(((x) & 0xFF) <<  8)
#define LCD_TIMING0_PPL(x)	(((x) & 0x3F) <<  2)

#define LCD_TIMING1	0x04
#define LCD_TIMING1_VBP(x)	(((x) & 0xFF) << 24)
#define LCD_TIMING1_VFP(x)	(((x) & 0xFF) << 16)
#define LCD_TIMING1_VSW(x)	(((x) & 0x3F) << 10)
#define LCD_TIMING1_LPP(x)	(((x) & 0x3FF))

#define LCD_TIMING2	0x08
#define LCD_TIMING2_BCD	(1 << 26)
#define LCD_TIMING2_CPL(x)	(((x) & 0x3FF) << 16)
#define LCD_TIMING2_IOE	(1 << 14)
#define LCD_TIMING2_IPC	(1 << 13)
#define LCD_TIMING2_IHS	(1 << 12)
#define LCD_TIMING2_IVS	(1 << 11)
#define LCD_TIMING2_ACB(x)	(((x) & 0x1F) << 6)
#define LCD_TIMING2_PCD(x)	(((x) & 0x1F))

#define LCD_TIMING3	0x0c
#define LCD_TIMING3_LEE	(1 << 16)
#define LCD_TIMING3_LED(x)	(((x) & 0x7F))

#define LCD_UPBASE	0x10
#define LCD_UPBASE_V(x)		(((x) & 0x3FFFFFFF)<<2)

#define LCD_LPBASE	0x14
#define LCD_PBASE_V(x)		(((x) & 0x3FFFFFFF)<<2)

#define LCD_IRENABLE	0x18
#define LCD_IRENABLE_MBERR	(1 << 3)
#define LCD_IRENABLE_VCOMP	(1 << 2)
#define LCD_IRENABLE_LNBU	(1 << 1)

#define LCD_CONTROL	0x1c
#define LCD_CONTROL_WATERMARK	(1 << 16)
#define LCD_CONTROL_VCOMP_MA	(3 << 12)
#define LCD_CONTROL_VCOMP_VS	(0 << 12)
#define LCD_CONTROL_VCOMP_BP	(1 << 12)
#define LCD_CONTROL_VCOMP_AV	(2 << 12)
#define LCD_CONTROL_VCOMP_FP	(3 << 12)
#define LCD_CONTROL_PWR		(1 << 11)
#define LCD_CONTROL_BEPO	(1 << 10)
#define LCD_CONTROL_BEBO	(1 << 9)
#define LCD_CONTROL_BGR		(1 << 8)
#define LCD_CONTROL_DUAL	(1 << 7)
#define LCD_CONTROL_MONO8	(1 << 6)
#define LCD_CONTROL_TFT		(1 << 5)
#define LCD_CONTROL_BW		(1 << 4)
#define LCD_CONTROL_BPP_MA	(7 << 1)
#define LCD_CONTROL_BPP_1	(0 << 1)
#define LCD_CONTROL_BPP_2	(1 << 1)
#define LCD_CONTROL_BPP_4	(2 << 1)
#define LCD_CONTROL_BPP_8	(3 << 1)
#define LCD_CONTROL_BPP_16	(4 << 1)
#define LCD_CONTROL_EN		(1 << 0)

#define LCD_STATUS	0x20
#define LCD_STATUS_MBERROR	(4 << 1)
#define LCD_STATUS_VCOMP	(3 << 1)
#define LCD_STATUS_LNBU		(2 << 1)

#define LCD_INTR	0x24
#define LCD_INTR_MBERROR	(4 << 1)
#define LCD_INTR_VCOMP		(3 << 1)
#define LCD_INTR_LNBUINTR	(2 << 1)

#define LCD_UPCURRENT	0x28
#define LCD_LPCURRENT	0x2c

#endif

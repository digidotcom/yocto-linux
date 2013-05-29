/* -*- linux-c -*-
 *
 * arch/arm/mach-ns9xxx/include/mach/fim-uncompress.h
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 *  !Authors:     Hector Palacios, Luis Galdos
 *  !Desc:
 *  !References:
 */

#ifndef __FIM_UNCROMPRESS_H
#define __FIM_UNCROMPRESS_H

#include <asm/io.h>

#define NS921X_FIM0				__REG(0x90000000)
#define NS921X_FIM1				__REG(0x90008000)

#define FIM_SERIAL_DATA_BITS			(8)

#define NS92XX_FIM_CTRL0_REG                    (0x10)
#define NS92XX_FIM_EXP0_REG                     (0x50)
#define NS92XX_FIM_CTRL_REG(i)                  (NS92XX_FIM_CTRL0_REG + 4*i)
#define NS92XX_FIM_EXP_REG(i)                   (NS92XX_FIM_EXP0_REG + 4*i)

#define NS92XX_FIM_REG_BASE_PA                  (0x90001000)
#define NS92XX_FIM_REG_BASE_PA                  (0x90001000)
#define NS92XX_FIM_REG_OFFSET                   (0x8000)
#define FIM_REG_ADDR(x)				(NS92XX_FIM_REG_BASE_PA + \
							(x * NS92XX_FIM_REG_OFFSET)) 
#define NS921X_FIM_ENABLED(base)		(__raw_readl((base) + 0x1000) \
							& (1 << 31))
#define NS92XX_FIM_GEN_CTRL_REG                 (0x00)
#define NS92XX_FIM_GEN_CTRL_INTTOPIC            (0x00007f00)
#define NS92XX_FIM_GEN_CTRL_INTACKRD            (0x00000080)
#define NS92XX_FIM_INT_MASK(code)               (code<<8)

#define FIM_SERIAL_INT_INSERT_CHAR              (0x01)

static int fim_send_interrupt(int pic_num, unsigned int code)
{
	unsigned int stopcnt;
	u32 status;

	code = NS92XX_FIM_INT_MASK(code);
	status = readl(FIM_REG_ADDR(pic_num) + NS92XX_FIM_GEN_CTRL_REG);
	writel(status | code, FIM_REG_ADDR(pic_num) + NS92XX_FIM_GEN_CTRL_REG);

	/* This loop is perhaps problematic, exit with a timeout */
	stopcnt = 0xFFFF;
	do {
		status = readl(FIM_REG_ADDR(pic_num)  + NS92XX_FIM_GEN_CTRL_REG);
		stopcnt--;
	} while (!(status & NS92XX_FIM_GEN_CTRL_INTACKRD) && stopcnt);

	if (!stopcnt) {
		return 1;
	}

	/* Reset the interrupt bits for the PIC acknowledge */
	status &= ~NS92XX_FIM_GEN_CTRL_INTTOPIC;
	writel(status, FIM_REG_ADDR(pic_num) + NS92XX_FIM_GEN_CTRL_REG);

	stopcnt = 0xFFFF;
	do {
		status = readl(FIM_REG_ADDR(pic_num) + NS92XX_FIM_GEN_CTRL_REG);
		stopcnt--;
	} while ((status & NS92XX_FIM_GEN_CTRL_INTACKRD) && stopcnt);

	if (!stopcnt) {
		return 1;
	}

	return 0;
}

static void fim_set_ctrl_reg(int pic_num, int reg, unsigned int val)
{
	writel(val, FIM_REG_ADDR(pic_num) + NS92XX_FIM_CTRL_REG(reg));
}

static int fim_get_exp_reg(int pic_num, int nr)
{
	return readl(FIM_REG_ADDR(pic_num) + NS92XX_FIM_EXP_REG(nr));
}

#endif /* __FIM_UNCROMPRESS_H */

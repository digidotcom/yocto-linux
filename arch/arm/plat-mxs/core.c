/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <asm/proc-fns.h>

#include <mach/hardware.h>


/* CLKCTRL regs and bitfields */
#define HW_CLKCTRL_RESET	(0x000001e0)
#define BM_CLKCTRL_RESET_DIG	0x00000001
#define BM_CLKCTRL_RESET_CHIP	0x00000002

/* Common IP block flags */
#define MXS_MODULE_CLKGATE              (1 << 30)
#define MXS_MODULE_SFTRST               (1 << 31)

void arch_idle(void)
{
	cpu_do_idle();
}

void arch_reset(char mode, const char *cmd)
{
	void *base = IO_ADDRESS(CLKCTRL_PHYS_ADDR);

	__raw_writel(BM_CLKCTRL_RESET_CHIP, base + HW_CLKCTRL_RESET);

	cpu_reset(0);
}

/*
 * Clear the bit and poll it cleared.  This is usually called with
 * a reset address and mask being either SFTRST(bit 31) or CLKGATE
 * (bit 30).
 */
static int clear_poll_bit(void __iomem *addr, u32 mask)
{
	int timeout = 0x400;

	/* clear the bit */
	__mxs_clrl(mask, addr);

	/*
	 * SFTRST needs 3 GPMI clocks to settle, the reference manual
	 * recommends to wait 1us.
	 */
	udelay(1);

	/* poll the bit becoming clear */
	while ((__raw_readl(addr) & mask) && --timeout)
		/* nothing */;

	return !timeout;
}

static int __mxs_reset_block(void __iomem *reset_addr, int just_enable)
{
	int ret;
	int timeout = 0x400;

	/* clear and poll SFTRST */
	ret = clear_poll_bit(reset_addr, MXS_MODULE_SFTRST);
	if (unlikely(ret))
		goto error;

	/* clear CLKGATE */
	__mxs_clrl(MXS_MODULE_CLKGATE, reset_addr);

	if (!just_enable) {
		/* set SFTRST to reset the block */
		__mxs_setl(MXS_MODULE_SFTRST, reset_addr);
		udelay(1);

		/* poll CLKGATE becoming set */
		while ((!(__raw_readl(reset_addr) & MXS_MODULE_CLKGATE)) && --timeout)
			/* nothing */;
		if (unlikely(!timeout))
			goto error;

		/* clear and poll SFTRST */
		ret = clear_poll_bit(reset_addr, MXS_MODULE_SFTRST);
		if (unlikely(ret))
			goto error;
	}

	/* clear and poll CLKGATE */
	ret = clear_poll_bit(reset_addr, MXS_MODULE_CLKGATE);
	if (unlikely(ret))
		goto error;

	return 0;

error:
	pr_err("%s(%p): module reset timeout\n", __func__, reset_addr);
	return -ETIMEDOUT;
}

int mxs_reset_block(void __iomem *hwreg, int just_enable)
{
	int try = 10;
	int r = 0;

	while (try--) {
		r = __mxs_reset_block(hwreg, just_enable);
		if (!r)
			break;
		pr_debug("%s: try %d failed\n", __func__, 10 - try);
	}
	return r;
}
EXPORT_SYMBOL(mxs_reset_block);

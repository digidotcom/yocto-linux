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

#ifndef __ASM_PLAT_UNCOMPRESS_H
#define __ASM_PLAT_UNCOMPRESS_H

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/mx28.h>
#include "../../../mach-mx28/regs-pinctrl.h"
#include <linux/kernel.h>
#include <asm/io.h>

static unsigned long uart_base;

/*
 * Register includes are for when the MMU enabled; we need to define our
 * own stuff here for pre-MMU use
 */
#define UART(x)			(*(volatile unsigned long *)(uart_base + (x)))

#define DUART_DR		0x00
#define DUART_FR		0x18
#define DUART_FR_TXFE		(1 << 7)
#define DUART_FR_BUSY		(1 << 3)

#define AUART_CTRL2		0x20
#define AUART_DATA		0x60
#define AUART_STAT		0x70
#define AUART_STAT_TXFF		(1 << 25)
#define AUART_STAT_BUSY		(1 << 29)
#define AUART_CTRL2_UARTEN	(1 << 0)

#if 0
static void debug_led(int value)
{
	unsigned long addr;
	int bank = 2;
	int pin = 19;

	/* Configure pin as output */
	addr = PINCTRL_PHYS_ADDR + HW_PINCTRL_DOE0_SET + (0x10 * bank);
	__raw_writel(1 << pin, addr);

	/* Set output to value (inverse logic) */
	if (!value) {
		addr = PINCTRL_PHYS_ADDR + HW_PINCTRL_DOUT0_SET +(0x10 * bank);
		__raw_writel(1 << pin, addr);
	}
	else {
		addr = PINCTRL_PHYS_ADDR + HW_PINCTRL_DOUT0_CLR +(0x10 * bank);
		__raw_writel(1 << pin, addr);
	}
}
#endif

/*
 * This does not append a newline
 */
static void duart_putc(char c)
{
	/* Wait for TX fifo empty */
	while ((UART(DUART_FR) & DUART_FR_TXFE) == 0)
		continue;

	/* Write byte */
	UART(DUART_DR) = c;

	/* Wait for last bit to exit the UART */
	while (UART(DUART_FR) & DUART_FR_BUSY)
		continue;
}

static void auart_putc(char c)
{
	/* Wait for TX fifo empty */
	while ((UART(AUART_STAT) & AUART_STAT_TXFF))
		;

	/* Write byte */
	UART(AUART_DATA) = c;

	/* Wait for last bit to exit the UART */
	while (UART(AUART_STAT) & AUART_STAT_BUSY)
		;

}

static void putc(char c)
{
	if (DUART_PHYS_ADDR == uart_base)
		duart_putc(c);
	else
		auart_putc(c);
}

#define flush()	do { } while (0)

#if defined(CONFIG_MODULE_CCARDIMX28) || defined(CONFIG_MACH_WR21)
static unsigned long get_console_uart_baddr(unsigned long arch_id)
{
	if (MACH_TYPE_CCARDIMX28JS == arch_id) {
		unsigned long auarts[] = {
			AUART1_PHYS_ADDR,
			AUART2_PHYS_ADDR,
			AUART3_PHYS_ADDR,
			AUART4_PHYS_ADDR,
		};
		int i;

		for (i=0; i < ARRAY_SIZE(auarts); i++) {
			uart_base = auarts[i];
			if (UART(AUART_CTRL2) & AUART_CTRL2_UARTEN) {
				return uart_base;
			}
		}
		/* Use DUART by default if no AUART enabled */
		return DUART_PHYS_ADDR;
	}
	else if (MACH_TYPE_WR21 == arch_id) {
		/* Don't know how many of these make sense (are available on the module) */
		unsigned long auarts[] = {
			AUART1_PHYS_ADDR,
			AUART3_PHYS_ADDR,
		};
		int i;

		for (i=0; i < ARRAY_SIZE(auarts); i++) {
			uart_base = auarts[i];
			if (UART(AUART_CTRL2) & AUART_CTRL2_UARTEN)
				return uart_base;
		}
		/* Use AUART1 by default if no AUART enabled */
		return AUART1_PHYS_ADDR;
	}
	return 0;
}
#endif /* CONFIG_MODULE_CCARDIMX28 */

static __inline__ void __arch_decomp_setup(unsigned long arch_id)
{
	switch (arch_id) {
#if defined(CONFIG_MODULE_CCARDIMX28) || defined(CONFIG_MACH_WR21)
	case MACH_TYPE_CCARDIMX28JS:
	case MACH_TYPE_WR21:
		uart_base = get_console_uart_baddr(arch_id);
		break;
#endif /* CONFIG_MODULE_CCARDIMX28 || CONFIG_MACH_WR21 */
	default:
		uart_base = DUART_PHYS_ADDR;
		break;
	}
}

#define arch_decomp_setup()	__arch_decomp_setup(arch_id)

#define arch_decomp_wdog()

#endif /* __ASM_PLAT_UNCOMPRESS_H */

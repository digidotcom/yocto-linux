/*
 * arch/arm/mach-ns9xxx/include/mach/uncompress.h
 *
 * Copyright (C) 2006 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

#include <asm/io.h>

#define __REG(x)		((void __iomem __force *)(x))
static void putc_dummy(char c, void __iomem *base)
{
	/* nothing */
}

static int timeout;

#if defined(CONFIG_PROCESSOR_NS9360)
# define NS9360_UARTA	__REG(0x90200040)
# define NS9360_UARTB	__REG(0x90200000)
# define NS9360_UARTC	__REG(0x90300000)
# define NS9360_UARTD	__REG(0x90300040)
# define NS9360_UART_ENABLED(base)				\
		(__raw_readl(NS9360_UARTA) & (1 << 31))
static void putc_ns9360(char c, void __iomem *base)
{
	do {
		if (timeout)
			--timeout;

		if (__raw_readl(base + 8) & (1 << 3)) {
			__raw_writeb(c, base + 16);
			timeout = 0x10000;
			break;
		}
	} while (timeout);
}
#endif

#if defined(CONFIG_PROCESSOR_NS921X)
# define NS921XSYS_CLOCK	__REG(0xa090017c)
# define NS921X_UARTA	__REG(0x90010000)
# define NS921X_UARTB	__REG(0x90018000)
# define NS921X_UARTC	__REG(0x90020000)
# define NS921X_UARTD	__REG(0x90028000)
# define NS921X_UART_ENABLED(base)				\
		(__raw_readl((base) + 0x1000) & (1 << 29))

static void putc_ns921x(char c, void __iomem *base)
{
	do {
		if (timeout)
			--timeout;

		if (!(__raw_readl(base) & (1 << 11))) {
			__raw_writeb(c, base + 0x0028);
			timeout = 0x10000;
			break;
		}
	} while (timeout);
}

# if defined(CONFIG_SERIAL_FIM_CONSOLE)
#include "fim-uncompress.h"

static void putc_ns921x_fim(char ch, void __iomem *base)
{
	unsigned int status;
	unsigned short data = 1;
	int pic = 0;

	if ('\n' == ch)
		ch = '\r';

	data = (data << FIM_SERIAL_DATA_BITS) | (ch & ((1 << FIM_SERIAL_DATA_BITS) - 1));

	/* Check if the PIC is tasked with another send-char request */
        do {
                status = fim_get_exp_reg(pic, 0);
        } while (status & FIM_SERIAL_INT_INSERT_CHAR);

        /* And send the char using the interrupt function */
        fim_set_ctrl_reg(pic, 0, data & 0xFF);
        fim_set_ctrl_reg(pic, 1, (data >> 8) & 0xFF);
        fim_send_interrupt(pic, FIM_SERIAL_INT_INSERT_CHAR);
}
#endif /* CONFIG_SERIAL_FIM_CONSOLE */
#endif /* CONFIG_PROCESSOR_NS921X */

static void autodetect(void (**putc)(char, void __iomem *), void __iomem **base)
{
#if defined(CONFIG_PROCESSOR_NS9360)
	/* ns9360 */
	if (NS9360_UART_ENABLED(NS9360_UARTA)) {
		*putc = putc_ns9360;
		*base = NS9360_UARTA;
		return;
	} else if (NS9360_UART_ENABLED(NS9360_UARTB)) {
		*putc = putc_ns9360;
		*base = NS9360_UARTB;
		return;
	} else if (NS9360_UART_ENABLED(NS9360_UARTC)) {
		*putc = putc_ns9360;
		*base = NS9360_UARTC;
		return;
	} else if (NS9360_UART_ENABLED(NS9360_UARTD)) {
		*putc = putc_ns9360;
		*base = NS9360_UARTD;
		return;
	}
#elif defined(CONFIG_PROCESSOR_NS921X)
	/* ns921x */
	u32 clock = __raw_readl(NS921XSYS_CLOCK);

	if ((clock & (1 << 1)) &&
			NS921X_UART_ENABLED(NS921X_UARTA)) {
		*putc = putc_ns921x;
		*base = NS921X_UARTA;
		return;
	} else if ((clock & (1 << 2)) &&
			NS921X_UART_ENABLED(NS921X_UARTB)) {
		*putc = putc_ns921x;
		*base = NS921X_UARTB;
		return;
	} else if ((clock & (1 << 3)) &&
			NS921X_UART_ENABLED(NS921X_UARTC)) {
		*putc = putc_ns921x;
		*base = NS921X_UARTC;
		return;
	} else if ((clock & (1 << 4)) &&
			NS921X_UART_ENABLED(NS921X_UARTD)) {
		*putc = putc_ns921x;
		*base = NS921X_UARTD;
		return;
	}
# if defined(CONFIG_SERIAL_FIM_CONSOLE)
	/* None of the standard UARTs is enabled. Try with the FIMs */
#  if defined(CONFIG_FIM_ZERO_SERIAL_SELECTED) && !defined(CONFIG_FIM_ONE_SERIAL_SELECTED)
	/* Try with UART on FIM0 */
	else if (NS921X_FIM_ENABLED(NS921X_FIM0)) {
		*putc = putc_ns921x_fim;
		*base = NS921X_FIM0;
		return;
	}
#  elif !defined(CONFIG_FIM_ZERO_SERIAL_SELECTED) && defined(CONFIG_FIM_ONE_SERIAL_SELECTED)
	/* Try with UART on FIM1 */
	else if (NS921X_FIM_ENABLED(NS921X_FIM1)) {
		*putc = putc_ns921x_fim;
		*base = NS921X_FIM1;
		return;
	}
#  endif /* FIM_x */
# endif /* SERIAL_FIM_CONSOLE */
#endif /* PROCESSOR_x */
	*putc = putc_dummy;
}

void (*myputc)(char, void __iomem *);
void __iomem *base;

static void putc(char c)
{
 	  myputc(c, base);
}

static void arch_decomp_setup(void)
{
	autodetect(&myputc, &base);
}
#define arch_decomp_wdog()

static void flush(void)
{
	/* nothing */
}

#endif /* ifndef __ASM_ARCH_UNCOMPRESS_H */

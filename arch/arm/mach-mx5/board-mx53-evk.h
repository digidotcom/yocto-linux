/*
 * Copyright 2011 Digi International, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __ASM_ARCH_MXC_BOARD_CCIMX5X_H__
#define __ASM_ARCH_MXC_BOARD_CCIMX5X_H__

#include <mach/mxc_uart.h>

/* UART 1 configuration */
#if defined CONFIG_UART1_ENABLED
#define UART1_ENABLED		1
#else
#define UART1_ENABLED		0
#endif
#if defined CONFIG_UART1_IRDA_ENABLED
#define UART1_IR		IRDA
#else
#define UART1_IR		NO_IRDA
#endif
#define UART1_MODE		MODE_DCE
#define UART1_DMA_ENABLED	0


/* UART 2 configuration */
#if defined CONFIG_UART2_ENABLED
#define UART2_ENABLED		1
#else
#define UART2_ENABLED		0
#endif
#if defined CONFIG_UART2_IRDA_ENABLED
#define UART2_IR		IRDA
#else
#define UART2_IR		NO_IRDA
#endif
#define UART2_MODE		MODE_DCE
#define UART2_DMA_ENABLED	0

/* UART 3 configuration */
#if defined CONFIG_UART3_ENABLED
#define UART3_ENABLED		1
#else
#define UART3_ENABLED		0
#endif
#if defined CONFIG_UART3_IRDA_ENABLED
#define UART3_IR		IRDA
#else
#define UART3_IR		NO_IRDA
#endif
#define UART3_MODE		MODE_DCE
#define UART3_DMA_ENABLED	0

/*!
 * Specifies if the Irda transmit path is inverting
 */
#define MXC_IRDA_TX_INV	0

#define MXC_LL_UART_PADDR	UART1_BASE_ADDR
#define MXC_LL_UART_VADDR	AIPS1_IO_ADDRESS(UART1_BASE_ADDR)

#endif		/* __ASM_ARCH_MXC_BOARD_CCIMX5X_H__ */

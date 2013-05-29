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
#if defined(CONFIG_UART1_ENABLED)
#define UART1_ENABLED		1
#else
#define UART1_ENABLED		0
#endif
#if defined(CONFIG_UART1_MODE_IRDA)
#define UART1_IR		IRDA
#elif defined(CONFIG_UART1_MODE_RS485)
#define UART1_IR		RS485_HALF
#else
#define UART1_IR		NO_IRDA
#endif
#define UART1_MODE		MODE_DCE
#define UART1_DMA_ENABLED	0
#define UART1_RS485_TXDIR_LVL	0


/* UART 2 configuration */
#if defined(CONFIG_UART2_ENABLED)
#define UART2_ENABLED		1
#else
#define UART2_ENABLED		0
#endif
#if defined(CONFIG_UART2_MODE_IRDA)
#define UART2_IR		IRDA
#elif defined(CONFIG_UART2_MODE_RS485)
#define UART2_IR		RS485_HALF
#else
#define UART2_IR		NO_IRDA
#endif
#define UART2_MODE		MODE_DCE
#define UART2_DMA_ENABLED	0
#define UART2_RS485_TXDIR_LVL	0

/* UART 3 configuration */
#if defined(CONFIG_UART3_ENABLED)
#define UART3_ENABLED		1
#else
#define UART3_ENABLED		0
#endif
#if defined(CONFIG_UART3_MODE_IRDA)
#define UART3_IR		IRDA
#elif defined(CONFIG_UART3_MODE_RS485)
#define UART3_IR		RS485_HALF
#else
#define UART3_IR		NO_IRDA
#endif
#define UART3_MODE		MODE_DCE
#define UART3_DMA_ENABLED	0
#define UART3_RS485_TXDIR_LVL	0

/* UART 4 configuration */
#if defined(CONFIG_UART4_ENABLED)
#define UART4_ENABLED		1
#else
#define UART4_ENABLED		0
#endif
#if defined(CONFIG_UART4_MODE_IRDA)
#define UART4_IR		IRDA
#elif defined(CONFIG_UART4_MODE_RS485)
#define UART4_IR		RS485_HALF
#else
#define UART4_IR		NO_IRDA
#endif
#define UART4_MODE		MODE_DCE
#define UART4_DMA_ENABLED	0
#define UART4_RS485_TXDIR_LVL	0

/* UART 5 configuration */
#if defined(CONFIG_UART5_ENABLED)
#define UART5_ENABLED		1
#else
#define UART5_ENABLED		0
#endif
#if defined(CONFIG_UART5_MODE_IRDA)
#define UART5_IR		IRDA
#elif defined(CONFIG_UART5_MODE_RS485)
#define UART5_IR		RS485_HALF
#else
#define UART5_IR		NO_IRDA
#endif
#define UART5_MODE		MODE_DCE
#define UART5_DMA_ENABLED	0
#define UART5_RS485_TXDIR_LVL	0

/*!
 * Specifies if the Irda transmit path is inverting
 */
#define MXC_IRDA_TX_INV	0

#define MXC_LL_UART_PADDR	UART1_BASE_ADDR
#define MXC_LL_UART_VADDR	AIPS1_IO_ADDRESS(UART1_BASE_ADDR)

#ifdef CONFIG_CONSOLE_UART1
#define CONSOLE_UART_BASE_ADDR	UART1_BASE_ADDR
#elif defined(CONFIG_CONSOLE_UART2)
#define CONSOLE_UART_BASE_ADDR	UART2_BASE_ADDR
#elif defined(CONFIG_CONSOLE_UART3)
#define CONSOLE_UART_BASE_ADDR	UART3_BASE_ADDR
#elif defined(CONFIG_CONSOLE_UART4)
#define CONSOLE_UART_BASE_ADDR	UART4_BASE_ADDR
#elif defined(CONFIG_CONSOLE_UART5)
#define CONSOLE_UART_BASE_ADDR	UART5_BASE_ADDR
#else
#define CONSOLE_UART_BASE_ADDR	0
#endif

/* framebuffer settings */
#if defined(CONFIG_CCIMX5X_DISP0) && defined(CONFIG_CCIMX5X_DISP1)
#define FB_MEM_SIZE             (SZ_1M * 48)    /* 1920x1080x32bpp x 3 buffers x 2 interfaces*/
#else
#define FB_MEM_SIZE             (SZ_1M * 24)    /* 1920x1080x32bpp x 3 buffers */
#endif

#if defined(CONFIG_MXC_AMD_GPU) || defined(CONFIG_MXC_AMD_GPU_MODULE)
#define GPU_MEM_SIZE		SZ_64M
#else
#define GPU_MEM_SIZE		0
#endif

#endif		/* __ASM_ARCH_MXC_BOARD_CCIMX5X_H__ */

/*
 * Copyright 2010 Digi International, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __ASM_ARCH_MXC_BOARD_CCIMX51_H__
#define __ASM_ARCH_MXC_BOARD_CCIMX51_H__

#include "board-ccimx5x.h"

/* UARTs RS485 direction GPIOs */
#define UART1_RS485_TXDIR_GPIO	IOMUX_TO_GPIO(MX51_PIN_UART1_CTS)
#define UART1_RS485_TXDIR_LVL	0
#define UART2_RS485_TXDIR_GPIO	IOMUX_TO_GPIO(MX51_PIN_USBH1_DATA0)
#define UART2_RS485_TXDIR_LVL	0
#define UART3_RS485_TXDIR_GPIO	IOMUX_TO_GPIO(MX51_PIN_KEY_COL5)
#define UART3_RS485_TXDIR_LVL	0

/* Second touch interface configuration */
#ifdef CONFIG_CCIMX5X_SECOND_TOUCH
#ifdef CONFIG_JSCCIMX51_V1
/* Settings for the JSCCIMX51 Board RevA, for the DISP0 */
#elif defined(CONFIG_JSCCIMX51_V2)
/* Settings for the JSCCIMX51 Board RevB, for the DISP0/DISP1 */
#endif /* CONFIG_JSCCIMX51_VX */
#endif /* CONFIG_CCIMX5X_SECOND_TOUCH */

/* Set Base board revision */
#ifdef CONFIG_JSCCIMX51_V1
/* Board revision and mach name postfix */
#define BASE_BOARD_REV		1
#define BOARD_NAME		" on a EAK board"
/* SD1 card detect irq */
#define CCIMX51_SD1_CD_IRQ	IOMUX_TO_IRQ(MX51_PIN_GPIO1_0)
#define CCIMX51_SD2_CD_IRQ	0 /* Customize this value to support a CD irq on the SD2 */
#define AD9389_GPIO_IRQ		MX51_PIN_GPIO1_4	/* AD9389 interrupt */

/* Second touch settings */
#define SECOND_TS_IRQ_PIN	MX51_PIN_DI1_D0_CS
#define SECOND_TS_SPI_SS_PIN	MX51_PIN_DI1_D1_CS
#elif defined(CONFIG_JSCCIMX51_V2)
/* Board revision */
#define BASE_BOARD_REV		2
#define BOARD_NAME		" on a JSK board"
/* SD1 card detect irq, not present CD line... */
#define CCIMX51_SD1_CD_IRQ	0
#define CCIMX51_SD2_CD_IRQ	0 /* Customize this value to support a CD irq on the SD2 */
#define AD9389_GPIO_IRQ		MX51_PIN_GPIO1_0	/* AD9389 interrupt */
/* Second touch settings */
#define SECOND_TS_IRQ_PIN	MX51_PIN_DI1_D0_CS
#define SECOND_TS_SPI_SS_PIN	MX51_PIN_CSPI1_RDY
#else
#define BASE_BOARD_REV		0
#define BOARD_NAME		" on a custom board"
#define CCIMX51_SD1_CD_IRQ	0	/* Customize this value to support a CD irq on the SD1 */
#define CCIMX51_SD2_CD_IRQ	0 /* Customize this value to support a CD irq on the SD2 */
/* #define AD9389_GPIO_IRQ */
#endif

void ccimx51_2nd_touch_gpio_init(void);
void ccimx51_init_2nd_touch(void);

/* CPU models */
enum {
	IMX512	= 2,
	IMX513	= 3,
	IMX514	= 4,
	IMX515	= 5,
	IMX516	= 6,
};

#endif		/* __ASM_ARCH_MXC_BOARD_CCIMX51_H__ */

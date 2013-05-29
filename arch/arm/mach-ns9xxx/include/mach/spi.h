/*
 * arch/arm/mach-ns9xxx/include/mach/spi.h
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
*/
#ifndef __ASM_ARCH_SPI_H
#define __ASM_ARCH_SPI_H

#define SPI_MOSI_GPIO_OFFSET	0
#define SPI_MISO_GPIO_OFFSET	1
#define SPI_CLK_GPIO_OFFSET	2
#define SPI_EN_GPIO_OFFSET	3


#define SPI_MAX_GPIO	10

struct spi_ns9xxx_data {
	char gpios[SPI_MAX_GPIO];
	char gpio_funcs[SPI_MAX_GPIO];
	char nr_gpios;
};

#endif /* ifndef __ASM_ARCH_SPI_H */

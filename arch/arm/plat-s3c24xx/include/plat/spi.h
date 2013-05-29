/* -*- linux-c -*-
 *
 * include/asm-arm/plat-s3c24xx/spi.h
 *
 * Copyright (c) 2008 Digi International Spain
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Author : Luis Galdos
 *
 */

#ifndef __ASM_ARM_S3C24XX_SPI_H
#define __ASM_ARM_S3C24XX_SPI_H

/* Macros for the configuration of the GPIOs */
#define SPI_MOSI_GPIO_OFFSET		0
#define SPI_MISO_GPIO_OFFSET		1
#define SPI_CLK_GPIO_OFFSET		2
#define SPI_EN_GPIO_OFFSET		3

#define MAX_SPI_GPIOS			10
#define S3C2443_SPI_MAX_CS		1

struct s3c24xx_spi_gpio {
	unsigned long nr;
	unsigned long cfg;
};

/*
 * The HS SPI controller supports two different input clocks:
 * PCLK : 33MHz
 * EPLL : 48MHz
 */
enum s3c2443_hsspi_clk {
        S3C2443_HSSPI_INCLK_EPLL,
        S3C2443_HSSPI_INCLK_PCLK,
};

/* Platform data for the SPI-controller */
struct s3c2443_spi_info {
	signed short bus_num;
	struct s3c24xx_spi_gpio mosi;
	struct s3c24xx_spi_gpio miso;
	struct s3c24xx_spi_gpio clk;
	struct s3c24xx_spi_gpio cs[S3C2443_SPI_MAX_CS];
        unsigned int num_chipselect;
        enum s3c2443_hsspi_clk input_clk;
};


#endif /* __ASM_ARM_S3C24XX_SPI_H */

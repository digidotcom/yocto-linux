/* arch/arm/mach-s3c24100/include/mach/gpio-core.h
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * S3C2410 - GPIO core support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_GPIO_CORE_H
#define __ASM_ARCH_GPIO_CORE_H __FILE__

#include <mach/regs-gpio.h>

#ifdef CONFIG_DEFAULT_GPIOLIB
extern struct s3c_gpio_chip s3c24xx_gpios[];

static inline struct s3c_gpio_chip *s3c_gpiolib_getchip(unsigned int pin)
{
	struct s3c_gpio_chip *chip;

	if (pin > S3C_GPIO_END)
		return NULL;

	chip = &s3c24xx_gpios[pin/32];
	return ((pin - chip->chip.base) < chip->chip.ngpio) ? chip : NULL;
}
#else

extern struct s3c_gpio_chip s3c2443_gpio_ports[];
extern size_t s3c2443_gpio_ports_size(void);

static inline struct s3c_gpio_chip *s3c_gpiolib_getchip(unsigned int pin)
{
	struct s3c_gpio_chip *chip = NULL;
	u32 i;

	if (pin >= (S3C2410_GPM(0) + S3C2410_GPIO_M_NR))
		return NULL;

	for (i = 0; i < s3c2443_gpio_ports_size(); i++) {

		if ( (pin >= s3c2443_gpio_ports[i].chip.base) &&
			(pin < (s3c2443_gpio_ports[i].chip.base + s3c2443_gpio_ports[i].chip.ngpio))) {
			chip = &s3c2443_gpio_ports[i];
		}
	}

	return chip;
}

#endif

#endif /* __ASM_ARCH_GPIO_CORE_H */

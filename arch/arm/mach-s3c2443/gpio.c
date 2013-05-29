/* -*- linux-c -*-
 *
 * linux/arch/arm/mach-s3c2443/gpio.c
 *
 * Copyright (c) 2009 Digi International Spain
 *	Luis Galdos <luis.galdos@digi.com>
 *
 * http://www.digi.com/products/embeddedsolutions/connectcore9m.jsp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/sysdev.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <plat/gpio-core.h>
#include <mach/gpio.h>
#include <mach/gpio-track.h>
#include <plat/gpio-cfg-helpers.h>
#include <plat/pm.h>

struct s3c_gpio_chip s3c2443_gpio_ports[];

unsigned int s3c2443_gpio_read_porta(unsigned int pin)
{
	unsigned long gpacdh, gpacdl;
	unsigned long flags;
	unsigned long res, mask;
	int i;

	/* Port A requieres special handling... */
	local_irq_save(flags);
	gpacdl = __raw_readl(S3C2410_GPACON);
	gpacdh = __raw_readl(S3C2410_GPACON + 0x4);
	local_irq_restore(flags);

	if (pin > S3C2410_GPA(7)) {
		gpacdl >>= 8;
		gpacdh >>= 8;
	}

	for (i=0, res = 0, mask = 0x1; i < 8; i++) {
		res |= (((gpacdh & mask) | ((gpacdl & mask) << 1)) << i);
		mask = mask << 1;
	}

	if (pin > S3C2410_GPA(7))
		res |= (gpacdh & mask) << i;

	return res;
}

/* Code coming from the U-Boot 1.1.6 */
void s3c2443_gpio_setpin(unsigned int pin, unsigned int to)
{
	unsigned long dat;
	struct s3c_gpio_chip *port;
	unsigned long offs;

	port = s3c_gpiolib_getchip(pin);
	if (port == NULL)
		return;

	offs = pin - port->chip.base;

	if (pin < S3C2410_GPB(0)) {
		dat = s3c2443_gpio_read_porta(pin);
		if (pin > S3C2410_GPA(7)) {
			dat &= ~(1 << ((offs - S3C2410_GPA(7) - 1)  * 2));
			dat |= to << ((offs - S3C2410_GPA(7) - 1) * 2);
			writel(dat, port->base + 0x04);
		} else {
			dat &= ~(1 << (offs * 2));
			dat |= to << (offs * 2);
			writel(dat, port->base);
		}
	} else
		s3c2410_gpio_setpin(pin, to);
}

void s3c2443_gpio_cfgpin(unsigned int pin, unsigned int function)
{
	struct s3c_gpio_chip *port;
	unsigned long mask;
	unsigned long con;
	unsigned long offs;
	unsigned long flags;

	if (pin >= S3C2410_GPB(0)) {
		s3c2410_gpio_cfgpin(pin, function);
		return;
	}

	port = s3c_gpiolib_getchip(pin);
	if (port == NULL)
		return;

	/* Port A requieres special handling... */
	con  = s3c2443_gpio_read_porta(pin);

	offs = pin - port->chip.base;
	if (pin > S3C2410_GPA(7))
		offs = offs - S3C2410_GPA(7) - 1;

	mask = 1 << ((offs * 2) + 1);
	con &= ~mask;
	con |= (function << ((offs * 2) + 1));

	local_irq_save(flags);
	if (pin > S3C2410_GPA(7) )
		__raw_writel(con, port->base + 0x4);
	else
		__raw_writel(con, port->base);
	local_irq_restore(flags);
}

/* Setup the UDP register of the S3C2443 ports */
void s3c2443_gpio_set_udp(unsigned int pin, int val)
{
	struct s3c_gpio_chip *port;
	unsigned long offs;
	unsigned long flags;
	unsigned long up;

	port = s3c_gpiolib_getchip(pin);
	if (port == NULL)
		return;

	offs = (pin - port->chip.base) * 2;

	local_irq_save(flags);

	up = __raw_readl(port->base + 0x08);
	up &= ~(3L << offs);
	up |= val << offs;
	__raw_writel(up, port->base + 0x08);

	local_irq_restore(flags);
}

/* Enable the pull-down of an external interrupt GPIO */
int s3c2443_gpio_extpull(unsigned int pin, int pullup)
{
	struct s3c_gpio_chip *port;
	void __iomem *base;
	unsigned long regval;
	unsigned int offs;

	if (pin < S3C2410_GPF(0) || pin > S3C2410_GPG(7))
		return -ENODEV;

	if (pin >= S3C2410_GPG(0))
		base = S3C24XX_EXTINT1;
	else
		base = S3C24XX_EXTINT0;

	port = s3c_gpiolib_getchip(pin);
	if (port == NULL)
		return -ENODEV;
	offs = ((pin - port->chip.base) * 4) + 3;

	/*
	 * First clear the control bit with the corresponding offset and then
	 * use the passed configuration value for setting the control bit
	 */
	pullup &= 0x1;

	/* There is a HW bug in the EXTINT0 register (code coming from the SMDK) */
	regval = __raw_readl(base);
	if (base == S3C24XX_EXTINT0) {
		unsigned int t;
		t  =   (regval  &  0x0000000f)  <<  28;
		t  |=  (regval  &  0x000000f0)  <<  20;
		t  |=  (regval  &  0x00000f00)  <<  12;
		t  |=  (regval  &  0x0000f000)  <<  4;
		t  |=  (regval  &  0x000f0000)  >>  4;
		t  |=  (regval  &  0x00f00000)  >>  12;
		t  |=  (regval  &  0x0f000000)  >>  20;
		t  |=  (regval  &  0xf0000000)  >>  28;
		regval  =  t;
	}

	regval &= (~0x1UL << offs);
	regval |= (pullup << offs);
	__raw_writel(regval, base);
	return 0;
}

int s3c2443_gpio_getirq(unsigned gpio)
{
	return gpio_to_irq(gpio);
}

static irqreturn_t s3c2443_wakeup_irq(int irq, void *data)
{
	return IRQ_HANDLED;
}

static int s3c2443_gpio_wakeup_conf(struct gpio_chip *chip, unsigned gpio, int enable)
{
	struct s3c_gpio_chip *port = to_s3c_gpio(chip);
	int irq, ret = 0;

	/* Configure as wakeup interrupt if capable */
	if ( (port->base == S3C2410_GPGCON) &&
		(gpio >= 8) ) {
		pr_err("[ ERROR ] GPIO %i not wakeup capable\n", gpio);
		return -ENODEV;
	}

	irq = chip->to_irq(chip, gpio);
	if (irq < 0) {
		pr_err("[ ERROR ] Couldn't get an IRQ for gpio %i\n", gpio);
		return -ENODEV;
	}

	if (enable) {
		ret = request_irq(irq, s3c2443_wakeup_irq,
				  IRQF_DISABLED | IRQF_TRIGGER_FALLING,
				  "s3c2443-wio", NULL);
		if (ret) {
			pr_err("[ ERROR ] Couldn't request IRQ %i\n", irq);
			goto exit_wakeup;
		}
		enable_irq_wake(irq);
	} else {
		disable_irq_wake(irq);
		free_irq(irq, NULL);
		chip->direction_input(chip, gpio);
	}

 exit_wakeup:
	return ret;
}

static void s3c2443_set_pullupdown(struct gpio_chip *chip, unsigned offset, int value)
{
	struct s3c_gpio_chip *ourchip = to_s3c_gpio(chip);
	void __iomem *base = ourchip->base;
	int pin = offset + chip->base;

	/* The following ports don't have pull-up/down control */
	if (S3C2410_GPACON == base ||
	    S3C2410_GPCCON == base ||
	    S3C2410_GPDCON == base ||
	    S3C2410_GPFCON == base)
		return;

	s3c2443_gpio_set_udp(pin, value);
}

static int s3c24xx_gpiolib_banka_input(struct gpio_chip *chip, unsigned offset)
{
	return -EINVAL;
}

static int s3c24xx_gpiolib_banka_output(struct gpio_chip *chip,
					unsigned offset, int value)
{
	struct s3c_gpio_chip *ourchip = to_s3c_gpio(chip);
	void __iomem *base = ourchip->base;
	unsigned long flags;

	local_irq_save(flags);
	if (S3C2410_GPACON == base) {
		s3c2443_gpio_cfgpin(offset, 0);
	}
	else {
		s3c2410_gpio_cfgpin(offset, S3C2410_GPIO_OUTPUT);
	}
	s3c2443_gpio_setpin(offset, value);
	local_irq_restore(flags);

	return 0;
}

static int s3c24xx_gpiolib_bankf_toirq(struct gpio_chip *chip, unsigned offset)
{
	if (offset < 4)
		return IRQ_EINT0 + offset;

	if (offset < 8)
		return IRQ_EINT4 + offset - 4;

	return -EINVAL;
}

static int s3c24xx_gpiolib_bankg_toirq(struct gpio_chip *chip, unsigned offset)
{
	return IRQ_EINT8 + offset;
}

static int s3c2443_set_pull(struct s3c_gpio_chip *chip, unsigned int off, s3c_gpio_pull_t pull)
{
	switch (pull) {
	case S3C_GPIO_PULL_NONE:
		pull = 0x01;
		break;
	case S3C_GPIO_PULL_DOWN:
		pull = 0x02;
		break;
	case S3C_GPIO_PULL_UP:
		pull = 0x00;
		break;
	}
	return s3c_gpio_setpull_updown(chip, off, pull);
}

static s3c_gpio_pull_t s3c2443_get_pull(struct s3c_gpio_chip *chip, unsigned int off)
{
	s3c_gpio_pull_t pull;

	pull = s3c_gpio_getpull_updown(chip, off);
	switch (pull) {
	case 0x00:
		pull = S3C_GPIO_PULL_UP;
		break;
	case 0x01:
	case 0x03:
		pull = S3C_GPIO_PULL_NONE;
		break;
	case 0x02:
		pull = S3C_GPIO_PULL_DOWN;
		break;
	}

	return pull;
}

static struct s3c_gpio_cfg s3c24xx_gpiocfg_banka = {
	.set_config	= s3c_gpio_setcfg_s3c24xx_a,
	.get_config	= s3c_gpio_getcfg_s3c24xx_a,
};

struct s3c_gpio_cfg s3c24xx_gpiocfg_default = {
	.set_config	= s3c_gpio_setcfg_s3c24xx,
	.get_config	= s3c_gpio_getcfg_s3c24xx,
	.set_pull	= s3c2443_set_pull,
	.get_pull	= s3c2443_get_pull,
};

static void s3c2443_gpio_pm_porta_save(struct s3c_gpio_chip *chip)
{
	chip->pm_save[0] = s3c2443_gpio_read_porta(S3C2410_GPA(0));
	chip->pm_save[1] = s3c2443_gpio_read_porta(S3C2410_GPA(8));
}

static void s3c2443_gpio_pm_porta_resume(struct s3c_gpio_chip *chip)
{
	u32 old_gpal = s3c2443_gpio_read_porta(S3C2410_GPA(0));
	u32 old_gpah = s3c2443_gpio_read_porta(S3C2410_GPA(8));
	u32 gps_gpal = chip->pm_save[0];
	u32 gps_gpah = chip->pm_save[1];

	__raw_writel(gps_gpal, chip->base);
	__raw_writel(gps_gpah, chip->base + 0x04);

	S3C_PMDBG("%s: GPACDL %08x => %08x, GPACDH %08x => %08x\n",
		  chip->chip.label, old_gpal, gps_gpal, old_gpah, gps_gpah);
}

struct s3c_gpio_pm s3c2443_gpio_pm_porta = {
	.save	= s3c2443_gpio_pm_porta_save,
	.resume = s3c2443_gpio_pm_porta_resume,
};

struct s3c_gpio_chip s3c2443_gpio_ports[] = {
	[0] = {
		.base	= S3C2410_GPACON,
		.pm	= __gpio_pm(&s3c2443_gpio_pm_porta),
		.config	= &s3c24xx_gpiocfg_banka,
		.chip	= {
			.base			= S3C2410_GPA(0),
			.owner			= THIS_MODULE,
			.label			= "PORTA",
			.ngpio			= S3C2410_GPIO_A_NR,
			.direction_input	= s3c24xx_gpiolib_banka_input,
			.direction_output	= s3c24xx_gpiolib_banka_output,
			.pullupdown	  	= s3c2443_set_pullupdown,
		},
	},
	[1] = {
		.base	= S3C2410_GPBCON,
		.pm	= __gpio_pm(&s3c_gpio_pm_2bit),
		.config	= &s3c24xx_gpiocfg_default,
		.chip	= {
			.base			= S3C2410_GPB(0),
			.owner			= THIS_MODULE,
			.label			= "PORTB",
			.ngpio			= S3C2410_GPIO_B_NR,
			.pullupdown	  	= s3c2443_set_pullupdown,
		},
	},
	[2] = {
		.base	= S3C2410_GPCCON,
		.pm	= __gpio_pm(&s3c_gpio_pm_2bit),
		.config	= &s3c24xx_gpiocfg_default,
		.chip	= {
			.base			= S3C2410_GPC(0),
			.owner			= THIS_MODULE,
			.label			= "PORTC",
			.ngpio			= S3C2410_GPIO_C_NR,
			.pullupdown	  	= s3c2443_set_pullupdown,
		},
	},
	[3] = {
		.base	= S3C2410_GPDCON,
		.pm	= __gpio_pm(&s3c_gpio_pm_2bit),
		.config	= &s3c24xx_gpiocfg_default,
		.chip	= {
			.base			= S3C2410_GPD(0),
			.owner			= THIS_MODULE,
			.label			= "PORTD",
			.ngpio			= S3C2410_GPIO_D_NR,
			.pullupdown	  	= s3c2443_set_pullupdown,
		},
	},
	[4] = {
		.base	= S3C2410_GPECON,
		.pm	= __gpio_pm(&s3c_gpio_pm_2bit),
		.config	= &s3c24xx_gpiocfg_default,
		.chip	= {
			.base			= S3C2410_GPE(0),
			.label			= "PORTE",
			.owner			= THIS_MODULE,
			.ngpio			= S3C2410_GPIO_E_NR,
			.pullupdown	  	= s3c2443_set_pullupdown,
		},
	},
	[5] = {
		.base	= S3C2410_GPFCON,
		.pm	= __gpio_pm(&s3c_gpio_pm_2bit),
		.config	= &s3c24xx_gpiocfg_default,
		.chip	= {
			.base			= S3C2410_GPF(0),
			.owner			= THIS_MODULE,
			.label			= "PORTF",
			.ngpio			= S3C2410_GPIO_F_NR,
			.to_irq			= s3c24xx_gpiolib_bankf_toirq,
			.wakeup_configure	= s3c2443_gpio_wakeup_conf,
			.pullupdown	  	= s3c2443_set_pullupdown,
		},
	},
	[6] = {
		.base	= S3C2410_GPGCON,
		.pm	= __gpio_pm(&s3c_gpio_pm_2bit),
		.config	= &s3c24xx_gpiocfg_default,
		.chip	= {
			.base			= S3C2410_GPG(0),
			.owner			= THIS_MODULE,
			.label			= "PORTG",
			.ngpio			= S3C2410_GPIO_G_NR,
			.to_irq			= s3c24xx_gpiolib_bankg_toirq,
			.wakeup_configure	= s3c2443_gpio_wakeup_conf,
			.pullupdown	  	= s3c2443_set_pullupdown,
		},
	},
	[7] = {
		.base	= S3C2410_GPHCON,
		.pm	= __gpio_pm(&s3c_gpio_pm_2bit),
		.config	= &s3c24xx_gpiocfg_default,
		.chip	= {
			.base			= S3C2410_GPH(0),
			.owner			= THIS_MODULE,
			.label			= "PORTH",
			.ngpio			= S3C2410_GPIO_H_NR,
			.pullupdown	  	= s3c2443_set_pullupdown,
		},
	},
	[8] = {
		.base	= S3C2440_GPJCON,
		.pm	= __gpio_pm(&s3c_gpio_pm_2bit),
		.config	= &s3c24xx_gpiocfg_default,
		.chip	= {
			.base			= S3C2410_GPJ(0),
			.owner			= THIS_MODULE,
			.label			= "PORTJ",
			.ngpio			= S3C2410_GPIO_J_NR,
			.pullupdown	  	= s3c2443_set_pullupdown,
		},
	},
	[9] = {
		.base	= S3C2443_GPLCON,
		.pm	= __gpio_pm(&s3c_gpio_pm_2bit),
		.config	= &s3c24xx_gpiocfg_default,
		.chip	= {
			.base			= S3C2410_GPL(0),
			.owner			= THIS_MODULE,
			.label			= "PORTL",
			.ngpio			= S3C2410_GPIO_L_NR,
			.pullupdown	  	= s3c2443_set_pullupdown,
		},
	},
	[10] = {
		.base	= S3C2443_GPMCON,
		.pm	= __gpio_pm(&s3c_gpio_pm_2bit),
		.config	= &s3c24xx_gpiocfg_default,
		.chip	= {
			.base			= S3C2410_GPM(0),
			.owner			= THIS_MODULE,
			.label			= "PORTM",
			.ngpio			= S3C2410_GPIO_M_NR,
			.pullupdown	  	= s3c2443_set_pullupdown,
		},
	},
};

size_t s3c2443_gpio_ports_size(void)
{
	return ARRAY_SIZE(s3c2443_gpio_ports);
}

/* Gets the s3c2443 GPIO number from a s3c2410 GPIO number */
unsigned int s3c2443_gpio_num(unsigned gpio)
{
	int i;
	u32 bankbase = gpio / 32;

	for (i = 0; i < ARRAY_SIZE(s3c2443_gpio_ports); i++) {
		if (bankbase == (unsigned int)s3c2443_gpio_ports[i].base) {
			return (s3c2443_gpio_ports[i].chip.base + (gpio & 0x31));
		}
	}

	return -1;
}

static int __init s3c2443_gpio_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(s3c2443_gpio_ports); i++) {
		s3c_gpiolib_add(&s3c2443_gpio_ports[i]);
	}

	return 0;
}
pure_initcall(s3c2443_gpio_init);

EXPORT_SYMBOL(s3c2443_gpio_getirq);
EXPORT_SYMBOL(s3c2443_gpio_extpull);
EXPORT_SYMBOL(s3c2443_gpio_cfgpin);
EXPORT_SYMBOL(s3c2443_gpio_set_udp);
EXPORT_SYMBOL(s3c2443_gpio_setpin);
EXPORT_SYMBOL(s3c2443_gpio_num);

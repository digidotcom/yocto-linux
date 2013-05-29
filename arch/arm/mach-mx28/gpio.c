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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#include <mach/pinctrl.h>

#include "regs-pinctrl.h"

#define PINCTRL_BASE_ADDR IO_ADDRESS(PINCTRL_PHYS_ADDR)

#define MAX_GPIO_BANKS	(5)

static unsigned int mx28_gpio_wake_interrupt_enable[MAX_GPIO_BANKS];
static unsigned int saved_pin2irq[MAX_GPIO_BANKS];
static unsigned int saved_irqen[MAX_GPIO_BANKS];

static int
mx28_gpio_direction(struct mxs_gpio_port *port, int pin, unsigned int input)
{
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;
	if (input)
		__raw_writel(1 << pin, base + HW_PINCTRL_DOE0_CLR);
	else
		__raw_writel(1 << pin, base + HW_PINCTRL_DOE0_SET);

	return 0;
}

static int mx28_gpio_get(struct mxs_gpio_port *port, int pin)
{
	unsigned int data;
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;

	data = __raw_readl(base + HW_PINCTRL_DIN0);
	return ((data >> pin) & 1);
}

static void mx28_gpio_set(struct mxs_gpio_port *port, int pin, int data)
{
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;
	if (data)
		__raw_writel(1 << pin, base + HW_PINCTRL_DOUT0_SET);
	else
		__raw_writel(1 << pin, base + HW_PINCTRL_DOUT0_CLR);
}

static unsigned int mx28_gpio_irq_stat(struct mxs_gpio_port *port)
{
	unsigned int mask;
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;
	mask = __raw_readl(base + HW_PINCTRL_IRQSTAT0);
	mask &= __raw_readl(base + HW_PINCTRL_IRQEN0);
	return mask;
}

static int
mx28_gpio_set_irq_type(struct mxs_gpio_port *port, int pin, unsigned int type)
{
	unsigned int level, pol;
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		level = 0;
		pol = 1;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		level = 0;
		pol = 0;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		level = 1;
		pol = 1;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		level = 1;
		pol = 0;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		printk(KERN_WARNING"IRQ type BOTH not supported\n");
	default:
		pr_debug("%s: Incorrect GPIO interrupt type 0x%x\n",
			 __func__, type);
		return -ENXIO;
	}

	if (level)
		__raw_writel(1 << pin, base + HW_PINCTRL_IRQLEVEL0_SET);
	else
		__raw_writel(1 << pin, base + HW_PINCTRL_IRQLEVEL0_CLR);

	if (pol)
		__raw_writel(1 << pin, base + HW_PINCTRL_IRQPOL0_SET);
	else
		__raw_writel(1 << pin, base + HW_PINCTRL_IRQPOL0_CLR);

	return 0;
}

static void mx28_gpio_unmask_irq(struct mxs_gpio_port *port, int pin)
{
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;
	__raw_writel(1 << pin, base + HW_PINCTRL_IRQEN0_SET);
}

static void mx28_gpio_mask_irq(struct mxs_gpio_port *port, int pin)
{
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;
	__raw_writel(1 << pin, base + HW_PINCTRL_IRQEN0_CLR);
}

static void mx28_gpio_ack_irq(struct mxs_gpio_port *port, int pin)
{
	unsigned int mask;
	void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * port->id;
	mask = 1 << pin;
	if (mask)
		__raw_writel(mask, base + HW_PINCTRL_IRQSTAT0_CLR);
}

/*
 * Store whether a GPIO pin will be selected as a possible wake up source.
 */
static int mx28_gpio_wakeup_configure(struct mxs_gpio_port *port, unsigned int pin, int enable)
{
	if (enable) {
		mx28_gpio_wake_interrupt_enable[port->id] |= 1 << GPIO_TO_PINS(pin);
	} else {
		mx28_gpio_wake_interrupt_enable[port->id] &= ~(1 << GPIO_TO_PINS(pin));
	}

	return (enable != 0);
}

/*
 * Save the current GPIO interrupt configuration and then disable interrupts
 * from all GPIOs that are not wake up sources.
 */
void mx28_gpio_suspend(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(saved_pin2irq); i++) {
		unsigned int offset = 0x10 * i;

		saved_pin2irq[i] = __raw_readl(PINCTRL_BASE_ADDR + HW_PINCTRL_PIN2IRQ0 + offset);
		saved_irqen[i] = __raw_readl(PINCTRL_BASE_ADDR + HW_PINCTRL_IRQEN0 + offset);
		__raw_writel(mx28_gpio_wake_interrupt_enable[i], PINCTRL_BASE_ADDR + HW_PINCTRL_PIN2IRQ0 + offset);
		__raw_writel(mx28_gpio_wake_interrupt_enable[i], PINCTRL_BASE_ADDR + HW_PINCTRL_IRQEN0 + offset);
	}
}

/*
 * Restore GPIO interrupt configuration saved by mx28_gpio_suspend.
 */
void mx28_gpio_resume(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(saved_pin2irq); i++) {
		unsigned int offset = 0x10 * i;

		__raw_writel(saved_pin2irq[i], PINCTRL_BASE_ADDR + HW_PINCTRL_PIN2IRQ0 + offset);
		__raw_writel(saved_irqen[i], PINCTRL_BASE_ADDR + HW_PINCTRL_IRQEN0 + offset);
	}
}


static struct mxs_gpio_port mx28_gpios[] = {
	{
	 .irq = IRQ_GPIO0,
	 },
	{
	 .irq = IRQ_GPIO1,
	 },
	{
	 .irq = IRQ_GPIO2,
	 },
	{
	 .irq = IRQ_GPIO3,
	 },
	{
	 .irq = IRQ_GPIO4,
	 },
};

static struct mxs_gpio_chip mx28_gpio_chip = {
	.set_dir = mx28_gpio_direction,
	.get = mx28_gpio_get,
	.set = mx28_gpio_set,
	.get_irq_stat = mx28_gpio_irq_stat,
	.set_irq_type = mx28_gpio_set_irq_type,
	.unmask_irq = mx28_gpio_unmask_irq,
	.mask_irq = mx28_gpio_mask_irq,
	.ack_irq = mx28_gpio_ack_irq,
	.wakeup_configure = mx28_gpio_wakeup_configure,
};

static int mx28_gpio_set_wake_irq(unsigned int irq, unsigned int enabled)
{
#define NUMBER_OF_BANKS ARRAY_SIZE(mx28_gpios)
	int result = -1;
	int gpio = (irq - ARCH_NR_IRQS);
	int bank = GPIO_TO_BANK(gpio);
	int real_irq = IRQ_GPIO0 - bank;
	static int real_irq_wake_count[NUMBER_OF_BANKS] = {0};
	static int gpio_wake_enabled[PINS_PER_BANK * NUMBER_OF_BANKS] = {0};

	if ((gpio >= 0) && (gpio < (PINS_PER_BANK * NUMBER_OF_BANKS))) {
		enabled = !!enabled;
		if ((enabled) && (!gpio_wake_enabled[gpio])) {
			gpio_wake_enabled[gpio] = 1;
			if (++real_irq_wake_count[bank] == 1) {
				set_irq_wake(real_irq, 1);
			}
		} else if ((!enabled) && (gpio_wake_enabled[gpio])) {
			gpio_wake_enabled[gpio] = 0;
			if (--real_irq_wake_count[bank] == 0) {
				set_irq_wake(real_irq, 0);
			}
		}
		result = 0;
	}

	return result;
}


int __init mx28_gpio_init(void)
{
	int i;
	unsigned int reg;
	if (__raw_readl(PINCTRL_BASE_ADDR + HW_PINCTRL_CTRL_CLR) &
	    BM_PINCTRL_CTRL_SFTRST) {
		__raw_writel(BM_PINCTRL_CTRL_SFTRST,
			     PINCTRL_BASE_ADDR + HW_PINCTRL_CTRL_CLR);
		for (i = 0; i < 10000; i++) {
			if (!(__raw_readl(PINCTRL_BASE_ADDR + HW_PINCTRL_CTRL) &
			      BM_PINCTRL_CTRL_SFTRST))
				break;
			udelay(2);
		}
		if (i >= 10000)
			return -EFAULT;

		__raw_writel(BM_PINCTRL_CTRL_CLKGATE,
			     PINCTRL_BASE_ADDR + HW_PINCTRL_CTRL_CLR);
	}

	reg = __raw_readl(PINCTRL_BASE_ADDR + HW_PINCTRL_CTRL);
	mxs_set_gpio_wake(mx28_gpio_set_wake_irq);
	for (i = 0; i < ARRAY_SIZE(mx28_gpios); i++) {
		void __iomem *base = PINCTRL_BASE_ADDR + 0x10 * i;
		if (!(reg & (BM_PINCTRL_CTRL_PRESENT0 << i)))
			continue;
		mxs_set_gpio_chip(&mx28_gpios[i], &mx28_gpio_chip);
		mx28_gpios[i].id = i;
		__raw_writel(0, base + HW_PINCTRL_IRQEN0);
		__raw_writel(0xFFFFFFFF, base + HW_PINCTRL_PIN2IRQ0);
		mx28_gpios[i].child_irq = MXS_GPIO_IRQ_START +
		    (i * PINS_PER_BANK);
		mxs_add_gpio_port(&mx28_gpios[i]);
	}
	return 0;
}

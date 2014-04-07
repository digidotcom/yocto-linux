/*
 * GPIO Driver for Dialog DA9063 PMICs.
 *
 * Copyright(c) 2012 Dialog Semiconductor Ltd.
 *
 * Author: David Dajun Chen <dchen@diasemi.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/irqdomain.h>

#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/registers.h>

#define DA9063_VDD_IO			0x1

#define DA9063_GPI			0x1
#define DA9063_OPEN_DRAIN		0x2
#define DA9063_PUSH_PULL		0x3
#define DA9063_OPEN_DRAIN		0x2

#define DA9063_ACT_LOW			0x0
#define DA9063_ACT_HIGH			0x1

#define DA9063_PORT_MASK		0x3
#define DA9063_PORT_SHIFT(offset)	(4 * (offset % 2))

#define DA9063_INPUT			DA9063_GPI
#define DA9063_OUTPUT			DA9063_PUSH_PULL

struct da9063_gpio {
	struct da9063 *da9063;
	struct gpio_chip gp;
};

static inline struct da9063_gpio *to_da9063_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct da9063_gpio, gp);
}

static int da9063_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct da9063_gpio *gpio = to_da9063_gpio(gc);
	int gpio_direction = 0;
	int ret;

	/* Get GPIO direction */
	ret = da9063_reg_read(gpio->da9063, (offset >> 1) + DA9063_REG_GPIO_0_1);
	if (ret < 0)
		return ret;

	gpio_direction = ret & (DA9063_PORT_MASK) << DA9063_PORT_SHIFT(offset);
	gpio_direction >>= DA9063_PORT_SHIFT(offset);
	switch (gpio_direction) {
	case DA9063_INPUT:
		ret = da9063_reg_read(gpio->da9063, DA9063_REG_STATUS_B);
		if (ret < 0)
			return ret;
		break;
	case DA9063_OUTPUT:
		ret = da9063_reg_read(gpio->da9063, DA9063_REG_GPIO_MODE0_7);
		if (ret < 0)
			return ret;
	}

	return ret & (1 << offset);

}

static void da9063_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct da9063_gpio *gpio = to_da9063_gpio(gc);

	da9063_reg_update(gpio->da9063,
			DA9063_REG_GPIO_MODE0_7,
			1 << offset,
			value << offset);
}

static int da9063_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	struct da9063_gpio *gpio = to_da9063_gpio(gc);
	unsigned char reg_byte;

	reg_byte = (DA9063_ACT_LOW | DA9063_GPI)
				<< DA9063_PORT_SHIFT(offset);

	return da9063_reg_update(gpio->da9063, (offset >> 1) +
				DA9063_REG_GPIO_0_1,
				DA9063_PORT_MASK <<
				DA9063_PORT_SHIFT(offset),
				reg_byte);
}

static int da9063_gpio_direction_output(struct gpio_chip *gc,
					unsigned offset, int value)
{
	struct da9063_gpio *gpio = to_da9063_gpio(gc);
	unsigned char reg_byte;
	int ret;

	reg_byte = (DA9063_VDD_IO | DA9063_PUSH_PULL)
					<< DA9063_PORT_SHIFT(offset);

	ret = da9063_reg_update(gpio->da9063, (offset >> 1) +
				DA9063_REG_GPIO_0_1,
				DA9063_PORT_MASK <<
				DA9063_PORT_SHIFT(offset),
				reg_byte);
	if (ret < 0)
		return ret;

	da9063_gpio_set(gc, offset, value);

	return 0;
}

static int da9063_gpio_to_irq(struct gpio_chip *gc, u32 offset)
{
	struct da9063_gpio *gpio = to_da9063_gpio(gc);
	struct da9063 *da9063 = gpio->da9063;

	return irq_find_mapping(da9063->irq_domain,
				  DA9063_IRQ_GPI0 + offset);
}

static struct gpio_chip reference_gp = {
	.label = "da9063-gpio",
	.owner = THIS_MODULE,
	.get = da9063_gpio_get,
	.set = da9063_gpio_set,
	.direction_input = da9063_gpio_direction_input,
	.direction_output = da9063_gpio_direction_output,
	.to_irq = da9063_gpio_to_irq,
	.can_sleep = 1,
	.ngpio = 16,
	.base = -1,
};

static const struct of_device_id da9063_gpio_dt_ids[] = {
	{ .compatible = "dlg,da9063-gpio", },
	{ /* sentinel */ }
};

static int da9063_gpio_probe(struct platform_device *pdev)
{
	struct da9063_gpio *gpio;
	int ret;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (gpio == NULL)
		return -ENOMEM;

	gpio->da9063 = dev_get_drvdata(pdev->dev.parent);
	if(gpio->da9063  == NULL)
		return -EPROBE_DEFER;

	gpio->gp = reference_gp;

	gpio->gp.of_node = pdev->dev.of_node;
	ret = gpiochip_add(&gpio->gp);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n", ret);
		goto err_mem;
	}

	platform_set_drvdata(pdev, gpio);

	return 0;

err_mem:
	return ret;
}

static int da9063_gpio_remove(struct platform_device *pdev)
{
	struct da9063_gpio *gpio = platform_get_drvdata(pdev);
	return gpiochip_remove(&gpio->gp);
}

static struct platform_driver da9063_gpio_driver = {
	.probe = da9063_gpio_probe,
	.remove = da9063_gpio_remove,
	.driver = {
		.name	= "da9063-gpio",
		.owner	= THIS_MODULE,
		.of_match_table = da9063_gpio_dt_ids,
	},
};

static int da9063_gpio_init(void)
{
	return platform_driver_register(&da9063_gpio_driver);
}
subsys_initcall(da9063_gpio_init);

static void da9063_gpio_exit(void)
{
	platform_driver_unregister(&da9063_gpio_driver);
}
module_exit(da9063_gpio_exit);

MODULE_AUTHOR("Digi International <support@digi.com>");
MODULE_DESCRIPTION("DA9063 GPIO Device Driver");
MODULE_LICENSE("GPL");

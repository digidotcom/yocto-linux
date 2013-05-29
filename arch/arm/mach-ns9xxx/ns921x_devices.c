/*
 * arch/arm/mach-ns9xxx/ns921x_devices.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/ns9xxx-eth.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>

#include <mach/regs-sys-ns921x.h>
#include <mach/ns921x-serial.h>
#include <mach/fim-ns921x.h>
#include <mach/gpio.h>

#include "clock.h"
#include "ns921x_devices.h"
#include "cme9210_devices.h"
#include "processor-ns921x.h"

/* Watchdog timer */
#if defined(CONFIG_NS9XXX_WATCHDOG) || defined(CONFIG_NS9XXX_WATCHDOG_MODULE)
static struct clk wdt_clk = {
	.name	= "ns9xxx-wdt",
	.id	= -1,
	.owner	= THIS_MODULE,
};

static struct resource wdt_resources[] = {
	{
		.start	= 0xa0900174,
		.end	= 0xa0900177,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ns9xxx_device_ns921x_wdt = {
	.name		= "ns9xxx-wdt",
	.id		= -1,
	.resource	= wdt_resources,
	.num_resources	= ARRAY_SIZE(wdt_resources),
};

void __init ns9xxx_add_device_ns921x_wdt(void)
{
	wdt_clk.parent = clk_get(NULL, "ahbclock");
	if (IS_ERR(wdt_clk.parent))
		return;

	if (clk_register(&wdt_clk))
		return;

	platform_device_register(&ns9xxx_device_ns921x_wdt);
}
#else
void __init ns9xxx_add_device_ns921x_wdt(void) {}
#endif

/* Ethernet */
#if (defined(CONFIG_NS9XXX_ETH) || defined(CONFIG_NS9XXX_ETH_MODULE)) && !defined(CONFIG_MACH_CWME9210JS)
int __init eth_register_gpios(int gpio[], int func[], int dir[], int num)
{
	int i;

	for (i = 0; i < num; i++) {
		if (gpio_request(gpio[i], "ns9xxx-eth"))
			goto err;
		gpio_configure_ns921x(gpio[i], dir[i], 0, func[i], 0);
	}

	return 0;
err:
	for (; i >= 0; i--)
		gpio_free(gpio[i]);

	return -EBUSY;
}

static struct ns921x_sysclk eth_clk = {
	.clk = {
		.name		= "ns9xxx-eth",
		.id		= -1,
		.owner		= THIS_MODULE,
		.endisable	= ns921x_endisable_sysclock,
	},
	.mask = SYS_CLOCK_ETH,
};

static struct plat_ns9xxx_eth ns9xxx_device_ns921x_eth_data = {
	.irqrx = IRQ_NS9XXX_ETHRX,
	.irqtx = IRQ_NS9XXX_ETHTX,
};

static struct resource eth_resources[] = {
	{
		.start	= 0xa0600000,
		.end	= 0xa06027ff,
		.flags	= IORESOURCE_MEM,
	}
};

static struct platform_device ns9xxx_device_ns921x_eth = {
	.name		= "ns9xxx-eth",
	.id		= -1,
	.resource	= eth_resources,
	.num_resources	= ARRAY_SIZE(eth_resources),
	.dev = {
		.platform_data	= &ns9xxx_device_ns921x_eth_data,
	},
};

void __init ns9xxx_add_device_ns921x_eth(struct clk *phyclk, u32 phy_mask,
		int gpio[], int func[], int dir[], int num)
{
	if (eth_register_gpios(gpio, func, dir, num))
		return;

	eth_clk.clk.parent = phyclk;
	ns9xxx_device_ns921x_eth_data.phy_mask = phy_mask;
#ifdef CONFIG_GPIO_ETH_ACTIVITY_LED
	ns9xxx_device_ns921x_eth_data.activityled = gpio[0];
#endif

	if (clk_register(&eth_clk.clk))
		return;

	platform_device_register(&ns9xxx_device_ns921x_eth);
}
#else
void __init ns9xxx_add_device_ns921x_eth(struct clk *phyclk, u32 phy_mask,
		int gpio[], int func[], int dir[], int num) {}
#endif

/* I2C controller */
#if defined(CONFIG_I2C_NS9XXX) || defined(CONFIG_I2C_NS9XXX_MODULE)
static struct ns921x_sysclk i2c_clk = {
	.clk = {
		.name		= "i2c-ns9xxx",
		.id		= -1,
		.owner		= THIS_MODULE,
		.endisable	= ns921x_endisable_sysclock,
	},
	.mask	= SYS_CLOCK_I2C,
};

static struct resource i2c_resources[] = {
	{
		.start	= 0x90050000,
		.end	= 0x9005000f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS921X_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device ns9xxx_device_ns921x_i2c = {
	.name		= "i2c-ns9xxx",
	.id		= -1,
	.resource	= i2c_resources,
	.num_resources	= ARRAY_SIZE(i2c_resources),
};

void __init ns9xxx_add_device_ns921x_i2c(struct plat_ns9xxx_i2c *i2c_data)
{
	i2c_clk.clk.parent = clk_get(NULL, "systemclock");
	if (IS_ERR(i2c_clk.clk.parent))
		return;

	if (clk_register(&i2c_clk.clk))
		return;

	ns9xxx_device_ns921x_i2c.dev.platform_data = i2c_data;
	platform_device_register(&ns9xxx_device_ns921x_i2c);
}
#else
void __init ns9xxx_add_device_ns921x_i2c(struct plat_ns9xxx_i2c *i2c_data) {}
#endif

/* UARTs */
#if defined(CONFIG_SERIAL_NS921X) || defined(CONFIG_SERIAL_NS921X_MODULE)
static struct ns921x_sysclk uarta_clk = {
	.clk = {
		.name		= "ns921x-serial",
		.id		= 0,
		.owner		= THIS_MODULE,
		.rate		= NS921X_REFCLOCK,
		.endisable	= ns921x_endisable_sysclock,
	},
	.mask	= SYS_CLOCK_UARTA,
};

static struct ns921x_sysclk uartb_clk = {
	.clk = {
		.name		= "ns921x-serial",
		.id		= 1,
		.owner		= THIS_MODULE,
		.rate		= NS921X_REFCLOCK,
		.endisable	= ns921x_endisable_sysclock,
	},
	.mask	= SYS_CLOCK_UARTB,
};

static struct ns921x_sysclk uartc_clk = {
	.clk = {
		.name		= "ns921x-serial",
		.id		= 2,
		.owner		= THIS_MODULE,
		.rate		= NS921X_REFCLOCK,
		.endisable	= ns921x_endisable_sysclock,
	},
	.mask	= SYS_CLOCK_UARTC,
};

static struct ns921x_sysclk uartd_clk = {
	.clk = {
		.name		= "ns921x-serial",
		.id		= 3,
		.owner		= THIS_MODULE,
		.rate		= NS921X_REFCLOCK,
		.endisable	= ns921x_endisable_sysclock,
	},
	.mask	= SYS_CLOCK_UARTD,
};

static struct resource uarta_resources[] = {
	{
		.start	= 0x90010000,
		.end	= 0x90017fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS921X_UARTA,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct resource uartb_resources[] = {
	{
		.start	= 0x90018000,
		.end	= 0x9001ffff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS921X_UARTB,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct resource uartc_resources[] = {
	{
		.start	= 0x90020000,
		.end	= 0x90027fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS921X_UARTC,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct resource uartd_resources[] = {
	{
		.start	= 0x90028000,
		.end	= 0x9002ffff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS921X_UARTD,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct ns921x_uart_data uarta_data;
static struct ns921x_uart_data uartb_data;
static struct ns921x_uart_data uartc_data;
static struct ns921x_uart_data uartd_data;

static struct platform_device ns9xxx_device_ns921x_uarta = {
	.name		= "ns921x-serial",
	.id		= 0,
	.dev            = {
		.platform_data = &uarta_data,
	},
	.resource	= uarta_resources,
	.num_resources	= ARRAY_SIZE(uarta_resources),
};

static struct platform_device ns9xxx_device_ns921x_uartb = {
	.name		= "ns921x-serial",
	.id		= 1,
	.dev            = {
		.platform_data = &uartb_data,
	},
	.resource	= uartb_resources,
	.num_resources	= ARRAY_SIZE(uartb_resources),
};

static struct platform_device ns9xxx_device_ns921x_uartc = {
	.name		= "ns921x-serial",
	.id		= 2,
	.dev            = {
		.platform_data = &uartc_data,
	},
	.resource	= uartc_resources,
	.num_resources	= ARRAY_SIZE(uartc_resources),
};

static struct platform_device ns9xxx_device_ns921x_uartd = {
	.name		= "ns921x-serial",
	.id		= 3,
	.dev            = {
		.platform_data = &uartd_data,
	},
	.resource	= uartd_resources,
	.num_resources	= ARRAY_SIZE(uartd_resources),
};

int __init uart_register_gpios(int gpio_start,
				int gpio_nr,
				int func,
				struct ns921x_uart_data *data)
{
	int i;
	int gpio[] = { 3, 7, 1, 5, 0, 2, 4, 6 };

	for (i = 0; i < gpio_nr; i++) {
		if (i!=2 || !data->rtsen) /* Skip request of CTS line */
		{
			int invert = 0;

			if (gpio_request(gpio_start + gpio[i], "ns921x-serial"))
				goto err;
			if (i==3) {
				/* RTS polarity defined by configuration for RS485 mode */
				invert = data->rtsinvert;
				printk("Inverting RTS\n");
#ifdef CONFIG_NS921X_SERIAL_RTS_AS_GPIO
				/* RTS needs to be used as gpio to allow using AFE */
				func = 3;
#endif
			}
			gpio_configure_ns921x(gpio_start + gpio[i], 0, invert, func, 0);
		}
		data->gpios[i] = gpio_start + gpio[i];
	}

	return 0;
err:
	for (; i >= 0; i--)
		gpio_free(gpio_start + gpio[i]);

	return -EBUSY;
}

void __init ns9xxx_add_device_ns921x_uarta(int gpio_start,
		int gpio_nr, int func)
{
	uarta_data.nr_gpios = gpio_nr;
#if defined(CONFIG_CC9P9215JS_SERIAL_PORTA_RXTX485) || \
	defined(CONFIG_CME9210JS_SERIAL_PORTA_RXTX485)
	uarta_data.rtsen = 1;
#if defined(CONFIG_CC9P9215JS_SERIAL_PORTA_RTS485POLHIGH) || \
	defined(CONFIG_CME9210JS_SERIAL_PORTA_RTS485POLHIGH)
	uarta_data.rtsinvert = 1;
#endif
#else
	uarta_data.rtsen = 0;
#endif
	if (uart_register_gpios(gpio_start, gpio_nr, func, &uarta_data))
		return;

	if (clk_register(&uarta_clk.clk))
		return;

	platform_device_register(&ns9xxx_device_ns921x_uarta);
}

void __init ns9xxx_add_device_ns921x_uartb(int gpio_start,
		int gpio_nr, int func)
{
	uartb_data.nr_gpios = gpio_nr;
#ifdef CONFIG_CC9P9215JS_SERIAL_PORTB_RXTX485
	uartb_data.rtsen = 1;
#ifdef CONFIG_CC9P9215JS_SERIAL_PORTB_RTS485POLHIGH
	uartb_data.rtsinvert = 1;
#endif
#else
	uartb_data.rtsen = 0;
#endif

	if (uart_register_gpios(gpio_start, gpio_nr, func, &uartb_data))
		return;

	if (clk_register(&uartb_clk.clk))
		return;

	platform_device_register(&ns9xxx_device_ns921x_uartb);
}

void __init ns9xxx_add_device_ns921x_uartc(int gpio_start,
		int gpio_nr, int func)
{
	uartc_data.nr_gpios = gpio_nr;
#ifdef CONFIG_CC9P9215JS_SERIAL_PORTC_RXTX485
	uartc_data.rtsen = 1;
#ifdef CONFIG_CC9P9215JS_SERIAL_PORTC_RTS485POLHIGH
	uartc_data.rtsinvert = 1;
#endif
#else
	uartc_data.rtsen = 0;
#endif

	if (uart_register_gpios(gpio_start, gpio_nr, func, &uartc_data))
		return;

	if (clk_register(&uartc_clk.clk))
		return;

	platform_device_register(&ns9xxx_device_ns921x_uartc);
}

void __init ns9xxx_add_device_ns921x_uartd(int gpio_start,
		int gpio_nr, int func)
{
	uartd_data.nr_gpios = gpio_nr;
#ifdef CONFIG_CC9P9215JS_SERIAL_PORTD_RXTX485
	uartd_data.rtsen = 1;
#ifdef CONFIG_CC9P9215JS_SERIAL_PORTD_RTS485POLHIGH
	uartd_data.rtsinvert = 1;
#endif
#else
	uartd_data.rtsen = 0;
#endif

	if (uart_register_gpios(gpio_start, gpio_nr, func, &uartd_data))
		return;

	if (clk_register(&uartd_clk.clk))
		return;

	platform_device_register(&ns9xxx_device_ns921x_uartd);
}
#else
void __init ns9xxx_add_device_ns921x_uarta(int gpio_start,
		int gpio_nr, int func) {}
void __init ns9xxx_add_device_ns921x_uartb(int gpio_start,
		int gpio_nr, int func) {}
void __init ns9xxx_add_device_ns921x_uartc(int gpio_start,
		int gpio_nr, int func) {}
void __init ns9xxx_add_device_ns921x_uartd(int gpio_start,
		int gpio_nr, int func) {}
#endif

/* Flash */
#if defined(CONFIG_MTD_PHYSMAP) || defined(CONFIG_MTD_PHYSMAP)
static struct resource flash_resources[] = {
	{
		.start	= 0x50000000,
		.end	= 0x5fffffff,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ns9xxx_device_ns921x_flash = {
	.name		= "physmap-flash",
	.id		= 0,
	.resource	= flash_resources,
	.num_resources	= ARRAY_SIZE(flash_resources),
};

void __init ns9xxx_add_device_ns921x_flash(
		struct physmap_flash_data *flash_data)
{

	ns9xxx_device_ns921x_flash.dev.platform_data = flash_data;
	platform_device_register(&ns9xxx_device_ns921x_flash);
}
#else
void __init ns9xxx_add_device_ns921x_flash(
		struct physmap_flash_data *flash_data) {}
#endif

/* SPI port */
#if defined(CONFIG_SPI_NS921X) || defined(CONFIG_SPI_NS921X_MODULE)
int __init spi_register_gpios(struct spi_ns9xxx_data *data)
{
	int i;

	for (i = 0; i < data->nr_gpios; i++) {
		if (gpio_request(data->gpios[i], "spi_ns921x"))
			goto err;
		gpio_configure_ns921x(data->gpios[i],
					0,
					0,
					data->gpio_funcs[i],
					0 );
	}

	return 0;
err:
	for (; i >= 0; i--)
		gpio_free(data->gpios[i]);

	return -EBUSY;
}

static struct ns921x_sysclk spi_clk = {
	.clk = {
		.name		= "spi_ns921x",
		.id		= 1,
		.owner		= THIS_MODULE,
		.endisable	= ns921x_endisable_sysclock,
	},
	.mask	= SYS_CLOCK_SPI,
};

static struct resource spi_resources[] = {
	{
		.start	= 0x90030000,
		.end	= 0x90037fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS921X_SPI,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 spi_dmamask = DMA_BIT_MASK(32);

static struct platform_device ns9xxx_device_ns921x_spi = {
	.name		= "spi_ns921x",
	.id		= 1,
	.resource	= spi_resources,
	.num_resources	= ARRAY_SIZE(spi_resources),
	.dev = {
		.dma_mask = &spi_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

void __init ns9xxx_add_device_ns921x_spi(struct spi_ns9xxx_data *data)
{
	if (spi_register_gpios(data))
		return;

	spi_clk.clk.parent = clk_get(NULL, "systemclock");
	if (IS_ERR(spi_clk.clk.parent))
		return;

	if (clk_register(&spi_clk.clk))
		return;

	ns9xxx_device_ns921x_spi.dev.platform_data = data;
	platform_device_register(&ns9xxx_device_ns921x_spi);
}
#else
void __init ns9xxx_add_device_ns921x_spi(struct spi_ns9xxx_data *data) {}
#endif

/* AES HW Encryption module */
#if defined(CONFIG_CRYPTO_DEV_NS921X_AES) || \
	defined(CONFIG_CRYPTO_DEV_NS921X_AES_MODULE)
static struct ns921x_sysclk aes_clk = {
	.clk = {
		.name		= "ns921x-aes",
		.id		= -1,
		.owner		= THIS_MODULE,
		.endisable	= ns921x_endisable_sysclock,
	},
	.mask	= SYS_CLOCK_AES,
};

static struct resource aes_resources[] = {
	{
		.start	= 0xa0800000,
		.end	= 0xa080000f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS921X_EXTDMA,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 aes_dmamask = DMA_BIT_MASK(32);

static struct platform_device ns9xxx_device_ns921x_aes = {
	.name		= "ns921x-aes",
	.id		= 1,
	.resource	= aes_resources,
	.num_resources	= ARRAY_SIZE(aes_resources),
	.dev = {
		.dma_mask = &aes_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

void __init ns9xxx_add_device_ns921x_aes(void)
{
	aes_clk.clk.parent = clk_get(NULL, "dmaclock");
	if (IS_ERR(aes_clk.clk.parent))
		return;

	if (clk_register(&aes_clk.clk))
		return;

	platform_device_register(&ns9xxx_device_ns921x_aes);
}
#else
void __init ns9xxx_add_device_ns921x_aes(void);
#endif


#if defined(CONFIG_PM)
static irqreturn_t ns921x_ack_extirq(int irq, void *dev_id)
{
	u32 eixctl;

	BUG_ON((unsigned)(irq - IRQ_NS9XXX_EXT0) > 4);

	eixctl = __raw_readl(SYS_EIxCTRL(irq - IRQ_NS9XXX_EXT0));

	/* ack ext irq */
	REGSETIM(eixctl, SYS_EIxCTRL, CLEAR, 1);
	__raw_writel(eixctl, SYS_EIxCTRL(irq - IRQ_NS9XXX_EXT0));
	REGSETIM(eixctl, SYS_EIxCTRL, CLEAR, 0);
	__raw_writel(eixctl, SYS_EIxCTRL(irq - IRQ_NS9XXX_EXT0));

	return IRQ_HANDLED;
}

int __init ns921x_extgpio_pm_wakeup_init(unsigned int gpio)
{
	int ret;
	const struct gpio_to_irq_map *map;

	map = gpio_get_map_ns921x(gpio);
	if (!map) {
		pr_debug("%s: selected gpio %d not irq capable\n", __func__, gpio);
		return -EINVAL;
	}

	ret = gpio_request(gpio, "cpuwake");
	if (ret) {
		pr_debug("%s: err_gpio_request -> %d\n", __func__, ret);
		goto err_gpio_request;
	}

	gpio_configure_ns921x(gpio, 0, 0, map->func, 0);

	ret = request_irq(map->irq, ns921x_ack_extirq, IRQF_TRIGGER_FALLING,
			  "extwakeirq", NULL);
	if (ret) {
		pr_debug("%s: err_request_irq_extwakeirq %d -> %d\n", __func__, map->irq, ret);
		goto err_irq_request;
	}

	ret = enable_irq_wake(map->irq);
	if (ret) {
		pr_debug("%s: err_enable_irq_extwakeirq %d -> %d\n", __func__, map->irq, ret);
		goto err_enable_wake;
	}
	return 0;

err_enable_wake:
	free_irq(map->irq, NULL);
err_irq_request:
	gpio_free(gpio);
err_gpio_request:
	return ret;
}
#else /* CONFIG_PM */
int __init ns921x_extgpio_pm_wakeup_init(unsigned int gpio)
{
	return 0;
}
#endif /* CONFIG_PM */

#if defined(CONFIG_FIM_ZERO_SDIO)
void __init ns9xxx_add_device_ns921x_fim_sdio0(void)
{
	extern struct platform_device ns921x_fim_sdio0;
	platform_device_register(&ns921x_fim_sdio0);
}
#else
void __init ns9xxx_add_device_ns921x_fim_sdio0(void) {}
#endif

#if defined(CONFIG_FIM_ONE_SDIO) && !defined(CONFIG_MACH_CWME9210JS) && !defined(CONFIG_MACH_CWME9210)
void __init ns9xxx_add_device_ns921x_fim_sdio1(void)
{
	extern struct platform_device ns921x_fim_sdio1;
	platform_device_register(&ns921x_fim_sdio1);
}
#else
void __init ns9xxx_add_device_ns921x_fim_sdio1(void) {}
#endif


#if defined(CONFIG_FIM_ZERO_SPI)
void __init ns9xxx_add_device_ns921x_fim_spi0(void)
{
    extern struct platform_device ns921x_fim_spi0;
    platform_device_register(&ns921x_fim_spi0);
}
#else
void __init ns9xxx_add_device_ns921x_fim_spi0(void) {}
#endif

#if defined(CONFIG_FIM_ONE_SPI)
void __init ns9xxx_add_device_ns921x_fim_spi1(void)
{
    extern struct platform_device ns921x_fim_spi1;
    platform_device_register(&ns921x_fim_spi1);
}
#else
void __init ns9xxx_add_device_ns921x_fim_spi1(void) {}
#endif

#if defined(CONFIG_FIM_ZERO_SERIAL)
void __init ns9xxx_add_device_ns921x_fim_serial0(void)
{
	extern struct platform_device ns921x_fim_serial0;
	platform_device_register(&ns921x_fim_serial0);
}
#else
void __init ns9xxx_add_device_ns921x_fim_serial0(void) {}
#endif /* CONFIG_FIM_SERIAL */

#if defined(CONFIG_FIM_ONE_SERIAL)
void __init ns9xxx_add_device_ns921x_fim_serial1(void)
{
	extern struct platform_device ns921x_fim_serial1;
	platform_device_register(&ns921x_fim_serial1);
}
#else
void __init ns9xxx_add_device_ns921x_fim_serial1(void) {}
#endif /* CONFIG_FIM_SERIAL */

#if defined(CONFIG_FIM_ZERO_CAN)
void __init ns9xxx_add_device_ns921x_fim_can0(void)
{
	extern struct platform_device ns921x_fim_can0;
	platform_device_register(&ns921x_fim_can0);
}
#else
void __init ns9xxx_add_device_ns921x_fim_can0(void) {}
#endif

#if defined(CONFIG_FIM_ONE_CAN)
void __init ns9xxx_add_device_ns921x_fim_can1(void)
{
	extern struct platform_device ns921x_fim_can1;
	platform_device_register(&ns921x_fim_can1);
}
#else
void __init ns9xxx_add_device_ns921x_fim_can1(void) {}
#endif

#if defined(CONFIG_FIM_ZERO_W1)
static void __init ns9xxx_add_device_ns921x_fim0_w1(void)
{
	extern struct platform_device ns921x_fim0_w1;
	platform_device_register(&ns921x_fim0_w1);
}
#else
static inline void __init ns9xxx_add_device_ns921x_fim0_w1(void) { }
#endif

#if defined(CONFIG_FIM_ONE_W1)
static void __init ns9xxx_add_device_ns921x_fim1_w1(void)
{
	extern struct platform_device ns921x_fim1_w1;
	platform_device_register(&ns921x_fim1_w1);
}
#else
static inline void __init ns9xxx_add_device_ns921x_fim1_w1(void) { }
#endif

#if defined(CONFIG_FIM_ZERO_USB)
void __init ns9xxx_add_device_ns921x_fim_usb0(void)
{
	extern struct platform_device ns921x_fim_usb0;
	platform_device_register(&ns921x_fim_usb0);
}
#else
static inline void __init ns9xxx_add_device_ns921x_fim_usb0(void) {}
#endif

#if defined(CONFIG_FIM_ONE_USB)
void __init ns9xxx_add_device_ns921x_fim_usb1(void)
{
	extern struct platform_device ns921x_fim_usb1;
	platform_device_register(&ns921x_fim_usb1);
}
#else
static inline void __init ns9xxx_add_device_ns921x_fim_usb1(void) {}
#endif

#if defined(CONFIG_FIM_CORE) || defined(CONFIG_FIM_MODULES)
void __init ns9xxx_add_device_ns921x_fims(void)
{
	/*
	 * On the CME9210s with the CAN-support the following GPIOs are inter-connected:
	 *  - GPIO15 (CAN_TX) with GPIO6
	 *  - GPIO14 (CAN_RX) with GPIO2
	 * If the CAN is to be used, GPIO2 and GPIO6 must be tristated before CAN TX and RX
	 * can be made active.
	 */
#if defined(CONFIG_FIM_CORE) && \
		(defined(CONFIG_MACH_CME9210) || defined(CONFIG_MACH_CME9210JS) || \
		 defined(CONFIG_MACH_CWME9210) || defined(CONFIG_MACH_CWME9210JS))
	enum cme9210_variant variant;

	variant = get_cme9210_variant();
	if (CME9210_NEW_8M_FLASH == variant ||
	    CME9210_NEW_4M_FLASH == variant ||
	    CME9210_NEW_2M_FLASH == variant) {
		gpio_direction_input_ns921x_unlocked(6);
		gpio_direction_input_ns921x_unlocked(2);
	}
#endif

	/* FIM 0 */
	ns9xxx_add_device_ns921x_fim_serial0();
	ns9xxx_add_device_ns921x_fim_sdio0();
	ns9xxx_add_device_ns921x_fim_can0();
	ns9xxx_add_device_ns921x_fim0_w1();
	ns9xxx_add_device_ns921x_fim_usb0();
	ns9xxx_add_device_ns921x_fim_spi0();

        /* FIM 1 */
	ns9xxx_add_device_ns921x_fim_serial1();
	ns9xxx_add_device_ns921x_fim_sdio1();
	ns9xxx_add_device_ns921x_fim_can1();
	ns9xxx_add_device_ns921x_fim1_w1();
	ns9xxx_add_device_ns921x_fim_usb1();
	ns9xxx_add_device_ns921x_fim_spi1();
}
#else
void __init ns9xxx_add_device_ns921x_fims(void) {}
#endif /* CONFIG_FIM_CORE */


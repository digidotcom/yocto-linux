/*
 * arch/arm/mach-ns9xxx/ns9360_devices.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/ns9xxx-eth.h>
#include <linux/platform_device.h>

#include <mach/regs-lcd-ns9360.h>
#include <mach/regs-sys-ns9360.h>
#include <mach/gpio.h>
#include <mach/ns9360fb.h>
#include <mach/display/displays.h>
#include <mach/irqs.h>

#include "clock.h"
#include "ns9360_devices.h"


/* USB host */
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static int usbh_endisable(struct clk *clk, int enable)
{
	if (enable) {
		u32 mrst = __raw_readl(NS9360_BBU_MSR);
		mrst |= 1 << 11;
		__raw_writel(mrst, NS9360_BBU_MSR);

		__raw_writel(0 << 2, NS9360_BBUS_USB);

		mrst = __raw_readl(NS9360_BBU_MSR);
		mrst &= ~(1 << 11);
		__raw_writel(mrst, NS9360_BBU_MSR);
	}

	return 0;
}

static struct clk usbh_clk = {
	.name		= "ns9360-ohci",
	.id		= -1,
	.owner		= THIS_MODULE,
	.endisable	= usbh_endisable,
};

static u64 ohci_dmamask = DMA_BIT_MASK(32);
static struct resource usbh_resources[] = {
	{
		.start	= 0x90801000,
		.end	= 0x90801fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 0x90800000,
		.end	= 0x90800fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS9360_USBHOST,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device ns9xxx_device_ns9360_usbh = {
	.name		= "ns9360-ohci",
	.id		= -1,
	.resource	= usbh_resources,
	.num_resources	= ARRAY_SIZE(usbh_resources),
	.dev = {
		.dma_mask		= &ohci_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};

void __init ns9xxx_add_device_ns9360_usbh(void)
{
	if (clk_register(&usbh_clk))
		return;

	platform_device_register(&ns9xxx_device_ns9360_usbh);
}
#else
void __init ns9xxx_add_device_ns9360_usbh(void) {}
#endif

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

static struct platform_device ns9xxx_device_ns9360_wdt = {
	.name		= "ns9xxx-wdt",
	.id		= -1,
	.resource	= wdt_resources,
	.num_resources	= ARRAY_SIZE(wdt_resources),
};

void __init ns9xxx_add_device_ns9360_wdt(void)
{
	wdt_clk.parent = clk_get(NULL, "cpuclock");
	if (IS_ERR(wdt_clk.parent))
		return;

	if (clk_register(&wdt_clk))
		return;

	platform_device_register(&ns9xxx_device_ns9360_wdt);
}
#else
void __init ns9xxx_add_device_ns9360_wdt(void) {}
#endif

/* RTC */
#if defined(CONFIG_RTC_DRV_NS9XXX) || defined(CONFIG_RTC_DRV_NS9XXX_MODULE)
static struct clk rtc_clk = {
	.name	= "rtc-ns9xxx",
	.id	= -1,
	.owner	= THIS_MODULE,
};

static struct resource rtc_resources[] = {
	{
		.start	= 0x90700000,
		.end	= 0x907000ff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS9360_RTC,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device ns9xxx_device_ns9360_rtc = {
	.name		= "rtc-ns9xxx",
	.id		= 0,
	.resource	= rtc_resources,
	.num_resources	= ARRAY_SIZE(rtc_resources),
};

void __init ns9xxx_add_device_ns9360_rtc(void)
{
	struct clk *sysclk = clk_get(NULL, "systemclock");
	if (IS_ERR(sysclk))
		return;

	__raw_writel(clk_get_rate(sysclk) / 200, SYS_RTCCC);
	clk_put(sysclk);

	if (clk_register(&rtc_clk))
		return;

	platform_device_register(&ns9xxx_device_ns9360_rtc);
}
#else
void __init ns9xxx_add_device_ns9360_rtc(void) {}
#endif

/* Ethernet */
#if defined(CONFIG_NS9XXX_ETH) || defined(CONFIG_NS9XXX_ETH_MODULE)
int __init eth_register_gpios(int gpio[], int gpio_nr, int func)
{
	int i;

	for (i = 0; i < gpio_nr; i++) {
		if (gpio_request(gpio[i], "ns9xxx-eth"))
			goto err;
		gpio_configure_ns9360(gpio[i], 0, 0, func);
	}

	return 0;
err:
	for (; i >= 0; i--)
		gpio_free(gpio[i]);

	return -EBUSY;
}

static struct clk eth_clk = {
	.name	= "ns9xxx-eth",
	.id	= -1,
	.owner	= THIS_MODULE,
};

static struct plat_ns9xxx_eth ns9xxx_device_ns9360_eth_data = {
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

static struct platform_device ns9xxx_device_ns9360_eth = {
	.name		= "ns9xxx-eth",
	.id		= -1,
	.resource	= eth_resources,
	.num_resources	= ARRAY_SIZE(eth_resources),
	.dev = {
		.platform_data	= &ns9xxx_device_ns9360_eth_data,
	},
};

void __init ns9xxx_add_device_ns9360_eth(int gpio[], int gpio_nr, int func, u32 phy_mask)
{
	if (eth_register_gpios(gpio, gpio_nr, func))
		return;

	if (clk_register(&eth_clk))
		return;

	ns9xxx_device_ns9360_eth_data.phy_mask = phy_mask;

	platform_device_register(&ns9xxx_device_ns9360_eth);
}
#else
void __init ns9xxx_add_device_ns9360_eth(int gpio[], int gpio_nr, int func, u32 phy_mask) {}
#endif

/* UARTs */
#if defined(CONFIG_SERIAL_NS9360) || defined(CONFIG_SERIAL_NS9360_MODULE)
static struct clk uarta_clk = {
	.name	= "ns9360-serial",
	.id	= 1,
	.owner	= THIS_MODULE,
};

static struct clk uartb_clk = {
	.name	= "ns9360-serial",
	.id	= 0,
	.owner	= THIS_MODULE,
};

static struct clk uartc_clk = {
	.name	= "ns9360-serial",
	.id	= 2,
	.owner	= THIS_MODULE,
};

static struct clk uartd_clk = {
	.name	= "ns9360-serial",
	.id	= 3,
	.owner	= THIS_MODULE,
};

static struct resource uarta_resources[] = {
	{
		.start	= 0x90200040,
		.end	= 0x9020007f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS9360_SERARX,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct resource uartb_resources[] = {
	{
		.start	= 0x90200000,
		.end	= 0x9020003f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS9360_SERBRX,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct resource uartc_resources[] = {
	{
		.start	= 0x90300000,
		.end	= 0x9030003f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS9360_SERCRX,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct resource uartd_resources[] = {
	{
		.start	= 0x90300040,
		.end	= 0x9030007f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS9360_BBUS_SERDRX,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device ns9xxx_device_ns9360_uarta = {
	.name		= "ns9360-serial",
	.id		= 1,
	.resource	= uarta_resources,
	.num_resources	= ARRAY_SIZE(uarta_resources),
};

static struct platform_device ns9xxx_device_ns9360_uartb = {
	.name		= "ns9360-serial",
	.id		= 0,
	.resource	= uartb_resources,
	.num_resources	= ARRAY_SIZE(uartb_resources),
};

static struct platform_device ns9xxx_device_ns9360_uartc = {
	.name		= "ns9360-serial",
	.id		= 2,
	.resource	= uartc_resources,
	.num_resources	= ARRAY_SIZE(uartc_resources),
};

static struct platform_device ns9xxx_device_ns9360_uartd = {
	.name		= "ns9360-serial",
	.id		= 3,
	.resource	= uartd_resources,
	.num_resources	= ARRAY_SIZE(uartd_resources),
};

int __init uart_register_gpios(int gpio[], int gpio_nr, int func)
{
	int i;

	for (i = 0; i < gpio_nr; i++) {
		if (gpio_request(gpio[i], "ns9360-serial"))
			goto err;
		gpio_configure_ns9360(gpio[i], 0, 0, func);
	}

	return 0;
err:
	for (; i >= 0; i--)
		gpio_free(gpio[i]);

	return -EBUSY;
}

void __init ns9xxx_add_device_ns9360_uarta(int gpio[], int gpio_nr, int func)
{
	if (uart_register_gpios(gpio, gpio_nr, func))
		return;

	uarta_clk.parent = clk_get(NULL, "bbusclock");
	if (IS_ERR(uarta_clk.parent))
		return;

	if (clk_register(&uarta_clk))
		return;

	platform_device_register(&ns9xxx_device_ns9360_uarta);
}

void __init ns9xxx_add_device_ns9360_uartb(int gpio[], int gpio_nr, int func)
{
	if (uart_register_gpios(gpio, gpio_nr, func))
		return;

	uartb_clk.parent = clk_get(NULL, "bbusclock");
	if (IS_ERR(uartb_clk.parent))
		return;

	if (clk_register(&uartb_clk))
		return;

	platform_device_register(&ns9xxx_device_ns9360_uartb);
}

void __init ns9xxx_add_device_ns9360_uartc(int gpio[], int gpio_nr, int func)
{
	if (uart_register_gpios(gpio, gpio_nr, func))
		return;

	uartc_clk.parent = clk_get(NULL, "bbusclock");
	if (IS_ERR(uartc_clk.parent))
		return;

	if (clk_register(&uartc_clk))
		return;

	platform_device_register(&ns9xxx_device_ns9360_uartc);
}

void __init ns9xxx_add_device_ns9360_uartd(int gpio[], int gpio_nr, int func)
{
	if (uart_register_gpios(gpio, gpio_nr, func))
		return;

	uartd_clk.parent = clk_get(NULL, "bbusclock");
	if (IS_ERR(uartd_clk.parent))
		return;

	if (clk_register(&uartd_clk))
		return;

	platform_device_register(&ns9xxx_device_ns9360_uartd);
}
#else
void __init ns9xxx_add_device_ns9360_uarta(int gpio[], int gpio_nr, int func) {}
void __init ns9xxx_add_device_ns9360_uartb(int gpio[], int gpio_nr, int func) {}
void __init ns9xxx_add_device_ns9360_uartc(int gpio[], int gpio_nr, int func) {}
void __init ns9xxx_add_device_ns9360_uartd(int gpio[], int gpio_nr, int func) {}
#endif

/* NAND flash */
#if defined(CONFIG_MTD_NAND_CCX9X) || defined(CONFIG_MTD_NAND_CCX9X_MODULE)
static struct resource nand_resources[] = {
	{
		.start	= 0x50000000,
		.end	= 0x5fffffff,
		.flags	= IORESOURCE_MEM,
	}
};

static struct platform_device ns9xxx_device_ns9360_nand = {
	.name		= "ccx9x_nand",
	.id		= -1,
	.resource	= nand_resources,
	.num_resources	= ARRAY_SIZE(nand_resources),
};

void __init ns9xxx_add_device_ns9360_nand(struct ccx9x_nand_info *nand_data)
{
	ns9xxx_device_ns9360_nand.dev.platform_data = nand_data;
	platform_device_register(&ns9xxx_device_ns9360_nand);
}
#else
void __init ns9xxx_add_device_ns9360_nand(struct ccx9x_nand_info *nand_data) {}
#endif

/* I2C controller */
#if defined(CONFIG_I2C_NS9XXX) || defined(CONFIG_I2C_NS9XXX_MODULE)
static struct clk i2c_clk = {
	.name	= "i2c-ns9xxx",
	.id	= -1,
	.owner	= THIS_MODULE,
};

static struct resource i2c_resources[] = {
	{
		.start	= 0x90500000,
		.end	= 0x9050000f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_NS9360_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device ns9xxx_device_ns9360_i2c = {
	.name		= "i2c-ns9xxx",
	.id		= -1,
	.resource	= i2c_resources,
	.num_resources	= ARRAY_SIZE(i2c_resources),
};

void __init ns9xxx_add_device_ns9360_i2c(struct plat_ns9xxx_i2c *i2c_data)
{
	i2c_clk.parent = clk_get(NULL, "cpuclock");
	if (IS_ERR(i2c_clk.parent))
		return;

	if (clk_register(&i2c_clk))
		return;

	ns9xxx_device_ns9360_i2c.dev.platform_data = i2c_data;
	platform_device_register(&ns9xxx_device_ns9360_i2c);
}
#else
void __init ns9xxx_add_device_ns9360_i2c(struct plat_ns9xxx_i2c *i2c_data) {}
#endif

/* Framebuffer controller */
#if (defined(CONFIG_FB_NS9360) || defined(CONFIG_FB_NS9360_MODULE)) && !defined(NS9XXX_NODISPLAY)
int __init fb_register_gpios(int gpio[], int gpio_nr, int func[], int power)
{
	int i;

	for (i = 0; i < gpio_nr; i++) {
		if (gpio_request(gpio[i], "ns9360fb"))
			goto err;
		gpio_configure_ns9360(gpio[i], 0, 0, func[i]);
	}

	if (gpio_request(power, "ns9360fb")) {
		i--;
		goto err;
	}
	gpio_direction_output(power, 0);

	return 0;
err:
	for (; i >= 0; i--)
		gpio_free(gpio[i]);

	return -EBUSY;
}

static int fb_endisable(struct clk *clk, int enable)
{
	/* Currently, nothing to do */
	return 0;
}

static struct clk fb_clk = {
	.name		= "ns9360fb",
	.id		= -1,
	.owner		= THIS_MODULE,
	.endisable	= fb_endisable,

};
static struct ns9360fb_pdata ns9xxx_device_ns9360_fb_data = {
 	.displays	= display_list,
 	.num_displays	= ARRAY_SIZE(display_list),
};

static struct resource fb_resources[] = {
	{
		.start	= 0xa0800000,
		.end	= 0xa080002f,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ns9xxx_device_ns9360_fb = {
	.name		= "ns9360fb",
	.id		= -1,
	.resource	= fb_resources,
	.num_resources	= ARRAY_SIZE(fb_resources),
	.dev = {
		.platform_data	= &ns9xxx_device_ns9360_fb_data,
	},
};

void __init ns9xxx_add_device_ns9360_fb(int gpio[], int gpio_nr,
		int func[], int power)
{
	fb_clk.parent = clk_get(NULL, "ahbclock");
	if (IS_ERR(fb_clk.parent))
		return;

	if (clk_register(&fb_clk))
		return;

	if (fb_register_gpios(gpio, gpio_nr, func, power))
		return;

	platform_device_register(&ns9xxx_device_ns9360_fb);
}
#else
void __init ns9xxx_add_device_ns9360_fb(int gpio[], int gpio_nr,
		int func[], int power) {}
#endif

#if defined(CONFIG_SPI_NS9360) || defined(CONFIG_SPI_NS9360_MODULE)
int __init spi_register_gpios(struct spi_ns9xxx_data *data)
{
	int i;

	for (i = 0; i < data->nr_gpios; i++) {
		if (gpio_request(data->gpios[i], "spi_ns9360"))
			goto err;
		gpio_configure_ns9360( data->gpios[i],
					0,
					0,
					data->gpio_funcs[i] );
	}

	return 0;
err:
	for (; i >= 0; i--)
		gpio_free(data->gpios[i]);

	return -EBUSY;
}

static struct clk spi_porta_clk = {
	.name	= "spi_ns9360",
	.id	= 1,
	.owner	= THIS_MODULE,
};

static struct clk spi_portb_clk = {
	.name	= "spi_ns9360",
	.id	= 0,
	.owner	= THIS_MODULE,
};

static struct clk spi_portc_clk = {
	.name	= "spi_ns9360",
	.id	= 2,
	.owner	= THIS_MODULE,
};

static struct clk spi_portd_clk = {
	.name	= "spi_ns9360",
	.id	= 3,
	.owner	= THIS_MODULE,
};

static struct resource spi_porta_resources[] = {
	{
		.start	= 0x90200040,
		.end	= 0x9020007f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 0x90000040,
		.end	= 0x9000007f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 60,
		.flags	= IORESOURCE_IRQ,
	}, {
		.start	= 61,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource spi_portb_resources[] = {
	{
		.start	= 0x90200000,
		.end	= 0x9020003f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 0x90000000,
		.end	= 0x9000003f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 58,
		.flags	= IORESOURCE_IRQ,
	}, {
		.start	= 59,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource spi_portc_resources[] = {
	{
		.start	= 0x90300000,
		.end	= 0x9020003f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 0x90000080,
		.end	= 0x900000bf,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 62,
		.flags	= IORESOURCE_IRQ,
	}, {
		.start	= 63,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource spi_portd_resources[] = {
	{
		.start	= 0x90300040,
		.end	= 0x9030007f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 0x900000c0,
		.end	= 0x900000ff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 64,
		.flags	= IORESOURCE_IRQ,
	}, {
		.start	= 65,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 spi_dmamask = DMA_BIT_MASK(32);

static struct platform_device ns9xxx_device_ns9360_spi_porta = {
	.name		= "spi_ns9360",
	.id		= 1,
	.resource	= spi_porta_resources,
	.num_resources	= ARRAY_SIZE(spi_porta_resources),
	.dev = {
		.dma_mask = &spi_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct platform_device ns9xxx_device_ns9360_spi_portb = {
	.name		= "spi_ns9360",
	.id		= 0,
	.resource	= spi_portb_resources,
	.num_resources	= ARRAY_SIZE(spi_portb_resources),
	.dev = {
		.dma_mask = &spi_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct platform_device ns9xxx_device_ns9360_spi_portc = {
	.name		= "spi_ns9360",
	.id		= 2,
	.resource	= spi_portc_resources,
	.num_resources	= ARRAY_SIZE(spi_portc_resources),
	.dev = {
		.dma_mask = &spi_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct platform_device ns9xxx_device_ns9360_spi_portd = {
	.name		= "spi_ns9360",
	.id		= 3,
	.resource	= spi_portd_resources,
	.num_resources	= ARRAY_SIZE(spi_portd_resources),
	.dev = {
		.dma_mask = &spi_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

void __init ns9xxx_add_device_ns9360_spi_porta(struct spi_ns9xxx_data *data)
{
	if (spi_register_gpios(data))
		return;

	spi_porta_clk.parent = clk_get(NULL, "bbusclock");
	if (IS_ERR(spi_porta_clk.parent))
		return;

	if (clk_register(&spi_porta_clk))
		return;

	/* Set platform data */
	ns9xxx_device_ns9360_spi_porta.dev.platform_data = data;

	platform_device_register(&ns9xxx_device_ns9360_spi_porta);
}

void __init ns9xxx_add_device_ns9360_spi_portb( struct spi_ns9xxx_data *data )
{
	if (spi_register_gpios(data))
		return;

	spi_portb_clk.parent = clk_get(NULL, "bbusclock");
	if (IS_ERR(spi_portb_clk.parent))
		return;

	if (clk_register(&spi_portb_clk))
		return;

	/* Set platform data */
	ns9xxx_device_ns9360_spi_portb.dev.platform_data = data;

	platform_device_register(&ns9xxx_device_ns9360_spi_portb);
}


void __init ns9xxx_add_device_ns9360_spi_portc(struct spi_ns9xxx_data *data)
{
	if (spi_register_gpios(data))
		return;

	spi_portc_clk.parent = clk_get(NULL, "bbusclock");
	if (IS_ERR(spi_portc_clk.parent))
		return;

	if (clk_register(&spi_portc_clk))
		return;

	/* Set platform data */
	ns9xxx_device_ns9360_spi_portc.dev.platform_data = data;

	platform_device_register(&ns9xxx_device_ns9360_spi_portc);
}

void __init ns9xxx_add_device_ns9360_spi_portd(struct spi_ns9xxx_data *data)
{
	if (spi_register_gpios(data))
		return;

	spi_portd_clk.parent = clk_get(NULL, "bbusclock");
	if (IS_ERR(spi_portd_clk.parent))
		return;

	if (clk_register(&spi_portd_clk))
		return;

	/* Set platform data */
	ns9xxx_device_ns9360_spi_portd.dev.platform_data = data;

	platform_device_register(&ns9xxx_device_ns9360_spi_portd);
}
#else
void __init ns9xxx_add_device_ns9360_spi_porta(struct spi_ns9xxx_data *data) {}
void __init ns9xxx_add_device_ns9360_spi_portb(struct spi_ns9xxx_data *data) {}
void __init ns9xxx_add_device_ns9360_spi_portc(struct spi_ns9xxx_data *data) {}
void __init ns9xxx_add_device_ns9360_spi_portd(struct spi_ns9xxx_data *data) {}
#endif

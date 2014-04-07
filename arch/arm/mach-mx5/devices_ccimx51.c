/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2009 - 2010 Digi International, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/smsc911x.h>
#include <linux/sysfs.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include <mach/i2c.h>
#include <video/ad9389.h>
#include "board-ccimx51.h"
#include "iomux.h"
#include "crm_regs.h"
#include "devices.h"
#include "mx51_pins.h"
#include <linux/smc911x.h>
#include "displays/displays.h"
#include "devices_ccimx5x.h"

#include <mach/mxc_iim.h>
#include <mach/i2c.h>

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/flash.h>
#endif

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
#include <asm/mach/bootldr_shmem.h>
#include <linux/crc32.h>
#endif

#if defined(CONFIG_VIDEO_AD9389) || defined(CONFIG_VIDEO_AD9389_MODULE)
static int debug = 0;
#endif

#define AD9389_DBG		0x0001
#define DBG(flag, fmt, args...)	do {						\
					if (debug & flag)			\
						printk(fmt, ## args);		\
				} while (0)

struct ccimx51_ident {
	const char	*id_string;
	const int	mem[2];
	const int	cpu_freq;
	const int	cpu_model;
};

/**
 * To add new valid variant ID, append new lines in this array and edit the
 * is_valid_hw_id() function, accordingly
 */
struct ccimx51_ident ccimx51_id[] = {
/* 0x00 */	{ "Unknown",						{         0,     0},         0,	0},
/* 0x01 */	{ "Not supported",					{         0,     0},         0,	0},
/* 0x02 */	{ "i.MX515@800MHz, Wireless, PHY, Ext. Eth, Accel",	{0x20000000,   512}, 800000000,	IMX515},
/* 0x03 */	{ "i.MX515@800MHz, PHY, Ext. Eth, Accel",		{0x20000000,   512}, 800000000,	IMX515},
/* 0x04 */	{ "i.MX515@600MHz, Wireless, PHY, Ext. Eth, Accel",	{0x20000000,   512}, 600000000,	IMX515},
/* 0x05 */	{ "i.MX515@600MHz, PHY, Ext. Eth, Accel",		{0x20000000,   512}, 600000000,	IMX515},
/* 0x06 */	{ "i.MX515@800MHz, Wireless, PHY, Accel",		{0x20000000,  2048}, 800000000,	IMX515},
/* 0x07 */	{ "i.MX515@800MHz, PHY, Acceleromter",			{0x20000000,  2048}, 800000000,	IMX515},
/* 0x08 */	{ "i.MX515@800MHz, Wireless, PHY, Accel",		{0x10000000,   256}, 800000000,	IMX515},
/* 0x09 */	{ "i.MX515@800MHz, PHY, Acceleromter",			{0x10000000,   256}, 800000000,	IMX515},
/* 0x0a */	{ "i.MX515@600MHz, Wireless, PHY, Accel",		{0x10000000,   256}, 600000000,	IMX515},
/* 0x0b */	{ "i.MX515@600MHz, PHY, Acceleromter",			{0x10000000,   256}, 600000000,	IMX515},
/* 0x0c */	{ "i.MX515@800MHz, Wireless, PHY, Accel",		{0x08000000,   128}, 800000000,	IMX515},
/* 0x0d */	{ "i.MX512@800MHz",					{0x08000000,   128}, 800000000,	IMX512},
/* 0x0e */	{ "i.MX515@800MHz, Wireless, PHY, Accel",		{0x20000000,   512}, 800000000,	IMX515},
/* 0x0f */	{ "i.MX515@600MHz, PHY, Accel",				{0x08000000,   128}, 600000000,	IMX515},
/* 0x10 */	{ "i.MX515@600MHz, Wireless, PHY, Accel",		{0x08000000,   128}, 600000000,	IMX515},
/* 0x11 */	{ "i.MX515@800MHz, PHY, Accel",				{0x08000000,   128}, 800000000,	IMX515},
/* 0x12 */	{ "i.MX515@600MHz, Wireless, PHY, Accel",		{0x20000000,   512}, 600000000,	IMX515},
/* 0x13 */	{ "i.MX515@800MHz, PHY, Accel",				{0x20000000,   512}, 800000000,	IMX515},
/* 0x14 */	{ "i.MX515@800MHz, Wireless, PHY, Ext. Eth, Accel",	{0x20000000,   512}, 800000000,	IMX515},
/* 0x15 */	{ "i.MX515@600MHz, PHY, Acccel",			{0x20000000,   512}, 600000000,	IMX515},
/* 0x16 */	{ "i.MX515@800MHz, Wireless, PHY",			{0x20000000,   512}, 800000000,	IMX515},
/* 0x17 */	{ "i.MX515@600MHz, PHY, Ext. Eth, Accel",		{0x20000000,  2048}, 600000000,	IMX515},
/* 0x18 */	{ "Reserved for future use",				{         0,     0},         0,	0},
/* 0x19 */	{ "i.MX512@600MHz",					{0x08000000,   128}, 600000000,	IMX512},
/* (55001445-13) 0x1a */ { "i.MX512@800MHz, Wireless, PHY",		{0x20000000,   512}, 800000000,	IMX512},
/* (55001445-14) 0x1b */ { "i.MX512@800MHz, Wireless, PHY, Accel",	{0x08000000,   128}, 800000000,	IMX512},
/* (55001585-13) 0x1c */ { "i.MX512@800MHz, PHY, Accel",		{0x08000000,   128}, 800000000, IMX512},
};

int ccimx5x_get_cpumodel_from_variant( void )
{
	u8 variant = ccimx5x_get_mod_variant();

	if (variant < ARRAY_SIZE(ccimx51_id)) {
		return ccimx51_id[variant].cpu_model;
	}
	return -1;
}

int ccimx5x_get_cpufreq_from_variant( void )
{
	u8 variant = ccimx5x_get_mod_variant();

	if (variant < ARRAY_SIZE(ccimx51_id)) {
		return ccimx51_id[variant].cpu_freq;
	}
	return -1;
}

#if defined(CONFIG_HAS_EARLY_USER_LEDS)
void ccimx51_user_led(int led, int val)
{
	__iomem void *base;
	u32 reg, mask;

	if (led == 1)
		mask = 1 << 10;
	else if (led == 2)
		mask = 1 << 9;
	else
		return;

	base = ioremap(GPIO3_BASE_ADDR, SZ_4K);
	reg = __raw_readl(base);

	if (val)
		reg |= mask;
	else
		reg &= ~mask;

	__raw_writel(reg, base);
	iounmap(base);
}
#endif /* CONFIG_HAS_EARLY_USER_LEDS */

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
static int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	if (to_platform_device(dev)->id == 0)
		rc = 0; /* Not supported WP on JSK board, therefore write is enabled */
	else if (to_platform_device(dev)->id == 1)
#ifdef CCIMX51_SD2_WP
		rc = gpio_get_value(IOMUX_TO_GPIO(CCIMX51_SD2_WP));
#else
		rc = 0;
#endif
	else if (to_platform_device(dev)->id == 2)
		rc = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_NANDF_CS1));
	return rc;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret = 1;

	if (to_platform_device(dev)->id == 0)
#ifdef CONFIG_JSCCIMX51_V1
		ret = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_0));
#else
		ret = 0;
#endif
	else if (to_platform_device(dev)->id == 1)
#ifdef CCIMX51_SD2_CD
		ret = gpio_get_value(IOMUX_TO_GPIO(CCIMX51_SD2_CD));
#else
		ret = 0;
#endif
	else if (to_platform_device(dev)->id == 2)
		ret = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO_NAND));
	return ret;
}

struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 52000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
	    MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

struct mxc_mmc_platform_data mmc3_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
		    MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

void ccimx51_register_sdio(int interface)
{
	switch (interface) {
	case 0:
		mxcsdhc1_device.resource[2].start = CCIMX51_SD1_CD_IRQ;
		mxcsdhc1_device.resource[2].end = CCIMX51_SD1_CD_IRQ;
		mxc_register_device(&mxcsdhc1_device, &mmc1_data);
		break;
	case 1:
		mxcsdhc2_device.resource[2].start = CCIMX51_SD2_CD_IRQ;
		mxcsdhc2_device.resource[2].end = CCIMX51_SD2_CD_IRQ;
		mxc_register_device(&mxcsdhc2_device, &mmc2_data);
		break;
	case 2:
		mxcsdhc3_device.resource[2].start = IOMUX_TO_IRQ(MX51_PIN_GPIO_NAND);
		mxcsdhc3_device.resource[2].end = IOMUX_TO_IRQ(MX51_PIN_GPIO_NAND);
		mxc_register_device(&mxcsdhc3_device, &mmc3_data);
		break;
	}
}
#endif

#if defined(CONFIG_MTD_NAND_MXC) \
	|| defined(CONFIG_MTD_NAND_MXC_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V2) \
	|| defined(CONFIG_MTD_NAND_MXC_V2_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V3) \
	|| defined(CONFIG_MTD_NAND_MXC_V3_MODULE)

extern void gpio_nand_active(void);
extern void gpio_nand_inactive(void);

static int nand_init(void)
{
       /* Configure the pins */
      gpio_nand_active();
      return 0;
}

static void nand_exit(void)
{
      /* Free the pins */
      gpio_nand_inactive();
}

struct flash_platform_data mxc_nand_data = {
	.width = 1,
	.init = nand_init,
	.exit = nand_exit,
};
#endif

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
struct smsc911x_platform_config ccimx51_smsc9118 = {
	.flags          = SMSC911X_USE_32BIT,
	.irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type       = SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,    /* push-pull irq */
};
#endif

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
struct resource mxcfb_resources[2] = {
	{
		.flags = IORESOURCE_MEM,
		.start = 0,
		.end   = 0,
	},
	{
		.flags = IORESOURCE_MEM,
		.start = 0,
		.end   = 0,
	},
};
#endif

static struct i2c_board_info ccimx51_i2c_devices[] __initdata = {
#if defined(CONFIG_INPUT_MMA7455L) || defined(CONFIG_INPUT_MMA7455L_MODULE)
	{
		I2C_BOARD_INFO("mma7455l", 0x1d),
		.irq = IOMUX_TO_IRQ(MX51_PIN_GPIO1_7),
	},
#endif
#if defined(CONFIG_SND_SOC_IMX_CCIMX51_WM8753) || defined(CONFIG_SND_SOC_IMX_CCIMX51_WM8753_MODULE)
	{
		I2C_BOARD_INFO("wm8753", 0x1A),
        },
#endif

#if defined (CONFIG_MXC_CAMERA_MICRON111_1) || defined(CONFIG_MXC_CAMERA_MICRON111_1_MODULE)
	{
		I2C_BOARD_INFO("mt9v111_1", 0xB8>>1),
	},
#endif
#if defined (CONFIG_MXC_CAMERA_MICRON111_2) || defined(CONFIG_MXC_CAMERA_MICRON111_2_MODULE)
	{
		I2C_BOARD_INFO("mt9v111_2", 0x90>>1),
	},
#endif
#if defined (CONFIG_CCIMX5X_FUSION_4_MULTITOUCH)
    {
        I2C_BOARD_INFO("fusion_F04B", 0x05),
        .irq = IOMUX_TO_IRQ(SECOND_TS_IRQ_PIN),
    },
#endif
#if defined (CONFIG_CCIMX5X_FUSION_7_10_MULTITOUCH)
    {
        I2C_BOARD_INFO("fusion", 0x10),
        .irq = IOMUX_TO_IRQ(SECOND_TS_IRQ_PIN),
    },
#endif
};

int __init ccimx51_init_i2c2(void)
{
#if defined (CONFIG_CCIMX5X_FUSION_4_MULTITOUCH) || defined(CONFIG_CCIMX5X_FUSION_7_10_MULTITOUCH)
    ccimx51_fusion_gpio_init();
#endif
	return i2c_register_board_info(1, ccimx51_i2c_devices , ARRAY_SIZE(ccimx51_i2c_devices) );
}

#if defined(CONFIG_SPI_MXC_SELECT1_SS1) && (defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE))
#if defined(CONFIG_CCIMX5X_SECOND_TOUCH)
static int touch_pendown_state(void)
{
	return gpio_get_value(IOMUX_TO_GPIO(SECOND_TS_IRQ_PIN)) ? 0 : 1;
}

static struct ads7846_platform_data ccimx51js_touch_data = {
	.model			= 7843,
	.x_min			= 0,
	.y_min			= 0,
	.x_max			= 4095,
	.y_max			= 4095,
	.get_pendown_state	= touch_pendown_state,
	.buflen			= 10,
	.skip_samples		= 2,
	.rotate			= 0,
};

static struct spi_board_info ccimx51_2nd_touch[] = {
	{
		.modalias	= "ads7846",
		.max_speed_hz	= 500000,
		.irq		= IOMUX_TO_IRQ(SECOND_TS_IRQ_PIN),
		.bus_num        = 1,
		.chip_select    = 3,
		.platform_data	= &ccimx51js_touch_data,
	},
};

void ccimx51_init_2nd_touch(void)
{
	ccimx51_2nd_touch_gpio_init();
	spi_register_board_info(ccimx51_2nd_touch, ARRAY_SIZE(ccimx51_2nd_touch));
}
#else
void ccimx51_init_2nd_touch(void) {}
#endif

static struct spi_board_info spi_devices[] = {
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
#if defined(CONFIG_SPI_MXC_SELECT1) || defined(CONFIG_SPI_MXC_SELECT1_MODULE)
	{       /* SPIDEV */
		.modalias	= "spidev",
		.max_speed_hz	= 6000000,
		.bus_num	= 1,
		.chip_select	= 1,
	},
#endif
#endif
        /* Add here other SPI devices, if any... */
};

void ccimx51_init_spidevices(void)
{
	spi_register_board_info(spi_devices, ARRAY_SIZE(spi_devices));
}
#else
void ccimx51_init_spidevices(void) { }
#endif

extern void ccimx51_gpio_spi_chipselect_active(int cspi_mode, int status,
						    int chipselect);
extern void ccimx51_gpio_spi_chipselect_inactive(int cspi_mode, int status,
						      int chipselect);

struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = ccimx51_gpio_spi_chipselect_active,
	.chipselect_inactive = ccimx51_gpio_spi_chipselect_inactive,
};

struct mxc_spi_master mxcspi2_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = ccimx51_gpio_spi_chipselect_active,
	.chipselect_inactive = ccimx51_gpio_spi_chipselect_inactive,
};

struct mxc_spi_master mxcspi3_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = ccimx51_gpio_spi_chipselect_active,
	.chipselect_inactive = ccimx51_gpio_spi_chipselect_inactive,
};

extern void mx5_ipu_reset(void);
struct mxc_ipu_config mxc_ipu_data = {
	.rev = 2,
	.reset = mx5_ipu_reset,
};

extern void mx5_vpu_reset(void);
struct mxc_vpu_platform_data mxc_vpu_data = {
#ifdef CONFIG_MXC_VPU_IRAM
	.iram_enable = true,
	.iram_size = CONFIG_MXC_VPU_IRAM_SIZE,
#else
	.iram_enable = false,
#endif
	.reset = mx5_vpu_reset,
};

#if defined(CONFIG_W1_MASTER_MXC) || defined(CONFIG_W1_MASTER_MXC_MODULE)
struct mxc_w1_config mxc_w1_data = {
	.search_rom_accelerator = 1,
};
#endif

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)

static struct resource smsc911x_device_resources[] = {
        {
		.name	= "smsc911x-memory",
		.start	= CS5_BASE_ADDR,
		.end	= CS5_BASE_ADDR + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IOMUX_TO_IRQ(MX51_PIN_GPIO1_9),
		.end	= IOMUX_TO_IRQ(MX51_PIN_GPIO1_9),
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device smsc911x_device = {
	.name             = "smsc911x",
	.id               = -1,
	.num_resources    = ARRAY_SIZE(smsc911x_device_resources),
	.resource         = smsc911x_device_resources,
};

/* WEIM registers */
#define CSGCR1	0x00
#define CSGCR2	0x04
#define CSRCR1	0x08
#define CSRCR2	0x0C
#define CSWCR1	0x10

static void ccimx51_init_ext_eth_mac(void)
{
	__iomem void *weim_vbaddr;

	weim_vbaddr = ioremap(WEIM_BASE_ADDR, SZ_4K);
	if (weim_vbaddr == 0) {
		printk(KERN_ERR "Unable to ioremap 0x%08x in %s\n", WEIM_BASE_ADDR, __func__);
		return;
	}

	/** Configure the CS timing, bus width, etc.
	 * 16 bit on DATA[31..16], not multiplexed, async
	 * RWSC=8, RADVA=0, RADVN=0, OEA=0, OEN=0, RCSA=0, RCSN=0, APR=0
	 * WAL=0, WBED=1, WWSC=15, WADVA=0, WADVN=0, WEA=7, WEN=4, WCSA=0
	 */
	__raw_writel(0x00420081, (u32)weim_vbaddr + 0x78 + CSGCR1);
	__raw_writel(0, (u32)weim_vbaddr + 0x78 + CSGCR2);
	__raw_writel(0x08000000, (u32)weim_vbaddr + 0x78 + CSRCR1);
	__raw_writel(0, (u32)weim_vbaddr + 0x78 + CSRCR2);
	__raw_writel(0x4f000f00, (u32)weim_vbaddr + 0x78 + CSWCR1);

	iounmap(weim_vbaddr);

	/* Configure interrupt line as GPIO input, the iomux should be already setup */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_9), "LAN2-irq");
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_9));
}
#endif

#if defined(CONFIG_SND_SOC_IMX_CCIMX51_WM8753) || defined(CONFIG_SND_SOC_IMX_CCIMX51_WM8753_MODULE)
struct platform_device mxc_wm8753_device = {
	.name = "ccimx51js",
};
#endif

struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,
	.spdif_rx = 0,
	.spdif_clk_44100 = 0,	/* spdif_ext_clk source for 44.1KHz */
	.spdif_clk_48000 = 7,	/* audio osc source */
	.spdif_clkid = 0,
	.spdif_clk = NULL,	/* spdif bus clk */
};

struct tve_platform_data tve_data = {
	.dac_reg = "VVIDEO",
	.dig_reg = "VDIG",
};

struct mxc_bus_freq_platform_data bus_freq_data = {
	.gp_reg_id = "SW1",
	.lp_reg_id = "SW2",
};

struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 78770,
};

struct mxc_audio_platform_data wm8753_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.sysclk = 0 /* Set on the fly */,
	.ext_ram_rx = 0,
	.ext_ram_tx = 0,
};

struct mxc_fb_platform_data mx51_fb_data[2] = {
	/* DISP0 */
	{
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,
		.mode_str = "1024x768M-16@60",  /* Default */
	},
	/* DISP1 */
	{
		.interface_pix_fmt = IPU_PIX_FMT_RGB666,
		.mode_str = "800x480-16@60",    /* Default */
	}
};

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
struct ccimx5x_lcd_pdata plcd_platform_data[2];
static int fbprimary = 0;

#if defined(CONFIG_VIDEO_AD9389) || defined(CONFIG_VIDEO_AD9389_MODULE)
static u32 ccimx51_get_max_video_pclk(void)
{
	/**
	 * TODO get this value from the clock subsystem.
	 * 133MHz seems to cause problems with the ext clk.
	 */
	return KHZ2PICOS(132000);
}
#endif

#if defined(CONFIG_CCIMX5X_DISP1)
static char *video2_options[FB_MAX] __read_mostly;
static int ofonly2 __read_mostly;

int fb2_get_options(char *name, char **option)
{
	char *opt, *options = NULL;
	int opt_len, retval = 0;
	int name_len = strlen(name), i;

	if (name_len && ofonly2 && strncmp(name, "offb", 4))
		retval = 1;

	if (name_len && !retval) {
		for (i = 0; i < FB_MAX; i++) {
			if (video2_options[i] == NULL)
				continue;
			opt_len = strlen(video2_options[i]);
			if (!opt_len)
				continue;
			opt = video2_options[i];
			if (!strncmp(name, opt, name_len) &&
			    opt[name_len] == ':')
				options = opt + name_len + 1;
		}
	}
	if (options && !strncmp(options, "off", 3))
		retval = 1;

	if (option)
		*option = options;

	return retval;
}

static int __init video2_setup(char *options)
{
	int i, global = 0;

	if (!options || !*options)
		global = 1;

	if (!global && !strncmp(options, "ofonly", 6)) {
		ofonly2 = 1;
		global = 1;
	}

	if (!global && !strstr(options, "fb:")) {
		fb_mode_option = options;
		global = 1;
	}

	if (!global) {
		for (i = 0; i < FB_MAX; i++) {
			if (video2_options[i] == NULL) {
				video2_options[i] = options;
				break;
			}
		}
	}

        return 1;
}
__setup("video2=", video2_setup);
#endif /* defined(CONFIG_CCIMX5X_DISP1) */

#if defined(CONFIG_CCIMX5X_DISP0) && defined(CONFIG_CCIMX5X_DISP1)
static int __init fbprimary_setup(char *options)
{
	if (options && *options) {
		if (!strncmp(options, "video2", 6))
			fbprimary = 1;
	}

	return 0;
}
__setup("fbprimary=", fbprimary_setup);
#endif /* CONFIG_CCIMX5X_DISP0 && CONFIG_CCIMX5X_DISP1 */

void ccimx51_set_lcd_struct_from_bl_shared(nv_lcd_config_t *lcd_config)
{
	int i;
	uint8_t lcd1_valid, lcd2_valid;
	uint32_t crc;

	lcd1_valid = 0;
	lcd2_valid = 0;

	/* In case of valid structure and valid configuration, copy the LCD settings */
	/* Otherwise clear the LCD settings structure */
	if ( strncmp(lcd_config->header.magic_string, NV_LCD_CONFIG_MAGIC,
		sizeof(lcd_config->header.magic_string)) ) {
		return;
	}

	/* Check version */
	if (lcd_config->header.version < 2) {
		return;
	}

	/* Calculate and check CRC */
	crc = ~crc32_le(~0,
			(const uint8_t *)&lcd_config->lcd1,
			sizeof(nv_lcd_config_data_t));
	crc = ~crc32_le(~crc,
			(const uint8_t *)&lcd_config->lcd2,
			sizeof(nv_lcd_config_data_t));
	if ( crc != lcd_config->header.crc ) {
		return;
	}

	if ( lcd_config->header.lcd1_valid ) {
		lcd1_valid = 1;
	}

	if ( lcd_config->header.lcd2_valid ) {
		lcd2_valid = 1;
	}

	/* Search for custom_nv structures */
	for (i = 0; i < ARRAY_SIZE(lcd_panel_list); i++) {
		if (!strncmp(lcd_panel_list[i].fb_pdata.mode_str,
			"custom3_nv", strlen(lcd_panel_list[i].fb_pdata.mode->name))) {
			struct fb_videomode *videomode = lcd_panel_list[i].fb_pdata.mode;

			if ( lcd1_valid ) {
				/* Skip the name field */
				memcpy(&videomode->refresh, &lcd_config->lcd1.video_mode.refresh,
					sizeof(lcd_custom3_nvram) - sizeof(lcd_custom3_nvram.name));

				/* Set backlight enable function, if it is necessarry */
				if (lcd_config->lcd1.is_bl_enable_func) {
					lcd_panel_list[i].bl_enable = lcd_bl_enable;
				}
			}
		}
		else if (!strncmp(lcd_panel_list[i].fb_pdata.mode_str,
			"custom4_nv", strlen(lcd_panel_list[i].fb_pdata.mode->name))) {
			struct fb_videomode *videomode = lcd_panel_list[i].fb_pdata.mode;

			if ( lcd2_valid ) {
				/* Skip the name field */
				memcpy(&videomode->refresh, &lcd_config->lcd2.video_mode.refresh,
					sizeof(lcd_custom4_nvram) - sizeof(lcd_custom4_nvram.name));

				/* Set backlight enable function, if it is necessarry */
				if (lcd_config->lcd2.is_bl_enable_func) {
					lcd_panel_list[i].bl_enable = lcd_bl_enable;
				}
			}
		}
	}
}

struct ccimx5x_lcd_pdata * ccimx51_find_video_config(struct ccimx5x_lcd_pdata list[],
						     int len,
						     const char *name)
{
	int i;

	for (i = 0; i < len; i++)
		if (!strncmp(list[i].fb_pdata.mode->name,
			     name, strlen(list[i].fb_pdata.mode->name))) {

			/* If it's a passed config from bootloader, check whether it's valid or not */
			if ( !strncmp(list[i].fb_pdata.mode->name, "custom3_nv",
				strlen(list[i].fb_pdata.mode->name)) ||
				!strncmp(list[i].fb_pdata.mode->name, "custom4_nv",
				strlen(list[i].fb_pdata.mode->name)) ) {

				if ( (list[i].fb_pdata.mode->xres == 0) ||
					(list[i].fb_pdata.mode->yres == 0) ) {

					pr_info("%s LCD configuration is not valid\n", name);
					return NULL;
				}
			}

			return &list[i];
		}
	return NULL;
}

static char *ccimx51_get_video_cmdline_opt(int dispif, const char *str)
{
	char *options = NULL;
	int ret = 1;
	int len = strlen(str);

#if defined(CONFIG_CCIMX5X_DISP0)
	if (dispif == 0) {
		ret = fb_get_options("displayfb", &options);
	}
#endif
#if defined(CONFIG_CCIMX5X_DISP1)
	if (dispif == 1) {
		ret = fb2_get_options("displayfb", &options);
	}
#endif
	if (ret || !options)
		return NULL;
	if (!len || !strncasecmp(options, str, len))
		return &options[len];

	return NULL;
}

#if defined(CONFIG_VIDEO_AD9389) || defined(CONFIG_VIDEO_AD9389_MODULE)
static void fb_dump_mode(const char *str, const struct fb_videomode *vm)
{
	if (!(debug & AD9389_DBG))
		return;
	if (vm == NULL)
		return;

	printk(KERN_INFO "%s geometry %u %u %u\n",
	       str, vm->xres, vm->yres, vm->pixclock);
	printk(KERN_INFO "%s timings %u %u %u %u %u %u %u\n", str, vm->pixclock, vm->left_margin,
	       vm->right_margin, vm->upper_margin, vm->lower_margin, vm->hsync_len, vm->vsync_len);
	printk(KERN_INFO "%s flag %u sync %u vmode %u %s\n", str, vm->flag, vm->sync, vm->vmode,
	       vm->flag & FB_MODE_IS_FIRST ? "preferred" : "");
}

static void fb_dump_var(const char *str, struct fb_var_screeninfo *var)
{
	if (!(debug & AD9389_DBG))
		return;
	if (var == NULL)
		return;

	printk(KERN_INFO "%s geometry %u %u %u %u\n",
	       str,  var->xres, var->yres, var->xres_virtual, var->yres_virtual);
	printk(KERN_INFO "%s offset %u %u %u %u %u\n",
	       str, var->xoffset, var->yoffset, var->height, var->width, var->bits_per_pixel);
	printk(KERN_INFO "%s timings %u %u %u %u %u %u %u\n",
	       str, var->pixclock, var->left_margin, var->right_margin,
	       var->upper_margin, var->lower_margin, var->hsync_len, var->vsync_len);
	printk(KERN_INFO "%s accel_flags %u sync %u vmode %u\n",
	       str, var->accel_flags, var->sync, var->vmode);
	printk(KERN_INFO "%d bpp\n", var->bits_per_pixel);
}

enum hdmi_mode get_hdmi_mode(struct ad9389_dev *ad9389, struct fb_videomode **vm, char **str, unsigned int *vpclk, int *ext_clk)
{
	struct ad9389_pdata *pdata = ad9389->client->dev.platform_data;
	struct ccimx5x_lcd_pdata *panel;
	char *p, *temp;

	if ((p = ccimx51_get_video_cmdline_opt(pdata->dispif, "HDMI")) != NULL) {
		DBG(AD9389_DBG, "HDMI: %s config on DISP%d\n", p, pdata->dispif);

		/* Get the desired configuration provided by the bootloader */
		if (vpclk != NULL ) {
			*vpclk = 0;
			/* Parse pclk, it was passed through cmdline */
			if ((temp = strstr(p, "pclk=")) != NULL) {
				*vpclk = (unsigned int)simple_strtoul(temp + 5, NULL, 10);
				if (*vpclk < ccimx51_get_max_video_pclk())
					*vpclk = 0;
			}
			DBG(AD9389_DBG, "HDMI: using cmdline pclk %d\n", *vpclk);
		}
		if (ext_clk != NULL ) {
			/* For single display, default is internal clk and can be overrided by cmdline */
#if !defined(CONFIG_CCIMX5X_DISP0) || !defined(CONFIG_CCIMX5X_DISP1)
			*ext_clk = 1;
#else
			*ext_clk = 0;
#endif
			/* Parse ext_clk, it was passed through cmdline */
			if ((temp = strstr(p, "int_clk")) != NULL)
				*ext_clk = 0;
			if ((temp = strstr(p, "ext_clk")) != NULL)
				*ext_clk = 1;
			DBG(AD9389_DBG, "HDMI: using %s\n", ext_clk ? "ext_clk" : "int_clk");
		}
		if (*p++ != '@') {
			pr_info("Video resolution for HDMI interface not provided, using auto\n");
			return MODE_AUTO;
		} else if (!strncasecmp(p, "auto@", 5)) {
			*str = p + 5;
			if ((temp = strchr(*str, ',')) != NULL)
				*temp = '\0';
			DBG(AD9389_DBG, "HDMI: auto string %s\n", *str);
			return MODE_AUTO_STRING;
		} else if (!strncasecmp(p, "auto", 4)) {
			DBG(AD9389_DBG, "HDMI: auto\n");
			return MODE_AUTO;
		} else  if ((panel = ccimx51_find_video_config(ad9389_panel_list,
						      ARRAY_SIZE(ad9389_panel_list),
						      p)) != NULL) {
			*vm = panel->fb_pdata.mode;
			memcpy(&mx51_fb_data[pdata->dispif],
			       &plcd_platform_data[pdata->dispif].fb_pdata,
			       sizeof(struct mxc_fb_platform_data));
			DBG(AD9389_DBG, "HDMI: forced mode\n");
			return MODE_FORCED;
		} else {
			*str = p;
			if ((temp = strchr(*str, ',')) != NULL)
				*temp = '\0';
			DBG(AD9389_DBG, "HDMI: string %s\n", *str);
			return MODE_STRING;
		}
	}
	return MODE_UNKNOWN;
}

#define	AD9389_STR_LEN		30
static void mxc_videomode_to_var(struct ad9389_dev *ad9389, struct fb_var_screeninfo *var)
{
	struct fb_info *info = ad9389->fbi;
	const struct fb_videomode *fbvmode = NULL;
	char *modestr = NULL, str[AD9389_STR_LEN];
	unsigned int tpclk = 0;
	int modeidx;
	int ext_clk = 0;
	enum hdmi_mode mode;

	var->bits_per_pixel = CONFIG_CCIMX5X_DEFAULT_VIDEO_BPP;	/* Set default bpp  */
	/* First, check if we have a predefined mode through the kernel command line */
	mode = get_hdmi_mode(ad9389, (struct fb_videomode **)&fbvmode, &modestr, &tpclk, &ext_clk);
	if (mode == MODE_AUTO) {
		/* auto, or no video mode provided */
		strncpy(str, "HDMI auto selected mode:", AD9389_STR_LEN - 1);
		fbvmode = fb_find_best_mode(var, &info->modelist);
		if (!fbvmode) {
			fbvmode = fb_find_best_display(&info->monspecs, &info->modelist);
			if (!fbvmode) {
				printk(KERN_WARNING
				      "HDMI: unable to find a valid video mode/screen,"
				      " try forcing a mode\n");
				/* Use default... */
				fbvmode = &ad9389_1024x768x24;
				strncpy(str, "HDMI default mode:", AD9389_STR_LEN - 1);
			}
		}
	} else if (mode == MODE_FORCED) {
		/* Selected video mode through cmd line parameters provided */
		strncpy(str, "HDMI forced mode:", AD9389_STR_LEN - 1);
	} else if ((mode == MODE_STRING || mode == MODE_AUTO_STRING) && modestr) {
		DBG(AD9389_DBG, "HDMI mode string: %s\n", modestr);
		modeidx = fb_find_mode(var, info, modestr,
				       info->monspecs.modedb,
				       info->monspecs.modedb_len,
				       NULL, var->bits_per_pixel);
		if (!(modeidx == 1 || modeidx == 1)) {
			DBG(AD9389_DBG, "HDMI: unable to find valid mode (%s)\n", modestr);
			return;
		}
		strncpy(str, "HDMI string mode:", AD9389_STR_LEN - 1);
	}
	str[AD9389_STR_LEN - 1] = 0;
	if ((mode == MODE_AUTO) || (mode == MODE_FORCED)) {
		fb_dump_mode(str, fbvmode);
		fb_videomode_to_var(var, fbvmode);
	}

	if (ext_clk)
		var->sync |= FB_SYNC_EXT;

	/* Check if clock must be readjusted */
	if (tpclk != 0)
		var->pixclock = tpclk;

	fb_dump_var(str, var);
}

/**
 * This function parses the list of supported video modes (got from fb_edid_to_monspecs) and
 * filters out not supported configurations
 */
static void mxcfb_vmode_to_modelist(struct fb_videomode *modedb, int num,
				    struct list_head *head,  struct fb_var_screeninfo *var)
{
	int i, xres = 0, yres = 0, aspratio = 0;

	INIT_LIST_HEAD(head);

	/**
	 * Add the modes we got through the monitor specs, filtering out those
	 * unsupported configurations.
	 */
	for (i = 0; i < num; i++) {
		struct list_head *pos, *n;
		struct fb_modelist *modelist;
		int remove, vmaspratio;

		remove = 0;
		vmaspratio = -1;

		/* Use the preferred mode to compute the aspect ratio */
		if (modedb[i].flag & FB_MODE_IS_FIRST) {
			DBG(AD9389_DBG, "PREFERRED: %ux%u%s%u pclk=%u\n",
			    modedb[i].xres, modedb[i].yres,
			    (modedb[i].vmode & FB_VMODE_INTERLACED ) ? "i@" : "@",
			    modedb[i].refresh, modedb[i].pixclock);

			aspratio = modedb[i].xres * 10 / modedb[i].yres;
			DBG(AD9389_DBG, "Aspect Ratio: %d\n", aspratio);
		}

		if (modedb[i].yres)
			vmaspratio = modedb[i].xres * 10 / modedb[i].yres;

		if (vmaspratio != aspratio) {
			DBG(AD9389_DBG, "REMOVED: %ux%u%s%u pclk=%u (aspect ratio)\n",
			    modedb[i].xres, modedb[i].yres,
			    (modedb[i].vmode & FB_VMODE_INTERLACED ) ? "i@" : "@",
			    modedb[i].refresh, modedb[i].pixclock);
			continue;
		}

		/* Interlaced not supported */
		if (modedb[i].vmode & FB_VMODE_INTERLACED) {
			DBG(AD9389_DBG, "REMOVED: %ux%u%s%u pclk=%u (interlaced modes not supported)\n",
			    modedb[i].xres, modedb[i].yres,
			    (modedb[i].vmode & FB_VMODE_INTERLACED ) ? "i@" : "@",
			    modedb[i].refresh, modedb[i].pixclock);
			continue;
		}

		/* If clock exceeds the max pixel clock supported, remove that video mode */
		if ((modedb[i].pixclock * 115 / 100) < ccimx51_get_max_video_pclk()) {
			DBG(AD9389_DBG, "REMOVED: %ux%u%s%u pclk=%u (exceed %u limit)\n",
			    modedb[i].xres, modedb[i].yres,
			    (modedb[i].vmode & FB_VMODE_INTERLACED ) ? "i@" : "@",
			    modedb[i].refresh, modedb[i].pixclock, ccimx51_get_max_video_pclk());
			continue;
		}

		/* If over the pixel clock limix, but close enough, set the max pixel clock freq */
		if (modedb[i].pixclock < ccimx51_get_max_video_pclk())
			modedb[i].pixclock = ccimx51_get_max_video_pclk();

		/**
		 * Adjust timing to IPU restrictions (better done here, to avoid ipu driver to
		 * incorrectly calculate settings based on our configuration).
		 */
		if (modedb[i].lower_margin < 2) {
			/* This will not affect much, so we dont adjust the pixel clock */
			DBG(AD9389_DBG, "ADJUSTED: lower margin from %u to 2\n",
				    modedb[i].lower_margin);
			modedb[i].lower_margin = 2;
		}

		/**
		 * Remove duplicated modes, selecting the best modes accordingly to the
		 * platform video constraints.
		 */
		list_for_each_safe(pos, n, head) {
			modelist = list_entry(pos, struct fb_modelist, list);

			if ((modelist->mode.xres == modedb[i].xres) &&
			    (modelist->mode.yres == modedb[i].yres)) {

				if (modedb[i].pixclock == ccimx51_get_max_video_pclk()) {
					/* If current mode pixclk is set to max clock, do not
					 * add this mode and use the existing one. */
					remove = 1;
				} else if ((modelist->mode.refresh == modedb[i].refresh) &&
				    (modedb[i].flag & FB_MODE_IS_DETAILED)) {
					DBG(AD9389_DBG, "REMOVED: %ux%u%s%u pclk=%u (duplicated)\n",
					    modelist->mode.xres, modelist->mode.yres,
					    (modelist->mode.vmode & FB_VMODE_INTERLACED ) ? "i@" : "@",
					    modelist->mode.refresh, modelist->mode.pixclock);
					list_del(pos);
					kfree(pos);
				} else {
					/* Do not add this mode, it is not a detailed timing */
					remove = 1;
				}
			}
		}

		if (!remove) {
			fb_add_videomode(&modedb[i], head);
			DBG(AD9389_DBG, "ADDING: Video mode %ux%u%s%u pclk=%u, %s detailed\n",
			    modedb[i].xres, modedb[i].yres,
			    (modedb[i].vmode & FB_VMODE_INTERLACED ) ? "i@" : "@",
			    modedb[i].refresh, modedb[i].pixclock,
			    (modedb[i].flag & FB_MODE_IS_DETAILED) ? "" : "no");

			if (modedb[i].xres > xres && modedb[i].yres > yres) {
				xres = modedb[i].xres;
				yres = modedb[i].yres;
			}
		}
	}

	/* Update var->xres and var->yres, used to determine the best video mode*/
	if (var->xres != xres || var->yres != yres) {
		var->xres = xres;
		var->yres = yres;
	}
}

static int ccimx51_hdmi_hw_init(struct ad9389_dev *ad9389)
{
	struct ad9389_pdata *pdata = ad9389->client->dev.platform_data;

#ifdef AD9389_GPIO_IRQ
	if (pdata->dispif == 0) {
		mxc_request_iomux(AD9389_GPIO_IRQ, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION);
		mxc_iomux_set_pad(AD9389_GPIO_IRQ, PAD_CTL_SRE_SLOW | PAD_CTL_DRV_MEDIUM |
				  PAD_CTL_100K_PU | PAD_CTL_HYS_ENABLE |
				  PAD_CTL_DRV_VOT_HIGH);

		gpio_request(IOMUX_TO_GPIO(AD9389_GPIO_IRQ), "ad9389_irq");
		gpio_direction_input(IOMUX_TO_GPIO(AD9389_GPIO_IRQ));

		set_irq_type(IOMUX_TO_GPIO(AD9389_GPIO_IRQ), IRQ_TYPE_EDGE_BOTH);
	}
#endif
	/* Configure here the hot plug detection for HDMI on DISP1 */
	/* if (pdata->dispif == 1) { } */

	gpio_video_active(pdata->dispif,
			  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);

	return 0;
}

static void ccimx51_hdmi_disp_connected(struct ad9389_dev *ad9389)
{
	printk(KERN_DEBUG "%s: display connected\n", __func__);
}

static void ccimx51_hdmi_disp_disconnected(struct ad9389_dev *ad9389)
{
	printk(KERN_DEBUG "%s: display disconnected\n", __func__);
}

static struct ad9389_pdata hdmi_pdata = {
	.hw_init		= &ccimx51_hdmi_hw_init,
	.disp_connected		= &ccimx51_hdmi_disp_connected,
	.disp_disconnected	= &ccimx51_hdmi_disp_disconnected,
	.vmode_to_modelist	= &mxcfb_vmode_to_modelist,
	.vmode_to_var		= &mxc_videomode_to_var,
	.edid_addr		= (0x7e >> 1),
	.dispif			= 0,
	.debounce_ms		= 500,
};

struct i2c_board_info ccimx51_hdmi[] __initdata = {
	{
		I2C_BOARD_INFO("ad9389", 0x39),
		.platform_data	= &hdmi_pdata,
#ifdef AD9389_GPIO_IRQ
		.irq		= IOMUX_TO_IRQ(AD9389_GPIO_IRQ),
#endif
	},
};
#endif

#define MAX_VIDEO_IF		2
int __init ccimx51_init_fb(void)
{
	struct ccimx5x_lcd_pdata *panel;
	char *p, *mstr;
	int i, regfbdev, fborg = 0, fbend = MAX_VIDEO_IF, fbinc = 1;

	plcd_platform_data[0].vif = -1;
	plcd_platform_data[1].vif = -1;

	/* If primary is video2, loop in revers order */
	if (fbprimary == 1) {
		fborg = MAX_VIDEO_IF - 1;
		fbend = -1;
		fbinc = -1;
	}

	for (i = fborg, regfbdev = 0; i != fbend; i += fbinc) {
#if !defined(CONFIG_CCIMX5X_DISP0)
		if (i == 0)	continue;
#endif
#if !defined(CONFIG_CCIMX5X_DISP1)
		if (i == 1)	continue;
#endif

		/* Set default pixel format, maybe overwritten later */
		mx51_fb_data[i].interface_pix_fmt = i ? DISP1_PIX_FMT : DISP0_PIX_FMT;

		if ((p = ccimx51_get_video_cmdline_opt(i, "HDMI")) != NULL) {
#if defined(CONFIG_VIDEO_AD9389) || defined(CONFIG_VIDEO_AD9389_MODULE)
			pr_info("HDMI interface in DISP%d\n", i);
			i2c_register_board_info(1, ccimx51_hdmi, 1);
#else
			pr_info("HDMI selected in DISP%d, but driver unavailable\n", i);
			continue;
#endif
		} else 	if ((p = ccimx51_get_video_cmdline_opt(i, "LCD")) != NULL) {
			pr_info("LCD interface in DISP%d", i);
			if (*p++ != '@') {
				pr_info("Panel not provided, video interface will be disabled\n");
				continue;
			}
			if ((panel = ccimx51_find_video_config(lcd_panel_list,
							      ARRAY_SIZE(lcd_panel_list),
							      p)) != NULL) {
				pr_info("Panel: %s", p);
				memcpy(&plcd_platform_data[i],
				       panel,
				       sizeof(struct ccimx5x_lcd_pdata));
				memcpy(&mx51_fb_data[i],
				       &plcd_platform_data[i].fb_pdata,
				       sizeof(struct mxc_fb_platform_data));
				plcd_platform_data[i].vif = i;
				if (!plcd_platform_data[i].fb_pdata.interface_pix_fmt)
					mx51_fb_data[i].interface_pix_fmt =
						i ? DISP1_PIX_FMT : DISP0_PIX_FMT;
				mxc_register_device(&lcd_pdev[i], (void *)&plcd_platform_data[i]);
			}
		} else if ((p = ccimx51_get_video_cmdline_opt(i, "VGA")) != NULL) {
			pr_info("VGA interface in DISP%d\n", i);
			gpio_video_active(i, PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
			mstr = p - 3;

			/* Get the desired configuration provided by the bootloader */
			if (*p++ != '@') {
				pr_info("Video resolution for VGA interface not provided, using default\n");
			} else {
				/* Check string to see if its one of the configurations we alaredy have...
				 * and if not, pass it as mode string, just in case we want to use one
				 * of the standard video configurations
				 */
				if ((panel = ccimx51_find_video_config(vga_panel_list,
								       ARRAY_SIZE(vga_panel_list),
								       p)) != NULL) {
					pr_info("Panel: %s", p);
					memcpy(&mx51_fb_data[i],
					       &plcd_platform_data[i].fb_pdata,
					       sizeof(struct mxc_fb_platform_data));
				} else {
					/* Pass the video configuration as mode string */
					pr_info("VGA: string %s", p);

					if (!strcmp(p, "800x600")) {
						strcpy(mx51_fb_data[0].mode_str, "VGA@800x600M-32");
					} else if (!strcmp(p, "1280x1024")) {
						strcpy(mx51_fb_data[0].mode_str, "VGA@1280x1024M-32");
					} else {
						strcpy(mx51_fb_data[0].mode_str, mstr);
					}
				}
			}
		}
		mxc_fb_devices[i].num_resources = 1;
		mxc_fb_devices[i].resource = &mxcfb_resources[regfbdev];
		mxc_register_device(&mxc_fb_devices[i], &mx51_fb_data[i]);
		regfbdev++;
	}

	/* DI0/1 DP-FG channel, used by the VPU */
	mxc_register_device(&mxc_fb_devices[2], NULL);

	return 0;
}
#endif

#if defined(CONFIG_PATA_FSL) || defined(CONFIG_PATA_FSL_MODULE)
extern void gpio_ata_active(void);
extern void gpio_ata_inactive(void);

static int ccimx51_init_ata(struct platform_device *pdev)
{
	gpio_ata_active();
	return 0;
}

static void ccimx51_deinit_ata(void)
{
	gpio_ata_inactive();
}

struct fsl_ata_platform_data ata_data = {
#ifndef CONFIG_PATA_FSL_DISABLE_DMA
	.udma_mask  = ATA_UDMA3,
	.mwdma_mask = ATA_MWDMA2,
#endif
	.pio_mask   = ATA_PIO4,
	.fifo_alarm = MXC_IDE_DMA_WATERMARK / 2,
	.max_sg     = MXC_IDE_DMA_BD_NR,
	.init       = ccimx51_init_ata,
	.exit       = ccimx51_deinit_ata,
	.core_reg   = NULL,
	.io_reg     = NULL,
};
#endif

#ifdef CONFIG_MXC_IIM
static void mxc_iim_enable_fuse(void)
{
	u32 reg;

	if (!ccm_base)
		return;
	/* Enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg |= 0x10;
	writel(reg, ccm_base + 0x64);
}

static void mxc_iim_disable_fuse(void)
{
	u32 reg;

	/* Disable fuse blown */
	if (!ccm_base)
		return;

	reg = readl(ccm_base + 0x64);
	reg &= ~0x10;
	writel(reg, ccm_base + 0x64);
}

struct mxc_iim_data iim_data = {
	.bank_start = MXC_IIM_MX51_BANK_START_ADDR,
	.bank_end   = MXC_IIM_MX51_BANK_END_ADDR,
	.enable_fuse = mxc_iim_enable_fuse,
	.disable_fuse = mxc_iim_disable_fuse,
};
#endif

void ccimx51_init_devices(void)
{
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
	ccimx51_init_ext_eth_mac();
#endif
}

/*
 * Copyright 2011 Digi International, Inc. All Rights Reserved.
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
#include <mach/iomux-mx53.h>
#include <video/ad9389.h>
#include <linux/smc911x.h>
#include <linux/fec.h>
#include <mach/mxc_iim.h>
#include <mach/iomux-v3.h>

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/flash.h>
#endif

#include "devices_ccimx53.h"
#include "board-ccimx53.h"
#include "crm_regs.h"
#include "devices.h"
#include "displays/displays.h"

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
#include <asm/mach/bootldr_shmem.h>
#include <linux/crc32.h>
#endif

extern void ccimx53_gpio_spi_chipselect_active(int cspi_mode, int status,
						    int chipselect);
extern void ccimx53_gpio_spi_chipselect_inactive(int cspi_mode, int status,
						      int chipselect);

/* Spaceholder */
int ccimx5x_get_cpufreq_from_variant( void )
{
	return -1;
}

void ccimx53_user_led(int led, int val)
{
#if defined(CONFIG_HAS_EARLY_USER_LEDS)
	__iomem void *base;
	u32 reg, mask;

	if (led == 1) {
		/* USER LED1 GPIO 5, 20 */
		mask = 1 << 20;
		base = ioremap(GPIO5_BASE_ADDR - MX53_OFFSET, SZ_4K);
	} else if (led == 2) {
		/* USER LED2 GPIO 7, 12 */
		mask = 1 << 12;
		base = ioremap(GPIO7_BASE_ADDR - MX53_OFFSET, SZ_4K);
	} else
		return;

	reg = __raw_readl(base);
	if (val)
		reg |= mask;
	else
		reg &= ~mask;

	__raw_writel(reg, base);
	iounmap(base);
#endif /* CONFIG_HAS_EARLY_USER_LEDS */
}

/* Only SD2 has connected WP and CD */
#define ESDHC2_WP_GPIO MX53_GPIO(1,2)
#define ESDHC2_CD_GPIO MX53_GPIO(1,4)

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
static int sdhc_write_protect(struct device *dev)
{
	int ret = 0;

#ifdef ESDHC1_WP_GPIO
	if (to_platform_device(dev)->id == 0)
		ret = gpio_get_value(ESDHC1_WP_GPIO);
#endif
#ifdef ESDHC2_WP_GPIO
	if (to_platform_device(dev)->id == 1)
		ret = gpio_get_value(ESDHC2_WP_GPIO);
#endif
#ifdef ESDHC3_WP_GPIO
	if (to_platform_device(dev)->id == 2)
		ret = gpio_get_value(ESDHC3_WP_GPIO);
#endif
#ifdef ESDHC4_WP_GPIO
	if (to_platform_device(dev)->id == 3)
		ret = gpio_get_value(ESDHC4_WP_GPIO);
#endif
	return ret;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret = 0;

#ifdef ESDHC1_CD_GPIO
	if (to_platform_device(dev)->id == 0)
		ret = gpio_get_value(ESDHC1_CD_GPIO);
#endif
#ifdef ESDHC2_CD_GPIO
	if (to_platform_device(dev)->id == 1)
		ret = gpio_get_value(ESDHC2_CD_GPIO);
#endif
#ifdef ESDHC3_CD_GPIO
	if (to_platform_device(dev)->id == 2)
		ret = gpio_get_value(ESDHC3_CD_GPIO);
#endif
#ifdef ESDHC4_CD_GPIO
	if (to_platform_device(dev)->id == 3)
		ret = gpio_get_value(ESDHC4_CD_GPIO);
#endif
	return ret;
}

#ifdef CONFIG_ESDHCI_MXC_SELECT1
static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};
#endif

#ifdef CONFIG_ESDHCI_MXC_SELECT2
static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};
#endif

#ifdef CONFIG_ESDHCI_MXC_SELECT3
static struct mxc_mmc_platform_data mmc3_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
//	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA
//		| MMC_CAP_DATA_DDR,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};
#endif


void ccimx53_register_sdio(int interface)
{
	gpio_sdhc_active(interface);

	switch (interface) {
#ifdef CONFIG_ESDHCI_MXC_SELECT1
	case 0:
#ifdef ESDHC1_CD_GPIO
		mxcsdhc1_device.resource[2].start = gpio_to_irq(ESDHC1_CD_GPIO);
		mxcsdhc1_device.resource[2].end = gpio_to_irq(ESDHC1_CD_GPIO);
#endif
		mxc_register_device(&mxcsdhc1_device, &mmc1_data);
		break;
#endif
#ifdef CONFIG_ESDHCI_MXC_SELECT2
	case 1:
#ifdef ESDHC2_CD_GPIO
		mxcsdhc2_device.resource[2].start = gpio_to_irq(ESDHC2_CD_GPIO);
		mxcsdhc2_device.resource[2].end = gpio_to_irq(ESDHC2_CD_GPIO);
#endif
		mxc_register_device(&mxcsdhc2_device, &mmc2_data);
		break;
#endif
#ifdef CONFIG_ESDHCI_MXC_SELECT3
	case 2:
#ifdef ESDHC3_CD_GPIO
		mxcsdhc3_device.resource[2].start = gpio_to_irq(ESDHC3_CD_GPIO);
		mxcsdhc3_device.resource[2].end = gpio_to_irq(ESDHC3_CD_GPIO);
#endif
		mxc_register_device(&mxcsdhc3_device, &mmc3_data);
		break;
#endif
#ifdef CONFIG_ESDHCI_MXC_SELECT4
	case 3:
#ifdef ESDHC4_CD_GPIO
		mxcsdhc4_device.resource[2].start = gpio_to_irq(ESDHC4_CD_GPIO);
		mxcsdhc4_device.resource[2].end = gpio_to_irq(ESDHC4_CD_GPIO);
#endif
		mxc_register_device(&mxcsdhc4_device, &mmc4_data);
		break;
#endif
	}
}
#else
void ccimx53_register_sdio(int interface) {}
#endif


#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
struct ccimx5x_lcd_pdata plcd_platform_data[2];
static int fbprimary = 0;
#if defined(CONFIG_VIDEO_AD9389) || defined(CONFIG_VIDEO_AD9389_MODULE)
static int debug = 0;
#endif

#define AD9389_DBG		0x0001
#define DBG(flag, fmt, args...)	do {						\
					if (debug & flag)			\
						printk(fmt, ## args);		\
				} while (0)

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

struct mxc_fb_platform_data mx53_fb_data[2] = {
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

struct ldb_platform_data ldb_data = {
	.ext_ref = 1,
};

static struct tve_platform_data tve_data = {
	.dac_reg 	= "DA9052_LDO7",
	.boot_enable	= MXC_TVE_VGA,
};

#if defined(CONFIG_VIDEO_AD9389) || defined(CONFIG_VIDEO_AD9389_MODULE)
static u32 ccimx53_get_max_video_pclk(void)
{
	/* get this value from the clock subsystem */
	return KHZ2PICOS(200000);
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

void ccimx53_set_lcd_struct_from_bl_shared(nv_lcd_config_t *lcd_config)
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

struct ccimx5x_lcd_pdata * ccimx53_find_video_config(struct ccimx5x_lcd_pdata list[],
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

static char *ccimx53_get_video_cmdline_opt(int dispif, const char *str)
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

int ccimx53_check_display_type(int dispif, const char *str)
{
#if defined(CONFIG_JSCCIMX53_V2)
	char *options = NULL;
	int len = strlen(str);
	int ret = 0;
#if defined(CONFIG_CCIMX5X_DISP0)
	if (dispif == 0) {
		ret = fb_get_options("displayfb", &options);
		if ( !ret && options ){
			return (!strncasecmp(options, str, len) );
		}
	}
#endif
#if defined(CONFIG_CCIMX5X_DISP1)
	if (dispif == 1) {
		ret = fb2_get_options("displayfb", &options);
		if ( !ret && options ){
			return (!strncasecmp(options, str, len) );
		}
	}
#endif
#endif
	return 0;
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

	if ((p = ccimx53_get_video_cmdline_opt(pdata->dispif, "HDMI")) != NULL) {
		DBG(AD9389_DBG, "HDMI: %s config on DISP%d\n", p, pdata->dispif);

		/* Get the desired configuration provided by the bootloader */
		if (vpclk != NULL ) {
			*vpclk = 0;
			/* Parse pclk, it was passed through cmdline */
			if ((temp = strstr(p, "pclk=")) != NULL) {
				*vpclk = (unsigned int)simple_strtoul(temp + 5, NULL, 10);
				if (*vpclk < ccimx53_get_max_video_pclk())
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
			DBG(AD9389_DBG, "HDMI: using %s\n", *ext_clk ? "ext_clk" : "int_clk");
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
		} else  if ((panel = ccimx53_find_video_config(ad9389_panel_list,
						      ARRAY_SIZE(ad9389_panel_list),
						      p)) != NULL) {
			*vm = panel->fb_pdata.mode;
			memcpy(&mx53_fb_data[pdata->dispif],
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
	unsigned int tpclk;
	int modeidx, ext_clk;
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

	/* TODO, configure clk active edge depending on edid values read */
	var->sync |= FB_SYNC_CLK_LAT_FALL;
	if (ext_clk)
		var->sync |= FB_SYNC_EXT;

	/* Check if clock must be readjusted */
	if (tpclk != 0)
		var->pixclock = tpclk;

	/* PPH, TODO, select video interface properly */
	if (fbvmode && fbvmode->xres <= 1366)
		gpio_video_active(0, PAD_CTL_DSE_LOW);
	else
		gpio_video_active(0, PAD_CTL_DSE_MED);

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
		if ((modedb[i].pixclock * 115 / 100) < ccimx53_get_max_video_pclk()) {
			DBG(AD9389_DBG, "REMOVED: %ux%u%s%u pclk=%u (exceed %u limit)\n",
			    modedb[i].xres, modedb[i].yres,
			    (modedb[i].vmode & FB_VMODE_INTERLACED ) ? "i@" : "@",
			    modedb[i].refresh, modedb[i].pixclock, ccimx53_get_max_video_pclk());
			continue;
		}

		/* If over the pixel clock limix, but close enough, set the max pixel clock freq */
		if (modedb[i].pixclock < ccimx53_get_max_video_pclk())
			modedb[i].pixclock = ccimx53_get_max_video_pclk();

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

				if (modedb[i].pixclock == ccimx53_get_max_video_pclk()) {
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

static int ccimx53_hdmi_hw_init(struct ad9389_dev *ad9389)
{
	struct ad9389_pdata *pdata = ad9389->client->dev.platform_data;

	if (pdata->dispif == 0) {
		mxc_iomux_v3_setup_pad(AD9389_IRQ_PAD);
		gpio_request(AD9389_GPIO_IRQ, "ad9389_irq");
		gpio_direction_input(AD9389_GPIO_IRQ);
	}
	gpio_video_active(pdata->dispif, PAD_CTL_DSE_MED);

	return 0;
}

static void ccimx53_hdmi_disp_connected(struct ad9389_dev *ad9389)
{
	printk(KERN_DEBUG "%s: display connected\n", __func__);
}

static void ccimx53_hdmi_disp_disconnected(struct ad9389_dev *ad9389)
{
	printk(KERN_DEBUG "%s: display disconnected\n", __func__);
}

static struct ad9389_pdata hdmi_pdata = {
	.hw_init		= &ccimx53_hdmi_hw_init,
	.disp_connected		= &ccimx53_hdmi_disp_connected,
	.disp_disconnected	= &ccimx53_hdmi_disp_disconnected,
	.vmode_to_modelist	= &mxcfb_vmode_to_modelist,
	.vmode_to_var		= &mxc_videomode_to_var,
	.edid_addr		= (0x7e >> 1),
	.dispif			= 0,
	.debounce_ms		= 500,
};

struct i2c_board_info ccimx53_hdmi[] __initdata = {
	{
		I2C_BOARD_INFO("ad9389", 0x39),
		.irq		= gpio_to_irq(AD9389_GPIO_IRQ),
		.platform_data	= &hdmi_pdata,
	},
};
#endif

#define MAX_VIDEO_IF		2
int __init ccimx5x_init_fb(void)
{
	struct ccimx5x_lcd_pdata *panel;
	char *p, *mstr;
	int i, regfbdev, fborg = 0, fbend = MAX_VIDEO_IF, fbinc = 1;
	int present = 0;

	plcd_platform_data[0].vif = -1;
	plcd_platform_data[1].vif = -1;

	/* If primary is video2, loop in revers order */
	if (fbprimary == 1) {
		fborg = MAX_VIDEO_IF - 1;
		fbend = -1;
		fbinc = -1;
	}

	for (i = fborg, regfbdev = 0; i != fbend; i += fbinc) {
		present = 0;
#if !defined(CONFIG_CCIMX5X_DISP0)
		if (i == 0)	continue;
#endif
#if !defined(CONFIG_CCIMX5X_DISP1)
		if (i == 1)	continue;
#endif
		/* Set default pixel format, maybe overwritten later */
		mx53_fb_data[i].interface_pix_fmt = i ? DISP1_PIX_FMT : DISP0_PIX_FMT;

		if ((p = ccimx53_get_video_cmdline_opt(i, "disabled")) != NULL) {
			/* Skip this interface */
			present = 1;
			continue;
		} else if ((p = ccimx53_get_video_cmdline_opt(i, "HDMI")) != NULL) {
			present = 1;
#if defined(CONFIG_VIDEO_AD9389) || defined(CONFIG_VIDEO_AD9389_MODULE)
			pr_info("HDMI interface in DISP%d\n", i);
			i2c_register_board_info(2, ccimx53_hdmi, 1);
			hdmi_pdata.fbidx = regfbdev;
#else
			pr_info("HDMI selected in DISP%d, but driver unavailable\n", i);
			continue;
#endif
		} else 	if (((p = ccimx53_get_video_cmdline_opt(i, "LCD")) != NULL) ||
			    ((p = ccimx53_get_video_cmdline_opt(i, "LVDS")) != NULL)) {
			present = 1;
			pr_info("LCD/LVDS interface in DISP%d", i);
			if (*p++ != '@') {
				pr_info("Panel not provided, video interface will be disabled\n");
				continue;
			}
			if ((panel = ccimx53_find_video_config(lcd_panel_list,
							      ARRAY_SIZE(lcd_panel_list),
							      p)) != NULL) {
				pr_info("Panel: %s", p);
				memcpy(&plcd_platform_data[i],
				       panel,
				       sizeof(struct ccimx5x_lcd_pdata));
				memcpy(&mx53_fb_data[i],
				       &plcd_platform_data[i].fb_pdata,
				       sizeof(struct mxc_fb_platform_data));
				plcd_platform_data[i].vif = i;
				if (!plcd_platform_data[i].fb_pdata.interface_pix_fmt)
					mx53_fb_data[i].interface_pix_fmt =
						i ? DISP1_PIX_FMT : DISP0_PIX_FMT;
				mxc_register_device(&lcd_pdev[i], (void *)&plcd_platform_data[i]);
			}
		} else if ((p = ccimx53_get_video_cmdline_opt(i, "VGA")) != NULL) {
			present = 1;
			pr_info("VGA interface in DISP%d\n", i);
			gpio_vga_active(i);
			mstr = p - 3;

			/* Get the desired configuration provided by the bootloader */
			if (*p++ != '@') {
				pr_info("Video resolution for VGA interface not provided, using default\n");
			} else {
				/* Check string to see if its one of the configurations we alaredy have...
				 * and if not, pass it as mode string, just in case we want to use one
				 * of the standard video configurations
				 */
				if ((panel = ccimx53_find_video_config(vga_panel_list,
								       ARRAY_SIZE(vga_panel_list),
								       p)) != NULL) {
					pr_info("Panel: %s", p);
					memcpy(&mx53_fb_data[i],
					       &plcd_platform_data[i].fb_pdata,
					       sizeof(struct mxc_fb_platform_data));
				} else {
					/* Pass the video configuration as mode string */
					pr_info("VGA: string %s", p);
					if (!strcmp(p, "640x480")) {
						strcpy(mx53_fb_data[i].mode_str, "VGA-VGA");
					} else if (!strcmp(p, "800x600")) {
						strcpy(mx53_fb_data[i].mode_str, "VGA-SVGA");
					} else if (!strcmp(p, "1024x768")) {
						strcpy(mx53_fb_data[i].mode_str, "VGA-XGA");
					} else if (!strcmp(p, "1280x1024")) {
						strcpy(mx53_fb_data[i].mode_str, "VGA-SXGA");
					} else if (!strcmp(p, "1680x1050")) {
						strcpy(mx53_fb_data[i].mode_str, "VGA-WSXGA+");
					} else {
						strcpy(mx53_fb_data[i].mode_str, mstr);
					}
				}
			}
			/* Register the TVE device and set the pixel format for the TVE-VGA interface */
			mx53_fb_data[i].interface_pix_fmt = IPU_PIX_FMT_GBR24;
			mxc_register_device(&mxc_tve_device, &tve_data);
		}
		if( present ){
			mxc_fb_devices[i].num_resources = 1;
			mxc_fb_devices[i].resource = &mxcfb_resources[regfbdev];
			mxc_register_device(&mxc_fb_devices[i], &mx53_fb_data[i]);
			regfbdev++;
		}
	}

	/* Register the LVDS bridge */
	mxc_register_device(&mxc_ldb_device, &ldb_data);

	/* DI0/1 DP-FG channel, used by the VPU */
	mxc_register_device(&mxc_fb_devices[2], NULL);

	return 0;
}
#else
int __init ccimx5x_init_fb(void) {return 0;}
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
      gpio_nand_active();
      return 0;
}

static void nand_exit(void)
{
      gpio_nand_inactive();
}

struct flash_platform_data mxc_nand_data = {
	.width = 1,
	.init = nand_init,
	.exit = nand_exit,
};

void ccimx53_register_nand(void)
{
	mxc_register_device(&mxc_nandv2_mtd_device, &mxc_nand_data);
}
#else
void ccimx53_register_nand(void) {}
#endif

#if defined(CONFIG_SND_SOC_IMX_CCIMX53_SGTL5000) || defined(CONFIG_SND_SOC_IMX_CCIMX53_SGTL5000_MODULE)

static int mxc_sgtl5000_amp_enable(int enable)
{
	return 0;
}

#define MX53_HP_DETECT	MX53_GPIO(4,3)	/* GPIO_4_3 */

static int headphone_det_status(void)
{
	return (gpio_get_value(MX53_HP_DETECT) == 0);
}

static int mxc_sgtl5000_init(void);

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
	.hp_irq = IOMUX_TO_IRQ(MX53_HP_DETECT),
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
	.init = mxc_sgtl5000_init,
	.ext_ram_rx = 0,
	.ext_ram_tx = 0,
};

static int mxc_sgtl5000_init(void)
{
	sgtl5000_data.sysclk = 13000000;
	return 0;
}

struct platform_device mxc_sgtl5000_device = {
	.name = "imx-ccimx53-sgtl5000",
};

void ccimx53_register_sgtl5000(void)
{
	sgtl5000_data.ext_ram_clk = clk_get(NULL, "emi_fast_clk");
	clk_put(sgtl5000_data.ext_ram_clk);
	mxc_register_device(&mxc_sgtl5000_device, &sgtl5000_data);
}
#else
void ccimx53_register_sgtl5000(void) {}
#endif

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
int ccimx53_fec_init( void )
{
	int ret;

	pr_debug("Resetting ethernet PHY\n");

	/* Reset the interface */
	ret = gpio_request(ETH0_RESET_GPIO, "eth0_reset");
	if (ret) {
		printk(KERN_ERR
		"Couldn't request GPIO %d (PHY reset pin)\n",
		ETH0_RESET_GPIO);
	    /* Dont exit with error, the interface maybe already powered on */
	} else {
		gpio_direction_output(ETH0_RESET_GPIO, 0);
		mdelay(50);
		gpio_set_value_cansleep(ETH0_RESET_GPIO, 1);
	    gpio_free(ETH0_RESET_GPIO);
	}
	return ret;
}

static struct fec_platform_data fec_pdata = {
	.phy = PHY_INTERFACE_MODE_RMII,
	.init = ccimx53_fec_init,
};

void ccimx53_register_fec(void)
{
	gpio_fec_active();
	mxc_register_device(&mxc_fec_device, &fec_pdata);
	mxc_register_device(&mxc_ptp_device, NULL);
}
#else
void ccimx53_register_fec(void) {}
#endif


#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
int ccimx53_smsc911x_init( void )
{
	int ret;

	pr_debug("Resetting smc911x ethernet controller.\n");

	/* Reset the interface */
	ret = gpio_request(EXT_ETH_RESET_GPIO, "eth1_reset");
	if (ret) {
		printk(KERN_ERR
		"Couldn't request GPIO %d (EXT ETH reset pin)\n",
		EXT_ETH_RESET_GPIO);
	    /* Dont exit with error, the interface maybe already powered on */
	} else {
		gpio_direction_output(EXT_ETH_RESET_GPIO, 0);
		mdelay(50);
		gpio_set_value_cansleep(EXT_ETH_RESET_GPIO, 1);
	    gpio_free(EXT_ETH_RESET_GPIO);
	}
	return ret;
}

struct smsc911x_platform_config ccimx53_smsc9118 = {
	.flags          = SMSC911X_USE_32BIT,
	.irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type       = SMSC911X_IRQ_TYPE_PUSH_PULL,    /* push-pull irq */
	.init 			= ccimx53_smsc911x_init,
};

static struct resource smsc911x_device_resources[] = {
        {
		.name	= "smsc911x-memory",
		.start	= MX53_CS1_BASE_ADDR,
		.end	= MX53_CS1_BASE_ADDR + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= gpio_to_irq(CCIMX53_EXT_IRQ_GPIO),
		.end	= gpio_to_irq(CCIMX53_EXT_IRQ_GPIO),
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
#define CSWCR2	0x14

void ccimx53_register_ext_eth(void)
{
	__iomem void *weim_vbaddr, *iomux_vbaddr;
	u32 reg;

	gpio_smsc911x_active();

	weim_vbaddr = ioremap(MX53_BASE_ADDR(WEIM_BASE_ADDR), SZ_4K);
	if (weim_vbaddr == 0) {
		printk(KERN_ERR "Unable to ioremap 0x%08x in %s\n", WEIM_BASE_ADDR, __func__);
		return;
	}

	iomux_vbaddr = ioremap(MX53_BASE_ADDR(IOMUXC_BASE_ADDR), SZ_4K);
	if (iomux_vbaddr == 0) {
		printk(KERN_ERR "Unable to ioremap 0x%08x in %s\n", IOMUXC_BASE_ADDR, __func__);
		goto unmap_weim;
	}

	/** Configure the CS timing, bus width, etc.
	 * 16 bit on DATA[31..16], not multiplexed, async
	 * RWSC=8, RADVA=0, RADVN=0, OEA=0, OEN=0, RCSA=0, RCSN=0, APR=0
	 * WAL=0, WBED=1, WWSC=15, WADVA=0, WADVN=0, WEA=7, WEN=4, WCSA=0
	 */
	__raw_writel(0x00420081, (u32)weim_vbaddr + 0x18 + CSGCR1);
	__raw_writel(0, (u32)weim_vbaddr + 0x18 + CSGCR2);
	__raw_writel(0x08000000, (u32)weim_vbaddr + 0x18 + CSRCR1);
	__raw_writel(0, (u32)weim_vbaddr + 0x18 + CSRCR2);
	__raw_writel(0x4f000f00, (u32)weim_vbaddr + 0x18 + CSWCR1);
	__raw_writel(0, (u32)weim_vbaddr + 0x18 + CSWCR2);

	/* Configure CS1 iomem with 32MB length */
	reg = __raw_readl(iomux_vbaddr + 0x4);
	reg &= ~0x3f;
	reg |= 0x1b;
	__raw_writel(reg, (u32)iomux_vbaddr + 0x4);

	mxc_register_device(&smsc911x_device, &ccimx53_smsc9118);

	iounmap(iomux_vbaddr);
unmap_weim:
	iounmap(weim_vbaddr);
}
#else
void ccimx53_register_ext_eth(void) {}
#endif

static struct i2c_board_info ccimx53_i2c_devices[] __initdata = {
#if defined(CONFIG_INPUT_MMA7455L) || defined(CONFIG_INPUT_MMA7455L_MODULE)
	{
		I2C_BOARD_INFO("mma7455l", 0x1d),
		.irq = gpio_to_irq(CCIMX53_MMA7455_IRQ_GPIO),
	},
#endif
#if defined(CONFIG_SND_SOC_IMX_CCIMX53_SGTL5000) || defined(CONFIG_SND_SOC_IMX_CCIMX53_SGTL5000_MODULE)
	{
		I2C_BOARD_INFO("sgtl5000-i2c", 0x0A),
	},
#endif
#if defined (CONFIG_MXC_CAMERA_MICRON111_1) || defined(CONFIG_MXC_CAMERA_MICRON111_1_MODULE)
	{
		I2C_BOARD_INFO("mt9v111_1", 0xB8>>1 /*0x5C*/),
	},
#endif
#if defined (CONFIG_MXC_CAMERA_MICRON111_2) || defined(CONFIG_MXC_CAMERA_MICRON111_2_MODULE)
	{
		I2C_BOARD_INFO("mt9v111_2", 0x90>>1 /*0x48*/),
	},
#endif
};

int __init ccimx53_init_i2c_devices(void)
{
#if defined(CONFIG_INPUT_MMA7455L) || defined(CONFIG_INPUT_MMA7455L_MODULE)
	/* Accelerometer interrupt line configuration */
	mxc_iomux_v3_setup_pad(MX53_PAD_GPIO_14__GPIO4_4);
	gpio_request(CCIMX53_MMA7455_IRQ_GPIO, "accelerometer_irq");
	gpio_direction_input(CCIMX53_MMA7455_IRQ_GPIO);
#endif

	return i2c_register_board_info(2, ccimx53_i2c_devices , ARRAY_SIZE(ccimx53_i2c_devices) );
}

#if (defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE))
#if defined(CONFIG_CCIMX5X_SECOND_TOUCH)
static int touch_pendown_state(void)
{
	return gpio_get_value(SECOND_TS_IRQ_PIN) ? 0 : 1;
}

static struct ads7846_platform_data ccimx53js_touch_data = {
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

static struct spi_board_info ccimx53_2nd_touch[] = {
	{
		.modalias	= "ads7846",
		.max_speed_hz	= 500000,
		.irq		= gpio_to_irq(SECOND_TS_IRQ_PIN),
		.bus_num        = 1,
		.chip_select    = 3,
		.platform_data	= &ccimx53js_touch_data,
	},
};

void ccimx53_init_2nd_touch(void)
{
	spi_register_board_info(ccimx53_2nd_touch, ARRAY_SIZE(ccimx53_2nd_touch));
}
#else
void ccimx53_init_2nd_touch(void) {}
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

void ccimx53_init_spidevices(void)
{
	spi_register_board_info(spi_devices, ARRAY_SIZE(spi_devices));
}
#else
void ccimx53_init_spidevices(void) { }
#endif

#if (defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE))
struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = ccimx53_gpio_spi_chipselect_active,
	.chipselect_inactive = ccimx53_gpio_spi_chipselect_inactive,
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
	.bank_start = MXC_IIM_MX53_BANK_START_ADDR,
	.bank_end   = MXC_IIM_MX53_BANK_END_ADDR,
	.enable_fuse = mxc_iim_enable_fuse,
	.disable_fuse = mxc_iim_disable_fuse,
};
#endif /* CONFIG_MXC_IIM */

#if defined(CONFIG_CAN_FLEXCAN) || defined(CONFIG_CAN_FLEXCAN_MODULE)
static struct flexcan_platform_data flexcan0_data = {
	.core_reg = NULL,
	.io_reg = NULL,
	.root_clk_id = "lp_apm", /*lp_apm is 24MHz */
	.xcvr_enable = NULL,
	.br_clksrc = 0,
	.br_rjw = 2,
	.br_presdiv = 3,
	.br_propseg = 2,
	.br_pseg1 = 3,
	.br_pseg2 = 3,
	.bcc = 1,
	.srx_dis = 1,
	.smp = 1,
	.boff_rec = 1,
	.berr_reporting = 1,
	.ext_msg = 1,
	.std_msg = 1,
};
static struct flexcan_platform_data flexcan1_data = {
	.core_reg = NULL,
	.io_reg = NULL,
	.root_clk_id = "lp_apm", /*lp_apm is 24MHz */
	.xcvr_enable = NULL,
	.br_clksrc = 0,
	.br_rjw = 2,
	.br_presdiv = 3,
	.br_propseg = 2,
	.br_pseg1 = 3,
	.br_pseg2 = 3,
	.bcc = 1,
	.srx_dis = 1,
	.boff_rec = 1,
	.berr_reporting = 1,
	.ext_msg = 1,
	.std_msg = 1,
};

void ccimx53_register_can(int interface)
{
	/* Configure GPIOs */
	gpio_can_active(interface);

	/* Register interface */
	switch (interface) {
#ifdef CONFIG_CCIMX53_CAN1
	case 0:
		mxc_register_device(&mxc_flexcan0_device, &flexcan0_data);
		break;
#endif /* CONFIG_CCIMX53_CAN1 */
#ifdef CONFIG_CCIMX53_CAN2
	case 1:
		mxc_register_device(&mxc_flexcan1_device, &flexcan1_data);
		break;
#endif /* CONFIG_CCIMX53_CAN2 */
	}
}
#else
void ccimx53_register_can(int interface) {}
#endif /* CONFIG_CAN_FLEXCAN */

#if defined(CONFIG_SATA_AHCI_PLATFORM)
void ccimx53_register_sata(void)
{
	/* SATA uses dedicated pins so, no special IOMUX configuration needed.
	 * Just, register the device */
	mxc_register_device(&ahci_fsl_device, &sata_data);
}
#else
void ccimx53_register_sata(void) {}
#endif /* CONFIG_SATA_AHCI_PLATFORM */

#if defined (CONFIG_CCIMX5X_FUSION_4_MULTITOUCH) || \
	defined(CONFIG_CCIMX5X_FUSION_7_10_MULTITOUCH)
struct i2c_board_info ccimx53_fusion_ts[] __initdata = {
#ifdef CONFIG_CCIMX5X_FUSION_4_MULTITOUCH
	{
		/* Fusion 4 */
		I2C_BOARD_INFO("fusion_F04B", 0x05),
		.irq = gpio_to_irq(SECOND_TS_IRQ_PIN),
	},
#endif
#ifdef CONFIG_CCIMX5X_FUSION_7_10_MULTITOUCH
	{
		/* Fusion 7 and Fusion 10 */
		I2C_BOARD_INFO("fusion", 0x10),
		.irq = gpio_to_irq(SECOND_TS_IRQ_PIN),
	},
#endif
};

__init void ccimx53_register_fusion_touch(void)
{
	/* Configure external touch interrupt line and register the i2c device */
	mxc_iomux_v3_setup_pad(MX53_PAD_CSI0_DAT10__GPIO5_28);
	gpio_request(SECOND_TS_IRQ_PIN, "fusion_ts_irq");
	gpio_direction_input(SECOND_TS_IRQ_PIN);

	i2c_register_board_info(2, ccimx53_fusion_ts, ARRAY_SIZE(ccimx53_fusion_ts));
}
#else
void ccimx53_register_fusion_touch(void) {}
#endif

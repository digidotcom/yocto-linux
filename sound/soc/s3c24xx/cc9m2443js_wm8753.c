/* -*- linux-c -*-
 *
 * cc9m2443js_wm8753.c  --  SoC audio for the Digi CC9M2443JS
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Based on the driver of the Neo1973
 *
 *  IMPORTANT: In difference to the Neo1973, we are using the S3C2443 as master
 *  and not as slave. That means, the configuration of the codec is quite different
 *  (Luis Galdos)
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/math64.h>
#include <linux/clk.h>

#include <asm/mach-types.h>
#include <asm/hardware/scoop.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <mach/hardware.h>
#include <plat/audio.h>
#include <mach/spi-gpio.h>
#include <sound/pcm_params.h>

#include <plat/regs-iis.h>

#include "../codecs/wm8753.h"
#include "lm4857.h"
#include "s3c-dma.h"
#include "s3c2412-i2s.h"

/* Additional info and error messages */
#define printk_err(fmt, args...)                printk(KERN_ERR "[ ERROR ] cc9m2443-snd: " fmt, ## args)
#define printk_info(fmt, args...)               printk(KERN_INFO "cc9m2443-snd: " fmt, ## args)

#if 0
#define CC9M2443JS_SND_DEBUG
#endif

#ifdef CC9M2443JS_SND_DEBUG
#  define printk_debug(fmt, args...)            printk(KERN_DEBUG "cc9m2443-snd: %s() " fmt, __func__ , ## args)
#else
#  define printk_debug(fmt, args...)
#endif

static int cc9m2443js_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int pll_out = 0, bclk = 0;
	int ret = 0;
	unsigned long bitfs, is16bit;
	struct s3c_i2sv2_rate_calc info;

	switch (params_rate(params)) {
	case 8000:
	case 16000:
		pll_out = 12288000;
		break;
	case 48000:
		bclk = WM8753_BCLK_DIV_4;
		pll_out = 12288000;
		break;
	case 96000:
		bclk = WM8753_BCLK_DIV_2;
		pll_out = 12288000;
		break;
	case 11025:
		bclk = WM8753_BCLK_DIV_16;
		pll_out = 11289600;
		break;
	case 22050:
		bclk = WM8753_BCLK_DIV_8;
		pll_out = 11289600;
		break;
	case 44100:
		bclk = WM8753_BCLK_DIV_4;
		pll_out = 11289600;
		break;
	case 88200:
		bclk = WM8753_BCLK_DIV_2;
		pll_out = 11289600;
		break;
	}

	/* cpu_iis_mclk = iis_clkrate / params_rate(params); */
	s3c_i2sv2_iis_calc_rate(&info, NULL, params_rate(params), s3c_i2sv2_get_clock(cpu_dai));
	
	printk_debug("Sample rate %i | Master Clk %lu Hz | FS %u | Clk divisor %u\n",
		     params_rate(params), clk_get_rate(s3c_i2sv2_get_clock(cpu_dai)),
		     info.fs_div, info.clk_div);

	/* REQUIRED: Set codec DAI configuration (normal bclk and frm ) and as slave */
	ret = snd_soc_dai_set_fmt(codec_dai,
					 SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
					 SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* REQUIRED: Set cpu DAI configuration (normal bclk and frm) and CPU as master */
	ret = snd_soc_dai_set_fmt(cpu_dai , SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* REQUIRED: Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8753_MCLK, pll_out,
					    SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		printk_err("set_sysclk() failed (%i)\n", ret);
		return ret;
	}

	/* REQUIRED: Set MCLK division for sample rate (256, 384, 512, 768) */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C2412_DIV_RCLK, info.fs_div);
	if (ret < 0)
		return ret;
	
	/*
	 * Set the BCLK configuration for the CPU
	 * The required value depends on the configured root clock (RFS) according
	 * to the table 25-2 of the data sheet of the S3C2443
	 * @XXX: Define the corresponding macros for the Bit FS selection
	 */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_U8:
	case SNDRV_PCM_FORMAT_S8:
		is16bit = 0;
		break;
	case SNDRV_PCM_FORMAT_U16_LE:
	case SNDRV_PCM_FORMAT_S16_LE:
		is16bit = 1;
		break;
	default:
		return -EINVAL;
	}

	switch (info.fs_div) {
	case 256:
		bitfs = (is16bit) ? 32 : 16;
		break;
	case 512:
		bitfs = (is16bit) ? 32 : 16;
		break;
	case 384:
		bitfs = (is16bit) ? 32 : 16;
		break;
	case 768:
		bitfs = (is16bit) ? 32 : 16;
		break;
	default:
		printk_err("Unsupported root clock FS %i divisor found\n", info.fs_div);
		ret = -EINVAL;
		goto exit_all;
		break;
	}
	
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C2412_DIV_BCLK, bitfs);
	if (ret) {
		printk_err("Couldn't set the BLCK, %i\n", ret);
		return ret;
	}
	
	/* Set the prescaler divisor for the master clock */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C2412_DIV_PRESCALER,
					  info.clk_div);
	if (ret < 0) {
		printk_err("Prescaler setting failed, %i\n", ret);
		return ret;
	}

 exit_all:
	return ret;
}

static int cc9m2443js_hifi_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;

	/* disable the PLL */
	return snd_soc_dai_set_pll(codec_dai, WM8753_PLL1, 0, 0, 0);
}

static int cc9m2443js_hifi_startup(struct snd_pcm_substream *substream)
{
	printk_debug("Going to startup the codec\n");
	
	return 0;
}

/* We don't need this function, but we know that they are available */
static void cc9m2443js_hifi_shutdown(struct snd_pcm_substream *substream)
{
	printk_debug("Going to shutdown the codec\n");
}

/*
 * WM8753 HiFi DAI operations.
 */
static struct snd_soc_ops cc9m2443js_hifi_ops = {
	.hw_params = cc9m2443js_hifi_hw_params,
	.hw_free = cc9m2443js_hifi_hw_free,
	.startup = cc9m2443js_hifi_startup,
	.shutdown = cc9m2443js_hifi_shutdown,
};

/*
 * Structure for the interface with the DAIs of the codec and CPU
 * The two below elements will be probed by the same I2S-driver
 */
static struct snd_soc_dai_link cc9m2443js_dai[] = {
	{
		/*
		 * Hifi Playback
		 * Infos under: proc/asound/card0/pcm0p/info
		 */
		.name = "WM8753",
		.stream_name = "WM8753 HiFi",
		.cpu_dai = &s3c2443_i2s_dai,
		.codec_dai = &wm8753_dai[WM8753_DAI_HIFI],
		/* .init = cc9m2443js_wm8753_snd_init, */
		.ops = &cc9m2443js_hifi_ops,
	},{
		/*
		 * Voice
		 * Infos under: proc/asound/card0/pcm1c/info
		 */
		.name = "Inline",
		.stream_name = "WM8753 Inline",
		.cpu_dai = &s3c2443_i2s_dai,
		.codec_dai = &wm8753_dai[WM8753_DAI_HIFI],
		.ops = &cc9m2443js_hifi_ops,
	}
};

/* This is the machine-structure */
static struct snd_soc_card cc9m2443js_snd = {
	.name = "cc9m2443js",
	.platform = &s3c24xx_soc_platform,
	.dai_link = cc9m2443js_dai,
	.num_links = ARRAY_SIZE(cc9m2443js_dai),
};

static struct snd_soc_device cc9m2443js_snd_devdata = {
	.card = &cc9m2443js_snd,
	.codec_dev = &soc_codec_dev_wm8753,
};

static struct platform_device *cc9m2443js_snd_device;

static int __init cc9m2443js_wm8753_init(void)
{
	int ret;

	cc9m2443js_snd_device = platform_device_alloc("soc-audio", -1);
	if (!cc9m2443js_snd_device)
		return -ENOMEM;

	platform_set_drvdata(cc9m2443js_snd_device, &cc9m2443js_snd_devdata);
	cc9m2443js_snd_devdata.dev = &cc9m2443js_snd_device->dev;
	ret = platform_device_add(cc9m2443js_snd_device);
	if (ret)
		platform_device_put(cc9m2443js_snd_device);

	return ret;
}

static void __exit cc9m2443js_wm8753_exit(void)
{
	platform_device_unregister(cc9m2443js_snd_device);
}

module_init(cc9m2443js_wm8753_init);
module_exit(cc9m2443js_wm8753_exit);

/* Module information */
MODULE_AUTHOR("Graeme Gregory, graeme@openmoko.org, www.openmoko.org");
MODULE_AUTHOR("Luis Galdos, luis.galdos[at]digi.com");
MODULE_DESCRIPTION("ALSA SoC WM8753 Digi CC9M2443JS");
MODULE_LICENSE("GPL");

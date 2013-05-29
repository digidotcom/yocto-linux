/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2011 Digi International, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
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
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/ahci_platform.h>
#include <linux/gpio_keys.h>
#include <linux/mfd/da9052/da9052.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/keypad.h>
#include <asm/mach/flash.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include <mach/iomux-mx53.h>
#include <mach/i2c.h>
#include <mach/mxc_iim.h>
#include <mach/iomux-mx53.h>

#include "devices_ccimx53.h"
#include "devices_ccimx5x.h"
#include "crm_regs.h"
#include "devices.h"
#include "usb.h"
#include "board-ccimx53.h"
#include "linux/fsl_devices.h"

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
#include <asm/mach/bootldr_shmem.h>
extern void ccimx53_set_lcd_struct_from_bl_shared(nv_lcd_config_t *lcd_config);
#endif

#define TZIC_WAKEUP0_OFFSET	(0x0E00)
#define TZIC_WAKEUP1_OFFSET	(0x0E04)
#define TZIC_WAKEUP2_OFFSET	(0x0E08)
#define TZIC_WAKEUP3_OFFSET	(0x0E0C)
#define GPIO7_0_11_IRQ_BIT	(0x1<<11)

/*!
 * @file mach-mx5/mx53_ccimx53js.c
 *
 * @brief This file contains MX53 ccimx53js board specific initialization routines.
 *
 * @ingroup MSL_MX53
 */

/* MX53 LOCO GPIO PIN configurations */
#define MX53_nONKEY			(0*32 + 8)	/* GPIO_1_8 */

u8 ccimx51_swap_bi = 1;
/* Global used to differentiate variants at early boot */
int ccimx5x_total_mem, ccimx5x_left_mem;

extern int __init mx53_ccimx53js_init_da9052(void);
extern void gpio_dio_active(void);
extern void gpio_usb_active(void);
extern void pm_i2c_init(u32 base_addr);
extern int ccimx53_pm_da9053_mask_irqs(void);
extern int ccimx53_pm_da9053_unmask_irqs(void);

static iomux_v3_cfg_t mx53_ccimx53js_pads[] = {
#if defined(CONFIG_I2C_MX_SELECT1) || defined(CONFIG_I2C_MX_SELECT1_MODULE)
	/* I2C1, unconnected in JSK, conflicts with Camera1 and Camera2 */
	/* Alternative pins conflict with SMSC */
#if !defined (CONFIG_MXC_CAMERA_MICRON111_1) && \
	!defined(CONFIG_MXC_CAMERA_MICRON111_1_MODULE)
	MX53_PAD_CSI0_DAT9__I2C1_SCL,
#endif
#if !defined (CONFIG_MXC_CAMERA_MICRON111_2) &&	\
	!defined(CONFIG_MXC_CAMERA_MICRON111_2_MODULE)
	MX53_PAD_CSI0_DAT8__I2C1_SDA,
#endif
#endif
#if defined(CONFIG_I2C_MX_SELECT2) || defined(CONFIG_I2C_MX_SELECT2_MODULE)
	/* I2C2, unconnected in JSK */
	MX53_PAD_KEY_COL3__I2C2_SCL,
	MX53_PAD_KEY_ROW3__I2C2_SDA,
#endif
#if defined(CONFIG_I2C_MX_SELECT3) || defined(CONFIG_I2C_MX_SELECT3_MODULE)
	/* I2C3, connected to the DA9053, MMA7455, SGTL5000 */
	MX53_PAD_GPIO_5__I2C3_SCL,
	MX53_PAD_GPIO_6__I2C3_SDA,
#endif
};

static iomux_v3_cfg_t ccimx53js_keys_leds_pads[] = {
	USER_LED1_PAD,
	USER_LED2_PAD,
	USER_KEY1_PAD,
	USER_KEY2_PAD,
};

extern void mx5_ipu_reset(void);
static struct mxc_ipu_config mxc_ipu_data = {
	.rev = 3,
	.reset = mx5_ipu_reset,
};

extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data mxc_vpu_data = {
#ifdef CONFIG_MXC_VPU_IRAM
	.iram_enable = true,
	.iram_size = CONFIG_MXC_VPU_IRAM_SIZE,
#else
	.iram_enable = false,
#endif
	.reset = mx5_vpu_reset,
};

static struct mxc_dvfs_platform_data dvfs_core_data = {
	.reg_id = "DA9052_BUCK_CORE",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 30,
};

static struct mxc_bus_freq_platform_data bus_freq_data = {
	.gp_reg_id = "DA9052_BUCK_CORE",
	.lp_reg_id = "DA9052_BUCK_PRO",
};

/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	struct tag *t;
	struct tag *mem_tag = 0;
	int total_mem = SZ_512M;
	int left_mem = 0;
	int gpu_mem = GPU_MEM_SIZE;
	int fb_mem = FB_MEM_SIZE;
	char *str;

	mxc_set_cpu_type(MXC_CPU_MX53);

	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
			if (total_mem == SZ_128M && gpu_mem)
				gpu_mem = SZ_16M;
			break;
		}
	}

	ccimx5x_total_mem = total_mem;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "mem=");
			if (str != NULL) {
				str += 4;
				left_mem = memparse(str, &str);
			}
			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_nommu");
			if (str != NULL)
				gpu_data.enable_mmu = 0;

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				gpu_mem = memparse(str, &str);
			}

			break;
		}
	}

	if (gpu_data.enable_mmu)
		gpu_mem = 0;

	if (left_mem == 0 || left_mem > total_mem)
		left_mem = total_mem - gpu_mem - fb_mem;

	ccimx5x_left_mem = left_mem;

	if (mem_tag) {
		fb_mem = total_mem - left_mem - gpu_mem;
		if (fb_mem < 0) {
			gpu_mem = total_mem - left_mem;
			fb_mem = 0;
		}
		mem_tag->u.mem.size = left_mem;
		/*reserve memory for gpu*/
		if (!gpu_data.enable_mmu) {
			gpu_device.resource[5].start =
				mem_tag->u.mem.start + left_mem;
			gpu_device.resource[5].end =
				gpu_device.resource[5].start + gpu_mem - 1;
		}
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		if (fb_mem) {
#if defined(CONFIG_CCIMX5X_DISP0) && defined(CONFIG_CCIMX5X_DISP1)
			fb_mem = fb_mem / 2;	/* Divide the mem for between the displays */
#endif
			mxcfb_resources[0].start =
				gpu_data.enable_mmu ?
				mem_tag->u.mem.start + left_mem :
				gpu_device.resource[5].end + 1;
			mxcfb_resources[0].end =
				mxcfb_resources[0].start + fb_mem - 1;
#if defined(CONFIG_CCIMX5X_DISP0) && defined(CONFIG_CCIMX5X_DISP1)
			mxcfb_resources[1].start =
				mxcfb_resources[0].end + 1;
			mxcfb_resources[1].end =
				mxcfb_resources[1].start + fb_mem - 1;
#endif
		} else {
			mxcfb_resources[0].start = 0;
			mxcfb_resources[0].end = 0;
			mxcfb_resources[1].start = 0;
			mxcfb_resources[1].end = 0;
		}
#endif
	}

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
       defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
	ccimx53_set_lcd_struct_from_bl_shared(
		(nv_lcd_config_t *)((unsigned int)desc->boot_params + BL_SHARED_RAM_OFFS_LCD));
#endif
}

static void __init mx53_ccimx53js_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx53_ccimx53js_pads,
					ARRAY_SIZE(mx53_ccimx53js_pads));
	mxc_iomux_v3_setup_multiple_pads(ccimx53js_keys_leds_pads,
					 ARRAY_SIZE(ccimx53js_keys_leds_pads));
#if defined(CONFIG_HAS_EARLY_USER_LEDS)
	/* Configure leds direction */
	gpio_direction_output(USER_LED1_GPIO, 0);
	gpio_direction_output(USER_LED2_GPIO, 0);
#endif
	gpio_wireless_active();
	gpio_dio_active();
	gpio_usb_active();
}

static void ccimx53_suspend_enter(void)
{
	ccimx53_pm_da9053_mask_irqs();
}

static void ccimx53_suspend_exit(void)
{
	ccimx53_pm_da9053_unmask_irqs();
}

static struct mxc_pm_platform_data ccimx53_pm_data = {
	.suspend_enter = ccimx53_suspend_enter,
	.suspend_exit = ccimx53_suspend_exit,
};

#if defined(CONFIG_I2C_IMX_RECOVER_FROM_STUCK)
#define I2C_RECOVERY_MAX_CLOKS		50
/* Generate a pulse on the i2c clock pin. */
void i2c_recover_from_stuck(void)
{
	int i = 0, i2c_ready = 1;

	gpio_request(CCIMX53_I2C3_SCL_GPIO, "i2c-2_clk");
	gpio_request(CCIMX53_I2C3_SDA_GPIO, "i2c-2_sda");

	/* Reconfigure pins as gpios */
	gpio_direction_input(CCIMX53_I2C3_SCL_GPIO);
	mxc_iomux_v3_setup_pad(MX53_PAD_GPIO_5__GPIO1_5);
	gpio_direction_input(CCIMX53_I2C3_SDA_GPIO);
	mxc_iomux_v3_setup_pad(MX53_PAD_GPIO_6__GPIO1_6);

	/* First of all, verify that both i2c lines are ready (pulled up) */
	if (gpio_get_value(CCIMX53_I2C3_SCL_GPIO) != 1)
		i2c_ready = 0;

	if (gpio_get_value(CCIMX53_I2C3_SDA_GPIO) != 1)
		i2c_ready = 0;

	/* If not, try to recover by clocking the bus (bitbanging) */
	if (!i2c_ready) {
		for (i = 0; i < I2C_RECOVERY_MAX_CLOKS; i++) {
			if (gpio_get_value(CCIMX53_I2C3_SDA_GPIO) == 1)
				break;

			gpio_direction_output(CCIMX53_I2C3_SCL_GPIO, 0);
			udelay(50);
			gpio_direction_input(CCIMX53_I2C3_SCL_GPIO);
			udelay(50);
		}
	}

	if (!i2c_ready && i != I2C_RECOVERY_MAX_CLOKS) {
		printk(KERN_INFO "%s: successfully recovered!\n", __func__);
	}

	/* Free gpios adn go back to i2c operational mode */
	gpio_free(CCIMX53_I2C3_SCL_GPIO);
	gpio_free(CCIMX53_I2C3_SDA_GPIO);
	mxc_iomux_v3_setup_pad(MX53_PAD_GPIO_5__I2C3_SCL);
	mxc_iomux_v3_setup_pad(MX53_PAD_GPIO_6__I2C3_SDA);
}
#endif /* CONFIG_I2C_IMX_RECOVER_FROM_STUCK */

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	/* Setup hwid information, passed through Serial ATAG */
	ccimx5x_set_mod_variant(system_serial_low & 0xff);
	ccimx5x_set_mod_revision((system_serial_low >> 8) & 0xff);
	ccimx5x_set_mod_sn(((system_serial_low << 8) & 0xff000000) |
			   ((system_serial_low >> 8) & 0x00ff0000) |
			   ((system_serial_high << 8) & 0x0000ff00) |
			   ((system_serial_high >> 8) & 0xff));

	mxc_ipu_data.di_clk[0] = clk_get(NULL, "ipu_di0_clk");
	mxc_ipu_data.di_clk[1] = clk_get(NULL, "ipu_di1_clk");
	mxc_ipu_data.csi_clk[0] = clk_get(NULL, "ssi_ext1_clk");
	mxc_ipu_data.csi_clk[1] = clk_get(NULL, "ssi_ext1_clk");

	mxc_cpu_common_init();
	mx53_ccimx53js_io_init();

	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);

#if defined(CONFIG_I2C_MX_SELECT1) || defined(CONFIG_I2C_MX_SELECT1_MODULE)
	mxc_register_device(&mxci2c_devices[0], &mxci2c1_data);
#endif
#if defined(CONFIG_I2C_MX_SELECT2) || defined(CONFIG_I2C_MX_SELECT2_MODULE)
	mxc_register_device(&mxci2c_devices[1], &mxci2c2_data);
#endif
#if defined(CONFIG_I2C_MX_SELECT3) || defined(CONFIG_I2C_MX_SELECT3_MODULE)
	mxc_register_device(&mxci2c_devices[2], &mxci2c3_data);
#endif

	mx53_ccimx53js_init_da9052();

	mxc_register_device(&mxc_ipu_device, &mxc_ipu_data);
	mxc_register_device(&mxcvpu_device, &mxc_vpu_data);
	mxc_register_device(&gpu_device, &gpu_data);
	mxc_register_device(&mxcscc_device, NULL);
	mxc_register_device(&pm_device, &ccimx53_pm_data);
#if defined(CONFIG_PMIC_DA9052)
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
#endif
	mxc_register_device(&busfreq_device, &bus_freq_data);
	mxc_register_device(&mxc_iim_device, &iim_data);
	mxc_register_device(&mxc_v4l2_device, NULL);
	mxc_register_device(&mxc_v4l2out_device, NULL);
#if defined(CONFIG_SPI_MXC_SELECT1) || defined(CONFIG_SPI_MXC_SELECT1_MODULE)
	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
#endif
	ccimx53_init_spidevices();
//	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);

	ccimx53_register_sdio(1);
#ifdef CONFIG_ESDHCI_MXC_SELECT2
	ccimx53_register_sdio(2);
#endif
#ifdef CONFIG_ESDHCI_MXC_SELECT3
	ccimx53_register_sdio(3);
#endif
#ifdef CONFIG_ESDHCI_MXC_SELECT4
	ccimx53_register_sdio(4);
#endif
	ccimx53_register_nand();
	ccimx53_register_sata();
	ccimx53_register_fec();
	ccimx53_register_ext_eth();
	ccimx53_register_sgtl5000();
	ccimx5x_init_fb();
	ccimx53_init_i2c_devices();
	ccimx53_register_can(0);
	ccimx53_register_can(1);

	mx5_usb_dr_init();
#if defined(CONFIG_USB_EHCI_ARC_H1) || defined(CONFIG_USB_EHCI_ARC_H1_MODULE)
	mx5_usbh1_init();
#endif

#ifdef CONFIG_CCIMX5X_SECOND_TOUCH
	ccimx53_init_2nd_touch();
#endif
	ccimx53_register_fusion_touch();
	ccimx5x_create_sysfs_entries();

#if defined (CONFIG_PMIC_DA9052)
	pm_power_off = da9053_power_off;
#endif
	pm_i2c_init(I2C3_BASE_ADDR - MX53_OFFSET);
}

static void __init mx53_ccimx53js_timer_init(void)
{
	struct clk *uart_clk;

	mx53_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("mxcintuart.0", NULL);

	if (CONSOLE_UART_BASE_ADDR)
		early_console_setup(MX53_BASE_ADDR(CONSOLE_UART_BASE_ADDR), uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx53_ccimx53js_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_CCIMX53 data structure.
 */

MACHINE_START(CCWMX53JS, "ConnectCore Wi-i.MX53"BOARD_NAME)
	/* Maintainer: Digi International, Inc. */
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END

MACHINE_START(CCMX53JS, "ConnectCore i.MX53"BOARD_NAME)
	/* Maintainer: Digi International, Inc. */
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END

MACHINE_START(CCIMX53JS, "ConnectCore for MX53"BOARD_NAME)
	/* Maintainer: Digi International, Inc. */
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END

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
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
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

#include "board-ccimx51.h"
#include "iomux.h"
#include "crm_regs.h"
#include "devices.h"
#include "mx51_pins.h"
#include "devices_ccimx51.h"
#include "devices_ccimx5x.h"
#include "usb.h"

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
#include <asm/mach/bootldr_shmem.h>
extern void ccimx51_set_lcd_struct_from_bl_shared(nv_lcd_config_t *lcd_config);
#endif

extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);
extern struct dvfs_wp *(*get_dvfs_core_wp)(int *wp);
static int num_cpu_wp = 3;
u8 ccimx51_swap_bi = 0;

/* Global used to differentiate variants at early boot */
int ccimx5x_total_mem, ccimx5x_left_mem;

static struct dvfs_wp dvfs_core_setpoint[] = {
	{33, 13, 33, 10, 10, 0x08}, /* 800MHz*/
	{28, 8, 33, 10, 10, 0x08},   /* 400MHz */
	{20, 0, 33, 20, 10, 0x08},   /* 160MHz*/
	{28, 8, 33, 20, 30, 0x08},   /*160MHz, AHB 133MHz, LPAPM mode*/
	{29, 0, 33, 20, 10, 0x08},}; /* 160MHz, AHB 24MHz */

/* working point(wp): 0 - 800MHz; 1 - 166.25MHz; */
static struct cpu_wp cpu_wp_auto_800[] = {
	{
	 .pll_rate = 1000000000,
	 .cpu_rate = 1000000000,
	 .pdf = 0,
	 .mfi = 10,
	 .mfd = 11,
	 .mfn = 5,
	 .cpu_podf = 0,
	 .cpu_voltage = 1175000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1100000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 166250000,
	 .pdf = 4,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 4,
	 .cpu_voltage = 900000,},
};

static struct cpu_wp cpu_wp_auto_600[] = {
	{
	 .pll_rate = 600000000,
	 .cpu_rate = 600000000,
	 .pdf = 0,
	 .mfi = 6,
	 .mfd = 3,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1000000,},
	{
	 .pll_rate = 600000000,
	 .cpu_rate = 150000000,
	 .pdf = 3,
	 .mfi = 6,
	 .mfd = 3,
	 .mfn = 1,
	 .cpu_podf = 3,
	 .cpu_voltage = 950000,},
};

static u32 ccimx51_get_cpu_freq(void)
{
	u32 cpu_freq = 800000000;

	switch (system_serial_low & 0xff) {
	case 0x04:
	case 0x05:
	case 0x0a:
	case 0x0b:
	case 0x0f:
	case 0x10:
	case 0x12:
	case 0x15:
	case 0x17:
	case 0x19:
		cpu_freq = 600000000;
		num_cpu_wp = 2;
		break;
	}

	return cpu_freq;
}

struct cpu_wp *mx51_get_cpu_wp(int *wp)
{
	u32 cpu_clk_rate = ccimx51_get_cpu_freq();

	*wp = num_cpu_wp;

	if (cpu_clk_rate == 800000000) {
		return cpu_wp_auto_800;
	} else if (cpu_clk_rate == 600000000) {
		return cpu_wp_auto_600;
	}
	return NULL;
}

void mx51_set_num_cpu_wp(int num)
{
	num_cpu_wp = num;
	return;
}

static struct mxc_dvfs_platform_data dvfs_core_data = {
	.reg_id = "SW1",
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

static struct mxc_dvfsper_data dvfs_per_data = {
	.reg_id = "SW2",
	.clk_id = "gpc_dvfs_clk",
	.gpc_cntr_reg_addr = MXC_GPC_CNTR,
	.gpc_vcr_reg_addr = MXC_GPC_VCR,
	.gpc_adu = 0x0,
	.vai_mask = MXC_DVFSPMCR0_FSVAI_MASK,
	.vai_offset = MXC_DVFSPMCR0_FSVAI_OFFSET,
	.dvfs_enable_bit = MXC_DVFSPMCR0_DVFEN,
	.irq_mask = MXC_DVFSPMCR0_FSVAIM,
	.div3_offset = 0,
	.div3_mask = 0x7,
	.div3_div = 2,
	.lp_high = 1250000,
	.lp_low = 1250000,
};

static struct dvfs_wp *mx51_get_dvfs_core_table(int *wp)
{
	*wp = ARRAY_SIZE(dvfs_core_setpoint);
	return dvfs_core_setpoint;
}

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
	char *str;
	struct tag *t;
	struct tag *mem_tag = 0;
	int total_mem = SZ_512M;
	int left_mem = 0;
	int gpu_mem = GPU_MEM_SIZE;
	int fb_mem = FB_MEM_SIZE;

	mxc_set_cpu_type(MXC_CPU_MX51);

	get_cpu_wp = mx51_get_cpu_wp;
	set_num_cpu_wp = mx51_set_num_cpu_wp;
	get_dvfs_core_wp = mx51_get_dvfs_core_table;

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
#if defined(CONFIG_CCIMX5X_DISP0) && defined(CONFIG_CCIMX5X_DISP1)
		fb_mem = fb_mem / 2;	/* Divide the mem for between the displays */
#endif
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
	ccimx51_set_lcd_struct_from_bl_shared(
		(nv_lcd_config_t *)((unsigned int)desc->boot_params + BL_SHARED_RAM_OFFS_LCD));
#endif
}

#define PWGT1SPIEN (1<<15)
#define PWGT2SPIEN (1<<16)
#define USEROFFSPI (1<<3)

static void mxc_power_off(void)
{
	/* We can do power down one of two ways:
	   Set the power gating
	   Set USEROFFSPI */

	/* Set the power gate bits to power down */
#ifdef CONFIG_MXC_PMIC_MC13892
	pmic_write_reg(REG_POWER_MISC, (PWGT1SPIEN|PWGT2SPIEN),
		(PWGT1SPIEN|PWGT2SPIEN));
#endif
}

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	int cpu_model;

	/* Setup hwid information, passed through Serial ATAG */
	ccimx5x_set_mod_variant(system_serial_low & 0xff);
	ccimx5x_set_mod_revision((system_serial_low >> 8) & 0xff);
	ccimx5x_set_mod_sn(((system_serial_low << 8) & 0xff000000) |
			   ((system_serial_low >> 8) & 0x00ff0000) |
			   ((system_serial_high << 8) & 0x0000ff00) |
			   ((system_serial_high >> 8) & 0xff));

	ccimx51_swap_bi = system_serial_high >> 16;

	mxc_ipu_data.di_clk[0] = clk_get(NULL, "ipu_di0_clk");
	mxc_ipu_data.di_clk[1] = clk_get(NULL, "ipu_di1_clk");
	mxc_ipu_data.csi_clk[0] = clk_get(NULL, "csi_mclk1");
	mxc_ipu_data.csi_clk[1] = clk_get(NULL, "csi_mclk2");

	mxc_spdif_data.spdif_core_clk = clk_get(NULL, "spdif_xtal_clk");
	clk_put(mxc_spdif_data.spdif_core_clk);

	mxc_cpu_common_init();
	ccimx51_io_init();
	ccimx51_init_devices();

	mxc_register_device(&mxc_wdt_device, NULL);
#if defined(CONFIG_SPI_MXC_SELECT1) || defined(CONFIG_SPI_MXC_SELECT1_MODULE)
	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
#endif
#if defined(CONFIG_SPI_MXC_SELECT2) || defined(CONFIG_SPI_MXC_SELECT2_MODULE)
	mxc_register_device(&mxcspi2_device, &mxcspi2_data);
#endif
#if defined(CONFIG_SPI_MXC_SELECT3) || defined(CONFIG_SPI_MXC_SELECT3_MODULE)
	mxc_register_device(&mxcspi3_device, &mxcspi3_data);
#endif
#if defined(CONFIG_I2C_MX_SELECT1) || defined(CONFIG_I2C_MX_SELECT1_MODULE)
	mxc_register_device(&mxci2c_devices[0], &mxci2c1_data);
#endif
#if defined(CONFIG_I2C_MX_SELECT2) || defined(CONFIG_I2C_MX_SELECT2_MODULE)
	mxc_register_device(&mxci2c_devices[1], &mxci2c2_data);
#endif
#if defined(CONFIG_I2C_MX_SELECT3) || defined(CONFIG_I2C_MX_SELECT3_MODULE)
	mxc_register_device(&mxci2c_devices[2], &mxci2c3_data);
#endif
#if defined(CONFIG_I2C_MXC_HS) || defined(CONFIG_I2C_MXC_HS_MODULE)
	mxc_register_device(&mxci2c_hs_device, &mxci2c_hs_data);
#endif
	mxc_register_device(&mxc_rtc_device, NULL);
//	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
	mxc_register_device(&mxc_dma_device, NULL);
#if defined(CONFIG_W1_MASTER_MXC) || defined(CONFIG_W1_MASTER_MXC_MODULE)
	mxc_register_device(&mxc_w1_master_device, &mxc_w1_data);
#endif
	mxc_register_device(&mxc_ipu_device, &mxc_ipu_data);

	/* i.MX512 CPU does not have VPU module.
	 * Variants with this CPU should not load this driver.
	 */
	cpu_model = ccimx5x_get_cpumodel_from_variant();
	if (IMX512 == cpu_model) {
		printk(KERN_WARNING "CPU model i.MX51%d does not have VPU module\n", cpu_model);
	}
	else {
		mxc_register_device(&mxcvpu_device, &mxc_vpu_data);
	}
	mxc_register_device(&mxc_alsa_spdif_device, &mxc_spdif_data);
	mxc_register_device(&mxc_tve_device, &tve_data);
	mxc_register_device(&mx51_lpmode_device, NULL);
	mxc_register_device(&busfreq_device, &bus_freq_data);
	mxc_register_device(&sdram_autogating_device, NULL);
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	mxc_register_device(&mxc_dvfs_per_device, &dvfs_per_data);
#if defined (CONFIG_MXC_IIM) || defined (CONFIG_MXC_IIM_MODULE)
	mxc_register_device(&mxc_iim_device, &iim_data);
#endif
	mxc_register_device(&mxc_v4l2_device, NULL);
	mxc_register_device(&mxc_v4l2out_device, NULL);
	/* i.MX512 and i.MX13 CPUs do not have GPU module.
	 * Variants with this CPU should not load this driver.
	 */
	if (IMX512 == cpu_model || IMX513 == cpu_model) {
		printk(KERN_WARNING "CPU model i.MX51%d does not have GPU module\n", cpu_model);
	}
	else {
		mxc_register_device(&gpu_device,&gpu_data);
	}
#if defined (CONFIG_MXC_SECURITY_SCC2) || defined(CONFIG_MXC_SECURITY_SCC2_MODULE)
	mxc_register_device(&mxcscc_device, NULL);
#endif
	mxc_register_device(&mxc_pwm1_device, NULL);

#ifdef CONFIG_ESDHCI_MXC_SELECT1
	ccimx51_register_sdio(0);	/* SDHC1 */
#endif /* CONFIG_ESDHCI_MXC_SELECT1 */
#ifdef CONFIG_ESDHCI_MXC_SELECT2
	ccimx51_register_sdio(1);	/* SDHC1 */
#endif /* CONFIG_ESDHCI_MXC_SELECT1 */
#if defined(CONFIG_ESDHCI_MXC_SELECT3) && \
    (!defined(CONFIG_PATA_FSL) && !defined(CONFIG_PATA_FSL_MODULE))
	ccimx51_register_sdio(2);	/* SDHC3 */
#endif /* CONFIG_ESDHCI_MXC_SELECT3 && !CONFIG_PATA_FSL && !CONFIG_PATA_FSL_MODULE */

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
	mxc_register_device(&mxc_fec_device, &fec_data);
#endif
#if defined(CONFIG_MTD_NAND_MXC) \
	|| defined(CONFIG_MTD_NAND_MXC_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V2) \
	|| defined(CONFIG_MTD_NAND_MXC_V2_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V3) \
	|| defined(CONFIG_MTD_NAND_MXC_V3_MODULE)
	mxc_register_device(&mxc_nandv2_mtd_device, &mxc_nand_data);
#endif
#if defined(CONFIG_PATA_FSL) || defined(CONFIG_PATA_FSL_MODULE)
	mxc_register_device(&pata_fsl_device, &ata_data);
#endif
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
	mxc_register_device(&smsc911x_device, &ccimx51_smsc9118);
#endif
#if defined(CONFIG_SND_SOC_IMX_CCIMX51_WM8753) || defined(CONFIG_SND_SOC_IMX_CCIMX51_WM8753_MODULE)
	wm8753_data.ext_ram_clk = clk_get(NULL, "emi_fast_clk");
	clk_put(wm8753_data.ext_ram_clk);
	mxc_register_device(&mxc_wm8753_device, &wm8753_data);
#endif
	ccimx51_init_spidevices();
	ccimx51_init_i2c2();
	mx5_usb_dr_init();
#if defined(CONFIG_USB_EHCI_ARC_H1) || defined(CONFIG_USB_EHCI_ARC_H1_MODULE)
	mx5_usbh1_init();
#endif
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE) && \
   (defined(CONFIG_CCIMX5X_DISP0) || defined(CONFIG_CCIMX5X_DISP1))
	ccimx51_init_fb();
#endif /* defined(CONFIG_FB_MXC_SYNC_PANEL) || ... */

#ifdef CONFIG_MXC_PMIC_MC13892
	ccimx51_init_mc13892();
	/* Configure PMIC irq line */
	set_irq_type(IOMUX_TO_GPIO(MX51_PIN_GPIO1_5), IRQ_TYPE_EDGE_BOTH);
#endif

#ifdef CONFIG_CCIMX5X_SECOND_TOUCH
	ccimx51_init_2nd_touch();
#endif
	ccimx5x_create_sysfs_entries();

	pm_power_off = mxc_power_off;
}

static void __init ccimx51_timer_init(void)
{
	struct clk *uart_clk;

	/* Change the CPU voltages for TO2*/
	if (mx51_revision() == IMX_CHIP_REVISION_2_0) {
		cpu_wp_auto_800[0].cpu_voltage = 1175000;
		cpu_wp_auto_800[1].cpu_voltage = 1100000;
		cpu_wp_auto_800[2].cpu_voltage = 1000000;
	}

	mx51_clocks_init(32768, 24000000, 22579200, 24576000);

	uart_clk = clk_get_sys("mxcintuart.0", NULL);

	if (CONSOLE_UART_BASE_ADDR)
		early_console_setup(CONSOLE_UART_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= ccimx51_timer_init,
};

MACHINE_START(CCWMX51JS, "ConnectCore Wi-i.MX51"BOARD_NAME)
	/* Maintainer: Digi International, Inc. */
	.phys_io	= AIPS1_BASE_ADDR,
	.io_pg_offst	= ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup		= fixup_mxc_board,
	.map_io		= mx5_map_io,
	.init_irq	= mx5_init_irq,
	.init_machine	= mxc_board_init,
	.timer		= &mxc_timer,
MACHINE_END

MACHINE_START(CCMX51JS, "ConnectCore i.MX51"BOARD_NAME)
	/* Maintainer: Digi International, Inc. */
	.phys_io	= AIPS1_BASE_ADDR,
	.io_pg_offst	= ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup		= fixup_mxc_board,
	.map_io		= mx5_map_io,
	.init_irq	= mx5_init_irq,
	.init_machine	= mxc_board_init,
	.timer		= &mxc_timer,
MACHINE_END

MACHINE_START(CCIMX51JS, "ConnectCore for MX51"BOARD_NAME)
	/* Maintainer: Digi International, Inc. */
	.phys_io	= AIPS1_BASE_ADDR,
	.io_pg_offst	= ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup		= fixup_mxc_board,
	.map_io		= mx5_map_io,
	.init_irq	= mx5_init_irq,
	.init_machine	= mxc_board_init,
	.timer		= &mxc_timer,
MACHINE_END

/* Space holder, used for the CCIMX53 */
void gpio_camera_active(void) {}
EXPORT_SYMBOL(gpio_camera_active);

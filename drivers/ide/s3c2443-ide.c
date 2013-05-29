/* linux/drivers/ide/s3c2443-ide.c
 *
 * Copyright (c) 2009 Digi Internationa Inc.
 * http://www.digi.com
 * 
 * Partially based on previous driver by Seung-Chull, Suh <sc.suh@samsung.com>
 * Copyright(C) Samsung Electronics 2006
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/ide.h>
#include <linux/blkdev.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <mach/map.h>
#include <mach/gpio.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>
#include <mach/regs-bus.h>
#include <mach/regs-cfata.h>
#include <mach/gpio.h>

#define DRV_NAME "s3c2443-ide"

MODULE_AUTHOR("Pedro Perez de Heredia <pedro.perez@digi.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("S3C2443 IDE driver");

struct  _s3c2443_ide_hwif {
	ide_hwif_t		*hwif;
	int			irq;
	void __iomem		*membase;
	struct platform_device	*dev;
	struct clk		*clk;
};

static struct _s3c2443_ide_hwif s3c2443_ide_hwif;

static inline void s3c2443_wait_until_ready(void)
{
	u32 i, reg;

	for (i = 0; i < 100000; i++) {
		reg = readl(s3c2443_ide_hwif.membase + S3C2443_ATA_FIFO_STATUS);
		if ((reg >> 28) == 0)
			return;
	}
}

static __inline__ u8 s3c2443_ide_readb(unsigned long port)
{
	s3c2443_wait_until_ready();
	(void)readb(port);
	s3c2443_wait_until_ready();
	return readb(s3c2443_ide_hwif.membase + S3C2443_ATA_PIO_RDATA);
}

static __inline__ u16 s3c2443_ide_readw(unsigned long port)
{
	s3c2443_wait_until_ready();
	(void)readw(port);
	s3c2443_wait_until_ready();
	return readw(s3c2443_ide_hwif.membase + S3C2443_ATA_PIO_RDATA);
}

static __inline__ void s3c2443_ide_writeb(u8 val, unsigned long port)
{
	s3c2443_wait_until_ready();
	writeb(val, port);
}

static __inline__ void s3c2443_ide_writew(u16 val, unsigned long port)
{
	s3c2443_wait_until_ready();
	writew(val, port);
}

static void s3c2443_ide_exec_command(ide_hwif_t *hwif, u8 cmd)
{
	s3c2443_ide_writeb(cmd, hwif->io_ports.command_addr);
}

static u8 s3c2443_read_status(ide_hwif_t *hwif)
{
	return s3c2443_ide_readb(hwif->io_ports.status_addr);
}

static u8 s3c2443_read_altstatus(ide_hwif_t *hwif)
{
	return s3c2443_ide_readb(hwif->io_ports.ctl_addr);
}

static void s3c2443_write_devctl(ide_hwif_t *hwif, u8 ctl)
{
	s3c2443_ide_writeb(ctl, hwif->io_ports.ctl_addr);
}

static void s3c2443_tf_load(ide_drive_t *drive, struct ide_taskfile *tf, u8 valid)
{
	ide_hwif_t *hwif = drive->hwif;
	struct ide_io_ports *io_ports = &hwif->io_ports;

	if (valid & IDE_VALID_FEATURE)
		s3c2443_ide_writeb(tf->feature, io_ports->feature_addr);
	if (valid & IDE_VALID_NSECT)
		s3c2443_ide_writeb(tf->nsect, io_ports->nsect_addr);
	if (valid & IDE_VALID_LBAL)
		s3c2443_ide_writeb(tf->lbal, io_ports->lbal_addr);
	if (valid & IDE_VALID_LBAM)
		s3c2443_ide_writeb(tf->lbam, io_ports->lbam_addr);
	if (valid & IDE_VALID_LBAH)
		s3c2443_ide_writeb(tf->lbah, io_ports->lbah_addr);
	if (valid & IDE_VALID_DEVICE)
		s3c2443_ide_writeb(tf->device, io_ports->device_addr);
}

static void s3c2443_tf_read(ide_drive_t *drive, struct ide_taskfile *tf, u8 valid)
{
	ide_hwif_t *hwif = drive->hwif;
	struct ide_io_ports *io_ports = &hwif->io_ports;

	if (valid & IDE_VALID_ERROR)
		tf->error  = s3c2443_ide_readb(io_ports->feature_addr);
	if (valid & IDE_VALID_NSECT)
		tf->nsect  = s3c2443_ide_readb(io_ports->nsect_addr);
	if (valid & IDE_VALID_LBAL)
		tf->lbal   = s3c2443_ide_readb(io_ports->lbal_addr);
	if (valid & IDE_VALID_LBAM)
		tf->lbam   = s3c2443_ide_readb(io_ports->lbam_addr);
	if (valid & IDE_VALID_LBAH)
		tf->lbah   = s3c2443_ide_readb(io_ports->lbah_addr);
	if (valid & IDE_VALID_DEVICE)
		tf->device = s3c2443_ide_readb(io_ports->device_addr);
}

static void s3c2443_input_data(ide_drive_t *drive, struct ide_cmd *cmd, void *buf,
		    unsigned int len)
{
	ide_hwif_t *hwif = drive->hwif;
	struct ide_io_ports *io_ports = &hwif->io_ports;
	u16 *buffer = (u16 *)buf;

	/* round up, it should be power of 2 */
	len = (len + 1) >> 1;

	while (len--) {
		*buffer++ = s3c2443_ide_readw(io_ports->data_addr);
	}
}

static void s3c2443_output_data(ide_drive_t *drive, struct ide_cmd *cmd, void *buf,
		     unsigned int len)
{
	ide_hwif_t *hwif = drive->hwif;
	struct ide_io_ports *io_ports = &hwif->io_ports;
	u16 *buffer = (u16 *)buf;

	len >>= 1;

	while (len--) {
		s3c2443_ide_writew(*buffer++, io_ports->data_addr);
	}
}

static int s3c2443_ide_test_irq(ide_hwif_t *hwif)
{
	u32 reg = 0, tout = 100;

	while (tout--) {
		reg = readl(s3c2443_ide_hwif.membase + S3C2443_ATA_IRQ);
		if (reg) {
			writel(reg, s3c2443_ide_hwif.membase + S3C2443_ATA_IRQ);
			break;
		}
	}

	return (int)reg;
}

static const struct ide_tp_ops s3c2443_tp_ops = {
	.exec_command		= s3c2443_ide_exec_command,
	.read_status		= s3c2443_read_status,
	.read_altstatus		= s3c2443_read_altstatus,
	.write_devctl		= s3c2443_write_devctl,
	.dev_select		= ide_dev_select,
	.tf_load		= s3c2443_tf_load,
	.tf_read		= s3c2443_tf_read,
	.input_data		= s3c2443_input_data,
	.output_data		= s3c2443_output_data,
};

static const struct ide_port_ops s3c2443_port_ops = {
	.test_irq		= s3c2443_ide_test_irq,
};

static const struct ide_port_info s3c2443_port_info = {
	.tp_ops 		= &s3c2443_tp_ops,
	.port_ops		= &s3c2443_port_ops,
	.host_flags		= IDE_HFLAG_MMIO | IDE_HFLAG_NO_DMA | \
				  IDE_HFLAG_NO_IO_32BIT,
	.chipset		= ide_generic,
};

static int s3c2443_ide_hw_config(struct _s3c2443_ide_hwif *hw)
{
	int ret = 0;
	u32 reg;
	void __iomem *ebicon;

	ebicon = ioremap(S3C2443_PA_EBI + 0x8, 0x4);
	if (ebicon == NULL) {
		ret =-EBUSY;
		goto err_unmap;
	}

	/* Configure EBI bank 2 and 3 for the CF interface */
	reg = readl(ebicon);
	reg |= S3C2443_EBICON_BANK3_CFG | S3C2443_EBICON_BANK2_CFG;
	writel(reg, ebicon);

	/* Configure the IO lines*/
	s3c2443_gpio_cfgpin(S3C2410_GPG(15), S3C2443_GPG15_CF_PWR);
	s3c2443_gpio_cfgpin(S3C2410_GPG(14), S3C2443_GPG14_CF_RESET);
	s3c2443_gpio_cfgpin(S3C2410_GPG(13), S3C2443_GPG13_CF_nREG);
	s3c2443_gpio_cfgpin(S3C2410_GPG(12), S3C2443_GPG12_nINPACK);
	s3c2443_gpio_cfgpin(S3C2410_GPG(11), S3C2443_GPG11_CF_nIREQ);

	s3c2443_gpio_cfgpin(S3C2410_GPA(10), 0);
	s3c2443_gpio_cfgpin(S3C2410_GPA(11), 1);
	s3c2443_gpio_cfgpin(S3C2410_GPA(12), 1);
	s3c2443_gpio_cfgpin(S3C2410_GPA(15), 1);

	/* Clear the card detect condition */
	reg = readl(S3C24XX_MISCCR) & ~S3C2443_MISCCR_nCD_CF;
	writel(reg, S3C24XX_MISCCR);

	/* Output Port disabled, Card power off, ATA mode */
	writel(0x07, hw->membase + S3C2443_MUX_REG);
	mdelay(10);
	/* Output Port enable, Card power off, ATA mode */
	writel(0x03, hw->membase + S3C2443_MUX_REG);
	mdelay(10);
	/* Output Port enable, Card power on, ATA mode */
	writel(0x01, hw->membase + S3C2443_MUX_REG);
	mdelay(500);	/* wait for 500ms */

	writel(0x1C238, hw->membase + S3C2443_ATA_PIO_TIME);
	writel(0x20B1362, hw->membase + S3C2443_ATA_UDMA_TIME);

	/* Enable ATA */
	reg = readl(hw->membase + S3C2443_ATA_CONTROL);
	writel(reg | 0x1, hw->membase + S3C2443_ATA_CONTROL);

	mdelay(200);
	/* remove IRQ Status and enable only ATA device interrupt */
	writel(0x1f, hw->membase + S3C2443_ATA_IRQ);
	writel(0x1b, hw->membase + S3C2443_ATA_IRQ_MASK);

	mdelay(200);

err_unmap:
	iounmap(ebicon);
	return ret;
}


static int __devinit s3c2443_ide_probe(struct platform_device *pdev)
{
	int ret = 0, i;
	struct _s3c2443_ide_hwif *shwif = &s3c2443_ide_hwif;
	struct ide_hw hw, *hws[] = { &hw };
	struct resource *res;
	struct ide_host *host;

	memset(&s3c2443_ide_hwif, 0, sizeof(struct _s3c2443_ide_hwif));

	shwif->dev = pdev;
	
	/* find and map our resources */
	shwif->irq = platform_get_irq(pdev, 0);
	if (shwif->irq < 0) {
		pr_debug("%s: %s, can not get IORESOURCE_IRQ\n", DRV_NAME, __func__);
		ret =-ENOENT;
		goto error;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		pr_debug("%s: %s, can not get IORESOURCE_MEM\n", DRV_NAME, __func__);
		ret = -ENOENT;
		goto error;
	}

	if (!request_mem_region(res->start, res->end - res->start + 1,
				pdev->name)) {
		pr_debug("%s: %s,request_mem_region failed\n", DRV_NAME, __func__);
		ret = -EBUSY;
		goto error;
	}

	shwif->membase = ioremap(res->start, res->end - res->start + 1);
	if (shwif->membase == NULL) {
		pr_debug("%s: %s, can not map IO space at 0x%08x\n", DRV_NAME, __func__, res->start);
		ret = -ENOMEM;
		goto error_map;
	}

	shwif->clk = clk_get(&pdev->dev, "cfc");
	if (IS_ERR(shwif->clk)) {
		pr_debug("%s: %s, failed to find clock source\n", DRV_NAME, __func__);
		ret = PTR_ERR(shwif->clk);
		goto error_clock;
	}

	if ((ret = clk_enable(shwif->clk))) {
		pr_debug("%s: %s, failed to enable clock\n", DRV_NAME, __func__);
		goto error_clk_en;
	}

	ret = s3c2443_ide_hw_config(shwif);
	if (ret) {
		pr_debug("%s: %s, failed to configure IDE hardware\n", DRV_NAME, __func__);
		goto error_hwcfg;
	}

	memset(&hw, 0, sizeof(hw));

	for (i = 0; i < 9; i++)
		hw.io_ports_array[i] = (unsigned long)(shwif->membase + S3C2443_ATA_PIO_DTR + (i * 4));
	hw.io_ports.ctl_addr = (unsigned long)(shwif->membase + S3C2443_ATA_PIO_DTR + 8 * 4);
	hw.irq = shwif->irq;
	hw.dev = &pdev->dev;

	ret = ide_host_add(&s3c2443_port_info, hws, 1, &host);
	if (ret) {
		pr_debug("%s: %s, failed add IDE host device\n", DRV_NAME, __func__);
		goto error_add;
	}

	shwif->hwif = host->ports[0];
	platform_set_drvdata(pdev, host);

	printk(KERN_INFO "S3C2443 IDE driver\n");
	return 0;

error_add:
	/* XXX unconfig hardware here ? */
error_hwcfg:
	clk_disable(shwif->clk);
error_clk_en:
	clk_put(shwif->clk);
error_clock:
	iounmap(shwif->membase);
error_map:
	release_mem_region(res->start, res->end - res->start + 1);
error:
	return ret;
}

static int __devexit s3c2443_ide_remove(struct platform_device *pdev)
{
	struct ide_host *host = dev_get_drvdata(&pdev->dev);
	struct _s3c2443_ide_hwif *shwif = &s3c2443_ide_hwif;
	struct resource *res;

	ide_host_remove(host);

	clk_disable(shwif->clk);
	clk_put(shwif->clk);

	iounmap(shwif->membase);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res != NULL) 
		release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#ifdef CONFIG_PM
static int s3c2443_ide_resume(struct platform_device *dev)
{
	struct _s3c2443_ide_hwif *shwif = &s3c2443_ide_hwif;

	return s3c2443_ide_hw_config(shwif);
}
#else
# define s3c2443_ide_suspend NULL
# define s3c2443_ide_resume  NULL
#endif


static struct platform_driver s3c2443_ide_driver = {
	.probe		= s3c2443_ide_probe,
	.remove		= __devexit_p(s3c2443_ide_remove),
	.resume		= s3c2443_ide_resume,
	.driver = {
		.name	= "s3c2443-ide",
		.owner	= THIS_MODULE,
	},
};

static int __init s3c2443_ide_init(void)
{
	return platform_driver_register(&s3c2443_ide_driver);
}

static void __exit s3c2443_ide_exit(void)
{
	platform_driver_unregister(&s3c2443_ide_driver);
}

module_init(s3c2443_ide_init);
module_exit(s3c2443_ide_exit);


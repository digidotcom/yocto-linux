/* linux/drivers/pcmcia/s3c2443_pcmcia.c
 *
 * Copyright (c) 2009 Digi Internationa Inc.
 * http://www.digi.com
 * 
 * Based on similar driver for other ARM SoC, like the at91_cf.c, from
 * David Brownell and other authors.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>

#include <pcmcia/cs_types.h>
#include <pcmcia/ss.h>
#include <pcmcia/cistpl.h>

#include <plat/pcmcia.h>
#include <mach/regs-cfata.h>
#include <mach/regs-bus.h>
#include <mach/gpio.h>

#define DRV_NAME	"s3c2443-pcmcia"

struct s3c2443_pcmcia_socket {
	struct pcmcia_socket		socket;
	struct platform_device		*pdev;
	struct s3c2443_pcmcia_pdata	*pdata;
	unsigned			present:1;
	int				irq;
	int				irq_cd;
	unsigned long			phys_io;
	unsigned long			phys_attr;
	unsigned long			phys_comm;
	void __iomem			*pcmcia_base;
};

static inline int s3c2443_pcmcia_present(struct s3c2443_pcmcia_socket *psock)
{
	return s3c2410_gpio_getpin(psock->pdata->gpio_detect) ? 0 : 1;
}

static int s3c2443_pcmcia_socket_init(struct pcmcia_socket *s)
{
	return 0;
}

static irqreturn_t s3c2443_pcmcia_irq_cd(int irq, void *data)
{
	struct s3c2443_pcmcia_socket *psock = data;
	unsigned present;

	present = s3c2443_pcmcia_present(psock);
	if (present != psock->present) {
		psock->present = present;
		pr_debug("%s: card %s\n", DRV_NAME,
			 present ? "present" : "gone");

		pcmcia_parse_events(&psock->socket, SS_DETECT);
	}

	return IRQ_HANDLED;
}

static irqreturn_t s3c2443_pcmcia_irq(int irq, void *data)
{
	struct s3c2443_pcmcia_socket *psock = data;
	irqreturn_t ret = IRQ_NONE;
	u32 irqreg;

	/* read interrupt sources and acknowledge irqs */
	irqreg = readl(psock->pcmcia_base + S3C2443_PCCARD_INT);
	writel(irqreg, psock->pcmcia_base + S3C2443_PCCARD_INT);

	/* Mask before checking the interrupt source */
	irqreg = irqreg & ~(irqreg >> 8) & 0x07;

	if (irqreg) {
		ret = IRQ_HANDLED;

		if (irqreg & S3C2443_PCC_INTSRC_IREQ)
			pcmcia_parse_events(&psock->socket, SS_STSCHG);
		if (irqreg & S3C2443_PCC_INTSRC_ERR_N)
			printk(KERN_WARNING "%s: %s, error interrupt\n",
			       DRV_NAME, __func__);
		if (irqreg & S3C2443_PCC_INTSRC_CD)
			printk(KERN_WARNING "%s: %s, card detect irq ???\n",
			       DRV_NAME, __func__);
	}

	return ret;
}

static int s3c2443_pcmcia_get_status(struct pcmcia_socket *s, u_int *sp)
{
	struct s3c2443_pcmcia_socket *psock;
	u32 muxreg;

	if (!sp)
		return -EINVAL;

	psock = container_of(s, struct s3c2443_pcmcia_socket, socket);

	/* NOTE: CF is always 3VCARD */
	if (s3c2443_pcmcia_present(psock)) {
		*sp = SS_READY | SS_DETECT | SS_3VCARD;

		muxreg = readl(psock->pcmcia_base + S3C2443_MUX_REG);
		if (!(muxreg & S3C2443_MUX_PWREN_PWOFF))
			*sp |= SS_POWERON;

		s->pcmcia_irq = 0;
		s->pci_irq = psock->irq;
	} else
		*sp = 0;

	return 0;
}

static int
s3c2443_pcmcia_set_socket(struct pcmcia_socket *sock, struct socket_state_t *s)
{
	struct s3c2443_pcmcia_socket *psock = 
		      container_of(sock, struct s3c2443_pcmcia_socket, socket);
	u32 muxreg;
	u32 cfg;

	muxreg = readl(psock->pcmcia_base + S3C2443_MUX_REG);
	muxreg &= ~S3C2443_MUX_PWREN_PWOFF;
	cfg = readl(psock->pcmcia_base + S3C2443_PCCARD_CFG);
	cfg &= ~S3C2443_PCC_CARD_RESET;

	switch (s->Vcc) {
	case 0:
		writel(muxreg | S3C2443_MUX_PWREN_PWOFF,
		       psock->pcmcia_base + S3C2443_MUX_REG);
		break;
	case 33:
		writel(muxreg | S3C2443_MUX_PWREN_PWON,
		       psock->pcmcia_base + S3C2443_MUX_REG);
		break;
	default:
		return -EINVAL;
	}

	if (s->flags & SS_RESET)
		cfg |= S3C2443_PCC_CARD_RESET;

	writel(cfg, psock->pcmcia_base + S3C2443_PCCARD_CFG);
	pr_debug("%s: Vcc %d, io_irq %d, flags %04x csc %04x\n",
		DRV_NAME, s->Vcc, s->io_irq, s->flags, s->csc_mask);

	return 0;
}

static int s3c2443_pcmcia_suspend(struct pcmcia_socket *s)
{
	return s3c2443_pcmcia_set_socket(s, &dead_socket);
}

/* we already mapped the I/O region */
static int s3c2443_pcmcia_set_io_map(struct pcmcia_socket *s, struct pccard_io_map *io)
{
	struct s3c2443_pcmcia_socket *psock;
	u32 cfg;

	psock = container_of(s, struct s3c2443_pcmcia_socket, socket);
	io->flags &= MAP_ACTIVE | MAP_16BIT | MAP_AUTOSZ;

	/* Use 16 bit accesses */
	cfg = readl(psock->pcmcia_base + S3C2443_PCCARD_CFG);
	cfg |= S3C2443_PCC_DEVICE_IO_16;
	writel(cfg, psock->pcmcia_base + S3C2443_PCCARD_CFG);

	io->start = psock->socket.io_offset;
	io->stop = io->start + SZ_2K - 1;

	return 0;
}

/* pcmcia layer maps/unmaps mem regions */
static int
s3c2443_pcmcia_set_mem_map(struct pcmcia_socket *s, struct pccard_mem_map *map)
{
	struct s3c2443_pcmcia_socket *psock;

	if (map->card_start)
		return -EINVAL;

	psock = container_of(s, struct s3c2443_pcmcia_socket, socket);

	map->flags &= (MAP_ACTIVE | MAP_ATTRIB | MAP_16BIT);
	map->static_start = (map->flags & MAP_ATTRIB) ? 
			    psock->phys_attr : psock->phys_comm;

	return 0;
}

static struct pccard_operations s3c2443_pcmcia_ops = {
	.init			= s3c2443_pcmcia_socket_init,
	.suspend		= s3c2443_pcmcia_suspend,
	.get_status		= s3c2443_pcmcia_get_status,
	.set_socket		= s3c2443_pcmcia_set_socket,
	.set_io_map		= s3c2443_pcmcia_set_io_map,
	.set_mem_map		= s3c2443_pcmcia_set_mem_map,
};


static int s3c2443_pcmcia_hw_config(struct s3c2443_pcmcia_socket *psock)
{
	void __iomem *ebicon;
	int ret = 0;
	u32 reg;

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

	/* TODO use the proper api after adding s3c2443_gpio_pullup */  
	/* Enable the pullup for interrupt line  */
	reg = readl(S3C2410_GPGUP);
	writel(reg & ~(3 << 22), S3C2410_GPGUP);

	/* Clear the card detect condition */
	reg = readl(S3C24XX_MISCCR) & ~S3C2443_MISCCR_nCD_CF;
	writel(reg, S3C24XX_MISCCR);
 
	/* Output Port enabled, Card power off, PCCARD mode */
	writel(S3C2443_MUX_OUTPUT_ENABLE | S3C2443_MUX_PWREN_PWOFF | 
	       S3C2443_MUX_MODE_PCCARD, psock->pcmcia_base + S3C2443_MUX_REG);

	/* Acknowledge any pending interrupt and mask all irqs to get started */
	writel(S3C2443_PCC_INTSRC_ALL, psock->pcmcia_base + S3C2443_PCCARD_INT);
	writel(S3C2443_PCC_INTMSK_ALL, psock->pcmcia_base + S3C2443_PCCARD_INT);

 	writel(S3C2443_PCC_INTMSK_CD | S3C2443_PCC_INTSRC_ALL,
 	       psock->pcmcia_base + S3C2443_PCCARD_INT);

	/* Configure timmings for IO, ATTR and COMM areas */
	writel(0x061D04, psock->pcmcia_base + S3C2443_PCCARD_ATTR);
	writel(0x031109, psock->pcmcia_base + S3C2443_PCCARD_IO);
	writel(0x061D04, psock->pcmcia_base + S3C2443_PCCARD_COMM);

err_unmap:
	iounmap(ebicon);
	return ret;
}

static int __init s3c2443_pcmcia_probe(struct platform_device *pdev)
{
	struct s3c2443_pcmcia_socket *psock;
	struct s3c2443_pcmcia_pdata *pdata;
	struct resource *res;
	int ret;

	pdata = pdev->dev.platform_data;

	if (!pdata || !pdata->gpio_detect) {
		pr_debug("%s: %s, platform data not available\n", DRV_NAME, __func__);
		ret = -ENODEV;
		goto error;
	}

	psock = kzalloc(sizeof(struct s3c2443_pcmcia_socket), GFP_KERNEL);
	if (psock == NULL) {
		pr_debug("%s: %s, can not allocate memory\n", DRV_NAME, __func__);
		ret = -ENOMEM;
		goto error;
	}

	psock->irq = platform_get_irq(pdev, 0);
	if (psock->irq < 0) {
		pr_debug("%s: %s, can not get IORESOURCE_IRQ\n", DRV_NAME, __func__);
		ret =-ENOENT;
		goto error_get_irq;
	}
	psock->socket.pci_irq = psock->irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		pr_debug("%s: %s, can not get IORESOURCE_MEM\n", DRV_NAME, __func__);
		ret = -ENOENT;
		goto error_get_irq;
	}

	psock->pdata = pdata;
	psock->pdev = pdev;
	platform_set_drvdata(pdev, psock);
	psock->phys_io = res->start + S3C2443_IO_BASE;
	psock->phys_attr = res->start + S3C2443_ATTR_BASE;

	if (!request_mem_region(res->start, res->end - res->start + 1,
				pdev->name)) {
		pr_debug("%s: %s,request_mem_region failed\n", DRV_NAME, __func__);
		ret = -EBUSY;
		goto error_get_irq;
	}

	psock->socket.io_offset = (unsigned long)ioremap(psock->phys_io, SZ_2K);
	if (!psock->socket.io_offset) {
		pr_debug("%s: %s, can not map IO space at 0x%08x\n", 
			 DRV_NAME, __func__, (unsigned int)psock->phys_io);
		ret = -ENXIO;
		goto error_map;
	}

	psock->pcmcia_base = ioremap(res->start, res->end - res->start + 1);
	if (!psock->pcmcia_base) {
		pr_debug("%s: %s, can not map IO space at 0x%08x\n", 
			 DRV_NAME, __func__, (unsigned int)(res->start + S3C2443_MUX_REG));
		ret = -ENXIO;
		goto error_map2;
	}

	ret = s3c2443_pcmcia_hw_config(psock);
	if (ret) {
		pr_debug("%s: %s, failed to configure PCMCIA hardware\n", DRV_NAME, __func__);
		goto error_hwcfg;
	}

	psock->irq_cd = gpio_to_irq(pdata->gpio_detect);
	if (psock->irq_cd < 0) {
		pr_debug("%s: %s, not irq available for card detection\n", 
			 DRV_NAME, __func__);
		ret = -ENOENT;
		goto error_irq_cd;
	}

	ret = request_irq(psock->irq_cd, s3c2443_pcmcia_irq_cd, 
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, DRV_NAME, psock);
	if (ret < 0) {
		pr_debug("%s: %s, can not request irq %d (ret %d)\n", 
			DRV_NAME, __func__, psock->irq_cd, ret);
		ret = -EBUSY;
		goto error_irq_cd;
	}

	ret = request_irq(psock->irq, s3c2443_pcmcia_irq,
				IRQF_SHARED, DRV_NAME, psock);
	if (ret < 0) {
		pr_debug("%s: %s, can not request irq %d (ret %d)\n", 
			DRV_NAME, __func__, psock->irq, ret);
		ret = -EBUSY;
		goto error_req_irq;
	}

	psock->socket.owner = THIS_MODULE;
	psock->socket.dev.parent = &pdev->dev;
	psock->socket.ops = &s3c2443_pcmcia_ops;
	psock->socket.resource_ops = &pccard_static_ops;
	psock->socket.features = SS_CAP_PCCARD | SS_CAP_STATIC_MAP
				| SS_CAP_MEM_ALIGN;
	psock->socket.map_size = SZ_2K;
	psock->socket.io[0].res = res;


	ret = pcmcia_register_socket(&psock->socket);
	if (ret < 0) {
		pr_debug("%s: %s, can not register pcmcia socket\n", 
			DRV_NAME, __func__);
		goto error_reqister;
	}

	pr_info("%s: PCMCIA driver\n", DRV_NAME);
	return 0;

error_reqister:
	free_irq(psock->irq, psock);
error_req_irq:
	free_irq(psock->irq_cd, psock);
error_irq_cd:
error_hwcfg:
	iounmap(psock->pcmcia_base);
error_map2:
	iounmap((void __iomem *)psock->socket.io_offset);
error_map:
	release_mem_region(res->start, res->end - res->start + 1);
error_get_irq:
	kfree(psock);
error:
	return ret;
}

static int __exit s3c2443_pcmcia_remove(struct platform_device *pdev)
{
	struct s3c2443_pcmcia_socket *psock = platform_get_drvdata(pdev);
	struct s3c2443_pcmcia_pdata *pdata = psock->pdata;
	struct resource *res = psock->socket.io[0].res;

	pcmcia_unregister_socket(&psock->socket);
	iounmap((void __iomem *)psock->socket.io_offset);
	iounmap(psock->pcmcia_base);
	release_mem_region(res->start, res->end - res->start + 1);
	free_irq(psock->irq, psock);
	device_init_wakeup(&pdev->dev, 0);
	
	if (pdata->gpio_detect) {
		free_irq(psock->irq_cd, psock);
		gpio_free(pdata->gpio_detect);
	}
	kfree(psock);

	return 0;
}

static struct platform_driver s3c2443_pcmcia_driver = {
	.probe		= s3c2443_pcmcia_probe,
	.remove		= __exit_p(s3c2443_pcmcia_remove),
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init s3c2443_pcmcia_init(void)
{
	return platform_driver_register(&s3c2443_pcmcia_driver);
}

static void __exit s3c2443_pcmcia_exit(void)
{
	platform_driver_unregister(&s3c2443_pcmcia_driver);
}

module_init(s3c2443_pcmcia_init);
module_exit(s3c2443_pcmcia_exit);

MODULE_AUTHOR("Pedro Perez de Heredia");
MODULE_DESCRIPTION("S3C2443 PCMCIA core socket driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:s3c2443-pcmcia");


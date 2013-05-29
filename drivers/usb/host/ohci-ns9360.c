/*
 * drivers/usb/host/ohci-ns9360.c
 *
 * based on drivers/usb/host/ohci-sa1111.c which has
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2002 David Brownell <dbrownell@users.sourceforge.net>
 * (C) Copyright 2002 Hewlett-Packard Company
 *
 * Copyright (C) 2008 by Digi International Inc.
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/platform_device.h>

#define NS9360_UHFE_IEN		0x0c
#define NS9360_UHFE_IEN_OHCIIRQ		(1 << 1)
#define NS9360_UHFE_ISTAT	0x10
#define NS9360_UHFE_ISTAT_OHCIIRQ	(1 << 1)

#define DRIVER_NAME "ns9360-ohci"

struct ns9360_ohci_device {
	struct ohci_hcd ohci;

	struct resource *uhfe;
	void __iomem *ioaddr;
	struct clk *clk;
};

#define hcd2nod(up) ((struct ns9360_ohci_device *)(up->hcd_priv))

extern int usb_disabled(void);

static irqreturn_t usb_hcd_ns9360_hcim_irq(struct usb_hcd *hcd)
{
	u32 stat;
	struct ns9360_ohci_device *priv = hcd2nod(hcd);

	/* let OHCI driver handles this */
	irqreturn_t ret = ohci_irq(hcd);

	/* acknowledge interrupt in UHFE */
	stat = ioread32(priv->ioaddr + NS9360_UHFE_ISTAT);
	iowrite32(stat, priv->ioaddr + NS9360_UHFE_ISTAT);

	return ret;
}

static int __devinit ohci_ns9360_start(struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);
	int ret;

	ret = ohci_init(ohci);
	if (ret < 0)
		return ret;

	ret = ohci_run(ohci);
	if (ret < 0) {
		ohci_err(ohci, "can't start %s", hcd->self.bus_name);
		ohci_stop(hcd);
		return ret;
	}

	return 0;
}

int usb_hcd_ns9360_probe(const struct hc_driver *driver,
		struct platform_device *pdev)
{
	int ret, irq;
	struct usb_hcd *hcd;
	struct resource *mem;
	struct ns9360_ohci_device *priv;

	hcd = usb_create_hcd(driver, &pdev->dev, "ns9360");
	if (!hcd) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "%s: err_create_hcd\n", __func__);
		goto err_create_hcd;
	}
	priv = hcd2nod(hcd);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = -ENODEV;
		dev_dbg(&pdev->dev, "%s: err_get_irq\n", __func__);
		goto err_get_irq;
	}

	priv->uhfe = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!priv->uhfe) {
		ret = -ENODEV;
		dev_dbg(&pdev->dev, "%s: err_get_uhfe\n", __func__);
		goto err_get_uhfe;
	}

	if (!request_mem_region(priv->uhfe->start,
				priv->uhfe->end - priv->uhfe->start + 1,
				DRIVER_NAME)) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "%s: err_request_uhfe\n", __func__);
		goto err_request_uhfe;
	}

	priv->ioaddr = ioremap(priv->uhfe->start,
			priv->uhfe->end - priv->uhfe->start + 1);
	if (!priv->ioaddr) {
		ret = -ENODEV;
		dev_dbg(&pdev->dev, "%s: err_map_uhfe\n", __func__);
		goto err_map_uhfe;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		ret = -ENODEV;
		dev_dbg(&pdev->dev, "%s: err_get_mem\n", __func__);
		goto err_get_mem;
	}

	hcd->rsrc_start = mem->start;
	hcd->rsrc_len = mem->end - mem->start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name))  {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "%s: err_request_mem\n", __func__);
		goto err_request_mem;
}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		ret = -EIO;
		dev_dbg(&pdev->dev, "%s: err_map_mem\n", __func__);
		goto err_map_mem;
	}

	priv->clk = clk_get(&pdev->dev, DRIVER_NAME);
	if (IS_ERR(priv->clk)) {
		ret = PTR_ERR(priv->clk);
		dev_dbg(&pdev->dev, "%s: err_clk_get\n", __func__);
		goto err_clk_get;
	}

	ret = clk_enable(priv->clk);
	if (ret) {
		dev_dbg(&pdev->dev, "%s: err_clk_enable\n", __func__);
		goto err_clk_enable;
	}

	/* enable irq */
	iowrite32(NS9360_UHFE_IEN_OHCIIRQ, priv->ioaddr + NS9360_UHFE_IEN);

	ohci_hcd_init(hcd_to_ohci(hcd));

	ret = usb_add_hcd(hcd, irq, IRQF_DISABLED);
	if (ret != 0) {
		dev_dbg(&pdev->dev, "%s: err_add_hcd\n", __func__);
		goto err_add_hcd;
	}

	return 0;

err_add_hcd:
	clk_disable(priv->clk);
err_clk_enable:
	clk_put(priv->clk);
err_clk_get:
	iounmap(hcd->regs);
err_map_mem:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err_request_mem:
err_get_mem:
	iounmap(priv->ioaddr);
err_map_uhfe:
	release_mem_region(priv->uhfe->start,
			priv->uhfe->end - priv->uhfe->start + 1);
err_request_uhfe:
err_get_uhfe:
err_get_irq:
	usb_put_hcd(hcd);
err_create_hcd:
	return ret;
}

static int usb_hcd_ns9360_remove(struct usb_hcd *hcd,
		struct platform_device *pdev)
{
	struct ns9360_ohci_device *priv = hcd2nod(hcd);

	usb_remove_hcd(hcd);

	clk_disable(priv->clk);
	clk_put(priv->clk);

	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);

	iounmap(priv->ioaddr);
	release_mem_region(priv->uhfe->start,
			priv->uhfe->end - priv->uhfe->start + 1);

	usb_put_hcd(hcd);

	return 0;
}

static const struct hc_driver ohci_ns9360_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"NS9360 OHCI",
	.hcd_priv_size =	sizeof(struct ns9360_ohci_device),

	/* generic hardware linkage */
	.irq =			usb_hcd_ns9360_hcim_irq,
	.flags =		HCD_USB11 | HCD_MEMORY ,

	/* basic lifecycle operations */
	.start =		ohci_ns9360_start,
	.stop =			ohci_stop,
	.shutdown =		ohci_shutdown,

	/* managing i/o requests and associated device resources */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/* scheduling support */
	.get_frame_number =	ohci_get_frame,

	/* root hub support */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
#ifdef CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

static int ohci_hcd_ns9360_drv_probe(struct platform_device *dev)
{
	return usb_hcd_ns9360_probe(&ohci_ns9360_hc_driver, dev);
}

static int ohci_hcd_ns9360_drv_remove(struct platform_device *dev)
{
	return usb_hcd_ns9360_remove(platform_get_drvdata(dev), dev);
}

static struct platform_driver ohci_hcd_ns9360_driver = {
	.probe		= ohci_hcd_ns9360_drv_probe,
	.remove		= ohci_hcd_ns9360_drv_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

MODULE_ALIAS("platform:" DRIVER_NAME);

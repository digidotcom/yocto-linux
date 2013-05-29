/*
 * drivers/mtd/nand/ccx9x_nand.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * Based on drivers/mtd/nand/ccx9x_nand.c by Markus Pietrek
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mtd/ccx9x_nand.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <mach/gpio.h>

#define DRIVER_NAME "ccx9x_nand"
#define DEVICE_NAME "onboard_boot"

#define mi2ncd(mi) container_of(mi, struct nand_ccx9x, mtd)

#ifdef CONFIG_MTD_PARTITIONS
static const char *probes[] = { "cmdlinepart", NULL };
#endif

struct nand_ccx9x {
	struct mtd_info mtd;
	struct nand_chip chip;

	struct resource *mem;
	struct ccx9x_nand_info *nand_info;
};

/* read in gpio ready pin */
static int ccx9x_nand_dev_ready(struct mtd_info *mtd)
{
	struct nand_ccx9x *ncd = mi2ncd(mtd);

	return gpio_get_value(ncd->nand_info->busy_pin);
}

static void ccx9x_nand_cmd_ctrl(struct mtd_info *mtd, int cmd,
		unsigned int ctrl)
{
	struct nand_ccx9x *ncd = mi2ncd(mtd);

	if (cmd == NAND_CMD_NONE)
		return;

	if (ctrl & NAND_CLE)
		writeb(cmd, ncd->chip.IO_ADDR_W + ncd->nand_info->cmd_offset);
	else
		writeb(cmd, ncd->chip.IO_ADDR_W + ncd->nand_info->addr_offset);
}

static void ccx9x_nand_read_buf(struct mtd_info *mtd,
		u_char *buf, int len)
{
	struct nand_ccx9x *ncd = mi2ncd(mtd);

	while (len) {
		*buf = ioread8(ncd->chip.IO_ADDR_R);
		buf++;
		len--;
	}
}

static void ccx9x_nand_write_buf(struct mtd_info *mtd,
		const u_char *buf, int len)
{
	struct nand_ccx9x *ncd = mi2ncd(mtd);

	while (len) {
		iowrite8(*buf, ncd->chip.IO_ADDR_W);
		buf++;
		len--;
	}

	/* FIXME: 32bit burst writing */
}

static int ccx9x_nand_verify_buf(struct mtd_info *mtd,
		const u_char *buf, int len)
{
	static u_char *tmp;
	int i, ret = 0;

	tmp = kmalloc(len, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	/* read fromd device */
	ccx9x_nand_read_buf(mtd, tmp, len);

	/* compare */
	for (i = 0; i < len; i++)
		if (tmp[i] != buf[i]) {
			ret = -EFAULT;
			break;
		}

	kfree(tmp);
	return ret;

}

static int ccx9x_nand_probe(struct platform_device *pdev)
{
	struct nand_ccx9x *ncd;
	struct resource *mem;
	struct mtd_partition *mtd_parts;
	int num_parts;
	int ret;

	ncd = kzalloc(sizeof(*ncd), GFP_KERNEL);
	if (!ncd) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "%s: err_alloc\n", __func__);
		goto err_alloc;
	}

	ncd->nand_info = pdev->dev.platform_data;
	if (!ncd->nand_info) {
		ret = -ENODEV;
		dev_dbg(&pdev->dev, "%s: err_pdata\n", __func__);
		goto err_pdata;
	}

	platform_set_drvdata(pdev, ncd);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		ret = -ENODEV;
		dev_dbg(&pdev->dev, "%s: err_get_mem\n", __func__);
		goto err_get_mem;
	}

	if (!request_mem_region(mem->start, mem->end - mem->start,
				DRIVER_NAME)) {
		ret = -EBUSY;
		dev_dbg(&pdev->dev, "%s: err_request_mem\n", __func__);
		goto err_request_mem;
	}

	ncd->chip.IO_ADDR_W = ioremap(mem->start, mem->end - mem->start);
	if (!ncd->chip.IO_ADDR_W) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "%s: err_ioremap\n", __func__);
		goto err_ioremap;
	}
	ncd->chip.IO_ADDR_R = ncd->chip.IO_ADDR_W;
	ncd->mem = mem;

	if (ncd->nand_info->busy_pin) {
		ret = gpio_request(ncd->nand_info->busy_pin, DRIVER_NAME);
		if (ret) {
			dev_dbg(&pdev->dev, "%s: err_gpio_request\n", __func__);
			goto err_gpio_request;
		}
		gpio_direction_input(ncd->nand_info->busy_pin);

		ncd->chip.dev_ready = ccx9x_nand_dev_ready;
	}

	ncd->chip.write_buf = ccx9x_nand_write_buf;
	ncd->chip.read_buf = ccx9x_nand_read_buf;
	ncd->chip.verify_buf = ccx9x_nand_verify_buf;
	ncd->chip.cmd_ctrl = ccx9x_nand_cmd_ctrl;
	ncd->chip.ecc.mode = NAND_ECC_SOFT;
	ncd->chip.chip_delay = ncd->nand_info->delay;
	ncd->chip.controller = NULL;

	ncd->mtd.name = DEVICE_NAME;
	ncd->mtd.priv = &ncd->chip;

	if (nand_scan(&ncd->mtd, 1)) {
		ret = -ENODEV;
		dev_dbg(&pdev->dev, "%s: err_scan\n", __func__);
		goto err_scan;
	}

	dev_dbg(&pdev->dev, "NAND Flash memory mapped to virtual %p\n",
			ncd->chip.IO_ADDR_W);

#ifdef CONFIG_MTD_CMDLINE_PARTS
	/* read partition information from cmdline*/
	num_parts = parse_mtd_partitions(&ncd->mtd, probes, &mtd_parts, 0);

	/* register partitions */
	add_mtd_partitions(&ncd->mtd, mtd_parts, num_parts);
#endif

	return 0;

err_scan:
	if (ncd->nand_info->busy_pin)
		gpio_free(ncd->nand_info->busy_pin);
err_gpio_request:
	iounmap(ncd->chip.IO_ADDR_W);
err_ioremap:
	release_mem_region(mem->start, mem->end - mem->start);
err_request_mem:
	release_resource(mem);
err_get_mem:
err_pdata:
	kfree(ncd);
err_alloc:
	return ret;
}

static int ccx9x_nand_remove(struct platform_device *pdev)
{
	struct nand_ccx9x *ncd = platform_get_drvdata(pdev);

	if (ncd->nand_info->busy_pin)
		gpio_free(ncd->nand_info->busy_pin);
	iounmap(ncd->chip.IO_ADDR_W);
	release_mem_region(ncd->mem->start,
			ncd->mem->end - ncd->mem->start);
	release_resource(ncd->mem);
	kfree(ncd);

	return 0;
}

static struct platform_driver ccx9x_nand_driver = {
	.probe = ccx9x_nand_probe,
	.remove = ccx9x_nand_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ccx9x_nand_init(void)
{
	int ret = -ENOMEM;

	ret = platform_driver_register(&ccx9x_nand_driver);
	if (ret) {
		pr_debug("%s: err_pdev_register\n", __func__);
		goto err_pdev_register;
	}

	return 0;

err_pdev_register:
	return ret;
}

static void __exit ccx9x_nand_exit(void)
{
	platform_driver_unregister(&ccx9x_nand_driver);
}

module_init(ccx9x_nand_init);
module_exit(ccx9x_nand_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Matthias Ludwig");
MODULE_DESCRIPTION("Digi CCx9x MTD NAND driver");

/*
 * drivers/crypto/ns921x-aes.c
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/dma-ns921x.h>
#include <linux/sched.h>

#define DMA_BDPOINTER	0x0

#define DMA_CONTROL	0x4
#define DMA_CONTROL_SW32	(1 << 28)
#define DMA_CONTROL_DW32	(1 << 26)
#define DMA_CONTROL_RST		(1 << 16)

#define DMA_STATUS		0x8
#define DMA_MAX_BUFFERS		256
#define DRIVER_NAME "ns921x-aes"

union ns9xxx_dma_desc {
	struct {
		u32 source;
		u16 source_len;
		u16 dest_len;
		u32 dest;

#define ENCMODE_CBC	0
#define ENCMODE_CFB	1
#define ENCMODE_OFB	2
#define ENCMODE_CTR	3
#define ENCMODE_ECB	4
#define ENCMODE_CCM	5
#define ENCMODE_KEYEXP	7

#define	ENCACTION_ENC	(0 << 3)
#define	ENCACTION_DEC	(1 << 3)
#define ECCKEYSIZE_128	(0 << 4)
#define ENCKEYSIZE_192	(1 << 4)
#define ENCKEYSIZE_256	(2 << 4)
#define ENCKEYSIZE(i)	((i == 16) ? ECCKEYSIZE_128 : \
		(i == 24) ? ENCKEYSIZE_192 : ENCKEYSIZE_256)
		u16 control;
#define ENCOP_NONAES	0
#define ENCOP_KEY	1
#define ENCOP_IV	2
#define ENCOP_NONCE	3
#define ENCOP_ADD	4
#define ENCOP_DATA	5
		u16 flags;
	};
	u32 data[4];
};

#define KEYEXP_SIZE128	0x176
#define KEYEXP_SIZE192	0x208
#define KEYEXP_SIZE256	0x240
#define KEYEXP_SIZE(i)	((i == 16) ? KEYEXP_SIZE128 : \
		(i == 24) ? KEYEXP_SIZE192 : KEYEXP_SIZE256)

struct ns921x_aes_data {
	struct resource		*mem;
	struct clk		*clk;

	struct platform_device	*pdev;

	wait_queue_head_t	waitq;

	void __iomem		*membase;
	dma_addr_t		p_dma_descr;
	union ns9xxx_dma_desc	*v_dma_descr;

	dma_addr_t		p_key;
	u8			*v_key;
	int			keylen;

	int			irq;
	int			irq_pending;
};

struct ns921x_aes_data dev_data;

static inline void cpu_to_be(u32 *ptr, unsigned num)
{
	unsigned i;

	for (i = 0; i < num; i++)
		ptr[i] = cpu_to_be32(ptr[i]);
}

static inline void be_to_cpu(u32 *ptr, unsigned num)
{
	unsigned i;

	for (i = 0; i < num; i++)
		ptr[i] = be32_to_cpu(ptr[i]);
}

static irqreturn_t ns921x_aes_int(int irq, void *dev_id)
{
	u32 status;

	status = ioread32(dev_data.membase + DMA_STATUS);
	if (!(status & NS921X_DMA_STIE_NCIP) ) {
		/* check dma errors */
		if ( status & (NS921X_DMA_STIE_ECIP |
			NS921X_DMA_STIE_NRIP | NS921X_DMA_STIE_CAIP |
			NS921X_DMA_STIE_PCIP) )
			dev_dbg(&dev_data.pdev->dev,
				"DMA error. Status = 0x%08x\n", status);
	}
	/* ack interrupt */
	iowrite32(status, dev_data.membase + DMA_STATUS);

	/* disable dma channel*/
	iowrite32(0, dev_data.membase + DMA_CONTROL);

	/* wake up caller*/
	dev_data.irq_pending = 0;
	wake_up(&dev_data.waitq);

	return IRQ_HANDLED;
}

static int ns921x_aes_cra_init(struct crypto_tfm *tfm)
{
	if (dev_data.irq_pending)
		return -EBUSY;

	memset(dev_data.v_dma_descr, 0,
		DMA_MAX_BUFFERS * sizeof(*dev_data.v_dma_descr));

	return 0;
}

static int ns921x_aes_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
		unsigned int keylen)
{
	int ret;

	if (!(keylen == 16 || keylen == 24 || keylen == 32)) {
		ret = -EINVAL;
		dev_dbg(&dev_data.pdev->dev, "err_keylen\n");
		goto err_keylen;
	}

	/* Free DMA allocation, just in case this function was
	 * called but there was not a call to crypt or decrypt */
	if (NULL != dev_data.v_key) {
		dma_free_coherent(&dev_data.pdev->dev, dev_data.keylen,
				dev_data.v_key, dev_data.p_key);
		dev_data.v_key = NULL;
	}

	dev_data.v_key = dma_alloc_coherent(&dev_data.pdev->dev,
			keylen, &dev_data.p_key, GFP_KERNEL);
	if (!dev_data.v_key) {
		ret = -ENOMEM;
		dev_dbg(&dev_data.pdev->dev, "err_alloc\n");
		goto err_alloc;
	}

	memcpy(dev_data.v_key, key, keylen);
	dev_data.keylen = keylen;

	cpu_to_be((u32 *)dev_data.v_key, dev_data.keylen / 4);

	/* setup dma descr. for key buffer, .control is set in
	 * de-/encryption routine as it depends on the mode */
	dev_data.v_dma_descr[0].source = (u32)dev_data.p_key;
	dev_data.v_dma_descr[0].source_len = dev_data.keylen;
	dev_data.v_dma_descr[0].flags =
		ENCOP_KEY | EXT_DMA_DESC_CTRL_FULL;

	return 0;

err_alloc:
err_keylen:
	return ret;
}

int ns921x_aes_dma(unsigned timeout)
{
	u32 ctrl;

	/* fire up dma */
	iowrite32(NS921X_DMA_STIE_NCIE | NS921X_DMA_STIE_ECIE |
		NS921X_DMA_STIE_NRIE | NS921X_DMA_STIE_NRIE |
		NS921X_DMA_STIE_CAIE |NS921X_DMA_STIE_PCIE,
		dev_data.membase + DMA_STATUS);
	ctrl = DMA_CONTROL_SW32 | NS921X_DMA_CR_SW_32b |
		NS921X_DMA_CR_CE | NS921X_DMA_CR_CG;
	iowrite32(ctrl, dev_data.membase + DMA_CONTROL);

	/* wait for isr */
	dev_data.irq_pending = 1;
	if (!wait_event_timeout(dev_data.waitq,
				(dev_data.irq_pending == 0), timeout)) {
		dev_err(&dev_data.pdev->dev, "interrupt timed out! Retrying\n" );
		dev_dbg(&dev_data.pdev->dev, "DMA_STATUS = 0x%x\n",
				ioread32(dev_data.membase + DMA_STATUS) );
		return -EAGAIN;
	}

	return 0;
}

int ns921x_aes_crypt(struct ablkcipher_request *req, int iv)
{
	int ret = 0, i, n;
	int v_buffers, p_buffers;

	/* get number of used pages */
	for (v_buffers = 0, n = 0; n < req->nbytes; v_buffers++) {
		/* ensure big endian mode of input data */
		cpu_to_be((u32 *)page_address(sg_page(&req->src[v_buffers]))
				+ req->src[v_buffers].offset / 4,
				req->src[v_buffers].length / 4);

		n += req->src[v_buffers].length;
	}

	/* src must be same page as dst */
	if (req->src->page_link != req->dst->page_link) {
		ret = -EINVAL;
		goto err_mismatch;
	}


	/* map pages */
map:	p_buffers = dma_map_sg(&dev_data.pdev->dev, req->dst,
			v_buffers, DMA_BIDIRECTIONAL);

	/* create dma descriptors for every used page */
	for (i = 0; i < p_buffers; i++) {
		/* setup dma descriptors */
		dev_data.v_dma_descr[1 + iv + i].source =
			(u32)sg_dma_address(req->src + i);
		dev_data.v_dma_descr[1 + iv + i].source_len =
			(u16)sg_dma_len(req->src + i);
		dev_data.v_dma_descr[1 + iv + i].dest =
			(u32)sg_dma_address(req->src + i);
		dev_data.v_dma_descr[1 + iv + i].flags =
			ENCOP_DATA | EXT_DMA_DESC_CTRL_FULL;
	}

	/* add additional dma flags to last descriptor */
	dev_data.v_dma_descr[iv + p_buffers].flags |=
		EXT_DMA_DESC_CTRL_WRAP | EXT_DMA_DESC_CTRL_LAST;

	/* let hardware do its work */
	ret = ns921x_aes_dma(HZ * req->src->length / 100);

	/* release dma mappings */
	dma_sync_sg_for_cpu(&dev_data.pdev->dev, req->src,
			v_buffers, DMA_BIDIRECTIONAL);
	dma_unmap_sg(&dev_data.pdev->dev, req->src, i, DMA_BIDIRECTIONAL);
	/* If DMA transmission did not complete, try again */
	if (-EAGAIN == ret )
		goto map;

err_mismatch:
	/* overwrite key buffer to avoid security breakage! */
	memset(dev_data.v_key, 0, dev_data.keylen);
	dma_free_coherent(&dev_data.pdev->dev, dev_data.keylen,
			dev_data.v_key, dev_data.p_key);
	dev_data.v_key = NULL;

	/* convert output back to cpu endianes */
	for (i = 0; i < v_buffers; i++)
		be_to_cpu((u32 *)page_address(sg_page(&req->dst[i])) +
				req->dst[i].offset / 4, req->dst[i].length / 4);

	return ret;
}

static int ns921x_aes_keyexpander(void)
{
	int ret;
	dma_addr_t p_expkey;
	u8 *v_expkey;

	v_expkey = dma_alloc_coherent(&dev_data.pdev->dev,
			KEYEXP_SIZE(dev_data.keylen),
			&p_expkey, GFP_KERNEL);
	if (!v_expkey)
		return -ENOMEM;

buff:	dev_data.v_dma_descr[0].control =
		ENCMODE_KEYEXP | ENCKEYSIZE(dev_data.keylen);

	dev_data.v_dma_descr[1].source_len = KEYEXP_SIZE(dev_data.keylen);
	dev_data.v_dma_descr[1].dest = p_expkey;
	dev_data.v_dma_descr[1].flags =
		ENCOP_DATA | EXT_DMA_DESC_CTRL_WRAP |
		EXT_DMA_DESC_CTRL_FULL | EXT_DMA_DESC_CTRL_LAST;

	ret = ns921x_aes_dma(HZ/10);
	if (!ret) {
		switch (dev_data.keylen) {
		case 16:
			memcpy(dev_data.v_key, (u8 *)v_expkey + 160, 16);
			break;
		case 24:
			memcpy(dev_data.v_key, (u8 *)v_expkey + 192, 16);
			memcpy(dev_data.v_key + 16, (u8 *)v_expkey + 184, 8);
			break;
		case 32:
			memcpy(dev_data.v_key, (u8 *)v_expkey + 224, 16);
			memcpy(dev_data.v_key + 16, (u8 *)v_expkey + 208, 16);
			break;
		}

		/* reset used dma_descriptors */
		memset(dev_data.v_dma_descr, 0,
				2 * sizeof(*dev_data.v_dma_descr));
		dev_data.v_dma_descr[0].source = (u32)dev_data.p_key;
		dev_data.v_dma_descr[0].source_len = dev_data.keylen;
		dev_data.v_dma_descr[0].flags =
			ENCOP_KEY | EXT_DMA_DESC_CTRL_FULL ;
	}
	else if (-EAGAIN == ret)
		goto buff; /* retry dma */

	/* destroy expanded key */
	memset(v_expkey, 0, KEYEXP_SIZE(dev_data.keylen));
	dma_free_coherent(&dev_data.pdev->dev, KEYEXP_SIZE(dev_data.keylen),
			v_expkey, p_expkey);
	
	/*
	 * We need the key for the next operations, or? Fixed Vantive 31365
	 * (Luis Galdos)
	 */
	/* dev_data.v_key = NULL; */

	return ret;
}

static int ns921x_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	dev_data.v_dma_descr[0].control =
		ENCMODE_ECB | ENCKEYSIZE(dev_data.keylen) | ENCACTION_ENC;
	return ns921x_aes_crypt(req, 0);
}

static int ns921x_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	int ret;

	ret = ns921x_aes_keyexpander();
	if (!ret) {
		dev_data.v_dma_descr[0].control = ENCMODE_ECB |
			ENCKEYSIZE(dev_data.keylen) | ENCACTION_DEC;
		ret = ns921x_aes_crypt(req, 0);
	}

	return ret;
}

static int ns921x_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	int ret, ivsize;
	u8 *v_iv;
	dma_addr_t p_iv;

	dev_data.v_dma_descr[0].control =
		ENCMODE_CBC | ENCKEYSIZE(dev_data.keylen) | ENCACTION_ENC;

	ivsize = 16;
	/* XXX: crypto_ablkcipher_ivsize(crypto_ablkcipher_reqtfm(req)); */
	v_iv = dma_alloc_coherent(&dev_data.pdev->dev, ivsize,
			&p_iv, GFP_KERNEL);

	memcpy(v_iv, req->info, ivsize);
	cpu_to_be((u32 *)v_iv, ivsize);

	dev_data.v_dma_descr[1].source = (u32)p_iv;
	dev_data.v_dma_descr[1].source_len = ivsize;
	dev_data.v_dma_descr[1].flags =
		EXT_DMA_DESC_CTRL_FULL | ENCOP_IV;

	ret = ns921x_aes_crypt(req, 1);

	dma_free_coherent(&dev_data.pdev->dev, ivsize, v_iv, p_iv);

	return ret;
return -ENOMEM;
}

static int ns921x_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	int ret, ivsize;
	u8 *v_iv;
	dma_addr_t p_iv;

	ret = ns921x_aes_keyexpander();
	if (!ret) {
		dev_data.v_dma_descr[0].control = ENCMODE_CBC |
			ENCKEYSIZE(dev_data.keylen) | ENCACTION_DEC;

		ivsize = 16;
		/* XXX: crypto_ablkcipher_ivsize(crypto_ablkcipher_reqtfm(req)); */
		v_iv = dma_alloc_coherent(&dev_data.pdev->dev, ivsize,
				&p_iv, GFP_KERNEL);

		memcpy(v_iv, req->info, ivsize);
		cpu_to_be((u32 *)v_iv, ivsize);

		dev_data.v_dma_descr[1].source = (u32)p_iv;
		dev_data.v_dma_descr[1].source_len = ivsize;
		dev_data.v_dma_descr[1].flags =
			 EXT_DMA_DESC_CTRL_FULL | ENCOP_IV;

		ret = ns921x_aes_crypt(req, 1);

		dma_free_coherent(&dev_data.pdev->dev, ivsize, v_iv, p_iv);
	}

	return ret;
return -ENOMEM;
}

static struct ablkcipher_alg ciphers[] = {
	{
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.setkey		= ns921x_aes_setkey,
		.encrypt	= ns921x_aes_ecb_encrypt,
		.decrypt	= ns921x_aes_ecb_decrypt,
	}, {

		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.setkey		= ns921x_aes_setkey,
		.encrypt	= ns921x_aes_cbc_encrypt,
		.decrypt	= ns921x_aes_cbc_decrypt,
	},
};

static struct crypto_alg ns921x_aes_algs[] = {
	{
		.cra_name = "ecb(aes)",
		.cra_driver_name = DRIVER_NAME,
		.cra_priority = 400,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = 16,
		.cra_alignmask = 15,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = ns921x_aes_cra_init,
		.cra_list = LIST_HEAD_INIT(ns921x_aes_algs[0].cra_list),
	}, {
		.cra_name = "cbc(aes)",
		.cra_driver_name = DRIVER_NAME,
		.cra_priority = 0,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = 16,
		.cra_alignmask = 15,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = ns921x_aes_cra_init,
		.cra_list = LIST_HEAD_INIT(ns921x_aes_algs[1].cra_list),
	},
};

static int __devinit ns921x_aes_probe(struct platform_device *pdev)
{
	int i, ret = -ENOMEM;

	memset(&dev_data, 0, sizeof(dev_data));
	dev_data.pdev = pdev;

	dev_data.mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!dev_data.mem) {
		ret = -ENODEV;
		dev_info(&pdev->dev, "err_get_mem\n");
		goto err_get_mem;
	}

	if (!request_mem_region(dev_data.mem->start,
				dev_data.mem->end - dev_data.mem->start + 1,
				DRIVER_NAME)) {
		ret = -EBUSY;
		dev_info(&pdev->dev, "err_request_mem\n");
		goto err_request_mem;
	}

	dev_data.membase = ioremap(dev_data.mem->start,
			dev_data.mem->end - dev_data.mem->start + 1);
	if (!dev_data.membase) {
		ret = -EBUSY;
		dev_info(&pdev->dev, "err_ioremap\n");
		goto err_ioremap;
	}

	dev_data.irq = platform_get_irq(pdev, 0);
	if (dev_data.irq <= 0) {
		ret = -EINVAL;
		dev_info(&pdev->dev, "err_get_irq\n");
		goto err_get_irq;
	}

	ret = request_irq(dev_data.irq, ns921x_aes_int, 0, DRIVER_NAME, pdev);
	if (ret) {
		dev_info(&pdev->dev, "err_request_irq\n");
		goto err_request_irq;
	}

	dev_data.clk = clk_get(&pdev->dev, DRIVER_NAME);
	if (IS_ERR(dev_data.clk)) {
		ret = PTR_ERR(dev_data.clk);
		dev_info(&pdev->dev, "err_clk_get\n");
		goto err_clk_get;
	}

	ret = clk_enable(dev_data.clk);
	if (ret) {
		ret = -EBUSY;
		dev_info(&pdev->dev, "err_clk_enable\n");
		goto err_clk_enable;
	}

	/* register memory for maximal number of dma descriptors */
	pdev->dev.coherent_dma_mask = (u32)-1;
	dev_data.v_dma_descr = dma_alloc_coherent(&pdev->dev,
			sizeof(*dev_data.v_dma_descr) * DMA_MAX_BUFFERS,
			&dev_data.p_dma_descr, GFP_KERNEL);
	if (!dev_data.v_dma_descr) {
		ret = -ENOMEM;
		goto err_dma_descr;
	}

	iowrite32(DMA_CONTROL_RST, dev_data.membase + DMA_CONTROL);
	iowrite32((u32)dev_data.p_dma_descr, dev_data.membase + DMA_BDPOINTER);

	init_waitqueue_head(&dev_data.waitq);

	for (i = 0; i < ARRAY_SIZE(ns921x_aes_algs); i++) {
		ns921x_aes_algs[i].cra_u.ablkcipher = ciphers[i];
		ret = crypto_register_alg(&ns921x_aes_algs[i]);
		if (ret) {
			dev_info(&pdev->dev, "err_register\n");
			goto err_register;
		}
	}

	dev_info(&pdev->dev, "NS921x AES encryption/decryption module at 0x%p (irq: %d)\n",
			dev_data.membase, dev_data.irq);

	return 0;

err_register:
	while (--i >= 0)
		crypto_unregister_alg(&ns921x_aes_algs[i]);
	dma_free_coherent(&pdev->dev,
			sizeof(*dev_data.v_dma_descr) * DMA_MAX_BUFFERS,
			dev_data.v_dma_descr, dev_data.p_dma_descr);
err_dma_descr:
	clk_disable(dev_data.clk);
err_clk_enable:
	clk_put(dev_data.clk);
err_clk_get:
	free_irq(dev_data.irq, pdev);
err_request_irq:
err_get_irq:
	iounmap(dev_data.membase);
err_ioremap:
	release_mem_region(dev_data.mem->start,
			dev_data.mem->end - dev_data.mem->start + 1);
err_request_mem:
err_get_mem:

	return ret;
}

static int __devexit ns921x_aes_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ns921x_aes_algs); i++)
		crypto_unregister_alg(&ns921x_aes_algs[i]);

	dma_free_coherent(&pdev->dev,
			sizeof(*dev_data.v_dma_descr) * DMA_MAX_BUFFERS,
			dev_data.v_dma_descr, dev_data.p_dma_descr);

	clk_disable(dev_data.clk);
	clk_put(dev_data.clk);

	free_irq(dev_data.irq, pdev);

	iounmap(dev_data.membase);
	release_mem_region(dev_data.mem->start,
			dev_data.mem->end - dev_data.mem->start + 1);

	return 0;
}

static struct platform_driver ns921x_aes_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= ns921x_aes_probe,
	.remove		= __devexit_p(ns921x_aes_remove),
};

static int __init ns921x_aes_init(void)
{
	return platform_driver_register(&ns921x_aes_driver);
}

static void __exit ns921x_aes_exit(void)
{
	platform_driver_unregister(&ns921x_aes_driver);
}

module_init(ns921x_aes_init);
module_exit(ns921x_aes_exit);

MODULE_DESCRIPTION("Digi ns921x AES algorithm support");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Digi International Inc.");

MODULE_ALIAS("aes");

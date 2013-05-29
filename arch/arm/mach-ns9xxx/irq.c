/*
 * arch/arm/mach-ns9xxx/irq.c
 *
 * Copyright (C) 2006-2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/interrupt.h>
#include <linux/kernel_stat.h>
#include <asm/io.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>
#include <mach/regs-bbu.h>
#include <mach/regs-sys-common.h>
#include <mach/irqs.h>
#include <mach/board.h>
#include <mach/processor.h>

#include "processor-ns921x.h"
#include "irq.h"

static unsigned prio2irq(unsigned prio)
{
	u32 ic = __raw_readl(SYS_IC(prio / 4));
	return REGGETIM_IDX(ic, SYS_IC, ISD, __SYS_IC_FIELDNUM(prio));
}

static unsigned irq2prio_map[32];

static unsigned irq2prio(unsigned irq)
{
	unsigned cachedprio = irq2prio_map[irq];
	int timeout;

	if (likely(irq == prio2irq(cachedprio)))
		return cachedprio;

	for (timeout = 32; timeout; --timeout) {
		static unsigned i;
		unsigned iirq;

		i = (i + 1) % 32;

		iirq = prio2irq(i);

		irq2prio_map[iirq] = i;

		pr_debug("%s: update %u -> %u\n", __func__, iirq, i);

		if (iirq == irq)
			return i;
	}

	BUG();

	return 0; /* not reached, clean compiler warning */
}

#define prio2irq_init(p) (p)

static void ns9xxx_mask_irq(unsigned int irq)
{
	int prio = irq2prio(irq);
	u32 ic = __raw_readl(SYS_IC(prio / 4));
	REGSET_IDX(ic, SYS_IC, IE, __SYS_IC_FIELDNUM(prio), DIS);
	__raw_writel(ic, SYS_IC(prio / 4));
}

int ns9xxx_is_enabled_irq(unsigned int irq)
{
	int prio = irq2prio(irq);
	u32 ic, en_bit_mask;

	en_bit_mask = 0x80 << ((3 - (prio & 3)) * 8);
	ic = __raw_readl(SYS_IC(prio / 4));

	return ic & en_bit_mask;
}
EXPORT_SYMBOL(ns9xxx_is_enabled_irq);

static void ns9xxx_disable_irq(unsigned int irq)
{
	struct irq_desc *desc = irq_desc + irq;

	ns9xxx_mask_irq(irq);
	desc->status &= IRQ_MASKED;
}

static void ns9xxx_ack_irq(unsigned int irq)
{
	if (irq >= IRQ_NS9XXX_EXT0) {
		u32 eixctl;

		BUG_ON((unsigned)(irq - IRQ_NS9XXX_EXT0) > 4);

		eixctl = __raw_readl(SYS_EIxCTRL(irq - IRQ_NS9XXX_EXT0));

		if (REGGET(eixctl, SYS_EIxCTRL, TYPE) ==
				SYS_EIxCTRL_TYPE_EDGE) {
			REGSETIM(eixctl, SYS_EIxCTRL, CLEAR, 1);
			__raw_writel(eixctl,
					SYS_EIxCTRL(irq - IRQ_NS9XXX_EXT0));

			REGSETIM(eixctl, SYS_EIxCTRL, CLEAR, 0);
			__raw_writel(eixctl,
					SYS_EIxCTRL(irq - IRQ_NS9XXX_EXT0));
		}
	}
}

static void ns9xxx_maskack_irq(unsigned int irq)
{
	ns9xxx_mask_irq(irq);
	ns9xxx_ack_irq(irq);
}

static void ns9xxx_unmask_irq(unsigned int irq)
{
	int prio = irq2prio(irq);
	u32 ic = __raw_readl(SYS_IC(prio / 4));
	REGSET_IDX(ic, SYS_IC, IE, __SYS_IC_FIELDNUM(prio), EN);
	__raw_writel(ic, SYS_IC(prio / 4));
}

static void ns9xxx_eoi_irq(unsigned int irq)
{
	__raw_writel(irq2prio(irq), SYS_ISRADDR);
}

static int ns9xxx_set_type_irq(unsigned int irq, unsigned int flow_type)
{
	void __iomem *regeixctrl = SYS_EIxCTRL(irq - IRQ_NS9XXX_EXT0);
	u32 eixctrl = 0;

	REGSETIM(eixctrl, SYS_EIxCTRL, CLEAR, 1);

	if (irq < IRQ_NS9XXX_EXT0 || irq > IRQ_NS9XXX_EXT3)
		return -EINVAL;

	switch (flow_type) {
	case IRQF_TRIGGER_HIGH:
		REGSET(eixctrl, SYS_EIxCTRL, PLTY, HIGH);
		REGSET(eixctrl, SYS_EIxCTRL, TYPE, LEVEL);
		break;

	case IRQF_TRIGGER_LOW:
		REGSET(eixctrl, SYS_EIxCTRL, PLTY, LOW);
		REGSET(eixctrl, SYS_EIxCTRL, TYPE, LEVEL);
		break;

	case IRQF_TRIGGER_RISING:
		REGSET(eixctrl, SYS_EIxCTRL, PLTY, HIGH);
		REGSET(eixctrl, SYS_EIxCTRL, TYPE, EDGE);
		break;

	case IRQF_TRIGGER_FALLING:
		REGSET(eixctrl, SYS_EIxCTRL, PLTY, LOW);
		REGSET(eixctrl, SYS_EIxCTRL, TYPE, EDGE);
		break;

	default:
		pr_warning("%s: cannot configure for flow type %u\n",
				__func__, flow_type);
		return -ENODEV;
	}

	__raw_writel(eixctrl, regeixctrl);
	REGSETIM(eixctrl, SYS_EIxCTRL, CLEAR, 0);
	__raw_writel(eixctrl, regeixctrl);

	return 0;
}

static int ns9xxx_set_wake_irq(unsigned int irq, unsigned int on)
{
#if defined(CONFIG_PROCESSOR_NS921X)
	if (processor_is_ns921x())
		return ns921x_set_wake_irq(irq, on);
#endif

	return -EINVAL;
}

static struct irq_chip ns9xxx_chip = {
	.name		= "ns9xxx",
	.disable	= ns9xxx_disable_irq,
	.ack		= ns9xxx_ack_irq,
	.mask		= ns9xxx_mask_irq,
	.mask_ack	= ns9xxx_maskack_irq,
	.unmask		= ns9xxx_unmask_irq,
	.eoi		= ns9xxx_eoi_irq,
	.set_type	= ns9xxx_set_type_irq,
	.set_wake	= ns9xxx_set_wake_irq,
};

#if defined(CONFIG_PROCESSOR_NS9360)
static void ns9360_mask_bbus_irq(unsigned int irq)
{
	u32 ier = __raw_readl(NS9360_BBUS_IEN);
	ier &= ~(1 << (irq - IRQ_NS9360_BBUS(0)));
	__raw_writel(ier, NS9360_BBUS_IEN);
}

static void ns9360_unmask_bbus_irq(unsigned int irq)
{
	u32 ier = __raw_readl(NS9360_BBUS_IEN);
	ier |= 1 << (irq - IRQ_NS9360_BBUS(0));
	__raw_writel(ier, NS9360_BBUS_IEN);
}

static void ns9360_demux_bbus_irq(unsigned int irq, struct irq_desc *desc)
{
	unsigned bbus_irq_plus1;
	u32 stat = __raw_readl(NS9360_BBUS_ISTAT);

	while ((bbus_irq_plus1 = fls(stat))) {
		unsigned bbus_irq = bbus_irq_plus1 - 1;
		stat &= ~(1 << bbus_irq);

		desc_handle_irq(IRQ_NS9360_BBUS(bbus_irq),
				irq_desc + IRQ_NS9360_BBUS(bbus_irq));
	}

	/* unmask parent irq */
	desc->chip->unmask(irq);
	desc->chip->eoi(irq);
}

static struct irq_chip ns9360_bbus_chip = {
	.name		= "ns9xxx_bbus",
	.ack		= ns9360_mask_bbus_irq,
	.mask		= ns9360_mask_bbus_irq,
	.mask_ack	= ns9360_mask_bbus_irq,
	.unmask		= ns9360_unmask_bbus_irq,
};

static void ns9360_mask_bbus_dma_irq(unsigned int irq)
{
	u32 ien = __raw_readl(NS9360_BBUS_DMA_IEN);
	ien &= ~(1 << (irq - IRQ_NS9360_BBUDMA(0)));
	__raw_writel(ien, NS9360_BBUS_DMA_IEN);
}

static void ns9360_unmask_bbus_dma_irq( unsigned int irq )
{
	u32 ien = __raw_readl(NS9360_BBUS_DMA_IEN);
	ien |= 1 << (irq - IRQ_NS9360_BBUDMA(0));
	__raw_writel(ien, NS9360_BBUS_DMA_IEN);
}

static void ns9360_demux_bbus_dma_irq(unsigned int irq, struct irq_desc *desc)
{
	unsigned dma_irq_plus1;
	u32 stat = __raw_readl(NS9360_BBUS_DMA_ISTAT);

	/* mask parent irq */
	desc->chip->mask_ack(irq);

	while ((dma_irq_plus1 = fls(stat))) {
		unsigned dma_irq = dma_irq_plus1 - 1;
		stat &= ~(1 << dma_irq);

		desc_handle_irq(IRQ_NS9360_BBUDMA(dma_irq),
				irq_desc + IRQ_NS9360_BBUDMA(dma_irq));
	}

	/* unmask parent */
	desc->chip->unmask(irq);
}

static struct irq_chip ns9360_bbus_dma_chip = {
	.name		= "ns9xxx_bbus_dma",
	.ack		= ns9360_mask_bbus_dma_irq,
	.mask		= ns9360_mask_bbus_dma_irq,
	.mask_ack	= ns9360_mask_bbus_dma_irq,
	.unmask		= ns9360_unmask_bbus_dma_irq,
};
#endif /* if defined(CONFIG_PROCESSOR_NS9360) */

extern int noirqdebug;

/* this is similar to handle_fasteoi_irq.  The differences are:
 *  - handle_prio_irq calls desc->chip->ack at the beginning;
 *  - handle_prio_irq disables an irq directly after handle_IRQ_event to work
 *    around the bug in the ns9xxx' irq priority encoder;
 *  - currently some debug code;
 */
static void handle_prio_irq(unsigned int irq, struct irq_desc *desc)
{
	struct irqaction *action;
	irqreturn_t action_ret;

	raw_spin_lock(&desc->lock);

	desc->chip->ack(irq);

	if (unlikely(desc->status & IRQ_INPROGRESS)) {
		desc->status |= IRQ_PENDING;
		goto out_unlock;
	}

	desc->status &= ~(IRQ_REPLAY | IRQ_WAITING);
	kstat_incr_irqs_this_cpu(irq, desc);

	action = desc->action;
	if (unlikely(!action || (desc->status & IRQ_DISABLED))) {
		desc->status |= IRQ_PENDING;
		goto out_mask;
	}

	desc->status |= IRQ_INPROGRESS;
	desc->status &= ~IRQ_PENDING;
	raw_spin_unlock(&desc->lock);

	action_ret = handle_IRQ_event(irq, action);
	if (!noirqdebug)
		note_interrupt(irq, desc, action_ret);

	raw_spin_lock(&desc->lock);
	desc->status &= ~IRQ_INPROGRESS;

	if (desc->status & IRQ_DISABLED)
out_mask:
		desc->chip->mask(irq);

	desc->chip->eoi(irq);

out_unlock:
	raw_spin_unlock(&desc->lock);
}

void __init ns9xxx_init_irq(void)
{
	int i;

	/* disable all IRQs */
	for (i = 0; i < 8; ++i) {
		u32 ic = 0;

		REGSETIM_IDX(ic, SYS_IC, ISD, __SYS_IC_FIELDNUM(0),
				prio2irq_init(4 * i));
		REGSETIM_IDX(ic, SYS_IC, ISD, __SYS_IC_FIELDNUM(1),
				prio2irq_init(4 * i + 1));
		REGSETIM_IDX(ic, SYS_IC, ISD, __SYS_IC_FIELDNUM(2),
				prio2irq_init(4 * i + 2));
		REGSETIM_IDX(ic, SYS_IC, ISD, __SYS_IC_FIELDNUM(3),
				prio2irq_init(4 * i + 3));

		__raw_writel(ic, SYS_IC(i));
	}

	for (i = 0; i < 32; ++i)
		__raw_writel(prio2irq(i), SYS_IVA(i));

	for (i = 0; i <= 31; ++i) {
		set_irq_chip(i, &ns9xxx_chip);
		set_irq_handler(i, handle_prio_irq);
		set_irq_flags(i, IRQF_VALID);
	}

#ifdef CONFIG_PROCESSOR_NS9360
	if (processor_is_ns9360()) {
		/* set up the BBUS interrupt handlers */
		__raw_writel(NS9360_BBUS_IEN_GLBL, NS9360_BBUS_IEN);
		for (i = IRQ_NS9360_BBUS(0); i <= IRQ_NS9360_BBUS(25); i++) {
			set_irq_chip(i, &ns9360_bbus_chip);
			set_irq_handler(i, handle_level_irq);
			set_irq_flags(i, IRQF_VALID);
		}
		set_irq_chained_handler(IRQ_NS9360_BBUSAGG,
				ns9360_demux_bbus_irq);

		/* set up the BBUS DMA interrupt handlers */
		__raw_writel(0, NS9360_BBUS_DMA_IEN);
		for (i = IRQ_NS9360_BBUDMA(0); i <= IRQ_NS9360_BBUDMA(15); i++) {
			set_irq_chip(i, &ns9360_bbus_dma_chip);
			set_irq_handler(i, handle_level_irq);
			set_irq_flags(i, IRQF_VALID);
		}
		set_irq_chained_handler(IRQ_NS9360_BBUS_DMA,
				ns9360_demux_bbus_dma_irq);
	}
#endif
}

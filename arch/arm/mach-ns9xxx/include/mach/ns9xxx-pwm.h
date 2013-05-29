/*
 * drivers/pwm/ns9xxx-pwm.h
 *
 * Copyright (C) 2009 Digi International Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __NS9XXX_PWM_HEADER_H
#define __NS9XXX_PWM_HEADER_H

/* Maximal number of available channels */
#define NS9XXX_PWM_CHANNEL_MAX		10

/* This is the data for the PWM channels */
struct ns9xxx_pwm_channel {
	int timer;
	int gpio;

	/* Additional channel configuration variables ... */
};

/* */
struct ns9xxx_pwm_pdata {
	unsigned int number_channels;
 	struct ns9xxx_pwm_channel *channels;
};




#define NS921X_TMC_T0E			(1 << 0)	/* Timer enable */
#define NS921X_TMC_T1E			(1 << 1)
#define NS921X_TMC_T2E			(1 << 2)
#define NS921X_TMC_T3E			(1 << 3)
#define NS921X_TMC_T4E			(1 << 4)
#define NS921X_TMC_T5E			(1 << 5)
#define NS921X_TMC_T6E			(1 << 6)
#define NS921X_TMC_T7E			(1 << 7)
#define NS921X_TMC_T8E			(1 << 8)
#define NS921X_TMC_T9E			(1 << 9)
#define NS921X_TMC_T6HSE		(1 << 10)	/* High step enable */
#define NS921X_TMC_T6LSE		(1 << 11)	/* Low step enable */
#define NS921X_TMC_T6RSE		(1 << 12)	/* Reload step enable */
#define NS921X_TMC_T7HSE		(1 << 13)
#define NS921X_TMC_T7LSE		(1 << 14)
#define NS921X_TMC_T7RSE		(1 << 15)
#define NS921X_TMC_T8HSE		(1 << 16)
#define NS921X_TMC_T8LSE		(1 << 17)
#define NS921X_TMC_T8RSE		(1 << 18)
#define NS921X_TMC_T9HSE		(1 << 19)
#define NS921X_TMC_T9LSE		(1 << 20)
#define NS921X_TMC_T9RSE		(1 << 21)

#define NS921X_TCR_RELOADEN		(1 << 0)	/* Reload enable */
#define NS921X_TCR_BITTIMER		(1 << 1)	/* 32 or 16 bit timer */
#define NS921X_TCR_UPDOWN		(1 << 2)	/* Up/Down select */
#define NS921X_TCR_INTSEL		(1 << 3)	/* Interrupt select */

#define NS921X_TCR_TMODE(x)		((x) & (0x3 << 4))	/* Timer mode */
#define NS921X_TCR_TMODE_MASK		(0x3 << 4)
#define NS921X_TCR_TMODE_INT            (0 << 4)
#define NS921X_TCR_TMODE_EXTLOW         (1 << 4)
#define NS921X_TCR_TMODE_EXTHIGH        (2 << 4)
/* @TODO: Complete the remaining macros */
//#define NS921X_TCR_TMODE		(2 << 4)	/* Timer mode */

#define NS921X_TCR_TCLKSEL		(0xf << 6)	/* Timer clock set */
#define NS921X_TCR_TCLKSEL_MASK		(0xf << 6)
#define NS921X_TCR_TCLKSEL_AHB		(0x1 << 6)
#define NS921X_TCR_INTCLR		(1 << 10)	/* Interrupt clear */
#define NS921X_TCR_DEBUGMODE		(1 << 11)	/* Debug mode */
#define NS921X_TCR_CAPCOMP		(7 << 12)	/* Capture and compare mode functions */
#define NS921X_TCR_TE			(1 << 15)	/* Timer enable */
#define NS921X_TCR_TMODE2		(3 << 16)	/* Timer mode 2 */
#define NS921X_TCR_TMODE2_PWM		(1 << 16)	/* Timer mode 2 */

#define NS921X_TCR_RELOADMODE2		(1 << 18)	/* Reload mode */

#define NS921X_THIGHREG			(32 << 0)	/* PWM output toggles high when timer counter reaches this value */
#define NS921X_TLOWREG			(32 << 0)	/* PWM toggles low when counter reaches this value */

#define NS921X_HISTEP_DIR		(1 << 31)	/* High step direction */
#define NS921X_HISTEP			(15 << 16)	/* High step */
#define NS921X_LOWSTEP_DIR		(1 << 15)	/* Low step direction */
#define NS921X_LOWSTEP			(15 << 0)	/* Low step */

#define NS921X_RELOADSTEPDIR		(1 << 15)	/* Reload step direction */
#define NS921X_RELOADSTEP		(15 << 0)	/* Reload step */

#define NS921X_TIMECOMP			(16 << 16)	/* Timer compare register or timer reload bits 31:16 count register */
#define NS921X_TIMERELOADBITS		(16 << 0)	/* Timer reload bits 15:00 count register */

#define NS921X_TIMERCAP			(16 << 16)	/* Timer capture register or timer read bits 31:16 register */
#define NS921X_TIMERREAD		(16 << 0)	/* Timer read bits 15:00 register */



#if 0
struct pwm_channel {
	void __iomem	*regs;
	unsigned	index;
	unsigned long	mck;
};

extern int pwm_channel_alloc(int index, struct pwm_channel *ch);
extern int pwm_channel_free(struct pwm_channel *ch);

extern int pwm_clk_alloc(unsigned prescale, unsigned div);
extern void pwm_clk_free(unsigned clk);

extern int __pwm_channel_onoff(struct pwm_channel *ch, int enabled);

#define pwm_channel_enable(ch)	__pwm_channel_onoff((ch), 1)
#define pwm_channel_disable(ch)	__pwm_channel_onoff((ch), 0)

/* periodic interrupts, mostly for CUPD changes to period or cycle */
extern int pwm_channel_handler(struct pwm_channel *ch,
		void (*handler)(struct pwm_channel *ch));

static inline void
pwm_channel_writel(struct pwm_channel *pwmc, unsigned offset, u32 val)
{
	__raw_writel(val, pwmc->regs + offset);
}

static inline u32 pwm_channel_readl(struct pwm_channel *pwmc, unsigned offset)
{
	return __raw_readl(pwmc->regs + offset);
}

#endif /* __LINUX_NS9XXX_PWM_H */

#endif /* __NS9XXX_PWM_HEADER_H */

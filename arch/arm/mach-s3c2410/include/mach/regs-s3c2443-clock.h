/* arch/arm/mach-s3c2410/include/mach/regs-s3c2443-clock.h
 *
 * Copyright (c) 2007 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * S3C2443 clock register definitions
*/

#ifndef __ASM_ARM_REGS_S3C2443_CLOCK
#define __ASM_ARM_REGS_S3C2443_CLOCK

#define S3C2443_CLKREG(x)		((x) + S3C24XX_VA_CLKPWR)

#define S3C2443_PLLCON_MDIVSHIFT	16
#define S3C2443_PLLCON_PDIVSHIFT	8
#define S3C2443_PLLCON_SDIVSHIFT	0
#define S3C2443_PLLCON_MDIVMASK		((1<<(1+(23-16)))-1)
#define S3C2443_PLLCON_PDIVMASK		((1<<(1+(9-8)))-1)
#define S3C2443_PLLCON_SDIVMASK		(3)

#define S3C2443_LOCKTIME0               S3C2443_CLKREG(0x00)
#define S3C2443_LOCKTIME1               S3C2443_CLKREG(0x04)
#define S3C2443_LOCKCON0                S3C2443_CLKREG(0x00)
#define S3C2443_LOCKCON1                S3C2443_CLKREG(0x04)
#define S3C2443_OSCSET                  S3C2443_CLKREG(0x08)
#define S3C2443_MPLLCON			S3C2443_CLKREG(0x10)
#define S3C2443_EPLLCON			S3C2443_CLKREG(0x18)
#define S3C2443_CLKSRC			S3C2443_CLKREG(0x20)
#define S3C2443_CLKDIV0			S3C2443_CLKREG(0x24)
#define S3C2443_CLKDIV1			S3C2443_CLKREG(0x28)
#define S3C2443_HCLKCON			S3C2443_CLKREG(0x30)
#define S3C2443_PCLKCON			S3C2443_CLKREG(0x34)
#define S3C2443_SCLKCON			S3C2443_CLKREG(0x38)
#define S3C2443_PWRMODE			S3C2443_CLKREG(0x40)
#define S3C2443_SWRST			S3C2443_CLKREG(0x44)
#define S3C2443_BUSPRI0			S3C2443_CLKREG(0x50)
#define S3C2443_SYSID			S3C2443_CLKREG(0x5C)
#define S3C2443_PWRCFG			S3C2443_CLKREG(0x60)
#define S3C2443_RSTCON			S3C2443_CLKREG(0x64)
#define S3C2443_RSTSTAT 		S3C2443_CLKREG(0x68)
#define S3C2443_PHYCTRL                 S3C2443_CLKREG(0x80)
#define S3C2443_PHYPWR                  S3C2443_CLKREG(0x84)
#define S3C2443_URSTCON                 S3C2443_CLKREG(0x88)
#define S3C2443_UCLKCON                 S3C2443_CLKREG(0x8C)

/* Control bits of the power management configuration register */
#define S3C2443_PWRCFG_USBPHY_ON        (1 << 4)

/* Control bits for the USB reset control register */
#define S3C2443_URSTCON_PHY             (1 << 0)
#define S3C2443_URSTCON_HOST            (1 << 1)
#define S3C2443_URSTCON_FUNC            (1 << 2)

/* Control bits for the USB clock configuration register */
#define S3C2443_UCLKCON_VBUS_PULLUP     (1 << 31)
#define S3C2443_UCLKCON_THOST_ENABLE    (0 << 4)
#define S3C2443_UCLKCON_THOST_DISABLE   (1 << 4)
#define S3C2443_UCLKCON_FUNC_ENABLE     (1 << 2)
#define S3C2443_UCLKCON_FUNC_DISABLE    (0 << 2)
#define S3C2443_UCLKCON_HOST_ENABLE     (1 << 1)
#define S3C2443_UCLKCON_HOST_DISABLE    (0 << 1)
#define S3C2443_UCLKCON_TFUNC_ENABLE    (1 << 0)
#define S3C2443_UCLKCON_TFUNC_DISABLE   (0 << 0)

/* Control bits for the USB phy control */
#define S3C2443_PHYCTRL_CLKSEL(x)       (((x) & 0x3) << 3)
#define S3C2443_PHYCTRL_CLKSEL_24MHZ    (0x3 << 3)
#define S3C2443_PHYCTRL_CLKSEL_12MHZ    (0x2 << 3)
#define S3C2443_PHYCTRL_CLKSEL_48MHZ    (0x0 << 3)
#define S3C2443_PHYCTRL_EXTCLK_OSCI     (1 << 2)
#define S3C2443_PHYCTRL_EXTCLK_CRYSTAL  (0 << 2)
#define S3C2443_PHYCTRL_INTPLL_SYS      (0 << 1)
#define S3C2443_PHYCTRL_INTPLL_USB      (1 << 1)
#define S3C2443_PHYCTRL_DOWN_DEV        (1 << 0)
#define S3C2443_PHYCTRL_DOWN_HOST       (0 << 0)

/* Control bits for the USB phy power control */
#define S3C2443_PHYPWR_COMMON_ON        (1 << 31)
#define S3C2443_PHYPWR_ANALOG_DOWN      (1 << 4)
#define S3C2443_PHYPWR_ANALOG_UP        (0 << 4)
#define S3C2443_PHYPWR_PLLREF_EXT       (0 << 3)
#define S3C2443_PHYPWR_PLLREF_INT       (1 << 3)
#define S3C2443_PHYPWR_XO_UP            (1 << 2)
#define S3C2443_PHYPWR_XO_DOWN          (0 << 2)
#define S3C2443_PHYPWR_PLL_UP           (0 << 1)
#define S3C2443_PHYPWR_PLL_DOWN         (1 << 1)
#define S3C2443_PHYPWR_SUSP_ENABLE      (1 << 0)
#define S3C2443_PHYPWR_SUSP_DISABLE     (0 << 0)

#define S3C2443_SWRST_RESET		(0x533c2443)

#define S3C2443_PLLCON_OFF		(1<<24)

#define S3C2443_CLKSRC_EPLLREF_XTAL	(2<<7)
#define S3C2443_CLKSRC_EPLLREF_EXTCLK	(3<<7)
#define S3C2443_CLKSRC_EPLLREF_MPLLREF	(0<<7)
#define S3C2443_CLKSRC_EPLLREF_MPLLREF2	(1<<7)
#define S3C2443_CLKSRC_EPLLREF_MASK	(3<<7)

#define S3C2443_CLKSRC_EXTCLK_DIV	(1<<3)

#define S3C2443_CLKDIV0_HALF_HCLK	(1<<3)
#define S3C2443_CLKDIV0_HALF_PCLK	(1<<2)

#define S3C2443_CLKDIV0_HCLKDIV_MASK	(3<<0)

#define S3C2443_CLKDIV0_EXTDIV_MASK	(3<<6)
#define S3C2443_CLKDIV0_EXTDIV_SHIFT	(6)

#define S3C2443_CLKDIV0_PREDIV_MASK	(3<<4)
#define S3C2443_CLKDIV0_PREDIV_SHIFT	(4)

#define S3C2443_CLKDIV0_ARMDIV_MASK	(15<<9)
#define S3C2443_CLKDIV0_ARMDIV_SHIFT	(9)
#define S3C2443_CLKDIV0_ARMDIV_1	(0<<9)
#define S3C2443_CLKDIV0_ARMDIV_2	(8<<9)
#define S3C2443_CLKDIV0_ARMDIV_3	(2<<9)
#define S3C2443_CLKDIV0_ARMDIV_4	(9<<9)
#define S3C2443_CLKDIV0_ARMDIV_6	(10<<9)
#define S3C2443_CLKDIV0_ARMDIV_8	(11<<9)
#define S3C2443_CLKDIV0_ARMDIV_12	(13<<9)
#define S3C2443_CLKDIV0_ARMDIV_16	(15<<9)

/* S3C2443_CLKDIV1 removed, only used in clock.c code */

#define S3C2443_CLKCON_NAND

#define S3C2443_HCLKCON_DMA0		(1<<0)
#define S3C2443_HCLKCON_DMA1		(1<<1)
#define S3C2443_HCLKCON_DMA2		(1<<2)
#define S3C2443_HCLKCON_DMA3		(1<<3)
#define S3C2443_HCLKCON_DMA4		(1<<4)
#define S3C2443_HCLKCON_DMA5		(1<<5)
#define S3C2443_HCLKCON_CAMIF		(1<<8)
#define S3C2443_HCLKCON_LCDC		(1<<9)
#define S3C2443_HCLKCON_USBH		(1<<11)
#define S3C2443_HCLKCON_USBD		(1<<12)
#define S3C2443_HCLKCON_HSMMC		(1<<16)
#define S3C2443_HCLKCON_CFC		(1<<17)
#define S3C2443_HCLKCON_SSMC		(1<<18)
#define S3C2443_HCLKCON_DRAMC		(1<<19)

#define S3C2443_PCLKCON_UART0		(1<<0)
#define S3C2443_PCLKCON_UART1		(1<<1)
#define S3C2443_PCLKCON_UART2		(1<<2)
#define S3C2443_PCLKCON_UART3		(1<<3)
#define S3C2443_PCLKCON_IIC		(1<<4)
#define S3C2443_PCLKCON_SDI		(1<<5)
#define S3C2443_PCLKCON_HSSPI		(1<<6)
#define S3C2443_PCLKCON_ADC		(1<<7)
#define S3C2443_PCLKCON_AC97		(1<<8)
#define S3C2443_PCLKCON_IIS		(1<<9)
#define S3C2443_PCLKCON_PWMT		(1<<10)
#define S3C2443_PCLKCON_WDT		(1<<11)
#define S3C2443_PCLKCON_RTC		(1<<12)
#define S3C2443_PCLKCON_GPIO		(1<<13)
#define S3C2443_PCLKCON_SPI0		(1<<14)
#define S3C2443_PCLKCON_SPI1		(1<<15)

#define S3C2443_SCLKCON_DDRCLK		(1<<16)
#define S3C2443_SCLKCON_SSMCCLK		(1<<15)
#define S3C2443_SCLKCON_HSSPICLK	(1<<14)
#define S3C2443_SCLKCON_HSMMCCLK_EXT	(1<<13)
#define S3C2443_SCLKCON_HSMMCCLK_EPLL	(1<<12)
#define S3C2443_SCLKCON_CAMCLK		(1<<11)
#define S3C2443_SCLKCON_DISPCLK		(1<<10)
#define S3C2443_SCLKCON_I2SCLK		(1<<9)
#define S3C2443_SCLKCON_UARTCLK		(1<<8)
#define S3C2443_SCLKCON_USBHOST		(1<<1)

#include <asm/div64.h>

static inline unsigned int
s3c2443_get_mpll(unsigned int pllval, unsigned int baseclk)
{
	unsigned int mdiv, pdiv, sdiv;
	uint64_t fvco;

	mdiv = pllval >> S3C2443_PLLCON_MDIVSHIFT;
	pdiv = pllval >> S3C2443_PLLCON_PDIVSHIFT;
	sdiv = pllval >> S3C2443_PLLCON_SDIVSHIFT;

	mdiv &= S3C2443_PLLCON_MDIVMASK;
	pdiv &= S3C2443_PLLCON_PDIVMASK;
	sdiv &= S3C2443_PLLCON_SDIVMASK;

	fvco = (uint64_t)baseclk * (2 * (mdiv + 8));
	do_div(fvco, pdiv << sdiv);

	return (unsigned int)fvco;
}

static inline unsigned int
s3c2443_get_epll(unsigned int pllval, unsigned int baseclk)
{
	unsigned int mdiv, pdiv, sdiv;
	uint64_t fvco;

	mdiv = pllval >> S3C2443_PLLCON_MDIVSHIFT;
	pdiv = pllval >> S3C2443_PLLCON_PDIVSHIFT;
	sdiv = pllval >> S3C2443_PLLCON_SDIVSHIFT;

	mdiv &= S3C2443_PLLCON_MDIVMASK;
	pdiv &= S3C2443_PLLCON_PDIVMASK;
	sdiv &= S3C2443_PLLCON_SDIVMASK;

	fvco = (uint64_t)baseclk * (mdiv + 8);
	do_div(fvco, (pdiv + 2) << sdiv);

	return (unsigned int)fvco;
}

#endif /*  __ASM_ARM_REGS_S3C2443_CLOCK */


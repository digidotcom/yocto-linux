/* arch/arm/mach-s3c2410/include/mach/regs-spi.h
 *
 * Copyright (c) 2004 Fetron GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * S3C2410 SPI register definition
*/

#ifndef __ASM_ARCH_REGS_SPI_H
#define __ASM_ARCH_REGS_SPI_H

#define S3C2410_SPI1	(0x20)
#define S3C2412_SPI1	(0x100)

#define S3C2410_SPCON	(0x00)

#define S3C2412_SPCON_RXFIFO_RB2	(0<<14)
#define S3C2412_SPCON_RXFIFO_RB4	(1<<14)
#define S3C2412_SPCON_RXFIFO_RB12	(2<<14)
#define S3C2412_SPCON_RXFIFO_RB14	(3<<14)
#define S3C2412_SPCON_TXFIFO_RB2	(0<<12)
#define S3C2412_SPCON_TXFIFO_RB4	(1<<12)
#define S3C2412_SPCON_TXFIFO_RB12	(2<<12)
#define S3C2412_SPCON_TXFIFO_RB14	(3<<12)
#define S3C2412_SPCON_RXFIFO_RESET	(1<<11) /* RxFIFO reset */
#define S3C2412_SPCON_TXFIFO_RESET	(1<<10) /* TxFIFO reset */
#define S3C2412_SPCON_RXFIFO_EN		(1<<9)  /* RxFIFO Enable */
#define S3C2412_SPCON_TXFIFO_EN		(1<<8)  /* TxFIFO Enable */

#define S3C2412_SPCON_DIRC_RX	  (1<<7)

#define S3C2410_SPCON_SMOD_DMA	  (2<<5)	/* DMA mode */
#define S3C2410_SPCON_SMOD_INT	  (1<<5)	/* interrupt mode */
#define S3C2410_SPCON_SMOD_POLL   (0<<5)	/* polling mode */
#define S3C2410_SPCON_ENSCK	  (1<<4)	/* Enable SCK */
#define S3C2410_SPCON_MSTR	  (1<<3)	/* Master/Slave select
						   0: slave, 1: master */
#define S3C2410_SPCON_CPOL_HIGH	  (1<<2)	/* Clock polarity select */
#define S3C2410_SPCON_CPOL_LOW	  (0<<2)	/* Clock polarity select */

#define S3C2410_SPCON_CPHA_FMTB	  (1<<1)	/* Clock Phase Select */
#define S3C2410_SPCON_CPHA_FMTA	  (0<<1)	/* Clock Phase Select */

#define S3C2410_SPCON_TAGD	  (1<<0)	/* Tx auto garbage data mode */


#define S3C2410_SPSTA	 (0x04)

#define S3C2412_SPSTA_RXFIFO_AE		(1<<11)
#define S3C2412_SPSTA_TXFIFO_AE		(1<<10)
#define S3C2412_SPSTA_RXFIFO_ERROR	(1<<9)
#define S3C2412_SPSTA_TXFIFO_ERROR	(1<<8)
#define S3C2412_SPSTA_RXFIFO_FIFO	(1<<7)
#define S3C2412_SPSTA_RXFIFO_EMPTY	(1<<6)
#define S3C2412_SPSTA_TXFIFO_NFULL	(1<<5)
#define S3C2412_SPSTA_TXFIFO_EMPTY	(1<<4)

#define S3C2410_SPSTA_DCOL	  (1<<2)	/* Data Collision Error */
#define S3C2410_SPSTA_MULD	  (1<<1)	/* Multi Master Error */
#define S3C2410_SPSTA_READY	  (1<<0)	/* Data Tx/Rx ready */
#define S3C2412_SPSTA_READY_ORG	  (1<<3)

#define S3C2410_SPPIN	 (0x08)

#define S3C2410_SPPIN_ENMUL	  (1<<2)	/* Multi Master Error detect */
#define S3C2410_SPPIN_RESERVED	  (1<<1)
#define S3C2400_SPPIN_nCS     	  (1<<1)	/* SPI Card Select */
#define S3C2410_SPPIN_KEEP	  (1<<0)	/* Master Out keep */

#define S3C2410_SPPRE	 (0x0C)
#define S3C2410_SPTDAT	 (0x10)
#define S3C2410_SPRDAT	 (0x14)

#define S3C2412_TXFIFO	 (0x18)
#define S3C2412_RXFIFO	 (0x18)
#define S3C2412_SPFIC	 (0x24)

/* 
 * High Speed SPI registers and control bits
 * Coming from the SMDK (Luis Galdos)
 */
#define S3C2443_SPI0_CH_CFG                     (0x00)      /* Configuration */
#define S3C2443_SPI0_CLK_CFG                    (0x04)      /* Clock configuration */
#define S3C2443_SPI0_MODE_CFG                   (0x08)      /* FIFO control */
#define S3C2443_SPI0_SLAVE_SEL                  (0x0C)      /* Slave selection */
#define S3C2443_SPI0_INT_EN                     (0x10)      /* interrupt enable */
#define S3C2443_SPI0_STATUS                     (0x14)      /* status */
#define S3C2443_SPI0_TX_DATA                    (0x18)      /* TX data */
#define S3C2443_SPI0_RX_DATA                    (0x1C)      /* RX data */
#define S3C2443_SPI0_PACKET_CNT                 (0x20)      /* Count how many data master gets */
#define S3C2443_SPI0_PENDING_CLR                (0x24)      /* Pending clear */

#define S3C2443_SPI0_TX_DATA_PA                 (0x52000018)
#define S3C2443_SPI0_RX_DATA_PA                 (0x5200001C)

#define S3C2443_SPI0_CH_SW_RST                  (1<<5)
#define S3C2443_SPI0_CH_MASTER                  (0<<4)
#define S3C2443_SPI0_CH_SLAVE                   (1<<4)
#define S3C2443_SPI0_CH_RISING                  (0<<3)
#define S3C2443_SPI0_CH_FALLING                 (1<<3)
#define S3C2443_SPI0_CH_FORMAT_A                (0<<2)
#define S3C2443_SPI0_CH_FORMAT_B                (1<<2)
#define S3C2443_SPI0_CH_RXCH_OFF                (0<<1)
#define S3C2443_SPI0_CH_RXCH_ON                 (1<<1)
#define S3C2443_SPI0_CH_TXCH_OFF                (0<<0)
#define S3C2443_SPI0_CH_TXCH_ON                 (1<<0)

#define S3C2443_SPI0_CLKSEL_PCLK                (0<<9)
#define S3C2443_SPI0_CLKSEL_HCLK                (1<<9)
#define S3C2443_SPI0_CLKSEL_ECLK                (2<<9)
#define S3C2443_SPI0_CLKSEL_EPLL                (3<<9)
#define S3C2443_SPI0_ENCLK_DISABLE              (0<<8)
#define S3C2443_SPI0_ENCLK_ENABLE               (1<<8)
#define S3C2443_SPI0_CLK_PRE_MASK               (0xff)

#define S3C2443_SPI0_MODE_CH_TSZ_BYTE           (0<<18)
#define S3C2443_SPI0_MODE_CH_TSZ_WORD           (1<<18)
#define S3C2443_SPI0_FEED_BACK_DELAY            (1<<17)
#define S3C2443_SPI0_MODE_RXDMA_OFF             (0<<2)
#define S3C2443_SPI0_MODE_RXDMA_ON              (1<<2)
#define S3C2443_SPI0_MODE_TXDMA_OFF             (0<<1)
#define S3C2443_SPI0_MODE_TXDMA_ON              (1<<1)
#define S3C2443_SPI0_MODE_SINGLE                (0<<0)
#define S3C2443_SPI0_MODE_4BURST                (1<<0)

#define S3C2443_SPI0_STUS_TX_DONE               (1<<21)
#define S3C2443_SPI0_STUS_TRAILCNT_ZERO         (1<<20)

#define S3C2443_SPI0_STUS_TX_LEVEL(x)           ((x >> 6) & 0x7F)
#define S3C2443_SPI0_STUS_RX_LEVEL(x)           ((x >> 13) & 0x7F)

#define S3C2443_SPI0_STUS_RX_OVERRUN_ERR        (1<<5)
#define S3C2443_SPI0_STUS_RX_UNDERRUN_ERR       (1<<4)
#define S3C2443_SPI0_STUS_TX_OVERRUN_ERR        (1<<3)
#define S3C2443_SPI0_STUS_TX_UNDERRUN_ERR       (1<<2)
#define S3C2443_SPI0_STUS_RX_FIFORDY            (1<<1)
#define S3C2443_SPI0_STUS_TX_FIFORDY            (1<<0)

#define S3C2443_SPI0_SLAVE_SIG_ACT              (0<<0)
#define S3C2443_SPI0_SLAVE_SIG_INACT            (1<<0)

#define S3C2443_SPI0_STUS_TX_DONE               (1<<21)
#define S3C2443_SPI0_STUS_TRAILCNT_ZERO         (1<<20)
#define S3C2443_SPI0_STUS_RX_OVERRUN            (1<<5)
#define S3C2443_SPI0_STUS_RX_UNDERRUN           (1<<4)
#define S3C2443_SPI0_STUS_TX_OVERRUN            (1<<3)
#define S3C2443_SPI0_STUS_TX_UNDERRUN           (1<<2)
#define S3C2443_SPI0_STUS_RX_FIFORDY            (1<<1)
#define S3C2443_SPI0_STUS_TX_FIFORDY            (1<<0)

#define S3C2443_SPI0_INT_TRAILING_DIS           (0<<6)
#define S3C2443_SPI0_INT_TRAILING_EN            (1<<6)
#define S3C2443_SPI0_INT_RX_OVERRUN_DIS         (0<<5)
#define S3C2443_SPI0_INT_RX_OVERRUN_EN          (1<<5)
#define S3C2443_SPI0_INT_RX_UNDERRUN_DIS        (0<<4)
#define S3C2443_SPI0_INT_RX_UNDERRUN_EN         (1<<4)
#define S3C2443_SPI0_INT_TX_OVERRUN_DIS         (0<<3)
#define S3C2443_SPI0_INT_TX_OVERRUN_EN          (1<<3)
#define S3C2443_SPI0_INT_TX_UNDERRUN_DIS        (0<<2)
#define S3C2443_SPI0_INT_TX_UNDERRUN_EN         (1<<2)
#define S3C2443_SPI0_INT_RX_FIFORDY_DIS         (0<<1)
#define S3C2443_SPI0_INT_RX_FIFORDY_EN          (1<<1)
#define S3C2443_SPI0_INT_TX_FIFORDY_DIS         (0<<0)
#define S3C2443_SPI0_INT_TX_FIFORDY_EN          (1<<0)

#define S3C2443_SPI0_PACKET_CNT_DIS             (0<<16)
#define S3C2443_SPI0_PACKET_CNT_EN              (1<<16)

#define S3C2443_SPI0_PND_TX_UNDERRUN_CLR        (1<<4)
#define S3C2443_SPI0_PND_TX_OVERRUN_CLR         (1<<3)
#define S3C2443_SPI0_PND_RX_UNDERRUN_CLR        (1<<2)
#define S3C2443_SPI0_PND_RX_OVERRUN_CLR         (1<<1)
#define S3C2443_SPI0_PND_TRAILING_CLR           (1<<0)

#endif /* __ASM_ARCH_REGS_SPI_H */

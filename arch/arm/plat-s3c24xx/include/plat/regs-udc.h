/* linux/include/asm-arm/arch-s3c2410/regs-udc.h
 *
 * Copyright (C) 2004 Herbert Poetzl <herbert@13thfloor.at>
 *
 * This include file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
*/

#ifndef __ASM_ARCH_REGS_UDC_H
#define __ASM_ARCH_REGS_UDC_H

#define S3C2410_USBDREG(x) (x)

#define S3C2410_UDC_FUNC_ADDR_REG	S3C2410_USBDREG(0x0140)
#define S3C2410_UDC_PWR_REG		S3C2410_USBDREG(0x0144)
#define S3C2410_UDC_EP_INT_REG		S3C2410_USBDREG(0x0148)

#define S3C2410_UDC_USB_INT_REG		S3C2410_USBDREG(0x0158)
#define S3C2410_UDC_EP_INT_EN_REG	S3C2410_USBDREG(0x015c)

#define S3C2410_UDC_USB_INT_EN_REG	S3C2410_USBDREG(0x016c)

#define S3C2410_UDC_FRAME_NUM1_REG	S3C2410_USBDREG(0x0170)
#define S3C2410_UDC_FRAME_NUM2_REG	S3C2410_USBDREG(0x0174)

#define S3C2410_UDC_EP0_FIFO_REG	S3C2410_USBDREG(0x01c0)
#define S3C2410_UDC_EP1_FIFO_REG	S3C2410_USBDREG(0x01c4)
#define S3C2410_UDC_EP2_FIFO_REG	S3C2410_USBDREG(0x01c8)
#define S3C2410_UDC_EP3_FIFO_REG	S3C2410_USBDREG(0x01cc)
#define S3C2410_UDC_EP4_FIFO_REG	S3C2410_USBDREG(0x01d0)

#define S3C2410_UDC_EP1_DMA_CON		S3C2410_USBDREG(0x0200)
#define S3C2410_UDC_EP1_DMA_UNIT	S3C2410_USBDREG(0x0204)
#define S3C2410_UDC_EP1_DMA_FIFO	S3C2410_USBDREG(0x0208)
#define S3C2410_UDC_EP1_DMA_TTC_L	S3C2410_USBDREG(0x020c)
#define S3C2410_UDC_EP1_DMA_TTC_M	S3C2410_USBDREG(0x0210)
#define S3C2410_UDC_EP1_DMA_TTC_H	S3C2410_USBDREG(0x0214)

#define S3C2410_UDC_EP2_DMA_CON		S3C2410_USBDREG(0x0218)
#define S3C2410_UDC_EP2_DMA_UNIT	S3C2410_USBDREG(0x021c)
#define S3C2410_UDC_EP2_DMA_FIFO	S3C2410_USBDREG(0x0220)
#define S3C2410_UDC_EP2_DMA_TTC_L	S3C2410_USBDREG(0x0224)
#define S3C2410_UDC_EP2_DMA_TTC_M	S3C2410_USBDREG(0x0228)
#define S3C2410_UDC_EP2_DMA_TTC_H	S3C2410_USBDREG(0x022c)

#define S3C2410_UDC_EP3_DMA_CON		S3C2410_USBDREG(0x0240)
#define S3C2410_UDC_EP3_DMA_UNIT	S3C2410_USBDREG(0x0244)
#define S3C2410_UDC_EP3_DMA_FIFO	S3C2410_USBDREG(0x0248)
#define S3C2410_UDC_EP3_DMA_TTC_L	S3C2410_USBDREG(0x024c)
#define S3C2410_UDC_EP3_DMA_TTC_M	S3C2410_USBDREG(0x0250)
#define S3C2410_UDC_EP3_DMA_TTC_H	S3C2410_USBDREG(0x0254)

#define S3C2410_UDC_EP4_DMA_CON		S3C2410_USBDREG(0x0258)
#define S3C2410_UDC_EP4_DMA_UNIT	S3C2410_USBDREG(0x025c)
#define S3C2410_UDC_EP4_DMA_FIFO	S3C2410_USBDREG(0x0260)
#define S3C2410_UDC_EP4_DMA_TTC_L	S3C2410_USBDREG(0x0264)
#define S3C2410_UDC_EP4_DMA_TTC_M	S3C2410_USBDREG(0x0268)
#define S3C2410_UDC_EP4_DMA_TTC_H	S3C2410_USBDREG(0x026c)

#define S3C2410_UDC_INDEX_REG		S3C2410_USBDREG(0x0178)

/* indexed registers */

#define S3C2410_UDC_MAXP_REG		S3C2410_USBDREG(0x0180)

#define S3C2410_UDC_EP0_CSR_REG		S3C2410_USBDREG(0x0184)

#define S3C2410_UDC_IN_CSR1_REG		S3C2410_USBDREG(0x0184)
#define S3C2410_UDC_IN_CSR2_REG		S3C2410_USBDREG(0x0188)

#define S3C2410_UDC_OUT_CSR1_REG	S3C2410_USBDREG(0x0190)
#define S3C2410_UDC_OUT_CSR2_REG	S3C2410_USBDREG(0x0194)
#define S3C2410_UDC_OUT_FIFO_CNT1_REG	S3C2410_USBDREG(0x0198)
#define S3C2410_UDC_OUT_FIFO_CNT2_REG	S3C2410_USBDREG(0x019c)

#define S3C2410_UDC_FUNCADDR_UPDATE	(1<<7)

#define S3C2410_UDC_PWR_ISOUP		(1<<7) // R/W
#define S3C2410_UDC_PWR_RESET		(1<<3) // R
#define S3C2410_UDC_PWR_RESUME		(1<<2) // R/W
#define S3C2410_UDC_PWR_SUSPEND		(1<<1) // R
#define S3C2410_UDC_PWR_ENSUSPEND	(1<<0) // R/W

#define S3C2410_UDC_PWR_DEFAULT		0x00

#define S3C2410_UDC_INT_EP4		(1<<4) // R/W (clear only)
#define S3C2410_UDC_INT_EP3		(1<<3) // R/W (clear only)
#define S3C2410_UDC_INT_EP2		(1<<2) // R/W (clear only)
#define S3C2410_UDC_INT_EP1		(1<<1) // R/W (clear only)
#define S3C2410_UDC_INT_EP0		(1<<0) // R/W (clear only)

#define S3C2410_UDC_USBINT_RESET	(1<<2) // R/W (clear only)
#define S3C2410_UDC_USBINT_RESUME	(1<<1) // R/W (clear only)
#define S3C2410_UDC_USBINT_SUSPEND	(1<<0) // R/W (clear only)

#define S3C2410_UDC_INTE_EP4		(1<<4) // R/W
#define S3C2410_UDC_INTE_EP3		(1<<3) // R/W
#define S3C2410_UDC_INTE_EP2		(1<<2) // R/W
#define S3C2410_UDC_INTE_EP1		(1<<1) // R/W
#define S3C2410_UDC_INTE_EP0		(1<<0) // R/W

#define S3C2410_UDC_USBINTE_RESET	(1<<2) // R/W
#define S3C2410_UDC_USBINTE_SUSPEND	(1<<0) // R/W


#define S3C2410_UDC_INDEX_EP0		(0x00)
#define S3C2410_UDC_INDEX_EP1		(0x01) // ??
#define S3C2410_UDC_INDEX_EP2		(0x02) // ??
#define S3C2410_UDC_INDEX_EP3		(0x03) // ??
#define S3C2410_UDC_INDEX_EP4		(0x04) // ??

#define S3C2410_UDC_ICSR1_CLRDT		(1<<6) // R/W
#define S3C2410_UDC_ICSR1_SENTSTL	(1<<5) // R/W (clear only)
#define S3C2410_UDC_ICSR1_SENDSTL	(1<<4) // R/W
#define S3C2410_UDC_ICSR1_FFLUSH	(1<<3) // W   (set only)
#define S3C2410_UDC_ICSR1_UNDRUN	(1<<2) // R/W (clear only)
#define S3C2410_UDC_ICSR1_PKTRDY	(1<<0) // R/W (set only)

#define S3C2410_UDC_ICSR2_AUTOSET	(1<<7) // R/W
#define S3C2410_UDC_ICSR2_ISO		(1<<6) // R/W
#define S3C2410_UDC_ICSR2_MODEIN	(1<<5) // R/W
#define S3C2410_UDC_ICSR2_DMAIEN	(1<<4) // R/W

#define S3C2410_UDC_OCSR1_CLRDT		(1<<7) // R/W
#define S3C2410_UDC_OCSR1_SENTSTL	(1<<6) // R/W (clear only)
#define S3C2410_UDC_OCSR1_SENDSTL	(1<<5) // R/W
#define S3C2410_UDC_OCSR1_FFLUSH	(1<<4) // R/W
#define S3C2410_UDC_OCSR1_DERROR	(1<<3) // R
#define S3C2410_UDC_OCSR1_OVRRUN	(1<<2) // R/W (clear only)
#define S3C2410_UDC_OCSR1_PKTRDY	(1<<0) // R/W (clear only)

#define S3C2410_UDC_OCSR2_AUTOCLR	(1<<7) // R/W
#define S3C2410_UDC_OCSR2_ISO		(1<<6) // R/W
#define S3C2410_UDC_OCSR2_DMAIEN	(1<<5) // R/W

#define S3C2410_UDC_EP0_CSR_OPKRDY	(1<<0)
#define S3C2410_UDC_EP0_CSR_IPKRDY	(1<<1)
#define S3C2410_UDC_EP0_CSR_SENTSTL	(1<<2)
#define S3C2410_UDC_EP0_CSR_DE		(1<<3)
#define S3C2410_UDC_EP0_CSR_SE		(1<<4)
#define S3C2410_UDC_EP0_CSR_SENDSTL	(1<<5)
#define S3C2410_UDC_EP0_CSR_SOPKTRDY	(1<<6)
#define S3C2410_UDC_EP0_CSR_SSE	(1<<7)

#define S3C2410_UDC_MAXP_8		(1<<0)
#define S3C2410_UDC_MAXP_16		(1<<1)
#define S3C2410_UDC_MAXP_32		(1<<2)
#define S3C2410_UDC_MAXP_64		(1<<3)



#define S3C24XX_USBDREG(x)              (x)

/* Registers for the S3C2443 */
/* Non-Indexed Registers */
#if defined(CONFIG_CPU_S3C2443)

#define S3C24XX_UDC_IR_REG                  S3C24XX_USBDREG(0x00) /* Index register */
#define S3C24XX_UDC_EIR_REG                 S3C24XX_USBDREG(0x04) /* EP IRQ pending */
#define S3C24XX_UDC_EIER_REG                S3C24XX_USBDREG(0x08) /* EP IRQ enable */
#define S3C24XX_UDC_FAR_REG                 S3C24XX_USBDREG(0x0c) /* Function address */
#define S3C24XX_UDC_FNR_REG                 S3C24XX_USBDREG(0x10) /* Frame number */
#define S3C24XX_UDC_EDR_REG                 S3C24XX_USBDREG(0x14) /* Endpoint direction */
#define S3C24XX_UDC_TR_REG                  S3C24XX_USBDREG(0x18) /* Test register */
#define S3C24XX_UDC_SSR_REG                 S3C24XX_USBDREG(0x1c) /* System status */
#define S3C24XX_UDC_SCR_REG                 S3C24XX_USBDREG(0x20) /* System control */
#define S3C24XX_UDC_EP0SR_REG               S3C24XX_USBDREG(0x24) /* Endpoint 0 status */
#define S3C24XX_UDC_EP0CR_REG               S3C24XX_USBDREG(0x28) /* Endpoint 0 control */
#define S3C24XX_UDC_EP0BR_REG               S3C24XX_USBDREG(0x60) /* Endpoint 0 Buffer */
#define S3C24XX_UDC_EP1BR_REG               S3C24XX_USBDREG(0x64) /* Endpoint 1 Buffer */
#define S3C24XX_UDC_EP2BR_REG               S3C24XX_USBDREG(0x68) /* Endpoint 2 Buffer */
#define S3C24XX_UDC_EP3BR_REG               S3C24XX_USBDREG(0x6C) /* Endpoint 3 Buffer */
#define S3C24XX_UDC_EP4BR_REG               S3C24XX_USBDREG(0x70) /* Endpoint 4 Buffer */
#define S3C24XX_UDC_EP5BR_REG               S3C24XX_USBDREG(0x74) /* Endpoint 5 Buffer */
#define S3C24XX_UDC_EP6BR_REG               S3C24XX_USBDREG(0x78) /* Endpoint 6 Buffer */
#define S3C24XX_UDC_EP7BR_REG               S3C24XX_USBDREG(0x7C) /* Endpoint 7 Buffer */
#define S3C24XX_UDC_EP8BR_REG               S3C24XX_USBDREG(0x80) /* Endpoint 8 Buffer */

#define S3C24XX_UDC_FIFO_CON_REG            S3C24XX_USBDREG(0x100) /* Burst FIFO-DMA Control */
#define S3C24XX_UDC_FIFO_STATUS_REG         S3C24XX_USBDREG(0x104) /* Burst FIFO Status */

/* Indexed Registers */
#define S3C24XX_UDC_EP_STATUS_REG           S3C24XX_USBDREG(0x2c) /* Endpoints status */
#define S3C24XX_UDC_EP_CON_REG              S3C24XX_USBDREG(0x30) /* Endpoints control */
#define S3C24XX_UDC_BYTE_READ_CNT_REG       S3C24XX_USBDREG(0x34) /* Byte read count */
#define S3C24XX_UDC_BYTE_WRITE_CNT_REG      S3C24XX_USBDREG(0x38) /* Byte write count */
#define S3C24XX_UDC_MAXP_REG                S3C24XX_USBDREG(0x3c) /* Max packet size */
#define S3C24XX_UDC_DMA_CON_REG             S3C24XX_USBDREG(0x40) /* DMA control */
#define S3C24XX_UDC_DMA_CNT_REG             S3C24XX_USBDREG(0x44) /* DMA count */
#define S3C24XX_UDC_DMA_FIFO_CNT_REG        S3C24XX_USBDREG(0x48) /* DMA FIFO count */
#define S3C24XX_UDC_DMA_TOTAL_CNT1_REG      S3C24XX_USBDREG(0x4c) /* DMA Total Transfer count1 */
#define S3C24XX_UDC_DMA_TOTAL_CNT2_REG      S3C24XX_USBDREG(0x50) /* DMA Total Transfer count2 */
#define S3C24XX_UDC_DMA_IF_CON_REG          S3C24XX_USBDREG(0x84) /* DMA interface Control */
#define S3C24XX_UDC_DMA_MEM_BASE_ADDR_REG   S3C24XX_USBDREG(0x88) /* Mem Base Addr */
#define S3C24XX_UDC_DMA_MEM_CURRENT_ADDR_REG        S3C24XX_USBDREG(0x8c) /* Mem current Addr */

/* @TODO: Indexed registers according to the data sheet */
#define S3C24XX_UDC_ESR_REG                 S3C24XX_USBDREG(0x2C) /* EP status */
#define S3C24XX_UDC_ECR_REG                 S3C24XX_USBDREG(0x30) /* EP control */
#define S3C24XX_UDC_BRCR_REG                S3C24XX_USBDREG(0x34) /* Byte read count */
#define S3C24XX_UDC_BWCR_REG                S3C24XX_USBDREG(0x38) /* Byte write count */

#else
#error "Unsupported platform for the UDC-S3C24XX?"
#endif


/* EP interrupt register Bits */
#define S3C24XX_UDC_INT_EP3         (1<<3) // R/C
#define S3C24XX_UDC_INT_EP2         (1<<2) // R/C
#define S3C24XX_UDC_INT_EP1         (1<<1) // R/C
#define S3C24XX_UDC_INT_EP0         (1<<0) // R/C

/* System status register Bits */
#define S3C24XX_UDC_INT_CHECK       (0xff8f)
#define S3C24XX_UDC_INT_ERR         (0xff80) // R/C
#define S3C24XX_UDC_SSR_DCERR       (1 << 11) /* Data CRC error */
#define S3C24XX_UDC_SSR_EOERR       (1 << 10) /* EB overrun error */
#define S3C24XX_UDC_SSR_VBUSOFF     (1 << 9) /* Vbus OFF */
#define S3C24XX_UDC_INT_VBUSON      (1 << 8) /* Vbus ON */ 
#define S3C24XX_UDC_SSR_TBM         (1 << 7) /* Toggle bit mismatch */
#define S3C24XX_UDC_INT_HSP         (1 << 4) // R
#define S3C24XX_UDC_INT_SDE         (1 << 3) // R/C
#define S3C24XX_UDC_INT_RESUME      (1 << 2) // R/C
#define S3C24XX_UDC_INT_SUSPEND     (1 << 1) // R/C
#define S3C24XX_UDC_INT_RESET       (1 << 0) // R/C

/* system control register Bits */
#define S3C24XX_UDC_DTZIEN_EN       (1 << 14)
#define S3C24XX_UDC_DI_EN           (1 << 12)
#define S3C24XX_UDC_VBUSOFF_EN      (1 << 11)
#define S3C24XX_UDC_VBUSON_EN       (1 << 10)
#define S3C24XX_UDC_RWDE_EN         (1 << 9)
#define S3C24XX_UDC_EIE_EN          (1 << 8)
#define S3C24XX_UDC_BIS_EN          (1 << 7)
#define S3C24XX_UDC_SPDEN_EN        (1 << 6)
#define S3C24XX_UDC_RRD_EN          (1 << 5)
#define S3C24XX_UDC_IPS_EN          (1 << 4)
#define S3C24XX_UDC_MFRM_EN         (1 << 2)
#define S3C24XX_UDC_SUS_EN          (1 << 1)
#define S3C24XX_UDC_RST_EN          (1 << 0)

/* EP0 status register Bits */
#define S3C24XX_UDC_EP0_LWO         (1<<6)
#define S3C24XX_UDC_EP0_STALL       (1<<4)
#define S3C24XX_UDC_EP0_TX_SUCCESS  (1<<1)
#define S3C24XX_UDC_EP0_RX_SUCCESS  (1<<0)

/* EP status register Bits */
#define S3C24XX_UDC_EP_FPID         (1<<11)
#define S3C24XX_UDC_EP_OSD          (1<<10)
#define S3C24XX_UDC_EP_DTCZ         (1<<9)
#define S3C24XX_UDC_EP_SPT          (1<<8)

/* EP0 status register bits */
#define S3C24XX_UDC_EP0SR_RSR       (1 << 0)
#define S3C24XX_UDC_EP0SR_TST       (1 << 1)
#define S3C24XX_UDC_EP0SR_SHT       (1 << 4)
#define S3C24XX_UDC_EP0SR_LWO       (1 << 6)

/* EP0 control register bits */
#define S3C24XX_UDC_EP0CR_TTE       (1 << 3)
#define S3C24XX_UDC_EP0CR_TSS       (1 << 2)
#define S3C24XX_UDC_EP0CR_ESS       (1 << 1)
#define S3C24XX_UDC_EP0CR_TZLS      (1 << 0)

/* Indexed EP control register bits */
#define S3C24XX_UDC_ECR_INPKTHLD    (1 << 12)
#define S3C24XX_UDC_ECR_OUTPKTHLD   (1 << 11)
#define S3C24XX_UDC_ECR_TNPMF       (1 << 10)
#define S3C24XX_UDC_ECR_IME         (1 << 9)
#define S3C24XX_UDC_ECR_DUEN        (1 << 7)
#define S3C24XX_UDC_ECR_FLUSH       (1 << 6)
#define S3C24XX_UDC_ECR_TTE         (1 << 5)
#define S3C24XX_UDC_ECR_CDP         (1 << 2)
#define S3C24XX_UDC_ECR_ESS         (1 << 1)
#define S3C24XX_UDC_ECR_TZLS        (1 << 0)

/* Index EP status register bits */
#define S3C24XX_UDC_ESR_RPS         (1 << 0) /* Rx packet success */
#define S3C24XX_UDC_ESR_TPS         (1 << 1) /* Tx packet success */
#define S3C24XX_UDC_ESR_PSIF        (3 << 2) /* Packet status in FIFO */
#define S3C24XX_UDC_ESR_PSIFNR(x)   (((x) >> 2) & (0x3)) /* Number of packets in FIFO */
#define S3C24XX_UDC_ESR_PSIF_ONE    (1 << 2) /* TRUE if one packet in the FIFO*/
#define S3C24XX_UDC_ESR_PSIF_TWO    (2 << 2) /* TRUE if two packets in the FIFO */
#define S3C24XX_UDC_ESR_LWO         (1 << 4) /* Last word odd */
#define S3C24XX_UDC_ESR_FSC         (1 << 5) /* Function stall condition */
#define S3C24XX_UDC_ESR_FFS         (1 << 6) /* FIFO flushed */
#define S3C24XX_UDC_ESR_SPT         (1 << 8) /* Short packet received */
#define S3C24XX_UDC_ESR_DTCZ        (1 << 9) /* DMA total count zero */
#define S3C24XX_UDC_ESR_OSD         (1 << 10) /* OUT start DMA operation */
#define S3C24XX_UDC_ESR_FPID        (1 << 11) /* First OUT packet interrupt disable */
#define S3C24XX_UDC_ESR_FOVF        (1 << 14) /* FIFO overflow */
#define S3C24XX_UDC_ESR_FUDR        (1 << 15) /* FIFO underflow */

/* Test register bits */
#define S3C24XX_UDC_TR_TSNS         (1 << 0) /* Test SE0 NAK select */
#define S3C24XX_UDC_TR_TJS          (1 << 1) /* Test J select */
#define S3C24XX_UDC_TR_TKS          (1 << 2) /* Test K select */
#define S3C24XX_UDC_TR_TPS          (1 << 3) /* Test packets */
#define S3C24XX_UDC_TR_TMD          (1 << 4) /* Test mode */
#define S3C24XX_UDC_TR_PERR         (1 << 12) /* PID error */
#define S3C24XX_UDC_TR_EUERR        (1 << 13) /* EB underrun error */
#define S3C24XX_UDC_TR_VBUS         (1 << 15) /* Vbus */

/* System control register bits */
#define S3C24XX_UDC_SCR_HRESE       (1 << 0) /* Reset enable */
#define S3C24XX_UDC_SCR_HSUSPE      (1 << 1) /* Suspend enable */
#define S3C24XX_UDC_SCR_MFRM        (1 << 2) /* Resume by MCU */
#define S3C24XX_UDC_SCR_IPS         (1 << 4) /* Interrupt polarity */
#define S3C24XX_UDC_SCR_RRDE        (1 << 5) /* Reverse read data enable */
#define S3C24XX_UDC_SCR_SPDEN       (1 << 6) /* Speed detect end interrupt enable */
#define S3C24XX_UDC_SCR_BIS         (1 << 7) /* Bus interface select */
#define S3C24XX_UDC_SCR_EIE         (1 << 8) /* Error interrupt enable */
#define S3C24XX_UDC_SCR_RWDE        (1 << 9) /* Reverse write data enable */
#define S3C24XX_UDC_SCR_VBUSONEN    (1 << 10) /* VBUS on enable */
#define S3C24XX_UDC_SCR_VBUSOFFEN   (1 << 11) /* VBUS off enable */
#define S3C24XX_UDC_SCR_DIEN        (1 << 12) /* DUAL interrupt enable */
#define S3C24XX_UDC_SCR_DTZIEN      (1 << 13) /* DMA total counter zero interrupt enable */

/* @IMPORTANT: Dont use the below control bits */
#define S3C24XX_UDC_EP_DOM          (1<<7)
#define S3C24XX_UDC_EP_FIFO_FLUSH   (1<<6)
#define S3C24XX_UDC_EP_STALL        (1<<5)
#define S3C24XX_UDC_EP_LWO          (1<<4)
#define S3C24XX_UDC_EP_PSIF_ONE     (1<<2)
#define S3C24XX_UDC_EP_PSIF_TWO     (2<<2)
#define S3C24XX_UDC_EP_TX_SUCCESS   (1<<1)
#define S3C24XX_UDC_EP_RX_SUCCESS   (1<<0)

#endif /* __ASM_ARCH_REGS_UDC_H */



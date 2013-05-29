/*
 * arch/arm/mach-ns9xxx/include/mach/regs-sys-ns921x.h
 *
 * Copyright (C) 2007 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 *  !Revision:   $Revision: 1.0 $
 *  !Author:     Luis Galdos
 *  !Desc:
 *  !References:
 */

#ifndef __ASM_ARCH_REGSIOHUB_NS921X_H
#define __ASM_ARCH_REGSIOHUB_NS921X_H

#include <mach/hardware.h>
#include <mach/regs-sys-common.h>

#define IOHUB_REG_BASE_PA                       (0x90000000)
#define IOHUB_REG_BASE_VA                       io_p2v(IOHUB_REG_BASE_PA)
#define IOHUB_REG_OFFSET                        (0x8000)
#define IOHUB_FIM0_BASE_PA                      IOHUB_REG_BASE_PA
#define IOHUB_FIM1_BASE_PA                      (IOHUB_REG_BASE_PA + IOHUB_REG_OFFSET)
#define IOHUB_FIM0_BASE_VA                      io_p2v(IOHUB_FIM0_BASE_PA)
#define IOHUB_FIM1_BASE_VA                      io_p2v(IOHUB_FIM1_BASE_PA)

/* Register offsets for the IOHUB-components */
#define IOHUB_IFS_REG                                   (0x00)

#define IOHUB_RX_DMA_CTRL_REG                           (0x04)
#define IOHUB_RX_DMA_BUFPTR_REG                         (0x08)
#define IOHUB_RX_DMA_ICTRL_REG                          (0x0C)
#define IOHUB_RX_DIR_REG                                (0x10)
#define IOHUB_RX_DIR_FIFO_REG                           (0x14)
#define IOHUB_TX_DMA_CTRL_REG                           (0x18)
#define IOHUB_TX_DMA_BUFPTR_REG                         (0x1C)
#define IOHUB_TX_ICTRL_REG                              (0x20)
#define IOHUB_TX_FIFO_REG                               (0x28)
#define IOHUB_TX_DIR_REG                                (0x2C)

/* DMA RX control bits */
#define IOHUB_RX_DMA_CTRL_CE                            (1<<31)
#define IOHUB_RX_DMA_CTRL_CA                            (1<<30)
#define IOHUB_RX_DMA_CTRL_FLEXIO                        (1<<29)
#define IOHUB_RX_DMA_CTRL_DIRECT                        (1<<28)
#define IOHUB_RX_DMA_CTRL_STATE                         (0xFC00)
#define IOHUB_RX_DMA_CTRL_INDEX                         (0x03FF)


/* DMA TX control bits */
#define IOHUB_TX_DMA_CTRL_CE                            (1<<31)
#define IOHUB_TX_DMA_CTRL_CA                            (1<<30)
#define IOHUB_TX_DMA_CTRL_FLEXIO                        (1<<29)
#define IOHUB_TX_DMA_CTRL_DIRECT                        (1<<28)
#define IOHUB_TX_DMA_CTRL_STATE                         (0xFC00)
#define IOHUB_TX_DMA_CTRL_INDEXEN                       (1<<27)
#define IOHUB_TX_DMA_CTRL_INDEX(i)                      (i & 0x03FF)



/* Interrupt and FIFO status register */
#define IOHUB_IFS_RXNCIP                        (1<<31)
#define IOHUB_IFS_RXECIP                        (1<<30)
#define IOHUB_IFS_RXNRIP                        (1<<29)
#define IOHUB_IFS_RXCAIP                        (1<<28)
#define IOHUB_IFS_TXNCIP                        (1<<24)
#define IOHUB_IFS_TXECIP                        (1<<23)
#define IOHUB_IFS_TXNRIP                        (1<<22)
#define IOHUB_IFS_TXCAIP                        (1<<21)
#define IOHUB_IFS_TXFUFIP                       (1<<20)
#define IOHUB_IFS_TXFSRIP                       (1<<19)
#define IOHUB_IFS_MODIP                         (1<<18)
#define IOHUB_IFS_DMA_TX                        (IOHUB_IFS_TXNCIP | \
                                                IOHUB_IFS_TXNRIP | \
                                                IOHUB_IFS_TXECIP | \
                                                IOHUB_IFS_TXCAIP)
#define IOHUB_IFS_DMA_RX                        (IOHUB_IFS_RXNCIP | \
                                                IOHUB_IFS_RXNRIP | \
                                                IOHUB_IFS_RXECIP | \
                                                IOHUB_IFS_RXCAIP)

/* Interrupt configuration register */
#define IOHUB_ICTRL_RXTHRS(val)                 (val<<28)
#define IOHUB_ICTRL_RXFOFIE                     (1<<26)
#define IOHUB_ICTRL_RXFSRIE                     (1<<25)
#define IOHUB_ICTRL_RXNCIE                      (1<<24)
#define IOHUB_ICTRL_RXECIE                      (1<<23)
#define IOHUB_ICTRL_RXNRIE                      (1<<22)
#define IOHUB_ICTRL_RXCAIE                      (1<<21)
#define IOHUB_ICTRL_RXPCIE                      (1<<20)
#define IOHUB_ICTRL_WSTAT                       (1<<19)
#define IOHUB_ICTRL_ISTAT                       (1<<18)
#define IOHUB_ICTRL_LSTAT                       (1<<17)
#define IOHUB_ICTRL_FSTAT                       (1<<16)
#define IOHUB_ICTRL_BLENSTAT                    (0xFF)
#define IOHUB_ICTRL_RXALLE                      (IOHUB_ICTRL_RXFOFIE | \
                                                IOHUB_ICTRL_RXFSRIE | \
                                                IOHUB_ICTRL_RXNCIE | \
                                                IOHUB_ICTRL_RXECIE | \
                                                IOHUB_ICTRL_RXNRIE | \
                                                IOHUB_ICTRL_RXCAIE | \
                                                IOHUB_ICTRL_RXPCIE)


#define IOHUB_ICTRL_TXTHRS(val)                 (val<<28)
#define IOHUB_ICTRL_TXFUFIE                     (1<<26)
#define IOHUB_ICTRL_TXFSRIE                     (1<<25)
#define IOHUB_ICTRL_TXNCIE                      (1<<24)
#define IOHUB_ICTRL_TXECIE                      (1<<23)
#define IOHUB_ICTRL_TXNRIE                      (1<<22)
#define IOHUB_ICTRL_TXCAIE                      (1<<21)
#define IOHUB_ICTRL_WSTAT                       (1<<19)
#define IOHUB_ICTRL_ISTAT                       (1<<18)
#define IOHUB_ICTRL_LSTAT                       (1<<17)
#define IOHUB_ICTRL_FSTAT                       (1<<16)
#define IOHUB_ICTRL_BLENSTAT                    (0xFF)
#define IOHUB_ICTRL_TXALLE                      (IOHUB_ICTRL_TXFUFIE | \
                                                IOHUB_ICTRL_TXFSRIE | \
                                                IOHUB_ICTRL_TXNCIE | \
                                                IOHUB_ICTRL_TXECIE | \
                                                IOHUB_ICTRL_TXNRIE | \
                                                IOHUB_ICTRL_TXCAIE)



#endif /* ifndef __ASM_ARCH_REGSIOHUB_NS921X_H */




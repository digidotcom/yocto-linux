/*
 * digiDebug.c
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/*
 * This file contains some debugging routines.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>

#include "pipermain.h"
#include "mac.h"

#define DUMP_WORDS_MAX      (700)
static unsigned int dumpWordsWord[DUMP_WORDS_MAX];
static unsigned int dumpWordsCount = 0;

void digiWifiDumpWordsAdd(unsigned int word)
{
    if (dumpWordsCount < DUMP_WORDS_MAX)
    {
        dumpWordsWord[dumpWordsCount++] = word;
    }
}
void digiWifiDumpWordsDump(void)
{
    unsigned int *p = dumpWordsWord;
    unsigned int wordsToGo = dumpWordsCount;

    dumpWordsCount = 0;

    while (wordsToGo >= 4)
    {
        digi_dbg("%8.8X %8.8X - %8.8X %8.8X\n", p[0], p[1], p[2], p[3]);
        p += 4;
        wordsToGo -= 4;
    }
    if (wordsToGo == 3)
    {
        digi_dbg("%8.8X %8.8X - %8.8X\n", p[0], p[1], p[2]);
    }
    else if (wordsToGo == 2)
    {
        digi_dbg("%8.8X %8.8X \n", p[0], p[1]);
    }
    else if (wordsToGo == 1)
    {
        digi_dbg("%8.8X \n", p[0]);
    }
    digi_dbg("--------------\n");
}

void digiWifiDumpWordsReset(void)
{
    dumpWordsCount = 0;
}


void digiWifiDumpBuffer(u8 *buffer, unsigned int length)
{
    unsigned int i, word;

    digiWifiDumpWordsReset();

    for (i = 0; i < length / sizeof(unsigned int); i++)
    {
        memcpy(&word, &buffer[i*sizeof(unsigned int)], sizeof(word));
        digiWifiDumpWordsAdd(cpu_to_be32(word));
    }

    digiWifiDumpWordsDump();
}


void digiWifiDumpSkb(struct sk_buff *skb)
{
    unsigned int bytesLeft = skb->len;
    unsigned char *p = skb->data;

    digi_dbg("skb has %d bytes\n", skb->len);
    while (bytesLeft >= 16)
    {
        digi_dbg("%2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X - %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X\n",
                 p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
        p += 16;
        bytesLeft -= 16;
    }
    if (bytesLeft >= 8)
    {
        digi_dbg("%2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X\n",
                 p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
        p += 8;
        bytesLeft -= 8;
    }
    if (bytesLeft >= 4)
    {
        digi_dbg("%2.2X %2.2X %2.2X %2.2X \n",
                 p[0], p[1], p[2], p[3]);
        p += 4;
        bytesLeft -= 4;
    }
    if (bytesLeft >= 2)
    {
        digi_dbg("%2.2X %2.2X \n",
                 p[0], p[1]);
        p += 2;
        bytesLeft -= 2;
    }
    if (bytesLeft >= 1)
    {
        digi_dbg("%2.2X \n",
                 p[0]);
        p += 1;
        bytesLeft -= 1;
    }
}

EXPORT_SYMBOL_GPL(digiWifiDumpSkb);


void digiWifiDumpRegisters(struct piper_priv *digi, unsigned int regs)
{
#ifdef WANT_DEBUG
    unsigned int i;

    if (regs & CTRL_STATUS_REGS)
    {
                printk(KERN_ERR "  %10.10s = 0x%8.8X  %10.10s = 0x%8.8X\n", "Gen Ctrl", digi->ac->rd_reg(digi, BB_GENERAL_CTL),
                         "Gen Status", digi->ac->rd_reg(digi, BB_GENERAL_STAT));
    }
    else if (regs & IRQ_REGS)
    {
                printk(KERN_ERR "  %10.10s = 0x%8.8X  %10.10s = 0x%8.8X\n", "IRQ Mask", digi->ac->rd_reg(digi, BB_IRQ_MASK),
                         "IRQ Status", digi->ac->rd_reg(digi, BB_IRQ_STAT));
    }
    else if (regs & MAIN_REGS)
    {
        const char *regNames[] = {"Version", "Gen Ctrl", "Gen Status", "RSSI/AES",
                                  "Int Mask", "Int Status", "SPI Data", "SPI Ctrl",
                                  "Data FIFO", "not used", "conf-1", "conf-2", "AES FIFO",
                                "not used", "AES Ctrl", "IO Ctrl"};
        printk(KERN_ERR "Main Registers:\n");
        for (i = BB_VERSION; i <= BB_OUTPUT_CONTROL; i = i+8)
        {
            if ((i != BB_DATA_FIFO) && (i != BB_AES_FIFO))
            {
                printk(KERN_ERR "  %10.10s = 0x%8.8X  %10.10s = 0x%8.8X\n", regNames[i>>2], digi->ac->rd_reg(digi, i), regNames[(i>>2) + 1], digi->ac->rd_reg(digi, i+4));
            }
        }
    }
    if (regs & MAC_REGS)
    {
        const char *regNames[] = {"STA ID0", "STA ID1", "BSS ID0", "BSS ID1",
                                  "OFDM/PSK", "Backoff", "DTIM/List", "B Int",
                                "Rev/M Stat", "C C/M CTL", "Measure", "Beac Fltr"};

        printk(KERN_ERR "Secondary Registers:\n");
        for (i = MAC_STA_ID0; i <= MAC_BEACON_FILT; i = i+8)
        {
            printk(KERN_ERR "  %10.10s = 0x%8.8X  %10.10s = 0x%8.8X\n", regNames[((i - MAC_STA_ID0) >>2)], digi->ac->rd_reg(digi, i), regNames[((i - MAC_STA_ID0)>>2) + 1], digi->ac->rd_reg(digi, i+4));
        }
    }
    if (regs & FRAME_BUFFER_REGS)
    {
        unsigned int word[4];
        printk(KERN_ERR "Real time frame buffer\n");

        word[0] = be32_to_cpu(digi->ac->rd_reg(digi, 0xc0));
        word[1] = be32_to_cpu(digi->ac->rd_reg(digi, 0xc4));
        word[2] = be32_to_cpu(digi->ac->rd_reg(digi, 0xc8));
        word[3] = be32_to_cpu(digi->ac->rd_reg(digi, 0xcc));
        printk(KERN_ERR " %8.8X %8.8X - %8.8X %8.8X\n", word[0], word[1], word[2], word[3]);
        word[0] = be32_to_cpu(digi->ac->rd_reg(digi, 0xd0));
        word[1] = be32_to_cpu(digi->ac->rd_reg(digi, 0xd4));
        word[2] = be32_to_cpu(digi->ac->rd_reg(digi, 0xd8));
        word[3] = be32_to_cpu(digi->ac->rd_reg(digi, 0xdc));
        printk(KERN_ERR " %8.8X %8.8X - %8.8X %8.8X\n", word[0], word[1], word[2], word[3]);
    }
    if (regs & FIFO_REGS)
    {
        unsigned int word[4];
        printk(KERN_ERR "FIFO contents\n");

        word[0] = digi->ac->rd_reg(digi, BB_DATA_FIFO);
        word[1] = digi->ac->rd_reg(digi, BB_DATA_FIFO);
        word[2] = digi->ac->rd_reg(digi, BB_DATA_FIFO);
        word[3] = digi->ac->rd_reg(digi, BB_DATA_FIFO);
        printk(KERN_ERR " %8.8X %8.8X - %8.8X %8.8X\n", word[0], word[1], word[2], word[3]);
        word[0] = digi->ac->rd_reg(digi, BB_DATA_FIFO);
        word[1] = digi->ac->rd_reg(digi, BB_DATA_FIFO);
        word[2] = digi->ac->rd_reg(digi, BB_DATA_FIFO);
        word[3] = digi->ac->rd_reg(digi, BB_DATA_FIFO);
        printk(KERN_ERR " %8.8X %8.8X - %8.8X %8.8X\n", word[0], word[1], word[2], word[3]);
    }
#endif
}
EXPORT_SYMBOL_GPL(digiWifiDumpRegisters);

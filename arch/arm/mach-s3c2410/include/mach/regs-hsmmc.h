

/*
 * HS MMC Interface (chapter28)
 */

#ifndef __ASM_ARM_REGS_HSMMC
#define __ASM_ARM_REGS_HSMMC "regs-hsmmc.h"


#define S3C2410_HSMMC_SYSAD                (0x00)
#define S3C2410_HSMMC_BLKSIZE              (0x04)
#define S3C2410_HSMMC_BLKCNT               (0x06)
#define S3C2410_HSMMC_ARGUMENT             (0x08)
#define S3C2410_HSMMC_TRNMOD               (0x0c)
#define S3C2410_HSMMC_CMDREG               (0x0e)
#define S3C2410_HSMMC_RSPREG0              (0x10)
#define S3C2410_HSMMC_RSPREG1              (0x14)
#define S3C2410_HSMMC_RSPREG2              (0x18)
#define S3C2410_HSMMC_RSPREG3              (0x1c)
#define S3C2410_HSMMC_BDATA                (0x20)
#define S3C2410_HSMMC_PRNSTS               (0x24)
#define S3C2410_HSMMC_HOSTCTL              (0x28)
#define S3C2410_HSMMC_PWRCON               (0x29)
#define S3C2410_HSMMC_BLKGAP               (0x2a)
#define S3C2410_HSMMC_WAKCON               (0x2b)
#define S3C2410_HSMMC_CLKCON               (0x2c)
#define S3C2410_HSMMC_TIMEOUTCON           (0x2e)
#define S3C2410_HSMMC_SWRST                (0x2f)
#define S3C2410_HSMMC_NORINTSTS            (0x30)
#define S3C2410_HSMMC_ERRINTSTS            (0x32)
#define S3C2410_HSMMC_NORINTSTSEN          (0x34)
#define S3C2410_HSMMC_ERRINTSTSEN          (0x36)
#define S3C2410_HSMMC_NORINTSIGEN          (0x38)
#define S3C2410_HSMMC_ERRINTSIGEN          (0x3a)
#define S3C2410_HSMMC_ACMD12ERRSTS         (0x3c)
#define S3C2410_HSMMC_CAPAREG              (0x40)
#define S3C2410_HSMMC_MAXCURR              (0x48)
#define S3C2410_HSMMC_CONTROL2             (0x80) 
#define S3C2410_HSMMC_CONTROL3             (0x84) 
#define S3C2410_HSMMC_HCVER                (0xfe)



#endif /* __ASM_ARM_REGS_HSMMC */

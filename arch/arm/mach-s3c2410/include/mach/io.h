/*
 * arch/arm/mach-s3c2410/include/mach/io.h
 *  from arch/arm/mach-rpc/include/mach/io.h
 *
 * Copyright (C) 1997 Russell King
 *	     (C) 2003 Simtec Electronics
*/

#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#include <mach/hardware.h>

#define IO_SPACE_LIMIT 0xffffffff

/*
 * We use two different types of addressing - PC style addresses, and ARM
 * addresses.  PC style accesses the PC hardware with the normal PC IO
 * addresses, eg 0x3f8 for serial#1.  ARM addresses are above A28
 * and are translated to the start of IO.  Note that all addresses are
 * not shifted left!
 */

#define __PORT_PCIO(x)	((x) < (1<<28))

#define PCIO_BASE	 (S3C24XX_VA_ISA_WORD)
#define PCIO_BASE_b	 (S3C24XX_VA_ISA_BYTE)
#define PCIO_BASE_w	 (S3C24XX_VA_ISA_WORD)
#define PCIO_BASE_l	 (S3C24XX_VA_ISA_WORD)
/*
 * Dynamic IO functions - let the compiler
 * optimize the expressions
 */

#define DECLARE_DYN_OUT(sz,fnsuffix,instr) \
static inline void __out##fnsuffix (unsigned int val, unsigned int port) \
{ \
	unsigned long temp;				      \
	__asm__ __volatile__(				      \
	"cmp	%2, #(1<<28)\n\t"			      \
	"mov	%0, %2\n\t"				      \
	"addcc	%0, %0, %3\n\t"				      \
	"str" instr " %1, [%0, #0 ]	@ out" #fnsuffix      \
	: "=&r" (temp)					      \
	: "r" (val), "r" (port), "Ir" (PCIO_BASE_##fnsuffix)  \
	: "cc");					      \
}


#define DECLARE_DYN_IN(sz,fnsuffix,instr)				\
static inline unsigned sz __in##fnsuffix (unsigned int port)		\
{									\
	unsigned long temp, value;					\
	__asm__ __volatile__(						\
	"cmp	%2, #(1<<28)\n\t"					\
	"mov	%0, %2\n\t"						\
	"addcc	%0, %0, %3\n\t"						\
	"ldr" instr "	%1, [%0, #0 ]	@ in" #fnsuffix		\
	: "=&r" (temp), "=r" (value)					\
	: "r" (port), "Ir" (PCIO_BASE_##fnsuffix)	\
	: "cc");							\
	return (unsigned sz)value;					\
}

static inline void __iomem *__ioaddr (unsigned long port)
{
	return __PORT_PCIO(port) ? (PCIO_BASE + port) : (void __iomem *)port;
}

#define DECLARE_IO(sz,fnsuffix,instr)	\
	DECLARE_DYN_IN(sz,fnsuffix,instr) \
	DECLARE_DYN_OUT(sz,fnsuffix,instr)

DECLARE_IO(char,b,"b")
DECLARE_IO(short,w,"h")
DECLARE_IO(int,l,"")


#define __mem_pci(x)	(x)
#define __io(x)		((void __iomem *)(x))

#endif

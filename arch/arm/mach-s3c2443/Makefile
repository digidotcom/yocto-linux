# arch/arm/mach-s3c2443/Makefile
#
# Copyright 2007 Simtec Electronics
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:=
obj-				:=

obj-$(CONFIG_CPU_S3C2443)	+= s3c2443.o
obj-$(CONFIG_CPU_S3C2443)	+= irq.o
obj-$(CONFIG_CPU_S3C2443)	+= clock.o

obj-$(CONFIG_S3C2443_DMA)	+= dma.o

# Machine support

obj-$(CONFIG_MACH_SMDK2443)	+= mach-smdk2443.o
obj-$(CONFIG_MACH_CC9M2443JS)	+= mach-cc9m2443js.o gpio.o
obj-$(CONFIG_MACH_CCW9M2443JS)	+= mach-ccw9m2443js.o piper-ccw9m2443.o gpio.o

ifeq ($(CONFIG_MACH_CCW9M2443JS),y)
CFLAGS_piper-ccw9m2443.o	+= -I$(srctree)/drivers/net/wireless/digiPiper/
CFLAGS_mach-ccw9m2443js.o	+= -I$(srctree)/drivers/net/wireless/digiPiper/
endif


# Support for the USB overload protection
obj-$(CONFIG_MACH_CC9M2443JS_USB) += cc9m2443js-usb.o

# Power Management support
obj-$(CONFIG_MACH_CC9M2443JS_PM) += cc9m2443js-pm.o cc9m2443js-sleep.o

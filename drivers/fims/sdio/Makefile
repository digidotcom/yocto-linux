#
# Makefile for the kernel FIM device drivers
#

ifneq ($(CONFIG_FIM_SDIO_MODULE),)
	obj-m					+= fim_sdio.o
else
	obj-$(CONFIG_FIM_ZERO_SDIO)		+= fim_sdio.o
	obj-$(CONFIG_FIM_ONE_SDIO)		+= fim_sdio.o
endif

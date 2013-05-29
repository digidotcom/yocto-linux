#
# Makefile for the kernel FIM device drivers
#

ifneq ($(CONFIG_FIM_SPI_MODULE),)
	obj-m				+= fim_spi.o
else
	obj-$(CONFIG_FIM_ZERO_SPI)	+= fim_spi.o
	obj-$(CONFIG_FIM_ONE_SPI)	+= fim_spi.o
endif

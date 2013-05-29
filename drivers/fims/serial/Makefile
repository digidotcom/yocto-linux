#
# Makefile for the kernel FIM device drivers
#

ifneq ($(CONFIG_FIM_SERIAL_MODULE),)
	obj-m				+= fim_serial.o
else
	obj-$(CONFIG_FIM_ZERO_SERIAL)	+= fim_serial.o
	obj-$(CONFIG_FIM_ONE_SERIAL)	+= fim_serial.o
endif
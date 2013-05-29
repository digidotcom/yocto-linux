#
# Makefile for the kernel FIM device drivers
#

ifneq ($(CONFIG_FIM_CAN_MODULE),)
	obj-m				+= fim_can.o
else
	obj-$(CONFIG_FIM_ZERO_CAN)	+= fim_can.o
	obj-$(CONFIG_FIM_ONE_CAN)	+= fim_can.o
endif

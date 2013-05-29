#
# Makefile for the kernel FIM device drivers
#

ifneq ($(CONFIG_FIM_USB_MODULE),)
	obj-m					+= fim_usb.o
else
	obj-$(CONFIG_FIM_ZERO_USB)		+= fim_usb.o
	obj-$(CONFIG_FIM_ONE_USB)		+= fim_usb.o
endif

#
# Makefile for the linux kernel.
#
obj-y += pinctrl.o clock.o device.o serial.o bus_freq.o pm.o sleep.o

# Board select
obj-$(CONFIG_MACH_MX28EVK) += mx28evk.o mx28evk_pins.o
obj-$(CONFIG_MACH_CPX2) += mx28_cpx2.o mx28_cpx2_pins.o
obj-$(CONFIG_MODULE_CCARDIMX28) += mx28_ccardimx28.o mx28_ccardimx28_pins.o hwid_ccardimx28.o
obj-$(CONFIG_MACH_WR21) += mx28_wr21.o mx28_wr21_pins.o
obj-$(CONFIG_REGULATOR_MXS) += power.o
obj-$(CONFIG_GENERIC_GPIO) += gpio.o
obj-$(CONFIG_MXS_RAM_FREQ_SCALING) +=emi.o emi_settings.o
obj-$(if $(CONFIG_FB_MXS),y) += lcdif.o

# USB support
ifneq ($(strip $(CONFIG_USB_GADGET_ARC) $(CONFIG_USB_EHCI_ARC_OTG)),)
	obj-y   += usb_dr.o
endif
obj-$(CONFIG_USB_EHCI_ARC_H1)   += usb_h1.o

obj-y := clock.o gpio.o irq.o

ifeq ($(CONFIG_GPIOLIB),y)
gpio-$(CONFIG_PROCESSOR_NS921X) += gpiolib-ns921x.o
gpio-$(CONFIG_PROCESSOR_NS9360) += gpiolib-ns9360.o
endif

obj-$(CONFIG_MACH_CC7UCAMRY) += mach-cc7ucamry.o
obj-$(CONFIG_MACH_CC9C) += mach-cc9c.o
obj-$(CONFIG_MACH_CC9P9215) += mach-cc9p9215.o
obj-$(CONFIG_MACH_CC9P9215JS) += mach-cc9p9215js.o
obj-$(CONFIG_MACH_CC9P9360DEV) += mach-cc9p9360dev.o
obj-$(CONFIG_MACH_CC9P9360JS) += mach-cc9p9360js.o
obj-$(CONFIG_MACH_CME9210) += mach-cme9210.o
obj-$(CONFIG_MACH_CME9210JS) += mach-cme9210js.o
obj-$(CONFIG_MACH_CWME9210JS) += mach-cwme9210js.o
obj-$(CONFIG_MACH_CCW9P9215) += mach-ccw9p9215.o
obj-$(CONFIG_MACH_CCW9P9215JS) += mach-ccw9p9215js.o
obj-$(CONFIG_MACH_CCW9C) += mach-ccw9c.o
obj-$(CONFIG_MACH_INC20OTTER) += mach-inc20otter.o
obj-$(CONFIG_MACH_OTTER) += mach-otter.o

obj-$(CONFIG_PROCESSOR_NS921X) += gpio-ns921x.o irq-ns921x.o processor-ns921x.o time-ns921x.o ns921x_devices.o
obj-$(CONFIG_PROCESSOR_NS9215) += ns9215_devices.o
obj-$(CONFIG_PROCESSOR_NS9360) += gpio-ns9360.o processor-ns9360.o time-ns9360.o ns9360_devices.o

obj-$(CONFIG_MODULE_CC9P9215) += cc9p9215_devices.o
ifeq ($(CONFIG_MODULE_CCW9P9215),y)
CFLAGS_ccw9p9215_devices.o += -I$(srctree)/drivers/net/wireless/digiPiper/
CFLAGS_mach-ccw9p9215js.o += -I$(srctree)/drivers/net/wireless/digiPiper/
endif
obj-$(CONFIG_MODULE_CCW9P9215) += cc9p9215_devices.o ccw9p9215_devices.o
obj-$(CONFIG_MODULE_CC9P9360) += cc9p9360_devices.o
obj-$(CONFIG_MODULE_CC9C) += ccx9c_devices.o
obj-$(CONFIG_MODULE_CCW9C) += ccx9c_devices.o
obj-$(CONFIG_MODULE_CME9210) += cme9210_devices.o
obj-$(CONFIG_MODULE_CWME9210) += cme9210_devices.o

obj-$(CONFIG_LEDS_CPU) += leds.o

# Power Management
obj-$(CONFIG_PM) += pm.o
pm-$(CONFIG_PROCESSOR_NS921X) += pm-ns921x.o
ifeq ($(CONFIG_PM_VERBOSE),y)
CFLAGS_pm-ns921x.o += -DDEBUG
endif


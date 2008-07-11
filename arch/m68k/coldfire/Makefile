#
# Makefile for Linux arch/m68k/coldfire source directory
#

obj-y:= entry.o config.o cache.o signal.o muldi3.o traps.o ints.o
ifdef CONFIG_M5445X
ifneq ($(strip $(CONFIG_USB) $(CONFIG_USB_GADGET_MCF5445X)),)
	obj-y	+= usb.o usb/
endif
endif

ifdef CONFIG_M547X_8X
obj-$(CONFIG_PCI)       += mcf548x-pci.o
else
obj-$(CONFIG_PCI)	+= pci.o mcf5445x-pci.o iomap.o
endif
obj-$(CONFIG_M5445X)	+= mcf5445x-devices.o
obj-$(CONFIG_M547X_8X)	+= m547x_8x-devices.o
obj-$(CONFIG_M547X_8X)	+= mcf548x-devices.o
obj-$(CONFIG_MCD_DMA)	+= m547x_8x-dma.o

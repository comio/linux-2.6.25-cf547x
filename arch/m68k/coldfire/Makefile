#
# Makefile for Linux arch/m68k/coldfire source directory
#

obj-y:= entry.o config.o cache.o signal.o muldi3.o traps.o ints.o

ifneq ($(strip $(CONFIG_USB) $(CONFIG_USB_GADGET_MCF5445X)),)
	obj-y	+= usb.o usb/
endif

obj-$(CONFIG_PCI)	+= pci.o mcf5445x-pci.o iomap.o
obj-$(CONFIG_M54455)	+= mcf5445x-devices.o

obj-$(CONFIG_MCD_DMA)	+= dma.o

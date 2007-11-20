#
# Makefile for the linux kernel.
#

# Object file lists.

ifneq ($(CONFIG_USB_EHCI_HCD),)
	obj-y	+= otg_host.o
endif

ifneq ($(CONFIG_USB_GADGET_MCF5445X),)
	obj-y	+= otg_device.o
endif

ifneq ($(strip $(CONFIG_USB_GADGET_MCF5445X) $(CONFIG_USB_EHCI_HCD)),)
	obj-y	+= otg_cmn.o
endif

ifneq ($(CONFIG_USB_OTG),)
	obj-y	+= otg_otg.o
endif


# USB Transceiver driver:
ifneq ($(strip $(CONFIG_USB) $(CONFIG_USB_GADGET_MCF5445X)),)
	obj-y	+= xcvr.o
endif


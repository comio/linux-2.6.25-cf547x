#
# Makefile for ColdFire (MCF547x/8x) GPIO device driver.
# (c) 2007 Industrie Dial Face S.p.A.
# 
# by Luigi 'Comio' Mantellini
#

mcf_gpio-objs                    = mcf_gpio_base.o mcf_gpio_callbacks.o
ifeq ($(CONFIG_MCF_GPIO_PROCFS),y)
    mcf_gpio-objs += mcf_gpio_procfs.o
endif
ifeq ($(CONFIG_MCF_GPIO_IOCTL),y)
    mcf_gpio-objs += mcf_gpio_ioctl.o
endif
obj-$(CONFIG_MCF_GPIO) 	        += mcf_gpio.o
obj-$(CONFIG_MCF_GPIO_DMA)      += mcf_gpio_dma.o
obj-$(CONFIG_MCF_GPIO_FBCS)     += mcf_gpio_fbcs.o
obj-$(CONFIG_MCF_GPIO_FBCTL)    += mcf_gpio_fbctl.o
obj-$(CONFIG_MCF_GPIO_FEC1L)    += mcf_gpio_fec1l.o
obj-$(CONFIG_MCF_GPIO_FEC1H)    += mcf_gpio_fec1h.o
obj-$(CONFIG_MCF_GPIO_FEC0L)    += mcf_gpio_fec0l.o
obj-$(CONFIG_MCF_GPIO_FEC0H)    += mcf_gpio_fec0h.o
obj-$(CONFIG_MCF_GPIO_PCIBR)    += mcf_gpio_pcibr.o
obj-$(CONFIG_MCF_GPIO_PCIBG)    += mcf_gpio_pcibg.o
obj-$(CONFIG_MCF_GPIO_FECI2C)   += mcf_gpio_feci2c.o
obj-$(CONFIG_MCF_GPIO_DSPI)     += mcf_gpio_dspi.o
obj-$(CONFIG_MCF_GPIO_PSC1PSC0) += mcf_gpio_psc1psc0.o
obj-$(CONFIG_MCF_GPIO_PSC3PSC2) += mcf_gpio_psc3psc2.o
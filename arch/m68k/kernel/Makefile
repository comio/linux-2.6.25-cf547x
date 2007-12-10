#
# Makefile for the linux kernel.
#

ifdef CONFIG_SUN3
  extra-y := sun3-head.o vmlinux.lds
  obj-y := entry.o signal.o ints.o
else
ifndef CONFIG_COLDFIRE
  extra-y := head.o vmlinux.lds
  obj-y := entry.o signal.o traps.o ints.o
  obj-$(CONFIG_PCI) += bios32.o
else   # CONFIG_COLDFIRE
  extra-y := ../coldfire/head.o vmlinux.lds
endif
endif

obj-y	+= process.o ptrace.o module.o \
	   sys_m68k.o time.o semaphore.o setup.o m68k_ksyms.o devres.o

devres-y = ../../../kernel/irq/devres.o

obj-y$(CONFIG_MMU_SUN3) += dma.o	# no, it's not a typo

EXTRA_AFLAGS := -traditional

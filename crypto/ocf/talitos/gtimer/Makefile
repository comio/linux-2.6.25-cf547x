
ifdef obj
my_inc_base = $(obj)
else
my_inc_base = .
O_TARGET := gtimer.o
endif

obj-$(CONFIG_OCF_TALITOS_GTIMER)    += $(obj-base)gtimer.o

EXTRA_CFLAGS += -I$(my_inc_base)/.



ifdef obj
my_inc_base = $(obj)
else
my_inc_base = .
O_TARGET := ktimer.o
endif

obj-$(CONFIG_OCF_TALITOS_KTIMER)    += $(obj-base)ktimer.o

EXTRA_CFLAGS += -I$(my_inc_base)/.

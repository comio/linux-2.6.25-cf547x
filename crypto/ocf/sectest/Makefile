# Copyright (C) 2007 Freescale Semiconductor, Inc. All rights reserved.
#
# Author:Roy Zang <tie-fei.zang@freescale.com>, July, 2007
#
# Description:
# 2.6 Makefile for sec device driver test
#
# This is free software; you can redistribute it and/or modify
# it under the terms of  the GNU General  Public License as published by
# the Free Software Foundation;  either version 2 of the  License, or
# (at your option) any later version.
#

EXTRA_CFLAGS += -Icrypto/ocf

obj-$(CONFIG_SEC_TEST) += ocf_des.o ocf_aes.o ocf_arc4.o
sec2drvTest-objs := ocf_des.o

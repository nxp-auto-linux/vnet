#
# Freescale PCI Express virtual network driver for S32V234 
# Copyright 2017 NXP
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#

obj-$(CONFIG_VNET_S32) := fpx.o
PWD := $(shell pwd)
all:
	make -C $(KERNEL_DIR) SUBDIRS=$(PWD) modules
clean:
	make -C $(KERNEL_DIR) SUBDIRS=$(PWD) clean


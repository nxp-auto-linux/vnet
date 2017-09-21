#
# Freescale PCI Express virtual network driver for LayerScape 
# Copyright (C) 2017 NXP
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.


obj-$(CONFIG_VNET_LS2) := fpx.o
PWD := $(shell pwd)
all:
	make -C $(KERNEL_DIR) SUBDIRS=$(PWD) modules
clean:
	make -C $(KERNEL_DIR) SUBDIRS=$(PWD) clean


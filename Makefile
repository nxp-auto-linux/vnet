# Copyright 2018 NXP
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.

ifdef CONFIG_NXP_VETH
obj-$(CONFIG_NXP_VETH) += nxp-veth.o

nxp-veth-objs := nxp-ndev.o nxp-pci.o

ifeq ($(CONFIG_NXP_VETH_BBMINI),y)
nxp-veth-objs += bbmini.o
else ifeq ($(CONFIG_NXP_VETH_LS1043_CM7FPGA),y)
nxp-veth-objs += ls1043-cm7fpga.o
endif

endif #CONFIG_NXP_VETH

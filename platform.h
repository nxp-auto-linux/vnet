/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef DRIVERS_NET_ETHERNET_PCI_VDEV_PLATFORM_H_
#define DRIVERS_NET_ETHERNET_PCI_VDEV_PLATFORM_H_

int nxp_pfm_init(void **platform);
void nxp_pfm_free(void *platform);

void __iomem *nxp_pfm_alloc_local_shm(void *dev);
void nxp_pfm_free_local_shm(void *dev, void __iomem *addr);

int nxp_pfm_dma_write(void *platform, void *src_addr, phys_addr_t dest_addr,
			u32 size);
int nxp_pfm_trigger_remote_irq(void *platform);

#endif /* DRIVERS_NET_ETHERNET_PCI_VDEV_PLATFORM_H_ */

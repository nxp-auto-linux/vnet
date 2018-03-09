/*
 * Copyright (C) 2018 NXP Semiconductors
 *
 * SPDX-License-Identifier:		BSD-3-Clause
 */
#ifndef DRIVERS_NET_ETHERNET_PCI_VDEV_PLATFORM_H_
#define DRIVERS_NET_ETHERNET_PCI_VDEV_PLATFORM_H_

int nxp_pfm_init(void **platform);
void nxp_pfm_free(void *platform);

void __iomem *nxp_pfm_alloc_local_shm(void *platform, struct pci_dev *pdev);
void nxp_pfm_free_local_shm(void *platform, struct pci_dev *pdev,
			    void __iomem *addr);

void __iomem *nxp_pfm_alloc_remote_shm(void *platform, struct pci_dev *pdev);
void nxp_pfm_free_remote_shm(void *platform, struct pci_dev *pdev,
			     void __iomem *addr);

int nxp_pfm_dma_write(void *platform, const void *src_addr,
		      phys_addr_t dest_addr, u32 size);

int nxp_pfm_trigger_remote_irq(void *platform);

#endif /* DRIVERS_NET_ETHERNET_PCI_VDEV_PLATFORM_H_ */

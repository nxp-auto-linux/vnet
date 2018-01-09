/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef DRIVERS_NET_VPCIE_VPCIE_VPCIE_H_
#define DRIVERS_NET_VPCIE_PCIE_VPCIE_H_

#define LS_PCI_SMEM		0x83A0000000ULL	/* Remote, LS */
#define LS_PCI_SMEM_SIZE	0x00400000	/* 4 MB */

#define QDMA_BASE		0x8390100
#define QDMA_REG_SIZE		0x100

#define LS2S32V_INT_PIN		434	/* GPIO interrupt pin */

/* TODO: test w/o volatile */
struct nxp_pci_shm {
	volatile u64 write_index;
	volatile u64 read_index;
	volatile u64 pad[6];
	volatile void* data;
};

/**
 * struct nxp_pdev_priv - NXP generic PCI device
 *
 * @pci_dev:	PCI device structure
 * @upper_dev:	specific device built on nxp_pdev (i.e., net dev, tty dev, etc)
 *
 * Private member priv_data is populated by init function with internal data
 * needed by transport device for read/write operations.
 */
struct nxp_pdev_priv {
	/* Hardware registers of the fpx device */
	struct pci_dev *pci_dev;
	void *upper_dev;

	volatile u32* qdma_regs;
	u32 gpio_level;

	struct nxp_pci_shm *local_shm;
	struct nxp_pci_shm *remote_shm;
};

int nxp_pdev_init(struct pci_dev *pdev, void *upper_dev);
void nxp_pdev_free(struct pci_dev *pdev);
void *nxp_pdev_get_upper_dev(struct pci_dev *pdev);

int nxp_

#endif /* DRIVERS_NET_VPCIE_VPCIE_VPCIE_H_ */

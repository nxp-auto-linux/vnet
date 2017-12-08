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

#define S32_PCI_SMEM		0xfd000000

#define S32_PCI_SMEM_SIZE	0x00400000	/* 4 MB */
#define S32_PCI_MSI_MEM		(S32_PCI_SMEM + S32_PCI_SMEM_SIZE)
#define S32_PCI_MSI_SIZE	0x1000		/* 4 KB */

/* TODO: test w/o volatile */
struct nxp_pci_shm {
	volatile u64 write_index;
	volatile u64 read_index;
	volatile u64 pad[6];
	volatile void* data;
};

struct nxp_pci_priv {
	/* Hardware registers of the fpx device */
	struct pci_dev *pci_dev;

	spinlock_t spinlock;

	volatile u32* qdma_regs;
#if MSI_WORKAROUND
	int irq;
#endif
	struct nxp_pci_shm *local_shm;
	struct nxp_pci_shm *remote_shm;
};

void nxp_pci_dev_init(struct pci_dev *pdev);
void nxp_pci_dev_free(struct pci_dev *pdev);

#endif /* DRIVERS_NET_VPCIE_VPCIE_VPCIE_H_ */

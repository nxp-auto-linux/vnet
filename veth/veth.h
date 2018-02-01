/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef FPX_H
#define	FPX_H

#define MAX_NO_BUFFERS		512
#define MAX_BUFFER_SIZE		1536

#define S32_PCI_SMEM		0xfd000000

#define S32_PCI_SMEM_SIZE	0x00400000	/* 4 MB */
#define S32_PCI_MSI_MEM		(S32_PCI_SMEM + S32_PCI_SMEM_SIZE)
#define S32_PCI_MSI_SIZE	0x1000		/* 4 KB */

#define LS_PCI_SMEM		0x83A0000000ULL	/* Remote, LS */
#define LS_PCI_SMEM_SIZE	0x00400000	/* 4 MB */
#define LS_REMOTE_PCI_BASE	0x0000003840000000	/* Local, LS */

#define QDMA_BASE		0x8390100
#define QDMA_REG_SIZE		0x100

#define LS2S32V_INT_PIN		434


struct control_ved
{
	volatile u64 current_write_index;
	volatile u64 current_read_index;
	volatile u64 val[6];
};

#define OFFSET_TO_DATA		sizeof(struct control_ved)
struct fpx_enet_private {
	/* Hardware registers of the fpx device */
	struct pci_dev *pci_dev;
	struct control_ved *ctrl_ved_l; /* control virtual ethernet device, local */
	struct control_ved *ctrl_ved_r; /* control virtual ethernet device, remote */
	struct net_device *netdev;
	struct napi_struct napi;

	struct resource *local_res;
	struct resource *remote_res;

	u32 level;
	volatile u32* qdma_regs;

	volatile void* received_data_l;
	volatile void* received_data_r;

	spinlock_t spinlock;
};

#endif /* FPX_H */

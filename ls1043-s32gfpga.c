/*
 * Copyright (C) 2018 NXP Semiconductors
 *
 * SPDX-License-Identifier:		BSD-3-Clause
 */
#include <linux/kernel.h>
#include <linux/pci.h>
#include "platform.h"
#include "s32gfpga-mscm.h"

/* pci-vdev platform specific shared memory configuration */
#define PCIE_LSHM_OFFSET    (0x10000ul) /* Local Shared Memory Offset */
#define PCIE_RSHM_OFFSET    (0x00000ul) /* Remote Shared Memory Offset */

/* S32G-FPGA IP Block Base Addresses */
#define IRAM_BASE     (0x34000000ul) /* Internal Static Random Access Memory */
#define MSCM_BASE     (0x40198000ul) /* Miscellaneous System Control Module */

/* PCIe Base Address Registers  */
#define BAR_NUM 0

/**
 * struct nxp_pfm_priv - platform specific functionalities
 *
 * @s32gfpga_base:      pointer to PCIe-mapped S32G-FPGA address space
 */
struct nxp_pfm_priv {
	volatile u8* s32gfpga_base;
};

int nxp_pfm_init(void **platform)
{
	struct nxp_pfm_priv *priv;

	if (unlikely(!platform))
		return -EINVAL;

	priv = kzalloc(sizeof(struct nxp_pfm_priv), GFP_KERNEL);
	if (!priv) {
		return -EINVAL;
	}

	*platform = priv;
	return 0;
}

void nxp_pfm_free(void *platform)
{
	struct nxp_pfm_priv *priv;

	if (unlikely(!platform))
		return;

	priv = (struct nxp_pfm_priv *)platform;

	kfree(priv);
}

void __iomem *nxp_pfm_alloc_local_shm(void *platform, struct pci_dev *pdev)
{
	struct nxp_pfm_priv *priv;
	u32 io_len;
	
	/* read BAR length and flags */
	io_len = pci_resource_len(pdev, BAR_NUM);
	pr_debug("BAR%d len = 0x%x, flags = 0x%lx\n", BAR_NUM, io_len,
			pci_resource_flags(pdev, BAR_NUM));
	if (!(pci_resource_flags(pdev, BAR_NUM) & IORESOURCE_MEM_64)) {
		pr_err("Bad PCI resource\n");
		return NULL;
	}

	priv = (struct nxp_pfm_priv *)platform;
	
	priv->s32gfpga_base = (u8 *)pci_iomap(pdev, BAR_NUM, io_len);
	
	return (void *)priv->s32gfpga_base + IRAM_BASE + PCIE_LSHM_OFFSET;
}

void nxp_pfm_free_local_shm(void *platform, struct pci_dev *pdev,
			    void __iomem *addr)
{
	pci_iounmap(pdev, addr);
}

void __iomem *nxp_pfm_alloc_remote_shm(void *platform, struct pci_dev *pdev)
{
	struct nxp_pfm_priv *priv;
	
	priv = (struct nxp_pfm_priv *)platform;
	
	return (void *)priv->s32gfpga_base + IRAM_BASE + PCIE_RSHM_OFFSET;
}

void nxp_pfm_free_remote_shm(void *platform, struct pci_dev *pdev,
			     void __iomem *addr)
{
	/* already done in nxp_pfm_free_local_shm() */
}

int nxp_pfm_dma_write(void *platform, const void *src_addr,
			phys_addr_t dest_addr, u32 size)
{
	/* DMA not supported for this platform */
	return -EOPNOTSUPP;
}

int nxp_pfm_trigger_remote_irq(void *platform)
{
	/* platform support not implemented*/
	return -EOPNOTSUPP;
}

/*
 * Copyright (C) 2018 NXP Semiconductors
 *
 * SPDX-License-Identifier:		BSD-3-Clause
 */
#include <linux/kernel.h>
#include <linux/pci.h>

#include "platform.h"
#include "s32gfpga-mscm.h"

/* pci-vdev platform specific interrupt configuration */
#define PCIE_TX_CPU_TARGET    0x1u /* M7 core 0 */
#define PCIE_RX_CPU_TARGET    0x8u /* Root Complex */
#define PCIE_MSCM_IRQ_ID      0u /* Interrupt NVIC ID */
#define PCIE_MSCM_MSI_ID      0u /* Interrupt MSI ID */

/* pci-vdev platform specific shared memory configuration */
#define PCIE_LSHM_OFFSET    0x10000ul /* Local Shared Memory Offset */
#define PCIE_RSHM_OFFSET    0x00000ul /* Remote Shared Memory Offset */

/* S32G-FPGA IP Block Base Addresses */
#define IRAM_BASE     0x34000000ul /* Internal Static Random Access Memory */
#define MSCM_BASE     0x40198000ul /* Miscellaneous System Control Module */

/* PCIe Base Address Registers  */
#define BAR_NUM 0

/**
 * struct nxp_pfm_priv - platform specific functionalities
 *
 * @s32gfpga_base:      pointer to PCIe-mapped S32G-FPGA address space
 * @pci_res_start:      physical start address of PCIe resource
 */
struct nxp_pfm_priv {
	volatile u8* s32gfpga_base;
	volatile u64 pci_res_start;
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
	u64 io_len;
	
	/* read BAR length and flags */
	io_len = pci_resource_len(pdev, BAR_NUM);
	pr_debug("BAR%d len = 0x%016llx, flags = 0x%lx\n", BAR_NUM, io_len,
			pci_resource_flags(pdev, BAR_NUM));

	if (!(pci_resource_flags(pdev, BAR_NUM) & IORESOURCE_MEM_64)) {
		pr_err("Bad PCI resource\n");
		return NULL;
	}

	priv = (struct nxp_pfm_priv *)platform;

	priv->pci_res_start = pci_resource_start(pdev, BAR_NUM);
	
	priv->s32gfpga_base = (u8 *)pci_iomap(pdev, BAR_NUM, io_len);
	
	return (void *)(priv->s32gfpga_base + IRAM_BASE + PCIE_LSHM_OFFSET);
}

void nxp_pfm_free_local_shm(void *platform, struct pci_dev *pdev,
			    void __iomem *addr)
{
	struct nxp_pfm_priv *priv;
	
	priv = (struct nxp_pfm_priv *)platform;
	
	pci_iounmap(pdev, (void *)priv->s32gfpga_base);
}

void __iomem *nxp_pfm_alloc_remote_shm(void *platform, struct pci_dev *pdev)
{
	struct nxp_pfm_priv *priv;
	
	priv = (struct nxp_pfm_priv *)platform;
	
	return (void *)(priv->s32gfpga_base + IRAM_BASE + PCIE_RSHM_OFFSET);
}

void nxp_pfm_free_remote_shm(void *platform, struct pci_dev *pdev,
			     void __iomem *addr)
{
	return; /* already done in nxp_pfm_free_local_shm() */
}

int nxp_pfm_dma_write(void *platform, const void *src_addr,
			phys_addr_t dest_addr, u32 size)
{
	struct nxp_pfm_priv *priv;
	u64 write_offset;
	
	priv = (struct nxp_pfm_priv *)platform;

	write_offset = IRAM_BASE + dest_addr - priv->pci_res_start;

	/* DMA not supported for this platform */
	memcpy((u8*)priv->s32gfpga_base + write_offset, src_addr, size);

	return 0;
}

int nxp_pfm_trigger_remote_irq(void *platform)
{
	struct nxp_pfm_priv *priv;
	volatile struct mscm_memmap *mscm;
	
	if (unlikely(!platform))
		return -EINVAL;
	
	priv = (struct nxp_pfm_priv *)platform;
	
	mscm = (volatile struct mscm_memmap *)(priv->s32gfpga_base + MSCM_BASE);
	
	/* generate MSCM core-to-core interrupt */
	iowrite32(MSCM_IRCPGIR_TLF(0u) /* use CPUTL */
	          | MSCM_IRCPGIR_CPUTL(PCIE_TX_CPU_TARGET)
	          | MSCM_IRCPGIR_INTID(PCIE_MSCM_IRQ_ID),
	          &mscm->IRCPGIR);
	
	return 0;
}

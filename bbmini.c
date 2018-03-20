/*
 * Copyright (C) 2018 NXP Semiconductors
 *
 * SPDX-License-Identifier:		BSD-3-Clause
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/gpio.h>
#include <asm/cacheflush.h>

#include "platform.h"

/* LS2084 shared memory in DDR visible to remote endpoint */
/* TODO: read local shared mem addr from dts */
#define LS_PCI_SMEM		0x83A0000000ULL
#define LS_PCI_SMEM_SIZE	0x00100000	/* 1 MB same as endpoint */
#define SMEM_RES_NAME		"nxp-pci-vdev local shm"

#define BAR_NUM 0

#define QDMA_BASE		0x8390100
#define QDMA_REG_SIZE		0x100

#define LS2S32V_INT_PIN		434	/* GPIO interrupt pin */

/**
 * struct nxp_pfm_priv - platform specific functionalities
 *
 * @qdma_regs:	QDMA configuration registers pointer
 * @gpio_value:	GPIO pin value: 0/1
 */
struct nxp_pfm_priv {
	/* TODO: create memory mapped structure for QDMA regs */
	volatile u32* qdma_regs;

	int gpio_value;
};

/**
 * nxp_pfm_init - initialize platform resources
 * @platform: platform object
 *
 * Create platform object and initialize platform specific IPs: DMA, GPIO, etc.
 *
 * Return: 0 on success, error code otherwise
 */
int nxp_pfm_init(void **platform)
{
	struct nxp_pfm_priv *priv;
	int err;

	if (unlikely(!platform))
		return -EINVAL;

	priv = kzalloc(sizeof(struct nxp_pfm_priv), GFP_KERNEL);
	if (!priv) {
		return -EINVAL;
	}

	/* init qdma for tx */
	priv->qdma_regs = (u32*)ioremap_nocache(QDMA_BASE, QDMA_REG_SIZE);
	if (!priv->qdma_regs) {
		pr_err("Cannot map qdma registers\n");
		return -ENOMEM;
	}

	priv->gpio_value = 0;
	err = gpio_request(LS2S32V_INT_PIN, "LS2_S32V_INT");
	if (err) {
		pr_err("Cannot reserve GPIO LS2S32V_INT_PIN\n");
		err = -ENODEV;
		goto err_qdma_unmap;
	}
	err = gpio_direction_output(LS2S32V_INT_PIN, 0);
	if (err) {
		pr_err("Cannot set GPIO LS2S32V_INT_PIN as output\n");
		err = -ENODEV;
		goto err_gpio_free;
	}

	*platform = priv;
	return 0;

err_gpio_free:
	gpio_free(LS2S32V_INT_PIN);
err_qdma_unmap:
	iounmap(priv->qdma_regs);

	return err;
}

/**
 * nxp_pfm_free - release platform resources
 * @platform: platform object
 */
void nxp_pfm_free(void *platform)
{
	struct nxp_pfm_priv *priv;

	if (unlikely(!platform))
		return;

	priv = (struct nxp_pfm_priv *)platform;

	gpio_free(LS2S32V_INT_PIN);

	/* free QDMA configuration space */
	iounmap(priv->qdma_regs);

	kfree(priv);
}

/**
 * nxp_pfm_alloc_local_shm - alloc local shared memory
 * @platform:	platform object
 * @pdev:	pci device
 *
 * Return: local shared IO memory address
 */
void __iomem *nxp_pfm_alloc_local_shm(void *platform, struct pci_dev *pdev)
{
	request_mem_region(LS_PCI_SMEM, LS_PCI_SMEM_SIZE, SMEM_RES_NAME);
	return ioremap_cache(LS_PCI_SMEM, LS_PCI_SMEM_SIZE);
}

/**
 * nxp_pfm_free_local_shm - release local shared memory
 * @platform:	platform object
 * @pdev:	pci device
 * @addr:	IO memory address to release
 */
void nxp_pfm_free_local_shm(void *platform, struct pci_dev *pdev,
			    void __iomem *addr)
{
	iounmap(addr);
	release_mem_region(LS_PCI_SMEM, LS_PCI_SMEM_SIZE);
}

/**
 * nxp_pfm_alloc_remote_shm - alloc remote shared memory
 * @platform:	platform object
 * @pdev:	pci device
 *
 * Return: local shared IO memory address
 */
void __iomem *nxp_pfm_alloc_remote_shm(void *platform, struct pci_dev *pdev)
{
	u32 io_len;

	/* read bar 0 length and flags */
	io_len = pci_resource_len(pdev, BAR_NUM);
	pr_debug("bar0 len = 0x%x, flags = 0x%lx\n", io_len,
			pci_resource_flags(pdev, BAR_NUM));
	if (!(pci_resource_flags(pdev, BAR_NUM) & IORESOURCE_MEM)) {
		pr_err("Bad PCI resource\n");
		return NULL;
	}

	return pci_iomap(pdev, BAR_NUM, io_len);
}

/**
 * nxp_pfm_free_remote_shm - release remote shared memory
 * @platform:	platform object
 * @pdev:	pci device
 * @addr:	IO memory address to release
 */
void nxp_pfm_free_remote_shm(void *platform, struct pci_dev *pdev,
			     void __iomem *addr)
{
	pci_iounmap(pdev, addr);
}

/**
 * nxp_pdev_write_msg - write message to remote
 * @platform:	platform object
 * @src_addr:	source virtual address
 * @dest_addr:	destination physical address
 * @size:	destination physical address
 *
 * Return:	0 on success, error code otherwise
 *
 * NOTE: src_addr and dest_addr must be 4 bytes aligned for QDMA.
 */
int nxp_pfm_dma_write(void *platform, const void *src_addr,
			phys_addr_t dest_addr, u32 size)
{
	struct nxp_pfm_priv *priv;
	u32 status = 0;

	if (unlikely(!platform || !src_addr))
		return -EINVAL;

	priv = (struct nxp_pfm_priv *)platform;

	/* TODO: investigate the posibility to use public DMA sync API:
	 * dma_sync_single_for_cpu()/dma_sync_single_for_device() */
	__dma_flush_area(src_addr, size);

	*priv->qdma_regs &= ~1;

	status = *(priv->qdma_regs + 1) & 0x92;
	if (status)
		*(priv->qdma_regs + 1) = status;

	*(priv->qdma_regs + 4) = upper_32_bits(virt_to_phys(src_addr)) & 0xffff;
	*(priv->qdma_regs + 5) = lower_32_bits(virt_to_phys(src_addr));

	/* TODO: use upper/lower_32_bits */
	*(priv->qdma_regs + 6) = (u32)(dest_addr >> 32);
	*(priv->qdma_regs + 7) = (u32)dest_addr;
	*(priv->qdma_regs + 8) = size;

	/* start DMA */
	*priv->qdma_regs = 1;

	/* wait for DMA to complete
	 * TODO: use QDMA interrupt instead of polling */
	do {
		status = *(priv->qdma_regs + 1);
	} while (status & (1 << 2));

	/* check for errors */
	if (status & (1 << 7))
		return -EIO;

	return 0;
}

/**
 * nxp_pfm_trigger_remote_irq - Send data available notification to remote peer
 * @platform: platform object
 *
 * Return: local shared memory address
 */
int nxp_pfm_trigger_remote_irq(void *platform)
{
	struct nxp_pfm_priv *priv;

	if (unlikely(!platform))
		return -EINVAL;

	priv = (struct nxp_pfm_priv *)platform;

	priv->gpio_value ^= 1;
	gpio_set_value(LS2S32V_INT_PIN, priv->gpio_value);

	return 0;
}

/*
 * Copyright (C) 2018 NXP Semiconductors
 *
 * SPDX-License-Identifier:		BSD-3-Clause
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/gpio.h>
#include <asm/cacheflush.h>

#include "platform.h"

#define QDMA_BASE		0x8390100
#define QDMA_REG_SIZE		0x100

#define LS2S32V_INT_PIN		434	/* GPIO interrupt pin */

/* LS2084 shared memory in DDR visible to remote endpoint */
/* TODO: read local shared mem addr from dts */
#define LS_PCI_SMEM		0x83A0000000ULL
#define LS_PCI_SMEM_SIZE	0x00400000	/* 4 MB */
#define SMEM_RES_NAME		"ls2084-shm"


/**
 * struct nxp_pfm_priv - platform specific functionalities
 *
 * @qdma_regs:		QDMA configuration registers pointer
 * @gpio_value:		GPIO pin value: 0/1
 */
struct nxp_pfm_priv {
	/* TODO: create memory mapped structure for QDMA regs */
	volatile u32* qdma_regs;

	int gpio_value;
};

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

void __iomem *nxp_pfm_alloc_local_shm(void *dev)
{
	request_mem_region(LS_PCI_SMEM, LS_PCI_SMEM_SIZE, SMEM_RES_NAME);
	return ioremap_cache(LS_PCI_SMEM, LS_PCI_SMEM_SIZE);
}

void nxp_pfm_free_local_shm(void *dev, void __iomem *addr)
{
	iounmap(addr);
	release_mem_region(LS_PCI_SMEM, LS_PCI_SMEM_SIZE);
}

/**
 * nxp_pdev_write_msg - write message to remote
 * @pfm:	platform object
 * @src_addr:	source virtual address
 * @dest_addr:	destination physical address
 * @size:	destination physical address
 *
 * Return:	0 on success, error code otherwise
 *
 * NOTE: src_addr and dest_addr must be 4 bytes aligned for QDMA.
 */
int nxp_pfm_dma_write(void *platform, void *src_addr, phys_addr_t dest_addr,
			u32 size)
{
	struct nxp_pfm_priv *priv;
	u32 status = 0;

	if (unlikely(!platform || !src_addr))
		return -EINVAL;

	priv = (struct nxp_pfm_priv *)platform;

	__dma_flush_range(src_addr, src_addr + size);

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

/* Send data available notification to remote peer */
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

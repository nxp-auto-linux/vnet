/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/cacheflush.h>

#include "nxp-pci.h"
#include "platform.h"

#define PCI_DEV_NAME "nxp-pci-dev"
#define MEM_RES_NAME PCI_DEV_NAME"-pcie"
#define BAR0 0

/* TODO: buffer size (and number?) should be configurable on pdev init */
#define MAX_NO_BUFFERS		512
#define MAX_BUFFER_SIZE		1536

/* LS2084 shared memory in DDR visible by pci endpoint */
/* TODO: read local shared mem addr from dts */
#define LS_PCI_SMEM		0x83A0000000ULL
#define LS_PCI_SMEM_SIZE	0x00400000	/* 4 MB */

/**
 * struct nxp_pci_shm - shared memory queue
 * @write_index:	write index (head)
 * @read_index:		read index (tail)
 * @pad:		padding for DMA memory alignment of data_buf
 * @data_buf:		data buffer
 */
/* TODO: test w/o volatile (may need memory barriers) */
/* TODO: try remove padding as 16-byte alignment might be enough */
struct nxp_pci_shm {
	volatile u64 write_index;
	volatile u64 read_index;
	volatile u64 pad[6];
	volatile u8 *data_buf;
}__packed;

/**
 * struct nxp_pdev_priv - NXP generic PCI device
 * @pci_dev:	PCI device structure
 * @upper_ops:	specific device built on nxp_pdev (i.e., net dev, tty dev, etc)
 * @local_shm:	local shared memory queue
 * @remote_shm:	remote shared memory queue
 * @spinlock:	used to sync access to shared memory
 * @platform:	platform specific object
 */
struct nxp_pdev_priv {
	struct pci_dev *pci_dev;
	struct nxp_pdev_upper_ops *upper_ops;

	struct nxp_pci_shm *local_shm;
	struct nxp_pci_shm *remote_shm;

	spinlock_t spinlock;

	void *platform;
};

static const struct pci_device_id pdev_ids[] = {
	{
		PCI_DEVICE(0x1957, 0x4001)
	},
	{0, }
};

MODULE_DEVICE_TABLE(pci, pdev_ids);

/**
 * nxp_pci_register_driver - register a new pci virtual device driver
 * @drv: the pci driver structure to register. Only probe and remove fields
 * 	 must be initialized.
 *
 * This is a wrapper over pci_register_driver() function and its purpose is to
 * hide the pci specifics (i.e. pci device id table) from the upper driver.
 */
int nxp_pci_register_driver(struct pci_driver *drv)
{
	if (drv->name == NULL)
		drv->name = PCI_DEV_NAME;
	drv->id_table = pdev_ids;
	return pci_register_driver(drv);
}

/**
 * pci_unregister_driver - unregister a pci driver
 * @drv: the driver structure to unregister
 *
 * This is a wrapper over pci_unregister_driver().
 */
void nxp_pci_unregister_driver(struct pci_driver *drv)
{
	pci_unregister_driver(drv);
}

static irqreturn_t nxp_pdev_rx_irq(int irq, void *dev_instance)
{
	struct pci_dev *pdev = (struct pci_dev *)dev_instance;
	struct nxp_pdev_priv *priv = pci_get_drvdata(pdev);

	if (priv->upper_ops->rx_irq_cb)
		priv->upper_ops->rx_irq_cb(priv->upper_ops->dev);

	return IRQ_HANDLED;
}

/**
 * nxp_pdev_init - unregister a pci driver
 * @drv: the driver structure to unregister
 * @upper_ops:
 *
 * This is a wrapper over pci_unregister_driver().
 */
int nxp_pdev_init(struct pci_dev *pdev, struct nxp_pdev_upper_ops *upper_ops)
{
	struct device *dev;
	struct nxp_pdev_priv *priv;
	u32 io_len;
	int err;

	if (!pdev || !upper_ops)
		return -EINVAL;

	dev = &pdev->dev;
	dev_dbg(dev, "Init PCI dev: vendor = %04x, dev = %04x\n", pdev->vendor,
		pdev->device);

	/* alloc private data struct */
	priv = kzalloc(sizeof(struct nxp_pdev_priv), GFP_KERNEL);
	if (!priv) {
		return -ENOMEM;
	}
	priv->pci_dev = pdev;
	priv->upper_ops = upper_ops;

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(dev, "Error enabling PCI device\n");
		goto err_free_priv_data;
	}

	err = pci_request_regions(pdev, PCI_DEV_NAME);
	if (err) {
		dev_err(dev, "Error requesting pci region\n");
		goto err_disable_pci;
	}

	/* read bar 0 length and flags */
	io_len = pci_resource_len(pdev, BAR0);
	dev_info(dev, "bar0 len = %x, flags = %lx\n", io_len,
	       pci_resource_flags(pdev, BAR0));
	if (!(pci_resource_flags(pdev, BAR0) & IORESOURCE_MEM)) {
		dev_err(dev, "Bad PCI resource\n");
		err = -ENODEV;
		goto err_pci_release_regions;
	}

	/* alloc PCI local shared memory */
	devm_request_mem_region(dev, LS_PCI_SMEM, LS_PCI_SMEM_SIZE,
				MEM_RES_NAME);
	priv->local_shm = (struct nxp_pci_shm *)ioremap_cache(LS_PCI_SMEM,
	                                                      LS_PCI_SMEM_SIZE);
	if (!priv->local_shm) {
		err = -ENOMEM;
		goto err_release_mem_region;
	}
	priv->local_shm->read_index = 0;
	priv->local_shm->write_index = 0;

	/* alloc PCI remote shared memory */
	priv->remote_shm = (struct nxp_pci_shm *)pci_iomap(pdev, 0, io_len);
	if (!priv->remote_shm) {
		err = -ENOMEM;
		goto err_unmap;
	}

	pci_set_master(pdev);

	err = pci_enable_msi(pdev);
	if (err) {
		dev_err(dev, "Error enabling PCI MSI\n");
		goto err_pci_unmap;
	}

	err = nxp_pfm_init(&priv->platform);
	if (err) {
		dev_err(dev, "Cannot map qdma registers\n");
		goto err_pci_disable_msi;
	}

	/* init rx interrupt */
	err = request_irq(pdev->irq, nxp_pdev_rx_irq, IRQF_SHARED,
			  PCI_DEV_NAME, pdev);
	if (err) {
		dev_err(&pdev->dev, "Request interrupt %d failed\n", pdev->irq);
		goto err_platform_free;
	}
	/* disable rx intr until upper dev is ready to receive data) */
	disable_irq(pdev->irq);

	spin_lock_init(&priv->spinlock);

	pci_set_drvdata(pdev, priv);

	dev_dbg(dev, "PCI dev init successfully. Device mapped at: %016llx\n",
		pdev->resource[0].start);
	return 0;

err_platform_free:
	nxp_pfm_free(priv->platform);
err_pci_disable_msi:
	pci_disable_msi(pdev);
err_pci_unmap:
	pci_iounmap(pdev, priv->remote_shm);
err_unmap:
	iounmap(priv->local_shm);
err_release_mem_region:
	devm_release_mem_region(dev, LS_PCI_SMEM, LS_PCI_SMEM_SIZE);
err_pci_release_regions:
	pci_release_regions(pdev);
err_disable_pci:
	pci_disable_device(pdev);
err_free_priv_data:
	kfree(priv);

	return err;
}

void nxp_pdev_free(struct pci_dev *pdev)
{
	struct nxp_pdev_priv *priv = pci_get_drvdata(pdev);

	free_irq(pdev->irq, pdev);
//	gpio_free(LS2S32V_INT_PIN);
	nxp_pfm_free(priv->platform);
	pci_disable_msi(pdev);
	pci_iounmap(pdev, priv->remote_shm);
	iounmap(priv->local_shm);
	devm_release_mem_region(&pdev->dev, LS_PCI_SMEM, LS_PCI_SMEM_SIZE);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	kfree(priv);
}

void *nxp_pdev_get_upper_dev(struct pci_dev *pdev)
{
	struct nxp_pdev_priv *priv = pci_get_drvdata(pdev);

	return priv->upper_ops->dev;
}

/**
 * nxp_pdev_write_msg - write message to remote
 * @pdev:	pci device
 * @msg:	message to be written
 *
 * Return:	0 on success, error code otherwise
 */
int nxp_pdev_write_msg(struct pci_dev *pdev, struct nxp_pdev_msg *msg)
{
	struct nxp_pdev_priv *priv;
	struct nxp_pci_shm *shm;
	phys_addr_t dest_addr;
	unsigned long flags;
	void *src_addr;
	u32 dma_size;
	int err = 0;

	if (!pdev || !msg)
		return -EINVAL;

	priv = pci_get_drvdata(pdev);
	shm = priv->remote_shm;

	if (msg->size > MAX_BUFFER_SIZE)
		return -EMSGSIZE;

	spin_lock_irqsave(&priv->spinlock, flags);
	/* check if queue is full */
	if (shm->write_index == shm->read_index + MAX_NO_BUFFERS) {
		dev_dbg(&pdev->dev, "queue full");
		spin_unlock_irqrestore(&priv->spinlock, flags);
		return -ENOBUFS;
	}

	src_addr = (void *)(msg->data - NXP_PCI_MSG_HEADROOM);
	dma_size = NXP_PCI_MSG_HEADROOM + msg->size;
	dest_addr = pdev->resource[0].start +
		offsetof(struct nxp_pci_shm, data_buf) +
		(shm->write_index & (MAX_NO_BUFFERS - 1)) * MAX_BUFFER_SIZE;

	/* insert in-band message length */
	*((u16 *)src_addr) = msg->size;	/* TODO: handle endianess */

	err = nxp_pfm_dma_write(priv->platform, src_addr, dest_addr, dma_size);
	if (err)
		goto err_unlock;

	shm->write_index++;

	/* Send data available notification to remote peer */
	nxp_pfm_trigger_remote_irq(priv->platform);
//	priv->gpio_level ^= 1;
//	gpio_set_value(LS2S32V_INT_PIN, priv->gpio_level);

err_unlock:
	spin_unlock_irqrestore(&priv->spinlock, flags);
	return 0;
}

int nxp_pdev_read_msg(struct pci_dev *pdev, struct nxp_pdev_msg *msg)
{
	struct nxp_pdev_priv *priv;
	struct nxp_pci_shm *shm;
	u8 *start;

	if (!pdev || !msg) {
		dev_dbg(&pdev->dev, "invalid arguments");
		return -EINVAL;
	}

	priv = pci_get_drvdata(pdev);
	shm = priv->local_shm;

	/* check if queue is empty */
	if (shm->read_index == shm->write_index) {
		dev_dbg(&pdev->dev, "queue empty");
		return -ENODATA;
	}

	/* TODO: optimize mem usage: use write/read offset instead of index */
	start = ((u8 *) &shm->data_buf) +
		(shm->read_index & (MAX_NO_BUFFERS - 1)) * MAX_BUFFER_SIZE;

	msg->size = *((u16 *)start);
	msg->data = start + NXP_PCI_MSG_HEADROOM;
	shm->read_index++;

	if (msg->size > MAX_BUFFER_SIZE) {
		dev_dbg(&pdev->dev, "rx message too large");
		return -EIO;
	}

	return 0;
}

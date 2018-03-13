/*
 * Copyright (C) 2018 NXP Semiconductors
 *
 * SPDX-License-Identifier:		BSD-3-Clause
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>

#include "nxp-pci.h"
#include "platform.h"

#define PCI_DEV_NAME "nxp-pci-vdev"
#define BAR_NUM 0

/* TODO: buffer size (and number?) should be configurable on pdev init */
#define MAX_BUFFERS_NUM		512
#define MAX_BUFFER_SIZE		1536

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
};

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

	int rx_irq_enable;

	void *platform;
};

/**
 * PCI device id array passed to pci_register_driver().
 * Add pci device ids here when adding support for new platform.
 */
static const struct pci_device_id pdev_ids[] = {
	{
		PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, 0x4000)
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, 0x4001)
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x7021)
	},
	{0, }
};
MODULE_DEVICE_TABLE(pci, pdev_ids);

/**
 * nxp_pci_register_driver - register a new pci virtual device driver
 * @drv: the pci driver structure to register. Only probe and remove fields
 * 	 must be initialized.
 *
 * Hide pci specifics (i.e. pci device id table) from the upper driver.
 */
int nxp_pci_register_driver(struct pci_driver *drv)
{
	if (drv->name == NULL)
		drv->name = PCI_DEV_NAME;
	drv->id_table = pdev_ids;
	return pci_register_driver(drv);
}

/**
 * nxp_pci_unregister_driver - unregister a pci driver
 * @drv: the driver structure to unregister
 */
void nxp_pci_unregister_driver(struct pci_driver *drv)
{
	pci_unregister_driver(drv);
}

/* pci dev interrupt handler */
static irqreturn_t nxp_pdev_rx_irq(int irq, void *dev_instance)
{
	struct pci_dev *pdev = (struct pci_dev *)dev_instance;
	struct nxp_pdev_priv *priv = pci_get_drvdata(pdev);

	if (priv->rx_irq_enable)
		priv->upper_ops->rx_notify_cb(priv->upper_ops->dev);

	return IRQ_HANDLED;
}

/**
 * nxp_pdev_init - initialize the pci device
 * @pdev:	pci device
 * @upper_ops:	pci upper dev operations (see struct nxp_pdev_upper_ops)
 *
 * Return:	0 on success, error code otherwise
 */
int nxp_pdev_init(struct pci_dev *pdev, struct nxp_pdev_upper_ops *upper_ops)
{
	struct device *dev;
	struct nxp_pdev_priv *priv;
	int err;

	if (!pdev || !upper_ops
		|| !upper_ops->rx_notify_cb || !upper_ops->tx_done_cb)
		return -EINVAL;

	dev = &pdev->dev;
	dev_dbg(dev, "Init PCI dev: vendor = %#06x, dev = %#06x\n",
		pdev->vendor, pdev->device);

	/* alloc private data struct */
	priv = kzalloc(sizeof(struct nxp_pdev_priv), GFP_KERNEL);
	if (!priv) {
		return -ENOMEM;
	}
	priv->pci_dev = pdev;
	priv->upper_ops = upper_ops;

	/* init platform specifics: DMA, remote kick interrupt */
	err = nxp_pfm_init(&priv->platform);
	if (err)
		goto err_free_priv_data;

	/* init PCI specifics */
	err = pci_enable_device(pdev);
	if (err) {
		dev_err(dev, "Error enabling PCI device\n");
		goto err_platform_free;
	}

	err = pci_request_regions(pdev, PCI_DEV_NAME);
	if (err) {
		dev_err(dev, "Error requesting pci region\n");
		goto err_disable_pci;
	}

	/* alloc local shared memory (platform dependent: PCI or DDR) */
	priv->local_shm = (struct nxp_pci_shm *)nxp_pfm_alloc_local_shm(
							priv->platform, pdev);
	if (!priv->local_shm) {
		err = -ENOMEM;
		goto err_pci_release_regions;
	}
	priv->local_shm->read_index = 0;
	priv->local_shm->write_index = 0;

	/* alloc remote shared memory (platform dependent: PCI or DDR) */
	priv->remote_shm = (struct nxp_pci_shm *)nxp_pfm_alloc_remote_shm(
							priv->platform, pdev);
	if (!priv->remote_shm) {
		err = -ENOMEM;
		goto err_free_local_shm;
	}

	pci_set_master(pdev);

	err = pci_enable_msi(pdev);
	if (err) {
		dev_err(dev, "Error enabling PCI MSI\n");
		goto err_free_remote_shm;
	}

	/* init rx interrupt */
	err = request_irq(pdev->irq, nxp_pdev_rx_irq, 0,
			  PCI_DEV_NAME, pdev);
	if (err) {
		dev_err(&pdev->dev, "Request interrupt %d failed\n", pdev->irq);
		goto err_pci_disable_msi;
	}

	spin_lock_init(&priv->spinlock);

	pci_set_drvdata(pdev, priv);

	dev_dbg(dev, "PCI dev init successfully. Device mapped at: %016llx\n",
		pdev->resource[0].start);
	return 0;

err_pci_disable_msi:
	pci_disable_msi(pdev);
err_free_remote_shm:
	nxp_pfm_free_remote_shm(priv->platform, pdev, priv->remote_shm);
err_free_local_shm:
	nxp_pfm_free_local_shm(priv->platform, pdev, priv->local_shm);
err_pci_release_regions:
	pci_release_regions(pdev);
err_disable_pci:
	pci_disable_device(pdev);
err_platform_free:
	nxp_pfm_free(priv->platform);
err_free_priv_data:
	kfree(priv);

	return err;
}

/**
 * nxp_pdev_free - release the pci device
 * @pdev:	pci device
 */
void nxp_pdev_free(struct pci_dev *pdev)
{
	struct nxp_pdev_priv *priv = pci_get_drvdata(pdev);

	free_irq(pdev->irq, pdev);
	pci_disable_msi(pdev);
	nxp_pfm_free_remote_shm(priv->platform, pdev, priv->remote_shm);
	nxp_pfm_free_local_shm(priv->platform, pdev, priv->local_shm);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	nxp_pfm_free(priv->platform);
	kfree(priv);
}

/**
 * nxp_pdev_get_upper_dev - return upper dev pointer/cookie
 * @pdev:	pci device
 *
 * Return:	upper dev pointer
 */
void *nxp_pdev_get_upper_dev(struct pci_dev *pdev)
{
	struct nxp_pdev_priv *priv = pci_get_drvdata(pdev);

	return priv->upper_ops->dev;
}

/**
 * nxp_pdev_write_msg - write data to remote
 * @pdev:	pci device
 * @buf:	buffer to be written
 * @size:	size of buffer to be written
 * @tx_done_arg: argument used for tx done callback notification
 *
 * Return:	0 on success, error code otherwise
 */
int nxp_pdev_write(struct pci_dev *pdev, void *buf, u16 size,
			void *tx_done_arg)
{
	struct nxp_pdev_priv *priv;
	volatile struct nxp_pci_shm *shm;
	phys_addr_t dest_addr;
	unsigned long flags;
	void *src_addr;
	u32 dma_size;
	int err = 0;

	if (!pdev || !buf)
		return -EINVAL;

	priv = pci_get_drvdata(pdev);
	shm = priv->remote_shm;

	if (size > MAX_BUFFER_SIZE)
		return -EMSGSIZE;

	src_addr = buf - NXP_PCI_MSG_HEADROOM;
	dma_size = NXP_PCI_MSG_HEADROOM + size;

	/* insert in-band message length */
	*((u16 *)src_addr) = size;	/* TODO: handle endianess */

	/* lock shared mem queue access */
	spin_lock_irqsave(&priv->spinlock, flags);

	/* check if queue is full */
	if (shm->write_index == shm->read_index + MAX_BUFFERS_NUM) {
		dev_dbg(&pdev->dev, "queue full");
		spin_unlock_irqrestore(&priv->spinlock, flags);
		return -ENOBUFS;
	}

	dest_addr = pdev->resource[0].start +
		offsetof(struct nxp_pci_shm, data_buf) +
		(shm->write_index & (MAX_BUFFERS_NUM - 1)) * MAX_BUFFER_SIZE;

	err = nxp_pfm_dma_write(priv->platform, src_addr, dest_addr, dma_size);
	if (err)
		goto err_unlock;

	shm->write_index++;

	spin_unlock_irqrestore(&priv->spinlock, flags);

	/* Send data available notification to remote peer */
	nxp_pfm_trigger_remote_irq(priv->platform);

	/* Notify caller that write operation is completed.
	 * TODO: move this notification in DMA done notification from platform*/
	priv->upper_ops->tx_done_cb(priv->upper_ops->dev, tx_done_arg);

	return 0;

err_unlock:
	spin_unlock_irqrestore(&priv->spinlock, flags);
	return err;
}

/**
 * nxp_pdev_read_msg - read data from remote
 * @pdev:	pci device
 * @buf:	pointer to the buffer received
 * @size:	received size
 *
 * Return:	0 on success, error code otherwise
 */
int nxp_pdev_read(struct pci_dev *pdev, void **buf, u16 *size)
{
	struct nxp_pdev_priv *priv;
	volatile struct nxp_pci_shm *shm;
	u8 *start;

	if (!pdev || !buf || !size) {
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
		(shm->read_index & (MAX_BUFFERS_NUM - 1)) * MAX_BUFFER_SIZE;

	*size = *((u16 *)start);
	*buf = start + NXP_PCI_MSG_HEADROOM;
	shm->read_index++;

	if (unlikely(*size > MAX_BUFFER_SIZE)) {
		dev_dbg(&pdev->dev, "rx message too large");
		return -EIO;
	}

	return 0;
}

/**
 * nxp_pdev_enable_rx_irq - disable rx interrupt
 * @pdev:	pci device
 */
void nxp_pdev_disable_rx_irq(struct pci_dev *pdev)
{
	struct nxp_pdev_priv *priv = pci_get_drvdata(pdev);

	priv->rx_irq_enable = 0;

	/* TODO: The right implementation would be to tell the remote device
	 * to stop sending interrupts, through a control flag in the shared
	 * memory. */
}

/**
 * nxp_pdev_enable_rx_irq - enable rx interrupt
 * @pdev:	pci device
 */
void nxp_pdev_enable_rx_irq(struct pci_dev *pdev)
{
	struct nxp_pdev_priv *priv = pci_get_drvdata(pdev);

	priv->rx_irq_enable = 1;

	/* TODO: The right implementation would be to tell the remote device
	 * to start sending interrupts, through a control flag in the shared
	 * memory. */
}

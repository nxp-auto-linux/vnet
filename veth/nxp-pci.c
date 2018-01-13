/*
 * Copyright 2017 NXP
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

#include "nxp-pci.h"

#define PCI_RES_NAME "nxp-pci"
#define BAR0 0

static const struct pci_device_id fpx_ids[] = {
	{
		PCI_DEVICE(0x1957, 0x4001)
	},
	{0, }
};

MODULE_DEVICE_TABLE(pci, fpx_ids);

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
		drv->name = "NXP PCI virtual device driver";
	drv->id_table = fpx_ids;
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

/**
 * nxp_pdev_init - unregister a pci driver
 * @drv: the driver structure to unregister
 *
 * This is a wrapper over pci_unregister_driver().
 */
int nxp_pdev_init(struct pci_dev *pdev, void *upper_dev)
{
	struct device *dev = &pdev->dev;
	struct nxp_pdev_priv *priv;
	u32 io_len;
	int err;

	/* alloc private data struct */
	priv = kzalloc(sizeof(struct nxp_pdev_priv), GFP_KERNEL);
	if (!priv) {
		return -ENOMEM;
	}
	priv->pci_dev = pdev;

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(dev, "Error enabling PCI device\n");
		goto err_free_priv_data;
	}

	err = pci_request_regions(pdev, PCI_RES_NAME);
	if (err) {
		dev_err(dev, "Error requesting pci region\n");
		goto err_disable_pci;
	}

	/* read bar 0 length and flags */
	io_len = pci_resource_len(pdev, BAR0);
	dev_err(dev, "bar0 len = %u, flags = %lx\n", io_len,
	       pci_resource_flags(pdev, BAR0)); /* TODO: remove after debug */
	if (!(pci_resource_flags(pdev, BAR0) & IORESOURCE_MEM)) {
		dev_err(dev, "Bad PCI resource\n");
		err = -ENODEV;
		goto err_pci_release_regions;
	}

	/* alloc PCI local shared memory */
	devm_request_mem_region(dev, LS_PCI_SMEM, LS_PCI_SMEM_SIZE,
	                        "nxp-pci-local-shm");
	priv->local_shm = (struct nxp_pci_shm *)ioremap_cache(LS_PCI_SMEM,
	                                                      LS_PCI_SMEM_SIZE);
	if (!priv->local_shm) {
		err = -ENOMEM;
		goto err_release_mem_region;
	}

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

	/* init qdma for tx */ /* TODO: try remove cast from all ioremap */
	priv->qdma_regs = (u32*)ioremap_nocache(QDMA_BASE, QDMA_REG_SIZE);
	if (!priv->qdma_regs) {
		dev_err(dev, "Cannot map qdma registers\n");
		err = -ENOMEM;
		goto err_pci_disable_msi;
	}

	/* init tx gpio signaling interrupt */
	priv->gpio_level = 0;
	err = gpio_request(LS2S32V_INT_PIN, "LS2_S32V_INT");
	if (err) {
		dev_err(dev, "Cannot reserve GPIO LS2S32V_INT_PIN\n");
		err = -ENODEV;
		goto err_qdma_unmap;
	}
	err = gpio_direction_output(LS2S32V_INT_PIN, 0);
	if (err) {
		dev_err(dev, "Cannot configure GPIO LS2S32V_INT_PIN as output\n");
		err = -ENODEV;
		goto err_gpio_free;
	}

	spin_lock_init(&priv->spinlock);

	priv->upper_dev = upper_dev;
	pci_set_drvdata(pdev, priv);
	dev_dbg(dev, "PCI dev init success. Device mapped at: %016llx\n",
		pdev->resource[0].start);

	return 0;

err_gpio_free:
	gpio_free(LS2S32V_INT_PIN);
err_qdma_unmap:
	iounmap(priv->qdma_regs);
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

	gpio_free(LS2S32V_INT_PIN);
	iounmap(priv->qdma_regs);
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

	return priv->upper_dev;
}

/**
 * nxp_pci_dev_register_rx_cb() - allocate an interrupt line
 * @pdev:	Interrupt line to allocate
 * @handler:	Function to be called when the rx IRQ occurs.
 * @dev_id:	A cookie passed back to the handler function
 */
int nxp_pdev_register_rx_cb(struct pci_dev *pdev, irq_handler_t handler,
			       void *arg)
{
	int err;

	/* init rx interrupt */
	err = request_irq(pdev->irq, handler, IRQF_SHARED,
			  dev_name(&pdev->dev), arg);
	if (err)
		dev_err(&pdev->dev, "failed to register interrupt %d\n",
			pdev->irq);

	return err;
}

void nxp_pdev_unregister_rx_cb(struct pci_dev *pdev, void *dev_id)
{
	free_irq(pdev->irq, dev_id);
}

int nxp_pdev_write_msg(struct pci_dev *pdev, struct nxp_pdev_msg *msg)
{
	struct nxp_pdev_priv *priv;
	struct nxp_pci_shm *shm;
	phys_addr_t shm_paddr;
	unsigned long flags;
	void *start, *end;
	u32 status = 0;

	if (!pdev || !msg)
		return -EINVAL;

	priv = pci_get_drvdata(pdev);
	shm = priv->remote_shm;

	if (msg.size > MAX_BUFFER_SIZE)
		return -EMSGSIZE;

	spin_lock_irqsave(&priv->spinlock, flags);
	/* check if queue is full */
	if (shm->write_index == shm->read_index + MAX_NO_BUFFERS) {
		spin_unlock_irqrestore(&priv->spinlock, flags);
		return -ENOBUFS;
	}

	start = (void *)msg;
	end = start + NXP_PCI_TX_BUF_HEADROOM + msg->size;

	__dma_flush_range((const void *)start, (const void *)end);

	/* TODO: cleanup code; move qdma stuff to a separate function? */
	*priv->qdma_regs &= ~1;

	status = *(priv->qdma_regs + 1) & 0x92;
	if (status)
		*(priv->qdma_regs + 1) = status;

	/* TODO: Ask Radu why & 0xffff */
	*(priv->qdma_regs + 4) = upper_32_bits(virt_to_phys(start)) & 0xffff;
	*(priv->qdma_regs + 5) = lower_32_bits(virt_to_phys(start));

	shm_paddr = priv->pci_dev->resource[0].start;

	/* TODO: use upper/lower_32_bits */
	*(priv->qdma_regs + 6) = (u32)(shm_paddr >> 32);
	*(priv->qdma_regs + 7) = (u32)(shm_paddr +
		offsetof(struct nxp_pci_shm, data) +
		(shm->write_index & (MAX_NO_BUFFERS - 1)) * MAX_BUFFER_SIZE);
	*(priv->qdma_regs + 8) = msg.size + NXP_PCI_TX_BUF_HEADROOM;

	/* TODO: add retry counter or timeout */
	while (1) {
		/* TODO: should this be |= ? */
		*priv->qdma_regs = 1;

		do {
			status = *(priv->qdma_regs + 1);
		} while (status & (1 << 2));

		if (!(status & (1 << 7)))
			break;

		/* TODO: we don't need this? */
		*(priv->qdma_regs + 1) = status;
		*priv->qdma_regs &= ~1;
		// redo tx
	}

	shm->write_index++;

	/* Send data available notification to remote peer */
	priv->gpio_level ^= 1;
	gpio_set_value(LS2S32V_INT_PIN, priv->gpio_level);

	spin_unlock_irqrestore(&priv->spinlock, flags);

	return 0;
}

int nxp_pdev_read_msg(struct pci_dev *pdev, struct nxp_pdev_msg *msg)
{
	struct nxp_pdev_priv *priv;
	struct nxp_pci_shm *shm;

	if (!pdev || !msg)
		return -EINVAL;

	priv = pci_get_drvdata(pdev);
	shm = priv->local_shm;

	/* check if queue is empty */
	if (shm->read_index == shm->write_index) {
		dev_dbg(&pdev->dev, "queue empty");
		return -ENODATA;
	}

	/* TODO: optimize mem usage: use write/read offset instead of index */
	msg = (struct nxp_pdev_msg *) (shm->data +
		(shm->read_index & (MAX_NO_BUFFERS - 1)) * MAX_BUFFER_SIZE);

	if (msg->size > MAX_BUFFER_SIZE) {
		dev_dbg(&pdev->dev, "rx message too large");
		return -EIO;
	}

	shm->read_index++;

	return 0;
}

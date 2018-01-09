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
int nxp_pci_dev_register_rx_cb(struct pci_dev *pdev, irq_handler_t handler,
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

void nxp_pci_dev_unregister_rx_cb(struct pci_dev *pdev, void *dev_id)
{
	free_irq(pdev->irq, dev_id);
}

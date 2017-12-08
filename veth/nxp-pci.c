/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/slab.h>

#include "nxp-pci.h"

#define PCI_RES_NAME "nxp-pci"
#define BAR0 0

int nxp_pci_dev_init(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct nxp_pci_priv *priv;
	u32 io_len;
	int err;

	/* alloc private data struct */
	priv = kzalloc(sizeof(struct nxp_pci_priv), GFP_KERNEL);
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
	err = pci_resource_flags(pdev, BAR0);
	dev_err(dev, "bar0 len = %lu, flags = %08x\n", io_len,
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

	pci_set_drvdata(pdev, priv);

	return 0;

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

void nxp_pci_dev_free(struct pci_dev *pdev)
{

}

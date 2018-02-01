/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef DRIVERS_NET_VPCIE_VPCIE_VPCIE_H_
#define DRIVERS_NET_VPCIE_PCIE_VPCIE_H_

/* head room used to store in-band data length */
#define NXP_PCI_TX_BUF_HEADROOM 2

/**
 * struct nxp_pdev_msg - pci device message
 *
 * @data:	message buffer pointer
 * @size:	message size
 *
 * Used by read/write API to package data buffer and its size.
 */
struct nxp_pdev_msg {
	u8 *data;
	u16 size;
};

/**
 * struct nxp_pdev_upper_ops - pci upper dev operations
 *
 * @dev:	pointer to upper device
 * @rx_irq_cb:	receive notification callback
 * @tx_done_cb:	transmit completed notification callback
 */
struct nxp_pdev_upper_ops {
	void *dev;
	void (*rx_irq_cb)(void *dev);
	void (*tx_done_cb)(void *dev);
};

int nxp_pci_register_driver(struct pci_driver *drv);
void nxp_pci_unregister_driver(struct pci_driver *drv);

int nxp_pdev_init(struct pci_dev *pdev, struct nxp_pdev_upper_ops *upper_ops);
void nxp_pdev_free(struct pci_dev *pdev);
void *nxp_pdev_get_upper_dev(struct pci_dev *pdev);

int nxp_pdev_write_msg(struct pci_dev *pdev, struct nxp_pdev_msg *msg);
int nxp_pdev_read_msg(struct pci_dev *pdev, struct nxp_pdev_msg *msg);

#endif /* DRIVERS_NET_VPCIE_VPCIE_VPCIE_H_ */

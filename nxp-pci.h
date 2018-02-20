/*
 * Copyright (C) 2018 NXP Semiconductors
 *
 * SPDX-License-Identifier:		BSD-3-Clause
 */
#ifndef DRIVERS_NET_VPCIE_VPCIE_VPCIE_H_
#define DRIVERS_NET_VPCIE_PCIE_VPCIE_H_

/* head room used to insert in-band data length */
#define NXP_PCI_MSG_HEADROOM 2

/**
 * struct nxp_pdev_msg - pci device message
 *
 * @data:	message buffer pointer
 * @size:	message size
 * @tx_done_arg: argument used for tx done callback notification
 *
 * Used by read/write API to package data buffer and its size.
 * An example for using tx_done_arg is when an upper net dev wants to know
 * which skb to release when tx is done.
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
	void (*tx_done_cb)(void *dev, void *arg);
};

int nxp_pci_register_driver(struct pci_driver *drv);
void nxp_pci_unregister_driver(struct pci_driver *drv);

int nxp_pdev_init(struct pci_dev *pdev, struct nxp_pdev_upper_ops *upper_ops);
void nxp_pdev_free(struct pci_dev *pdev);
void *nxp_pdev_get_upper_dev(struct pci_dev *pdev);

int nxp_pdev_write_msg(struct pci_dev *pdev, struct nxp_pdev_msg *msg,
			void *tx_done_arg);
int nxp_pdev_read_msg(struct pci_dev *pdev, struct nxp_pdev_msg *msg);

void nxp_pdev_disable_rx_irq(struct pci_dev *pdev);
void nxp_pdev_enable_rx_irq(struct pci_dev *pdev);

#endif /* DRIVERS_NET_VPCIE_VPCIE_VPCIE_H_ */

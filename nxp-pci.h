/*
 * Copyright (C) 2018 NXP Semiconductors
 *
 * SPDX-License-Identifier:		BSD-3-Clause
 */
#ifndef DRIVERS_NET_VPCIE_VPCIE_VPCIE_H_
#define DRIVERS_NET_VPCIE_PCIE_VPCIE_H_

/* head-room used to insert in-band data length in front of the message */
/* TODO: some DMA controllers (QDMA) requires addresses to be 4-byte aligned.
 * This head-room works for now because the network stack aligns IP headers at
 * 16 bytes, so eth frame (skb->data) is 2-byte aligned. This plus the 2-byte
 * in-band length inserted in the head-room makes the DMA addr 4-byte aligned.
 *
 * The proper implementation for other upper devices (e.g. tty) is to reserve
 * a larger head-room, left-align the address to be DMA-ed and insert also the
 * data offset in the message after in-band length.
 */
#define NXP_PCI_MSG_HEADROOM 2

/**
 * struct nxp_pdev_upper_ops - pci upper dev operations
 *
 * @dev:	  upper device pointer/cookie
 * @rx_notify_cb: receive notification callback called from rx interrupt
 * @tx_done_cb:	  transmit completed notification callback
 */
struct nxp_pdev_upper_ops {
	void *dev;
	void (*rx_notify_cb)(void *dev);
	void (*tx_done_cb)(void *dev, void *arg);
};

int nxp_pci_register_driver(struct pci_driver *drv);
void nxp_pci_unregister_driver(struct pci_driver *drv);

int nxp_pdev_init(struct pci_dev *pdev, struct nxp_pdev_upper_ops *upper_ops);
void nxp_pdev_free(struct pci_dev *pdev);
void *nxp_pdev_get_upper_dev(struct pci_dev *pdev);

int nxp_pdev_write(struct pci_dev *pdev, void *buf, u16 size,
			void *tx_done_arg);
int nxp_pdev_read(struct pci_dev *pdev, void **buf, u16 *size);

void nxp_pdev_disable_rx_irq(struct pci_dev *pdev);
void nxp_pdev_enable_rx_irq(struct pci_dev *pdev);

#endif /* DRIVERS_NET_VPCIE_VPCIE_VPCIE_H_ */

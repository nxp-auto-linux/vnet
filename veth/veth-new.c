/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/pci.h>
#include "veth.h"
#include "nxp-pci.h"

#define DRIVER_NAME "fpx"

static netdev_tx_t fpx_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct fpx_enet_private *fep = netdev_priv(ndev);
	struct nxp_pdev_msg msg;
	int err;

	if (unlikely(skb_headroom(skb) < NXP_PCI_TX_BUF_HEADROOM)) {
		netdev_err(ndev, "Not enough head room\n");
		goto err_out;
	}

	msg.size = skb->len;
	msg.data = skb->data;
	err = nxp_pdev_write_msg(fep->pci_dev, &msg);
	if (err) {
		ndev->stats.tx_dropped++;
		goto err_out;
	}

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;

err_out:
	dev_kfree_skb_any(skb);

	return NETDEV_TX_OK;
}

static int fpx_enet_rx_napi(struct napi_struct *napi, int budget)
{
	struct net_device *ndev = napi->dev;
	struct fpx_enet_private *fep = netdev_priv(ndev);
	int pkts = 0;
	struct sk_buff *skb;
	struct nxp_pdev_msg msg;
	int err = 0;

	do {
		err = nxp_pdev_read_msg(fep->pci_dev, &msg);
		if (err) {
			if (err != -ENODATA)
				ndev->stats.rx_errors++;
			continue;
		}

		skb = napi_alloc_skb(napi, msg.size);
		if (unlikely(!skb)) {
			ndev->stats.rx_dropped++;
			continue;
		}

		/* TODO: check small performance improvement
		 * update skb alloc size and buf_len check */
		//skb_reserve(skb, NET_IP_ALIGN);

		/* do memcpy */
		skb_put(skb, msg.size);

		memcpy(skb->data, msg.data, msg.size);
		skb->protocol = eth_type_trans(skb, ndev);

		/* TODO: check small performance improvement */
		//skb->ip_summed = CHECKSUM_UNNECESSARY;

		/* update RX stats */
		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += msg.size;

		netif_receive_skb(skb);

		/* TODO: check small performance improvement */
		//napi_gro_receive(napi, skb);
	} while (err != -ENODATA && ++pkts < budget);

	if (pkts < budget) {
		napi_complete(napi);
		/* TODO: re-enable interrupt */
	}
	return pkts;
}

static irqreturn_t fpx_interrupt(int irq, void *dev_instance)
{
	struct net_device *ndev = (struct net_device *) dev_instance;
	struct fpx_enet_private *fep = netdev_priv(ndev);

	/* TODO: disable interrupt before napi schedule*/
	//disable_irq_nosync(fep->pci_dev->irq);
	napi_schedule(&fep->napi);

	return IRQ_HANDLED;
}

static int fpx_open(struct net_device *ndev)
{
	struct fpx_enet_private *fep = netdev_priv(ndev);

	napi_enable(&fep->napi);
	enable_irq(fep->pci_dev->irq);
	netif_tx_start_all_queues(ndev);

	netdev_info(ndev, "Interface %s up\n", ndev->name);
	return 0;
}

static int fpx_close(struct net_device *ndev)
{
	struct fpx_enet_private *fep = netdev_priv(ndev);

	disable_irq(fep->pci_dev->irq);
	/* wait for all pending rx frames to be processed by napi */
	msleep(500);

	napi_disable(&fep->napi);
	netif_tx_disable(ndev);

	netdev_info(ndev, "Interface %s down\n", ndev->name);
	return 0;
}

static const struct net_device_ops fpx_netdev_ops = {
	.ndo_open		= fpx_open,
	.ndo_stop		= fpx_close,
	.ndo_start_xmit		= fpx_start_xmit,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
};

static int fpx_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct net_device *ndev = NULL;
	struct fpx_enet_private *fep = NULL;
	int err = 0;

	ndev = alloc_netdev(sizeof(struct fpx_enet_private), "fpx%d",
				NET_NAME_ENUM, ether_setup);
	if (!ndev) {
		dev_err(&pdev->dev, "Error alloc_netdev.\n");
		return -ENOMEM;
	}

	ndev->flags = 0;
	ndev->priv_flags &= ~IFF_TX_SKB_SHARING;
	ndev->netdev_ops = &fpx_netdev_ops;
	ndev->needed_headroom = NXP_PCI_TX_BUF_HEADROOM;

	ndev->dev_addr[0] = 0x88;
	ndev->dev_addr[1] = 0x01;
	ndev->dev_addr[2] = 0x02;
	ndev->dev_addr[3] = 0x03;
	ndev->dev_addr[4] = 0x04;
	ndev->dev_addr[5] = 0x66;

	SET_NETDEV_DEV(ndev, &pdev->dev);

	fep = netdev_priv(ndev);
	fep->netdev = ndev;
	fep->pci_dev = pdev;

	err = nxp_pdev_init(pdev, ndev);
	if (err) {
		goto err_free_netdev;
	}

	/* init rx napi */
	netif_napi_add(ndev, &fep->napi, fpx_enet_rx_napi, NAPI_POLL_WEIGHT);
	/* init rx interrupt */
	err = request_irq(pdev->irq, fpx_interrupt, IRQF_SHARED, ndev->name, ndev);
	if (err) {
		dev_err(&pdev->dev, "Failed registering interrupt %d\n", pdev->irq);
		goto err_napi_del;
	}
	/* disable rx intr until interface is up (ndo_open) */
	disable_irq(fep->pci_dev->irq);

	err = register_netdev(ndev);
	if (err) {
		dev_err(&pdev->dev, "Error registering netdevice\n");
		goto err_free_irq;
	}

	netdev_info(ndev, "Interface %s probed successfully\n", ndev->name);
	return 0;

err_free_irq:
	free_irq(pdev->irq, ndev);
err_napi_del:
	netif_napi_del(&fep->napi);
	nxp_pdev_free(pdev);
err_free_netdev:
	free_netdev(ndev);
	return err;
}

static void fpx_remove(struct pci_dev *pdev)
{
	struct net_device *ndev = nxp_pdev_get_upper_dev(pdev);
	struct fpx_enet_private *fep = netdev_priv(ndev);

	if (ndev == NULL) {
		dev_err(&pdev->dev, "Remove fpx device failed.\n");
		return;
	}

	netdev_info(ndev, "Interface %s removed successfully\n", ndev->name);

	unregister_netdev(ndev);
	free_irq(pdev->irq, ndev);
	netif_napi_del(&fep->napi);

	nxp_pdev_free(pdev);

	free_netdev(ndev);
}

static struct pci_driver fpx_driver = {
	.probe = fpx_probe,
	.remove = fpx_remove,
};

static int __init fpx_init(void)
{
	pr_info("driver init - v0.2\n");
	return nxp_pci_register_driver(&fpx_driver);
}

static void __exit fpx_exit(void)
{
	pr_info("driver exit\n");
	nxp_pci_unregister_driver(&fpx_driver);
}

module_init(fpx_init);
module_exit(fpx_exit);

MODULE_ALIAS("platform:"DRIVER_NAME);
MODULE_LICENSE("GPL");

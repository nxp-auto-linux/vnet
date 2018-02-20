/*
 * Copyright (C) 2018 NXP Semiconductors
 *
 * SPDX-License-Identifier:		BSD-3-Clause
 */
#include <linux/module.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/pci.h>
#include "nxp-pci.h"

#define DRIVER_NAME "nxp-veth"
#define DEVICE_NAME "fpx"

MODULE_AUTHOR("NXP");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:"DRIVER_NAME);

struct veth_ndev_priv {
	struct pci_dev *pci_dev;
	struct napi_struct napi;
};

static netdev_tx_t veth_start_tx(struct sk_buff *skb, struct net_device *ndev)
{
	struct veth_ndev_priv *priv = netdev_priv(ndev);
	struct nxp_pdev_msg msg;
	int err;

	msg.data = skb->data;
	msg.size = skb->len;
	err = nxp_pdev_write_msg(priv->pci_dev, &msg, skb);
	if (err) {
		ndev->stats.tx_dropped++;
		goto err_out;
	}

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += msg.size;

	return NETDEV_TX_OK;

err_out:
	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}

static void veth_tx_done(void *dev, void *arg)
{
	struct sk_buff *skb = (struct sk_buff *)arg;

	dev_kfree_skb_any(skb);
}

static int veth_rx_napi(struct napi_struct *napi, int budget)
{
	struct net_device *ndev = napi->dev;
	struct veth_ndev_priv *priv = netdev_priv(ndev);
	struct sk_buff *skb;
	struct nxp_pdev_msg msg;
	int pkts = 0;
	int err = 0;

	do {
		err = nxp_pdev_read_msg(priv->pci_dev, &msg);
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

		/* TODO: check perf impr: reserve NET_IP_ALIGN
		 * must update skb alloc size and buf_len check too */
		//skb_reserve(skb, NET_IP_ALIGN);

		/* copy data in skb */
		skb_put(skb, msg.size);
		memcpy(skb->data, msg.data, msg.size);

		skb->protocol = eth_type_trans(skb, ndev);

		/* checksum not necessary as PCIe already handles this */
		skb->ip_summed = CHECKSUM_UNNECESSARY;

		/* update RX stats */
		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += msg.size;

		netif_receive_skb(skb);

		/* TODO: check perf impr: replace netif_receive_skb with: */
		//napi_gro_receive(napi, skb);
	} while (err != -ENODATA && ++pkts < budget);

	if (pkts < budget) {
		napi_complete(napi);
		/* re-enable rx interrupt */
		nxp_pdev_enable_rx_irq(priv->pci_dev);
	}

	return pkts;
}

static void veth_rx_irq(void *dev)
{
	struct net_device *ndev = (struct net_device *)dev;
	struct veth_ndev_priv *priv = netdev_priv(ndev);

	/* disable rx interrupt before napi schedule */
	nxp_pdev_disable_rx_irq(priv->pci_dev);

	napi_schedule(&priv->napi);
}

static int veth_open(struct net_device *ndev)
{
	struct veth_ndev_priv *priv = netdev_priv(ndev);

	napi_enable(&priv->napi);
	nxp_pdev_enable_rx_irq(priv->pci_dev);
	netif_tx_start_all_queues(ndev);

	netdev_info(ndev, "interface is up\n");
	return 0;
}

static int veth_close(struct net_device *ndev)
{
	struct veth_ndev_priv *priv = netdev_priv(ndev);

	nxp_pdev_disable_rx_irq(priv->pci_dev);
	/* wait for all pending rx frames to be processed by napi */
	msleep(500);

	napi_disable(&priv->napi);
	netif_tx_disable(ndev);

	netdev_info(ndev, "interface is down\n");
	return 0;
}

static const struct net_device_ops netdev_ops = {
	.ndo_open		= veth_open,
	.ndo_stop		= veth_close,
	.ndo_start_xmit		= veth_start_tx,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
};

static struct nxp_pdev_upper_ops pci_ops = {
	.rx_irq_cb = veth_rx_irq,
	.tx_done_cb = veth_tx_done,
};

static int veth_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct net_device *ndev = NULL;
	struct veth_ndev_priv *priv = NULL;
	int err = 0;

	ndev = alloc_netdev(sizeof(struct veth_ndev_priv), DEVICE_NAME"%d",
				NET_NAME_ENUM, ether_setup);
	if (!ndev) {
		dev_err(&pdev->dev, "Error alloc_netdev.\n");
		return -ENOMEM;
	}

	ndev->flags = 0;
	ndev->priv_flags &= ~IFF_TX_SKB_SHARING;
	ndev->netdev_ops = &netdev_ops;

	/* TODO: QDMA requires addresses to be 4-byte aligned. This works now
	 * because the network stack aligns IP headers at 16 bytes, so eth
	 * frame (skb->data) is 2-byte aligned. This plus the 2-byte in-band
	 * length inserted in the head-room makes the DMA addr 4-byte aligned.
	 *
	 * The proper impl is to reserve a larger headroom, left-align the
	 * address to be DMA-ed and insert also the data offset in the
	 * message after the in-band length.
	 */
	ndev->needed_headroom = NXP_PCI_MSG_HEADROOM;

	ndev->dev_addr[0] = 0x88;
	ndev->dev_addr[1] = 0x01;
	ndev->dev_addr[2] = 0x02;
	ndev->dev_addr[3] = 0x03;
	ndev->dev_addr[4] = 0x04;
	ndev->dev_addr[5] = 0x66;

	SET_NETDEV_DEV(ndev, &pdev->dev);

	priv = netdev_priv(ndev);
	priv->pci_dev = pdev;

	pci_ops.dev = ndev;
	err = nxp_pdev_init(pdev, &pci_ops);
	if (err) {
		goto err_free_netdev;
	}

	/* init rx napi */
	netif_napi_add(ndev, &priv->napi, veth_rx_napi, NAPI_POLL_WEIGHT);

	err = register_netdev(ndev);
	if (err) {
		dev_err(&pdev->dev, "Error registering netdevice\n");
		goto err_napi_del;
	}

	netdev_info(ndev, "interface probed successfully\n");
	return 0;

err_napi_del:
	netif_napi_del(&priv->napi);
	nxp_pdev_free(pdev);
err_free_netdev:
	free_netdev(ndev);
	return err;
}

static void veth_remove(struct pci_dev *pdev)
{
	struct net_device *ndev = nxp_pdev_get_upper_dev(pdev);
	struct veth_ndev_priv *priv = netdev_priv(ndev);

	if (ndev == NULL) {
		dev_err(&pdev->dev, "Remove veth device failed.\n");
		return;
	}

	netdev_info(ndev, "interface removed successfully\n");

	unregister_netdev(ndev);
	netif_napi_del(&priv->napi);

	nxp_pdev_free(pdev);

	free_netdev(ndev);
}

static struct pci_driver veth_driver = {
	.name = DRIVER_NAME,
	.probe = veth_probe,
	.remove = veth_remove,
};

static int __init veth_init(void)
{
	pr_info("driver init - v0.29\n");
	return nxp_pci_register_driver(&veth_driver);
}

static void __exit veth_exit(void)
{
	pr_info("driver exit\n");
	nxp_pci_unregister_driver(&veth_driver);
}

module_init(veth_init);
module_exit(veth_exit);

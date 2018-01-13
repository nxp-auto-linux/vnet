/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/pci.h>
#include <asm/cacheflush.h>
#include <linux/gpio.h>
#include "veth.h"
#include "nxp-pci.h"

#define DRIVER_NAME "fpx"

extern void __dma_flush_range(const void *, const void *);

//static netdev_tx_t fpx_start_xmit(struct sk_buff *skb, struct net_device *ndev)
//{
//	struct fpx_enet_private *fep;
//	u64 write_i, read_i;
//	unsigned long flags;
//
//	fep = netdev_priv(ndev);
//
//	spin_lock_irqsave(&fep->spinlock, flags);
//	write_i = fep->ctrl_ved_r->current_write_index;
//	read_i = fep->ctrl_ved_r->current_read_index;
//
//	if (write_i > (read_i + MAX_NO_BUFFERS)) {
//		/* discard frame */
//		ndev->stats.tx_dropped ++;
//	} else {
//		u32 status = 0;
//		void* start = (skb->data - sizeof(u16));
//		void* end = start + skb->len + sizeof(u16);
//
//		if (unlikely(skb->len > U16_MAX)) {
//			netdev_err(ndev,
//			        "Frame length exceeding 64kB\n");
//		}
//
//		if (unlikely(skb_headroom(skb) < sizeof(u16))) {
//			netdev_err(ndev,
//			        "\nNot enough head room to store data length");
//		}
//
//		*(u16*)(skb->data - sizeof(u16)) = (u16)skb->len;
//		__dma_flush_range((const void*)start, (const void*)end);
//
//		*fep->qdma_regs &= ~1;
//		status = *(fep->qdma_regs + 1) & 0x92;
//
//		if (status) *(fep->qdma_regs + 1) = status;
//
//		*(fep->qdma_regs + 4) = upper_32_bits(virt_to_phys(start)) & 0xffff;
//		*(fep->qdma_regs + 5) = lower_32_bits(virt_to_phys(start));
//
//		*(fep->qdma_regs + 6) = (u32)(fep->pci_dev->resource[0].start >> 32);
//		*(fep->qdma_regs + 7) = (u32)(fep->pci_dev->resource[0].start) +
//						OFFSET_TO_DATA + ((write_i & (MAX_NO_BUFFERS - 1)) * (MAX_BUFFER_SIZE));
//		*(fep->qdma_regs + 8) = skb->len + sizeof(u16);
//
//		while (1) {
//			*fep->qdma_regs = 1;
//
//			do {
//				status = *(fep->qdma_regs + 1);
//			}
//			while (status & (1 << 2));
//
//			if (status & (1 << 7)) {
//				*(fep->qdma_regs + 1) = status;
//				*fep->qdma_regs &= ~1;
//				// redo tx
//				ndev->stats.tx_errors ++;
//			}
//			else break;
//		}
//		write_i ++;
//		fep->ctrl_ved_r->current_write_index = write_i;
//		fep->level ^= 1;
//		gpio_set_value(LS2S32V_INT_PIN, fep->level);
//
//		ndev->stats.tx_packets++;
//		ndev->stats.tx_bytes += skb->len;
//	}
//	spin_unlock_irqrestore(&fep->spinlock, flags);
//
//	dev_kfree_skb_any(skb);	/* kfree skbuf */
//
//	return NETDEV_TX_OK;
//}

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

//static int fpx_enet_rx_napi(struct napi_struct *napi, int budget)
//{
//	struct net_device *ndev = napi->dev;
//	struct fpx_enet_private *fep = netdev_priv(ndev);
//	int pkts = 0;
//	struct sk_buff *skb;
//
//	u64 write_i, read_i;
//	volatile void* tmp_data;
//
//	write_i = fep->ctrl_ved_l->current_write_index;
//	read_i = fep->ctrl_ved_l->current_read_index;
//
//	tmp_data = ((volatile void *)fep->received_data_l) +
//		((read_i & (MAX_NO_BUFFERS - 1)) * MAX_BUFFER_SIZE);
//
//	while ((pkts < budget) && (write_i > read_i)) {
//		u16 buf_len;
//		pkts ++;
//		/* get the buffer length and restore the data */
//		buf_len = *(u16*)tmp_data;
////		printk("Rx buff len: %x\n", buf_len);
////		printk("Rx buff first 4 bytes: %x\n", tmp_data + sizeof(u16));
//
//		if (buf_len < MAX_BUFFER_SIZE) {
//			skb = napi_alloc_skb(napi, buf_len);
//			if (unlikely(!skb)) {
//				ndev->stats.rx_dropped++;
//			} else {
//				unsigned char* tmp = NULL;
//
//				/* update RX stats */
//				ndev->stats.rx_packets++;
//				ndev->stats.rx_bytes += buf_len;
//
//				/* TODO: check small performance improvement
//				 * update skb alloc size and buf_len check */
//				//skb_reserve(skb, NET_IP_ALIGN);
//
//				/* do memcpy */
//				tmp = skb_put(skb, buf_len);
//
//				memcpy((tmp), (const void *)tmp_data + sizeof(u16), buf_len);
//				skb->protocol = eth_type_trans(skb, ndev);
//
//				/* TODO: check small performance improvement */
//				//skb->ip_summed = CHECKSUM_UNNECESSARY;
//
//				netif_receive_skb(skb);
//
//				/* TODO: check small performance improvement */
//				//napi_gro_receive(napi, skb);
//			}
//		} else {
//			ndev->stats.rx_errors ++;
//		}
//		read_i ++;
//		write_i = fep->ctrl_ved_l->current_write_index;	/* read the new write index */
//		fep->ctrl_ved_l->current_read_index = read_i;	/* update the read index */
//		if (read_i & ((MAX_NO_BUFFERS - 1))) {
//			tmp_data += MAX_BUFFER_SIZE;
//		} else {
//			tmp_data = (void *)fep->received_data_l;
//		}
//	}
//
//	/* re-enable interrupts */
//	if (pkts < budget) {
//		napi_complete(napi);
//		/* reenable interrupt */
//	}
//	return pkts;
//}

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
	napi_schedule(&fep->napi);

	return IRQ_HANDLED;
}

static int
fpx_open(struct net_device *ndev)
{
	struct fpx_enet_private *fep = netdev_priv(ndev);

	printk(KERN_ERR"fpx_open()\n");

	/* clear local data */
//	fep->ctrl_ved_l->current_write_index = 0;
//	fep->ctrl_ved_l->current_read_index = 0;

	napi_enable(&fep->napi);
	enable_irq(fep->pci_dev->irq);
	netif_tx_start_all_queues(ndev);

	return 0;
}

static int
fpx_close(struct net_device *ndev)
{
	struct fpx_enet_private *fep = netdev_priv(ndev);

	printk(KERN_ERR"fpx_close()\n");

	disable_irq(fep->pci_dev->irq);
	/* wait for all pending rx frames to be processed by napi */
	msleep(500);

	napi_disable(&fep->napi);
	netif_tx_disable(ndev);

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
	int ret = 0;
	unsigned long io_len;

	printk(KERN_ERR"fpx probe\n");
	printk(KERN_ERR"PCI vendor = %04x, dev = %04x\n", pdev->vendor,
			pdev->device);

	ndev = alloc_netdev(sizeof(struct fpx_enet_private), "fpx%d",
				NET_NAME_ENUM, ether_setup);
	if (!ndev) {
		printk(KERN_ERR"Error alloc_netdev, exit.\n");
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


	ret = pci_enable_device(pdev);
	if (ret) {
		printk(KERN_ERR"Error enabling PCI device\n");
		goto err_pci_enable_device;
	}

	ret = pci_request_regions(pdev, DRIVER_NAME);
	if (ret) {
		printk(KERN_ERR"Error requesting region\n");
		goto err_pci_request_regions;
	}

	/////////////////////////////////////////////////
	io_len = pci_resource_len(pdev, 0);		//read bar 0
	ret = pci_resource_flags(pdev, 0);
	printk(KERN_ERR"bar0 len = %lu, %08x\n", io_len, ret);

	if (!(ret & IORESOURCE_MEM)) {
		printk(KERN_ERR"Bad PCI resource\n");
		ret = -ENODEV;
		goto err_bad_pci_resource;
	}
	/* alloc memory */
	fep->local_res = request_mem_region(LS_PCI_SMEM, LS_PCI_SMEM_SIZE,
			"pcie-local-ctrl");

	fep->ctrl_ved_l = (struct control_ved*)ioremap_cache(
		LS_PCI_SMEM, (unsigned long)sizeof(struct control_ved));

	fep->received_data_l = (volatile u32*)ioremap_cache(
		(resource_size_t)(LS_PCI_SMEM + sizeof(struct control_ved)),
		LS_PCI_SMEM_SIZE - sizeof(struct control_ved));

	fep->ctrl_ved_r = (struct control_ved*)pci_iomap(pdev, 0, io_len);
	fep->received_data_r = (volatile void*)(fep->ctrl_ved_r + 1);

	pci_set_master (pdev);
	ret = pci_enable_msi(pdev);
	if (ret) {
		printk(KERN_ERR"Error enabling PCI MSI\n");
		goto err_pci_enable_msi;
	}

	/* init qdma for tx */
	fep->qdma_regs = (unsigned int*)ioremap_nocache(QDMA_BASE, QDMA_REG_SIZE);
	if (!fep->qdma_regs) {
		printk(KERN_ERR"cannot map qdma registers\n");
		ret = -ENOMEM;
		goto err_init_qdma;
	}

	/* init tx gpio signaling interrupt */
	fep->level = 0;
	ret = gpio_request(LS2S32V_INT_PIN, "LS2_S32V_INT");
	if (ret) {
		printk(KERN_ERR"Cannot reserve GPIO LS2S32V_INT_PIN\n");
		goto err_gpio_request;
	}
	ret = gpio_direction_output(LS2S32V_INT_PIN, 0);
	if (ret) {
		printk(KERN_ERR"Cannot configure GPIO LS2S32V_INT_PIN as output\n");
		goto err_gpio_direction_output;
	}
	pci_set_drvdata (pdev, ndev);

//	nxp_pdev_init(pdev, ndev);

	/* init rx napi */
	netif_napi_add(ndev, &fep->napi, fpx_enet_rx_napi, NAPI_POLL_WEIGHT);
	/* init rx interrupt */
	ret = request_irq(pdev->irq, fpx_interrupt, IRQF_SHARED, ndev->name, ndev);
	if (ret) {
		printk(KERN_ERR"failed to register interrupt %d\n", pdev->irq);
		goto err_request_pci_irq;
	}
	/* disable rx intr until interface is up (ndo_open) */
	disable_irq(fep->pci_dev->irq);

	ret = register_netdev(ndev);
	if (ret) {
		printk(KERN_ERR"Error registering netdevice\n");
		goto err_register_netdev;
	}

	printk(KERN_ERR"Success %016llx\n", pdev->resource[0].start);

	return 0;

err_register_netdev:
	free_irq(pdev->irq, ndev);
err_request_pci_irq:
	netif_napi_del(&fep->napi);
err_gpio_direction_output:
	gpio_free(LS2S32V_INT_PIN);
err_gpio_request:
	iounmap((void*)fep->qdma_regs);
err_init_qdma:
	pci_disable_msi(pdev);
err_pci_enable_msi:
	iounmap((void*)fep->ctrl_ved_l);
	iounmap((void*)fep->received_data_l);
	release_mem_region(LS_PCI_SMEM, LS_PCI_SMEM_SIZE);
	pci_iounmap(pdev, fep->ctrl_ved_r);
err_bad_pci_resource:
	pci_release_regions(pdev);
err_pci_request_regions:
	pci_disable_device(pdev);
err_pci_enable_device:
	free_netdev(ndev);
	return ret;
}

static void fpx_remove(struct pci_dev *pdev)
{
//	struct net_device *ndev = nxp_pdev_get_upper_dev(pdev);
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct fpx_enet_private *fep = netdev_priv(ndev);

	if (ndev == NULL) {
		printk(KERN_ERR"Remove fpx device failed.\n");
		return;
	}

	unregister_netdev(ndev);
	free_irq(pdev->irq, ndev);
	netif_napi_del(&fep->napi);

//	nxp_pdev_free(pdev);

	gpio_free(LS2S32V_INT_PIN);
	iounmap((void*)fep->qdma_regs);
	pci_disable_msi(pdev);
	iounmap((void*)fep->ctrl_ved_l);
	iounmap((void*)fep->received_data_l);
	release_mem_region(LS_PCI_SMEM, LS_PCI_SMEM_SIZE);
	pci_iounmap(pdev, fep->ctrl_ved_r);
	pci_release_regions(pdev);
	pci_disable_device(pdev);

	free_netdev(ndev);

	printk(KERN_ERR"fpx device removed successfully.\n");
}

//static const struct pci_device_id fpx_ids[] = {
//{
//	PCI_DEVICE(0x1957, 0x4001)
//},
//{0, }
//};
//
//MODULE_DEVICE_TABLE(pci, fpx_ids);

static struct pci_driver fpx_driver = {
//	.name = "fpx_pci",
//	.id_table = fpx_ids,
	.probe = fpx_probe,
	.remove = fpx_remove,
};

static int __init fpx_init(void)
{
	printk(KERN_ERR"fpx_init() - v0.1\n");
	return nxp_pci_register_driver(&fpx_driver);
}

static void __exit fpx_exit(void)
{
	nxp_pci_unregister_driver(&fpx_driver);
}

module_init(fpx_init);
module_exit(fpx_exit);

MODULE_ALIAS("platform:"DRIVER_NAME);
MODULE_LICENSE("GPL");

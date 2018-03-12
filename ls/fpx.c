/*
 * Freescale PCI Express virtual network driver for LayerScape 
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
#include "fpx.h"

#define DRIVER_NAME	"fpx"

extern void __dma_flush_area(const void *, size_t);

static netdev_tx_t
fpx_enet_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct fpx_enet_private *fep;
	u64 write_i, read_i;
	unsigned long flags;

	fep = netdev_priv(ndev);

	spin_lock_irqsave(&fep->spinlock, flags);
	write_i = fep->ctrl_ved_r->current_write_index;
	read_i = fep->ctrl_ved_r->current_read_index;

	if (write_i > (read_i + MAX_NO_BUFFERS)) {
		/* discard frame */
		ndev->stats.tx_dropped ++;
	}
	else {
		u32 status = 0;
		void* start = (skb->data - sizeof(u16));

		*(u16*)(skb->data - sizeof(u16)) = (u16)skb->len;
		__dma_flush_area(start, skb->len + sizeof(u16));

		*fep->qdma_regs &= ~1;
		status = *(fep->qdma_regs + 1) & 0x92;

		if (status) *(fep->qdma_regs + 1) = status;

		*(fep->qdma_regs + 4) = upper_32_bits(virt_to_phys(start)) & 0xffff;
		*(fep->qdma_regs + 5) = lower_32_bits(virt_to_phys(start));

		*(fep->qdma_regs + 6) = (u32)(fep->pci_dev->resource[0].start >> 32);
		*(fep->qdma_regs + 7) = (u32)(fep->pci_dev->resource[0].start) + 
						OFFSET_TO_DATA + ((write_i & (MAX_NO_BUFFERS - 1)) * (MAX_BUFFER_SIZE));
		*(fep->qdma_regs + 8) = skb->len + sizeof(u16);

		while (1) {
			*fep->qdma_regs = 1;

			do {
				status = *(fep->qdma_regs + 1);
			}
			while (status & (1 << 2));

			if (status & (1 << 7)) {
				*(fep->qdma_regs + 1) = status;
				*fep->qdma_regs &= ~1;
				// redo tx
				ndev->stats.tx_errors ++;
			}
			else break;
		}
		write_i ++;
		fep->ctrl_ved_r->current_write_index = write_i;
		fep->level ^= 1;
		gpio_set_value(LS2S32V_INT_PIN, fep->level);

		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += skb->len;
	}
	spin_unlock_irqrestore(&fep->spinlock, flags);

	dev_kfree_skb_any(skb);	/* kfree skbuf */

	return NETDEV_TX_OK;
}


static int fpx_enet_rx_napi(struct napi_struct *napi, int budget)
{
	struct net_device *ndev = napi->dev;
	struct fpx_enet_private *fep = netdev_priv(ndev);
	int pkts = 0;
	struct sk_buff *sk;

	u64 write_i, read_i;
	volatile void* tmp_data;
	int sk_index;

	write_i = fep->ctrl_ved_l->current_write_index;
	read_i = fep->ctrl_ved_l->current_read_index;

	tmp_data = ((volatile void *)fep->received_data_l) +
		((read_i & (MAX_NO_BUFFERS - 1)) * MAX_BUFFER_SIZE);

	while ((pkts < budget) && (write_i > read_i)) {
		u16 buf_len;
		pkts ++;
		/* get the buffer length and restore the data */
		buf_len = *(u16*)tmp_data;

		if (buf_len < MAX_BUFFER_SIZE) {

			sk_index = fep->rx_sk_buff_index;
			if (SKBUF_Q_SIZE > sk_index) {
				sk = fep->sk_buff_queue_rx[sk_index];
				fep->sk_buff_queue_rx[sk_index] = NULL; /* debug */
				fep->rx_sk_buff_index ++;
			}
			else { /* no more sk_buf */
				for (sk_index = 0; sk_index < SKBUF_Q_SIZE; sk_index ++) {
					sk = __netdev_alloc_skb_ip_align(ndev, MAX_BUFFER_SIZE, GFP_ATOMIC);
					fep->sk_buff_queue_rx[sk_index] = sk;
				}

				sk = fep->sk_buff_queue_rx[0];
				fep->sk_buff_queue_rx[0] = NULL; /* debug */
				fep->rx_sk_buff_index = 1;
			}

			if (unlikely(!sk)) {
				ndev->stats.rx_dropped++;
			}
			else {
				unsigned char* tmp = NULL;

				/* update RX stats */
				ndev->stats.rx_packets++;
				ndev->stats.rx_bytes += buf_len;
				/* do memcpy */
				tmp = skb_put(sk, buf_len);

				memcpy((tmp), (const void *)tmp_data + sizeof(u16), buf_len);
				sk->protocol = eth_type_trans(sk, ndev);
				sk->ip_summed = CHECKSUM_UNNECESSARY;
				netif_receive_skb(sk);
			}
		}
		else {
			ndev->stats.rx_errors ++;
		}
		read_i ++;
		write_i = fep->ctrl_ved_l->current_write_index;	/* read the new write index */
		fep->ctrl_ved_l->current_read_index = read_i;	/* update the read index */
		if (read_i & ((MAX_NO_BUFFERS - 1))) {
			tmp_data += MAX_BUFFER_SIZE;
		}
		else {
			tmp_data = (void *)fep->received_data_l;
		}
	}

	/* re-enable interrupts */
	if (pkts < budget) {
		napi_complete(napi);
		/* reenable interrupt */
	}
	return pkts;
}

static irqreturn_t fpx_interrupt (int irq, void *dev_instance)
{
	struct net_device *ndev = (struct net_device *) dev_instance;
	struct fpx_enet_private *fep = netdev_priv(ndev);
	if (napi_schedule_prep(&fep->napi)) {
		__napi_schedule(&fep->napi);
	}

	return IRQ_RETVAL(1);
}


static int
fpx_enet_open(struct net_device *ndev)
{
	int retval = 0;
	struct fpx_enet_private *fep = netdev_priv(ndev);
	fep->qdma_regs = (unsigned int*)ioremap_nocache(QDMA_BASE, QDMA_REG_SIZE);
	if (!fep->qdma_regs) {
		printk(KERN_ERR"cannot map qdma registers\n");
		return -1;
	}

	/* clear local data */
	fep->ctrl_ved_l->current_write_index = 0;
	fep->ctrl_ved_l->current_read_index = 0;

	fep->level = 0;
	fep->rx_sk_buff_index = 0;

	for (retval = 0; retval < SKBUF_Q_SIZE; retval ++) {
		struct sk_buff *sk = __netdev_alloc_skb_ip_align(ndev, MAX_BUFFER_SIZE, GFP_KERNEL); /* max ethernet frame length */
		if (sk) {
			fep->sk_buff_queue_rx[retval] = sk;
		}
		else {
			printk(KERN_ERR"cannot alloc buffer %d\n", retval);
			retval --;
		}
	}

#if !(MSI_WORKAROUND)
	if (request_irq(fep->pci_dev->irq, fpx_interrupt, IRQF_SHARED, ndev->name, ndev)) {
		printk(KERN_ERR"failed to register interrupt %d\n", fep->pci_dev->irq);
	}
#endif
	napi_enable(&fep->napi);
	netif_tx_start_all_queues(ndev);

	device_set_wakeup_enable(&ndev->dev, 0);

	retval = gpio_request(LS2S32V_INT_PIN, "LS2_S32V_INT");
	if (retval) printk(KERN_ERR"Cannot reserve GPIO LS2S32V_INT_PIN\n");
	retval = gpio_direction_output(LS2S32V_INT_PIN, 0);
	if (retval) printk(KERN_ERR"Cannot configure GPIO LS2S32V_INT_PIN as output\n");

#if MSI_WORKAROUND
	/* Workaround MSI */
	retval = gpio_request(S32V2LS_INT_PIN, "S32V_LS2_INT");
	retval = gpio_direction_input(S32V2LS_INT_PIN);
	retval = gpio_to_irq(S32V2LS_INT_PIN);
	if (retval < 0) {
		printk(KERN_ERR"Cannot setup IRQ for GPIO S32V2LS_INT_PIN\n");
	}
	else {
		if (request_irq(retval, fpx_interrupt, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, ndev->name, ndev)) {
			printk(KERN_ERR"failed to register interrupt %d\n", retval);
		}
		else {
			fep->irq = retval;
			printk(KERN_ERR"register interrupt %d\n", retval);
		}
	}
#endif
	return 0;
}

static int
fpx_enet_close(struct net_device *ndev)
{
	int i;
	struct fpx_enet_private *fep = netdev_priv(ndev);

	gpio_free(LS2S32V_INT_PIN);
#if MSI_WORKAROUND
	/* Workaround MSI */
	gpio_free(S32V2LS_INT_PIN);
	free_irq(fep->irq, ndev);
#else
	free_irq(fep->pci_dev->irq, ndev);
#endif
	iounmap((void*)fep->qdma_regs);
	if (netif_device_present(ndev)) {
		napi_disable(&fep->napi);
		netif_tx_disable(ndev);
	}
	for (i = 0; i < SKBUF_Q_SIZE; i ++) {
		if (fep->sk_buff_queue_rx[i]) dev_kfree_skb(fep->sk_buff_queue_rx[i]);
	}

	return 0;
}
static const struct net_device_ops fpx_netdev_ops = {
	.ndo_open		= fpx_enet_open,
	.ndo_stop		= fpx_enet_close,
	.ndo_start_xmit		= fpx_enet_start_xmit,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
};


static int fpx_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct net_device *ndev = NULL;
	struct fpx_enet_private *fep = NULL;
	int ret = 0;
	unsigned long io_len;

	printk(KERN_ERR"rc fpx_probe vendor = %04x, dev = %04x\n", pdev->vendor,
			pdev->device);

	ndev = alloc_netdev(sizeof(struct fpx_enet_private),
			"fpx%d", NET_NAME_ENUM, ether_setup);
	if (!ndev) {
		printk(KERN_ERR"Error alloc_netdev, exit.\n");
		return -ENOMEM;
	}

	ndev->flags = 0;
	ndev->priv_flags &= ~IFF_TX_SKB_SHARING;
	ndev->netdev_ops = &fpx_netdev_ops;
	ndev->needed_headroom = sizeof(u16);

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

	netif_napi_add(ndev, &fep->napi, fpx_enet_rx_napi, NAPI_POLL_WEIGHT);
	ret = register_netdev(ndev);
	if (ret) {
		printk(KERN_ERR"Error registering netdevice\n");
		netif_napi_del(&fep->napi);
		goto err_register_netdev;
	}

	/* success */
	pci_set_drvdata (pdev, ndev);
	spin_lock_init(&fep->spinlock);
	printk(KERN_ERR"Success %016llx\n", pdev->resource[0].start);
	return 0;
err_register_netdev:
	netif_napi_del(&fep->napi);
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
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct fpx_enet_private *fep = netdev_priv(ndev);

	printk(KERN_ERR"Remove fpx device.\n");
	netif_napi_del(&fep->napi);
	pci_disable_msi(pdev);
	unregister_netdev(ndev);
	iounmap((void*)fep->ctrl_ved_l);
	iounmap((void*)fep->received_data_l);
	release_mem_region(LS_PCI_SMEM, LS_PCI_SMEM_SIZE);
	pci_iounmap(pdev, fep->ctrl_ved_r);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	free_netdev(ndev);
}

static const struct pci_device_id fpx_ids[] = {
{
	PCI_DEVICE(0x1957, 0x4000)
},
{
	PCI_DEVICE(0x1957, 0x4001)
},
{0, }
};

MODULE_DEVICE_TABLE(pci, fpx_ids);

static struct pci_driver fpx_driver = {
	.name = "fpx_pci",
	.id_table = fpx_ids,
	.probe = fpx_probe,
	.remove = fpx_remove,
};

static int __init fpx_init(void)
{
	printk(KERN_ERR"rc fpx_init\n");
	return pci_register_driver(&fpx_driver);
}
static void __exit fpx_exit(void)
{
	pci_unregister_driver(&fpx_driver);
}

module_init(fpx_init);
module_exit(fpx_exit);

MODULE_ALIAS("platform:"DRIVER_NAME);
MODULE_LICENSE("GPL");

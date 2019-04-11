/*
 * Freescale PCI Express virtual network driver for S32V234 
 * Copyright 2017-2019 NXP
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
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <linux/version.h>
#include <asm/cacheflush.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 18, 0)
#include <../drivers/pci/controller/dwc/pcie-designware.h>
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 4)
#include <../drivers/pci/dwc/pcie-designware.h>
#else
#include <../drivers/pci/host/pcie-designware.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 4)
extern struct dw_pcie *s32v_get_dw_pcie(void);
#else
extern struct pcie_port *s32v_get_pcie_port(void);
#endif

extern void register_callback(void*);
#include "fpx.h"
#define DRIVER_NAME	"fpx"

static struct net_device *s_ndev = NULL;

struct s32v_inbound_region {
	u32 bar_nr;
	u32 target_addr;
	u32 region;
};
extern int s32v_pcie_setup_inbound(void *data);

struct s32v_outbound_region {
	u64 target_addr;
	u64 base_addr;
	u32 size;
	u32 region;
	u32 region_type;
};


extern int s32v_pcie_setup_outbound(void *data);

static void fpx_flush_range(const void *start, const void *end)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 4)
	__dma_flush_area(start, (size_t)((void*)end - (void*)start));
#else
	__dma_flush_range(start, end);
#endif
}

inline void fpx_inval_range(const void *start, const void *end)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 4)
	__inval_dcache_area((void *)start, (size_t)((void*)end - (void*)start));
#else
	__inval_cache_range(start, end);
#endif
}

static netdev_tx_t
fpx_enet_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct fpx_enet_private *fep = netdev_priv(ndev);
	unsigned long flags;
	u64 write_i, read_i;

	// TEST ONLY - don't to dma chaining.
	while (STS_TX_IDLE != fep->transmiter_status);

	spin_lock_irqsave(&fep->spinlock, flags);
	write_i = fep->ctrl_ved_r->current_write_index;
	read_i = fep->ctrl_ved_r->current_read_index;

	if (STS_TX_IDLE == fep->transmiter_status) {
		if (write_i > (read_i + MAX_NO_BUFFERS)) {
			dev_kfree_skb_any(skb);
		}
		else {
			u64 dest_addr;

			void* start;
			void* end;

			fep->sk_buff_queue_tx_inprogress[0] = skb;
			fep->tail_txinprogress = 1;

			dest_addr = (unsigned long long int)(S32V_REMOTE_PCI_BASE +
				 OFFSET_TO_DATA + ((write_i & (MAX_NO_BUFFERS - 1)) * MAX_BUFFER_SIZE));
			write_i ++;

			/* prepare DMA transfer tmpTx*/
			fep->d_tx[0].chan_ctrl = 1;
			/* added the length of buffer in front of the data buffer */
			fep->d_tx[0].size =
				skb->len + sizeof(int);

			start = skb->data - sizeof(int);
			end = start + skb->len + sizeof(int);
			fep->d_tx[0].sar_low =
				lower_32_bits(virt_to_phys(start));
			fep->d_tx[0].sar_high =
				upper_32_bits(virt_to_phys(start));

			fep->d_tx[0].dar_low =
				lower_32_bits(dest_addr);
			fep->d_tx[0].dar_high =
				upper_32_bits(dest_addr);
			/* update inband length, write over unused ethernet header */
			*(int*)(start) = skb->len;

			/* flush cache */
			fpx_flush_range((const void*)start, (const void*)end);

			ndev->stats.tx_packets++;
			ndev->stats.tx_bytes += skb->len;

			/* update descriptor for write index */
			fep->ctrl_ved_l->val[0] = write_i;
			fep->d_tx[1].chan_ctrl = 0x19; /* LIE + RIE */
			fep->d_tx[1].size = sizeof(u64);
			fep->d_tx[1].sar_low =
				lower_32_bits(S32_PCI_SMEM + 4 * sizeof(u64));
			fep->d_tx[1].sar_high =
				upper_32_bits(S32_PCI_SMEM + 4 * sizeof(u64));

			fep->d_tx[1].dar_low =
				lower_32_bits(S32V_REMOTE_PCI_BASE);
			fep->d_tx[1].dar_high =
				upper_32_bits(S32V_REMOTE_PCI_BASE);

			fep->d_tx[2].chan_ctrl = 0x6; /* LLP */
			fep->d_tx[2].size = 0;
			fep->d_tx[2].sar_low =
				lower_32_bits(virt_to_phys(fep->d_tx));
			fep->d_tx[2].sar_high =
				upper_32_bits(virt_to_phys(fep->d_tx));

			/* do TX */
			/* Program DMA regs for LL mode */
			start = (void*)&fep->d_tx[0];
			end = (void*)&fep->d_tx[3];

			fpx_flush_range((const void*)start, (const void*)end);
			fep->transmiter_status = STS_TX_INPROGRESS;
			#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 4)
			dw_start_dma_llw(&fep->pcie->pp, virt_to_phys(fep->d_tx));
			#else
			dw_start_dma_llw(fep->pcie, virt_to_phys(fep->d_tx));
			#endif
		}
	}
	else {
		if ((fep->tail_totx - fep->head_totx) < SKBUF_Q_SIZE) {
			fep->sk_buff_queue_tx[fep->tail_totx & (SKBUF_Q_SIZE - 1)] = skb;
			fep->tail_totx ++; // add sk_buff to tail
		}
		else {
			dev_kfree_skb_any(skb);
			ndev->stats.tx_dropped++;
		}
	}
	spin_unlock_irqrestore(&fep->spinlock, flags);
	return NETDEV_TX_OK;
}

void fpx_irq_callback(u32 type)
{
	struct fpx_enet_private *fep = netdev_priv(s_ndev);

	int i = 0;
#if MSI_WORKAROUND
	/* Workaround MSI */
	fep->level ^= 1;
	gpio_set_value(S32V2LS_INT_PIN, fep->level);
#endif
	do {
		dev_kfree_skb_any(fep->sk_buff_queue_tx_inprogress[i ++]);

		if (fep->tail_txinprogress >= SKBUF_Q_SIZE) BUG();

	} while (i < fep->tail_txinprogress);
	fep->tail_txinprogress = 0;

	fep->transmiter_status = STS_TX_IDLE;
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
		unsigned int buf_len;
		pkts ++;
		/* get the buffer length and restore the data */
		fpx_inval_range((const void*)tmp_data, (const void*)tmp_data + 4);
		buf_len = *(unsigned int*)tmp_data;
		fpx_inval_range((const void*)tmp_data + 4, (const void*)tmp_data + 4 + buf_len);

		if (MAX_BUFFER_SIZE > buf_len) {

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

			sk_index = 0;
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


			/* only IPv4 support*/
			if ((*(((unsigned char*)tmp_data) + 4)) == 0x45) {
				sk->protocol = htons(ETH_P_IP);

				memcpy((tmp), (const void *)tmp_data + 4, buf_len);
				sk->ip_summed = CHECKSUM_UNNECESSARY;

				netif_receive_skb(sk);
			}
			else {
				fep->sk_buff_queue_rx[sk_index] = sk;	
				ndev->stats.rx_errors ++;
				fep->rx_sk_buff_index --;
				//sk->protocol = htons(ETH_P_IPV6);
			}
		}

		}
		else {
			ndev->stats.rx_errors ++;
		}
		read_i ++;
		write_i = fep->ctrl_ved_l->current_write_index; /* read the new write index */
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

static irqreturn_t fpx_rx (int irq, void *dev_instance) {

	struct fpx_enet_private *fep = netdev_priv((struct net_device *)dev_instance);

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

	#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 4)
	fep->pcie = s32v_get_dw_pcie();
	#else
	fep->pcie = s32v_get_pcie_port();
	#endif

	if (!fep->pcie) {
		printk(KERN_ERR"fatal error, cannot access PCI driver\n");
		retval = -EINVAL;
	}
	else {
		struct s32v_outbound_region outbound;

		/* MSI outbound area */
		outbound.target_addr = readl(fep->pcie->dbi_base + 0x54);

		if (!outbound.target_addr || (fep->ctrl_ved_l->magic_val != MAGIC_VAL_RC)) {
			printk(KERN_ERR
				"PCIe Root-Complex(RC) is not ready yet. Wait until Linux on the RC side enables the PCIe module.\n");
			return -EINVAL;
		}

		outbound.base_addr = S32_PCI_MSI_MEM;
		outbound.size = S32_PCI_MSI_SIZE;
		outbound.region = 3;
		outbound.region_type = 0; /* memory */
		s32v_pcie_setup_outbound(&outbound);
		/* do ioremap nocache, Remote area, outbound */
		fep->remote_res = request_mem_region(S32_PCI_MSI_MEM, S32_PCI_MSI_SIZE,
			"pcie-msi-buff");
		fep->msi_zone = (volatile int *)ioremap_nocache(
			(resource_size_t)S32_PCI_MSI_MEM, (unsigned long)S32_PCI_MSI_SIZE);

		/* set outbound area  */
		outbound.target_addr = fep->ctrl_ved_l->address_offset;
		outbound.base_addr = S32V_REMOTE_PCI_BASE;
		outbound.size = LS_PCI_SMEM_SIZE;
		outbound.region = 0;
		outbound.region_type = 0; /* memory */
		s32v_pcie_setup_outbound(&outbound);
		/* do ioremap nocache, Remote area, outbound */
		fep->remote_res = request_mem_region(S32V_REMOTE_PCI_BASE, LS_PCI_SMEM_SIZE,
                                    "pcie-remote-buff");
		fep->ctrl_ved_r = (struct control_ved*)ioremap_nocache(
			(resource_size_t)S32V_REMOTE_PCI_BASE, (unsigned long)LS_PCI_SMEM_SIZE);
		fep->received_data_r = (volatile u32*)(fep->ctrl_ved_r + 1);

		/* ENABLE MSI for DMA write */
		writel(S32_PCI_MSI_MEM, fep->pcie->dbi_base + 0x9d0);
		writel(0, fep->pcie->dbi_base + 0x9d4);
		writel(S32_PCI_MSI_MEM, fep->pcie->dbi_base + 0x9d8);
		writel(0, fep->pcie->dbi_base + 0x9dc);
		writel(0, fep->pcie->dbi_base + 0x9e0);

		fep->rx_sk_buff_index = 0;
		fep->ctrl_ved_l->current_write_index = 0;
		fep->ctrl_ved_l->current_read_index = 0;
		for (retval = 0; retval < SKBUF_Q_SIZE; retval ++) {
			/* max ethernet frame length */
			struct sk_buff *sk = __netdev_alloc_skb_ip_align(ndev, MAX_BUFFER_SIZE, GFP_KERNEL);
			if (sk) {
				fep->sk_buff_queue_rx[retval] = sk;
			}
			else {
				printk(KERN_ERR"cannot alloc buffer %d\n", retval);
				retval --;
			}
		}

		fep->transmiter_status = STS_TX_IDLE;
		fep->head_totx = fep->tail_totx = 0;
		fep->tail_txinprogress = 0;

		napi_enable(&fep->napi);
		netif_tx_start_all_queues(ndev);

		device_set_wakeup_enable(&ndev->dev, 0);

		/* register callback */
		#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 4)
		fep->pcie->pp.call_back = fpx_irq_callback;
		#else
		fep->pcie->call_back = fpx_irq_callback;
		#endif
		
#if MSI_WORKAROUND
		/* configure GPIO S32V2LS_INT_PIN to signal LS2, workaround MSI */
		retval = gpio_request(S32V2LS_INT_PIN, "S32V_LS2_INT");
		gpio_direction_output(S32V2LS_INT_PIN, 0);
		fep->level = 0;
#endif
		/* get GPIO LS2S32V_INT_PIN to receive interrupt from LS2 */
		retval = gpio_request(LS2S32V_INT_PIN, "LS2_S32V_INT");

		if (retval) printk(KERN_ERR"Cannot reserve GPIO LS2S32V_INT_PIN\n");
		
		retval = gpio_direction_input(LS2S32V_INT_PIN);
		if (retval) printk(KERN_ERR"Cannot configure GPIO LS2S32V_INT_PIN as input\n");

		retval = gpio_to_irq(LS2S32V_INT_PIN);

		if (retval < 0) {
			printk(KERN_ERR"Cannot setup IRQ for GPIO LS2S32V_INT_PIN\n");
		}
		else {
			fep->irq = retval;
			if (request_irq(retval, fpx_rx, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, ndev->name, ndev)) {
				printk(KERN_ERR"failed to register interrupt %d\n", retval);
			}
			else {
				retval = 0;
			}
		}
	}

	return retval;
}

static int
fpx_enet_close(struct net_device *ndev)
{
	struct fpx_enet_private *fep = netdev_priv(ndev);
	int i;

	free_irq(fep->irq, ndev);
	gpio_free(LS2S32V_INT_PIN);
#if MSI_WORKAROUND
	gpio_free(S32V2LS_INT_PIN);
#endif
	/* unregister callback */
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 4)
	fep->pcie->pp.call_back = NULL;
	#else
	fep->pcie->call_back = NULL;
	#endif
	fep->pcie = NULL;

	iounmap((void*)fep->msi_zone);
	release_mem_region(S32_PCI_MSI_MEM, S32_PCI_MSI_SIZE);

	if (netif_device_present(ndev)) {
		napi_disable(&fep->napi);
		netif_tx_disable(ndev);
	}

	for (i = 0; i < SKBUF_Q_SIZE; i ++) {
		if (fep->sk_buff_queue_rx[i]) dev_kfree_skb(fep->sk_buff_queue_rx[i]);
	}
	/* delete buffers */
	while (fep->head_totx != fep->tail_totx) {
		dev_kfree_skb(fep->sk_buff_queue_tx[fep->head_totx & (SKBUF_Q_SIZE - 1)]);
		fep->head_totx ++;
	}
	fep->head_totx = fep->tail_totx = 0;
	i = 0;
	while (i != fep->tail_txinprogress) {
		dev_kfree_skb(fep->sk_buff_queue_tx[i]);
		i ++;
	}
	fep->tail_txinprogress = 0;

	return 0;
}

static const struct net_device_ops fpx_netdev_ops = {
	.ndo_open		= fpx_enet_open,
	.ndo_stop		= fpx_enet_close,
	.ndo_start_xmit		= fpx_enet_start_xmit,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= fec_poll_controller,
#endif
};


static void
fpx_setup(struct net_device *dev)
{
	dev->mtu		= ETH_DATA_LEN;
	dev->tx_queue_len	= SKBUF_Q_SIZE;
	dev->flags		|= IFF_POINTOPOINT | IFF_NOARP;
	dev->netdev_ops		= &fpx_netdev_ops;
}


static int __init fpx_init(void)
{
	int ret = 0;
	struct fpx_enet_private *fep = NULL;
	struct net_device *ndev = NULL;
	struct s32v_inbound_region inbound;

	printk(KERN_ERR "installing new fep module\n");
	ndev = alloc_netdev(sizeof(struct fpx_enet_private),
			"fpx%d", NET_NAME_ENUM, fpx_setup);
	if (!ndev) {
		printk(KERN_ERR"Error alloc_netdev, exit.\n");
		return -ENOMEM;
	}
	/* init priv structure */
	fep = netdev_priv(ndev);
	fep->netdev = ndev;

	ndev->dev_addr[0] = 0x88;
	ndev->dev_addr[1] = 0x01;
	ndev->dev_addr[2] = 0x02;
	ndev->dev_addr[3] = 0x03;
	ndev->dev_addr[4] = 0x04;
	ndev->dev_addr[5] = 0x88;

	netif_napi_add(ndev, &fep->napi, fpx_enet_rx_napi, NAPI_POLL_WEIGHT);
	/* to config PCIx */
	/* set inbound area */
	inbound.bar_nr = 0;
	inbound.target_addr = S32_PCI_SMEM;
	inbound.region = 0;
	s32v_pcie_setup_inbound(&inbound);
	/* do ioremap nocache, Local area, inbound */
	fep->local_res = request_mem_region(S32_PCI_SMEM, S32_PCI_SMEM_SIZE,
                                    "pcie-local-buff");
	fep->ctrl_ved_l = (struct control_ved*)ioremap_nocache(
		(resource_size_t)S32_PCI_SMEM, (unsigned long)sizeof(struct control_ved));
	fep->received_data_l = (volatile void*)ioremap_cache((resource_size_t)S32_PCI_SMEM + sizeof(struct control_ved), 
		S32_PCI_SMEM_SIZE - sizeof(struct control_ved));

	ret = register_netdev(ndev);
	if (ret) {
		printk(KERN_ERR"Error registering netdevice\n");
		goto error_register;
	}

	spin_lock_init(&fep->spinlock);
	/* exit success */
	printk(KERN_ERR"Success!\n");
	goto return_et;
error_register:
	netif_napi_del(&fep->napi);
	iounmap((void*)fep->ctrl_ved_l);
	iounmap((void*)fep->received_data_l);
	iounmap((void*)fep->ctrl_ved_r);
	release_mem_region(S32_PCI_SMEM, S32_PCI_SMEM_SIZE);
	release_mem_region(S32V_REMOTE_PCI_BASE, LS_PCI_SMEM_SIZE);
	unregister_netdev(ndev);
	free_netdev(ndev);

return_et:
	s_ndev = ndev;
	return ret;
}

static void __exit fpx_exit(void)
{
	struct fpx_enet_private *fep = netdev_priv(s_ndev);
	netif_napi_del(&fep->napi);
	iounmap((void*)fep->ctrl_ved_l);
	iounmap((void*)fep->received_data_l);
	iounmap((void*)fep->ctrl_ved_r);
	release_mem_region(S32_PCI_SMEM, S32_PCI_SMEM_SIZE);
	release_mem_region(S32V_REMOTE_PCI_BASE, LS_PCI_SMEM_SIZE);
	unregister_netdev(s_ndev);
	free_netdev(s_ndev);
}

module_init(fpx_init);
module_exit(fpx_exit);

MODULE_ALIAS("platform:"DRIVER_NAME);
MODULE_LICENSE("GPL");

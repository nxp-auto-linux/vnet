/*
 * Freescale PCI Express virtual network driver for LayerScape 
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FPX_H
#define	FPX_H

#define MAX_NO_BUFFERS			512
#define SKBUF_Q_SIZE        	(MAX_NO_BUFFERS)
#define MAX_BUFFER_SIZE			(1536)


#define LS_PCI_SMEM_SIZE		0x00100000	/* 1MB */

#define MSI_WORKAROUND			0


#define QDMA_BASE			0x8390100
#define QDMA_REG_SIZE		0x100

#define LS2S32V_INT_PIN			434
#if MSI_WORKAROUND
	#define S32V2LS_INT_PIN		435
#endif


#define MAGIC_VAL_RC			0x12abdfcbed540312ULL

struct dma_desc {
	unsigned int chan_ctrl;
	unsigned int size;
	unsigned int sar_low;
	unsigned int sar_high;
	unsigned int dar_low;
	unsigned int dar_high;
};


struct control_ved
{
	volatile u64 current_write_index;
	volatile u64 current_read_index;
	volatile u64 magic_val;
	volatile u64 address_offset;
	volatile u64 val[4];
};

#define OFFSET_TO_DATA		sizeof(struct control_ved)
struct fpx_enet_private {
	/* Hardware registers of the fpx device */
	struct pci_dev *pci_dev;
	struct control_ved *ctrl_ved_l; /* control virtual ethernet device, local */
	struct control_ved *ctrl_ved_r; /* control virtual ethernet device, remote */
	struct net_device *netdev;
	struct napi_struct napi;
	spinlock_t spinlock;
	struct sk_buff *sk_buff_queue_rx[SKBUF_Q_SIZE];
	int rx_sk_buff_index;
	u32 level;
	volatile u32* qdma_regs;
#if MSI_WORKAROUND
	int irq;
#endif
	volatile void* received_data_l;
	volatile void* received_data_r;
	dma_addr_t dma_handle;
};

#endif /* FPX_H */

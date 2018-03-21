/*
 * Freescale PCI Express virtual network driver for S32V234 
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef FPX_H
#define	FPX_H

#define MAX_NO_BUFFERS			512
#define SKBUF_Q_SIZE			MAX_NO_BUFFERS
#define MAX_BUFFER_SIZE			(1536)

#define STS_TX_IDLE				0
#define STS_TX_INPROGRESS		1

#define S32_PCI_SMEM			0xfd000000

#define S32_PCI_SMEM_SIZE		0x00100000	/* 1MB */
#define LS_PCI_SMEM_SIZE		0x00100000	/* 1MB */
#define S32V_REMOTE_PCI_BASE	0x72000000	/* Local, S32V */

#define S32_PCI_MSI_SIZE		0x10000		/* 64 KB */
#define S32_PCI_MSI_MEM			0x72FB0000

#define CHAIN_SUPPORT			0

/* the MSI does not work on BlueBox Mini. As workaround,
 * the driver uses a GPIO to signal LS2 instead of MSI
 */
#define MSI_WORKAROUND			0

#define LS2S32V_INT_PIN			26
#if MSI_WORKAROUND
	#define S32V2LS_INT_PIN		27
#endif

#define MAGIC_VAL_RC            0x12abdfcbed540312ULL

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
	struct pcie_port *pp;
	struct control_ved *ctrl_ved_l; /* control virtual ethernet device, local */
	struct control_ved *ctrl_ved_r; /* control virtual ethernet device, remote */
	struct net_device *netdev;
	struct napi_struct napi;

	u64 head_totx;
	u64 tail_totx;
	volatile unsigned int transmiter_status;
	int tail_txinprogress;
	spinlock_t spinlock;
	volatile struct dma_desc d_tx[MAX_NO_BUFFERS + 2];

	struct sk_buff *sk_buff_queue_tx[SKBUF_Q_SIZE];
	struct sk_buff *sk_buff_queue_tx_inprogress[SKBUF_Q_SIZE];
	struct resource *local_res;
	struct resource *remote_res;
	volatile int* msi_zone;
	struct sk_buff *sk_buff_queue_rx[SKBUF_Q_SIZE];
	int rx_sk_buff_index;
	int irq;
#if MSI_WORKAROUND
	unsigned int level;
#endif
	volatile void* received_data_l;
	volatile void* received_data_r;
};

#endif /* FPX_H */

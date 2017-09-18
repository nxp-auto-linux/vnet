/*
 * Freescale PCI Express virtual network driver for S32V234 
 * Copyright (C) 2017 NXP
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

#define S32_PCI_SMEM_SIZE		0x00400000	/* 4 MB */

#define LS_PCI_SMEM				0x83A0000000ULL	/* Remote, LS */
#define LS_PCI_SMEM_SIZE		0x00400000	/* 4 MB */
#define S32V_REMOTE_PCI_BASE	0x72000000	/* Local, S32V */

#define S32_PCI_MSI_SIZE		0x10000		/* 64 KB */
#define S32_PCI_MSI_MEM			0x72FB0000



/* the MSI does not work on BlueBox Mini. As workaround,
 * the driver uses a GPIO to signal LS2 instead of MSI
 */
#define MSI_WORKAROUND			0

#define LS2S32V_INT_PIN			26
#if MSI_WORKAROUND
	#define S32V2LS_INT_PIN		27
#endif
struct dma_desc {
	unsigned int chan_ctrl;
	unsigned int size;
	unsigned int sar_low;
	unsigned int sar_high;
	unsigned int dar_low;
	unsigned int dar_high;
};

#define OFFSET_TO_DATA		0x3040

struct config_ved
{
	volatile unsigned int current_write_index;
	volatile unsigned int val[3];
	volatile struct dma_desc d_tx[MAX_NO_BUFFERS + 2];
	volatile unsigned int received_data[0];
};


struct fpx_enet_private {
	/* Hardware registers of the fpx device */
	struct pcie_port *pp;
	struct config_ved *cfg_ved_l; /* cfg + data virtual ethernet device, local */
	struct config_ved *cfg_ved_r; /* cfg + data virtual ethernet device, remote */
	struct net_device *netdev;
	struct napi_struct napi;

	int head_totx;
	int tail_totx;
	int tail_txinprogress;
	struct sk_buff *sk_buff_queue_tx[SKBUF_Q_SIZE];
	struct sk_buff *sk_buff_queue_tx_inprogress[SKBUF_Q_SIZE];
	volatile unsigned int transmiter_status;
	struct resource *local_res;
	struct resource *remote_res;
	spinlock_t spinlock;
	volatile int* msi_zone;
	struct sk_buff *sk_buff_queue_rx[SKBUF_Q_SIZE];
	int rx_sk_buff_index;
	int irq;
#if MSI_WORKAROUND
	unsigned int level;
#endif
	unsigned int remote_write_index;
	unsigned int current_read_index;
};

#endif /* FPX_H */

/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include "platform.h"

int nxp_pfm_dma_init(void *dma)
{
	/* DMA supported not implemented*/
	return -EOPNOTSUPP;
}

void nxp_pdev_dma_free(void **dma)
{
	/* DMA supported not implemented*/
	return;
}

int nxp_pdev_dma_write(void *dma, void *src_addr, phys_addr_t dest_addr,
			u32 size)
{
	/* DMA supported not implemented*/
	return -EOPNOTSUPP;
}

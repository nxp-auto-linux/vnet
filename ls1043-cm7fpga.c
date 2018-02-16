/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include "platform.h"

int nxp_pfm_init(void **platform)
{
	/* platform supported not implemented*/
	return -EOPNOTSUPP;
}

void nxp_pfm_free(void *platform)
{
	/* platform supported not implemented*/
}

void __iomem *nxp_pfm_alloc_local_shm(void *dev)
{
	/* platform supported not implemented*/
	return NULL;
}

void nxp_pfm_free_local_shm(void *dev, void __iomem *addr)
{
	/* platform supported not implemented*/
}

int nxp_pfm_dma_write(void *platform, void *src_addr, phys_addr_t dest_addr,
			u32 size)
{
	/* platform supported not implemented*/
	return -EOPNOTSUPP;
}

int nxp_pfm_trigger_remote_irq(void *platform)
{
	/* platform supported not implemented*/
	return -EOPNOTSUPP;
}

/*
 * Copyright (C) 2018 NXP Semiconductors
 *
 * SPDX-License-Identifier:		BSD-3-Clause
 */
#ifndef DRIVERS_NET_ETHERNET_PCI_VDEV_S32G_FPGA_MSCM_H
#define DRIVERS_NET_ETHERNET_PCI_VDEV_S32G_FPGA_MSCM_H

/* MSCM Peripheral Register Structure */
struct mscm_memmap {
	u32 CPXTYPE; /* Processor x Type Register, offset 0x0 */
	u32 CPXNUM; /* Processor x Number Register, offset 0x4 */
	u32 CPXMASTER; /* Processor x Master Number Register, offset 0x8 */
	u32 CPXCOUNT; /* Processor x Count Register, offset 0xC */
	u32 CPXCFGn[4]; /* Processor x Configuration n Register,
	                   offset 0x10 + 4*n */
	u32 CP0TYPE; /* Processor 0 Type Register, offset 0x20 */
	u32 CP0NUM; /* Processor 0 Number Register, offset 0x24 */
	u32 CP0MASTER; /* Processor 0 Master Number Register, offset 0x28 */
	u32 CP0COUNT; /* Processor 0 Count Register, offset 0x2C */
	u32 CP0CFGn[4]; /* Processor 0 Configuration n Register,
	                   offset 0x30 + 4*n */
	u32 CP1TYPE; /* Processor 1 Type Register, offset 0x40 */
	u32 CP1NUM; /* Processor 1 Number Register, offset 0x44 */
	u32 CP1MASTER; /* Processor 1 Master Number Register, offset 0x48 */
	u32 CP1COUNT; /* Processor 1 Count Register, offset 0x4C */
	u32 CP1CFGn[4]; /* Processor 1 Configuration n Register,
	                   offset 0x50 + 4*n */
	u8 reserved0[928];
	u32 OCMDRn[4]; /* On-Chip Memory Descriptor Register n,
	                  offset 0x400 + 4*n */
	u8 reserved1[112];
	u32 TCMDR0; /* Generic Tightly Coupled Memory Descriptor Register,
	               offset 0x480 */
	u8 reserved2[124];
	u32 CPCE0; /* Core Parity Checking Enable Register 0, offset 0x500 */
	u8 reserved3[764];
	u32 IRCP0IR; /* Interrupt Router CP0 Interrupt Register, offset 0x800 */
	u32 IRCP1IR; /* Interrupt Router CP1 Interrupt Register, offset 0x804 */
	u32 IRCP2IR; /* Interrupt Router CP2 Interrupt Register, offset 0x808 */
	u32 IRCP3IR; /* Interrupt Router CP3 Interrupt Register, offset 0x80C */
	u8 reserved4[16];
	u32 IRCPGIR; /* Interrupt Router CPU Generate Interrupt Register,
	                offset 0x820 */
	u8 reserved5[92];
	u16 IRSPRCn[175]; /* Interrupt Router Shared Peripheral Routing Control
	                     Register n, offset 0x880 + 2*n */
	u8 reserved6[800];
	u32 IPCGE; /* Interconnect Parity Checking Global Enable Register,
	              offset 0xD00 */
	u8 reserved7[12];
	u32 IPCEn[4]; /* Interconnect Parity Checking Enable Register n,
	                 offset 0xD10 + 4*n */
	u8 reserved8[32];
	u32 IPCGIE; /* Interconnect Parity Checking Global Injection Enable
	               Register, offset 0xD40 */
	u8 reserved9[12];
	u32 IPCIEn[4]; /* Interconnect Parity Checking Injection Enable
	                  Register n, offset 0xD50 + 4*n */
	u8 reserved10[656];
	u32 SYSCTRL; /* System Control Register, offset 0xFF0 */
};

/* MSCM Hardware Register Bit Fields Definition */
#define MSCM_IRCPGIR_TLF_MASK       (0x03000000ul) /* Target List Field */
#define MSCM_IRCPGIR_CPUTL_MASK     (0x000F0000ul) /* CPU Target List */
#define MSCM_IRCPGIR_INTID_MASK     (0x00000003ul) /* Interrupt ID */

#define MSCM_IRCPGIR_TLF(n)       ((n << 24u) & MSCM_IRCPGIR_TLF_MASK)
#define MSCM_IRCPGIR_CPUTL(n)     ((n << 16u) & MSCM_IRCPGIR_CPUTL_MASK)
#define MSCM_IRCPGIR_INTID(n)     ((n <<  0u) & MSCM_IRCPGIR_INTID_MASK)

#endif /* DRIVERS_NET_ETHERNET_PCI_VDEV_S32G_FPGA_MSCM_H */

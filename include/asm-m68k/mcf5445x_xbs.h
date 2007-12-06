/*
 * Kurt Mahan kmahan@freescale.com
 *
 * Copyright Freescale Semiconductor, Inc. 2007
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __MCF5445X_XBS_H__
#define __MCF5445X_XBS_H__

/*
 * Crossbar Switch (XBS)
 */

/* Register read/write macros */
#define MCF_XBS_PRS1		MCF_REG32(0xFC004100) /* Flexbus Priority */
#define MCF_XBS_CRS1		MCF_REG32(0xFC004110) /* Flexbus Control */
#define MCF_XBS_PRS2		MCF_REG32(0xFC004200) /* SDRam Priority */
#define MCF_XBS_CRS2		MCF_REG32(0xFC004210) /* SDRam Control */
#define MCF_XBS_PRS3		MCF_REG32(0xFC004300) /* ATA Priority */
#define MCF_XBS_CRS3		MCF_REG32(0xFC004310) /* ATA Control */
#define MCF_XBS_PRS4		MCF_REG32(0xFC004400) /* SRAM Priority */
#define MCF_XBS_CRS4		MCF_REG32(0xFC004410) /* SRAM Control */
#define MCF_XBS_PRS5		MCF_REG32(0xFC004500) /* PCI Priority */
#define MCF_XBS_CRS5		MCF_REG32(0xFC004510) /* PCI Control */
#define MCF_XBS_PRS6		MCF_REG32(0xFC004600) /* Slave6 Priority */
#define MCF_XBS_CRS6		MCF_REG32(0xFC004610) /* Slave6 Control */
#define MCF_XBS_PRS7		MCF_REG32(0xFC004700) /* Other Priority */
#define MCF_XBS_CRS7		MCF_REG32(0xFC004710) /* Other Control */

/* Priorities */
#define MCF_XBS_PRI_1		0	/* Level 1 (highest) */
#define MCF_XBS_PRI_2		1	/* Level 2 */
#define MCF_XBS_PRI_3		2	/* Level 3 */
#define MCF_XBS_PRI_4		3	/* Level 4 */
#define MCF_XBS_PRI_5		4	/* Level 5 */
#define MCF_XBS_PRI_6		5	/* Level 6 */
#define MCF_XBS_PRI_7		6	/* Level 7 (lowest) */
#define MCF_XBS_PRI_MASK	7	/* Mask (Not a valid level) */

/* Priority Register (PRSn) Defs */
#define	MCF_XBS_PRS_MACRO(m,p)	((p)<<((m)<<2))
#define	MCF_XBS_PRS_M0(p)	MCF_XBS_PRS_MACRO(0, p)	/* Coldfire Core */
#define	MCF_XBS_PRS_M1(p)	MCF_XBS_PRS_MACRO(1, p) /* eDMA */
#define	MCF_XBS_PRS_M2(p)	MCF_XBS_PRS_MACRO(2, p) /* FEC0 */
#define	MCF_XBS_PRS_M3(p)	MCF_XBS_PRS_MACRO(3, p) /* FEC1 */
#define	MCF_XBS_PRS_M4(p)	MCF_XBS_PRS_MACRO(4, p) /* Master 4 */
#define	MCF_XBS_PRS_M5(p)	MCF_XBS_PRS_MACRO(5, p) /* PCI */
#define	MCF_XBS_PRS_M6(p)	MCF_XBS_PRS_MACRO(6, p) /* USB OTG */
#define	MCF_XBS_PRS_M7(p)	MCF_XBS_PRS_MACRO(7, p) /* Serial Boot */

/* Control Register (CRSn) Defs */
#define MCF_XBS_CRS_RO		0x80000000	/* Read Only */
#define	MCF_XBS_CRS_ARB		0x00000100	/* Arbitration Mode */
#define	MCF_XBS_CRS_PCTL	0x00000030	/* Parking Control */
#define	MCF_XBS_CRS_PARK	0x00000007	/* Park Location */

/* MCF_XBS_CRS_ARB Defs */
#define MCF_ABS_CRS_ARB_FIXED	0x00000000	/* Fixed priority */
#define MCF_ABS_CRS_ARB_ROUND	0x00000100	/* Round Robin priority */

/* MCF_XBS_CRS_PCTL Defs */
#define MCF_ABS_CRS_PCTL_PARK	0x00000000	/* Park on the defined PARK */
#define MCF_ABS_CRS_PCTL_LAST 	0x00000010	/* Park on the last master */
#define MCF_ABS_CRS_PCTL_NONE 	0x00000020	/* Don't park */

/* MCF_XBS_CRS_PARK Defs */
#define MCF_ABS_CRS_PARK_M0	0x00000000	/* Park on Coldfire Core */
#define MCF_ABS_CRS_PARK_M1	0x00000001	/* Park on eDMA */
#define MCF_ABS_CRS_PARK_M2	0x00000002	/* Park on FEC0 */
#define MCF_ABS_CRS_PARK_M3	0x00000003	/* Park on FEC1 */
#define MCF_ABS_CRS_PARK_M4	0x00000004	/* Park on Reserved */
#define MCF_ABS_CRS_PARK_M5	0x00000005	/* Park on PCI */
#define MCF_ABS_CRS_PARK_M6	0x00000006	/* Park on USB OTG */
#define MCF_ABS_CRS_PARK_M7	0x00000007	/* Park on Serial Boot */

#endif /* __MCF5445X_XBS_H__ */

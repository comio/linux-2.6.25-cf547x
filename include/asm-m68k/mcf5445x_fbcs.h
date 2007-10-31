/*
 * Matt Waddel Matt.Waddel@freescale.com
 *
 * Copyright Freescale Semiconductor, Inc. 2007
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __MCF5445X_FBCS_H__
#define __MCF5445X_FBCS_H__

/*********************************************************************
*
* FlexBus Chip Selects (FBCS)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_FBCS_CSAR0	MCF_REG32(0xFC008000)   /* Chip-select Addr Register */
#define MCF_FBCS_CSMR0	MCF_REG32(0xFC008004)   /* Chip-select Mask Register */
#define MCF_FBCS_CSCR0	MCF_REG32(0xFC008008)   /* Chip-select Cntl Register */
#define MCF_FBCS_CSAR1	MCF_REG32(0xFC00800C)   /* Chip-select Addr Register */
#define MCF_FBCS_CSMR1	MCF_REG32(0xFC008010)   /* Chip-select Mask Register */
#define MCF_FBCS_CSCR1	MCF_REG32(0xFC008014)   /* Chip-select Cntl Register */
#define MCF_FBCS_CSAR2	MCF_REG32(0xFC008018)   /* Chip-select Addr Register */
#define MCF_FBCS_CSMR2	MCF_REG32(0xFC00801C)   /* Chip-select Mask Register */
#define MCF_FBCS_CSCR2	MCF_REG32(0xFC008020)   /* Chip-select Cntl Register */
#define MCF_FBCS_CSAR3	MCF_REG32(0xFC008024)   /* Chip-select Addr Register */
#define MCF_FBCS_CSMR3	MCF_REG32(0xFC008028)   /* Chip-select Mask Register */
#define MCF_FBCS_CSCR3	MCF_REG32(0xFC00802C)   /* Chip-select Cntl Register */

/* Parameterized register read/write macros for multiple registers */
#define MCF_FBCS_CSAR(x)	MCF_REG32(0xFC008000+((x)*0x00C))   /* Chip-select Addr Register */
#define MCF_FBCS_CSMR(x)	MCF_REG32(0xFC008004+((x)*0x00C))   /* Chip-select Mask Register */
#define MCF_FBCS_CSCR(x)	MCF_REG32(0xFC008008+((x)*0x00C))   /* Chip-select Cntl Register */

/* Bit definitions and macros for CSAR group */
#define MCF_FBCS_CSAR_BA(x)     ((x)&0xFFFF0000)

/* Bit definitions and macros for CSAR0 */
#define MCF_FBCS_CSAR0_BA(x)    ((x)&0xFFFF0000)

/* Bit definitions and macros for CSMR group */
#define MCF_FBCS_CSMR_V             (0x00000001)            /* Valid bit */
#define MCF_FBCS_CSMR_WP            (0x00000100)            /* Write protect */
#define MCF_FBCS_CSMR_BAM(x)        (((x)&0x0000FFFF)<<16)  /* Base addr mask */
#define MCF_FBCS_CSMR_BAM_4G        (0xFFFF0000)
#define MCF_FBCS_CSMR_BAM_2G        (0x7FFF0000)
#define MCF_FBCS_CSMR_BAM_1G        (0x3FFF0000)
#define MCF_FBCS_CSMR_BAM_1024M     (0x3FFF0000)
#define MCF_FBCS_CSMR_BAM_512M      (0x1FFF0000)
#define MCF_FBCS_CSMR_BAM_256M      (0x0FFF0000)
#define MCF_FBCS_CSMR_BAM_128M      (0x07FF0000)
#define MCF_FBCS_CSMR_BAM_64M       (0x03FF0000)
#define MCF_FBCS_CSMR_BAM_32M       (0x01FF0000)
#define MCF_FBCS_CSMR_BAM_16M       (0x00FF0000)
#define MCF_FBCS_CSMR_BAM_8M        (0x007F0000)
#define MCF_FBCS_CSMR_BAM_4M        (0x003F0000)
#define MCF_FBCS_CSMR_BAM_2M        (0x001F0000)
#define MCF_FBCS_CSMR_BAM_1M        (0x000F0000)
#define MCF_FBCS_CSMR_BAM_1024K     (0x000F0000)
#define MCF_FBCS_CSMR_BAM_512K      (0x00070000)
#define MCF_FBCS_CSMR_BAM_256K      (0x00030000)
#define MCF_FBCS_CSMR_BAM_128K      (0x00010000)
#define MCF_FBCS_CSMR_BAM_64K       (0x00000000)

/* Bit definitions and macros for CSMR0 */
#define MCF_FBCS_CSMR0_V            (0x00000001)            /* Valid bit */
#define MCF_FBCS_CSMR0_WP           (0x00000100)            /* Write protect */
#define MCF_FBCS_CSMR0_BAM(x)       (((x)&0x0000FFFF)<<16)  /* Base addr mask */
#define MCF_FBCS_CSMR0_BAM_4G       (0xFFFF0000)
#define MCF_FBCS_CSMR0_BAM_2G       (0x7FFF0000)
#define MCF_FBCS_CSMR0_BAM_1G       (0x3FFF0000)
#define MCF_FBCS_CSMR0_BAM_1024M    (0x3FFF0000)
#define MCF_FBCS_CSMR0_BAM_512M     (0x1FFF0000)
#define MCF_FBCS_CSMR0_BAM_256M     (0x0FFF0000)
#define MCF_FBCS_CSMR0_BAM_128M     (0x07FF0000)
#define MCF_FBCS_CSMR0_BAM_64M      (0x03FF0000)
#define MCF_FBCS_CSMR0_BAM_32M      (0x01FF0000)
#define MCF_FBCS_CSMR0_BAM_16M      (0x00FF0000)
#define MCF_FBCS_CSMR0_BAM_8M       (0x007F0000)
#define MCF_FBCS_CSMR0_BAM_4M       (0x003F0000)
#define MCF_FBCS_CSMR0_BAM_2M       (0x001F0000)
#define MCF_FBCS_CSMR0_BAM_1M       (0x000F0000)
#define MCF_FBCS_CSMR0_BAM_1024K    (0x000F0000)
#define MCF_FBCS_CSMR0_BAM_512K     (0x00070000)
#define MCF_FBCS_CSMR0_BAM_256K     (0x00030000)
#define MCF_FBCS_CSMR0_BAM_128K     (0x00010000)
#define MCF_FBCS_CSMR0_BAM_64K      (0x00000000)

/* Bit definitions and macros for CSCR group */
#define MCF_FBCS_CSCR_BSTW      (0x00000008)            /* Burst-write enable */
#define MCF_FBCS_CSCR_BSTR      (0x00000010)            /* Burst-read enable */
#define MCF_FBCS_CSCR_BEM       (0x00000020)            /* Byte-enable mode */
#define MCF_FBCS_CSCR_PS(x)     (((x)&0x00000003)<<6)   /* Port size */
#define MCF_FBCS_CSCR_AA        (0x00000100)            /* Auto-acknowledge */
#define MCF_FBCS_CSCR_WS(x)     (((x)&0x0000003F)<<10)  /* Wait states */
#define MCF_FBCS_CSCR_WRAH(x)   (((x)&0x00000003)<<16)  /* Write address hold or deselect */
#define MCF_FBCS_CSCR_RDAH(x)   (((x)&0x00000003)<<18)  /* Read address hold or deselect */
#define MCF_FBCS_CSCR_ASET(x)   (((x)&0x00000003)<<20)  /* Address setup */
#define MCF_FBCS_CSCR_SWSEN     (0x00800000)            /* Secondary wait state enable */
#define MCF_FBCS_CSCR_SWS(x)    (((x)&0x0000003F)<<26)  /* Secondary wait states */
#define MCF_FBCS_CSCR_PS_8      (0x00000040)
#define MCF_FBCS_CSCR_PS_16     (0x00000080)
#define MCF_FBCS_CSCR_PS_32     (0x00000000)

/* Bit definitions and macros for CSCR0 */
#define MCF_FBCS_CSCR0_BSTW	(0x00000008)            /* Burst-write enable */
#define MCF_FBCS_CSCR0_BSTR	(0x00000010)            /* Burst-read enable */
#define MCF_FBCS_CSCR0_BEM	(0x00000020)            /* Byte-enable mode */
#define MCF_FBCS_CSCR0_PS(x)	(((x)&0x00000003)<<6)   /* Port size */
#define MCF_FBCS_CSCR0_AA	(0x00000100)            /* Auto-acknowledge */
#define MCF_FBCS_CSCR0_WS(x)	(((x)&0x0000003F)<<10)  /* Wait states */
#define MCF_FBCS_CSCR0_WRAH(x)	(((x)&0x00000003)<<16)  /* Write address hold or deselect */
#define MCF_FBCS_CSCR0_RDAH(x)	(((x)&0x00000003)<<18)  /* Read address hold or deselect */
#define MCF_FBCS_CSCR0_ASET(x)	(((x)&0x00000003)<<20)  /* Address setup */
#define MCF_FBCS_CSCR0_SWSEN	(0x00800000)            /* Secondary wait state enable */
#define MCF_FBCS_CSCR0_SWS(x)	(((x)&0x0000003F)<<26)  /* Secondary wait states */
#define MCF_FBCS_CSCR0_PS_8	(0x00000040)
#define MCF_FBCS_CSCR0_PS_16	(0x00000080)
#define MCF_FBCS_CSCR0_PS_32	(0x00000000)

/* Bit definitions and macros for CSMR1 */
#define MCF_FBCS_CSMR1_V	(0x00000001)            /* Valid bit */
#define MCF_FBCS_CSMR1_WP	(0x00000100)            /* Write protect */
#define MCF_FBCS_CSMR1_BAM(x)	(((x)&0x0000FFFF)<<16)  /* Base address mask */

/* Bit definitions and macros for CSCR1 */
#define MCF_FBCS_CSCR1_BSTW	(0x00000008)            /* Burst-write enable */
#define MCF_FBCS_CSCR1_BSTR	(0x00000010)            /* Burst-read enable */
#define MCF_FBCS_CSCR1_BEM	(0x00000020)            /* Byte-enable mode */
#define MCF_FBCS_CSCR1_PS(x)	(((x)&0x00000003)<<6)   /* Port size */
#define MCF_FBCS_CSCR1_AA	(0x00000100)            /* Auto-acknowledge */
#define MCF_FBCS_CSCR1_WS(x)	(((x)&0x0000003F)<<10)  /* Wait states */
#define MCF_FBCS_CSCR1_WRAH(x)	(((x)&0x00000003)<<16)  /* Write address hold or deselect */
#define MCF_FBCS_CSCR1_RDAH(x)	(((x)&0x00000003)<<18)  /* Read address hold or deselect */
#define MCF_FBCS_CSCR1_ASET(x)	(((x)&0x00000003)<<20)  /* Address setup */
#define MCF_FBCS_CSCR1_SWSEN	(0x00800000)            /* Secondary wait state enable */
#define MCF_FBCS_CSCR1_SWS(x)	(((x)&0x0000003F)<<26)  /* Secondary wait states */

/* Bit definitions and macros for CSMR2 */
#define MCF_FBCS_CSMR2_V	(0x00000001)            /* Valid bit */
#define MCF_FBCS_CSMR2_WP	(0x00000100)            /* Write protect */
#define MCF_FBCS_CSMR2_BAM(x)	(((x)&0x0000FFFF)<<16)  /* Base address mask */

/* Bit definitions and macros for CSCR2 */
#define MCF_FBCS_CSCR2_BSTW	(0x00000008)            /* Burst-write enable */
#define MCF_FBCS_CSCR2_BSTR	(0x00000010)            /* Burst-read enable */
#define MCF_FBCS_CSCR2_BEM	(0x00000020)            /* Byte-enable mode */
#define MCF_FBCS_CSCR2_PS(x)	(((x)&0x00000003)<<6)   /* Port size */
#define MCF_FBCS_CSCR2_AA	(0x00000100)            /* Auto-acknowledge */
#define MCF_FBCS_CSCR2_WS(x)	(((x)&0x0000003F)<<10)  /* Wait states */
#define MCF_FBCS_CSCR2_WRAH(x)	(((x)&0x00000003)<<16)  /* Write address hold or deselect */
#define MCF_FBCS_CSCR2_RDAH(x)	(((x)&0x00000003)<<18)  /* Read address hold or deselect */
#define MCF_FBCS_CSCR2_ASET(x)	(((x)&0x00000003)<<20)  /* Address setup */
#define MCF_FBCS_CSCR2_SWSEN	(0x00800000)            /* Secondary wait state enable */
#define MCF_FBCS_CSCR2_SWS(x)	(((x)&0x0000003F)<<26)  /* Secondary wait states */

/* Bit definitions and macros for CSMR3 */
#define MCF_FBCS_CSMR3_V	(0x00000001)            /* Valid bit */
#define MCF_FBCS_CSMR3_WP	(0x00000100)            /* Write protect */
#define MCF_FBCS_CSMR3_BAM(x)	(((x)&0x0000FFFF)<<16)  /* Base address mask */

/* Bit definitions and macros for CSCR3 */
#define MCF_FBCS_CSCR3_BSTW	(0x00000008)            /* Burst-write enable */
#define MCF_FBCS_CSCR3_BSTR	(0x00000010)            /* Burst-read enable */
#define MCF_FBCS_CSCR3_BEM	(0x00000020)            /* Byte-enable mode */
#define MCF_FBCS_CSCR3_PS(x)	(((x)&0x00000003)<<6)   /* Port size */
#define MCF_FBCS_CSCR3_AA	(0x00000100)            /* Auto-acknowledge */
#define MCF_FBCS_CSCR3_WS(x)	(((x)&0x0000003F)<<10)  /* Wait states */
#define MCF_FBCS_CSCR3_WRAH(x)	(((x)&0x00000003)<<16)  /* Write address hold or deselect */
#define MCF_FBCS_CSCR3_RDAH(x)	(((x)&0x00000003)<<18)  /* Read address hold or deselect */
#define MCF_FBCS_CSCR3_ASET(x)	(((x)&0x00000003)<<20)  /* Address setup */
#define MCF_FBCS_CSCR3_SWSEN	(0x00800000)            /* Secondary wait state enable */
#define MCF_FBCS_CSCR3_SWS(x)	(((x)&0x0000003F)<<26)  /* Secondary wait states */

/********************************************************************/

#endif /* __MCF5445X_FBCS_H__ */

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

#ifndef __MCF5445X_SDRAMC_H__
#define __MCF5445X_SDRAMC_H__

/*********************************************************************
*
* SDRAM Controller (SDRAMC)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_SDRAMC_SDMR         (*(vuint32 *)(0xFC0B8000))   /* SDRAM Mode/Extended Mode Register */
#define MCF_SDRAMC_SDCR         (*(vuint32 *)(0xFC0B8004))   /* SDRAM Control Register */
#define MCF_SDRAMC_SDCFG1       (*(vuint32 *)(0xFC0B8008))   /* SDRAM Configuration Register 1 */
#define MCF_SDRAMC_SDCFG2       (*(vuint32 *)(0xFC0B800C))   /* SDRAM Configuration Register 2 */
#define MCF_SDRAMC_SDCS0        (*(vuint32 *)(0xFC0B8110))   /* SDRAM Chip Select Register */
#define MCF_SDRAMC_SDCS1        (*(vuint32 *)(0xFC0B8114))   /* SDRAM Chip Select Register */

/* Parameterized register read/write macros for multiple registers */
#define MCF_SDRAMC_SDCS(x)      (*(vuint32 *)(0xFC0B8110+((x)*0x004)))   /* SDRAM Chip Select Register */

/* Bit definitions and macros for SDMR */
#define MCF_SDRAMC_SDMR_DDR2_AD(x)      (((x)&0x00003FFF))      /* Address for DDR2 */
#define MCF_SDRAMC_SDMR_CMD             (0x00010000)            /* Command */
#define MCF_SDRAMC_SDMR_AD(x)           (((x)&0x00000FFF)<<18)  /* Address */
#define MCF_SDRAMC_SDMR_BK(x)           (((x)&0x00000003)<<30)  /* Bank Address */
#define MCF_SDRAMC_SDMR_BK_LMR          (0x00000000)
#define MCF_SDRAMC_SDMR_BK_LEMR         (0x40000000)

/* Bit definitions and macros for SDCR */
#define MCF_SDRAMC_SDCR_DPD             (0x00000001)            /* Deep Power-Down Mode */
#define MCF_SDRAMC_SDCR_IPALL           (0x00000002)            /* Initiate Precharge All */
#define MCF_SDRAMC_SDCR_IREF            (0x00000004)            /* Initiate Refresh */
#define MCF_SDRAMC_SDCR_DQS_OE(x)       (((x)&0x00000003)<<10)  /* DQS Output Enable */
#define MCF_SDRAMC_SDCR_MEM_PS          (0x00002000)            /* Data Port Size */
#define MCF_SDRAMC_SDCR_REF_CNT(x)      (((x)&0x0000003F)<<16)  /* Periodic Refresh Counter */
#define MCF_SDRAMC_SDCR_OE_RULE         (0x00400000)            /* Drive Rule Selection */
#define MCF_SDRAMC_SDCR_ADDR_MUX(x)     (((x)&0x00000003)<<24)  /* Internal Address Mux Select */
#define MCF_SDRAMC_SDCR_DDR2_MODE       (0x08000000)            /* DDR2 Mode Select */
#define MCF_SDRAMC_SDCR_REF_EN          (0x10000000)            /* Refresh Enable */
#define MCF_SDRAMC_SDCR_DDR_MODE        (0x20000000)            /* DDR Mode Select */
#define MCF_SDRAMC_SDCR_CKE             (0x40000000)            /* Clock Enable */
#define MCF_SDRAMC_SDCR_MODE_EN         (0x80000000)            /* SDRAM Mode Register Programming Enable */
#define MCF_SDRAMC_SDCR_DQS_OE_BOTH     (0x00000C00)

/* Bit definitions and macros for SDCFG1 */
#define MCF_SDRAMC_SDCFG1_WT_LAT(x)     (((x)&0x00000007)<<4)   /* Write Latency */
#define MCF_SDRAMC_SDCFG1_REF2ACT(x)    (((x)&0x0000000F)<<8)   /* Refresh to active delay */
#define MCF_SDRAMC_SDCFG1_PRE2ACT(x)    (((x)&0x00000007)<<12)  /* Precharge to active delay */
#define MCF_SDRAMC_SDCFG1_ACT2RW(x)     (((x)&0x00000007)<<16)  /* Active to read/write delay */
#define MCF_SDRAMC_SDCFG1_RD_LAT(x)     (((x)&0x0000000F)<<20)  /* Read CAS Latency */
#define MCF_SDRAMC_SDCFG1_SWT2RWP(x)    (((x)&0x00000007)<<24)  /* Single write to read/write/precharge delay */
#define MCF_SDRAMC_SDCFG1_SRD2RWP(x)    (((x)&0x0000000F)<<28)  /* Single read to read/write/precharge delay */

/* Bit definitions and macros for SDCFG2 */
#define MCF_SDRAMC_SDCFG2_BL(x)         (((x)&0x0000000F)<<16)  /* Burst Length */
#define MCF_SDRAMC_SDCFG2_BRD2W(x)      (((x)&0x0000000F)<<20)  /* Burst read to write delay */
#define MCF_SDRAMC_SDCFG2_BWT2RWP(x)    (((x)&0x0000000F)<<24)  /* Burst write to read/write/precharge delay */
#define MCF_SDRAMC_SDCFG2_BRD2RP(x)     (((x)&0x0000000F)<<28)  /* Burst read to read/precharge delay */

/* Bit definitions and macros for SDCS group */
#define MCF_SDRAMC_SDCS_CSSZ(x)         (((x)&0x0000001F))      /* Chip-Select Size */
#define MCF_SDRAMC_SDCS_CSBA(x)         (((x)&0x00000FFF)<<20)  /* Chip-Select Base Address */
#define MCF_SDRAMC_SDCS_BA(x)           ((x)&0xFFF00000)
#define MCF_SDRAMC_SDCS_CSSZ_DISABLE    (0x00000000)
#define MCF_SDRAMC_SDCS_CSSZ_1MBYTE     (0x00000013)
#define MCF_SDRAMC_SDCS_CSSZ_2MBYTE     (0x00000014)
#define MCF_SDRAMC_SDCS_CSSZ_4MBYTE     (0x00000015)
#define MCF_SDRAMC_SDCS_CSSZ_8MBYTE     (0x00000016)
#define MCF_SDRAMC_SDCS_CSSZ_16MBYTE    (0x00000017)
#define MCF_SDRAMC_SDCS_CSSZ_32MBYTE    (0x00000018)
#define MCF_SDRAMC_SDCS_CSSZ_64MBYTE    (0x00000019)
#define MCF_SDRAMC_SDCS_CSSZ_128MBYTE   (0x0000001A)
#define MCF_SDRAMC_SDCS_CSSZ_256MBYTE   (0x0000001B)
#define MCF_SDRAMC_SDCS_CSSZ_512MBYTE   (0x0000001C)
#define MCF_SDRAMC_SDCS_CSSZ_1GBYTE     (0x0000001D)
#define MCF_SDRAMC_SDCS_CSSZ_2GBYTE     (0x0000001E)
#define MCF_SDRAMC_SDCS_CSSZ_4GBYTE     (0x0000001F)

/* Bit definitions and macros for SDCS0 */
#define MCF_SDRAMC_SDCS0_CSSZ(x)            (((x)&0x0000001F))      /* Chip-Select Size */
#define MCF_SDRAMC_SDCS0_CSBA(x)            (((x)&0x00000FFF)<<20)  /* Chip-Select Base Address */
#define MCF_SDRAMC_SDCS0_BA(x)              ((x)&0xFFF00000)
#define MCF_SDRAMC_SDCS0_CSSZ_DISABLE       (0x00000000)
#define MCF_SDRAMC_SDCS0_CSSZ_1MBYTE        (0x00000013)
#define MCF_SDRAMC_SDCS0_CSSZ_2MBYTE        (0x00000014)
#define MCF_SDRAMC_SDCS0_CSSZ_4MBYTE        (0x00000015)
#define MCF_SDRAMC_SDCS0_CSSZ_8MBYTE        (0x00000016)
#define MCF_SDRAMC_SDCS0_CSSZ_16MBYTE       (0x00000017)
#define MCF_SDRAMC_SDCS0_CSSZ_32MBYTE       (0x00000018)
#define MCF_SDRAMC_SDCS0_CSSZ_64MBYTE       (0x00000019)
#define MCF_SDRAMC_SDCS0_CSSZ_128MBYTE      (0x0000001A)
#define MCF_SDRAMC_SDCS0_CSSZ_256MBYTE      (0x0000001B)
#define MCF_SDRAMC_SDCS0_CSSZ_512MBYTE      (0x0000001C)
#define MCF_SDRAMC_SDCS0_CSSZ_1GBYTE        (0x0000001D)
#define MCF_SDRAMC_SDCS0_CSSZ_2GBYTE        (0x0000001E)
#define MCF_SDRAMC_SDCS0_CSSZ_4GBYTE        (0x0000001F)

/* Bit definitions and macros for SDCS1 */
#define MCF_SDRAMC_SDCS1_CSSZ(x)    (((x)&0x0000001F))      /* Chip-Select Size */
#define MCF_SDRAMC_SDCS1_CSBA(x)    (((x)&0x00000FFF)<<20)  /* Chip-Select Base Address */

/********************************************************************/

#endif /* __MCF5445X_SDRAMC_H__ */

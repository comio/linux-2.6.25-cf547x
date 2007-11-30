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

#ifndef __MCF5445X_CCM_H__
#define __MCF5445X_CCM_H__

/*********************************************************************
*
* Chip Configuration Module (CCM)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_CCM_CCR	MCF_REG16(0xFC0A0004)	/* Chip Configuration Register (Read-only) (256 or 360 TEPBGA) */
#define MCF_CCM_RCON	MCF_REG16(0xFC0A0008)	/* Reset Configuration (Read-only) (256 or 360 TEPBGA) */
#define MCF_CCM_CIR	MCF_REG16(0xFC0A000A)	/* Chip Identification Register (Read-only) */
#define MCF_CCM_MISCCR	MCF_REG16(0xFC0A0010)	/* Miscellaneous Control Register */
#define MCF_CCM_CDR	MCF_REG16(0xFC0A0012)	/* Clock Divider Register */
#define MCF_CCM_UOCSR	MCF_REG16(0xFC0A0014)	/* USB On-the-Go Controller Status Register */
#define MCF_CCM_SBFSR	MCF_REG16(0xFC0A0018)	/* Serial Boot Facility Status Register (Read Only) */
#define MCF_CCM_SBFCR	MCF_REG16(0xFC0A0020)	/* Serial Boot Facility Control Register */

/* Bit definitions and macros for CCR_360 */
#define MCF_CCM_CCR_360_PLLMULT2(x)         (((x) & 0x0003))	/* 2-Bit PLL clock mode */
#define MCF_CCM_CCR_360_PCISLEW             (0x0004)		/* PCI pad slew rate mode */
#define MCF_CCM_CCR_360_PCIMODE             (0x0008)		/* PCI host/agent mode */
#define MCF_CCM_CCR_360_PLLMODE             (0x0010)		/* PLL Mode */
#define MCF_CCM_CCR_360_FBCONFIG(x)         (((x) & 0x0007) << 5)  /* Flexbus/PCI port size configuration */
#define MCF_CCM_CCR_360_PLLMULT3(x)         (((x) & 0x0007))	/* 3-Bit PLL Clock Mode */
#define MCF_CCM_CCR_360_OSCMODE             (0x0008)		/* Oscillator Clock Mode */
#define MCF_CCM_CCR_360_FBCONFIG_MASK       (0x00E0)
#define MCF_CCM_CCR_360_PLLMULT2_MASK       (0x0003)
#define MCF_CCM_CCR_360_PLLMULT3_MASK       (0x0007)
#define MCF_CCM_CCR_360_FBCONFIG_NM_NP_32   (0x0000)
#define MCF_CCM_CCR_360_FBCONFIG_NM_NP_8    (0x0020)
#define MCF_CCM_CCR_360_FBCONFIG_NM_NP_16   (0x0040)
#define MCF_CCM_CCR_360_FBCONFIG_M_P_16     (0x0060)
#define MCF_CCM_CCR_360_FBCONFIG_M_NP_32    (0x0080)
#define MCF_CCM_CCR_360_FBCONFIG_M_NP_8     (0x00A0)
#define MCF_CCM_CCR_360_FBCONFIG_M_NP_16    (0x00C0)
#define MCF_CCM_CCR_360_FBCONFIG_M_P_8      (0x00E0)
#define MCF_CCM_CCR_360_PLLMULT2_12X        (0x0000)
#define MCF_CCM_CCR_360_PLLMULT2_6X         (0x0001)
#define MCF_CCM_CCR_360_PLLMULT2_16X        (0x0002)
#define MCF_CCM_CCR_360_PLLMULT2_8X         (0x0003)
#define MCF_CCM_CCR_360_PLLMULT3_20X        (0x0000)
#define MCF_CCM_CCR_360_PLLMULT3_10X        (0x0001)
#define MCF_CCM_CCR_360_PLLMULT3_24X        (0x0002)
#define MCF_CCM_CCR_360_PLLMULT3_18X        (0x0003)
#define MCF_CCM_CCR_360_PLLMULT3_12X        (0x0004)
#define MCF_CCM_CCR_360_PLLMULT3_6X         (0x0005)
#define MCF_CCM_CCR_360_PLLMULT3_16X        (0x0006)
#define MCF_CCM_CCR_360_PLLMULT3_8X         (0x0007)

/* Bit definitions and macros for CCR_256 */
#define MCF_CCM_CCR_256_PLLMULT3(x)         (((x) & 0x0007))  /* 3-Bit PLL clock mode */
#define MCF_CCM_CCR_256_OSCMODE             (0x0008)	      /* Oscillator clock mode */
#define MCF_CCM_CCR_256_PLLMODE             (0x0010)	      /* PLL Mode */
#define MCF_CCM_CCR_256_FBCONFIG(x)         (((x) & 0x0007) << 5)  /* Flexbus/PCI port size configuration */
#define MCF_CCM_CCR_256_FBCONFIG_MASK       (0x00E0)
#define MCF_CCM_CCR_256_FBCONFIG_NM_32      (0x0000)
#define MCF_CCM_CCR_256_FBCONFIG_NM_8       (0x0020)
#define MCF_CCM_CCR_256_FBCONFIG_NM_16      (0x0040)
#define MCF_CCM_CCR_256_FBCONFIG_M_32       (0x0080)
#define MCF_CCM_CCR_256_FBCONFIG_M_8        (0x00A0)
#define MCF_CCM_CCR_256_FBCONFIG_M_16       (0x00C0)
#define MCF_CCM_CCR_256_PLLMULT3_MASK       (0x0007)
#define MCF_CCM_CCR_256_PLLMULT3_20X        (0x0000)
#define MCF_CCM_CCR_256_PLLMULT3_10X        (0x0001)
#define MCF_CCM_CCR_256_PLLMULT3_24X        (0x0002)
#define MCF_CCM_CCR_256_PLLMULT3_18X        (0x0003)
#define MCF_CCM_CCR_256_PLLMULT3_12X        (0x0004)
#define MCF_CCM_CCR_256_PLLMULT3_6X         (0x0005)
#define MCF_CCM_CCR_256_PLLMULT3_16X        (0x0006)
#define MCF_CCM_CCR_256_PLLMULT3_8X         (0x0007)

/* Bit definitions and macros for RCON_360 */
#define MCF_CCM_RCON_360_PLLMULT(x)     (((x)&0x0003))	/* PLL clock mode */
#define MCF_CCM_RCON_360_PCISLEW        (0x0004)	/* PCI pad slew rate mode */
#define MCF_CCM_RCON_360_PCIMODE        (0x0008)	/* PCI host/agent mode */
#define MCF_CCM_RCON_360_PLLMODE        (0x0010)	/* PLL Mode */
#define MCF_CCM_RCON_360_FBCONFIG(x)    (((x) & 0x0007) << 5)	/* Flexbus/PCI port size configuration */

/* Bit definitions and macros for RCON_256 */
#define MCF_CCM_RCON_256_PLLMULT(x)     (((x) & 0x0007)) /* PLL clock mode */
#define MCF_CCM_RCON_256_OSCMODE        (0x0008)	/* Oscillator clock mode */
#define MCF_CCM_RCON_256_PLLMODE        (0x0010)	/* PLL Mode */
#define MCF_CCM_RCON_256_FBCONFIG(x)    (((x) & 0x0007) << 5)	/* Flexbus/PCI port size configuration */

/* Bit definitions and macros for CIR */
#define MCF_CCM_CIR_PRN(x)          (((x) & 0x003F))	/* Part revision number */
#define MCF_CCM_CIR_PIN(x)          (((x) & 0x03FF) << 6)	/* Part identification number */
#define MCF_CCM_CIR_PIN_MASK        (0xFFC0)
#define MCF_CCM_CIR_PRN_MASK        (0x003F)
#define MCF_CCM_CIR_PIN_MCF54450    (0x4F << 6)
#define MCF_CCM_CIR_PIN_MCF54451    (0x4D << 6)
#define MCF_CCM_CIR_PIN_MCF54452    (0x4B << 6)
#define MCF_CCM_CIR_PIN_MCF54453    (0x49 << 6)
#define MCF_CCM_CIR_PIN_MCF54454    (0x4A << 6)
#define MCF_CCM_CIR_PIN_MCF54455    (0x48 << 6)

/* Bit definitions and macros for MISCCR */
#define MCF_CCM_MISCCR_USBSRC           (0x0001)	/* USB clock source */
#define MCF_CCM_MISCCR_USBOC            (0x0002)	/* USB VBUS over-current sense polarity */
#define MCF_CCM_MISCCR_USBPUE           (0x0004)	/* USB transceiver pull-up enable */
#define MCF_CCM_MISCCR_SSISRC           (0x0010)	/* SSI clock source */
#define MCF_CCM_MISCCR_TIMDMA           (0x0020)	/* Timer DMA mux selection */
#define MCF_CCM_MISCCR_SSIPUS           (0x0040)	/* SSI RXD/TXD pull select */
#define MCF_CCM_MISCCR_SSIPUE           (0x0080)	/* SSI RXD/TXD pull enable */
#define MCF_CCM_MISCCR_BMT(x)           (((x) & 0x0007) << 8)	/* Bus monitor timing field */
#define MCF_CCM_MISCCR_BME              (0x0800)	/* Bus monitor external enable bit */
#define MCF_CCM_MISCCR_LIMP             (0x1000)	/* Limp mode enable */
#define MCF_CCM_MISCCR_BMT_65536        (0 << 8)
#define MCF_CCM_MISCCR_BMT_32768        (1 << 8)
#define MCF_CCM_MISCCR_BMT_16384        (2 << 8)
#define MCF_CCM_MISCCR_BMT_8192         (3 << 8)
#define MCF_CCM_MISCCR_BMT_4096         (4 << 8)
#define MCF_CCM_MISCCR_BMT_2048         (5 << 8)
#define MCF_CCM_MISCCR_BMT_1024         (6 << 8)
#define MCF_CCM_MISCCR_BMT_512          (7 << 8)
#define MCF_CCM_MISCCR_SSIPUE_UP        (1 << 7)
#define MCF_CCM_MISCCR_SSIPUE_DOWN      (0 << 7)
#define MCF_CCM_MISCCR_SSIPUS_UP        (1 << 6)
#define MCF_CCM_MISCCR_SSIPUS_DOWN      (0 << 6)
#define MCF_CCM_MISCCR_TIMDMA_TIM       (1 << 5)
#define MCF_CCM_MISCCR_TIMDMA_SSI       (0 << 5)
#define MCF_CCM_MISCCR_SSISRC_CLKIN     (0 << 4)
#define MCF_CCM_MISCCR_SSISRC_PLL       (1 << 4)
#define MCF_CCM_MISCCR_USBOC_ACTHI      (0 << 1)
#define MCF_CCM_MISCCR_USBOC_ACTLO      (1 << 1)
#define MCF_CCM_MISCCR_USBSRC_CLKIN     (0)
#define MCF_CCM_MISCCR_USBSRC_PLL       (1)

/* Bit definitions and macros for CDR */
#define MCF_CCM_CDR_SSIDIV(x)   (((x) & 0x00FF))	/* SSI oversampling clock divider */
#define MCF_CCM_CDR_LPDIV(x)    (((x) & 0x000F) << 8)	/* Low power clock divider */

/* Bit definitions and macros for UOCSR */
#define MCF_CCM_UOCSR_XPDE          (0x0001)	/* On-chip transceiver pull-down enable */
#define MCF_CCM_UOCSR_UOMIE         (0x0002)	/* USB OTG misc interrupt enable */
#define MCF_CCM_UOCSR_WKUP          (0x0004)	/* USB OTG controller wake-up event */
#define MCF_CCM_UOCSR_PWRFLT        (0x0008)	/* VBUS power fault */
#define MCF_CCM_UOCSR_SEND          (0x0010)	/* Session end */
#define MCF_CCM_UOCSR_VVLD          (0x0020)	/* VBUS valid indicator */
#define MCF_CCM_UOCSR_BVLD          (0x0040)	/* B-peripheral valid indicator */
#define MCF_CCM_UOCSR_AVLD          (0x0080)	/* A-peripheral valid indicator */
#define MCF_CCM_UOCSR_DPPU          (0x0100)	/* D+ pull-up for FS enabled (read-only) */
#define MCF_CCM_UOCSR_DCR_VBUS      (0x0200)	/* VBUS discharge resistor enabled (read-only) */
#define MCF_CCM_UOCSR_CRG_VBUS      (0x0400)	/* VBUS charge resistor enabled (read-only) */
#define MCF_CCM_UOCSR_DMPD          (0x1000)	/* D- 15Kohm pull-down (read-only) */
#define MCF_CCM_UOCSR_DPPD          (0x2000)	/* D+ 15Kohm pull-down (read-only) */

/********************************************************************/

#endif /* __MCF5445X_CCM_H__ */

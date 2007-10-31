/*
 * Yaroslav Vinogradov yaroslav.vinogradov@freescale.com
 *
 * Copyright Freescale Semiconductor, Inc. 2007
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __MCF5445X_SSI_H__
#define __MCF5445X_SSI_H__

/*********************************************************************
*
* Synchronous Serial Interface (SSI)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_SSI_TX0             MCF_REG32(0xFC0BC000)
#define MCF_SSI_TX1             MCF_REG32(0xFC0BC004)
#define MCF_SSI_RX0             MCF_REG32(0xFC0BC008)
#define MCF_SSI_RX1             MCF_REG32(0xFC0BC00C)
#define MCF_SSI_CR              MCF_REG32(0xFC0BC010)
#define MCF_SSI_ISR             MCF_REG32(0xFC0BC014)
#define MCF_SSI_IER             MCF_REG32(0xFC0BC018)
#define MCF_SSI_TCR             MCF_REG32(0xFC0BC01C)
#define MCF_SSI_RCR             MCF_REG32(0xFC0BC020)
#define MCF_SSI_CCR             MCF_REG32(0xFC0BC024)
#define MCF_SSI_FCSR            MCF_REG32(0xFC0BC02C)
#define MCF_SSI_ACR             MCF_REG32(0xFC0BC038)
#define MCF_SSI_ACADD           MCF_REG32(0xFC0BC03C)
#define MCF_SSI_ACDAT           MCF_REG32(0xFC0BC040)
#define MCF_SSI_ATAG            MCF_REG32(0xFC0BC044)
#define MCF_SSI_TMASK           MCF_REG32(0xFC0BC048)
#define MCF_SSI_RMASK           MCF_REG32(0xFC0BC04C)

/* Parameterized register read/write macros for multiple registers */
#define MCF_SSI_TX(x)           MCF_REG32(0xFC0BC000+((x)*0x004))
#define MCF_SSI_RX(x)           MCF_REG32(0xFC0BC008+((x)*0x004))

/* Bit definitions and macros for TX group */
#define MCF_SSI_TX_SSI_TX(x)    (x)

/* Bit definitions and macros for TX0 */
#define MCF_SSI_TX0_SSI_TX(x)   (x)

/* Bit definitions and macros for TX1 */
#define MCF_SSI_TX1_SSI_TX(x)   (x)

/* Bit definitions and macros for RX group */
#define MCF_SSI_RX_SSI_RX(x)    (x)

/* Bit definitions and macros for RX0 */
#define MCF_SSI_RX0_SSI_RX(x)   (x)

/* Bit definitions and macros for RX1 */
#define MCF_SSI_RX1_SSI_RX(x)   (x)

/* Bit definitions and macros for CR */
#define MCF_SSI_CR_SSI_EN       (0x00000001)
#define MCF_SSI_CR_TE           (0x00000002)
#define MCF_SSI_CR_RE           (0x00000004)
#define MCF_SSI_CR_NET          (0x00000008)
#define MCF_SSI_CR_SYN          (0x00000010)
#define MCF_SSI_CR_I2S(x)       (((x)&0x00000003)<<5)
#define MCF_SSI_CR_MCE          (0x00000080)
#define MCF_SSI_CR_TCH          (0x00000100)
#define MCF_SSI_CR_CIS          (0x00000200)
#define MCF_SSI_CR_I2S_NORMAL   (0x00000000)
#define MCF_SSI_CR_I2S_MASTER   (0x00000020)
#define MCF_SSI_CR_I2S_SLAVE    (0x00000040)

/* Bit definitions and macros for ISR */
#define MCF_SSI_ISR_TFE0        (0x00000001)
#define MCF_SSI_ISR_TFE1        (0x00000002)
#define MCF_SSI_ISR_RFF0        (0x00000004)
#define MCF_SSI_ISR_RFF1        (0x00000008)
#define MCF_SSI_ISR_RLS         (0x00000010)
#define MCF_SSI_ISR_TLS         (0x00000020)
#define MCF_SSI_ISR_RFS         (0x00000040)
#define MCF_SSI_ISR_TFS         (0x00000080)
#define MCF_SSI_ISR_TUE0        (0x00000100)
#define MCF_SSI_ISR_TUE1        (0x00000200)
#define MCF_SSI_ISR_ROE0        (0x00000400)
#define MCF_SSI_ISR_ROE1        (0x00000800)
#define MCF_SSI_ISR_TDE0        (0x00001000)
#define MCF_SSI_ISR_TDE1        (0x00002000)
#define MCF_SSI_ISR_RDR0        (0x00004000)
#define MCF_SSI_ISR_RDR1        (0x00008000)
#define MCF_SSI_ISR_RXT         (0x00010000)
#define MCF_SSI_ISR_CMDDU       (0x00020000)
#define MCF_SSI_ISR_CMDAU       (0x00040000)

/* Bit definitions and macros for IER */
#define MCF_SSI_IER_TFE0        (0x00000001)
#define MCF_SSI_IER_TFE1        (0x00000002)
#define MCF_SSI_IER_RFF0        (0x00000004)
#define MCF_SSI_IER_RFF1        (0x00000008)
#define MCF_SSI_IER_RLS         (0x00000010)
#define MCF_SSI_IER_TLS         (0x00000020)
#define MCF_SSI_IER_RFS         (0x00000040)
#define MCF_SSI_IER_TFS         (0x00000080)
#define MCF_SSI_IER_TUE0        (0x00000100)
#define MCF_SSI_IER_TUE1        (0x00000200)
#define MCF_SSI_IER_ROE0        (0x00000400)
#define MCF_SSI_IER_ROE1        (0x00000800)
#define MCF_SSI_IER_TDE0        (0x00001000)
#define MCF_SSI_IER_TDE1        (0x00002000)
#define MCF_SSI_IER_RDR0        (0x00004000)
#define MCF_SSI_IER_RDR1        (0x00008000)
#define MCF_SSI_IER_RXT         (0x00010000)
#define MCF_SSI_IER_CMDU        (0x00020000)
#define MCF_SSI_IER_CMDAU       (0x00040000)
#define MCF_SSI_IER_TIE         (0x00080000)
#define MCF_SSI_IER_TDMAE       (0x00100000)
#define MCF_SSI_IER_RIE         (0x00200000)
#define MCF_SSI_IER_RDMAE       (0x00400000)

/* Bit definitions and macros for TCR */
#define MCF_SSI_TCR_TEFS        (0x00000001)
#define MCF_SSI_TCR_TFSL        (0x00000002)
#define MCF_SSI_TCR_TFSI        (0x00000004)
#define MCF_SSI_TCR_TSCKP       (0x00000008)
#define MCF_SSI_TCR_TSHFD       (0x00000010)
#define MCF_SSI_TCR_TXDIR       (0x00000020)
#define MCF_SSI_TCR_TFDIR       (0x00000040)
#define MCF_SSI_TCR_TFEN0       (0x00000080)
#define MCF_SSI_TCR_TFEN1       (0x00000100)
#define MCF_SSI_TCR_TXBIT0      (0x00000200)

/* Bit definitions and macros for RCR */
#define MCF_SSI_RCR_REFS        (0x00000001)
#define MCF_SSI_RCR_RFSL        (0x00000002)
#define MCF_SSI_RCR_RFSI        (0x00000004)
#define MCF_SSI_RCR_RSCKP       (0x00000008)
#define MCF_SSI_RCR_RSHFD       (0x00000010)
#define MCF_SSI_RCR_RFEN0       (0x00000080)
#define MCF_SSI_RCR_RFEN1       (0x00000100)
#define MCF_SSI_RCR_RXBIT0      (0x00000200)
#define MCF_SSI_RCR_RXEXT       (0x00000400)

/* Bit definitions and macros for CCR */
#define MCF_SSI_CCR_PM(x)       (((x)&0x000000FF))
#define MCF_SSI_CCR_DC(x)       (((x)&0x0000001F)<<8)
#define MCF_SSI_CCR_WL(x)       (((x)&0x0000000F)<<13)
#define MCF_SSI_CCR_PSR         (0x00020000)
#define MCF_SSI_CCR_DIV2        (0x00040000)

/* Bit definitions and macros for FCSR */
#define MCF_SSI_FCSR_TFWM0(x)       (((x)&0x0000000F))
#define MCF_SSI_FCSR_RFWM0(x)       (((x)&0x0000000F)<<4)
#define MCF_SSI_FCSR_TFCNT0(x)      (((x)&0x0000000F)<<8)
#define MCF_SSI_FCSR_RFCNT0(x)      (((x)&0x0000000F)<<12)
#define MCF_SSI_FCSR_TFWM1(x)       (((x)&0x0000000F)<<16)
#define MCF_SSI_FCSR_RFWM1(x)       (((x)&0x0000000F)<<20)
#define MCF_SSI_FCSR_TFCNT1(x)      (((x)&0x0000000F)<<24)
#define MCF_SSI_FCSR_RFCNT1(x)      (((x)&0x0000000F)<<28)

/* Bit definitions and macros for ACR */
#define MCF_SSI_ACR_AC97EN      (0x00000001)
#define MCF_SSI_ACR_FV          (0x00000002)
#define MCF_SSI_ACR_TIF         (0x00000004)
#define MCF_SSI_ACR_RD          (0x00000008)
#define MCF_SSI_ACR_WR          (0x00000010)
#define MCF_SSI_ACR_FRDIV(x)    (((x)&0x0000003F)<<5)

/* Bit definitions and macros for ACADD */
#define MCF_SSI_ACADD_SSI_ACADD(x)      (((x)&0x0007FFFF))

/* Bit definitions and macros for ACDAT */
#define MCF_SSI_ACDAT_SSI_ACDAT(x)      (((x)&0x0007FFFF))

/* Bit definitions and macros for ATAG */
#define MCF_SSI_ATAG_DDI_ATAG(x)    (((x)&0x0000FFFF))

/* Bit definitions and macros for TMASK */
#define MCF_SSI_TMASK_SSI_TMASK(x)      (x)

/* Bit definitions and macros for RMASK */
#define MCF_SSI_RMASK_SSI_RMASK(x)      (x)

/********************************************************************/

#endif /* __MCF5445X_SSI_H__ */

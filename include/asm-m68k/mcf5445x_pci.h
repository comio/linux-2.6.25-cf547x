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

#ifndef __MCF5445X_PCI_H__
#define __MCF5445X_PCI_H__

/*********************************************************************
*
* PCI Bus Controller (PCI)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_PCI_PCIIDR          MCF_REG32(0xFC0A8000)
#define MCF_PCI_PCISCR          MCF_REG32(0xFC0A8004)
#define MCF_PCI_PCICCRIR        MCF_REG32(0xFC0A8008)
#define MCF_PCI_PCICR1          MCF_REG32(0xFC0A800C)
#define MCF_PCI_PCIBAR0         MCF_REG32(0xFC0A8010)
#define MCF_PCI_PCIBAR1         MCF_REG32(0xFC0A8014)
#define MCF_PCI_PCIBAR2         MCF_REG32(0xFC0A8018)
#define MCF_PCI_PCIBAR3         MCF_REG32(0xFC0A801C)
#define MCF_PCI_PCIBAR4         MCF_REG32(0xFC0A8020)
#define MCF_PCI_PCIBAR5         MCF_REG32(0xFC0A8024)
#define MCF_PCI_PCISID          MCF_REG32(0xFC0A802C)
#define MCF_PCI_PCICR2          MCF_REG32(0xFC0A803C)
#define MCF_PCI_PCIGSCR         MCF_REG32(0xFC0A8060)
#define MCF_PCI_PCITBATR0A      MCF_REG32(0xFC0A8064)
#define MCF_PCI_PCITBATR1A      MCF_REG32(0xFC0A8068)
#define MCF_PCI_PCITCR          MCF_REG32(0xFC0A806C)
#define MCF_PCI_PCIIW0BTAR      MCF_REG32(0xFC0A8070)
#define MCF_PCI_PCIIW1BTAR      MCF_REG32(0xFC0A8074)
#define MCF_PCI_PCIIW2BTAR      MCF_REG32(0xFC0A8078)
#define MCF_PCI_PCIIWCR         MCF_REG32(0xFC0A8080)
#define MCF_PCI_PCIICR          MCF_REG32(0xFC0A8084)
#define MCF_PCI_PCIISR          MCF_REG32(0xFC0A8088)
#define MCF_PCI_PCITCR2         MCF_REG32(0xFC0A808C)
#define MCF_PCI_PCITBATR0       MCF_REG32(0xFC0A8090)
#define MCF_PCI_PCITBATR1       MCF_REG32(0xFC0A8094)
#define MCF_PCI_PCITBATR2       MCF_REG32(0xFC0A8098)
#define MCF_PCI_PCITBATR3       MCF_REG32(0xFC0A809C)
#define MCF_PCI_PCITBATR4       MCF_REG32(0xFC0A80A0)
#define MCF_PCI_PCITBATR5       MCF_REG32(0xFC0A80A4)
#define MCF_PCI_PCICAR          MCF_REG32(0xFC0A80F8)

/* Parameterized register read/write macros for multiple registers */
#define MCF_PCI_PCIIWBTAR(x)    MCF_REG32(0xFC0A8070+((x)*0x004))

/* Bit definitions and macros for PCIIDR */
#define MCF_PCI_PCIIDR_VENDORID(x)      (((x)&0x0000FFFF))
#define MCF_PCI_PCIIDR_DEVICEID(x)      (((x)&0x0000FFFF)<<16)

/* Bit definitions and macros for PCISCR */
#define MCF_PCI_PCISCR_M        (0x00000002)
#define MCF_PCI_PCISCR_B        (0x00000004)
#define MCF_PCI_PCISCR_SP       (0x00000008)
#define MCF_PCI_PCISCR_MW       (0x00000010)
#define MCF_PCI_PCISCR_PER      (0x00000040)
#define MCF_PCI_PCISCR_S        (0x00000100)
#define MCF_PCI_PCISCR_F        (0x00000200)
#define MCF_PCI_PCISCR_C        (0x00100000)
#define MCF_PCI_PCISCR_66M      (0x00200000)
#define MCF_PCI_PCISCR_R        (0x00400000)
#define MCF_PCI_PCISCR_FC       (0x00800000)
#define MCF_PCI_PCISCR_DP       (0x01000000)
#define MCF_PCI_PCISCR_DT(x)    (((x)&0x00000003)<<25)
#define MCF_PCI_PCISCR_TS       (0x08000000)
#define MCF_PCI_PCISCR_TR       (0x10000000)
#define MCF_PCI_PCISCR_MA       (0x20000000)
#define MCF_PCI_PCISCR_SE       (0x40000000)
#define MCF_PCI_PCISCR_PE       (0x80000000)

/* Bit definitions and macros for PCICCRIR */
#define MCF_PCI_PCICCRIR_REVID(x)       (((x)&0x000000FF))
#define MCF_PCI_PCICCRIR_CLASSCODE(x)   (((x)&0x00FFFFFF)<<8)

/* Bit definitions and macros for PCICR1 */
#define MCF_PCI_PCICR1_CACHELINESIZE(x)     (((x)&0x0000000F))
#define MCF_PCI_PCICR1_LATTIMER(x)          (((x)&0x000000FF)<<8)
#define MCF_PCI_PCICR1_HEADERTYPE(x)        (((x)&0x000000FF)<<16)
#define MCF_PCI_PCICR1_BIST(x)              (((x)&0x000000FF)<<24)

/* Bit definitions and macros for PCIBAR0 */
#define MCF_PCI_PCIBAR0_IO          (0x00000001)
#define MCF_PCI_PCIBAR0_RANGE(x)    (((x)&0x00000003)<<1)
#define MCF_PCI_PCIBAR0_PREF        (0x00000008)
#define MCF_PCI_PCIBAR0_BAR0(x)     (((x)&0x00003FFF)<<18)

/* Bit definitions and macros for PCIBAR1 */
#define MCF_PCI_PCIBAR1_IO          (0x00000001)
#define MCF_PCI_PCIBAR1_PREF        (0x00000008)
#define MCF_PCI_PCIBAR1_BAR1(x)     (((x)&0x00000FFF)<<20)

/* Bit definitions and macros for PCIBAR2 */
#define MCF_PCI_PCIBAR2_IO          (0x00000001)
#define MCF_PCI_PCIBAR2_RANGE(x)    (((x)&0x00000003)<<1)
#define MCF_PCI_PCIBAR2_PREF        (0x00000008)
#define MCF_PCI_PCIBAR2_BAR2(x)     (((x)&0x000003FF)<<22)

/* Bit definitions and macros for PCIBAR3 */
#define MCF_PCI_PCIBAR3_IO          (0x00000001)
#define MCF_PCI_PCIBAR3_PREF        (0x00000008)
#define MCF_PCI_PCIBAR3_BAR3(x)     (((x)&0x000000FF)<<24)

/* Bit definitions and macros for PCIBAR4 */
#define MCF_PCI_PCIBAR4_IO          (0x00000001)
#define MCF_PCI_PCIBAR4_RANGE(x)    (((x)&0x00000003)<<1)
#define MCF_PCI_PCIBAR4_PREF        (0x00000008)
#define MCF_PCI_PCIBAR4_BAR4(x)     (((x)&0x0000001F)<<27)

/* Bit definitions and macros for PCIBAR5 */
#define MCF_PCI_PCIBAR5_IO          (0x00000001)
#define MCF_PCI_PCIBAR5_PREF        (0x00000008)
#define MCF_PCI_PCIBAR5_BAR5(x)     (((x)&0x0000000F)<<28)

/* Bit definitions and macros for PCISID */
#define MCF_PCI_PCISID_VENDORID(x)      (((x)&0x0000FFFF))
#define MCF_PCI_PCISID_ID(x)            (((x)&0x0000FFFF)<<16)

/* Bit definitions and macros for PCICR2 */
#define MCF_PCI_PCICR2_INTLINE(x)   (((x)&0x000000FF))
#define MCF_PCI_PCICR2_INTPIN(x)    (((x)&0x000000FF)<<8)
#define MCF_PCI_PCICR2_MINGNT(x)    (((x)&0x000000FF)<<16)
#define MCF_PCI_PCICR2_MAXLAT(x)    (((x)&0x000000FF)<<24)

/* Bit definitions and macros for PCIGSCR */
#define MCF_PCI_PCIGSCR_PR      (0x00000001)
#define MCF_PCI_PCIGSCR_SEE     (0x00001000)
#define MCF_PCI_PCIGSCR_PEE     (0x00002000)
#define MCF_PCI_PCIGSCR_SE      (0x10000000)
#define MCF_PCI_PCIGSCR_PE      (0x20000000)

/* Bit definitions and macros for PCITBATR0A */
#define MCF_PCI_PCITBATR0A_EN           (0x00000001)
#define MCF_PCI_PCITBATR0A_BAT0(x)      (((x)&0x00003FFF)<<18)

/* Bit definitions and macros for PCITBATR1A */
#define MCF_PCI_PCITBATR1A_EN           (0x00000001)
#define MCF_PCI_PCITBATR1A_BAT1(x)      (((x)&0x00000FFF)<<20)

/* Bit definitions and macros for PCITCR */
#define MCF_PCI_PCITCR_WCT(x)   (((x)&0x000000FF))
#define MCF_PCI_PCITCR_WCD      (0x00000100)
#define MCF_PCI_PCITCR_P        (0x00010000)
#define MCF_PCI_PCITCR_PID      (0x00020000)
#define MCF_PCI_PCITCR_LD       (0x01000000)

/* Bit definitions and macros for PCIIWBTAR group */
#define MCF_PCI_PCIIWBTAR_WBA(x)    ((((x)&0xFF000000)))
#define MCF_PCI_PCIIWBTAR_WAM(x)    ((((x)&0xFF000000)
#define MCF_PCI_PCIIWBTAR_WTA(x)    ((((x)&0xFF000000)

/* Bit definitions and macros for PCIIW0BTAR */
#define MCF_PCI_PCIIW0BTAR_WBA(x)   ((((x)&0xFF000000)))
#define MCF_PCI_PCIIW0BTAR_WAM(x)   ((((x)&0xFF000000)
#define MCF_PCI_PCIIW0BTAR_WTA(x)   ((((x)&0xFF000000)

/* Bit definitions and macros for PCIIWCR */
#define MCF_PCI_PCIIWCR_WINCTRL2(x)             (((x)&0x0000000F)<<8)
#define MCF_PCI_PCIIWCR_WINCTRL1(x)             (((x)&0x0000000F)<<16)
#define MCF_PCI_PCIIWCR_WINCTRL0(x)             (((x)&0x0000000F)<<24)
#define MCF_PCI_PCIIWCR_WINCTRL0_ENABLE         (0x01000000)
#define MCF_PCI_PCIIWCR_WINCTRL0_MEMREAD        (0x01000000)
#define MCF_PCI_PCIIWCR_WINCTRL0_MEMRDLINE      (0x03000000)
#define MCF_PCI_PCIIWCR_WINCTRL0_MEMRDMUL       (0x05000000)
#define MCF_PCI_PCIIWCR_WINCTRL0_IO             (0x09000000)
#define MCF_PCI_PCIIWCR_WINCTRL1_MEMREAD        (0x00010000)
#define MCF_PCI_PCIIWCR_WINCTRL1_ENABLE         (0x00010000)
#define MCF_PCI_PCIIWCR_WINCTRL1_MEMRDLINE      (0x00030000)
#define MCF_PCI_PCIIWCR_WINCTRL1_MEMRDMUL       (0x00050000)
#define MCF_PCI_PCIIWCR_WINCTRL1_IO             (0x00090000)
#define MCF_PCI_PCIIWCR_WINCTRL2_ENABLE         (0x00000100)
#define MCF_PCI_PCIIWCR_WINCTRL2_MEMREAD        (0x00000100)
#define MCF_PCI_PCIIWCR_WINCTRL2_MEMRDLINE      (0x00000300)
#define MCF_PCI_PCIIWCR_WINCTRL2_MEMRDMUL       (0x00000500)
#define MCF_PCI_PCIIWCR_WINCTRL2_IO             (0x00000900)

/* Bit definitions and macros for PCIICR */
#define MCF_PCI_PCIICR_MAXRETRY(x)      (((x)&0x000000FF))
#define MCF_PCI_PCIICR_TAE              (0x01000000)
#define MCF_PCI_PCIICR_IAE              (0x02000000)
#define MCF_PCI_PCIICR_REE              (0x04000000)

/* Bit definitions and macros for PCIISR */
#define MCF_PCI_PCIISR_TA       (0x01000000)
#define MCF_PCI_PCIISR_IA       (0x02000000)
#define MCF_PCI_PCIISR_RE       (0x04000000)

/* Bit definitions and macros for PCITCR2 */
#define MCF_PCI_PCITCR2_CR      (0x00000001)
#define MCF_PCI_PCITCR2_B0E     (0x00000100)
#define MCF_PCI_PCITCR2_B1E     (0x00000200)
#define MCF_PCI_PCITCR2_B2E     (0x00000400)
#define MCF_PCI_PCITCR2_B3E     (0x00000800)
#define MCF_PCI_PCITCR2_B4E     (0x00001000)
#define MCF_PCI_PCITCR2_B5E     (0x00002000)

/* Bit definitions and macros for PCITBATR0 */
#define MCF_PCI_PCITBATR0_EN        (0x00000001)
#define MCF_PCI_PCITBATR0_BAT0(x)   (((x)&0x00003FFF)<<18)

/* Bit definitions and macros for PCITBATR1 */
#define MCF_PCI_PCITBATR1_EN        (0x00000001)
#define MCF_PCI_PCITBATR1_BAT1(x)   (((x)&0x00000FFF)<<20)

/* Bit definitions and macros for PCITBATR2 */
#define MCF_PCI_PCITBATR2_EN        (0x00000001)
#define MCF_PCI_PCITBATR2_BAT2(x)   (((x)&0x000003FF)<<22)

/* Bit definitions and macros for PCITBATR3 */
#define MCF_PCI_PCITBATR3_EN        (0x00000001)
#define MCF_PCI_PCITBATR3_BAT3(x)   (((x)&0x000000FF)<<24)

/* Bit definitions and macros for PCITBATR4 */
#define MCF_PCI_PCITBATR4_EN        (0x00000001)
#define MCF_PCI_PCITBATR4_BAT4(x)   (((x)&0x0000001F)<<27)

/* Bit definitions and macros for PCITBATR5 */
#define MCF_PCI_PCITBATR5_EN        (0x00000001)
#define MCF_PCI_PCITBATR5_BAT5(x)   (((x)&0x0000000F)<<28)

/* Bit definitions and macros for PCICAR */
#define MCF_PCI_PCICAR_DWORD(x)     ((x)&0x000000FC)
#define MCF_PCI_PCICAR_FUNCNUM(x)   (((x)&0x00000007)<<8)
#define MCF_PCI_PCICAR_DEVNUM(x)    (((x)&0x0000001F)<<11)
#define MCF_PCI_PCICAR_BUSNUM(x)    (((x)&0x000000FF)<<16)
#define MCF_PCI_PCICAR_E            (0x80000000)

/********************************************************************/

#endif /* __MCF5445X_PCI_H__ */

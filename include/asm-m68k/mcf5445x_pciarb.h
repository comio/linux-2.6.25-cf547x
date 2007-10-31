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

#ifndef __MCF5445X_PCIARB_H__
#define __MCF5445X_PCIARB_H__

/*********************************************************************
*
* PCI Arbiter Module (PCIARB)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_PCIARB_PACR         	MCF_REG32(0xFC0AC000)
#define MCF_PCIARB_PASR         	MCF_REG32(0xFC0AC004)

/* Bit definitions and macros for PACR */
#define MCF_PCIARB_PACR_INTMPRI         (0x00000001)
#define MCF_PCIARB_PACR_EXTMPRI(x)      (((x)&0x0000001F)<<1)
#define MCF_PCIARB_PACR_RA              (0x00008000)
#define MCF_PCIARB_PACR_INTMINTEN       (0x00010000)
#define MCF_PCIARB_PACR_EXTMINTEN(x)    (((x)&0x0000001F)<<17)
#define MCF_PCIARB_PACR_PKMD            (0x40000000)
#define MCF_PCIARB_PACR_DS              (0x80000000)

/* Bit definitions and macros for PASR */
#define MCF_PCIARB_PASR_ITLMBK      	(0x00010000)
#define MCF_PCIARB_PASR_EXTMBK(x)   	(((x)&0x0000001F)<<17)

/********************************************************************/

#endif /* __MCF5445X_PCIARB_H__ */

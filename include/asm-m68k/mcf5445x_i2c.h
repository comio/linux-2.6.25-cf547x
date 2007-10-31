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

#ifndef __MCF5445X_I2C_H__
#define __MCF5445X_I2C_H__

/*********************************************************************
*
* I2C Module (I2C)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_I2C_I2ADR	(volatile u8 *)(0xFC058000)
#define MCF_I2C_I2FDR	(volatile u8 *)(0xFC058004)
#define MCF_I2C_I2CR	(volatile u8 *)(0xFC058008)
#define MCF_I2C_I2SR	(volatile u8 *)(0xFC05800C)
#define MCF_I2C_I2DR	(volatile u8 *)(0xFC058010)

/* Bit definitions and macros for I2AR */
#define MCF_I2C_I2AR_ADR(x)	(((x)&0x7F)<<1)

/* Bit definitions and macros for I2FDR */
#define MCF_I2C_I2FDR_IC(x)	(((x)&0x3F))

/* Bit definitions and macros for I2DR */
#define MCF_I2C_I2DR_DATA(x)	(x)

/********************************************************************/

#endif /* __MCF5445X_I2C_H__ */

/*
    i2c-mcf.h - header file for i2c-mcf.c

    Copyright (c) 2005, Derek CL Cheung <derek.cheung@sympatico.ca>
                                        <http://www3.sympatico.ca/derek.cheung>

    Copyright (c) 2006-2007, emlix
			Sebastian Hess <sh@emlix.com>

    Copyright (c) 2006-2007 Freescale Semiconductor, Inc
			Yaroslav Vinogradov <yaroslav.vinogradov@freescale.com>
			Matt Waddel <Matt.Waddel@freescale.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

    Changes:
    v0.1	26 March 2005
        	Initial Release - developed on uClinux with 2.6.9 kernel
    v0.2	29 May 2006
		Modified to be more generic and added support for
		i2c_master_xfer
*/


#ifndef __I2C_MCF_H__
#define __I2C_MCF_H__

enum I2C_START_TYPE { FIRST_START, REPEAT_START };
enum I2C_ACK_TYPE { ACK, NACK};

/* Function prototypes */
static u32 coldfire_func(struct i2c_adapter *adapter);
static s32 coldfire_i2c_access(struct i2c_adapter *adap, u16 address,
                              unsigned short flags, char read_write,
                              u8 command, int size, union i2c_smbus_data *data);
static int coldfire_write_data(const u8 data);
static int coldfire_i2c_start(const char read_write, const u16 target_address, const enum I2C_START_TYPE i2c_start);
static int coldfire_read_data(u8 * const rxData, const enum I2C_ACK_TYPE ackType);
static int coldfire_i2c_master(struct i2c_adapter *adap,struct i2c_msg *msgs, int num);
void dumpReg(char *, u16 addr, u8 data);

#define MCF_I2C_I2ADR_ADDR(x)	(((x)&0x7F)<<0x01)
#define MCF_I2C_I2FDR_IC(x)	(((x)&0x3F))

/* I2C Control Register */
#define MCF_I2C_I2CR_IEN	(0x80)	/* I2C enable */
#define MCF_I2C_I2CR_IIEN	(0x40)	/* interrupt enable */
#define MCF_I2C_I2CR_MSTA	(0x20)	/* master/slave mode */
#define MCF_I2C_I2CR_MTX	(0x10)	/* transmit/receive mode */
#define MCF_I2C_I2CR_TXAK	(0x08)	/* transmit acknowledge enable */
#define MCF_I2C_I2CR_RSTA	(0x04)	/* repeat start */

/* I2C Status Register */
#define MCF_I2C_I2SR_ICF	(0x80)	/* data transfer bit */
#define MCF_I2C_I2SR_IAAS	(0x40)	/* I2C addressed as a slave */
#define MCF_I2C_I2SR_IBB	(0x20)	/* I2C bus busy */
#define MCF_I2C_I2SR_IAL	(0x10)	/* aribitration lost */
#define MCF_I2C_I2SR_SRW	(0x04)	/* slave read/write */
#define MCF_I2C_I2SR_IIF	(0x02)	/* I2C interrupt */
#define MCF_I2C_I2SR_RXAK	(0x01)	/* received acknowledge */

/********************************************************************/
#endif /*  __I2C_MCF_H__ */

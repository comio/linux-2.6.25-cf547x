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

#ifndef __MCF5445X_EPORT_H__
#define __MCF5445X_EPORT_H__

/*********************************************************************
*
* Edge Port Module (EPORT)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_EPORT_EPPAR         MCF_REG16(0xFC094000)
#define MCF_EPORT_EPDDR         MCF_REG08(0xFC094002)
#define MCF_EPORT_EPIER         MCF_REG08(0xFC094003)
#define MCF_EPORT_EPDR          MCF_REG08(0xFC094004)
#define MCF_EPORT_EPPDR         MCF_REG08(0xFC094005)
#define MCF_EPORT_EPFR          MCF_REG08(0xFC094006)

/* Bit definitions and macros for EPPAR */
#define MCF_EPORT_EPPAR_EPPA1(x)        (((x)&0x0003)<<2)
#define MCF_EPORT_EPPAR_EPPA2(x)        (((x)&0x0003)<<4)
#define MCF_EPORT_EPPAR_EPPA3(x)        (((x)&0x0003)<<6)
#define MCF_EPORT_EPPAR_EPPA4(x)        (((x)&0x0003)<<8)
#define MCF_EPORT_EPPAR_EPPA5(x)        (((x)&0x0003)<<10)
#define MCF_EPORT_EPPAR_EPPA6(x)        (((x)&0x0003)<<12)
#define MCF_EPORT_EPPAR_EPPA7(x)        (((x)&0x0003)<<14)
#define MCF_EPORT_EPPAR_LEVEL           (0)
#define MCF_EPORT_EPPAR_RISING          (1)
#define MCF_EPORT_EPPAR_FALLING         (2)
#define MCF_EPORT_EPPAR_BOTH            (3)
#define MCF_EPORT_EPPAR_EPPA7_LEVEL     (0x0000)
#define MCF_EPORT_EPPAR_EPPA7_RISING    (0x4000)
#define MCF_EPORT_EPPAR_EPPA7_FALLING   (0x8000)
#define MCF_EPORT_EPPAR_EPPA7_BOTH      (0xC000)
#define MCF_EPORT_EPPAR_EPPA6_LEVEL     (0x0000)
#define MCF_EPORT_EPPAR_EPPA6_RISING    (0x1000)
#define MCF_EPORT_EPPAR_EPPA6_FALLING   (0x2000)
#define MCF_EPORT_EPPAR_EPPA6_BOTH      (0x3000)
#define MCF_EPORT_EPPAR_EPPA5_LEVEL     (0x0000)
#define MCF_EPORT_EPPAR_EPPA5_RISING    (0x0400)
#define MCF_EPORT_EPPAR_EPPA5_FALLING   (0x0800)
#define MCF_EPORT_EPPAR_EPPA5_BOTH      (0x0C00)
#define MCF_EPORT_EPPAR_EPPA4_LEVEL     (0x0000)
#define MCF_EPORT_EPPAR_EPPA4_RISING    (0x0100)
#define MCF_EPORT_EPPAR_EPPA4_FALLING   (0x0200)
#define MCF_EPORT_EPPAR_EPPA4_BOTH      (0x0300)
#define MCF_EPORT_EPPAR_EPPA3_LEVEL     (0x0000)
#define MCF_EPORT_EPPAR_EPPA3_RISING    (0x0040)
#define MCF_EPORT_EPPAR_EPPA3_FALLING   (0x0080)
#define MCF_EPORT_EPPAR_EPPA3_BOTH      (0x00C0)
#define MCF_EPORT_EPPAR_EPPA2_LEVEL     (0x0000)
#define MCF_EPORT_EPPAR_EPPA2_RISING    (0x0010)
#define MCF_EPORT_EPPAR_EPPA2_FALLING   (0x0020)
#define MCF_EPORT_EPPAR_EPPA2_BOTH      (0x0030)
#define MCF_EPORT_EPPAR_EPPA1_LEVEL     (0x0000)
#define MCF_EPORT_EPPAR_EPPA1_RISING    (0x0004)
#define MCF_EPORT_EPPAR_EPPA1_FALLING   (0x0008)
#define MCF_EPORT_EPPAR_EPPA1_BOTH      (0x000C)

/* Bit definitions and macros for EPDDR */
#define MCF_EPORT_EPDDR_EPDD1   (0x02)
#define MCF_EPORT_EPDDR_EPDD2   (0x04)
#define MCF_EPORT_EPDDR_EPDD3   (0x08)
#define MCF_EPORT_EPDDR_EPDD4   (0x10)
#define MCF_EPORT_EPDDR_EPDD5   (0x20)
#define MCF_EPORT_EPDDR_EPDD6   (0x40)
#define MCF_EPORT_EPDDR_EPDD7   (0x80)

/* Bit definitions and macros for EPIER */
#define MCF_EPORT_EPIER_EPIE1   (0x02)
#define MCF_EPORT_EPIER_EPIE2   (0x04)
#define MCF_EPORT_EPIER_EPIE3   (0x08)
#define MCF_EPORT_EPIER_EPIE4   (0x10)
#define MCF_EPORT_EPIER_EPIE5   (0x20)
#define MCF_EPORT_EPIER_EPIE6   (0x40)
#define MCF_EPORT_EPIER_EPIE7   (0x80)

/* Bit definitions and macros for EPDR */
#define MCF_EPORT_EPDR_EPD1     (0x02)
#define MCF_EPORT_EPDR_EPD2     (0x04)
#define MCF_EPORT_EPDR_EPD3     (0x08)
#define MCF_EPORT_EPDR_EPD4     (0x10)
#define MCF_EPORT_EPDR_EPD5     (0x20)
#define MCF_EPORT_EPDR_EPD6     (0x40)
#define MCF_EPORT_EPDR_EPD7     (0x80)

/* Bit definitions and macros for EPPDR */
#define MCF_EPORT_EPPDR_EPPD1   (0x02)
#define MCF_EPORT_EPPDR_EPPD2   (0x04)
#define MCF_EPORT_EPPDR_EPPD3   (0x08)
#define MCF_EPORT_EPPDR_EPPD4   (0x10)
#define MCF_EPORT_EPPDR_EPPD5   (0x20)
#define MCF_EPORT_EPPDR_EPPD6   (0x40)
#define MCF_EPORT_EPPDR_EPPD7   (0x80)

/* Bit definitions and macros for EPFR */
#define MCF_EPORT_EPFR_EPF1     (0x02)
#define MCF_EPORT_EPFR_EPF2     (0x04)
#define MCF_EPORT_EPFR_EPF3     (0x08)
#define MCF_EPORT_EPFR_EPF4     (0x10)
#define MCF_EPORT_EPFR_EPF5     (0x20)
#define MCF_EPORT_EPFR_EPF6     (0x40)
#define MCF_EPORT_EPFR_EPF7     (0x80)

/********************************************************************/

#endif /* __MCF5445X_EPORT_H__ */

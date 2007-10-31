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

#ifndef __MCF5445X_INTC_H__
#define __MCF5445X_INTC_H__

/*********************************************************************
*
* Interrupt Controller (INTC)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_INTC0_IPRH          MCF_REG32(0xFC048000)
#define MCF_INTC0_IPRL          MCF_REG32(0xFC048004)
#define MCF_INTC0_IMRH          MCF_REG32(0xFC048008)
#define MCF_INTC0_IMRL          MCF_REG32(0xFC04800C)
#define MCF_INTC0_INTFRCH       MCF_REG32(0xFC048010)
#define MCF_INTC0_INTFRCL       MCF_REG32(0xFC048014)
#define MCF_INTC0_ICONFIG       MCF_REG16(0xFC04801A)
#define MCF_INTC0_SIMR          MCF_REG08(0xFC04801C)
#define MCF_INTC0_CIMR          MCF_REG08(0xFC04801D)
#define MCF_INTC0_CLMASK        MCF_REG08(0xFC04801E)
#define MCF_INTC0_SLMASK        MCF_REG08(0xFC04801F)
#define MCF_INTC0_ICR1          MCF_REG08(0xFC048041)
#define MCF_INTC0_ICR2          MCF_REG08(0xFC048042)
#define MCF_INTC0_ICR3          MCF_REG08(0xFC048043)
#define MCF_INTC0_ICR4          MCF_REG08(0xFC048044)
#define MCF_INTC0_ICR5          MCF_REG08(0xFC048045)
#define MCF_INTC0_ICR6          MCF_REG08(0xFC048046)
#define MCF_INTC0_ICR7          MCF_REG08(0xFC048047)
#define MCF_INTC0_ICR8          MCF_REG08(0xFC048048)
#define MCF_INTC0_ICR9          MCF_REG08(0xFC048049)
#define MCF_INTC0_ICR10         MCF_REG08(0xFC04804A)
#define MCF_INTC0_ICR11         MCF_REG08(0xFC04804B)
#define MCF_INTC0_ICR12         MCF_REG08(0xFC04804C)
#define MCF_INTC0_ICR13         MCF_REG08(0xFC04804D)
#define MCF_INTC0_ICR14         MCF_REG08(0xFC04804E)
#define MCF_INTC0_ICR15         MCF_REG08(0xFC04804F)
#define MCF_INTC0_ICR16         MCF_REG08(0xFC048050)
#define MCF_INTC0_ICR17         MCF_REG08(0xFC048051)
#define MCF_INTC0_ICR18         MCF_REG08(0xFC048052)
#define MCF_INTC0_ICR19         MCF_REG08(0xFC048053)
#define MCF_INTC0_ICR20         MCF_REG08(0xFC048054)
#define MCF_INTC0_ICR21         MCF_REG08(0xFC048055)
#define MCF_INTC0_ICR22         MCF_REG08(0xFC048056)
#define MCF_INTC0_ICR23         MCF_REG08(0xFC048057)
#define MCF_INTC0_ICR24         MCF_REG08(0xFC048058)
#define MCF_INTC0_ICR25         MCF_REG08(0xFC048059)
#define MCF_INTC0_ICR26         MCF_REG08(0xFC04805A)
#define MCF_INTC0_ICR27         MCF_REG08(0xFC04805B)
#define MCF_INTC0_ICR28         MCF_REG08(0xFC04805C)
#define MCF_INTC0_ICR29         MCF_REG08(0xFC04805D)
#define MCF_INTC0_ICR30         MCF_REG08(0xFC04805E)
#define MCF_INTC0_ICR31         MCF_REG08(0xFC04805F)
#define MCF_INTC0_ICR32         MCF_REG08(0xFC048060)
#define MCF_INTC0_ICR33         MCF_REG08(0xFC048061)
#define MCF_INTC0_ICR34         MCF_REG08(0xFC048062)
#define MCF_INTC0_ICR35         MCF_REG08(0xFC048063)
#define MCF_INTC0_ICR36         MCF_REG08(0xFC048064)
#define MCF_INTC0_ICR37         MCF_REG08(0xFC048065)
#define MCF_INTC0_ICR38         MCF_REG08(0xFC048066)
#define MCF_INTC0_ICR39         MCF_REG08(0xFC048067)
#define MCF_INTC0_ICR40         MCF_REG08(0xFC048068)
#define MCF_INTC0_ICR41         MCF_REG08(0xFC048069)
#define MCF_INTC0_ICR42         MCF_REG08(0xFC04806A)
#define MCF_INTC0_ICR43         MCF_REG08(0xFC04806B)
#define MCF_INTC0_ICR44         MCF_REG08(0xFC04806C)
#define MCF_INTC0_ICR45         MCF_REG08(0xFC04806D)
#define MCF_INTC0_ICR46         MCF_REG08(0xFC04806E)
#define MCF_INTC0_ICR47         MCF_REG08(0xFC04806F)
#define MCF_INTC0_ICR48         MCF_REG08(0xFC048070)
#define MCF_INTC0_ICR49         MCF_REG08(0xFC048071)
#define MCF_INTC0_ICR50         MCF_REG08(0xFC048072)
#define MCF_INTC0_ICR51         MCF_REG08(0xFC048073)
#define MCF_INTC0_ICR52         MCF_REG08(0xFC048074)
#define MCF_INTC0_ICR53         MCF_REG08(0xFC048075)
#define MCF_INTC0_ICR54         MCF_REG08(0xFC048076)
#define MCF_INTC0_ICR55         MCF_REG08(0xFC048077)
#define MCF_INTC0_ICR56         MCF_REG08(0xFC048078)
#define MCF_INTC0_ICR57         MCF_REG08(0xFC048079)
#define MCF_INTC0_ICR58         MCF_REG08(0xFC04807A)
#define MCF_INTC0_ICR59         MCF_REG08(0xFC04807B)
#define MCF_INTC0_ICR60         MCF_REG08(0xFC04807C)
#define MCF_INTC0_ICR61         MCF_REG08(0xFC04807D)
#define MCF_INTC0_ICR62         MCF_REG08(0xFC04807E)
#define MCF_INTC0_ICR63         MCF_REG08(0xFC04807F)
#define MCF_INTC0_SWIACK        MCF_REG08(0xFC0480E0)
#define MCF_INTC0_L1IACK        MCF_REG08(0xFC0480E4)
#define MCF_INTC0_L2IACK        MCF_REG08(0xFC0480E8)
#define MCF_INTC0_L3IACK        MCF_REG08(0xFC0480EC)
#define MCF_INTC0_L4IACK        MCF_REG08(0xFC0480F0)
#define MCF_INTC0_L5IACK        MCF_REG08(0xFC0480F4)
#define MCF_INTC0_L6IACK        MCF_REG08(0xFC0480F8)
#define MCF_INTC0_L7IACK        MCF_REG08(0xFC0480FC)
#define MCF_INTC1_IPRH          MCF_REG32(0xFC04C000)
#define MCF_INTC1_IPRL          MCF_REG32(0xFC04C004)
#define MCF_INTC1_IMRH          MCF_REG32(0xFC04C008)
#define MCF_INTC1_IMRL          MCF_REG32(0xFC04C00C)
#define MCF_INTC1_INTFRCH       MCF_REG32(0xFC04C010)
#define MCF_INTC1_INTFRCL       MCF_REG32(0xFC04C014)
#define MCF_INTC1_ICONFIG       MCF_REG16(0xFC04C01A)
#define MCF_INTC1_SIMR          MCF_REG08(0xFC04C01C)
#define MCF_INTC1_CIMR          MCF_REG08(0xFC04C01D)
#define MCF_INTC1_CLMASK        MCF_REG08(0xFC04C01E)
#define MCF_INTC1_SLMASK        MCF_REG08(0xFC04C01F)
#define MCF_INTC1_ICR1          MCF_REG08(0xFC04C041)
#define MCF_INTC1_ICR2          MCF_REG08(0xFC04C042)
#define MCF_INTC1_ICR3          MCF_REG08(0xFC04C043)
#define MCF_INTC1_ICR4          MCF_REG08(0xFC04C044)
#define MCF_INTC1_ICR5          MCF_REG08(0xFC04C045)
#define MCF_INTC1_ICR6          MCF_REG08(0xFC04C046)
#define MCF_INTC1_ICR7          MCF_REG08(0xFC04C047)
#define MCF_INTC1_ICR8          MCF_REG08(0xFC04C048)
#define MCF_INTC1_ICR9          MCF_REG08(0xFC04C049)
#define MCF_INTC1_ICR10         MCF_REG08(0xFC04C04A)
#define MCF_INTC1_ICR11         MCF_REG08(0xFC04C04B)
#define MCF_INTC1_ICR12         MCF_REG08(0xFC04C04C)
#define MCF_INTC1_ICR13         MCF_REG08(0xFC04C04D)
#define MCF_INTC1_ICR14         MCF_REG08(0xFC04C04E)
#define MCF_INTC1_ICR15         MCF_REG08(0xFC04C04F)
#define MCF_INTC1_ICR16         MCF_REG08(0xFC04C050)
#define MCF_INTC1_ICR17         MCF_REG08(0xFC04C051)
#define MCF_INTC1_ICR18         MCF_REG08(0xFC04C052)
#define MCF_INTC1_ICR19         MCF_REG08(0xFC04C053)
#define MCF_INTC1_ICR20         MCF_REG08(0xFC04C054)
#define MCF_INTC1_ICR21         MCF_REG08(0xFC04C055)
#define MCF_INTC1_ICR22         MCF_REG08(0xFC04C056)
#define MCF_INTC1_ICR23         MCF_REG08(0xFC04C057)
#define MCF_INTC1_ICR24         MCF_REG08(0xFC04C058)
#define MCF_INTC1_ICR25         MCF_REG08(0xFC04C059)
#define MCF_INTC1_ICR26         MCF_REG08(0xFC04C05A)
#define MCF_INTC1_ICR27         MCF_REG08(0xFC04C05B)
#define MCF_INTC1_ICR28         MCF_REG08(0xFC04C05C)
#define MCF_INTC1_ICR29         MCF_REG08(0xFC04C05D)
#define MCF_INTC1_ICR30         MCF_REG08(0xFC04C05E)
#define MCF_INTC1_ICR31         MCF_REG08(0xFC04C05F)
#define MCF_INTC1_ICR32         MCF_REG08(0xFC04C060)
#define MCF_INTC1_ICR33         MCF_REG08(0xFC04C061)
#define MCF_INTC1_ICR34         MCF_REG08(0xFC04C062)
#define MCF_INTC1_ICR35         MCF_REG08(0xFC04C063)
#define MCF_INTC1_ICR36         MCF_REG08(0xFC04C064)
#define MCF_INTC1_ICR37         MCF_REG08(0xFC04C065)
#define MCF_INTC1_ICR38         MCF_REG08(0xFC04C066)
#define MCF_INTC1_ICR39         MCF_REG08(0xFC04C067)
#define MCF_INTC1_ICR40         MCF_REG08(0xFC04C068)
#define MCF_INTC1_ICR41         MCF_REG08(0xFC04C069)
#define MCF_INTC1_ICR42         MCF_REG08(0xFC04C06A)
#define MCF_INTC1_ICR43         MCF_REG08(0xFC04C06B)
#define MCF_INTC1_ICR44         MCF_REG08(0xFC04C06C)
#define MCF_INTC1_ICR45         MCF_REG08(0xFC04C06D)
#define MCF_INTC1_ICR46         MCF_REG08(0xFC04C06E)
#define MCF_INTC1_ICR47         MCF_REG08(0xFC04C06F)
#define MCF_INTC1_ICR48         MCF_REG08(0xFC04C070)
#define MCF_INTC1_ICR49         MCF_REG08(0xFC04C071)
#define MCF_INTC1_ICR50         MCF_REG08(0xFC04C072)
#define MCF_INTC1_ICR51         MCF_REG08(0xFC04C073)
#define MCF_INTC1_ICR52         MCF_REG08(0xFC04C074)
#define MCF_INTC1_ICR53         MCF_REG08(0xFC04C075)
#define MCF_INTC1_ICR54         MCF_REG08(0xFC04C076)
#define MCF_INTC1_ICR55         MCF_REG08(0xFC04C077)
#define MCF_INTC1_ICR56         MCF_REG08(0xFC04C078)
#define MCF_INTC1_ICR57         MCF_REG08(0xFC04C079)
#define MCF_INTC1_ICR58         MCF_REG08(0xFC04C07A)
#define MCF_INTC1_ICR59         MCF_REG08(0xFC04C07B)
#define MCF_INTC1_ICR60         MCF_REG08(0xFC04C07C)
#define MCF_INTC1_ICR61         MCF_REG08(0xFC04C07D)
#define MCF_INTC1_ICR62         MCF_REG08(0xFC04C07E)
#define MCF_INTC1_ICR63         MCF_REG08(0xFC04C07F)
#define MCF_INTC1_SWIACK        MCF_REG08(0xFC04C0E0)
#define MCF_INTC1_L1IACK        MCF_REG08(0xFC04C0E4)
#define MCF_INTC1_L2IACK        MCF_REG08(0xFC04C0E8)
#define MCF_INTC1_L3IACK        MCF_REG08(0xFC04C0EC)
#define MCF_INTC1_L4IACK        MCF_REG08(0xFC04C0F0)
#define MCF_INTC1_L5IACK        MCF_REG08(0xFC04C0F4)
#define MCF_INTC1_L6IACK        MCF_REG08(0xFC04C0F8
#define MCF_INTC1_L7IACK        MCF_REG08(0xFC04C0FC)

/* Parameterized register read/write macros for multiple registers */
#define MCF_INTC0_ICR(x)        MCF_REG08(0xFC048041+((x-1)*0x001))
#define MCF_INTC0_LIACK(x)      MCF_REG08(0xFC0480E4+((x-1)*0x004))
#define MCF_INTC1_ICR(x)        MCF_REG08(0xFC04C041+((x-1)*0x001))
#define MCF_INTC1_LIACK(x)      MCF_REG08(0xFC04C0E4+((x-1)*0x004))

/* Parameterized register read/write macros for multiple modules */
#define MCF_INTC_IPRH(x)        MCF_REG32(0xFC048000+((x)*0x4000))
#define MCF_INTC_IPRL(x)        MCF_REG32(0xFC048004+((x)*0x4000))
#define MCF_INTC_IMRH(x)        MCF_REG32(0xFC048008+((x)*0x4000))
#define MCF_INTC_IMRL(x)        MCF_REG32(0xFC04800C+((x)*0x4000))
#define MCF_INTC_INTFRCH(x)     MCF_REG32(0xFC048010+((x)*0x4000))
#define MCF_INTC_INTFRCL(x)     MCF_REG32(0xFC048014+((x)*0x4000))
#define MCF_INTC_ICONFIG(x)     MCF_REG16(0xFC04801A+((x)*0x4000))
#define MCF_INTC_SIMR(x)        MCF_REG08(0xFC04801C+((x)*0x4000))
#define MCF_INTC_CIMR(x)        MCF_REG08(0xFC04801D+((x)*0x4000))
#define MCF_INTC_CLMASK(x)      MCF_REG08(0xFC04801E+((x)*0x4000))
#define MCF_INTC_SLMASK(x)      MCF_REG08(0xFC04801F+((x)*0x4000))
#define MCF_INTC_ICR1(x)        MCF_REG08(0xFC048041+((x)*0x4000))
#define MCF_INTC_ICR2(x)        MCF_REG08(0xFC048042+((x)*0x4000))
#define MCF_INTC_ICR3(x)        MCF_REG08(0xFC048043+((x)*0x4000))
#define MCF_INTC_ICR4(x)        MCF_REG08(0xFC048044+((x)*0x4000))
#define MCF_INTC_ICR5(x)        MCF_REG08(0xFC048045+((x)*0x4000))
#define MCF_INTC_ICR6(x)        MCF_REG08(0xFC048046+((x)*0x4000))
#define MCF_INTC_ICR7(x)        MCF_REG08(0xFC048047+((x)*0x4000))
#define MCF_INTC_ICR8(x)        MCF_REG08(0xFC048048+((x)*0x4000))
#define MCF_INTC_ICR9(x)        MCF_REG08(0xFC048049+((x)*0x4000))
#define MCF_INTC_ICR10(x)       MCF_REG08(0xFC04804A+((x)*0x4000))
#define MCF_INTC_ICR11(x)       MCF_REG08(0xFC04804B+((x)*0x4000))
#define MCF_INTC_ICR12(x)       MCF_REG08(0xFC04804C+((x)*0x4000))
#define MCF_INTC_ICR13(x)       MCF_REG08(0xFC04804D+((x)*0x4000))
#define MCF_INTC_ICR14(x)       MCF_REG08(0xFC04804E+((x)*0x4000))
#define MCF_INTC_ICR15(x)       MCF_REG08(0xFC04804F+((x)*0x4000))
#define MCF_INTC_ICR16(x)       MCF_REG08(0xFC048050+((x)*0x4000))
#define MCF_INTC_ICR17(x)       MCF_REG08(0xFC048051+((x)*0x4000))
#define MCF_INTC_ICR18(x)       MCF_REG08(0xFC048052+((x)*0x4000))
#define MCF_INTC_ICR19(x)       MCF_REG08(0xFC048053+((x)*0x4000))
#define MCF_INTC_ICR20(x)       MCF_REG08(0xFC048054+((x)*0x4000))
#define MCF_INTC_ICR21(x)       MCF_REG08(0xFC048055+((x)*0x4000))
#define MCF_INTC_ICR22(x)       MCF_REG08(0xFC048056+((x)*0x4000))
#define MCF_INTC_ICR23(x)       MCF_REG08(0xFC048057+((x)*0x4000))
#define MCF_INTC_ICR24(x)       MCF_REG08(0xFC048058+((x)*0x4000))
#define MCF_INTC_ICR25(x)       MCF_REG08(0xFC048059+((x)*0x4000))
#define MCF_INTC_ICR26(x)       MCF_REG08(0xFC04805A+((x)*0x4000))
#define MCF_INTC_ICR27(x)       MCF_REG08(0xFC04805B+((x)*0x4000))
#define MCF_INTC_ICR28(x)       MCF_REG08(0xFC04805C+((x)*0x4000))
#define MCF_INTC_ICR29(x)       MCF_REG08(0xFC04805D+((x)*0x4000))
#define MCF_INTC_ICR30(x)       MCF_REG08(0xFC04805E+((x)*0x4000))
#define MCF_INTC_ICR31(x)       MCF_REG08(0xFC04805F+((x)*0x4000))
#define MCF_INTC_ICR32(x)       MCF_REG08(0xFC048060+((x)*0x4000))
#define MCF_INTC_ICR33(x)       MCF_REG08(0xFC048061+((x)*0x4000))
#define MCF_INTC_ICR34(x)       MCF_REG08(0xFC048062+((x)*0x4000))
#define MCF_INTC_ICR35(x)       MCF_REG08(0xFC048063+((x)*0x4000))
#define MCF_INTC_ICR36(x)       MCF_REG08(0xFC048064+((x)*0x4000))
#define MCF_INTC_ICR37(x)       MCF_REG08(0xFC048065+((x)*0x4000))
#define MCF_INTC_ICR38(x)       MCF_REG08(0xFC048066+((x)*0x4000))
#define MCF_INTC_ICR39(x)       MCF_REG08(0xFC048067+((x)*0x4000))
#define MCF_INTC_ICR40(x)       MCF_REG08(0xFC048068+((x)*0x4000))
#define MCF_INTC_ICR41(x)       MCF_REG08(0xFC048069+((x)*0x4000))
#define MCF_INTC_ICR42(x)       MCF_REG08(0xFC04806A+((x)*0x4000))
#define MCF_INTC_ICR43(x)       MCF_REG08(0xFC04806B+((x)*0x4000))
#define MCF_INTC_ICR44(x)       MCF_REG08(0xFC04806C+((x)*0x4000))
#define MCF_INTC_ICR45(x)       MCF_REG08(0xFC04806D+((x)*0x4000))
#define MCF_INTC_ICR46(x)       MCF_REG08(0xFC04806E+((x)*0x4000))
#define MCF_INTC_ICR47(x)       MCF_REG08(0xFC04806F+((x)*0x4000))
#define MCF_INTC_ICR48(x)       MCF_REG08(0xFC048070+((x)*0x4000))
#define MCF_INTC_ICR49(x)       MCF_REG08(0xFC048071+((x)*0x4000))
#define MCF_INTC_ICR50(x)       MCF_REG08(0xFC048072+((x)*0x4000))
#define MCF_INTC_ICR51(x)       MCF_REG08(0xFC048073+((x)*0x4000))
#define MCF_INTC_ICR52(x)       MCF_REG08(0xFC048074+((x)*0x4000))
#define MCF_INTC_ICR53(x)       MCF_REG08(0xFC048075+((x)*0x4000))
#define MCF_INTC_ICR54(x)       MCF_REG08(0xFC048076+((x)*0x4000))
#define MCF_INTC_ICR55(x)       MCF_REG08(0xFC048077+((x)*0x4000))
#define MCF_INTC_ICR56(x)       MCF_REG08(0xFC048078+((x)*0x4000))
#define MCF_INTC_ICR57(x)       MCF_REG08(0xFC048079+((x)*0x4000))
#define MCF_INTC_ICR58(x)       MCF_REG08(0xFC04807A+((x)*0x4000))
#define MCF_INTC_ICR59(x)       MCF_REG08(0xFC04807B+((x)*0x4000))
#define MCF_INTC_ICR60(x)       MCF_REG08(0xFC04807C+((x)*0x4000))
#define MCF_INTC_ICR61(x)       MCF_REG08(0xFC04807D+((x)*0x4000))
#define MCF_INTC_ICR62(x)       MCF_REG08(0xFC04807E+((x)*0x4000))
#define MCF_INTC_ICR63(x)       MCF_REG08(0xFC04807F+((x)*0x4000))
#define MCF_INTC_SWIACK(x)      MCF_REG08(0xFC0480E0+((x)*0x4000))
#define MCF_INTC_L1IACK(x)      MCF_REG08(0xFC0480E4+((x)*0x4000))
#define MCF_INTC_L2IACK(x)      MCF_REG08(0xFC0480E8+((x)*0x4000))
#define MCF_INTC_L3IACK(x)      MCF_REG08(0xFC0480EC+((x)*0x4000))
#define MCF_INTC_L4IACK(x)      MCF_REG08(0xFC0480F0+((x)*0x4000))
#define MCF_INTC_L5IACK(x)      MCF_REG08(0xFC0480F4+((x)*0x4000))
#define MCF_INTC_L6IACK(x)      MCF_REG08(0xFC0480F8+((x)*0x4000))
#define MCF_INTC_L7IACK(x)      MCF_REG08(0xFC0480FC+((x)*0x4000))

/* Bit definitions and macros for IPRH */
#define MCF_INTC_IPRH_INT32     (0x00000001)
#define MCF_INTC_IPRH_INT33     (0x00000002)
#define MCF_INTC_IPRH_INT34     (0x00000004)
#define MCF_INTC_IPRH_INT35     (0x00000008)
#define MCF_INTC_IPRH_INT36     (0x00000010)
#define MCF_INTC_IPRH_INT37     (0x00000020)
#define MCF_INTC_IPRH_INT38     (0x00000040)
#define MCF_INTC_IPRH_INT39     (0x00000080)
#define MCF_INTC_IPRH_INT40     (0x00000100)
#define MCF_INTC_IPRH_INT41     (0x00000200)
#define MCF_INTC_IPRH_INT42     (0x00000400)
#define MCF_INTC_IPRH_INT43     (0x00000800)
#define MCF_INTC_IPRH_INT44     (0x00001000)
#define MCF_INTC_IPRH_INT45     (0x00002000)
#define MCF_INTC_IPRH_INT46     (0x00004000)
#define MCF_INTC_IPRH_INT47     (0x00008000)
#define MCF_INTC_IPRH_INT48     (0x00010000)
#define MCF_INTC_IPRH_INT49     (0x00020000)
#define MCF_INTC_IPRH_INT50     (0x00040000)
#define MCF_INTC_IPRH_INT51     (0x00080000)
#define MCF_INTC_IPRH_INT52     (0x00100000)
#define MCF_INTC_IPRH_INT53     (0x00200000)
#define MCF_INTC_IPRH_INT54     (0x00400000)
#define MCF_INTC_IPRH_INT55     (0x00800000)
#define MCF_INTC_IPRH_INT56     (0x01000000)
#define MCF_INTC_IPRH_INT57     (0x02000000)
#define MCF_INTC_IPRH_INT58     (0x04000000)
#define MCF_INTC_IPRH_INT59     (0x08000000)
#define MCF_INTC_IPRH_INT60     (0x10000000)
#define MCF_INTC_IPRH_INT61     (0x20000000)
#define MCF_INTC_IPRH_INT62     (0x40000000)
#define MCF_INTC_IPRH_INT63     (0x80000000)

/* Bit definitions and macros for IPRL */
#define MCF_INTC_IPRL_INT0      (0x00000001)
#define MCF_INTC_IPRL_INT1      (0x00000002)
#define MCF_INTC_IPRL_INT2      (0x00000004)
#define MCF_INTC_IPRL_INT3      (0x00000008)
#define MCF_INTC_IPRL_INT4      (0x00000010)
#define MCF_INTC_IPRL_INT5      (0x00000020)
#define MCF_INTC_IPRL_INT6      (0x00000040)
#define MCF_INTC_IPRL_INT7      (0x00000080)
#define MCF_INTC_IPRL_INT8      (0x00000100)
#define MCF_INTC_IPRL_INT9      (0x00000200)
#define MCF_INTC_IPRL_INT10     (0x00000400)
#define MCF_INTC_IPRL_INT11     (0x00000800)
#define MCF_INTC_IPRL_INT12     (0x00001000)
#define MCF_INTC_IPRL_INT13     (0x00002000)
#define MCF_INTC_IPRL_INT14     (0x00004000)
#define MCF_INTC_IPRL_INT15     (0x00008000)
#define MCF_INTC_IPRL_INT16     (0x00010000)
#define MCF_INTC_IPRL_INT17     (0x00020000)
#define MCF_INTC_IPRL_INT18     (0x00040000)
#define MCF_INTC_IPRL_INT19     (0x00080000)
#define MCF_INTC_IPRL_INT20     (0x00100000)
#define MCF_INTC_IPRL_INT21     (0x00200000)
#define MCF_INTC_IPRL_INT22     (0x00400000)
#define MCF_INTC_IPRL_INT23     (0x00800000)
#define MCF_INTC_IPRL_INT24     (0x01000000)
#define MCF_INTC_IPRL_INT25     (0x02000000)
#define MCF_INTC_IPRL_INT26     (0x04000000)
#define MCF_INTC_IPRL_INT27     (0x08000000)
#define MCF_INTC_IPRL_INT28     (0x10000000)
#define MCF_INTC_IPRL_INT29     (0x20000000)
#define MCF_INTC_IPRL_INT30     (0x40000000)
#define MCF_INTC_IPRL_INT31     (0x80000000)

/* Bit definitions and macros for IMRH */
#define MCF_INTC_IMRH_INT_MASK32    (0x00000001)
#define MCF_INTC_IMRH_INT_MASK33    (0x00000002)
#define MCF_INTC_IMRH_INT_MASK34    (0x00000004)
#define MCF_INTC_IMRH_INT_MASK35    (0x00000008)
#define MCF_INTC_IMRH_INT_MASK36    (0x00000010)
#define MCF_INTC_IMRH_INT_MASK37    (0x00000020)
#define MCF_INTC_IMRH_INT_MASK38    (0x00000040)
#define MCF_INTC_IMRH_INT_MASK39    (0x00000080)
#define MCF_INTC_IMRH_INT_MASK40    (0x00000100)
#define MCF_INTC_IMRH_INT_MASK41    (0x00000200)
#define MCF_INTC_IMRH_INT_MASK42    (0x00000400)
#define MCF_INTC_IMRH_INT_MASK43    (0x00000800)
#define MCF_INTC_IMRH_INT_MASK44    (0x00001000)
#define MCF_INTC_IMRH_INT_MASK45    (0x00002000)
#define MCF_INTC_IMRH_INT_MASK46    (0x00004000)
#define MCF_INTC_IMRH_INT_MASK47    (0x00008000)
#define MCF_INTC_IMRH_INT_MASK48    (0x00010000)
#define MCF_INTC_IMRH_INT_MASK49    (0x00020000)
#define MCF_INTC_IMRH_INT_MASK50    (0x00040000)
#define MCF_INTC_IMRH_INT_MASK51    (0x00080000)
#define MCF_INTC_IMRH_INT_MASK52    (0x00100000)
#define MCF_INTC_IMRH_INT_MASK53    (0x00200000)
#define MCF_INTC_IMRH_INT_MASK54    (0x00400000)
#define MCF_INTC_IMRH_INT_MASK55    (0x00800000)
#define MCF_INTC_IMRH_INT_MASK56    (0x01000000)
#define MCF_INTC_IMRH_INT_MASK57    (0x02000000)
#define MCF_INTC_IMRH_INT_MASK58    (0x04000000)
#define MCF_INTC_IMRH_INT_MASK59    (0x08000000)
#define MCF_INTC_IMRH_INT_MASK60    (0x10000000)
#define MCF_INTC_IMRH_INT_MASK61    (0x20000000)
#define MCF_INTC_IMRH_INT_MASK62    (0x40000000)
#define MCF_INTC_IMRH_INT_MASK63    (0x80000000)

/* Bit definitions and macros for IMRL */
#define MCF_INTC_IMRL_INT_MASK0     (0x00000001)
#define MCF_INTC_IMRL_INT_MASK1     (0x00000002)
#define MCF_INTC_IMRL_INT_MASK2     (0x00000004)
#define MCF_INTC_IMRL_INT_MASK3     (0x00000008)
#define MCF_INTC_IMRL_INT_MASK4     (0x00000010)
#define MCF_INTC_IMRL_INT_MASK5     (0x00000020)
#define MCF_INTC_IMRL_INT_MASK6     (0x00000040)
#define MCF_INTC_IMRL_INT_MASK7     (0x00000080)
#define MCF_INTC_IMRL_INT_MASK8     (0x00000100)
#define MCF_INTC_IMRL_INT_MASK9     (0x00000200)
#define MCF_INTC_IMRL_INT_MASK10    (0x00000400)
#define MCF_INTC_IMRL_INT_MASK11    (0x00000800)
#define MCF_INTC_IMRL_INT_MASK12    (0x00001000)
#define MCF_INTC_IMRL_INT_MASK13    (0x00002000)
#define MCF_INTC_IMRL_INT_MASK14    (0x00004000)
#define MCF_INTC_IMRL_INT_MASK15    (0x00008000)
#define MCF_INTC_IMRL_INT_MASK16    (0x00010000)
#define MCF_INTC_IMRL_INT_MASK17    (0x00020000)
#define MCF_INTC_IMRL_INT_MASK18    (0x00040000)
#define MCF_INTC_IMRL_INT_MASK19    (0x00080000)
#define MCF_INTC_IMRL_INT_MASK20    (0x00100000)
#define MCF_INTC_IMRL_INT_MASK21    (0x00200000)
#define MCF_INTC_IMRL_INT_MASK22    (0x00400000)
#define MCF_INTC_IMRL_INT_MASK23    (0x00800000)
#define MCF_INTC_IMRL_INT_MASK24    (0x01000000)
#define MCF_INTC_IMRL_INT_MASK25    (0x02000000)
#define MCF_INTC_IMRL_INT_MASK26    (0x04000000)
#define MCF_INTC_IMRL_INT_MASK27    (0x08000000)
#define MCF_INTC_IMRL_INT_MASK28    (0x10000000)
#define MCF_INTC_IMRL_INT_MASK29    (0x20000000)
#define MCF_INTC_IMRL_INT_MASK30    (0x40000000)
#define MCF_INTC_IMRL_INT_MASK31    (0x80000000)

/* Bit definitions and macros for INTFRCH */
#define MCF_INTC_INTFRCH_INTFRC32   (0x00000001)
#define MCF_INTC_INTFRCH_INTFRC33   (0x00000002)
#define MCF_INTC_INTFRCH_INTFRC34   (0x00000004)
#define MCF_INTC_INTFRCH_INTFRC35   (0x00000008)
#define MCF_INTC_INTFRCH_INTFRC36   (0x00000010)
#define MCF_INTC_INTFRCH_INTFRC37   (0x00000020)
#define MCF_INTC_INTFRCH_INTFRC38   (0x00000040)
#define MCF_INTC_INTFRCH_INTFRC39   (0x00000080)
#define MCF_INTC_INTFRCH_INTFRC40   (0x00000100)
#define MCF_INTC_INTFRCH_INTFRC41   (0x00000200)
#define MCF_INTC_INTFRCH_INTFRC42   (0x00000400)
#define MCF_INTC_INTFRCH_INTFRC43   (0x00000800)
#define MCF_INTC_INTFRCH_INTFRC44   (0x00001000)
#define MCF_INTC_INTFRCH_INTFRC45   (0x00002000)
#define MCF_INTC_INTFRCH_INTFRC46   (0x00004000)
#define MCF_INTC_INTFRCH_INTFRC47   (0x00008000)
#define MCF_INTC_INTFRCH_INTFRC48   (0x00010000)
#define MCF_INTC_INTFRCH_INTFRC49   (0x00020000)
#define MCF_INTC_INTFRCH_INTFRC50   (0x00040000)
#define MCF_INTC_INTFRCH_INTFRC51   (0x00080000)
#define MCF_INTC_INTFRCH_INTFRC52   (0x00100000)
#define MCF_INTC_INTFRCH_INTFRC53   (0x00200000)
#define MCF_INTC_INTFRCH_INTFRC54   (0x00400000)
#define MCF_INTC_INTFRCH_INTFRC55   (0x00800000)
#define MCF_INTC_INTFRCH_INTFRC56   (0x01000000)
#define MCF_INTC_INTFRCH_INTFRC57   (0x02000000)
#define MCF_INTC_INTFRCH_INTFRC58   (0x04000000)
#define MCF_INTC_INTFRCH_INTFRC59   (0x08000000)
#define MCF_INTC_INTFRCH_INTFRC60   (0x10000000)
#define MCF_INTC_INTFRCH_INTFRC61   (0x20000000)
#define MCF_INTC_INTFRCH_INTFRC62   (0x40000000)
#define MCF_INTC_INTFRCH_INTFRC63   (0x80000000)

/* Bit definitions and macros for INTFRCL */
#define MCF_INTC_INTFRCL_INTFRC0    (0x00000001)
#define MCF_INTC_INTFRCL_INTFRC1    (0x00000002)
#define MCF_INTC_INTFRCL_INTFRC2    (0x00000004)
#define MCF_INTC_INTFRCL_INTFRC3    (0x00000008)
#define MCF_INTC_INTFRCL_INTFRC4    (0x00000010)
#define MCF_INTC_INTFRCL_INTFRC5    (0x00000020)
#define MCF_INTC_INTFRCL_INTFRC6    (0x00000040)
#define MCF_INTC_INTFRCL_INTFRC7    (0x00000080)
#define MCF_INTC_INTFRCL_INTFRC8    (0x00000100)
#define MCF_INTC_INTFRCL_INTFRC9    (0x00000200)
#define MCF_INTC_INTFRCL_INTFRC10   (0x00000400)
#define MCF_INTC_INTFRCL_INTFRC11   (0x00000800)
#define MCF_INTC_INTFRCL_INTFRC12   (0x00001000)
#define MCF_INTC_INTFRCL_INTFRC13   (0x00002000)
#define MCF_INTC_INTFRCL_INTFRC14   (0x00004000)
#define MCF_INTC_INTFRCL_INTFRC15   (0x00008000)
#define MCF_INTC_INTFRCL_INTFRC16   (0x00010000)
#define MCF_INTC_INTFRCL_INTFRC17   (0x00020000)
#define MCF_INTC_INTFRCL_INTFRC18   (0x00040000)
#define MCF_INTC_INTFRCL_INTFRC19   (0x00080000)
#define MCF_INTC_INTFRCL_INTFRC20   (0x00100000)
#define MCF_INTC_INTFRCL_INTFRC21   (0x00200000)
#define MCF_INTC_INTFRCL_INTFRC22   (0x00400000)
#define MCF_INTC_INTFRCL_INTFRC23   (0x00800000)
#define MCF_INTC_INTFRCL_INTFRC24   (0x01000000)
#define MCF_INTC_INTFRCL_INTFRC25   (0x02000000)
#define MCF_INTC_INTFRCL_INTFRC26   (0x04000000)
#define MCF_INTC_INTFRCL_INTFRC27   (0x08000000)
#define MCF_INTC_INTFRCL_INTFRC28   (0x10000000)
#define MCF_INTC_INTFRCL_INTFRC29   (0x20000000)
#define MCF_INTC_INTFRCL_INTFRC30   (0x40000000)
#define MCF_INTC_INTFRCL_INTFRC31   (0x80000000)

/* Bit definitions and macros for ICONFIG */
#define MCF_INTC_ICONFIG_EMASK      (0x0020)
#define MCF_INTC_ICONFIG_ELVLPRI1   (0x0200)
#define MCF_INTC_ICONFIG_ELVLPRI2   (0x0400)
#define MCF_INTC_ICONFIG_ELVLPRI3   (0x0800)
#define MCF_INTC_ICONFIG_ELVLPRI4   (0x1000)
#define MCF_INTC_ICONFIG_ELVLPRI5   (0x2000)
#define MCF_INTC_ICONFIG_ELVLPRI6   (0x4000)
#define MCF_INTC_ICONFIG_ELVLPRI7   (0x8000)

/* Bit definitions and macros for SIMR */
#define MCF_INTC_SIMR_SIMR(x)   (((x)&0x7F))

/* Bit definitions and macros for CIMR */
#define MCF_INTC_CIMR_CIMR(x)   (((x)&0x7F))

/* Bit definitions and macros for CLMASK */
#define MCF_INTC_CLMASK_CLMASK(x)   (((x)&0x0F))

/* Bit definitions and macros for SLMASK */
#define MCF_INTC_SLMASK_SLMASK(x)   (((x)&0x0F))

/* Bit definitions and macros for ICR group */
#define MCF_INTC_ICR_IL(x)      (((x)&0x07))

/* Bit definitions and macros for ICR1 */
#define MCF_INTC_ICR1_IL(x)     (((x)&0x07))

/* Bit definitions and macros for ICR2 */
#define MCF_INTC_ICR2_IL(x)     (((x)&0x07))

/* Bit definitions and macros for ICR3 */
#define MCF_INTC_ICR3_IL(x)     (((x)&0x07))

/* Bit definitions and macros for ICR4 */
#define MCF_INTC_ICR4_IL(x)     (((x)&0x07))

/* Bit definitions and macros for ICR5 */
#define MCF_INTC_ICR5_IL(x)     (((x)&0x07))

/* Bit definitions and macros for ICR6 */
#define MCF_INTC_ICR6_IL(x)     (((x)&0x07))

/* Bit definitions and macros for ICR7 */
#define MCF_INTC_ICR7_IL(x)     (((x)&0x07))

/* Bit definitions and macros for ICR8 */
#define MCF_INTC_ICR8_IL(x)     (((x)&0x07))

/* Bit definitions and macros for ICR9 */
#define MCF_INTC_ICR9_IL(x)     (((x)&0x07))

/* Bit definitions and macros for ICR10 */
#define MCF_INTC_ICR10_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR11 */
#define MCF_INTC_ICR11_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR12 */
#define MCF_INTC_ICR12_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR13 */
#define MCF_INTC_ICR13_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR14 */
#define MCF_INTC_ICR14_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR15 */
#define MCF_INTC_ICR15_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR16 */
#define MCF_INTC_ICR16_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR17 */
#define MCF_INTC_ICR17_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR18 */
#define MCF_INTC_ICR18_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR19 */
#define MCF_INTC_ICR19_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR20 */
#define MCF_INTC_ICR20_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR21 */
#define MCF_INTC_ICR21_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR22 */
#define MCF_INTC_ICR22_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR23 */
#define MCF_INTC_ICR23_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR24 */
#define MCF_INTC_ICR24_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR25 */
#define MCF_INTC_ICR25_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR26 */
#define MCF_INTC_ICR26_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR27 */
#define MCF_INTC_ICR27_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR28 */
#define MCF_INTC_ICR28_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR29 */
#define MCF_INTC_ICR29_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR30 */
#define MCF_INTC_ICR30_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR31 */
#define MCF_INTC_ICR31_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR32 */
#define MCF_INTC_ICR32_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR33 */
#define MCF_INTC_ICR33_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR34 */
#define MCF_INTC_ICR34_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR35 */
#define MCF_INTC_ICR35_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR36 */
#define MCF_INTC_ICR36_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR37 */
#define MCF_INTC_ICR37_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR38 */
#define MCF_INTC_ICR38_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR39 */
#define MCF_INTC_ICR39_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR40 */
#define MCF_INTC_ICR40_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR41 */
#define MCF_INTC_ICR41_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR42 */
#define MCF_INTC_ICR42_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR43 */
#define MCF_INTC_ICR43_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR44 */
#define MCF_INTC_ICR44_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR45 */
#define MCF_INTC_ICR45_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR46 */
#define MCF_INTC_ICR46_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR47 */
#define MCF_INTC_ICR47_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR48 */
#define MCF_INTC_ICR48_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR49 */
#define MCF_INTC_ICR49_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR50 */
#define MCF_INTC_ICR50_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR51 */
#define MCF_INTC_ICR51_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR52 */
#define MCF_INTC_ICR52_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR53 */
#define MCF_INTC_ICR53_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR54 */
#define MCF_INTC_ICR54_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR55 */
#define MCF_INTC_ICR55_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR56 */
#define MCF_INTC_ICR56_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR57 */
#define MCF_INTC_ICR57_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR58 */
#define MCF_INTC_ICR58_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR59 */
#define MCF_INTC_ICR59_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR60 */
#define MCF_INTC_ICR60_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR61 */
#define MCF_INTC_ICR61_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR62 */
#define MCF_INTC_ICR62_IL(x)    (((x)&0x07))

/* Bit definitions and macros for ICR63 */
#define MCF_INTC_ICR63_IL(x)    (((x)&0x07))

/* Bit definitions and macros for SWIACK */
#define MCF_INTC_SWIACK_VECTOR(x)   (x)

/* Bit definitions and macros for LIACK group */
#define MCF_INTC_LIACK_VECTOR(x)    (x)

/* Bit definitions and macros for L1IACK */
#define MCF_INTC_L1IACK_VECTOR(x)   (x)

/* Bit definitions and macros for L2IACK */
#define MCF_INTC_L2IACK_VECTOR(x)   (x)

/* Bit definitions and macros for L3IACK */
#define MCF_INTC_L3IACK_VECTOR(x)   (x)

/* Bit definitions and macros for L4IACK */
#define MCF_INTC_L4IACK_VECTOR(x)   (x)

/* Bit definitions and macros for L5IACK */
#define MCF_INTC_L5IACK_VECTOR(x)   (x)

/* Bit definitions and macros for L6IACK */
#define MCF_INTC_L6IACK_VECTOR(x)   (x)

/* Bit definitions and macros for L7IACK */
#define MCF_INTC_L7IACK_VECTOR(x)   (x)

/********************************************************************/

#endif /* __MCF5445X_INTC_H__ */

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
#ifndef __MCF5445X_EDMA_H__
#define __MCF5445X_EDMA_H__

/*
 * Enhanced DMA (EDMA)
 */

/* Channels */
#define MCF_EDMA_CHAN_DREQ0	0	/* External DMA request 0 */
#define MCF_EDMA_CHAN_DREQ1	1	/* External DMA request 1 */
#define MCF_EDMA_CHAN_UART0_RX	2	/* UART0 Receive */
#define MCF_EDMA_CHAN_UART0_TX	3	/* UART0 Transmit */
#define MCF_EDMA_CHAN_UART1_RX	4	/* UART1 Receive */
#define MCF_EDMA_CHAN_UART1_TX	5	/* UART1 Transmit */
#define MCF_EDMA_CHAN_UART2_RX	6	/* UART2 Receive */
#define MCF_EDMA_CHAN_UART2_TX	7	/* UART2 Transmit */
#define MCF_EDMA_CHAN_TIMER0	8	/* Timer 0 / SSI0 Rx */
#define MCF_EDMA_CHAN_TIMER1	9	/* Timer 1 / SSI1 Rx */
#define MCF_EDMA_CHAN_TIMER2	10	/* Timer 2 / SSI0 Tx */
#define MCF_EDMA_CHAN_TIMER3	11	/* Timer 3 / SSI1 Tx */
#define MCF_EDMA_CHAN_DSPI_RX	12	/* DSPI Receive */
#define MCF_EDMA_CHAN_DSPI_TX	13	/* DSPI Transmit */
#define MCF_EDMA_CHAN_ATA_RX	14	/* ATA Receive */
#define MCF_EDMA_CHAN_ATA_TX	15	/* ATA Transmit */

/* Register read/write macros */
#define MCF_EDMA_CR                     MCF_REG32(0xFC044000)
#define MCF_EDMA_ES                     MCF_REG32(0xFC044004)
#define MCF_EDMA_ERQ                    MCF_REG16(0xFC04400E)
#define MCF_EDMA_EEI                    MCF_REG16(0xFC044016)
#define MCF_EDMA_SERQ                   MCF_REG08(0xFC044018)
#define MCF_EDMA_CERQ                   MCF_REG08(0xFC044019)
#define MCF_EDMA_SEEI                   MCF_REG08(0xFC04401A)
#define MCF_EDMA_CEEI                   MCF_REG08(0xFC04401B)
#define MCF_EDMA_CINT                   MCF_REG08(0xFC04401C)
#define MCF_EDMA_CERR                   MCF_REG08(0xFC04401D)
#define MCF_EDMA_SSRT                   MCF_REG08(0xFC04401E)
#define MCF_EDMA_CDNE                   MCF_REG08(0xFC04401F)
#define MCF_EDMA_INTR                   MCF_REG16(0xFC044026)
#define MCF_EDMA_ERR                    MCF_REG16(0xFC04402E)
#define MCF_EDMA_DCHPRI0                MCF_REG08(0xFC044100)
#define MCF_EDMA_DCHPRI1                MCF_REG08(0xFC044101)
#define MCF_EDMA_DCHPRI2                MCF_REG08(0xFC044102)
#define MCF_EDMA_DCHPRI3                MCF_REG08(0xFC044103)
#define MCF_EDMA_DCHPRI4                MCF_REG08(0xFC044104)
#define MCF_EDMA_DCHPRI5                MCF_REG08(0xFC044105)
#define MCF_EDMA_DCHPRI6                MCF_REG08(0xFC044106)
#define MCF_EDMA_DCHPRI7                MCF_REG08(0xFC044107)
#define MCF_EDMA_DCHPRI8                MCF_REG08(0xFC044108)
#define MCF_EDMA_DCHPRI9                MCF_REG08(0xFC044109)
#define MCF_EDMA_DCHPRI10               MCF_REG08(0xFC04410A)
#define MCF_EDMA_DCHPRI11               MCF_REG08(0xFC04410B)
#define MCF_EDMA_DCHPRI12               MCF_REG08(0xFC04410C)
#define MCF_EDMA_DCHPRI13               MCF_REG08(0xFC04410D)
#define MCF_EDMA_DCHPRI14               MCF_REG08(0xFC04410E)
#define MCF_EDMA_DCHPRI15               MCF_REG08(0xFC04410F)
#define MCF_EDMA_TCD0_SADDR             MCF_REG32(0xFC045000)
#define MCF_EDMA_TCD0_ATTR              MCF_REG16(0xFC045004)
#define MCF_EDMA_TCD0_SOFF              MCF_REG16(0xFC045006)
#define MCF_EDMA_TCD0_NBYTES            MCF_REG32(0xFC045008)
#define MCF_EDMA_TCD0_SLAST             MCF_REG32(0xFC04500C)
#define MCF_EDMA_TCD0_DADDR             MCF_REG32(0xFC045010)
#define MCF_EDMA_TCD0_CITER_ELINK       MCF_REG16(0xFC045014)
#define MCF_EDMA_TCD0_CITER             MCF_REG16(0xFC045014)
#define MCF_EDMA_TCD0_DOFF              MCF_REG16(0xFC045016)
#define MCF_EDMA_TCD0_DLAST_SGA         MCF_REG32(0xFC045018)
#define MCF_EDMA_TCD0_BITER_ELINK       MCF_REG16(0xFC04501C)
#define MCF_EDMA_TCD0_BITER             MCF_REG16(0xFC04501C)
#define MCF_EDMA_TCD0_CSR               MCF_REG16(0xFC04501E)
#define MCF_EDMA_TCD1_SADDR             MCF_REG32(0xFC045020)
#define MCF_EDMA_TCD1_ATTR              MCF_REG16(0xFC045024)
#define MCF_EDMA_TCD1_SOFF              MCF_REG16(0xFC045026)
#define MCF_EDMA_TCD1_NBYTES            MCF_REG32(0xFC045028)
#define MCF_EDMA_TCD1_SLAST             MCF_REG32(0xFC04502C)
#define MCF_EDMA_TCD1_DADDR             MCF_REG32(0xFC045030)
#define MCF_EDMA_TCD1_CITER_ELINK       MCF_REG16(0xFC045034)
#define MCF_EDMA_TCD1_CITER             MCF_REG16(0xFC045034)
#define MCF_EDMA_TCD1_DOFF              MCF_REG16(0xFC045036)
#define MCF_EDMA_TCD1_DLAST_SGA         MCF_REG32(0xFC045038)
#define MCF_EDMA_TCD1_BITER             MCF_REG16(0xFC04503C)
#define MCF_EDMA_TCD1_BITER_ELINK       MCF_REG16(0xFC04503C)
#define MCF_EDMA_TCD1_CSR               MCF_REG16(0xFC04503E)
#define MCF_EDMA_TCD2_SADDR             MCF_REG32(0xFC045040)
#define MCF_EDMA_TCD2_ATTR              MCF_REG16(0xFC045044)
#define MCF_EDMA_TCD2_SOFF              MCF_REG16(0xFC045046)
#define MCF_EDMA_TCD2_NBYTES            MCF_REG32(0xFC045048)
#define MCF_EDMA_TCD2_SLAST             MCF_REG32(0xFC04504C)
#define MCF_EDMA_TCD2_DADDR             MCF_REG32(0xFC045050)
#define MCF_EDMA_TCD2_CITER             MCF_REG16(0xFC045054)
#define MCF_EDMA_TCD2_CITER_ELINK       MCF_REG16(0xFC045054)
#define MCF_EDMA_TCD2_DOFF              MCF_REG16(0xFC045056)
#define MCF_EDMA_TCD2_DLAST_SGA         MCF_REG32(0xFC045058)
#define MCF_EDMA_TCD2_BITER_ELINK       MCF_REG16(0xFC04505C)
#define MCF_EDMA_TCD2_BITER             MCF_REG16(0xFC04505C)
#define MCF_EDMA_TCD2_CSR               MCF_REG16(0xFC04505E)
#define MCF_EDMA_TCD3_SADDR             MCF_REG32(0xFC045060)
#define MCF_EDMA_TCD3_ATTR              MCF_REG16(0xFC045064)
#define MCF_EDMA_TCD3_SOFF              MCF_REG16(0xFC045066)
#define MCF_EDMA_TCD3_NBYTES            MCF_REG32(0xFC045068)
#define MCF_EDMA_TCD3_SLAST             MCF_REG32(0xFC04506C)
#define MCF_EDMA_TCD3_DADDR             MCF_REG32(0xFC045070)
#define MCF_EDMA_TCD3_CITER             MCF_REG16(0xFC045074)
#define MCF_EDMA_TCD3_CITER_ELINK       MCF_REG16(0xFC045074)
#define MCF_EDMA_TCD3_DOFF              MCF_REG16(0xFC045076)
#define MCF_EDMA_TCD3_DLAST_SGA         MCF_REG32(0xFC045078)
#define MCF_EDMA_TCD3_BITER_ELINK       MCF_REG16(0xFC04507C)
#define MCF_EDMA_TCD3_BITER             MCF_REG16(0xFC04507C)
#define MCF_EDMA_TCD3_CSR               MCF_REG16(0xFC04507E)
#define MCF_EDMA_TCD4_SADDR             MCF_REG32(0xFC045080)
#define MCF_EDMA_TCD4_ATTR              MCF_REG16(0xFC045084)
#define MCF_EDMA_TCD4_SOFF              MCF_REG16(0xFC045086)
#define MCF_EDMA_TCD4_NBYTES            MCF_REG32(0xFC045088)
#define MCF_EDMA_TCD4_SLAST             MCF_REG32(0xFC04508C)
#define MCF_EDMA_TCD4_DADDR             MCF_REG32(0xFC045090)
#define MCF_EDMA_TCD4_CITER             MCF_REG16(0xFC045094)
#define MCF_EDMA_TCD4_CITER_ELINK       MCF_REG16(0xFC045094)
#define MCF_EDMA_TCD4_DOFF              MCF_REG16(0xFC045096)
#define MCF_EDMA_TCD4_DLAST_SGA         MCF_REG32(0xFC045098)
#define MCF_EDMA_TCD4_BITER             MCF_REG16(0xFC04509C)
#define MCF_EDMA_TCD4_BITER_ELINK       MCF_REG16(0xFC04509C)
#define MCF_EDMA_TCD4_CSR               MCF_REG16(0xFC04509E)
#define MCF_EDMA_TCD5_SADDR             MCF_REG32(0xFC0450A0)
#define MCF_EDMA_TCD5_ATTR              MCF_REG16(0xFC0450A4)
#define MCF_EDMA_TCD5_SOFF              MCF_REG16(0xFC0450A6)
#define MCF_EDMA_TCD5_NBYTES            MCF_REG32(0xFC0450A8)
#define MCF_EDMA_TCD5_SLAST             MCF_REG32(0xFC0450AC)
#define MCF_EDMA_TCD5_DADDR             MCF_REG32(0xFC0450B0)
#define MCF_EDMA_TCD5_CITER             MCF_REG16(0xFC0450B4)
#define MCF_EDMA_TCD5_CITER_ELINK       MCF_REG16(0xFC0450B4)
#define MCF_EDMA_TCD5_DOFF              MCF_REG16(0xFC0450B6)
#define MCF_EDMA_TCD5_DLAST_SGA         MCF_REG32(0xFC0450B8)
#define MCF_EDMA_TCD5_BITER_ELINK       MCF_REG16(0xFC0450BC)
#define MCF_EDMA_TCD5_BITER             MCF_REG16(0xFC0450BC)
#define MCF_EDMA_TCD5_CSR               MCF_REG16(0xFC0450BE)
#define MCF_EDMA_TCD6_SADDR             MCF_REG32(0xFC0450C0)
#define MCF_EDMA_TCD6_ATTR              MCF_REG16(0xFC0450C4)
#define MCF_EDMA_TCD6_SOFF              MCF_REG16(0xFC0450C6)
#define MCF_EDMA_TCD6_NBYTES            MCF_REG32(0xFC0450C8)
#define MCF_EDMA_TCD6_SLAST             MCF_REG32(0xFC0450CC)
#define MCF_EDMA_TCD6_DADDR             MCF_REG32(0xFC0450D0)
#define MCF_EDMA_TCD6_CITER             MCF_REG16(0xFC0450D4)
#define MCF_EDMA_TCD6_CITER_ELINK       MCF_REG16(0xFC0450D4)
#define MCF_EDMA_TCD6_DOFF              MCF_REG16(0xFC0450D6)
#define MCF_EDMA_TCD6_DLAST_SGA         MCF_REG32(0xFC0450D8)
#define MCF_EDMA_TCD6_BITER_ELINK       MCF_REG16(0xFC0450DC)
#define MCF_EDMA_TCD6_BITER             MCF_REG16(0xFC0450DC)
#define MCF_EDMA_TCD6_CSR               MCF_REG16(0xFC0450DE)
#define MCF_EDMA_TCD7_SADDR             MCF_REG32(0xFC0450E0)
#define MCF_EDMA_TCD7_ATTR              MCF_REG16(0xFC0450E4)
#define MCF_EDMA_TCD7_SOFF              MCF_REG16(0xFC0450E6)
#define MCF_EDMA_TCD7_NBYTES            MCF_REG32(0xFC0450E8)
#define MCF_EDMA_TCD7_SLAST             MCF_REG32(0xFC0450EC)
#define MCF_EDMA_TCD7_DADDR             MCF_REG32(0xFC0450F0)
#define MCF_EDMA_TCD7_CITER             MCF_REG16(0xFC0450F4)
#define MCF_EDMA_TCD7_CITER_ELINK       MCF_REG16(0xFC0450F4)
#define MCF_EDMA_TCD7_DOFF              MCF_REG16(0xFC0450F6)
#define MCF_EDMA_TCD7_DLAST_SGA         MCF_REG32(0xFC0450F8)
#define MCF_EDMA_TCD7_BITER_ELINK       MCF_REG16(0xFC0450FC)
#define MCF_EDMA_TCD7_BITER             MCF_REG16(0xFC0450FC)
#define MCF_EDMA_TCD7_CSR               MCF_REG16(0xFC0450FE)
#define MCF_EDMA_TCD8_SADDR             MCF_REG32(0xFC045100)
#define MCF_EDMA_TCD8_ATTR              MCF_REG16(0xFC045104)
#define MCF_EDMA_TCD8_SOFF              MCF_REG16(0xFC045106)
#define MCF_EDMA_TCD8_NBYTES            MCF_REG32(0xFC045108)
#define MCF_EDMA_TCD8_SLAST             MCF_REG32(0xFC04510C)
#define MCF_EDMA_TCD8_DADDR             MCF_REG32(0xFC045110)
#define MCF_EDMA_TCD8_CITER             MCF_REG16(0xFC045114)
#define MCF_EDMA_TCD8_CITER_ELINK       MCF_REG16(0xFC045114)
#define MCF_EDMA_TCD8_DOFF              MCF_REG16(0xFC045116)
#define MCF_EDMA_TCD8_DLAST_SGA         MCF_REG32(0xFC045118)
#define MCF_EDMA_TCD8_BITER_ELINK       MCF_REG16(0xFC04511C)
#define MCF_EDMA_TCD8_BITER             MCF_REG16(0xFC04511C)
#define MCF_EDMA_TCD8_CSR               MCF_REG16(0xFC04511E)
#define MCF_EDMA_TCD9_SADDR             MCF_REG32(0xFC045120)
#define MCF_EDMA_TCD9_ATTR              MCF_REG16(0xFC045124)
#define MCF_EDMA_TCD9_SOFF              MCF_REG16(0xFC045126)
#define MCF_EDMA_TCD9_NBYTES            MCF_REG32(0xFC045128)
#define MCF_EDMA_TCD9_SLAST             MCF_REG32(0xFC04512C)
#define MCF_EDMA_TCD9_DADDR             MCF_REG32(0xFC045130)
#define MCF_EDMA_TCD9_CITER_ELINK       MCF_REG16(0xFC045134)
#define MCF_EDMA_TCD9_CITER             MCF_REG16(0xFC045134)
#define MCF_EDMA_TCD9_DOFF              MCF_REG16(0xFC045136)
#define MCF_EDMA_TCD9_DLAST_SGA         MCF_REG32(0xFC045138)
#define MCF_EDMA_TCD9_BITER_ELINK       MCF_REG16(0xFC04513C)
#define MCF_EDMA_TCD9_BITER             MCF_REG16(0xFC04513C)
#define MCF_EDMA_TCD9_CSR               MCF_REG16(0xFC04513E)
#define MCF_EDMA_TCD10_SADDR            MCF_REG32(0xFC045140)
#define MCF_EDMA_TCD10_ATTR             MCF_REG16(0xFC045144)
#define MCF_EDMA_TCD10_SOFF             MCF_REG16(0xFC045146)
#define MCF_EDMA_TCD10_NBYTES           MCF_REG32(0xFC045148)
#define MCF_EDMA_TCD10_SLAST            MCF_REG32(0xFC04514C)
#define MCF_EDMA_TCD10_DADDR            MCF_REG32(0xFC045150)
#define MCF_EDMA_TCD10_CITER_ELINK      MCF_REG16(0xFC045154)
#define MCF_EDMA_TCD10_CITER            MCF_REG16(0xFC045154)
#define MCF_EDMA_TCD10_DOFF             MCF_REG16(0xFC045156)
#define MCF_EDMA_TCD10_DLAST_SGA        MCF_REG32(0xFC045158)
#define MCF_EDMA_TCD10_BITER            MCF_REG16(0xFC04515C)
#define MCF_EDMA_TCD10_BITER_ELINK      MCF_REG16(0xFC04515C)
#define MCF_EDMA_TCD10_CSR              MCF_REG16(0xFC04515E)
#define MCF_EDMA_TCD11_SADDR            MCF_REG32(0xFC045160)
#define MCF_EDMA_TCD11_ATTR             MCF_REG16(0xFC045164)
#define MCF_EDMA_TCD11_SOFF             MCF_REG16(0xFC045166)
#define MCF_EDMA_TCD11_NBYTES           MCF_REG32(0xFC045168)
#define MCF_EDMA_TCD11_SLAST            MCF_REG32(0xFC04516C)
#define MCF_EDMA_TCD11_DADDR            MCF_REG32(0xFC045170)
#define MCF_EDMA_TCD11_CITER            MCF_REG16(0xFC045174)
#define MCF_EDMA_TCD11_CITER_ELINK      MCF_REG16(0xFC045174)
#define MCF_EDMA_TCD11_DOFF             MCF_REG16(0xFC045176)
#define MCF_EDMA_TCD11_DLAST_SGA        MCF_REG32(0xFC045178)
#define MCF_EDMA_TCD11_BITER            MCF_REG16(0xFC04517C)
#define MCF_EDMA_TCD11_BITER_ELINK      MCF_REG16(0xFC04517C)
#define MCF_EDMA_TCD11_CSR              MCF_REG16(0xFC04517E)
#define MCF_EDMA_TCD12_SADDR            MCF_REG32(0xFC045180)
#define MCF_EDMA_TCD12_ATTR             MCF_REG16(0xFC045184)
#define MCF_EDMA_TCD12_SOFF             MCF_REG16(0xFC045186)
#define MCF_EDMA_TCD12_NBYTES           MCF_REG32(0xFC045188)
#define MCF_EDMA_TCD12_SLAST            MCF_REG32(0xFC04518C)
#define MCF_EDMA_TCD12_DADDR            MCF_REG32(0xFC045190)
#define MCF_EDMA_TCD12_CITER            MCF_REG16(0xFC045194)
#define MCF_EDMA_TCD12_CITER_ELINK      MCF_REG16(0xFC045194)
#define MCF_EDMA_TCD12_DOFF             MCF_REG16(0xFC045196)
#define MCF_EDMA_TCD12_DLAST_SGA        MCF_REG32(0xFC045198)
#define MCF_EDMA_TCD12_BITER            MCF_REG16(0xFC04519C)
#define MCF_EDMA_TCD12_BITER_ELINK      MCF_REG16(0xFC04519C)
#define MCF_EDMA_TCD12_CSR              MCF_REG16(0xFC04519E)
#define MCF_EDMA_TCD13_SADDR            MCF_REG32(0xFC0451A0)
#define MCF_EDMA_TCD13_ATTR             MCF_REG16(0xFC0451A4)
#define MCF_EDMA_TCD13_SOFF             MCF_REG16(0xFC0451A6)
#define MCF_EDMA_TCD13_NBYTES           MCF_REG32(0xFC0451A8)
#define MCF_EDMA_TCD13_SLAST            MCF_REG32(0xFC0451AC)
#define MCF_EDMA_TCD13_DADDR            MCF_REG32(0xFC0451B0)
#define MCF_EDMA_TCD13_CITER_ELINK      MCF_REG16(0xFC0451B4)
#define MCF_EDMA_TCD13_CITER            MCF_REG16(0xFC0451B4)
#define MCF_EDMA_TCD13_DOFF             MCF_REG16(0xFC0451B6)
#define MCF_EDMA_TCD13_DLAST_SGA        MCF_REG32(0xFC0451B8)
#define MCF_EDMA_TCD13_BITER_ELINK      MCF_REG16(0xFC0451BC)
#define MCF_EDMA_TCD13_BITER            MCF_REG16(0xFC0451BC)
#define MCF_EDMA_TCD13_CSR              MCF_REG16(0xFC0451BE)
#define MCF_EDMA_TCD14_SADDR            MCF_REG32(0xFC0451C0)
#define MCF_EDMA_TCD14_ATTR             MCF_REG16(0xFC0451C4)
#define MCF_EDMA_TCD14_SOFF             MCF_REG16(0xFC0451C6)
#define MCF_EDMA_TCD14_NBYTES           MCF_REG32(0xFC0451C8)
#define MCF_EDMA_TCD14_SLAST            MCF_REG32(0xFC0451CC)
#define MCF_EDMA_TCD14_DADDR            MCF_REG32(0xFC0451D0)
#define MCF_EDMA_TCD14_CITER            MCF_REG16(0xFC0451D4)
#define MCF_EDMA_TCD14_CITER_ELINK      MCF_REG16(0xFC0451D4)
#define MCF_EDMA_TCD14_DOFF             MCF_REG16(0xFC0451D6)
#define MCF_EDMA_TCD14_DLAST_SGA        MCF_REG32(0xFC0451D8)
#define MCF_EDMA_TCD14_BITER_ELINK      MCF_REG16(0xFC0451DC)
#define MCF_EDMA_TCD14_BITER            MCF_REG16(0xFC0451DC)
#define MCF_EDMA_TCD14_CSR              MCF_REG16(0xFC0451DE)
#define MCF_EDMA_TCD15_SADDR            MCF_REG32(0xFC0451E0)
#define MCF_EDMA_TCD15_ATTR             MCF_REG16(0xFC0451E4)
#define MCF_EDMA_TCD15_SOFF             MCF_REG16(0xFC0451E6)
#define MCF_EDMA_TCD15_NBYTES           MCF_REG32(0xFC0451E8)
#define MCF_EDMA_TCD15_SLAST            MCF_REG32(0xFC0451EC)
#define MCF_EDMA_TCD15_DADDR            MCF_REG32(0xFC0451F0)
#define MCF_EDMA_TCD15_CITER_ELINK      MCF_REG16(0xFC0451F4)
#define MCF_EDMA_TCD15_CITER            MCF_REG16(0xFC0451F4)
#define MCF_EDMA_TCD15_DOFF             MCF_REG16(0xFC0451F6)
#define MCF_EDMA_TCD15_DLAST_SGA        MCF_REG32(0xFC0451F8)
#define MCF_EDMA_TCD15_BITER            MCF_REG16(0xFC0451FC)
#define MCF_EDMA_TCD15_BITER_ELINK      MCF_REG16(0xFC0451FC)
#define MCF_EDMA_TCD15_CSR              MCF_REG16(0xFC0451FE)

/* Parameterized register read/write macros for multiple registers */
#define MCF_EDMA_DCHPRI(x)              MCF_REG08(0xFC044100+((x)*0x001))
#define MCF_EDMA_TCD_SADDR(x)           MCF_REG32(0xFC045000+((x)*0x020))
#define MCF_EDMA_TCD_ATTR(x)            MCF_REG16(0xFC045004+((x)*0x020))
#define MCF_EDMA_TCD_SOFF(x)            MCF_REG16(0xFC045006+((x)*0x020))
#define MCF_EDMA_TCD_NBYTES(x)          MCF_REG32(0xFC045008+((x)*0x020))
#define MCF_EDMA_TCD_SLAST(x)           MCF_REG32(0xFC04500C+((x)*0x020))
#define MCF_EDMA_TCD_DADDR(x)           MCF_REG32(0xFC045010+((x)*0x020))
#define MCF_EDMA_TCD_CITER_ELINK(x)     MCF_REG16(0xFC045014+((x)*0x020))
#define MCF_EDMA_TCD_CITER(x)           MCF_REG16(0xFC045014+((x)*0x020))
#define MCF_EDMA_TCD_DOFF(x)            MCF_REG16(0xFC045016+((x)*0x020))
#define MCF_EDMA_TCD_DLAST_SGA(x)       MCF_REG32(0xFC045018+((x)*0x020))
#define MCF_EDMA_TCD_BITER_ELINK(x)     MCF_REG16(0xFC04501C+((x)*0x020))
#define MCF_EDMA_TCD_BITER(x)           MCF_REG16(0xFC04501C+((x)*0x020))
#define MCF_EDMA_TCD_CSR(x)             MCF_REG16(0xFC04501e +((x)*0x020))

/* Bit definitions and macros for CR */
#define MCF_EDMA_CR_EDBG        (0x00000002)
#define MCF_EDMA_CR_ERCA        (0x00000004)

/* Bit definitions and macros for ES */
#define MCF_EDMA_ES_DBE         (0x00000001)
#define MCF_EDMA_ES_SBE         (0x00000002)
#define MCF_EDMA_ES_SGE         (0x00000004)
#define MCF_EDMA_ES_NCE         (0x00000008)
#define MCF_EDMA_ES_DOE         (0x00000010)
#define MCF_EDMA_ES_DAE         (0x00000020)
#define MCF_EDMA_ES_SOE         (0x00000040)
#define MCF_EDMA_ES_SAE         (0x00000080)
#define MCF_EDMA_ES_ERRCHN(x)   (((x)&0x0000000F)<<8)
#define MCF_EDMA_ES_CPE         (0x00004000)
#define MCF_EDMA_ES_VLD         (0x80000000)

/* Bit definitions and macros for ERQ */
#define MCF_EDMA_ERQ_ERQ0       (0x0001)
#define MCF_EDMA_ERQ_ERQ1       (0x0002)
#define MCF_EDMA_ERQ_ERQ2       (0x0004)
#define MCF_EDMA_ERQ_ERQ3       (0x0008)
#define MCF_EDMA_ERQ_ERQ4       (0x0010)
#define MCF_EDMA_ERQ_ERQ5       (0x0020)
#define MCF_EDMA_ERQ_ERQ6       (0x0040)
#define MCF_EDMA_ERQ_ERQ7       (0x0080)
#define MCF_EDMA_ERQ_ERQ8       (0x0100)
#define MCF_EDMA_ERQ_ERQ9       (0x0200)
#define MCF_EDMA_ERQ_ERQ10      (0x0400)
#define MCF_EDMA_ERQ_ERQ11      (0x0800)
#define MCF_EDMA_ERQ_ERQ12      (0x1000)
#define MCF_EDMA_ERQ_ERQ13      (0x2000)
#define MCF_EDMA_ERQ_ERQ14      (0x4000)
#define MCF_EDMA_ERQ_ERQ15      (0x8000)

/* Bit definitions and macros for EEI */
#define MCF_EDMA_EEI_EEI0       (0x0001)
#define MCF_EDMA_EEI_EEI1       (0x0002)
#define MCF_EDMA_EEI_EEI2       (0x0004)
#define MCF_EDMA_EEI_EEI3       (0x0008)
#define MCF_EDMA_EEI_EEI4       (0x0010)
#define MCF_EDMA_EEI_EEI5       (0x0020)
#define MCF_EDMA_EEI_EEI6       (0x0040)
#define MCF_EDMA_EEI_EEI7       (0x0080)
#define MCF_EDMA_EEI_EEI8       (0x0100)
#define MCF_EDMA_EEI_EEI9       (0x0200)
#define MCF_EDMA_EEI_EEI10      (0x0400)
#define MCF_EDMA_EEI_EEI11      (0x0800)
#define MCF_EDMA_EEI_EEI12      (0x1000)
#define MCF_EDMA_EEI_EEI13      (0x2000)
#define MCF_EDMA_EEI_EEI14      (0x4000)
#define MCF_EDMA_EEI_EEI15      (0x8000)

/* Bit definitions and macros for SERQ */
#define MCF_EDMA_SERQ_SERQ(x)   (((x)&0x0F))
#define MCF_EDMA_SERQ_SAER      (0x40)

/* Bit definitions and macros for CERQ */
#define MCF_EDMA_CERQ_CERQ(x)   (((x)&0x0F))
#define MCF_EDMA_CERQ_CAER      (0x40)

/* Bit definitions and macros for SEEI */
#define MCF_EDMA_SEEI_SEEI(x)   (((x)&0x0F))
#define MCF_EDMA_SEEI_SAEE      (0x40)

/* Bit definitions and macros for CEEI */
#define MCF_EDMA_CEEI_CEEI(x)   (((x)&0x0F))
#define MCF_EDMA_CEEI_CAEE      (0x40)

/* Bit definitions and macros for CINT */
#define MCF_EDMA_CINT_CINT(x)   (((x)&0x0F))
#define MCF_EDMA_CINT_CAIR      (0x40)

/* Bit definitions and macros for CERR */
#define MCF_EDMA_CERR_CERR(x)   (((x)&0x0F))
#define MCF_EDMA_CERR_CAER      (0x40)

/* Bit definitions and macros for SSRT */
#define MCF_EDMA_SSRT_SSRT(x)   (((x)&0x0F))
#define MCF_EDMA_SSRT_SAST      (0x40)

/* Bit definitions and macros for CDNE */
#define MCF_EDMA_CDNE_CDNE(x)   (((x)&0x0F))
#define MCF_EDMA_CDNE_CADN      (0x40)

/* Bit definitions and macros for INTR */
#define MCF_EDMA_INTR_INT0      (0x0001)
#define MCF_EDMA_INTR_INT1      (0x0002)
#define MCF_EDMA_INTR_INT2      (0x0004)
#define MCF_EDMA_INTR_INT3      (0x0008)
#define MCF_EDMA_INTR_INT4      (0x0010)
#define MCF_EDMA_INTR_INT5      (0x0020)
#define MCF_EDMA_INTR_INT6      (0x0040)
#define MCF_EDMA_INTR_INT7      (0x0080)
#define MCF_EDMA_INTR_INT8      (0x0100)
#define MCF_EDMA_INTR_INT9      (0x0200)
#define MCF_EDMA_INTR_INT10     (0x0400)
#define MCF_EDMA_INTR_INT11     (0x0800)
#define MCF_EDMA_INTR_INT12     (0x1000)
#define MCF_EDMA_INTR_INT13     (0x2000)
#define MCF_EDMA_INTR_INT14     (0x4000)
#define MCF_EDMA_INTR_INT15     (0x8000)

/* Bit definitions and macros for ERR */
#define MCF_EDMA_ERR_ERR0       (0x0001)
#define MCF_EDMA_ERR_ERR1       (0x0002)
#define MCF_EDMA_ERR_ERR2       (0x0004)
#define MCF_EDMA_ERR_ERR3       (0x0008)
#define MCF_EDMA_ERR_ERR4       (0x0010)
#define MCF_EDMA_ERR_ERR5       (0x0020)
#define MCF_EDMA_ERR_ERR6       (0x0040)
#define MCF_EDMA_ERR_ERR7       (0x0080)
#define MCF_EDMA_ERR_ERR8       (0x0100)
#define MCF_EDMA_ERR_ERR9       (0x0200)
#define MCF_EDMA_ERR_ERR10      (0x0400)
#define MCF_EDMA_ERR_ERR11      (0x0800)
#define MCF_EDMA_ERR_ERR12      (0x1000)
#define MCF_EDMA_ERR_ERR13      (0x2000)
#define MCF_EDMA_ERR_ERR14      (0x4000)
#define MCF_EDMA_ERR_ERR15      (0x8000)

/* Bit definitions and macros for DCHPRI group */
#define MCF_EDMA_DCHPRI_CHPRI(x)    (((x)&0x0F))
#define MCF_EDMA_DCHPRI_ECP         (0x80)

/* Bit definitions and macros for DCHPRI0 */
#define MCF_EDMA_DCHPRI0_CHPRI(x)   (((x)&0x0F))
#define MCF_EDMA_DCHPRI0_ECP        (0x80)

/* Bit definitions and macros for DCHPRI1 */
#define MCF_EDMA_DCHPRI1_CHPRI(x)   (((x)&0x0F))
#define MCF_EDMA_DCHPRI1_ECP        (0x80)

/* Bit definitions and macros for DCHPRI2 */
#define MCF_EDMA_DCHPRI2_CHPRI(x)   (((x)&0x0F))
#define MCF_EDMA_DCHPRI2_ECP        (0x80)

/* Bit definitions and macros for DCHPRI3 */
#define MCF_EDMA_DCHPRI3_CHPRI(x)   (((x)&0x0F))
#define MCF_EDMA_DCHPRI3_ECP        (0x80)

/* Bit definitions and macros for DCHPRI4 */
#define MCF_EDMA_DCHPRI4_CHPRI(x)   (((x)&0x0F))
#define MCF_EDMA_DCHPRI4_ECP        (0x80)

/* Bit definitions and macros for DCHPRI5 */
#define MCF_EDMA_DCHPRI5_CHPRI(x)   (((x)&0x0F))
#define MCF_EDMA_DCHPRI5_ECP        (0x80)

/* Bit definitions and macros for DCHPRI6 */
#define MCF_EDMA_DCHPRI6_CHPRI(x)   (((x)&0x0F))
#define MCF_EDMA_DCHPRI6_ECP        (0x80)

/* Bit definitions and macros for DCHPRI7 */
#define MCF_EDMA_DCHPRI7_CHPRI(x)   (((x)&0x0F))
#define MCF_EDMA_DCHPRI7_ECP        (0x80)

/* Bit definitions and macros for DCHPRI8 */
#define MCF_EDMA_DCHPRI8_CHPRI(x)   (((x)&0x0F))
#define MCF_EDMA_DCHPRI8_ECP        (0x80)

/* Bit definitions and macros for DCHPRI9 */
#define MCF_EDMA_DCHPRI9_CHPRI(x)   (((x)&0x0F))
#define MCF_EDMA_DCHPRI9_ECP        (0x80)

/* Bit definitions and macros for DCHPRI10 */
#define MCF_EDMA_DCHPRI10_CHPRI(x)      (((x)&0x0F))
#define MCF_EDMA_DCHPRI10_ECP           (0x80)

/* Bit definitions and macros for DCHPRI11 */
#define MCF_EDMA_DCHPRI11_CHPRI(x)      (((x)&0x0F))
#define MCF_EDMA_DCHPRI11_ECP           (0x80)

/* Bit definitions and macros for DCHPRI12 */
#define MCF_EDMA_DCHPRI12_CHPRI(x)      (((x)&0x0F))
#define MCF_EDMA_DCHPRI12_ECP           (0x80)

/* Bit definitions and macros for DCHPRI13 */
#define MCF_EDMA_DCHPRI13_CHPRI(x)      (((x)&0x0F))
#define MCF_EDMA_DCHPRI13_ECP           (0x80)

/* Bit definitions and macros for DCHPRI14 */
#define MCF_EDMA_DCHPRI14_CHPRI(x)      (((x)&0x0F))
#define MCF_EDMA_DCHPRI14_ECP           (0x80)

/* Bit definitions and macros for DCHPRI15 */
#define MCF_EDMA_DCHPRI15_CHPRI(x)      (((x)&0x0F))
#define MCF_EDMA_DCHPRI15_ECP           (0x80)

/* Bit definitions and macros for TCD_SADDR group */
#define MCF_EDMA_TCD_SADDR_SADDR(x)     (x)

/* Bit definitions and macros for TCD0_SADDR */
#define MCF_EDMA_TCD0_SADDR_SADDR(x)    (x)

/* Bit definitions and macros for TCD_ATTR group */
#define MCF_EDMA_TCD_ATTR_DSIZE(x)          (((x)&0x0007))
#define MCF_EDMA_TCD_ATTR_DMOD(x)           (((x)&0x001F)<<3)
#define MCF_EDMA_TCD_ATTR_SSIZE(x)          (((x)&0x0007)<<8)
#define MCF_EDMA_TCD_ATTR_SMOD(x)           (((x)&0x001F)<<11)
#define MCF_EDMA_TCD_ATTR_SSIZE_8BIT        (0x0000)
#define MCF_EDMA_TCD_ATTR_SSIZE_16BIT       (0x0100)
#define MCF_EDMA_TCD_ATTR_SSIZE_32BIT       (0x0200)
#define MCF_EDMA_TCD_ATTR_SSIZE_16BYTE      (0x0400)
#define MCF_EDMA_TCD_ATTR_DSIZE_8BIT        (0x0000)
#define MCF_EDMA_TCD_ATTR_DSIZE_16BIT       (0x0001)
#define MCF_EDMA_TCD_ATTR_DSIZE_32BIT       (0x0002)
#define MCF_EDMA_TCD_ATTR_DSIZE_16BYTE      (0x0004)

/* Bit definitions and macros for TCD0_ATTR */
#define MCF_EDMA_TCD0_ATTR_DSIZE(x)         (((x)&0x0007))
#define MCF_EDMA_TCD0_ATTR_DMOD(x)          (((x)&0x001F)<<3)
#define MCF_EDMA_TCD0_ATTR_SSIZE(x)         (((x)&0x0007)<<8)
#define MCF_EDMA_TCD0_ATTR_SMOD(x)          (((x)&0x001F)<<11)
#define MCF_EDMA_TCD0_ATTR_SSIZE_8BIT       (0x0000)
#define MCF_EDMA_TCD0_ATTR_SSIZE_16BIT      (0x0100)
#define MCF_EDMA_TCD0_ATTR_SSIZE_32BIT      (0x0200)
#define MCF_EDMA_TCD0_ATTR_SSIZE_16BYTE     (0x0400)
#define MCF_EDMA_TCD0_ATTR_DSIZE_8BIT       (0x0000)
#define MCF_EDMA_TCD0_ATTR_DSIZE_16BIT      (0x0001)
#define MCF_EDMA_TCD0_ATTR_DSIZE_32BIT      (0x0002)
#define MCF_EDMA_TCD0_ATTR_DSIZE_16BYTE     (0x0004)

/* Bit definitions and macros for TCD_SOFF group */
#define MCF_EDMA_TCD_SOFF_SOFF(x)   (x)

/* Bit definitions and macros for TCD0_SOFF */
#define MCF_EDMA_TCD0_SOFF_SOFF(x)      (x)

/* Bit definitions and macros for TCD_NBYTES group */
#define MCF_EDMA_TCD_NBYTES_NBYTES(x)   (x)

/* Bit definitions and macros for TCD0_NBYTES */
#define MCF_EDMA_TCD0_NBYTES_NBYTES(x)      (x)

/* Bit definitions and macros for TCD_SLAST group */
#define MCF_EDMA_TCD_SLAST_SLAST(x)     (x)

/* Bit definitions and macros for TCD0_SLAST */
#define MCF_EDMA_TCD0_SLAST_SLAST(x)    (x)

/* Bit definitions and macros for TCD_DADDR group */
#define MCF_EDMA_TCD_DADDR_DADDR(x)     (x)

/* Bit definitions and macros for TCD0_DADDR */
#define MCF_EDMA_TCD0_DADDR_DADDR(x)    (x)

/* Bit definitions and macros for TCD_CITER_ELINK group */
#define MCF_EDMA_TCD_CITER_ELINK_CITER(x)       (((x)&0x01FF))
#define MCF_EDMA_TCD_CITER_ELINK_LINKCH(x)      (((x)&0x003F)<<9)
#define MCF_EDMA_TCD_CITER_ELINK_E_LINK         (0x8000)

/* Bit definitions and macros for TCD0_CITER_ELINK */
#define MCF_EDMA_TCD0_CITER_ELINK_CITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD0_CITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD0_CITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD_CITER group */
#define MCF_EDMA_TCD_CITER_CITER(x)     (((x)&0x7FFF))
#define MCF_EDMA_TCD_CITER_E_LINK       (0x8000)

/* Bit definitions and macros for TCD0_CITER */
#define MCF_EDMA_TCD0_CITER_CITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD0_CITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD_DOFF group */
#define MCF_EDMA_TCD_DOFF_DOFF(x)   (x)

/* Bit definitions and macros for TCD0_DOFF */
#define MCF_EDMA_TCD0_DOFF_DOFF(x)      (x)

/* Bit definitions and macros for TCD_DLAST_SGA group */
#define MCF_EDMA_TCD_DLAST_SGA_DLAST_SGA(x)     (x)

/* Bit definitions and macros for TCD0_DLAST_SGA */
#define MCF_EDMA_TCD0_DLAST_SGA_DLAST_SGA(x)    (x)

/* Bit definitions and macros for TCD_BITER_ELINK group */
#define MCF_EDMA_TCD_BITER_ELINK_BITER(x)       (((x)&0x01FF))
#define MCF_EDMA_TCD_BITER_ELINK_LINKCH(x)      (((x)&0x003F)<<9)
#define MCF_EDMA_TCD_BITER_ELINK_E_LINK         (0x8000)

/* Bit definitions and macros for TCD0_BITER_ELINK */
#define MCF_EDMA_TCD0_BITER_ELINK_BITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD0_BITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD0_BITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD_BITER group */
#define MCF_EDMA_TCD_BITER_BITER(x)     (((x)&0x7FFF))
#define MCF_EDMA_TCD_BITER_E_LINK       (0x8000)

/* Bit definitions and macros for TCD0_BITER */
#define MCF_EDMA_TCD0_BITER_BITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD0_BITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD_CSR group */
#define MCF_EDMA_TCD_CSR_START              (0x0001)
#define MCF_EDMA_TCD_CSR_INT_MAJOR          (0x0002)
#define MCF_EDMA_TCD_CSR_INT_HALF           (0x0004)
#define MCF_EDMA_TCD_CSR_D_REQ              (0x0008)
#define MCF_EDMA_TCD_CSR_E_SG               (0x0010)
#define MCF_EDMA_TCD_CSR_E_LINK             (0x0020)
#define MCF_EDMA_TCD_CSR_ACTIVE             (0x0040)
#define MCF_EDMA_TCD_CSR_DONE               (0x0080)
#define MCF_EDMA_TCD_CSR_LINKCH(x)          (((x)&0x003F)<<8)
#define MCF_EDMA_TCD_CSR_BWC(x)             (((x)&0x0003)<<14)
#define MCF_EDMA_TCD_CSR_BWC_NO_STALL       (0x0000)
#define MCF_EDMA_TCD_CSR_BWC_4CYC_STALL     (0x8000)
#define MCF_EDMA_TCD_CSR_BWC_8CYC_STALL     (0xC000)

/* Bit definitions and macros for TCD0_CSR */
#define MCF_EDMA_TCD0_CSR_START             (0x0001)
#define MCF_EDMA_TCD0_CSR_INT_MAJOR         (0x0002)
#define MCF_EDMA_TCD0_CSR_INT_HALF          (0x0004)
#define MCF_EDMA_TCD0_CSR_D_REQ             (0x0008)
#define MCF_EDMA_TCD0_CSR_E_SG              (0x0010)
#define MCF_EDMA_TCD0_CSR_E_LINK            (0x0020)
#define MCF_EDMA_TCD0_CSR_ACTIVE            (0x0040)
#define MCF_EDMA_TCD0_CSR_DONE              (0x0080)
#define MCF_EDMA_TCD0_CSR_LINKCH(x)         (((x)&0x003F)<<8)
#define MCF_EDMA_TCD0_CSR_BWC(x)            (((x)&0x0003)<<14)
#define MCF_EDMA_TCD0_CSR_BWC_NO_STALL      (0x0000)
#define MCF_EDMA_TCD0_CSR_BWC_4CYC_STALL    (0x8000)
#define MCF_EDMA_TCD0_CSR_BWC_8CYC_STALL    (0xC000)

/* Bit definitions and macros for TCD1_SADDR */
#define MCF_EDMA_TCD1_SADDR_SADDR(x)    (x)

/* Bit definitions and macros for TCD1_ATTR */
#define MCF_EDMA_TCD1_ATTR_DSIZE(x)     (((x)&0x0007))
#define MCF_EDMA_TCD1_ATTR_DMOD(x)      (((x)&0x001F)<<3)
#define MCF_EDMA_TCD1_ATTR_SSIZE(x)     (((x)&0x0007)<<8)
#define MCF_EDMA_TCD1_ATTR_SMOD(x)      (((x)&0x001F)<<11)

/* Bit definitions and macros for TCD1_SOFF */
#define MCF_EDMA_TCD1_SOFF_SOFF(x)      (x)

/* Bit definitions and macros for TCD1_NBYTES */
#define MCF_EDMA_TCD1_NBYTES_NBYTES(x)      (x)

/* Bit definitions and macros for TCD1_SLAST */
#define MCF_EDMA_TCD1_SLAST_SLAST(x)    (x)

/* Bit definitions and macros for TCD1_DADDR */
#define MCF_EDMA_TCD1_DADDR_DADDR(x)    (x)

/* Bit definitions and macros for TCD1_CITER_ELINK */
#define MCF_EDMA_TCD1_CITER_ELINK_CITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD1_CITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD1_CITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD1_CITER */
#define MCF_EDMA_TCD1_CITER_CITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD1_CITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD1_DOFF */
#define MCF_EDMA_TCD1_DOFF_DOFF(x)      (x)

/* Bit definitions and macros for TCD1_DLAST_SGA */
#define MCF_EDMA_TCD1_DLAST_SGA_DLAST_SGA(x)    (x)

/* Bit definitions and macros for TCD1_BITER */
#define MCF_EDMA_TCD1_BITER_BITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD1_BITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD1_BITER_ELINK */
#define MCF_EDMA_TCD1_BITER_ELINK_BITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD1_BITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD1_BITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD1_CSR */
#define MCF_EDMA_TCD1_CSR_START         (0x0001)
#define MCF_EDMA_TCD1_CSR_INT_MAJOR     (0x0002)
#define MCF_EDMA_TCD1_CSR_INT_HALF      (0x0004)
#define MCF_EDMA_TCD1_CSR_D_REQ         (0x0008)
#define MCF_EDMA_TCD1_CSR_E_SG          (0x0010)
#define MCF_EDMA_TCD1_CSR_E_LINK        (0x0020)
#define MCF_EDMA_TCD1_CSR_ACTIVE        (0x0040)
#define MCF_EDMA_TCD1_CSR_DONE          (0x0080)
#define MCF_EDMA_TCD1_CSR_LINKCH(x)     (((x)&0x003F)<<8)
#define MCF_EDMA_TCD1_CSR_BWC(x)        (((x)&0x0003)<<14)

/* Bit definitions and macros for TCD2_SADDR */
#define MCF_EDMA_TCD2_SADDR_SADDR(x)    (x)

/* Bit definitions and macros for TCD2_ATTR */
#define MCF_EDMA_TCD2_ATTR_DSIZE(x)     (((x)&0x0007))
#define MCF_EDMA_TCD2_ATTR_DMOD(x)      (((x)&0x001F)<<3)
#define MCF_EDMA_TCD2_ATTR_SSIZE(x)     (((x)&0x0007)<<8)
#define MCF_EDMA_TCD2_ATTR_SMOD(x)      (((x)&0x001F)<<11)

/* Bit definitions and macros for TCD2_SOFF */
#define MCF_EDMA_TCD2_SOFF_SOFF(x)      (x)

/* Bit definitions and macros for TCD2_NBYTES */
#define MCF_EDMA_TCD2_NBYTES_NBYTES(x)      (x)

/* Bit definitions and macros for TCD2_SLAST */
#define MCF_EDMA_TCD2_SLAST_SLAST(x)    (x)

/* Bit definitions and macros for TCD2_DADDR */
#define MCF_EDMA_TCD2_DADDR_DADDR(x)    (x)

/* Bit definitions and macros for TCD2_CITER */
#define MCF_EDMA_TCD2_CITER_CITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD2_CITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD2_CITER_ELINK */
#define MCF_EDMA_TCD2_CITER_ELINK_CITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD2_CITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD2_CITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD2_DOFF */
#define MCF_EDMA_TCD2_DOFF_DOFF(x)      (x)

/* Bit definitions and macros for TCD2_DLAST_SGA */
#define MCF_EDMA_TCD2_DLAST_SGA_DLAST_SGA(x)    (x)

/* Bit definitions and macros for TCD2_BITER_ELINK */
#define MCF_EDMA_TCD2_BITER_ELINK_BITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD2_BITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD2_BITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD2_BITER */
#define MCF_EDMA_TCD2_BITER_BITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD2_BITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD2_CSR */
#define MCF_EDMA_TCD2_CSR_START         (0x0001)
#define MCF_EDMA_TCD2_CSR_INT_MAJOR     (0x0002)
#define MCF_EDMA_TCD2_CSR_INT_HALF      (0x0004)
#define MCF_EDMA_TCD2_CSR_D_REQ         (0x0008)
#define MCF_EDMA_TCD2_CSR_E_SG          (0x0010)
#define MCF_EDMA_TCD2_CSR_E_LINK        (0x0020)
#define MCF_EDMA_TCD2_CSR_ACTIVE        (0x0040)
#define MCF_EDMA_TCD2_CSR_DONE          (0x0080)
#define MCF_EDMA_TCD2_CSR_LINKCH(x)     (((x)&0x003F)<<8)
#define MCF_EDMA_TCD2_CSR_BWC(x)        (((x)&0x0003)<<14)

/* Bit definitions and macros for TCD3_SADDR */
#define MCF_EDMA_TCD3_SADDR_SADDR(x)    (x)

/* Bit definitions and macros for TCD3_ATTR */
#define MCF_EDMA_TCD3_ATTR_DSIZE(x)     (((x)&0x0007))
#define MCF_EDMA_TCD3_ATTR_DMOD(x)      (((x)&0x001F)<<3)
#define MCF_EDMA_TCD3_ATTR_SSIZE(x)     (((x)&0x0007)<<8)
#define MCF_EDMA_TCD3_ATTR_SMOD(x)      (((x)&0x001F)<<11)

/* Bit definitions and macros for TCD3_SOFF */
#define MCF_EDMA_TCD3_SOFF_SOFF(x)      (x)

/* Bit definitions and macros for TCD3_NBYTES */
#define MCF_EDMA_TCD3_NBYTES_NBYTES(x)      (x)

/* Bit definitions and macros for TCD3_SLAST */
#define MCF_EDMA_TCD3_SLAST_SLAST(x)    (x)

/* Bit definitions and macros for TCD3_DADDR */
#define MCF_EDMA_TCD3_DADDR_DADDR(x)    (x)

/* Bit definitions and macros for TCD3_CITER */
#define MCF_EDMA_TCD3_CITER_CITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD3_CITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD3_CITER_ELINK */
#define MCF_EDMA_TCD3_CITER_ELINK_CITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD3_CITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD3_CITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD3_DOFF */
#define MCF_EDMA_TCD3_DOFF_DOFF(x)      (x)

/* Bit definitions and macros for TCD3_DLAST_SGA */
#define MCF_EDMA_TCD3_DLAST_SGA_DLAST_SGA(x)    (x)

/* Bit definitions and macros for TCD3_BITER_ELINK */
#define MCF_EDMA_TCD3_BITER_ELINK_BITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD3_BITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD3_BITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD3_BITER */
#define MCF_EDMA_TCD3_BITER_BITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD3_BITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD3_CSR */
#define MCF_EDMA_TCD3_CSR_START         (0x0001)
#define MCF_EDMA_TCD3_CSR_INT_MAJOR     (0x0002)
#define MCF_EDMA_TCD3_CSR_INT_HALF      (0x0004)
#define MCF_EDMA_TCD3_CSR_D_REQ         (0x0008)
#define MCF_EDMA_TCD3_CSR_E_SG          (0x0010)
#define MCF_EDMA_TCD3_CSR_E_LINK        (0x0020)
#define MCF_EDMA_TCD3_CSR_ACTIVE        (0x0040)
#define MCF_EDMA_TCD3_CSR_DONE          (0x0080)
#define MCF_EDMA_TCD3_CSR_LINKCH(x)     (((x)&0x003F)<<8)
#define MCF_EDMA_TCD3_CSR_BWC(x)        (((x)&0x0003)<<14)

/* Bit definitions and macros for TCD4_SADDR */
#define MCF_EDMA_TCD4_SADDR_SADDR(x)    (x)

/* Bit definitions and macros for TCD4_ATTR */
#define MCF_EDMA_TCD4_ATTR_DSIZE(x)     (((x)&0x0007))
#define MCF_EDMA_TCD4_ATTR_DMOD(x)      (((x)&0x001F)<<3)
#define MCF_EDMA_TCD4_ATTR_SSIZE(x)     (((x)&0x0007)<<8)
#define MCF_EDMA_TCD4_ATTR_SMOD(x)      (((x)&0x001F)<<11)

/* Bit definitions and macros for TCD4_SOFF */
#define MCF_EDMA_TCD4_SOFF_SOFF(x)      (x)

/* Bit definitions and macros for TCD4_NBYTES */
#define MCF_EDMA_TCD4_NBYTES_NBYTES(x)      (x)

/* Bit definitions and macros for TCD4_SLAST */
#define MCF_EDMA_TCD4_SLAST_SLAST(x)    (x)

/* Bit definitions and macros for TCD4_DADDR */
#define MCF_EDMA_TCD4_DADDR_DADDR(x)    (x)

/* Bit definitions and macros for TCD4_CITER */
#define MCF_EDMA_TCD4_CITER_CITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD4_CITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD4_CITER_ELINK */
#define MCF_EDMA_TCD4_CITER_ELINK_CITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD4_CITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD4_CITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD4_DOFF */
#define MCF_EDMA_TCD4_DOFF_DOFF(x)      (x)

/* Bit definitions and macros for TCD4_DLAST_SGA */
#define MCF_EDMA_TCD4_DLAST_SGA_DLAST_SGA(x)    (x)

/* Bit definitions and macros for TCD4_BITER */
#define MCF_EDMA_TCD4_BITER_BITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD4_BITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD4_BITER_ELINK */
#define MCF_EDMA_TCD4_BITER_ELINK_BITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD4_BITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD4_BITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD4_CSR */
#define MCF_EDMA_TCD4_CSR_START         (0x0001)
#define MCF_EDMA_TCD4_CSR_INT_MAJOR     (0x0002)
#define MCF_EDMA_TCD4_CSR_INT_HALF      (0x0004)
#define MCF_EDMA_TCD4_CSR_D_REQ         (0x0008)
#define MCF_EDMA_TCD4_CSR_E_SG          (0x0010)
#define MCF_EDMA_TCD4_CSR_E_LINK        (0x0020)
#define MCF_EDMA_TCD4_CSR_ACTIVE        (0x0040)
#define MCF_EDMA_TCD4_CSR_DONE          (0x0080)
#define MCF_EDMA_TCD4_CSR_LINKCH(x)     (((x)&0x003F)<<8)
#define MCF_EDMA_TCD4_CSR_BWC(x)        (((x)&0x0003)<<14)

/* Bit definitions and macros for TCD5_SADDR */
#define MCF_EDMA_TCD5_SADDR_SADDR(x)    (x)

/* Bit definitions and macros for TCD5_ATTR */
#define MCF_EDMA_TCD5_ATTR_DSIZE(x)     (((x)&0x0007))
#define MCF_EDMA_TCD5_ATTR_DMOD(x)      (((x)&0x001F)<<3)
#define MCF_EDMA_TCD5_ATTR_SSIZE(x)     (((x)&0x0007)<<8)
#define MCF_EDMA_TCD5_ATTR_SMOD(x)      (((x)&0x001F)<<11)

/* Bit definitions and macros for TCD5_SOFF */
#define MCF_EDMA_TCD5_SOFF_SOFF(x)      (x)

/* Bit definitions and macros for TCD5_NBYTES */
#define MCF_EDMA_TCD5_NBYTES_NBYTES(x)      (x)

/* Bit definitions and macros for TCD5_SLAST */
#define MCF_EDMA_TCD5_SLAST_SLAST(x)    (x)

/* Bit definitions and macros for TCD5_DADDR */
#define MCF_EDMA_TCD5_DADDR_DADDR(x)    (x)

/* Bit definitions and macros for TCD5_CITER */
#define MCF_EDMA_TCD5_CITER_CITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD5_CITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD5_CITER_ELINK */
#define MCF_EDMA_TCD5_CITER_ELINK_CITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD5_CITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD5_CITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD5_DOFF */
#define MCF_EDMA_TCD5_DOFF_DOFF(x)      (x)

/* Bit definitions and macros for TCD5_DLAST_SGA */
#define MCF_EDMA_TCD5_DLAST_SGA_DLAST_SGA(x)    (x)

/* Bit definitions and macros for TCD5_BITER_ELINK */
#define MCF_EDMA_TCD5_BITER_ELINK_BITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD5_BITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD5_BITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD5_BITER */
#define MCF_EDMA_TCD5_BITER_BITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD5_BITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD5_CSR */
#define MCF_EDMA_TCD5_CSR_START         (0x0001)
#define MCF_EDMA_TCD5_CSR_INT_MAJOR     (0x0002)
#define MCF_EDMA_TCD5_CSR_INT_HALF      (0x0004)
#define MCF_EDMA_TCD5_CSR_D_REQ         (0x0008)
#define MCF_EDMA_TCD5_CSR_E_SG          (0x0010)
#define MCF_EDMA_TCD5_CSR_E_LINK        (0x0020)
#define MCF_EDMA_TCD5_CSR_ACTIVE        (0x0040)
#define MCF_EDMA_TCD5_CSR_DONE          (0x0080)
#define MCF_EDMA_TCD5_CSR_LINKCH(x)     (((x)&0x003F)<<8)
#define MCF_EDMA_TCD5_CSR_BWC(x)        (((x)&0x0003)<<14)

/* Bit definitions and macros for TCD6_SADDR */
#define MCF_EDMA_TCD6_SADDR_SADDR(x)    (x)

/* Bit definitions and macros for TCD6_ATTR */
#define MCF_EDMA_TCD6_ATTR_DSIZE(x)     (((x)&0x0007))
#define MCF_EDMA_TCD6_ATTR_DMOD(x)      (((x)&0x001F)<<3)
#define MCF_EDMA_TCD6_ATTR_SSIZE(x)     (((x)&0x0007)<<8)
#define MCF_EDMA_TCD6_ATTR_SMOD(x)      (((x)&0x001F)<<11)

/* Bit definitions and macros for TCD6_SOFF */
#define MCF_EDMA_TCD6_SOFF_SOFF(x)      (x)

/* Bit definitions and macros for TCD6_NBYTES */
#define MCF_EDMA_TCD6_NBYTES_NBYTES(x)      (x)

/* Bit definitions and macros for TCD6_SLAST */
#define MCF_EDMA_TCD6_SLAST_SLAST(x)    (x)

/* Bit definitions and macros for TCD6_DADDR */
#define MCF_EDMA_TCD6_DADDR_DADDR(x)    (x)

/* Bit definitions and macros for TCD6_CITER */
#define MCF_EDMA_TCD6_CITER_CITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD6_CITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD6_CITER_ELINK */
#define MCF_EDMA_TCD6_CITER_ELINK_CITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD6_CITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD6_CITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD6_DOFF */
#define MCF_EDMA_TCD6_DOFF_DOFF(x)      (x)

/* Bit definitions and macros for TCD6_DLAST_SGA */
#define MCF_EDMA_TCD6_DLAST_SGA_DLAST_SGA(x)    (x)

/* Bit definitions and macros for TCD6_BITER_ELINK */
#define MCF_EDMA_TCD6_BITER_ELINK_BITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD6_BITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD6_BITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD6_BITER */
#define MCF_EDMA_TCD6_BITER_BITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD6_BITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD6_CSR */
#define MCF_EDMA_TCD6_CSR_START         (0x0001)
#define MCF_EDMA_TCD6_CSR_INT_MAJOR     (0x0002)
#define MCF_EDMA_TCD6_CSR_INT_HALF      (0x0004)
#define MCF_EDMA_TCD6_CSR_D_REQ         (0x0008)
#define MCF_EDMA_TCD6_CSR_E_SG          (0x0010)
#define MCF_EDMA_TCD6_CSR_E_LINK        (0x0020)
#define MCF_EDMA_TCD6_CSR_ACTIVE        (0x0040)
#define MCF_EDMA_TCD6_CSR_DONE          (0x0080)
#define MCF_EDMA_TCD6_CSR_LINKCH(x)     (((x)&0x003F)<<8)
#define MCF_EDMA_TCD6_CSR_BWC(x)        (((x)&0x0003)<<14)

/* Bit definitions and macros for TCD7_SADDR */
#define MCF_EDMA_TCD7_SADDR_SADDR(x)    (x)

/* Bit definitions and macros for TCD7_ATTR */
#define MCF_EDMA_TCD7_ATTR_DSIZE(x)     (((x)&0x0007))
#define MCF_EDMA_TCD7_ATTR_DMOD(x)      (((x)&0x001F)<<3)
#define MCF_EDMA_TCD7_ATTR_SSIZE(x)     (((x)&0x0007)<<8)
#define MCF_EDMA_TCD7_ATTR_SMOD(x)      (((x)&0x001F)<<11)

/* Bit definitions and macros for TCD7_SOFF */
#define MCF_EDMA_TCD7_SOFF_SOFF(x)      (x)

/* Bit definitions and macros for TCD7_NBYTES */
#define MCF_EDMA_TCD7_NBYTES_NBYTES(x)      (x)

/* Bit definitions and macros for TCD7_SLAST */
#define MCF_EDMA_TCD7_SLAST_SLAST(x)    (x)

/* Bit definitions and macros for TCD7_DADDR */
#define MCF_EDMA_TCD7_DADDR_DADDR(x)    (x)

/* Bit definitions and macros for TCD7_CITER */
#define MCF_EDMA_TCD7_CITER_CITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD7_CITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD7_CITER_ELINK */
#define MCF_EDMA_TCD7_CITER_ELINK_CITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD7_CITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD7_CITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD7_DOFF */
#define MCF_EDMA_TCD7_DOFF_DOFF(x)      (x)

/* Bit definitions and macros for TCD7_DLAST_SGA */
#define MCF_EDMA_TCD7_DLAST_SGA_DLAST_SGA(x)    (x)

/* Bit definitions and macros for TCD7_BITER_ELINK */
#define MCF_EDMA_TCD7_BITER_ELINK_BITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD7_BITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD7_BITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD7_BITER */
#define MCF_EDMA_TCD7_BITER_BITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD7_BITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD7_CSR */
#define MCF_EDMA_TCD7_CSR_START         (0x0001)
#define MCF_EDMA_TCD7_CSR_INT_MAJOR     (0x0002)
#define MCF_EDMA_TCD7_CSR_INT_HALF      (0x0004)
#define MCF_EDMA_TCD7_CSR_D_REQ         (0x0008)
#define MCF_EDMA_TCD7_CSR_E_SG          (0x0010)
#define MCF_EDMA_TCD7_CSR_E_LINK        (0x0020)
#define MCF_EDMA_TCD7_CSR_ACTIVE        (0x0040)
#define MCF_EDMA_TCD7_CSR_DONE          (0x0080)
#define MCF_EDMA_TCD7_CSR_LINKCH(x)     (((x)&0x003F)<<8)
#define MCF_EDMA_TCD7_CSR_BWC(x)        (((x)&0x0003)<<14)

/* Bit definitions and macros for TCD8_SADDR */
#define MCF_EDMA_TCD8_SADDR_SADDR(x)    (x)

/* Bit definitions and macros for TCD8_ATTR */
#define MCF_EDMA_TCD8_ATTR_DSIZE(x)     (((x)&0x0007))
#define MCF_EDMA_TCD8_ATTR_DMOD(x)      (((x)&0x001F)<<3)
#define MCF_EDMA_TCD8_ATTR_SSIZE(x)     (((x)&0x0007)<<8)
#define MCF_EDMA_TCD8_ATTR_SMOD(x)      (((x)&0x001F)<<11)

/* Bit definitions and macros for TCD8_SOFF */
#define MCF_EDMA_TCD8_SOFF_SOFF(x)      (x)

/* Bit definitions and macros for TCD8_NBYTES */
#define MCF_EDMA_TCD8_NBYTES_NBYTES(x)      (x)

/* Bit definitions and macros for TCD8_SLAST */
#define MCF_EDMA_TCD8_SLAST_SLAST(x)    (x)

/* Bit definitions and macros for TCD8_DADDR */
#define MCF_EDMA_TCD8_DADDR_DADDR(x)    (x)

/* Bit definitions and macros for TCD8_CITER */
#define MCF_EDMA_TCD8_CITER_CITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD8_CITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD8_CITER_ELINK */
#define MCF_EDMA_TCD8_CITER_ELINK_CITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD8_CITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD8_CITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD8_DOFF */
#define MCF_EDMA_TCD8_DOFF_DOFF(x)      (x)

/* Bit definitions and macros for TCD8_DLAST_SGA */
#define MCF_EDMA_TCD8_DLAST_SGA_DLAST_SGA(x)    (x)

/* Bit definitions and macros for TCD8_BITER_ELINK */
#define MCF_EDMA_TCD8_BITER_ELINK_BITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD8_BITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD8_BITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD8_BITER */
#define MCF_EDMA_TCD8_BITER_BITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD8_BITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD8_CSR */
#define MCF_EDMA_TCD8_CSR_START         (0x0001)
#define MCF_EDMA_TCD8_CSR_INT_MAJOR     (0x0002)
#define MCF_EDMA_TCD8_CSR_INT_HALF      (0x0004)
#define MCF_EDMA_TCD8_CSR_D_REQ         (0x0008)
#define MCF_EDMA_TCD8_CSR_E_SG          (0x0010)
#define MCF_EDMA_TCD8_CSR_E_LINK        (0x0020)
#define MCF_EDMA_TCD8_CSR_ACTIVE        (0x0040)
#define MCF_EDMA_TCD8_CSR_DONE          (0x0080)
#define MCF_EDMA_TCD8_CSR_LINKCH(x)     (((x)&0x003F)<<8)
#define MCF_EDMA_TCD8_CSR_BWC(x)        (((x)&0x0003)<<14)

/* Bit definitions and macros for TCD9_SADDR */
#define MCF_EDMA_TCD9_SADDR_SADDR(x)    (x)

/* Bit definitions and macros for TCD9_ATTR */
#define MCF_EDMA_TCD9_ATTR_DSIZE(x)     (((x)&0x0007))
#define MCF_EDMA_TCD9_ATTR_DMOD(x)      (((x)&0x001F)<<3)
#define MCF_EDMA_TCD9_ATTR_SSIZE(x)     (((x)&0x0007)<<8)
#define MCF_EDMA_TCD9_ATTR_SMOD(x)      (((x)&0x001F)<<11)

/* Bit definitions and macros for TCD9_SOFF */
#define MCF_EDMA_TCD9_SOFF_SOFF(x)      (x)

/* Bit definitions and macros for TCD9_NBYTES */
#define MCF_EDMA_TCD9_NBYTES_NBYTES(x)      (x)

/* Bit definitions and macros for TCD9_SLAST */
#define MCF_EDMA_TCD9_SLAST_SLAST(x)    (x)

/* Bit definitions and macros for TCD9_DADDR */
#define MCF_EDMA_TCD9_DADDR_DADDR(x)    (x)

/* Bit definitions and macros for TCD9_CITER_ELINK */
#define MCF_EDMA_TCD9_CITER_ELINK_CITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD9_CITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD9_CITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD9_CITER */
#define MCF_EDMA_TCD9_CITER_CITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD9_CITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD9_DOFF */
#define MCF_EDMA_TCD9_DOFF_DOFF(x)      (x)

/* Bit definitions and macros for TCD9_DLAST_SGA */
#define MCF_EDMA_TCD9_DLAST_SGA_DLAST_SGA(x)    (x)

/* Bit definitions and macros for TCD9_BITER_ELINK */
#define MCF_EDMA_TCD9_BITER_ELINK_BITER(x)      (((x)&0x01FF))
#define MCF_EDMA_TCD9_BITER_ELINK_LINKCH(x)     (((x)&0x003F)<<9)
#define MCF_EDMA_TCD9_BITER_ELINK_E_LINK        (0x8000)

/* Bit definitions and macros for TCD9_BITER */
#define MCF_EDMA_TCD9_BITER_BITER(x)    (((x)&0x7FFF))
#define MCF_EDMA_TCD9_BITER_E_LINK      (0x8000)

/* Bit definitions and macros for TCD9_CSR */
#define MCF_EDMA_TCD9_CSR_START         (0x0001)
#define MCF_EDMA_TCD9_CSR_INT_MAJOR     (0x0002)
#define MCF_EDMA_TCD9_CSR_INT_HALF      (0x0004)
#define MCF_EDMA_TCD9_CSR_D_REQ         (0x0008)
#define MCF_EDMA_TCD9_CSR_E_SG          (0x0010)
#define MCF_EDMA_TCD9_CSR_E_LINK        (0x0020)
#define MCF_EDMA_TCD9_CSR_ACTIVE        (0x0040)
#define MCF_EDMA_TCD9_CSR_DONE          (0x0080)
#define MCF_EDMA_TCD9_CSR_LINKCH(x)     (((x)&0x003F)<<8)
#define MCF_EDMA_TCD9_CSR_BWC(x)        (((x)&0x0003)<<14)

/* Bit definitions and macros for TCD10_SADDR */
#define MCF_EDMA_TCD10_SADDR_SADDR(x)   (x)

/* Bit definitions and macros for TCD10_ATTR */
#define MCF_EDMA_TCD10_ATTR_DSIZE(x)    (((x)&0x0007))
#define MCF_EDMA_TCD10_ATTR_DMOD(x)     (((x)&0x001F)<<3)
#define MCF_EDMA_TCD10_ATTR_SSIZE(x)    (((x)&0x0007)<<8)
#define MCF_EDMA_TCD10_ATTR_SMOD(x)     (((x)&0x001F)<<11)

/* Bit definitions and macros for TCD10_SOFF */
#define MCF_EDMA_TCD10_SOFF_SOFF(x)     (x)

/* Bit definitions and macros for TCD10_NBYTES */
#define MCF_EDMA_TCD10_NBYTES_NBYTES(x)     (x)

/* Bit definitions and macros for TCD10_SLAST */
#define MCF_EDMA_TCD10_SLAST_SLAST(x)   (x)

/* Bit definitions and macros for TCD10_DADDR */
#define MCF_EDMA_TCD10_DADDR_DADDR(x)   (x)

/* Bit definitions and macros for TCD10_CITER_ELINK */
#define MCF_EDMA_TCD10_CITER_ELINK_CITER(x)     (((x)&0x01FF))
#define MCF_EDMA_TCD10_CITER_ELINK_LINKCH(x)    (((x)&0x003F)<<9)
#define MCF_EDMA_TCD10_CITER_ELINK_E_LINK       (0x8000)

/* Bit definitions and macros for TCD10_CITER */
#define MCF_EDMA_TCD10_CITER_CITER(x)   (((x)&0x7FFF))
#define MCF_EDMA_TCD10_CITER_E_LINK     (0x8000)

/* Bit definitions and macros for TCD10_DOFF */
#define MCF_EDMA_TCD10_DOFF_DOFF(x)     (x)

/* Bit definitions and macros for TCD10_DLAST_SGA */
#define MCF_EDMA_TCD10_DLAST_SGA_DLAST_SGA(x)   (x)

/* Bit definitions and macros for TCD10_BITER */
#define MCF_EDMA_TCD10_BITER_BITER(x)   (((x)&0x7FFF))
#define MCF_EDMA_TCD10_BITER_E_LINK     (0x8000)

/* Bit definitions and macros for TCD10_BITER_ELINK */
#define MCF_EDMA_TCD10_BITER_ELINK_BITER(x)     (((x)&0x01FF))
#define MCF_EDMA_TCD10_BITER_ELINK_LINKCH(x)    (((x)&0x003F)<<9)
#define MCF_EDMA_TCD10_BITER_ELINK_E_LINK       (0x8000)

/* Bit definitions and macros for TCD10_CSR */
#define MCF_EDMA_TCD10_CSR_START        (0x0001)
#define MCF_EDMA_TCD10_CSR_INT_MAJOR    (0x0002)
#define MCF_EDMA_TCD10_CSR_INT_HALF     (0x0004)
#define MCF_EDMA_TCD10_CSR_D_REQ        (0x0008)
#define MCF_EDMA_TCD10_CSR_E_SG         (0x0010)
#define MCF_EDMA_TCD10_CSR_E_LINK       (0x0020)
#define MCF_EDMA_TCD10_CSR_ACTIVE       (0x0040)
#define MCF_EDMA_TCD10_CSR_DONE         (0x0080)
#define MCF_EDMA_TCD10_CSR_LINKCH(x)    (((x)&0x003F)<<8)
#define MCF_EDMA_TCD10_CSR_BWC(x)       (((x)&0x0003)<<14)

/* Bit definitions and macros for TCD11_SADDR */
#define MCF_EDMA_TCD11_SADDR_SADDR(x)   (x)

/* Bit definitions and macros for TCD11_ATTR */
#define MCF_EDMA_TCD11_ATTR_DSIZE(x)    (((x)&0x0007))
#define MCF_EDMA_TCD11_ATTR_DMOD(x)     (((x)&0x001F)<<3)
#define MCF_EDMA_TCD11_ATTR_SSIZE(x)    (((x)&0x0007)<<8)
#define MCF_EDMA_TCD11_ATTR_SMOD(x)     (((x)&0x001F)<<11)

/* Bit definitions and macros for TCD11_SOFF */
#define MCF_EDMA_TCD11_SOFF_SOFF(x)     (x)

/* Bit definitions and macros for TCD11_NBYTES */
#define MCF_EDMA_TCD11_NBYTES_NBYTES(x)     (x)

/* Bit definitions and macros for TCD11_SLAST */
#define MCF_EDMA_TCD11_SLAST_SLAST(x)   (x)

/* Bit definitions and macros for TCD11_DADDR */
#define MCF_EDMA_TCD11_DADDR_DADDR(x)   (x)

/* Bit definitions and macros for TCD11_CITER */
#define MCF_EDMA_TCD11_CITER_CITER(x)   (((x)&0x7FFF))
#define MCF_EDMA_TCD11_CITER_E_LINK     (0x8000)

/* Bit definitions and macros for TCD11_CITER_ELINK */
#define MCF_EDMA_TCD11_CITER_ELINK_CITER(x)     (((x)&0x01FF))
#define MCF_EDMA_TCD11_CITER_ELINK_LINKCH(x)    (((x)&0x003F)<<9)
#define MCF_EDMA_TCD11_CITER_ELINK_E_LINK       (0x8000)

/* Bit definitions and macros for TCD11_DOFF */
#define MCF_EDMA_TCD11_DOFF_DOFF(x)     (x)

/* Bit definitions and macros for TCD11_DLAST_SGA */
#define MCF_EDMA_TCD11_DLAST_SGA_DLAST_SGA(x)   (x)

/* Bit definitions and macros for TCD11_BITER */
#define MCF_EDMA_TCD11_BITER_BITER(x)   (((x)&0x7FFF))
#define MCF_EDMA_TCD11_BITER_E_LINK     (0x8000)

/* Bit definitions and macros for TCD11_BITER_ELINK */
#define MCF_EDMA_TCD11_BITER_ELINK_BITER(x)     (((x)&0x01FF))
#define MCF_EDMA_TCD11_BITER_ELINK_LINKCH(x)    (((x)&0x003F)<<9)
#define MCF_EDMA_TCD11_BITER_ELINK_E_LINK       (0x8000)

/* Bit definitions and macros for TCD11_CSR */
#define MCF_EDMA_TCD11_CSR_START        (0x0001)
#define MCF_EDMA_TCD11_CSR_INT_MAJOR    (0x0002)
#define MCF_EDMA_TCD11_CSR_INT_HALF     (0x0004)
#define MCF_EDMA_TCD11_CSR_D_REQ        (0x0008)
#define MCF_EDMA_TCD11_CSR_E_SG         (0x0010)
#define MCF_EDMA_TCD11_CSR_E_LINK       (0x0020)
#define MCF_EDMA_TCD11_CSR_ACTIVE       (0x0040)
#define MCF_EDMA_TCD11_CSR_DONE         (0x0080)
#define MCF_EDMA_TCD11_CSR_LINKCH(x)    (((x)&0x003F)<<8)
#define MCF_EDMA_TCD11_CSR_BWC(x)       (((x)&0x0003)<<14)

/* Bit definitions and macros for TCD12_SADDR */
#define MCF_EDMA_TCD12_SADDR_SADDR(x)   (x)

/* Bit definitions and macros for TCD12_ATTR */
#define MCF_EDMA_TCD12_ATTR_DSIZE(x)    (((x)&0x0007))
#define MCF_EDMA_TCD12_ATTR_DMOD(x)     (((x)&0x001F)<<3)
#define MCF_EDMA_TCD12_ATTR_SSIZE(x)    (((x)&0x0007)<<8)
#define MCF_EDMA_TCD12_ATTR_SMOD(x)     (((x)&0x001F)<<11)

/* Bit definitions and macros for TCD12_SOFF */
#define MCF_EDMA_TCD12_SOFF_SOFF(x)     (x)

/* Bit definitions and macros for TCD12_NBYTES */
#define MCF_EDMA_TCD12_NBYTES_NBYTES(x)     (x)

/* Bit definitions and macros for TCD12_SLAST */
#define MCF_EDMA_TCD12_SLAST_SLAST(x)   (x)

/* Bit definitions and macros for TCD12_DADDR */
#define MCF_EDMA_TCD12_DADDR_DADDR(x)   (x)

/* Bit definitions and macros for TCD12_CITER */
#define MCF_EDMA_TCD12_CITER_CITER(x)   (((x)&0x7FFF))
#define MCF_EDMA_TCD12_CITER_E_LINK     (0x8000)

/* Bit definitions and macros for TCD12_CITER_ELINK */
#define MCF_EDMA_TCD12_CITER_ELINK_CITER(x)     (((x)&0x01FF))
#define MCF_EDMA_TCD12_CITER_ELINK_LINKCH(x)    (((x)&0x003F)<<9)
#define MCF_EDMA_TCD12_CITER_ELINK_E_LINK       (0x8000)

/* Bit definitions and macros for TCD12_DOFF */
#define MCF_EDMA_TCD12_DOFF_DOFF(x)     (x)

/* Bit definitions and macros for TCD12_DLAST_SGA */
#define MCF_EDMA_TCD12_DLAST_SGA_DLAST_SGA(x)   (x)

/* Bit definitions and macros for TCD12_BITER */
#define MCF_EDMA_TCD12_BITER_BITER(x)   (((x)&0x7FFF))
#define MCF_EDMA_TCD12_BITER_E_LINK     (0x8000)

/* Bit definitions and macros for TCD12_BITER_ELINK */
#define MCF_EDMA_TCD12_BITER_ELINK_BITER(x)     (((x)&0x01FF))
#define MCF_EDMA_TCD12_BITER_ELINK_LINKCH(x)    (((x)&0x003F)<<9)
#define MCF_EDMA_TCD12_BITER_ELINK_E_LINK       (0x8000)

/* Bit definitions and macros for TCD12_CSR */
#define MCF_EDMA_TCD12_CSR_START        (0x0001)
#define MCF_EDMA_TCD12_CSR_INT_MAJOR    (0x0002)
#define MCF_EDMA_TCD12_CSR_INT_HALF     (0x0004)
#define MCF_EDMA_TCD12_CSR_D_REQ        (0x0008)
#define MCF_EDMA_TCD12_CSR_E_SG         (0x0010)
#define MCF_EDMA_TCD12_CSR_E_LINK       (0x0020)
#define MCF_EDMA_TCD12_CSR_ACTIVE       (0x0040)
#define MCF_EDMA_TCD12_CSR_DONE         (0x0080)
#define MCF_EDMA_TCD12_CSR_LINKCH(x)    (((x)&0x003F)<<8)
#define MCF_EDMA_TCD12_CSR_BWC(x)       (((x)&0x0003)<<14)

/* Bit definitions and macros for TCD13_SADDR */
#define MCF_EDMA_TCD13_SADDR_SADDR(x)   (x)

/* Bit definitions and macros for TCD13_ATTR */
#define MCF_EDMA_TCD13_ATTR_DSIZE(x)    (((x)&0x0007))
#define MCF_EDMA_TCD13_ATTR_DMOD(x)     (((x)&0x001F)<<3)
#define MCF_EDMA_TCD13_ATTR_SSIZE(x)    (((x)&0x0007)<<8)
#define MCF_EDMA_TCD13_ATTR_SMOD(x)     (((x)&0x001F)<<11)

/* Bit definitions and macros for TCD13_SOFF */
#define MCF_EDMA_TCD13_SOFF_SOFF(x)     (x)

/* Bit definitions and macros for TCD13_NBYTES */
#define MCF_EDMA_TCD13_NBYTES_NBYTES(x)     (x)

/* Bit definitions and macros for TCD13_SLAST */
#define MCF_EDMA_TCD13_SLAST_SLAST(x)   (x)

/* Bit definitions and macros for TCD13_DADDR */
#define MCF_EDMA_TCD13_DADDR_DADDR(x)   (x)

/* Bit definitions and macros for TCD13_CITER_ELINK */
#define MCF_EDMA_TCD13_CITER_ELINK_CITER(x)     (((x)&0x01FF))
#define MCF_EDMA_TCD13_CITER_ELINK_LINKCH(x)    (((x)&0x003F)<<9)
#define MCF_EDMA_TCD13_CITER_ELINK_E_LINK       (0x8000)

/* Bit definitions and macros for TCD13_CITER */
#define MCF_EDMA_TCD13_CITER_CITER(x)   (((x)&0x7FFF))
#define MCF_EDMA_TCD13_CITER_E_LINK     (0x8000)

/* Bit definitions and macros for TCD13_DOFF */
#define MCF_EDMA_TCD13_DOFF_DOFF(x)     (x)

/* Bit definitions and macros for TCD13_DLAST_SGA */
#define MCF_EDMA_TCD13_DLAST_SGA_DLAST_SGA(x)   (x)

/* Bit definitions and macros for TCD13_BITER_ELINK */
#define MCF_EDMA_TCD13_BITER_ELINK_BITER(x)     (((x)&0x01FF))
#define MCF_EDMA_TCD13_BITER_ELINK_LINKCH(x)    (((x)&0x003F)<<9)
#define MCF_EDMA_TCD13_BITER_ELINK_E_LINK       (0x8000)

/* Bit definitions and macros for TCD13_BITER */
#define MCF_EDMA_TCD13_BITER_BITER(x)   (((x)&0x7FFF))
#define MCF_EDMA_TCD13_BITER_E_LINK     (0x8000)

/* Bit definitions and macros for TCD13_CSR */
#define MCF_EDMA_TCD13_CSR_START        (0x0001)
#define MCF_EDMA_TCD13_CSR_INT_MAJOR    (0x0002)
#define MCF_EDMA_TCD13_CSR_INT_HALF     (0x0004)
#define MCF_EDMA_TCD13_CSR_D_REQ        (0x0008)
#define MCF_EDMA_TCD13_CSR_E_SG         (0x0010)
#define MCF_EDMA_TCD13_CSR_E_LINK       (0x0020)
#define MCF_EDMA_TCD13_CSR_ACTIVE       (0x0040)
#define MCF_EDMA_TCD13_CSR_DONE         (0x0080)
#define MCF_EDMA_TCD13_CSR_LINKCH(x)    (((x)&0x003F)<<8)
#define MCF_EDMA_TCD13_CSR_BWC(x)       (((x)&0x0003)<<14)

/* Bit definitions and macros for TCD14_SADDR */
#define MCF_EDMA_TCD14_SADDR_SADDR(x)   (x)

/* Bit definitions and macros for TCD14_ATTR */
#define MCF_EDMA_TCD14_ATTR_DSIZE(x)    (((x)&0x0007))
#define MCF_EDMA_TCD14_ATTR_DMOD(x)     (((x)&0x001F)<<3)
#define MCF_EDMA_TCD14_ATTR_SSIZE(x)    (((x)&0x0007)<<8)
#define MCF_EDMA_TCD14_ATTR_SMOD(x)     (((x)&0x001F)<<11)

/* Bit definitions and macros for TCD14_SOFF */
#define MCF_EDMA_TCD14_SOFF_SOFF(x)     (x)

/* Bit definitions and macros for TCD14_NBYTES */
#define MCF_EDMA_TCD14_NBYTES_NBYTES(x)     (x)

/* Bit definitions and macros for TCD14_SLAST */
#define MCF_EDMA_TCD14_SLAST_SLAST(x)   (x)

/* Bit definitions and macros for TCD14_DADDR */
#define MCF_EDMA_TCD14_DADDR_DADDR(x)   (x)

/* Bit definitions and macros for TCD14_CITER */
#define MCF_EDMA_TCD14_CITER_CITER(x)   (((x)&0x7FFF))
#define MCF_EDMA_TCD14_CITER_E_LINK     (0x8000)

/* Bit definitions and macros for TCD14_CITER_ELINK */
#define MCF_EDMA_TCD14_CITER_ELINK_CITER(x)     (((x)&0x01FF))
#define MCF_EDMA_TCD14_CITER_ELINK_LINKCH(x)    (((x)&0x003F)<<9)
#define MCF_EDMA_TCD14_CITER_ELINK_E_LINK       (0x8000)

/* Bit definitions and macros for TCD14_DOFF */
#define MCF_EDMA_TCD14_DOFF_DOFF(x)     (x)

/* Bit definitions and macros for TCD14_DLAST_SGA */
#define MCF_EDMA_TCD14_DLAST_SGA_DLAST_SGA(x)   (x)

/* Bit definitions and macros for TCD14_BITER_ELINK */
#define MCF_EDMA_TCD14_BITER_ELINK_BITER(x)     (((x)&0x01FF))
#define MCF_EDMA_TCD14_BITER_ELINK_LINKCH(x)    (((x)&0x003F)<<9)
#define MCF_EDMA_TCD14_BITER_ELINK_E_LINK       (0x8000)

/* Bit definitions and macros for TCD14_BITER */
#define MCF_EDMA_TCD14_BITER_BITER(x)   (((x)&0x7FFF))
#define MCF_EDMA_TCD14_BITER_E_LINK     (0x8000)

/* Bit definitions and macros for TCD14_CSR */
#define MCF_EDMA_TCD14_CSR_START        (0x0001)
#define MCF_EDMA_TCD14_CSR_INT_MAJOR    (0x0002)
#define MCF_EDMA_TCD14_CSR_INT_HALF     (0x0004)
#define MCF_EDMA_TCD14_CSR_D_REQ        (0x0008)
#define MCF_EDMA_TCD14_CSR_E_SG         (0x0010)
#define MCF_EDMA_TCD14_CSR_E_LINK       (0x0020)
#define MCF_EDMA_TCD14_CSR_ACTIVE       (0x0040)
#define MCF_EDMA_TCD14_CSR_DONE         (0x0080)
#define MCF_EDMA_TCD14_CSR_LINKCH(x)    (((x)&0x003F)<<8)
#define MCF_EDMA_TCD14_CSR_BWC(x)       (((x)&0x0003)<<14)

/* Bit definitions and macros for TCD15_SADDR */
#define MCF_EDMA_TCD15_SADDR_SADDR(x)   (x)

/* Bit definitions and macros for TCD15_ATTR */
#define MCF_EDMA_TCD15_ATTR_DSIZE(x)    (((x)&0x0007))
#define MCF_EDMA_TCD15_ATTR_DMOD(x)     (((x)&0x001F)<<3)
#define MCF_EDMA_TCD15_ATTR_SSIZE(x)    (((x)&0x0007)<<8)
#define MCF_EDMA_TCD15_ATTR_SMOD(x)     (((x)&0x001F)<<11)

/* Bit definitions and macros for TCD15_SOFF */
#define MCF_EDMA_TCD15_SOFF_SOFF(x)     (x)

/* Bit definitions and macros for TCD15_NBYTES */
#define MCF_EDMA_TCD15_NBYTES_NBYTES(x)     (x)

/* Bit definitions and macros for TCD15_SLAST */
#define MCF_EDMA_TCD15_SLAST_SLAST(x)   (x)

/* Bit definitions and macros for TCD15_DADDR */
#define MCF_EDMA_TCD15_DADDR_DADDR(x)   (x)

/* Bit definitions and macros for TCD15_CITER_ELINK */
#define MCF_EDMA_TCD15_CITER_ELINK_CITER(x)     (((x)&0x01FF))
#define MCF_EDMA_TCD15_CITER_ELINK_LINKCH(x)    (((x)&0x003F)<<9)
#define MCF_EDMA_TCD15_CITER_ELINK_E_LINK       (0x8000)

/* Bit definitions and macros for TCD15_CITER */
#define MCF_EDMA_TCD15_CITER_CITER(x)   (((x)&0x7FFF))
#define MCF_EDMA_TCD15_CITER_E_LINK     (0x8000)

/* Bit definitions and macros for TCD15_DOFF */
#define MCF_EDMA_TCD15_DOFF_DOFF(x)     (x)

/* Bit definitions and macros for TCD15_DLAST_SGA */
#define MCF_EDMA_TCD15_DLAST_SGA_DLAST_SGA(x)   (x)

/* Bit definitions and macros for TCD15_BITER */
#define MCF_EDMA_TCD15_BITER_BITER(x)   (((x)&0x7FFF))
#define MCF_EDMA_TCD15_BITER_E_LINK     (0x8000)

/* Bit definitions and macros for TCD15_BITER_ELINK */
#define MCF_EDMA_TCD15_BITER_ELINK_BITER(x)     (((x)&0x01FF))
#define MCF_EDMA_TCD15_BITER_ELINK_LINKCH(x)    (((x)&0x003F)<<9)
#define MCF_EDMA_TCD15_BITER_ELINK_E_LINK       (0x8000)

/* Bit definitions and macros for TCD15_CSR */
#define MCF_EDMA_TCD15_CSR_START        (0x0001)
#define MCF_EDMA_TCD15_CSR_INT_MAJOR    (0x0002)
#define MCF_EDMA_TCD15_CSR_INT_HALF     (0x0004)
#define MCF_EDMA_TCD15_CSR_D_REQ        (0x0008)
#define MCF_EDMA_TCD15_CSR_E_SG         (0x0010)
#define MCF_EDMA_TCD15_CSR_E_LINK       (0x0020)
#define MCF_EDMA_TCD15_CSR_ACTIVE       (0x0040)
#define MCF_EDMA_TCD15_CSR_DONE         (0x0080)
#define MCF_EDMA_TCD15_CSR_LINKCH(x)    (((x)&0x003F)<<8)
#define MCF_EDMA_TCD15_CSR_BWC(x)       (((x)&0x0003)<<14)

#endif /* __MCF5445X_EDMA_H__ */

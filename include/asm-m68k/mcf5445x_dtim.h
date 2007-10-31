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

#ifndef __MCF5445X_DTIM_H__
#define __MCF5445X_DTIM_H__

/*********************************************************************
*
* DMA Timers (DTIM)
*
*********************************************************************/
/* Register read/write macros */
#define MCF_DTIM0_DTMR	0xFC070000	/* DMA Timer Mode Register */
#define MCF_DTIM0_DTXMR	0xFC070002	/* DMA Timer Extended Mode Register */
#define MCF_DTIM0_DTER	0xFC070003	/* DMA Timer Event Register */
#define MCF_DTIM0_DTRR	0xFC070004	/* DMA Timer Reference Register */
#define MCF_DTIM0_DTCR	0xFC070008	/* DMA Timer Capture Register */
#define MCF_DTIM0_DTCN	0xFC07000C	/* DMA Timer Counter Register */
#define MCF_DTIM1_DTMR	0xFC074000	/* DMA Timer Mode Register */
#define MCF_DTIM1_DTXMR	0xFC074002	/* DMA Timer Extended Mode Register */
#define MCF_DTIM1_DTER	0xFC074003	/* DMA Timer Event Register */
#define MCF_DTIM1_DTRR	0xFC074004	/* DMA Timer Reference Register */
#define MCF_DTIM1_DTCR	0xFC074008	/* DMA Timer Capture Register */
#define MCF_DTIM1_DTCN	0xFC07400C	/* DMA Timer Counter Register */
#define MCF_DTIM2_DTMR	0xFC078000	/* DMA Timer Mode Register */
#define MCF_DTIM2_DTXMR	0xFC078002	/* DMA Timer Extended Mode Register */
#define MCF_DTIM2_DTER	0xFC078003	/* DMA Timer Event Register */
#define MCF_DTIM2_DTRR	0xFC078004	/* DMA Timer Reference Register */
#define MCF_DTIM2_DTCR	0xFC078008	/* DMA Timer Capture Register */
#define MCF_DTIM2_DTCN	0xFC07800C	/* DMA Timer Counter Register */
#define MCF_DTIM3_DTMR	0xFC07C000	/* DMA Timer Mode Register */
#define MCF_DTIM3_DTXMR	0xFC07C002	/* DMA Timer Extended Mode Register */
#define MCF_DTIM3_DTER	0xFC07C003	/* DMA Timer Event Register */
#define MCF_DTIM3_DTRR	0xFC07C004	/* DMA Timer Reference Register */
#define MCF_DTIM3_DTCR	0xFC07C008	/* DMA Timer Capture Register */
#define MCF_DTIM3_DTCN	0xFC07C00C	/* DMA Timer Counter Register */

/* Parameterized register read/write macros for multiple modules */
#define MCF_DTIM_DTMR(x)	0xFC070000+((x)*0x4000)  /* DMA Timer Mode Register */
#define MCF_DTIM_DTXMR(x)	0xFC070002+((x)*0x4000)  /* DMA Timer Extended Mode Register */
#define MCF_DTIM_DTER(x)	0xFC070003+((x)*0x4000)  /* DMA Timer Event Register */
#define MCF_DTIM_DTRR(x)	0xFC070004+((x)*0x4000)  /* DMA Timer Reference Register */
#define MCF_DTIM_DTCR(x)	0xFC070008+((x)*0x4000)  /* DMA Timer Capture Register */
#define MCF_DTIM_DTCN(x)	0xFC07000C+((x)*0x4000)  /* DMA Timer Counter Register */

/* Bit definitions and macros for DTMR */
#define MCF_DTIM_DTMR_RST	(0x0001)		/* Reset */
#define MCF_DTIM_DTMR_CLK(x)	(((x)&0x0003)<<1)	/* Input clock source */
#define MCF_DTIM_DTMR_FRR	(0x0008)		/* Free run/restart */
#define MCF_DTIM_DTMR_ORRI	(0x0010)		/* Output reference request/interrupt enable */
#define MCF_DTIM_DTMR_OM	(0x0020)		/* Output Mode */
#define MCF_DTIM_DTMR_CE(x)	(((x)&0x0003)<<6)	/* Capture Edge */
#define MCF_DTIM_DTMR_PS(x)	(((x)&0x00FF)<<8)	/* Prescaler value */
#define MCF_DTIM_DTMR_RST_EN	(0x0001)
#define MCF_DTIM_DTMR_RST_RST	(0x0000)
#define MCF_DTIM_DTMR_CE_ANY	(0x00C0)
#define MCF_DTIM_DTMR_CE_FALL	(0x0080)
#define MCF_DTIM_DTMR_CE_RISE	(0x0040)
#define MCF_DTIM_DTMR_CE_NONE	(0x0000)
#define MCF_DTIM_DTMR_CLK_DTIN	(0x0006)
#define MCF_DTIM_DTMR_CLK_DIV16	(0x0004)
#define MCF_DTIM_DTMR_CLK_DIV1	(0x0002)
#define MCF_DTIM_DTMR_CLK_STOP	(0x0000)

/* Bit definitions and macros for DTXMR */
#define MCF_DTIM_DTXMR_MODE16	(0x01)	/* Increment Mode */
#define MCF_DTIM_DTXMR_DMAEN	(0x80)	/* DMA request */

/* Bit definitions and macros for DTER */
#define MCF_DTIM_DTER_CAP	(0x01)	/* Capture event */
#define MCF_DTIM_DTER_REF	(0x02)	/* Output reference event */

/* Interrupts used for system timers */
#define MCFSIM_ICR_TIMER1       (0xFC048040+32)
#define MCFSIM_ICR_TIMER2       (0xFC048040+33)

/********************************************************************/

#endif /* __MCF5445X_DTIM_H__ */

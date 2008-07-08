/*
 * flexcan.h
 *
 * DESCRIPTION:
 *  Definitions of consts/structs to drive the Freescale FLEXCAN.
 *
 */

#ifndef __FLEXCAN_H__
#define __FLEXCAN_H__

#include <linux/autoconf.h>
#include <linux/types.h>

/* FLEXCAN module configuration register (CANMCR) bits */
#define CANMCR_MDIS				0x80000000
#define CANMCR_FRZ				0x40000000
#define CANMCR_HALT				0x10000000
#define CANMCR_SOFTRST				0x02000000
#define CANMCR_FRZACK				0x01000000
#define CANMCR_SUPV				0x00800000
#define CANMCR_MAXMB(x)				((x)&0x0f)

/* FLEXCAN control register (CANCTRL) bits */
#define CANCTRL_PRESDIV(x)			(((x)&0xff)<<24)
#define CANCTRL_RJW(x)				(((x)&0x03)<<22)
#define CANCTRL_PSEG1(x)			(((x)&0x07)<<19)
#define CANCTRL_PSEG2(x)			(((x)&0x07)<<16)
#define CANCTRL_BOFFMSK				0x00008000
#define CANCTRL_ERRMSK				0x00004000
#define CANCTRL_LPB				0x00001000
#define CANCTRL_SAMP(x)				(((x)&0x1)<<7)
#define CANCTRL_BOFFREC				0x00000040
#define CANCTRL_TSYNC				0x00000020
#define CANCTRL_LBUF				0x00000010
#define CANCTRL_LOM				0x00000008
#define CANCTRL_PROPSEG(x)			((x)&0x07)
#define CANCTRL_BITTIME				0x00c0d078

/* FLEXCAN error counter register (ERRCNT) bits */
#define ERRCNT_REXECTR(x)			(((x)&0xff)<<8)
#define ERRCNT_TXECTR(x)			((x)&0xff)

/* FLEXCAN error and status register (ERRSTAT) bits */
#define ERRSTAT_BITERR(x)			(((x)&0x03)<<14)
#define ERRSTAT_ACKERR				0x00002000
#define ERRSTAT_CRCERR				0x00001000
#define ERRSTAT_FRMERR				0x00000800
#define ERRSTAT_STFERR				0x00000400
#define ERRSTAT_TXWRN				0x00000200
#define ERRSTAT_RXWRN				0x00000100
#define ERRSTAT_IDLE 				0x00000080
#define ERRSTAT_TXRX				0x00000040
#define ERRSTAT_FLTCONF(x)			(((x)&0x03)<<4)
#define ERRSTAT_BOFFINT				0x00000004
#define ERRSTAT_ERRINT          		0x00000002

/* FLEXCAN interrupt mask register (IMASK) bits */
#define IMASK_BUF15M				0x8000
#define IMASK_BUF14M				0x4000
#define IMASK_BUF13M				0x2000
#define IMASK_BUF12M				0x1000
#define IMASK_BUF11M				0x0800
#define IMASK_BUF10M				0x0400
#define IMASK_BUF9M				0x0200
#define IMASK_BUF8M				0x0100
#define IMASK_BUF7M				0x0080
#define IMASK_BUF6M				0x0040
#define IMASK_BUF5M				0x0020
#define IMASK_BUF4M				0x0010
#define IMASK_BUF3M				0x0008
#define IMASK_BUF2M				0x0004
#define IMASK_BUF1M				0x0002
#define IMASK_BUF0M				0x0001
#define IMASK_BUFnM(x)				(0x1<<(x))
#define IMASK_BUFF_ENABLE_ALL			0xffff
#define IMASK_BUFF_DISABLE_ALL 			0x0000

/* FLEXCAN interrupt flag register (IFLAG) bits */
#define IFLAG_BUF15M				0x8000
#define IFLAG_BUF14M				0x4000
#define IFLAG_BUF13M				0x2000
#define IFLAG_BUF12M				0x1000
#define IFLAG_BUF11M				0x0800
#define IFLAG_BUF10M				0x0400
#define IFLAG_BUF9M				0x0200
#define IFLAG_BUF8M				0x0100
#define IFLAG_BUF7M				0x0080
#define IFLAG_BUF6M				0x0040
#define IFLAG_BUF5M				0x0020
#define IFLAG_BUF4M				0x0010
#define IFLAG_BUF3M				0x0008
#define IFLAG_BUF2M				0x0004
#define IFLAG_BUF1M				0x0002
#define IFLAG_BUF0M				0x0001
#define IFLAG_BUFnM(x)				(0x1<<(x))
#define IFLAG_BUFF_SET_ALL			0xffff
#define IFLAG_BUFF_DISABLE_ALL 			0x0000

/* FLEXCAN message buffers */
#define MB_CNT_CODE(x)				(((x)&0x0f)<<24)
#define MB_CNT_SRR				0x00400000
#define MB_CNT_IDE				0x00200000
#define MB_CNT_RTR				0x00100000
#define MB_CNT_LENGTH(x)			(((x)&0x0f)<<16)
#define MB_CNT_TIMESTAMP(x)			((x)&0xffff)

#define MB_ID_STD				((0x7ff)<<18)
#define MB_ID_EXT				0x1fffffff
#define MB_CODE_MASK				0xf0ffffff

/* Structure of the message buffer */
struct flexcan_mb {
	u32	can_dlc;
	u32	can_id;
	u8	data[8];
};

/* Structure of the hardware registers */
struct flexcan_regs {
	u32	canmcr;
	u32	canctrl;
	u32	timer;
	u32	reserved1;
	u32 	rxgmask;
	u32 	rx14mask;
	u32 	rx15mask;
	u32	errcnt;
	u32 	errstat;
	u32	reserved2;
	u32	imask;
	u32	reserved3;
	u32 	iflag;
	u32	reserved4[19];
	struct	flexcan_mb cantxfg[16];
};

struct flexcan_platform_data {
	u8 clock_src;		/* FLEXCAN clock source CRIN or SYSCLK */
	u32 clock_frq;		/* can ref. clock, in Hz */
};

struct net_device *alloc_flexcandev(void);

extern int register_flexcandev(struct net_device *dev, int clock_src);
extern void unregister_flexcandev(struct net_device *dev);

#endif				/* __FLEXCAN_H__ */

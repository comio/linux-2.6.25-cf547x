/*
 * Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * Freescale USB device/endpoint management registers
 */
#ifndef __FSL_USB2_H
#define __FSL_USB2_H


 /* USB DR device mode registers (Little Endian) */
struct fsl_usb_device_regs {
	/* Capability register */
	u32 id;
	u32 res1[63];
	u16 caplength;		/* Capability Register Length */
	u16 hciversion;		/* Host Controller Interface Version */
	u32 hcsparams;		/* Host Controller Structual Parameters */
	u32 hccparams;		/* Host Controller Capability Parameters */
	u32 res2[5];
	u32 dciversion;		/* Device Controller Interface Version */
	u32 dccparams;		/* Device Controller Capability Parameters */
	u32 res3[6];
	/* Operation register */
	u32 usbcmd;		/* USB Command Register */
	u32 usbsts;		/* USB Status Register */
	u32 usbintr;		/* USB Interrupt Enable Register */
	u32 frindex;		/* Frame Index Register */
	u32 res4;
	u32 deviceaddr;		/* Device Address */
	u32 endpointlistaddr;	/* Endpoint List Address Register */
	u32 res5;
	u32 burstsize;		/* Master Interface Data Burst Size Register */
	u32 txttfilltuning;	/* Transmit FIFO Tuning Controls Register */
	u32 res6[2];
	u32 ulpiview;
	u32 res7[3];
	u32 configflag;		/* Configure Flag Register */
	u32 portsc1;		/* Port 1 Status and Control Register */
	u32 res8[7];
	u32 otgsc;		/* On-The-Go Status and Control */
	u32 usbmode;		/* USB Mode Register */
	u32 endptsetupstat;	/* Endpoint Setup Status Register */
	u32 endpointprime;	/* Endpoint Initialization Register */
	u32 endptflush;		/* Endpoint Flush Register */
	u32 endptstatus;	/* Endpoint Status Register */
	u32 endptcomplete;	/* Endpoint Complete Register */
#if 0
	u32 endptctrl[USB_MAX_ENDPOINTS]; /* Endpoint Control Registers */
#else
	// DDD see DCCPARAMS:DEN for the real number of device endpoints
	// DDD 16 is the max
	u32 endptctrl[16];	/* Endpoint Control Registers */
#endif
};

 /* USB DR host mode registers (Little Endian) */
struct fsl_usb_host_regs {
	/* Capability register */
	u32 id;
	u32 res1[63];
	u16 caplength;		/* Capability Register Length */
	u16 hciversion;		/* Host Controller Interface Version */
	u32 hcsparams;		/* Host Controller Structual Parameters */
	u32 hccparams;		/* Host Controller Capability Parameters */
	u32 res2[5];
	u32 dciversion;		/* Device Controller Interface Version */
	u32 dccparams;		/* Device Controller Capability Parameters */
	u32 res3[6];
	/* Operation register */
	u32 usbcmd;		/* USB Command Register */
	u32 usbsts;		/* USB Status Register */
	u32 usbintr;		/* USB Interrupt Enable Register */
	u32 frindex;		/* Frame Index Register */
	u32 res4;
	u32 periodiclistbase;	/* Periodic Frame List Base Address Register */
	u32 asynclistaddr;	/* Current Asynchronous List Address Register */
	u32 res5;
	u32 burstsize;		/* Master Interface Data Burst Size Register */
	u32 txttfilltuning;	/* Transmit FIFO Tuning Controls Register */
	u32 res6[2];
	u32 ulpiview;
	u32 res7[3];
	u32 configflag;		/* Configure Flag Register */
	u32 portsc1;		/* Port 1 Status and Control Register */
	u32 res8[7];
	u32 otgsc;		/* On-The-Go Status and Control */
	u32 usbmode;		/* USB Mode Register */
	u32 endptsetupstat;	/* Endpoint Setup Status Register */
	u32 endpointprime;	/* Endpoint Initialization Register */
	u32 endptflush;		/* Endpoint Flush Register */
	u32 endptstatus;	/* Endpoint Status Register */
	u32 endptcomplete;	/* Endpoint Complete Register */
#if 0
	u32 endptctrl[USB_MAX_ENDPOINTS]; /* Endpoint Control Registers */
#else
	// DDD: do these exits in the host register set??
	// DDD see DCCPARAMS:DEN for the real number of device endpoints
	// DDD 16 is the max
	u32 endptctrl[16];	/* Endpoint Control Registers */
#endif
};

 /* non-EHCI USB system interface registers (Big Endian) */
struct usb_sys_interface {
	u32 snoop1;
	u32 snoop2;
	u32 age_cnt_thresh;	/* Age Count Threshold Register */
	u32 pri_ctrl;		/* Priority Control Register */
	u32 si_ctrl;		/* System Interface Control Register */
	u32 res[59];
	u32 control;		/* General Purpose Control Register */
};

/* ep0 transfer state */
#define WAIT_FOR_SETUP		0
#define DATA_STATE_XMIT		1
#define DATA_STATE_NEED_ZLP	2
#define WAIT_FOR_OUT_STATUS	3
#define DATA_STATE_RECV		4

/* Frame Index Register Bit Masks */
#define	USB_FRINDEX_MASKS			0x3fff



/* USBCMD Register Bit Masks */
#define USB_CMD_RUN_STOP			0x00000001
#define USB_CMD_CTRL_RESET			0x00000002
#define USB_CMD_PERIODIC_SCHEDULE_EN		0x00000010
#define USB_CMD_ASYNC_SCHEDULE_EN		0x00000020
#define USB_CMD_INT_AA_DOORBELL			0x00000040
#define USB_CMD_ASP				0x00000300
#define USB_CMD_ASYNC_SCH_PARK_EN		0x00000800
#define USB_CMD_SUTW				0x00002000
#define USB_CMD_ATDTW				0x00004000
#define USB_CMD_ITC				0x00FF0000

/* bit 15,3,2 are frame list size */
#define USB_CMD_FRAME_SIZE_1024			0x00000000
#define USB_CMD_FRAME_SIZE_512			0x00000004
#define USB_CMD_FRAME_SIZE_256			0x00000008
#define USB_CMD_FRAME_SIZE_128			0x0000000C
#define USB_CMD_FRAME_SIZE_64			0x00008000
#define USB_CMD_FRAME_SIZE_32			0x00008004
#define USB_CMD_FRAME_SIZE_16			0x00008008
#define USB_CMD_FRAME_SIZE_8			0x0000800C

/* bit 9-8 are async schedule park mode count */
#define USB_CMD_ASP_00				0x00000000
#define USB_CMD_ASP_01				0x00000100
#define USB_CMD_ASP_10				0x00000200
#define USB_CMD_ASP_11				0x00000300
#define USB_CMD_ASP_BIT_POS			8

/* bit 23-16 are interrupt threshold control */
#define USB_CMD_ITC_NO_THRESHOLD		0x00000000
#define USB_CMD_ITC_1_MICRO_FRM			0x00010000
#define USB_CMD_ITC_2_MICRO_FRM			0x00020000
#define USB_CMD_ITC_4_MICRO_FRM			0x00040000
#define USB_CMD_ITC_8_MICRO_FRM			0x00080000
#define USB_CMD_ITC_16_MICRO_FRM		0x00100000
#define USB_CMD_ITC_32_MICRO_FRM		0x00200000
#define USB_CMD_ITC_64_MICRO_FRM		0x00400000
#define USB_CMD_ITC_BIT_POS			16




/* USB STS Register Bit Masks */
#define USB_STS_INT				0x00000001
#define USB_STS_ERR				0x00000002
#define USB_STS_PORT_CHANGE			0x00000004
#define USB_STS_FRM_LST_ROLL			0x00000008
#define USB_STS_SYS_ERR				0x00000010
#define USB_STS_IAA				0x00000020
#define USB_STS_RESET				0x00000040
#define USB_STS_SOF				0x00000080
#define USB_STS_SUSPEND				0x00000100
#define USB_STS_HC_HALTED			0x00001000
#define USB_STS_RCL				0x00002000
#define USB_STS_PERIODIC_SCHEDULE		0x00004000
#define USB_STS_ASYNC_SCHEDULE			0x00008000

/* USB INTR Register Bit Masks */
#define USB_INTR_INT_EN				0x00000001
#define USB_INTR_ERR_INT_EN			0x00000002
#define USB_INTR_PTC_DETECT_EN			0x00000004
#define USB_INTR_FRM_LST_ROLL_EN		0x00000008
#define USB_INTR_SYS_ERR_EN			0x00000010
#define USB_INTR_ASYN_ADV_EN			0x00000020
#define USB_INTR_RESET_EN			0x00000040
#define USB_INTR_SOF_EN				0x00000080
#define USB_INTR_DEVICE_SUSPEND			0x00000100

/* Device Address bit masks */
#define USB_DEVICE_ADDRESS_MASK			0xFE000000
#define USB_DEVICE_ADDRESS_BIT_POS		25

/* endpoint list address bit masks */
#define USB_EP_LIST_ADDRESS_MASK		0xfffff800


/* x_PORTSCx */
/* bit 31-30 are port transceiver select */
#define PORTSCX_PTS_MASK		(3 << 30)	/* parallel xcvr select mask */
#define PORTSCX_PHY_TYPE_SEL		PORTSCX_PTS_MASK
#define PORTSCX_PTS_UTMI		(0 << 30)	/* UTMI/UTMI+ */
#define PORTSCX_PTS_PHILIPS		(1 << 30)	/* Philips classic */
#define PORTSCX_PTS_ULPI		(2 << 30)	/* ULPI */
#define PORTSCX_PTS_SERIAL		(3 << 30)	/* serial */
#define PORTSCX_PTS_FSLS		PORTSCX_PTS_SERIAL
#define PORTSCX_PTS_ONCHIP		PORTSCX_PTS_FSLS

#define PORTSCX_STS			(1 << 29)	/* serial xcvr select */

/* bit 28 is parallel transceiver width for UTMI interface */
#define PORTSCX_PTW_8BIT		(0 << 28)	/* 8 bit parallel xcvr */
#define PORTSCX_PTW_16BIT		(1 << 28)	/* 16 bi parallel xcvr */

/* bit 27-26 are port speed */
#define PORTSCX_PORT_SPEED_FULL		(0 << 26)
#define PORTSCX_PORT_SPEED_LOW		(1 << 26)
#define PORTSCX_PORT_SPEED_HIGH		(2 << 26)
#define PORTSCX_PORT_SPEED_UNDEF	(3 << 26)
#define PORTSCX_PORT_SPEED_MASK		(3 << 26)

#define PORTSCX_PHY_LOW_POWER_SPD	(1 << 23)	/* phy low pwr suspend/clk disable */

/* bit 19-16 are port test control */
#define PORTSCX_PTC_DISABLE		(0 << 16)
#define PORTSCX_PTC_JSTATE		(1 << 16)
#define PORTSCX_PTC_KSTATE		(2 << 16)
#define PORTSCX_PTC_SEQNAK		(3 << 16)	/* SE0 (host) / NAK (device) */
#define PORTSCX_PTC_PACKET		(4 << 16)
#define PORTSCX_PTC_FORCE_EN_HS		(5 << 16)
#define PORTSCX_PTC_FORCE_EN_FS		(6 << 16)
#define PORTSCX_PTC_FORCE_EN_LS		(7 << 16)


/* bit 15-14 are port indicator control */
#define PORTSCX_PIC_OFF			(0 << 14)
#define PORTSCX_PIC_AMBER		(1 << 14)
#define PORTSCX_PIC_GREEN		(2 << 14)
#define PORTSCX_PIC_UNDEF		(3 << 14)

#define PORTSCX_PORT_POWER		(1 << 12)	/* port power */

/* bit 11-10 are line status */
#define PORTSCX_LS_MASK			(3 << 10)	/* Line State mask */
#define PORTSCX_LS_SE0			(0 << 10)	/* SE0     */
#define PORTSCX_LS_K_STATE		(1 << 10)	/* K-state */
#define PORTSCX_LS_J_STATE		(2 << 10)	/* J-state */

#define PORTSCX_PORT_RESET		(1 <<  8)	/* Port reset */
#define PORTSCX_PORT_SUSPEND		(1 <<  7)	/* Suspend */
#define PORTSCX_PORT_FORCE_RESUME	(1 <<  6)	/* Force port resume */
#define PORTSCX_OVER_CURRENT_CHG	(1 <<  5)	/* over current change */
#define PORTSCX_OVER_CURRENT_ACT	(1 <<  4)	/* over currrent active */
#define PORTSCX_PORT_EN_DIS_CHANGE	(1 <<  3)	/* port {en,dis}able change */
#define PORTSCX_PORT_ENABLE		(1 <<  2)	/* port enabled */
#define PORTSCX_CONNECT_STATUS_CHANGE	(1 <<  1)	/* connect status change */
#define PORTSCX_CURRENT_CONNECT_STATUS	(1 <<  0)	/* current connect status */

#define PORTSCX_W1C_BITS                     \
       ( PORTSCX_CONNECT_STATUS_CHANGE |     \
         PORTSCX_PORT_EN_DIS_CHANGE    |     \
         PORTSCX_OVER_CURRENT_CHG )



/* UOG_OTGSC Register Bits */
/* control bits: */
#define  OTGSC_CTRL_VBUS_DISCHARGE	(1 <<  0)
#define  OTGSC_CTRL_VBUS_CHARGE		(1 <<  1)
#define  OTGSC_CTRL_OTG_TERM		(1 <<  3)	/* controls DM pulldown */
#define  OTGSC_CTRL_DATA_PULSING	(1 <<  4)
#define  OTGSC_CTRL_USB_ID_PU		(1 <<  5)	/* enable ID pullup */
/* current status: (R/O) */
#define  OTGSC_STS_USB_ID		(1 <<  8)	/* 0=A-device  1=B-device */
#define  OTGSC_STS_A_VBUS_VALID		(1 <<  9)
#define  OTGSC_STS_A_SESSION_VALID	(1 << 10)
#define  OTGSC_STS_B_SESSION_VALID	(1 << 11)
#define  OTGSC_STS_B_SESSION_END	(1 << 12)
#define  OTGSC_STS_1ms_TIMER		(1 << 13)
#define  OTGSC_STS_DATA_PULSE		(1 << 14)
/* interrupt status: (write to clear) */
#define  OTGSC_INTSTS_MASK		(0x7f << 16)
#define  OTGSC_INTSTS_USB_ID		(1 << 16)
#define  OTGSC_INTSTS_A_VBUS_VALID	(1 << 17)
#define  OTGSC_INTSTS_A_SESSION_VALID	(1 << 18)
#define  OTGSC_INTSTS_B_SESSION_VALID	(1 << 19)
#define  OTGSC_INTSTS_B_SESSION_END	(1 << 20)
#define  OTGSC_INTSTS_1MS_TIMER		(1 << 21)
#define  OTGSC_INTSTS_DATA_PULSE	(1 << 22)
/* interrupt enables: */
#define  OTGSC_IE_MASK			(0x7f << 24)
#define  OTGSC_IE_USB_ID		(1 << 24)
#define  OTGSC_IE_A_VBUS_VALID		(1 << 25)
#define  OTGSC_IE_A_SESSION_VALID	(1 << 26)
#define  OTGSC_IE_B_SESSION_VALID	(1 << 27)
#define  OTGSC_IE_B_SESSION_END		(1 << 28)
#define  OTGSC_IE_1ms_TIMER		(1 << 29)
#define  OTGSC_IE_DATA_PULSE		(1 << 30)

#if 1	/* DDD FIXME these here for compatibility between my names and Leo's */
/* OTG interrupt enable bit masks */
#define  OTGSC_INTERRUPT_ENABLE_BITS_MASK	OTGSC_IE_MASK

/* OTG interrupt status bit masks */
#define  OTGSC_INTERRUPT_STATUS_BITS_MASK	OTGSC_INTSTS_MASK
#endif // 1



/* x_USBMODE */
#undef USBMODE_SDIS	/* defined as bit 3 in drivers/usb/host/ehci.h */
#define USBMODE_SDIS		(1 << 4)	/* stream disable mode */
#define USBMODE_SLOM		(1 << 3)	/* setup lockout mode */
#define USBMODE_ES		(1 << 2)	/* (big) endian select */
#define USBMODE_CM_MASK		(3 << 0)	/* controller mode mask */
#define USBMODE_CM_HOST		(3 << 0)	/* host */
#define USBMODE_CM_DEVICE	(2 << 0)	/* device */
// DDD #define USBMODE_CM_IDLE		(0 << 0)	/* idle */

/* DDD for compatibility for now */
#define USB_MODE_CTRL_MODE_IDLE		USBMODE_CM_IDLE
#define USB_MODE_CTRL_MODE_DEVICE	USBMODE_CM_DEVICE
#define USB_MODE_CTRL_MODE_HOST		USBMODE_CM_HOST
#define USB_MODE_SETUP_LOCK_OFF		USBMODE_SLOM
#define USB_MODE_STREAM_DISABLE		USBMODE_SDIS


/* ULPIVIEW register bits */
#define ULPIVW_WU		(1 << 31)	/* Wakeup */
#define ULPIVW_RUN		(1 << 30)	/* read/write run */
#define ULPIVW_WRITE		(1 << 29)	/* 0=read  1=write */
#define ULPIVW_SS		(1 << 27)	/* SyncState */
#define ULPIVW_PORT_MASK	0x07		/* Port field */
#define ULPIVW_PORT_SHIFT	24
#define ULPIVW_ADDR_MASK	0xFF		/* data address field */
#define ULPIVW_ADDR_SHIFT	16
#define ULPIVW_RDATA_MASK	0xFF		/* read data field */
#define ULPIVW_RDATA_SHIFT	8
#define ULPIVW_WDATA_MASK	0xFF		/* write data field */
#define ULPIVW_WDATA_SHIFT	0


/* Endpoint Flush Register */
#define EPFLUSH_TX_OFFSET			0x00010000
#define EPFLUSH_RX_OFFSET			0x00000000

/* Endpoint Setup Status bit masks */
#define EP_SETUP_STATUS_MASK			0x0000003F
#define EP_SETUP_STATUS_EP0			0x00000001

/* ENDPOINTCTRLx  Register Bit Masks */
#define EPCTRL_TX_ENABLE			0x00800000
#define EPCTRL_TX_DATA_TOGGLE_RST		0x00400000	/* Not EP0 */
#define EPCTRL_TX_DATA_TOGGLE_INH		0x00200000	/* Not EP0 */
#define EPCTRL_TX_TYPE				0x000C0000
#define EPCTRL_TX_DATA_SOURCE			0x00020000	/* Not EP0 */
#define EPCTRL_TX_EP_STALL			0x00010000
#define EPCTRL_RX_ENABLE			0x00000080
#define EPCTRL_RX_DATA_TOGGLE_RST		0x00000040	/* Not EP0 */
#define EPCTRL_RX_DATA_TOGGLE_INH		0x00000020	/* Not EP0 */
#define EPCTRL_RX_TYPE				0x0000000C
#define EPCTRL_RX_DATA_SINK			0x00000002	/* Not EP0 */
#define EPCTRL_RX_EP_STALL			0x00000001

/* bit 19-18 and 3-2 are endpoint type */
#define EPCTRL_EP_TYPE_CONTROL			0
#define EPCTRL_EP_TYPE_ISO			1
#define EPCTRL_EP_TYPE_BULK			2
#define EPCTRL_EP_TYPE_INTERRUPT		3
#define EPCTRL_TX_EP_TYPE_SHIFT			18
#define EPCTRL_RX_EP_TYPE_SHIFT			2

/* SNOOPn Register Bit Masks */
#define SNOOP_ADDRESS_MASK			0xFFFFF000
#define SNOOP_SIZE_ZERO				0x00	/* snooping disable */
#define SNOOP_SIZE_4KB				0x0B	/* 4KB snoop size */
#define SNOOP_SIZE_8KB				0x0C
#define SNOOP_SIZE_16KB				0x0D
#define SNOOP_SIZE_32KB				0x0E
#define SNOOP_SIZE_64KB				0x0F
#define SNOOP_SIZE_128KB			0x10
#define SNOOP_SIZE_256KB			0x11
#define SNOOP_SIZE_512KB			0x12
#define SNOOP_SIZE_1MB				0x13
#define SNOOP_SIZE_2MB				0x14
#define SNOOP_SIZE_4MB				0x15
#define SNOOP_SIZE_8MB				0x16
#define SNOOP_SIZE_16MB				0x17
#define SNOOP_SIZE_32MB				0x18
#define SNOOP_SIZE_64MB				0x19
#define SNOOP_SIZE_128MB			0x1A
#define SNOOP_SIZE_256MB			0x1B
#define SNOOP_SIZE_512MB			0x1C
#define SNOOP_SIZE_1GB				0x1D
#define SNOOP_SIZE_2GB				0x1E	/* 2GB snoop size */

/* pri_ctrl Register Bit Masks */
#define PRI_CTRL_PRI_LVL1			0x0000000C
#define PRI_CTRL_PRI_LVL0			0x00000003

/* si_ctrl Register Bit Masks */
#define SI_CTRL_ERR_DISABLE			0x00000010
#define SI_CTRL_IDRC_DISABLE			0x00000008
#define SI_CTRL_RD_SAFE_EN			0x00000004
#define SI_CTRL_RD_PREFETCH_DISABLE		0x00000002
#define SI_CTRL_RD_PREFEFETCH_VAL		0x00000001


/* control Register Bit Masks */
#define USB_CTRL_IOENB				0x00000004
#define USB_CTRL_ULPI_INT0EN			0x00000001


/* Endpoint Transfer Descriptor bit Masks */
#define DTD_NEXT_TERMINATE			0x00000001
#define DTD_IOC					0x00008000
#define DTD_STATUS_ACTIVE			0x00000080
#define DTD_STATUS_HALTED			0x00000040
#define DTD_STATUS_DATA_BUFF_ERR		0x00000020
#define DTD_STATUS_TRANSACTION_ERR		0x00000008
#define DTD_RESERVED_FIELDS			0x80007300
#define DTD_ADDR_MASK				0xFFFFFFE0
#define DTD_PACKET_SIZE				0x7FFF0000
#define DTD_LENGTH_BIT_POS			16
#define DTD_ERROR_MASK (DTD_STATUS_HALTED | \
			DTD_STATUS_DATA_BUFF_ERR | \
			DTD_STATUS_TRANSACTION_ERR)
/* Alignment requirements; must be a power of two */
#define DTD_ALIGNMENT				0x20
#define QH_ALIGNMENT				2048

/* Controller dma boundary */
#define UDC_DMA_BOUNDARY			0x1000

#if defined CONFIG_PPC32
#define fsl_readl(addr)		in_le32((addr))
#define fsl_writel(addr, val32)	out_le32((addr), (val32))
#elif defined CONFIG_COLDFIRE
#define fsl_readl(addr)		in_be32((__force unsigned *)(addr))
#define fsl_writel(val32, addr)	out_be32((__force unsigned *)(addr), (val32))

#define fsl_readw(addr)		in_be16((__force unsigned *)(addr))
#define fsl_writew(val16, addr)	out_be16((__force unsigned *)(addr), (val16))
#else
#error fsl_{readl, writel} must be defined
#endif

#endif	/* __FSL_USB2_H */

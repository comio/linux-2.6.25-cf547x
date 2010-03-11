#ifndef __FEC_H__
#define __FEC_H__

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/mii.h>
#include <linux/ethtool.h>

#include <asm/MCD_dma.h>

#define   FEC_BASE_ADDR_FEC0                ((unsigned int)MCF_MBAR + 0x9000)
#define   FEC_BASE_ADDR_FEC1                ((unsigned int)MCF_MBAR + 0x9800)

//#define   FEC_INTC_IMRH_INT_MASK38          (0x00000040)
//#define   FEC_INTC_IMRH_INT_MASK39          (0x00000080)
//#define   FEC_INTC_ICR_FEC0                 (0x30)
//#define   FEC_INTC_ICR_FEC1                 (0x31)
#define   FEC_FECI2CIRQ                     (0xFFC0)
#define   FEC_GPIO_PAR_FECI2CIRQ            *(volatile unsigned short*)((unsigned int)MCF_MBAR + 0xA44)
//#define   FEC_INTC_ICRn(x)                  (*(volatile unsigned char *)(void*)((unsigned int) MCF_MBAR + 0x000740+((x)*0x001)))
//#define   FEC_INTC_IMRH                     *(volatile unsigned int*)((unsigned int)MCF_MBAR + 0x000708)

#define   FEC_ECR_DISABLE                   (0x00000000)

#define   FEC_ECR(x)                        *(volatile unsigned int*)(x + 0x024)
#define   FEC_EIR(x)                        *(volatile unsigned int*)(x + 0x004)
#define   FEC_PALR(x)                       *(volatile unsigned int*)(x + 0x0E4)
#define   FEC_PAUR(x)                       *(volatile unsigned int*)(x + 0x0E8)
#define   FEC_IALR(x)                       *(volatile unsigned int*)(x + 0x11C)
#define   FEC_IAUR(x)                       *(volatile unsigned int*)(x + 0x118)
#define   FEC_GALR(x)                       *(volatile unsigned int*)(x + 0x124)
#define   FEC_GAUR(x)                       *(volatile unsigned int*)(x + 0x120)
#define   FEC_RCR(x)                        *(volatile unsigned int*)(x + 0x084)
#define   FEC_FECRFCR(x)                    *(volatile unsigned int*)(x + 0x18C)
#define   FEC_FECRFAR(x)                    *(volatile unsigned int*)(x + 0x198)
#define   FEC_FECTFCR(x)                    *(volatile unsigned int*)(x + 0x1AC)
#define   FEC_FECTFAR(x)                    *(volatile unsigned int*)(x + 0x1B8)
#define   FEC_FECTFWR(x)                    *(volatile unsigned int*)(x + 0x144)
#define   FEC_CTCWR(x)                      *(volatile unsigned int*)(x + 0x1C8)
#define   FEC_EIMR(x)                       *(volatile unsigned int*)(x + 0x008)
#define   FEC_TCR(x)                        *(volatile unsigned int*)(x + 0x0C4)
#define   FEC_MIBC(x)                       *(volatile unsigned int*)(x + 0x064)
#define   FEC_MSCR(x)                       *(volatile unsigned int*)(x + 0x044)
#define   FEC_FECTFDR(x)                    *(volatile unsigned int*)(x + 0x1A4)
#define   FEC_FECRFDR(x)                    *(volatile unsigned int*)(x + 0x184)
#define   FEC_FECTFSR(x)                    *(volatile unsigned int*)(x + 0x1A8)
#define   FEC_FECRFSR(x)					*(volatile unsigned int*)(x + 0x188)
#define   FECSTAT_RMON_R_PACKETS(x)         *(volatile unsigned int*)(x + 0x284)
#define   FECSTAT_RMON_T_PACKETS(x)         *(volatile unsigned int*)(x + 0x204)
#define   FECSTAT_RMON_R_OCTETS(x)          *(volatile unsigned int*)(x + 0x2C4)
#define   FECSTAT_RMON_T_OCTETS(x)          *(volatile unsigned int*)(x + 0x244)
#define   FECSTAT_RMON_R_UNDERSIZE(x)       *(volatile unsigned int*)(x + 0x294)
#define   FECSTAT_RMON_R_OVERSIZE(x)        *(volatile unsigned int*)(x + 0x298)
#define   FECSTAT_RMON_R_FRAG(x)            *(volatile unsigned int*)(x + 0x29C)
#define   FECSTAT_RMON_R_JAB(x)             *(volatile unsigned int*)(x + 0x2A0)
#define   FECSTAT_RMON_R_MC_PKT(x)          *(volatile unsigned int*)(x + 0x28C)
#define   FECSTAT_RMON_T_COL(x)             *(volatile unsigned int*)(x + 0x224)
#define   FECSTAT_IEEE_R_ALIGN(x)           *(volatile unsigned int*)(x + 0x2D4)
#define   FECSTAT_IEEE_R_CRC(x)             *(volatile unsigned int*)(x + 0x2D0)
#define   FECSTAT_IEEE_R_MACERR(x)          *(volatile unsigned int*)(x + 0x2D8)
#define   FECSTAT_IEEE_T_CSERR(x)           *(volatile unsigned int*)(x + 0x268)
#define   FECSTAT_IEEE_T_MACERR(x)          *(volatile unsigned int*)(x + 0x264)
#define   FECSTAT_IEEE_T_LCOL(x)            *(volatile unsigned int*)(x + 0x25C)
#define   FECSTAT_IEEE_R_OCTETS_OK(x)       *(volatile unsigned int*)(x + 0x2E0)
#define   FECSTAT_IEEE_T_OCTETS_OK(x)       *(volatile unsigned int*)(x + 0x274)
#define   FECSTAT_IEEE_R_DROP(x)            *(volatile unsigned int*)(x + 0x2C8)
#define   FECSTAT_IEEE_T_DROP(x)            *(volatile unsigned int*)(x + 0x248)
#define   FECSTAT_IEEE_R_FRAME_OK(x)        *(volatile unsigned int*)(x + 0x2CC)
#define   FECSTAT_IEEE_T_FRAME_OK(x)        *(volatile unsigned int*)(x + 0x24C)
#define   FEC_MMFR(x)                       *(volatile unsigned int*)(x + 0x040)
#define   FEC_FECFRST(x)					*(volatile unsigned int*)(x + 0x1C4)

#define   FEC_MAX_FRM_SIZE                  (1518)
#define   FEC_MAXBUF_SIZE                   (1520)

// Register values
#define   FEC_ECR_RESET                     (0x00000001)
#define   FEC_EIR_CLEAR                     (0xFFFFFFFF)
#define   FEC_EIR_RL                        (0x00100000)
#define   FEC_EIR_HBERR                     (0x80000000)
#define   FEC_EIR_BABR						(0x40000000)	// babbling receive error
#define   FEC_EIR_BABT						(0x20000000)	// babbling transmit error
#define   FEC_EIR_TXF		              	(0x08000000)	// transmit frame interrupt
#define   FEC_EIR_MII						(0x00800000)	// MII interrupt
#define   FEC_EIR_LC						(0x00200000)	// late collision
#define   FEC_EIR_XFUN						(0x00080000)	// transmit FIFO underrun
#define   FEC_EIR_XFERR						(0x00040000)	// transmit FIFO error
#define   FEC_EIR_RFERR						(0x00020000)	// receive FIFO error
#define   FEC_RCR_MAX_FRM_SIZE              (FEC_MAX_FRM_SIZE << 16)
#define   FEC_RCR_FCE                       (0x00000020)
#define   FEC_RCR_BC_REJ                    (0x00000010)
#define   FEC_RCR_PROM                      (0x00000008)
#define   FEC_RCR_MII                       (0x00000004)
#define   FEC_RCR_MII                       (0x00000004)
#define   FEC_RCR_DRT                       (0x00000002)
#define   FEC_RCR_LOOP                      (0x00000001)

#define   FEC_FECRFCR_FAE					(0x00400000)	// frame accept error
#define   FEC_FECRFCR_RXW					(0x00200000)	// receive wait condition
#define   FEC_FECRFCR_UF					(0x00100000)	// receive FIFO underflow
#define   FEC_FECRFCR_FRM                   (0x08000000)
#define   FEC_FECRFCR_GR                    (0x7 << 24)

#define   FEC_EIMR_DISABLE					(0x00000000)

#define   FEC_FECRFAR_ALARM                 (0x300)
#define   FEC_FECTFCR_FRM                   (0x08000000)
#define   FEC_FECTFCR_GR                    (0x7 << 24)
#define   FEC_FECTFCR_FAE					(0x00400000)	// frame accept error
#define   FEC_FECTFCR_TXW					(0x00040000)	// transmit wait condition
#define   FEC_FECTFCR_UF					(0x00100000)	// transmit FIFO underflow
#define   FEC_FECTFCR_OF					(0x00080000)	// transmit FIFO overflow

#define   FEC_FECTFAR_ALARM                 (0x100)
#define   FEC_FECTFWR_XWMRK                 (0x00000000)

#define   FEC_FECTFSR_MSK                   (0xC0B00000)
#define   FEC_FECTFSR_TXW                   (0x40000000)	// transmit wait condition
#define   FEC_FECTFSR_FAE                   (0x00800000)	// frame accept error
#define   FEC_FECTFSR_UF                    (0x00200000)	// transmit FIFO underflow
#define   FEC_FECTFSR_OF                    (0x00100000)	// transmit FIFO overflow

#define   FEC_FECRFSR_MSK                   (0x80F00000)
#define   FEC_FECRFSR_FAE                   (0x00800000)	// frame accept error
#define   FEC_FECRFSR_RXW                   (0x00400000)	// receive wait condition
#define   FEC_FECRFSR_UF                    (0x00200000)	// receive FIFO underflow

#define   FEC_CTCWR_TFCW_CRC                (0x03000000)
#define   FEC_TCR_FDEN                      (0x00000004)
#define   FEC_TCR_HBC                       (0x00000002)
#define   FEC_RCR_DRT                       (0x00000002)
#define   FEC_EIMR_MASK                     (FEC_EIR_RL | FEC_EIR_HBERR)
#define   FEC_ECR_ETHEREN                   (0x00000002)
#define   FEC_FECTFCR_MSK                   (0x00FC0000)
#define   FEC_FECRFCR_MSK                   (0x00F80000)
#define   FEC_EIR_GRA                       (0x10000000)
#define   FEC_TCR_GTS                       (0x00000001)
#define   FEC_MIBC_ENABLE                   (0x00000000)
#define   FEC_MIB_LEN                       (228)
#define   FEC_PHY_ADDR                      (0x01)

#define FEC_RX_DMA_PRI                      (6)
#define FEC_TX_DMA_PRI                      (6)

#define   FEC_TX_BUF_NUMBER                 (8)
#define   FEC_RX_BUF_NUMBER                 (64)

#define   FEC_TX_INDEX_MASK                 (0x7)
#define   FEC_RX_INDEX_MASK                 (0x3f)

#define   FEC_RX_DESC_FEC0                  SYS_SRAM_FEC_START
#define   FEC_TX_DESC_FEC0                  FEC_RX_DESC_FEC0 + FEC_RX_BUF_NUMBER * sizeof(MCD_bufDescFec)

#define   FEC_RX_DESC_FEC1                  SYS_SRAM_FEC_START + SYS_SRAM_FEC_SIZE/2
#define   FEC_TX_DESC_FEC1                  FEC_RX_DESC_FEC1 + FEC_RX_BUF_NUMBER * sizeof(MCD_bufDescFec)

#define   FEC_EIR_MII                       (0x00800000)
#define   FEC_MMFR_READ                     (0x60020000)
#define   FEC_MMFR_WRITE                    (0x50020000)

#define   FEC_FLAGS_RX                      (0x00000001)

#define   FEC_CRCPOL                        (0xEDB88320)

#define   FEC_MII_TIMEOUT                   (2)
#define   FEC_GR_TIMEOUT                    (1)
#define   FEC_TX_TIMEOUT                    (1)
#define   FEC_RX_TIMEOUT                    (1)

#define   FEC_SW_RST                        0x2000000
#define   FEC_RST_CTL                       0x1000000

struct fec_rmon {
	u32 rmon_t_drop;		/* 0x200 Count of frames not counted correctly */
	u32 rmon_t_packets;		/* 0x204 RMON Tx packet count */
	u32 rmon_t_bc_pkt;		/* 0x208 RMON Tx Broadcast Packets */
	u32 rmon_t_mc_pkt;		/* 0x20C RMON Tx Multicast Packets */
	u32 rmon_t_crc_align;	/* 0x210 RMON Tx Packets w CRC/Align error */
	u32 rmon_t_undersize;	/* 0x214 RMON Tx Packets < 64 bytes, good crc */
	u32 rmon_t_oversize;	/* 0x218 RMON Tx Packets > MAX_FL bytes, good crc */
	u32 rmon_t_frag;		/* 0x21C RMON Tx Packets < 64 bytes, bad crc */
	u32 rmon_t_jab;			/* 0x220 RMON Tx Packets > MAX_FL bytes, bad crc */
	u32 rmon_t_col;			/* 0x224 RMON Tx collision count */
	u32 rmon_t_p64;			/* 0x228 RMON Tx 64 byte packets */
	u32 rmon_t_p65to127;	/* 0x22C RMON Tx 65 to 127 byte packets */
	u32 rmon_t_p128to255;	/* 0x230 RMON Tx 128 to 255 byte packets */
	u32 rmon_t_p256to511;	/* 0x234 RMON Tx 256 to 511 byte packets */
	u32 rmon_t_p512to1023;	/* 0x238 RMON Tx 512 to 1023 byte packets */
	u32 rmon_t_p1024to2047;	/* 0x23C RMON Tx 1024 to 2047 byte packets */
	u32 rmon_t_p_gte2048;	/* 0x240 RMON Tx packets w > 2048 bytes */
	u32 rmon_t_octets;		/* 0x244 RMON Tx Octets */
	u32 ieee_t_drop;		/* 0x248 Count of frames not counted correctly */
	u32 ieee_t_frame_ok;	/* 0x24C Frames Transmitted OK */
	u32 ieee_t_1col;		/* 0x250 Frames Transmitted with Single Collision */
	u32 ieee_t_mcol;		/* 0x254 Frames Transmitted with Multiple Collisions */
	u32 ieee_t_def;			/* 0x258 Frames Transmitted after Deferral Delay */
	u32 ieee_t_lcol;		/* 0x25c Frames Transmitted with Late Collision */
	u32 ieee_t_excol;		/* 0x260 Frames Transmitted with Excessive Collisions */
	u32 ieee_t_macerr;		/* 0x264 Frames Transmitted with Tx FIFO Underrun */
	u32 ieee_t_cserr;		/* 0x268 Frames Transmitted with Carrier Sense Error */
	u32 ieee_t_sqe;			/* 0x26C Frames Transmitted with SQE Error */
	u32 ieee_t_fdxfc;		/* 0x270 Flow Control Pause frames transmitted */
	u32 ieee_t_octets_ok;	/* 0x274 Octet count for Frames Transmitted w/o Error */
};

struct fecregs {
	u8  resv1[4];	/* 0x000 Reserved */
	u32 eir;		/* 0x004 Ethernet Interrupt Event Register */
	u32 eimr;		/* 0x008 Ethernet Interrupt Mask Register */
	u8  resv2[24];	/* 0x00C Reserved */
	u32 ecr;		/* 0x024 Ethernet Control Register */
	u8  resv3[24];	/* 0x028 Reserved */
	u32 mdata;		/* 0x040 MII Data Register */
	u32 mscr;		/* 0x044 MII Speed Control Register */
	u8  resv4[28];	/* 0x048 Reserved */
	u32 mibc;		/* 0x064 MIB Control/Status Register */
	u8  resv5[28];	/* 0x068 Reserved */
	u32 rcr;		/* 0x084 Receive Control Register */
	u32 rhr;		/* 0x088 Receive Hash Register */
	u8  resv6[56];	/* 0x08C Reserved */
	u32 tcr;		/* 0x0C4 Transmit Control Register */
	u8  resv7[28];	/* 0x0C8 Reserved */
	u32 palr;		/* 0x0E4 Physical Address Low Register */
	u32 pahr;		/* 0x0E8 Physical Address High Register */
	u32 opd;		/* 0x0EC Opcode / Pause Duration Register */
	u8  resv8[40];	/* 0x0F0 Reserved */
	u32 iaur;		/* 0x118 Individual Address Upper Register */
	u32 ialr;		/* 0x11C Individual Address Lower Register */
	u32 gaur;		/* 0x120 Group Address Upper Register */
	u32 galr;		/* 0x124 Group Address Lower Register */
	u8  resv9[28];	/* 0x128 Reserved */
	u32 fectfw;		/* 0x144 FEC Transmit FIFO Watermark*/
	u8  resv10[60];	/* 0x148 Reserved */
	u32 fecrfdr;	/* 0x184 FEC Receive FIFO Data Register*/
	u32 fecrfsr;	/* 0x188 FEC Receive FIFO Status Register*/
	u32 fecrcr;		/* 0x18C FEC Receive FIFO Control Register */
	u32 fecrlrfp;	/* 0x190 FEC Receive FIFO Last Read Frame Pointer*/
	u32 fecrlwfp;	/* 0x194 FEC Receive FIFO Last Write Frame Pointer*/
	u32 fecrfar;	/* 0x198 FEC Receive FIFO Alarm Register*/
	u32 fecrfrp;	/* 0x19C FEC Receive FIFO Read Pointer Register*/
	u32 fecrfwp;	/* 0x1A0 FEC Receive FIFO Write Pointer Register*/
	u32 fectfdr;	/* 0x1A4 FEC Transmit FIFO Data Register*/
	u32 fectfsr;	/* 0x1A8 FEC Transmit FIFO Status Register*/
	u32 fectfcr;	/* 0x1AC FEC Transmit FIFO Control Register*/
	u32 fectlrfp;	/* 0x1B0 FEC Transmit FIFO Last Read Frame Pointer*/
	u32 fectlwfp;	/* 0x1B4 FEC Transmit FIFO Last Write Frame Pointer*/
	u32 fectfar;	/* 0x1B8 FEC Transmit FIFO Alarm Register*/
	u32 fectfrp;	/* 0x1BC FEC Transmit FIFO Read Pointer Register*/
	u32 fectfwp;	/* 0x1C0 FEC Transmit FIFO Write Pointer Register*/
	u32 fecfrst;	/* 0x1C4 FIFO Reset Register*/
	u32 fecctcwr;	/* 0x1C8 CRC and Transmit Frame Control Word Register*/
	u8  resv11[52]; /* 0x1CC Reserved */
	struct fec_rmon rmon; /* 0x200 RMON values */
};

/* fec private */
struct fec_priv {
	int index;									/* fec hw number */
	volatile struct fecregs *regs;				/* FEC Registers */
	struct net_device *netdev;					/* owning net device */
	/* TX */
	volatile unsigned int fecpriv_current_tx;	/* current tx desc index */
	volatile unsigned int fecpriv_next_tx;		/* next tx desc index */
	void *fecpriv_txbuf[FEC_TX_BUF_NUMBER];		/* tx buffer ptrs */
	MCD_bufDescFec *fecpriv_txdesc;				/* tx descriptor ptrs */
	/* RX */
	unsigned int fecpriv_current_rx;			/* current rx desc index */
	struct sk_buff *askb_rx[FEC_RX_BUF_NUMBER];	/* rx SKB ptrs */
	MCD_bufDescFec *fecpriv_rxdesc;				/* rx descriptor ptrs */
#ifdef CONFIG_FEC_NAPI
	/* NAPI */
	struct napi_struct napi;
#endif

	/* DMA */
	unsigned int fecpriv_initiator_rx;			/* rx dma initiator */
	unsigned int fecpriv_initiator_tx;			/* tx dma initiator */
	int fecpriv_fec_rx_channel;					/* rx dma channel */
	int fecpriv_fec_tx_channel;					/* tx dma channel */
	int fecpriv_rx_requestor;					/* rx dma requestor */
	int fecpriv_tx_requestor;					/* tx dma requestor */
	void *fecpriv_interrupt_fec_rx_handler;		/* dma rx handler */
	void *fecpriv_interrupt_fec_tx_handler;		/* dma tx handler */
	unsigned char *fecpriv_mac_addr;	/* private fec mac addr */
	struct net_device_stats fecpriv_stat;	/* stats ptr */
	spinlock_t rx_lock;
	spinlock_t tx_lock;
	int fecpriv_rxflag;
	/* MDIO and PHY **/
	struct mii_bus *mdio;
	struct phy_device *phy;
	int oldduplex;
	int oldspeed;
	int oldlink;
	struct tasklet_struct fecpriv_tasklet_reinit;
};

#define fecpriv_lock rx_lock

#define VERSION "0.20"
#define FEC_DRV_NAME "FEC"
#define FEC_DRV_VERSION VERSION

/* Low level MII functions */
extern int fec_read_mii(unsigned int base_addr, unsigned int pa, unsigned int ra, int *data);
extern int fec_write_mii(unsigned int base_addr, unsigned int pa, unsigned int ra, int data);
extern int fec_reset_mii(unsigned int base_addr);

/* Phylib support functions */
extern void fec_adjust_link(struct net_device *dev);
extern const struct ethtool_ops *fec_set_ethtool(struct net_device *dev);
extern int fec_mdio_setup(struct net_device *dev);
extern int fec_mdio_remove(struct net_device *dev);

#endif

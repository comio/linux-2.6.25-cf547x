/*
* Performance and stability improvements: (C) Copyright 2008,
*      Daniel Krueger, SYSTEC electronic GmbH
* (C) Copyright 2010 Industrie Dial Face S.p.A.
*     Luigi 'Comio' Mantellini <luigi.mantellini@idf-hit.com>
*
* Code crunched to get it to work on 2.6.24 -- FEC cleanup coming
* soon -- Kurt Mahan
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/bitops.h>
#include <linux/phy.h>

#include <asm/coldfire.h>
#include <asm/mcfsim.h>
#include <asm/cf_cacheflush.h>

#include <asm/dma.h>
#include <asm/MCD_dma.h>
#include <asm/m5485sram.h>
#include <asm/m5485gpio.h>
#include <asm/virtconvert.h>
#include <asm/irq.h>

#include "fec.h"

#define FU_TEST

#ifdef	CONFIG_FEC_548x_ENABLE_FEC2
#define	FEC_MAX_PORTS	2
#define	FEC_2
#else
#define	FEC_MAX_PORTS	1
#undef	FEC_2
#endif

#define FEC_DEV_WEIGHT 64

MODULE_DESCRIPTION("DMA Fast Ethernet Controller driver ver " VERSION);

struct net_device *fec_dev[FEC_MAX_PORTS];


/* FEC functions */
int __init fec_init(void);
static struct net_device_stats *fec_get_stat(struct net_device *dev);
static int fec_open(struct net_device *dev);
static int fec_close(struct net_device *nd);
static int fec_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);
static int fec_tx(struct sk_buff *skb, struct net_device *dev);
static void fec_set_multicast_list(struct net_device *nd);
static int fec_set_mac_address(struct net_device *dev, void *p);
static void fec_tx_timeout(struct net_device *dev);
static void fec_interrupt_tx_handler(void *priv);
static void fec_interrupt_rx_handler(void *priv);
static irqreturn_t fec_interrupt_handler(int irq, void *dev_id);
static void fec_interrupt_fec_reinit(unsigned long data);

/* default fec0 address */
unsigned char fec_mac_addr_fec0[6] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x50 };

#ifdef   FEC_2
/* default fec1 address */
unsigned char fec_mac_addr_fec1[6] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x51 };
#endif

#ifndef MODULE
int fec_str_to_mac(char *str_mac, unsigned char *addr);
int __init fec_mac_setup0(char *s);
#endif

#ifndef MODULE
int __init fec_mac_setup1(char *s);
#endif

module_init(fec_init);
/* module_exit(fec_cleanup); */

__setup("mac0=", fec_mac_setup0);

#ifdef   FEC_2
__setup("mac1=", fec_mac_setup1);
#endif

/* tx ring support functions */
/* WARNING: THESE FUNCTIONS DON'T LOCK */
static inline void fec_zero_tx_ring(struct fec_priv *);
static inline void fec_init_tx_ring(struct fec_priv *);
static inline void fec_clear_tx_ring(struct fec_priv *);
static inline void fec_reset_tx_ring(struct fec_priv *);

/* rx ring support functions */
/* WARNING: THESE FUNCTIONS DON'T LOCK */
static inline void fec_zero_rx_ring(struct fec_priv *);
static inline void fec_setup_rxdma(MCD_bufDescFec *dma, struct sk_buff *skb);
static inline struct sk_buff *fec_alloc_rx_buffer(void);
static inline void fec_init_rx_ring(struct fec_priv *);

static inline void fec_zero_tx_ring(struct fec_priv *fp)
{
	int i;

	/* initialize the pointers to the socket buffers */
	for (i = 0; i < FEC_TX_BUF_NUMBER; i++) {
		fp->txbuf_na[i] = NULL;
		fp->txbuf[i] = NULL;
	}
	fp->txfree = 0;
	fp->current_tx = fp->next_tx = 0;
}

static inline void fec_init_tx_ring(struct fec_priv *fp)
{
	int i;

	/* Initialize tx descriptors and start DMA for the transmission */
	for (i = 0; i < FEC_TX_BUF_NUMBER; i++) {
		void *ptr = kmalloc(FEC_MAXBUF_SIZE + 15, GFP_DMA | GFP_ATOMIC);
		fp->txbuf_na[i] = ptr;
		fp->txbuf[i] = (void *)(((unsigned long)ptr + (FEC_RX_BUFFER_ALIGN -1)) & ~(FEC_RX_BUFFER_ALIGN-1));
		fp->tx_dma.desc[i].dataPointer = (unsigned int)virt_to_phys(fp->txbuf[i]);
	}
	fec_reset_tx_ring(fp);
}

static inline void fec_clear_tx_ring(struct fec_priv *fp)
{
	int i;

	for (i = 0; i < FEC_TX_BUF_NUMBER; i++) {
		if (fp->txbuf_na[i]) {
			kfree(fp->txbuf_na[i]);
			fp->txbuf_na[i] = NULL;
		}
		fp->txbuf[i] = NULL;
	}
	fp->txfree = 0;
}

static inline void fec_reset_tx_ring(struct fec_priv *fp)
{
	int i;

	for (i = 0; i < FEC_TX_BUF_NUMBER; i++) {
		fp->tx_dma.desc[i].statCtrl = MCD_FEC_INTERRUPT;
	}
	fp->tx_dma.desc[i - 1].statCtrl |= MCD_FEC_WRAP;
	fp->current_tx = fp->next_tx = 0;
	fp->txfree = FEC_TX_BUF_NUMBER;
}

static inline void fec_zero_rx_ring(struct fec_priv *fp)
{
	int i;

	for (i = 0; i < FEC_RX_BUF_NUMBER; i++) {
		fp->askb_rx[i] = NULL;
	}
	fp->current_rx = 0;
}

static inline struct sk_buff *fec_alloc_rx_buffer(void)
{
	struct sk_buff *skb = alloc_skb(FEC_MAXBUF_SIZE + FEC_RX_BUFFER_ALIGN - 1, GFP_DMA);

	if (skb) {
		unsigned int alignamount = FEC_RX_BUFFER_ALIGN - (((unsigned long) skb->data) & (FEC_RX_BUFFER_ALIGN - 1));

		skb_reserve(skb, alignamount);
	}

	return skb;
}

static inline void fec_zero_rxdma(MCD_bufDescFec *dma)
{
	dma->dataPointer = 0;
	dma->length = 0;
}

static inline void fec_setup_rxdma(MCD_bufDescFec *dma, struct sk_buff *skb)
{
	dma->dataPointer = (unsigned int)virt_to_phys(skb->tail);
	dma->length = FEC_MAXBUF_SIZE;
}

static inline void fec_init_rx_ring(struct fec_priv *fp)
{
	int i;

	for (i = 0; i < FEC_RX_BUF_NUMBER; i++) {
		fp->askb_rx[i] = fec_alloc_rx_buffer();

		if (!fp->askb_rx[i]) {
			fec_zero_rxdma(&fp->rx_dma.desc[i]);
			fp->rx_dma.desc[i].statCtrl = 0;
		} else {
			fp->askb_rx[i]->dev = fp->netdev;
			fec_setup_rxdma(&fp->rx_dma.desc[i], fp->askb_rx[i]);
			fp->rx_dma.desc[i].statCtrl = MCD_FEC_BUF_READY | MCD_FEC_INTERRUPT;
		}
	}

	fp->rx_dma.desc[i - 1].statCtrl |= MCD_FEC_WRAP;
	fp->current_rx = 0;
}

static inline void fec_clear_rx_ring(struct fec_priv *fp)
{
	int i;

	for (i = 0; i < FEC_RX_BUF_NUMBER; i++) {
		if (fp->askb_rx[i]) {
			kfree_skb(fp->askb_rx[i]);
			fp->askb_rx[i] = NULL;
		}
		fp->rx_dma.desc[i - 1].statCtrl = 0;
		fec_zero_rxdma(&fp->rx_dma.desc[i]);
	}
	fp->current_rx = 0;
}

static inline void fec_reset_rx_ring(struct fec_priv *fp)
{
	fec_clear_rx_ring(fp);
	fec_init_rx_ring(fp);
}

/*
* Initialize a FEC device
*/
int fec_enet_init(struct net_device *dev)
{
	static int index = 0;
	struct fec_priv *fp = netdev_priv(dev);

	fp->index = index;
	fp->netdev = dev;
	fec_dev[index] = dev;

	if (index == 0) {
		/* disable fec0 */
		FEC_ECR(FEC_BASE_ADDR_FEC0) = FEC_ECR_DISABLE;

		/* setup the interrupt handler */
		dev->irq = 64 + ISC_FEC0;

		if (request_irq(dev->irq, fec_interrupt_handler,
				IRQF_DISABLED, "ColdFire FEC 0", dev)) {
			dev->irq = 0;
			printk("Cannot allocate FEC0 IRQ\n");
		} else {
			/* interrupt priority and level */
			MCF_ICR(ISC_FEC0) = ILP_FEC0;
		}

		/* fec base address */
		fp->regs = (struct fecregs *)FEC_BASE_ADDR_FEC0;
		dev->base_addr = FEC_BASE_ADDR_FEC0;

		/* requestor numbers */
		fp->rx_dma.requestor = DMA_FEC0_RX;
		fp->tx_dma.requestor = DMA_FEC0_TX;

		/* fec0 handlers */
		fp->rx_dma.interrupt_handler = fec_interrupt_rx_handler;
		fp->tx_dma.interrupt_handler = fec_interrupt_tx_handler;

		/* tx descriptors */
		fp->tx_dma.desc = (void *)FEC_TX_DESC_FEC0;

		/* rx descriptors */
		fp->rx_dma.desc = (void *)FEC_RX_DESC_FEC0;

		/* mac addr */
		fp->mac_addr = fec_mac_addr_fec0;
	} else {
		/* disable fec1 */
		FEC_ECR(FEC_BASE_ADDR_FEC1) = FEC_ECR_DISABLE;
#ifdef FEC_2
		/* setup the interrupt handler */
		dev->irq = 64 + ISC_FEC1;

		if (request_irq(dev->irq, fec_interrupt_handler,
				IRQF_DISABLED, "ColdFire FEC 1", dev)) {
			dev->irq = 0;
			printk("Cannot allocate FEC1 IRQ\n");
		} else {
			/* interrupt priority and level */
			MCF_ICR(ISC_FEC1) = ILP_FEC1;
		}

		/* fec base address */
		fp->regs = (struct fecregs *)FEC_BASE_ADDR_FEC1;

		/* requestor numbers */
		fp->rx_dma.requestor = DMA_FEC1_RX;
		fp->tx_dma.requestor = DMA_FEC1_TX;

		/* fec1 handlers */
		fp->rx_dma.interrupt_handler = fec_interrupt_rx_handler;
		fp->tx_dma.interrupt_handler = fec_interrupt_tx_handler;

		/* tx descriptors */
		fp->tx_dma.desc = (void *)FEC_TX_DESC_FEC1;

		/* rx descriptors */
		fp->rx_dma.desc = (void *)FEC_RX_DESC_FEC1;

		/* mac addr */
		fp->mac_addr = fec_mac_addr_fec1;
#endif
	}

	dev->base_addr = (typeof(dev->base_addr))fp->regs;

	printk("RMON addr %p\n", &fp->regs->rmon);

	/* clear MIB */
	memset((void *)(&fp->regs->rmon), 0, sizeof(fp->regs->rmon));

	/* clear the statistics structure */
	memset((void *)&(fp->stat), 0, sizeof(struct net_device_stats));

	/* grab the FEC initiators */
	dma_set_initiator(fp->tx_dma.requestor);
	fp->tx_dma.initiator = dma_get_initiator(fp->tx_dma.requestor);
	dma_set_initiator(fp->rx_dma.requestor);
	fp->rx_dma.initiator = dma_get_initiator(fp->rx_dma.requestor);

	/* reset the DMA channels */
	fp->rx_dma.channel = -1;
	fp->tx_dma.channel = -1;

	fec_zero_rx_ring(fp);
	fec_zero_tx_ring(fp);

	ether_setup(dev);

	dev->open = fec_open;
	dev->stop = fec_close;
	dev->do_ioctl = fec_ioctl;
	dev->hard_start_xmit = fec_tx;
	dev->get_stats = fec_get_stat;
	dev->set_multicast_list = fec_set_multicast_list;
	dev->set_mac_address = fec_set_mac_address;
	dev->tx_timeout = fec_tx_timeout;
	dev->watchdog_timeo = FEC_TX_TIMEOUT * HZ;
	/* netif_napi_add(dev, &priv->napi, fec_poll, FEC_DEV_WEIGHT); */

	memcpy(dev->dev_addr, fp->mac_addr, ETH_ALEN);

	fp->oldlink = 0;
	fp->oldspeed = 0;
	fp->oldduplex = -1;

	/* Initialize spinlocks */
	spin_lock_init(&fp->rx_lock);
	spin_lock_init(&fp->tx_lock);

	// Initialize FEC/I2C/IRQ Pin Assignment Register
	FEC_GPIO_PAR_FECI2CIRQ &= 0xF;
	FEC_GPIO_PAR_FECI2CIRQ |= FEC_FECI2CIRQ;

	index++;
	return 0;
}

/*
* Module Initialization
*/
int __init fec_init(void)
{
	struct net_device *dev;
	int i;
	int err;
	DECLARE_MAC_BUF(mac);

	printk(KERN_INFO "FEC ENET (DMA) Version %s\n", VERSION);

	XARB_ADRTO = 0x2000;
	XARB_DATTO = 0x2500;
	XARB_BUSTO = 0x3000;

	XARB_CFG = XARB_CFG_AT | XARB_CFG_DT;

	/* Master Priority Enable */
	XARB_PRIEN = 0xff;
	XARB_PRI = 0;

	for (i = 0; i < FEC_MAX_PORTS; i++) {
		dev = alloc_etherdev(sizeof(struct fec_priv));
		if (!dev)
			return -ENOMEM;
		err = fec_enet_init(dev);
		if (err) {
			free_netdev(dev);
			continue;
		}
		if (register_netdev(dev) != 0) {
			free_netdev(dev);
			return -EIO;
		}

		fec_set_ethtool(dev);
		fec_mdio_setup(dev);

		fec_adjust_link(dev);

		printk(KERN_INFO "%s: ethernet %s\n",
			dev->name, print_mac(mac, dev->dev_addr));
	}
	return 0;
}

/*
* Stop a device
*/
void fec_stop(struct net_device *dev)
{
	struct fec_priv *fp = netdev_priv(dev);

	/* napi_disable(&fp->napi); */

	spin_lock_irq(&fp->tx_lock);
	dma_remove_initiator(fp->tx_dma.initiator);
	spin_unlock_irq(&fp->tx_lock);

	spin_lock_irq(&fp->rx_lock);
	dma_remove_initiator(fp->rx_dma.initiator);
	spin_unlock_irq(&fp->rx_lock);

	if (dev->irq) {
		free_irq(dev->irq, dev);
	}
}

/************************************************************************
* NAME: fec_open
*
* DESCRIPTION: This function performs the initialization of
*				of FEC and corresponding KS8721 transiver
*
* RETURNS: If no error occurs, this function returns zero.
*************************************************************************/
int fec_open(struct net_device *dev)
{
	struct fec_priv *fp = netdev_priv(dev);
	unsigned long base_addr = (unsigned long)dev->base_addr;
	int channel;
	int error_code = -EBUSY;

	/* RX Setup*/
	spin_lock_irq(&fp->rx_lock);

	channel = dma_set_channel_fec(fp->rx_dma.requestor);

	if (channel == -1) {
		printk("Dma channel cannot be reserved\n");
		spin_unlock_irq(&fp->rx_lock);
		goto ERRORS;
	}

	fp->rx_dma.channel = channel;
	dma_connect(channel, fp->rx_dma.interrupt_handler, dev);

	spin_unlock_irq(&fp->rx_lock);

	/* TX Setup */
	spin_lock_irq(&fp->tx_lock);

	channel = dma_set_channel_fec(fp->tx_dma.requestor);

	if (channel == -1) {
		printk("Dma channel cannot be reserved\n");
		spin_unlock_irq(&fp->tx_lock);
		goto ERRORS;
	}

	fp->tx_dma.channel = channel;
	dma_connect(channel, fp->tx_dma.interrupt_handler, dev);
	spin_unlock_irq(&fp->tx_lock);

	/* init tasklet for controller reinitialization */
	tasklet_init(&fp->tasklet_reinit, fec_interrupt_fec_reinit,
			(unsigned long)dev);

	/* Reset FIFOs */
	FEC_FECFRST(base_addr) |= FEC_SW_RST | FEC_RST_CTL;
	FEC_FECFRST(base_addr) &= ~FEC_SW_RST;

	/* Reset and disable FEC */
	FEC_ECR(base_addr) = FEC_ECR_RESET;

	udelay(10);

	/* Clear all events */
	FEC_EIR(base_addr) = FEC_EIR_CLEAR;

	/* Reset FIFO status */
	FEC_FECTFSR(base_addr) = FEC_FECTFSR_MSK;
	FEC_FECRFSR(base_addr) = FEC_FECRFSR_MSK;

	/* Set the default address */
	FEC_PALR(base_addr) = (fp->mac_addr[0] << 24) |
		(fp->mac_addr[1] << 16) |
		(fp->mac_addr[2] << 8) | fp->mac_addr[3];
	FEC_PAUR(base_addr) = (fp->mac_addr[4] << 24) |
		(fp->mac_addr[5] << 16) | 0x8808;

	/* Reset the group address descriptor */
	FEC_GALR(base_addr) = 0x00000000;
	FEC_GAUR(base_addr) = 0x00000000;

	/* Reset the individual address descriptor */
	FEC_IALR(base_addr) = 0x00000000;
	FEC_IAUR(base_addr) = 0x00000000;

	/* Set the receive control register */
	FEC_RCR(base_addr) = FEC_RCR_MAX_FRM_SIZE | FEC_RCR_MII;

	/* Set the receive FIFO control register */
	FEC_FECRFCR(base_addr) = FEC_FECRFCR_FRM | FEC_FECRFCR_GR | (FEC_FECRFCR_MSK	// disable all but ...
									& ~FEC_FECRFCR_FAE	// enable frame accept error
									& ~FEC_FECRFCR_RXW	// enable receive wait condition
//                                & ~FEC_FECRFCR_UF   // enable FIFO underflow
		);

	/* Set the receive FIFO alarm register */
	FEC_FECRFAR(base_addr) = FEC_FECRFAR_ALARM;

	/* Set the transmit FIFO control register */
	FEC_FECTFCR(base_addr) = FEC_FECTFCR_FRM | FEC_FECTFCR_GR | (FEC_FECTFCR_MSK	// disable all but ...
									& ~FEC_FECTFCR_FAE	// enable frame accept error
//                                & ~FEC_FECTFCR_TXW  // enable transmit wait condition
//                                & ~FEC_FECTFCR_UF   // enable FIFO underflow
									& ~FEC_FECTFCR_OF);	// enable FIFO overflow

	/* Set the transmit FIFO alarm register */
	FEC_FECTFAR(base_addr) = FEC_FECTFAR_ALARM;

	/* Set the Tx FIFO watermark */
	FEC_FECTFWR(base_addr) = FEC_FECTFWR_XWMRK;

	/* Enable the transmitter to append the CRC */
	FEC_CTCWR(base_addr) = FEC_CTCWR_TFCW_CRC;

	/* Enable the ethernet interrupts */
//      FEC_EIMR(base_addr) = FEC_EIMR_MASK;
	FEC_EIMR(base_addr) = FEC_EIMR_DISABLE
		| FEC_EIR_LC
		| FEC_EIR_RL
		| FEC_EIR_HBERR | FEC_EIR_XFUN | FEC_EIR_XFERR | FEC_EIR_RFERR;


	/* Enable MIB */
	FEC_MIBC(base_addr) = FEC_MIBC_ENABLE;

	/* Enable FEC */
	FEC_ECR(base_addr) |= FEC_ECR_ETHEREN;

	fec_reset_mii(base_addr);

	fec_init_tx_ring(fp);

	MCD_startDma(fp->tx_dma.channel, (char *)fp->tx_dma.desc, 0,
			(unsigned char *)&(FEC_FECTFDR(base_addr)), 0,
			FEC_MAX_FRM_SIZE, 0, fp->tx_dma.initiator,
			FEC_TX_DMA_PRI, MCD_FECTX_DMA | MCD_INTERRUPT,
			MCD_NO_CSUM | MCD_NO_BYTE_SWAP);

	/* Initialize rx descriptors and start DMA for the reception */

	fec_init_rx_ring(fp);

	MCD_startDma(fp->rx_dma.channel, (char *)fp->rx_dma.desc, 0,
			(unsigned char *)&(FEC_FECRFDR(base_addr)), 0,
			FEC_MAX_FRM_SIZE, 0, fp->rx_dma.initiator,
			FEC_RX_DMA_PRI, MCD_FECRX_DMA | MCD_INTERRUPT,
			MCD_NO_CSUM | MCD_NO_BYTE_SWAP);

	netif_start_queue(dev);
	return 0;

ERRORS:

	/* Remove the channels and return with the error code */
	if (fp->rx_dma.channel != -1) {
		dma_disconnect(fp->rx_dma.channel);
		dma_remove_channel_by_number(fp->rx_dma.channel);
		fp->rx_dma.channel = -1;
	}

	if (fp->tx_dma.channel != -1) {
		dma_disconnect(fp->tx_dma.channel);
		dma_remove_channel_by_number(fp->tx_dma.channel);
		fp->tx_dma.channel = -1;
	}

	return error_code;
}

/************************************************************************
* NAME: fec_close
*
* DESCRIPTION: This function performs the graceful stop of the
*				transmission and disables FEC
*
* RETURNS: This function always returns zero.
*************************************************************************/
int fec_close(struct net_device *dev)
{
	struct fec_priv *fp = netdev_priv(dev);
	unsigned long base_addr = (unsigned long)dev->base_addr;
	unsigned long time;
	int i;

	netif_stop_queue(dev);

	/* Perform the graceful stop */
	FEC_TCR(base_addr) |= FEC_TCR_GTS;

	time = jiffies;

	/* Wait for the graceful stop */
	while (!(FEC_EIR(base_addr) & FEC_EIR_GRA)
		&& jiffies - time < FEC_GR_TIMEOUT * HZ)
		schedule();

	/* Disable FEC */
	FEC_ECR(base_addr) = FEC_ECR_DISABLE;

	/* Reset the DMA channels */
	spin_lock_irq(&fp->tx_lock);
	MCD_killDma(fp->tx_dma.channel);

	dma_remove_channel_by_number(fp->tx_dma.channel);
	dma_disconnect(fp->tx_dma.channel);
	fp->tx_dma.channel = -1;

	fec_clear_tx_ring(fp);

	spin_unlock_irq(&fp->tx_lock);

	spin_lock_irq(&fp->rx_lock);
	MCD_killDma(fp->rx_dma.channel);

	dma_remove_channel_by_number(fp->rx_dma.channel);
	dma_disconnect(fp->rx_dma.channel);
	fp->rx_dma.channel = -1;

	fec_clear_rx_ring(fp);

	spin_unlock_irq(&fp->rx_lock);

	return 0;
}

int fec_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct fec_priv *priv = (struct fec_priv *)dev->priv;

	if (!netif_running(dev))
		return -EINVAL;

	if (!priv->phy)
		return -EINVAL;

	return phy_mii_ioctl(priv->phy, if_mii(rq), cmd);
}

/************************************************************************
* +NAME: fec_get_stat
*
* RETURNS: This function returns the statistical information.
*************************************************************************/
struct net_device_stats *fec_get_stat(struct net_device *dev)
{
	struct fec_priv *fp = netdev_priv(dev);
	unsigned long base_addr = dev->base_addr;

	/* Receive the statistical information */
	fp->stat.rx_packets = FECSTAT_RMON_R_PACKETS(base_addr);
	fp->stat.tx_packets = FECSTAT_RMON_T_PACKETS(base_addr);
	fp->stat.rx_bytes = FECSTAT_RMON_R_OCTETS(base_addr);
	fp->stat.tx_bytes = FECSTAT_RMON_T_OCTETS(base_addr);

	fp->stat.multicast = FECSTAT_RMON_R_MC_PKT(base_addr);
	fp->stat.collisions = FECSTAT_RMON_T_COL(base_addr);

	fp->stat.rx_length_errors =
		FECSTAT_RMON_R_UNDERSIZE(base_addr) +
		FECSTAT_RMON_R_OVERSIZE(base_addr) +
		FECSTAT_RMON_R_FRAG(base_addr) + FECSTAT_RMON_R_JAB(base_addr);
	fp->stat.rx_crc_errors = FECSTAT_IEEE_R_CRC(base_addr);
	fp->stat.rx_frame_errors = FECSTAT_IEEE_R_ALIGN(base_addr);
	fp->stat.rx_over_errors = FECSTAT_IEEE_R_MACERR(base_addr);

	fp->stat.tx_carrier_errors = FECSTAT_IEEE_T_CSERR(base_addr);
	fp->stat.tx_fifo_errors = FECSTAT_IEEE_T_MACERR(base_addr);
	fp->stat.tx_window_errors = FECSTAT_IEEE_T_LCOL(base_addr);

	/* I hope that one frame doesn't have more than one error */
	fp->stat.rx_errors = fp->stat.rx_length_errors +
		fp->stat.rx_crc_errors +
		fp->stat.rx_frame_errors +
		fp->stat.rx_over_errors + fp->stat.rx_dropped;
	fp->stat.tx_errors = fp->stat.tx_carrier_errors +
		fp->stat.tx_fifo_errors +
		fp->stat.tx_window_errors +
		fp->stat.tx_aborted_errors +
		fp->stat.tx_heartbeat_errors + fp->stat.tx_dropped;

	return &fp->stat;
}

/************************************************************************
* NAME: fec_set_multicast_list
*
* DESCRIPTION: This function sets the frame filtering parameters
*************************************************************************/
void fec_set_multicast_list(struct net_device *dev)
{
	struct dev_mc_list *dmi;
	unsigned int crc, data;
	int i, j, k;
	unsigned long base_addr = (unsigned long)dev->base_addr;

	if (dev->flags & IFF_PROMISC) {
		FEC_RCR(base_addr) |= FEC_RCR_PROM;
	} else {
		FEC_RCR(base_addr) &= ~FEC_RCR_PROM;
	}

	if (dev->flags & IFF_ALLMULTI) {
		/* Allow all incoming frames */
		FEC_GALR(base_addr) = 0xFFFFFFFF;
		FEC_GAUR(base_addr) = 0xFFFFFFFF;
	} else {
		/* Reset the group address register */
		FEC_GALR(base_addr) = 0x00000000;
		FEC_GAUR(base_addr) = 0x00000000;

		/* Process all addresses */
		for (i = 0, dmi = dev->mc_list; i < dev->mc_count; i++, dmi = dmi->next) {
			/* Processing must be only for the group addresses */
			if (!(dmi->dmi_addr[0] & 1))
				continue;

			/* Calculate crc value for the current address */
			crc = 0xFFFFFFFF;
			for (j = 0; j < dmi->dmi_addrlen; j++) {
				for (k = 0, data = dmi->dmi_addr[j]; k < 8;
					k++, data >>= 1) {
					if ((crc ^ data) & 1)
						crc = (crc >> 1) ^ FEC_CRCPOL;
					else
						crc >>= 1;
				}
			}

			/* Add this value */
			crc >>= 26;
			crc &= 0x3F;
			if (crc > 31)
				FEC_GAUR(base_addr) |= 0x1 << (crc - 32);
			else
				FEC_GALR(base_addr) |= 0x1 << crc;
		}
	}
}

/************************************************************************
* NAME: fec_set_mac_address
*
* DESCRIPTION: This function sets the MAC address
*************************************************************************/
int fec_set_mac_address(struct net_device *dev, void *p)
{
	struct fec_priv *fp = netdev_priv(dev);
	unsigned long base_addr = (unsigned long)dev->base_addr;
	struct sockaddr *addr = p;

	if (netif_running(dev))
		return -EBUSY;

	/* Copy a new address to the device structure */
	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	/* Copy a new address to the private structure */
	memcpy(fp->mac_addr, addr->sa_data, 6);

	/* Set the address to the registers */
	FEC_PALR(base_addr) = (fp->mac_addr[0] << 24) |
		(fp->mac_addr[1] << 16) |
		(fp->mac_addr[2] << 8) | fp->mac_addr[3];
	FEC_PAUR(base_addr) = (fp->mac_addr[4] << 24) |
		(fp->mac_addr[5] << 16) | 0x8808;

	return 0;
}

/************************************************************************
* NAME: fec_tx
*
* DESCRIPTION: This function starts transmission of the frame using DMA
*
* RETURNS: This function always returns zero.
*************************************************************************/
int fec_tx(struct sk_buff *skb, struct net_device *dev)
{
	struct fec_priv *fp = netdev_priv(dev);
	void *data;
	unsigned long flags;

	/* flush data cache before initializing the descriptor and starting DMA */
	/* FU: not needed, packet was just written to DRAM and cache is writethrough */

	spin_lock_irqsave(&fp->tx_lock , flags);

	if (fp->txfree > 0) {
		/* We have a free buffer... fill it! */

		/* Initialize the descriptor */
		data = fp->txbuf[fp->next_tx];
		memcpy(data, skb->data, skb->len);

		fp->txfree--;
		fp->tx_dma.desc[fp->next_tx].length = skb->len;
		fp->tx_dma.desc[fp->next_tx].statCtrl |= (MCD_FEC_END_FRAME | MCD_FEC_BUF_READY);
		fp->next_tx = (fp->next_tx + 1) & FEC_TX_INDEX_MASK;

	} else {
		/* We haven't free buff... drop the packet! sorry! */
		printk(KERN_WARNING "FEC%d: TX full... dropping packet!\n", fp->index);
		fp->stat.tx_dropped++;
		if (!netif_queue_stopped(dev)) {
			printk(KERN_WARNING "FEC%d: Stopping...\n", fp->index);
			netif_stop_queue(dev);
		}
	}

	spin_unlock_irqrestore(&fp->tx_lock , flags);

	/* Tell the DMA to continue the transmission */
	MCD_continDma(fp->tx_dma.channel);

	dev_kfree_skb(skb);

	dev->trans_start = jiffies;

	return 0;
}

/************************************************************************
* NAME: fec_tx_timeout
*
* DESCRIPTION: If the interrupt processing of received frames was lost
*              and DMA stopped the reception, this function clears
*              the transmission descriptors and starts DMA
*
*************************************************************************/
void fec_tx_timeout(struct net_device *dev)
{
	struct fec_priv *fp = netdev_priv(dev);
	unsigned long base_addr = (unsigned long)dev->base_addr;
       unsigned long flags;

	printk(KERN_WARNING "FEC: TX Timeout\n");
	MCD_killDma(fp->tx_dma.channel);

	spin_lock_irqsave(&fp->tx_lock , flags);

	fec_reset_tx_ring(fp);

	spin_unlock_irqrestore(&fp->tx_lock , flags);

	/* Reset FIFOs */
	FEC_FECFRST(base_addr) |= FEC_SW_RST;
	FEC_FECFRST(base_addr) &= ~FEC_SW_RST;

	/* Enable FEC */
	FEC_ECR(base_addr) |= FEC_ECR_ETHEREN;

	MCD_startDma(fp->tx_dma.channel, (char *)fp->tx_dma.desc, 0,
			(unsigned char *)&(FEC_FECTFDR(base_addr)), 0,
			FEC_MAX_FRM_SIZE, 0, fp->tx_dma.initiator,
			FEC_TX_DMA_PRI, MCD_FECTX_DMA | MCD_INTERRUPT,
			MCD_NO_CSUM | MCD_NO_BYTE_SWAP);

	netif_wake_queue(dev);
}

/************************************************************************
* NAME: fec_read_mii
*
* DESCRIPTION: This function reads the value from the MII register
*
* RETURNS: If no error occurs, this function returns zero.
*************************************************************************/
int fec_read_mii(unsigned int base_addr, unsigned int pa, unsigned int ra,
		int *data)
{
	int i;

	*data = 0x0000ffff;

	/* Clear the MII interrupt bit */
	FEC_EIR(base_addr) = FEC_EIR_MII;

	/* Write to the MII management frame register */
	FEC_MMFR(base_addr) = FEC_MMFR_READ | (pa << 23) | (ra << 18);

	/* Wait for the reading */
	for (i = 0; i < 10000; i++) {
		if (FEC_EIR(base_addr) & FEC_EIR_MII)
			break;
	}

	if (!(FEC_EIR(base_addr) & FEC_EIR_MII))
		return -ETIME;

	/* Clear the MII interrupt bit */
	FEC_EIR(base_addr) = FEC_EIR_MII;

	*data = FEC_MMFR(base_addr) & 0x0000FFFF;

	return 0;
}

/************************************************************************
* NAME: fec_write_mii
*
* DESCRIPTION: This function writes the value to the MII register
*
* RETURNS: If no error occurs, this function returns zero.
*************************************************************************/
int fec_write_mii(unsigned int base_addr, unsigned int pa, unsigned int ra,
		int data)
{
	int i;

	/* Clear the MII interrupt bit */
	FEC_EIR(base_addr) = FEC_EIR_MII;

	/* Write to the MII management frame register */
	FEC_MMFR(base_addr) =
		FEC_MMFR_WRITE | (pa << 23) | (ra << 18) | (data & 0x0000ffff);

	for (i = 0; i < 10000; i++) {
		if (FEC_EIR(base_addr) & FEC_EIR_MII)
			break;
	}

	if (!(FEC_EIR(base_addr) & FEC_EIR_MII))
		return -ETIME;

	/* Clear the MII interrupt bit */
	FEC_EIR(base_addr) = FEC_EIR_MII;

	return 0;
}

/************************************************************************
* NAME: fec_reset_mii
*
* DESCRIPTION: This function reset the MII registers
*
* RETURNS: If no error occurs, this function returns zero.
*************************************************************************/
int fec_reset_mii(unsigned int base_addr)
{
	FEC_MSCR(base_addr) = 20 << 1;

	if (base_addr == FEC_BASE_ADDR_FEC0)
		MCF_GPIO_PAR_FECI2CIRQ |= 0xF000;
	else
		MCF_GPIO_PAR_FECI2CIRQ |= 0x0FC0;

	return 0;
}


void fec_adjust_link(struct net_device *dev)
{
	struct fec_priv *priv = netdev_priv(dev);
	unsigned long flags;
	struct phy_device *phydev = priv->phy;
	int new_state = 0;
	unsigned long int base_addr = dev->base_addr;

	if (!dev||!dev->priv||!priv->phy)
		return;

	spin_lock_irqsave(&priv->tx_lock, flags);
	if (phydev->link) {

		/* Now we make sure that we can be in full duplex mode.
		* If not, we operate in half-duplex mode. */
		if (phydev->duplex != priv->oldduplex) {
			new_state = 1;
			if (!(phydev->duplex))
				/* Disable reception of frames while transmitting */
				FEC_RCR(base_addr) |= FEC_RCR_DRT;
			else
				/* Enable the full duplex mode */
				FEC_TCR(base_addr) = FEC_TCR_FDEN | FEC_TCR_HBC;

			priv->oldduplex = phydev->duplex;
		}

		if (phydev->speed != priv->oldspeed) {
			new_state = 1;
			switch (phydev->speed) {
				case 100:
				case 10:
					break;
				default:
					printk(KERN_WARNING
						"%s: Ack!  Speed (%d) is not 10/100/1000!\n",
							dev->name, phydev->speed);
							break;
			}

			priv->oldspeed = phydev->speed;
		}

		if (!priv->oldlink) {
			new_state = 1;
			priv->oldlink = 1;
			netif_schedule(dev);
		}
	} else if (priv->oldlink) {
		new_state = 1;
		priv->oldlink = 0;
		priv->oldspeed = 0;
		priv->oldduplex = -1;
	}

	if (new_state)
		phy_print_status(phydev);

	spin_unlock_irqrestore(&priv->tx_lock, flags);
}


/************************************************************************
* NAME: fec_interrupt_tx_handler
*
* DESCRIPTION: This function is called when the data
*              transmission from the buffer to the FEC is completed.
*
*************************************************************************/
void fec_interrupt_tx_handler(void *priv)
{
	struct net_device *dev = (struct net_device *)priv;
	struct fec_priv *fp = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&fp->tx_lock , flags);

	fp->txfree++;
	fp->current_tx = (fp->current_tx + 1) & FEC_TX_INDEX_MASK;

	spin_unlock_irqrestore(&fp->tx_lock , flags);

	if (netif_queue_stopped(dev)) {
		printk(KERN_WARNING "FEC%d: resume interface...\n", fp->index);
		netif_wake_queue(dev);
	}
}

/************************************************************************
* NAME: fec_interrupt_rx_handler
*
* DESCRIPTION: This function is called when the data
*              reception from the FEC to the reception buffer is completed.
*
*************************************************************************/
void fec_interrupt_rx_handler(void *priv)
{
	struct net_device *dev = (struct net_device *)priv;
	struct fec_priv *fp = netdev_priv(dev);
	struct sk_buff *skb;
	int i;
	unsigned long flags;

	spin_lock_irqsave(&fp->rx_lock , flags);

	/* Some buffers can be missed */
	if (!(fp->rx_dma.desc[fp->current_rx].statCtrl & MCD_FEC_END_FRAME)) {
		/* Find a valid index */
		for (i = 0; i < FEC_RX_BUF_NUMBER; i++) {
			if ((fp->rx_dma.desc[fp->current_rx].statCtrl & MCD_FEC_END_FRAME)) {
				/* This is a non-empty buffer */
				break;
			}
			fp->current_rx = (fp->current_rx + 1) & FEC_RX_INDEX_MASK;
		}

		if (i == FEC_RX_BUF_NUMBER) {
			/* There are no data to process */
			/* Tell the DMA to continue the reception */
			MCD_continDma(fp->rx_dma.channel);

			spin_unlock_irqrestore(&fp->rx_lock , flags);

			return;
		}
	}

	for (; fp->rx_dma.desc[fp->current_rx].statCtrl & MCD_FEC_END_FRAME;
		fp->current_rx = (fp->current_rx + 1) & FEC_RX_INDEX_MASK) {
		if ((fp->rx_dma.desc[fp->current_rx].length <= FEC_MAXBUF_SIZE) &&
			(fp->rx_dma.desc[fp->current_rx].length > 4)) {

			skb = fp->askb_rx[fp->current_rx];

			if (!skb) {
				fp->stat.rx_dropped++;
			} else {
				/* flush data cache before initializing the descriptor and starting DMA */
				/* Make sure CPU is not going to read cached data instead of actual packet data */
				cf_dcache_flush_range((unsigned)(fp->askb_rx[fp->current_rx]->tail),
									  (unsigned)(fp->askb_rx[fp->current_rx]->tail) + fp->rx_dma.desc[fp->current_rx].length);

				skb_put(skb, fp->rx_dma.desc[fp->current_rx].length - 4);
				skb->protocol = eth_type_trans(skb, dev);
				netif_rx(skb);
			}

			fp->rx_dma.desc[fp->current_rx].statCtrl &= ~MCD_FEC_END_FRAME;

			/* allocate new skbuff */
			fp->askb_rx[fp->current_rx] = fec_alloc_rx_buffer();
			if (!fp->askb_rx[fp->current_rx]) {
				fec_zero_rxdma(&fp->rx_dma.desc[fp->current_rx]);
				fp->stat.rx_dropped++;
			} else {
				fp->askb_rx[fp->current_rx]->dev = dev;
				fec_setup_rxdma(&fp->rx_dma.desc[fp->current_rx], fp->askb_rx[fp->current_rx]);
				fp->rx_dma.desc[fp->current_rx].statCtrl |= MCD_FEC_BUF_READY;
			}
		}
	}
	spin_unlock_irqrestore(&fp->rx_lock , flags);

	/* Tell the DMA to continue the reception */
	MCD_continDma(fp->rx_dma.channel);
}

/************************************************************************
* NAME: fec_interrupt_handler
*
* DESCRIPTION: This function is called when some special errors occur
*
*************************************************************************/
irqreturn_t fec_interrupt_handler(int irq, void *dev_id)
{

	struct net_device *dev = (struct net_device *)dev_id;
	struct fec_priv *fp = netdev_priv(dev);
	unsigned long base_addr = (unsigned long)dev->base_addr;
	unsigned long events;

	/* Read and clear the events */
	events = FEC_EIR(base_addr) & FEC_EIMR(base_addr);

	printk(KERN_INFO "FEC: Error on FEC%d events=%08lx\n", fp->index, events);

	if (events & FEC_EIR_HBERR) {
		fp->stat.tx_heartbeat_errors++;
		FEC_EIR(base_addr) = FEC_EIR_HBERR;
		dev->stats.tx_dropped++;
	}

	/* receive/transmit FIFO error */
	if (((events & FEC_EIR_RFERR) != 0) || ((events & FEC_EIR_XFERR) != 0)) {
		printk(KERN_INFO "FEC: FEC%d FFECRFSR=%08x FECTFSR=%08x\n", fp->index,
			FEC_FECRFSR(base_addr), FEC_FECTFSR(base_addr));

		/* kill DMA receive channel */
		MCD_killDma(fp->rx_dma.channel);

		/* kill running transmission by DMA */
		MCD_killDma(fp->tx_dma.channel);

		/* Reset FIFOs */
		FEC_FECFRST(base_addr) |= FEC_SW_RST;
		FEC_FECFRST(base_addr) &= ~FEC_SW_RST;

		/* reset receive FIFO status register */
		FEC_FECRFSR(base_addr) = FEC_FECRFSR_FAE |
			FEC_FECRFSR_RXW | FEC_FECRFSR_UF;

		/* reset transmit FIFO status register */
		FEC_FECTFSR(base_addr) = FEC_FECTFSR_FAE |
			FEC_FECTFSR_TXW | FEC_FECTFSR_UF | FEC_FECTFSR_OF;

		/* reset RFERR and XFERR event */
		FEC_EIR(base_addr) = FEC_EIR_RFERR | FEC_EIR_XFERR;

		/* stop queue */
		netif_stop_queue(dev);

		/* execute reinitialization as tasklet */
		tasklet_schedule(&fp->tasklet_reinit);

		fp->stat.rx_dropped++;
	}

	/* transmit FIFO underrun */
	if ((events & FEC_EIR_XFUN) != 0) {
		/* reset XFUN event */
		FEC_EIR(base_addr) = FEC_EIR_XFUN;
		fp->stat.tx_aborted_errors++;
		dev->stats.tx_dropped++;
	}

	/* late collision */
	if ((events & FEC_EIR_LC) != 0) {
		/* reset LC event */
		FEC_EIR(base_addr) = FEC_EIR_LC;
		fp->stat.tx_aborted_errors++;
		dev->stats.tx_window_errors++;
	}

	/* collision retry limit */
	if ((events & FEC_EIR_RL) != 0) {
		/* reset RL event */
		FEC_EIR(base_addr) = FEC_EIR_RL;
		fp->stat.tx_aborted_errors++;
		dev->stats.tx_aborted_errors++;
	}

	return 0;
}

/************************************************************************
* NAME: fec_interrupt_reinit
*
* DESCRIPTION: This function is called from interrupt handler
*              when controller must be reinitialized.
*
*************************************************************************/
void fec_interrupt_fec_reinit(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct fec_priv *fp = netdev_priv(dev);
	unsigned long base_addr = (unsigned long)dev->base_addr;

	/* Initialize reception descriptors and start DMA for the reception */
	fec_reset_rx_ring(fp);

	/* restart frame transmission */
	fp->stat.tx_dropped += fp->txfree;
	fec_reset_tx_ring(fp);

	/* restart DMA from beginning */
	MCD_startDma(fp->rx_dma.channel,
			(char *)fp->rx_dma.desc, 0,
			(unsigned char *)&(FEC_FECRFDR(base_addr)), 0,
			FEC_MAX_FRM_SIZE, 0, fp->rx_dma.initiator,
			FEC_RX_DMA_PRI, MCD_FECRX_DMA | MCD_INTERRUPT,
			MCD_NO_CSUM | MCD_NO_BYTE_SWAP);

	MCD_startDma(fp->tx_dma.channel, (char *)fp->tx_dma.desc, 0,
			(unsigned char *)&(FEC_FECTFDR(base_addr)), 0,
			FEC_MAX_FRM_SIZE, 0, fp->tx_dma.initiator,
			FEC_TX_DMA_PRI, MCD_FECTX_DMA | MCD_INTERRUPT,
			MCD_NO_CSUM | MCD_NO_BYTE_SWAP);

	/* Enable FEC */
	FEC_ECR(base_addr) |= FEC_ECR_ETHEREN;

	netif_wake_queue(dev);
}

#ifndef MODULE
/************************************************************************
* NAME: fec_mac_setup0
*
* DESCRIPTION: This function sets the MAC address of FEC0 from command line
*
*************************************************************************/
int __init fec_mac_setup0(char *s)
{
	if (!s || !*s)
		return 1;

	if (fec_str_to_mac(s, fec_mac_addr_fec0))
		printk
			("The MAC address of FEC0 cannot be set from command line");
	return 1;
}

#ifdef   FEC_2

/************************************************************************
* NAME: fec_mac_setup1
*
* DESCRIPTION: This function sets the MAC address of FEC1 from command line
*
*************************************************************************/
int __init fec_mac_setup1(char *s)
{
	if (!s || !*s)
		return 1;

	if (fec_str_to_mac(s, fec_mac_addr_fec1))
		printk
			("The MAC address of FEC1 cannot be set from command line");
	return 1;
}
#endif

/************************************************************************
* NAME: fec_str_to_mac
*
* DESCRIPTION: This function interprets the character string into MAC addr
*
*************************************************************************/
int fec_str_to_mac(char *str_mac, unsigned char *addr)
{
	unsigned long val;
	char c;
	unsigned long octet[6], *octetptr = octet;
	int i;

	again:
	val = 0;
	while ((c = *str_mac) != '\0') {
		if ((c >= '0') && (c <= '9')) {
			val = (val * 16) + (c - '0');
			str_mac++;
			continue;
		} else if (((c >= 'a') && (c <= 'f'))
			|| ((c >= 'A') && (c <= 'F'))) {
			val =
				(val << 4) + (c + 10 -
					(((c >= 'a')
						&& (c <= 'f')) ? 'a' : 'A'));
			str_mac++;
			continue;
		}
		break;
	}
	if (*str_mac == ':') {
		*octetptr++ = val, str_mac++;
		if (octetptr >= octet + 6)
			return 1;
		goto again;
	}

	/* Check for trailing characters */
	if (*str_mac && !(*str_mac == ' '))
		return 1;

	*octetptr++ = val;

	if ((octetptr - octet) == 6) {
		for (i = 0; i <= 6; i++)
			addr[i] = octet[i];
	} else
		return 1;

	return 0;
}
#endif

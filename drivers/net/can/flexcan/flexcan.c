/*
 * flexcan.c
 *
 * DESCRIPTION:
 *  CAN bus driver for the alone generic (as possible as) FLEXCAN controller.
 *
 * AUTHOR:
 *  Andrey Volkov <avolkov@varma-el.com>
 *
 * COPYRIGHT:
 *  2005-2006, Varma Electronics Oy
 *
 * LICENCE:
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  HISTORY:
 *  	2008-06-23 Support for MCF548x's FlexCAN
 * 			Huan, Wang <b18965@freescale.com>
 */



#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/can.h>
#include <linux/list.h>
#include <linux/io.h>

#include <linux/can/dev.h>
#include <linux/can/error.h>
#include "flexcan.h"
#include <asm/coldfire.h>
#include <asm/m5485sim.h>
#include <linux/can/version.h>	/* for RCSID. Removed by mkpatch script */
RCSID("$Id$");

struct flexcan_priv {
	struct can_priv can;
	volatile unsigned long flags;
	u8 shadow_statflg;
	u8 shadow_canrier;
	u8 cur_pri;
	u8 tx_active;

	struct list_head tx_head;
	struct napi_struct napi;
	struct net_device *dev;
};


static int flexcan_hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct can_frame *frame = (struct can_frame *)skb->data;
	struct flexcan_regs *regs = (struct flexcan_regs *)dev->base_addr;
	int i, len;
	int txbuf = 0;
	u32 can_id, can_ext, tmp, tmp1;

	/* Transmission inactive */
	regs->cantxfg[txbuf].can_dlc = MB_CNT_CODE(0x08);

	can_ext = frame->can_id;
	if (can_ext & CAN_EFF_FLAG) {
		/* Frame format is extended */
		regs->cantxfg[txbuf].can_dlc |= (1 << 21);
		regs->cantxfg[txbuf].can_dlc |= (1 << 22);
		can_id = frame->can_id & MB_ID_EXT;
		if (frame->can_id & CAN_RTR_FLAG)
			regs->cantxfg[txbuf].can_dlc |= (1 << 20);

		tmp = (can_id & CAN_SFF_MASK) << 18;
		tmp1 = can_id >> 11;
		can_id = tmp | tmp1;
		regs->cantxfg[txbuf].can_id = can_id;
	} else {
		/* Frame format is standard */
		can_id = frame->can_id & MB_ID_EXT;
		if (frame->can_id & CAN_RTR_FLAG)
			regs->cantxfg[txbuf].can_dlc |= (1 << 20);

		regs->cantxfg[txbuf].can_id = can_id << 18;
	}

	len = 8;
	for (i = 0; i < len; i++)
		regs->cantxfg[txbuf].data[i] = frame->data[i];

	regs->cantxfg[txbuf].can_dlc |= len << 16;
	/* Transmission active */
	regs->cantxfg[txbuf].can_dlc |= MB_CNT_CODE(0x0c);
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static void flexcan_tx_timeout(struct net_device *dev)
{
	struct sk_buff *skb;
	struct flexcan_regs *regs = (struct flexcan_regs *)dev->base_addr;
	struct can_frame *frame;
	int length = 8;

	/* Diable the interrupts */
	regs->imask = IMASK_BUFF_DISABLE_ALL;

	skb = dev_alloc_skb(sizeof(struct can_frame));
	if (!skb) {
		if (printk_ratelimit())
			dev_notice(ND2D(dev), "TIMEOUT packet dropped.\n");
		return;
	}
	frame = (struct can_frame *)skb_put(skb, sizeof(struct can_frame));

	frame->can_dlc = length;

	skb->dev = dev;
	skb->protocol = __constant_htons(ETH_P_CAN);
	skb->pkt_type = PACKET_BROADCAST;
	skb->ip_summed = CHECKSUM_UNNECESSARY;

	netif_rx(skb);
}

static irqreturn_t flexcan_isr(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct flexcan_regs *regs = (struct flexcan_regs *)dev->base_addr;
	struct net_device_stats *stats = dev->get_stats(dev);
	struct sk_buff *skb;
	struct can_frame *frame;
	u32 iflags, oflags;
	int i, k;
	int retval = 1;

	iflags = regs->iflag;
	oflags = iflags;
	for (i = 0; i < 16; i++) {
		if (iflags & (0x01 << i)) {
			struct flexcan_mb *mb = &regs->cantxfg[i];
			int ctrl = mb->can_dlc;
			int code = (ctrl >> 24) & 0x0f;
			int length = (ctrl >> 16) & 0x0f;
			u32 tmp, tmp1;

			if (code < 8 && (length > 0)) {
				/* receive frame */
				skb = dev_alloc_skb(sizeof(struct can_frame));
				if (!skb)
					dev_notice(ND2D(dev),
							"Packets dropped.\n");
				skb->dev = dev;
				frame = (struct can_frame *)skb_put(skb,
						sizeof(struct can_frame));

				frame->can_id &= 0x0;
				frame->can_dlc = length;
				tmp1 = mb->can_id & MB_ID_EXT;
				if (ctrl & MB_CNT_IDE) {
					tmp = tmp1;
					tmp = (tmp >> 18) & CAN_SFF_MASK;
					frame->can_id = (tmp1 << 11) | tmp;
					frame->can_id &= CAN_EFF_MASK;
					frame->can_id |= CAN_EFF_FLAG;
					if (ctrl & MB_CNT_RTR)
						frame->can_id |= CAN_RTR_FLAG;
				} else {
					frame->can_id  = tmp1 >> 18;
					if (ctrl & MB_CNT_RTR)
						frame->can_id |= CAN_RTR_FLAG;
				}

				for (k = 0; k < 8; k++)
					frame->data[k] = mb->data[k];

				mb->can_dlc &= MB_CODE_MASK;
				mb->can_dlc |= MB_CNT_CODE(0x04);

				stats->rx_packets++;
				stats->rx_bytes += frame->can_dlc;
				skb->dev = dev;
				skb->protocol = __constant_htons(ETH_P_CAN);
				skb->ip_summed = CHECKSUM_UNNECESSARY;

				retval = netif_rx(skb);
				if (retval == NET_RX_DROP)
					dev_notice(ND2D(dev),
							"Packets dropped.\n");
			} else {
				/* transmit frame */
				mb->can_dlc = MB_CNT_CODE(0x04);
			}
		}
	}
	regs->iflag = oflags;

	return IRQ_HANDLED;
}

static int flexcan_do_set_bit_time(struct net_device *dev,
				 struct can_bittime *bt)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	struct flexcan_regs *regs = (struct flexcan_regs *)dev->base_addr;
	int ret = 0;
	u32 reg;

	if (bt->type != CAN_BITTIME_STD)
		return -EINVAL;

	spin_lock_irq(&priv->can.irq_lock);

	reg = CANCTRL_PRESDIV(bt->std.brp) | CANCTRL_PSEG1(bt->std.phase_seg1
			- 1) | CANCTRL_PSEG2(bt->std.phase_seg2 - 1);
	regs->canctrl &= CANCTRL_BITTIME;
	regs->canctrl |= (reg | CANCTRL_SAMP(bt->std.sam) |
		CANCTRL_PROPSEG(bt->std.prop_seg - 1));

	spin_unlock_irq(&priv->can.irq_lock);
	return ret;
}


static int flexcan_open(struct net_device *dev)
{
	int ret, i, j;
	struct flexcan_regs *regs = (struct flexcan_regs *)dev->base_addr;

#if defined(CONFIG_M547X_8X)
	MCF_PAR_TIMER = MCF_PAR_TIMER | 0x28;
	MCF_PAR_TIMER = MCF_PAR_TIMER & 0xf8;
	MCF_PAR_DSPI = MCF_PAR_DSPI | 0x0a00;
	MCF_PAR_FECI2CIRQ = MCF_PAR_FECI2CIRQ | 0x0283;
	MCF_PAR_PSCn(2) = MCF_PAR_PSCn(2) & 0x0f;
	MCF_PAR_PSCn(2) = MCF_PAR_PSCn(2) | 0x50;
#endif

	regs->canmcr |= CANMCR_SOFTRST;
	regs->canmcr |= CANMCR_MDIS;
	udelay(10);

	if ((regs->canmcr & CANMCR_SOFTRST) != 0x0) {
		dev_err(ND2D(dev), "Failed to softreset can module.\n");
		return -1;
	}

	/* Enable error and bus off interrupt */
	regs->canctrl |= (CANCTRL_RJW(3) | CANCTRL_ERRMSK |
			CANCTRL_BOFFMSK);

	/* Set lowest buffer transmitted first */
	regs->canctrl |= CANCTRL_LBUF;

	for (i = 0; i < 16; i++) {
		regs->cantxfg[i].can_dlc = 0;
		regs->cantxfg[i].can_id = 0;
		for (j = 0; j < 8; j++)
			regs->cantxfg[i].data[j] = 0;

		/* Put MB into rx queue */
		regs->cantxfg[i].can_dlc = MB_CNT_CODE(0x04);
	}

	/* acceptance mask/acceptance code (accept everything) */
	regs->rxgmask = 0x00000000;
	regs->rx14mask = 0x00000000;
	regs->rx15mask = 0x00000000;
	/* extended frame */
	regs->cantxfg[14].can_dlc |= 0x600000;
	/* Enable flexcan module */
	regs->canmcr &= ~CANMCR_MDIS;
	/* Synchronize with the can bus */
	regs->canmcr &= ~CANMCR_HALT;

#if defined(CONFIG_M547X_8X)
	for (i = 0; i < 2; i++) {
		MCF_ICR(ISC_CANn_MBOR(i)) = 0x33;
		MCF_ICR(ISC_CANn_ERR(i)) = 0x33;
		MCF_ICR(ISC_CANn_BUSOFF(i)) = 0x33;
	}

	ret = request_irq(dev->irq + 64, flexcan_isr, IRQF_DISABLED,
			dev->name, dev);
	ret = request_irq(dev->irq + 1 + 64, flexcan_isr, IRQF_DISABLED,
			dev->name, dev);
	ret = request_irq(dev->irq + 2 + 64, flexcan_isr, IRQF_DISABLED,
			dev->name, dev);
	if (ret < 0) {
		printk(KERN_ERR "%s - failed to attach interrupt.\n",
		       dev->name);
		return ret;
	}
#endif

	/* Enable all interrupts */
	regs->imask = IMASK_BUFF_ENABLE_ALL;
	netif_start_queue(dev);
	return 0;
}

static int flexcan_close(struct net_device *dev)
{
	struct flexcan_regs *regs = (struct flexcan_regs *)dev->base_addr;

	netif_stop_queue(dev);

	/* Disable all interrupts */
	regs->imask = IMASK_BUFF_DISABLE_ALL;
	free_irq(dev->irq + 64, dev);
	free_irq(dev->irq + 1 + 64, dev);
	free_irq(dev->irq + 2 + 64, dev);

	/* Disable module */
	regs->canmcr |= CANMCR_MDIS;
	return 0;
}

int register_flexcandev(struct net_device *dev, int clock_src)
{
	struct flexcan_regs *regs = (struct flexcan_regs *)dev->base_addr;

	regs->canmcr &= ~CANMCR_MDIS;
	udelay(100);
	regs->canmcr |= (CANMCR_FRZ | CANMCR_HALT);
	return register_netdev(dev);
}
EXPORT_SYMBOL(register_flexcandev);

void unregister_flexcandev(struct net_device *dev)
{
	struct flexcan_regs *regs = (struct flexcan_regs *)dev->base_addr;

	regs->canmcr |= (CANMCR_FRZ | CANMCR_HALT);
	regs->canmcr |= CANMCR_MDIS;

	unregister_netdev(dev);
}
EXPORT_SYMBOL(unregister_flexcandev);

struct net_device *alloc_flexcandev(void)
{
	struct net_device *dev;
	struct flexcan_priv *priv;

	dev = alloc_candev(sizeof(struct flexcan_priv));
	if (!dev)
		return NULL;

	priv = netdev_priv(dev);
	priv->dev = dev;
	dev->open = flexcan_open;
	dev->stop = flexcan_close;
	dev->hard_start_xmit = flexcan_hard_start_xmit;
	dev->tx_timeout = flexcan_tx_timeout;
	dev->flags |= IFF_NOARP;
	priv->can.do_set_bit_time = flexcan_do_set_bit_time;
	return dev;
}
EXPORT_SYMBOL(alloc_flexcandev);

MODULE_AUTHOR("Andrey Volkov <avolkov@varma-el.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN port driver for flexcan based chip");

/*
 * linux/arch/m68k/coldfire/ints.c -- General interrupt handling code
 *
 * Copyright (C) 1999-2002  Greg Ungerer (gerg@snapgear.com)
 * Copyright (C) 1998  D. Jeff Dionne <jeff@lineo.ca>,
 *                     Kenneth Albanowski <kjahds@kjahds.com>,
 * Copyright (C) 2000  Lineo Inc. (www.lineo.com)
 *
 * Copyright (C) 2008  Industrie Dial Face S.p.A.
 * Luigi 'Comio' Mantellini <luigi.mantellini@idf-hit.com>
 *
 * Copyright Freescale Semiconductor, Inc. 2007, 2008
 * 	Kurt Mahan kmahan@freescale.com
 * 	Matt Waddel Matt.Waddel@freescale.com
 *
 * Based on:
 * linux/arch/m68k/kernel/ints.c &
 * linux/arch/m68knommu/5307/ints.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel_stat.h>
#include <linux/errno.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/traps.h>
#include <asm/page.h>
#include <asm/machdep.h>
#include <asm/irq_regs.h>

#include <asm/mcfsim.h>

/*
 * IRQ Handler lists.
 */
static struct irq_node *irq_list[SYS_IRQS];
static struct irq_controller *irq_controller[SYS_IRQS];
static int irq_depth[SYS_IRQS];

/*
 * IRQ Controller
 */
#if defined(CONFIG_M5445X)
void m5445x_irq_enable(unsigned int irq);
void m5445x_irq_disable(unsigned int irq);
static struct irq_controller m5445x_irq_controller = {
	.name		= "M5445X",
	.lock		= SPIN_LOCK_UNLOCKED,
	.enable		= m5445x_irq_enable,
	.disable	= m5445x_irq_disable,
};
#elif defined(CONFIG_M547X_8X)
void m547x_8x_irq_enable(unsigned int irq);
void m547x_8x_irq_disable(unsigned int irq);
static struct irq_controller m547x_8x_irq_controller = {
	.name		= "M547X_8X",
	.lock		= SPIN_LOCK_UNLOCKED,
	.enable		= m547x_8x_irq_enable,
	.disable	= m547x_8x_irq_disable,
};
#else
# error No IRQ controller defined
#endif

#define	POOL_SIZE 	SYS_IRQS
static struct irq_node  pool[POOL_SIZE];
static struct irq_node *get_irq_node(void);

/* The number of spurious interrupts */
unsigned int num_spurious;
asmlinkage void handle_badint(struct pt_regs *regs);


#ifdef CONFIG_M547X_8X
static unsigned char intc_ilp[64] = {
	0,				//  0: Reserved
	ILP_EPFn(1),	//  1: Edge port flag 1
	ILP_EPFn(2),	//  2: Edge port flag 2
	ILP_EPFn(3),	//  3: Edge port flag 3
	ILP_EPFn(4),	//  4: Edge port flag 4
	ILP_EPFn(5),	//  5: Edge port flag 5
	ILP_EPFn(6),	//  6: Edge port flag 6
	ILP_EPFn(7),	//  7: Edge port flag 7
	0,				//  8: Not Used
	0,				//  9: Not Used
	0,				// 10: Not Used
	0,				// 11: Not Used
	0,				// 12: Not Used
	0,				// 13: Not Used
	0,				// 14: Not Used
	ILP_USB_EPn(0),	// 15: Endpoint 0 interrupt
	ILP_USB_EPn(1),	// 16: Endpoint 1 interrupt
	ILP_USB_EPn(2),	// 17: Endpoint 2 interrupt
	ILP_USB_EPn(3),	// 18: Endpoint 3 interrupt
	ILP_USB_EPn(4),	// 19: Endpoint 4 interrupt
	ILP_USB_EPn(5),	// 20: Endpoint 5 interrupt
	ILP_USB_EPn(6),	// 21: Endpoint 6 interrupt
	ILP_USB_ISR,	// 22: USB 2.0 general interrupt
	ILP_USB_AISR,	// 23: USB 2.0 core interrupt
	ILP_USB_OR,		// 24: OR of all USB interrupts
	ILP_DSPI_OVRFW,	// 25: DSPI overflow or underflow
	ILP_DSPI_RFOF,	// 26: Receive FIFO overflow interrupt
	ILP_DSPI_RFDF,	// 27: Receive FIFO drain interrupt
	ILP_DSPI_TFUF,	// 28: Transmit FIFO underflow interrupt
	ILP_DSPI_TCF,	// 29: Transfer complete interrupt
	ILP_DSPI_TFFF,	// 30: Transfer FIFO fill interrupt
	ILP_DSPI_EOQF,	// 31: End of queue interrupt
	ILP_PSCn(3),	// 32: PSC3 interrupt
	ILP_PSCn(2),	// 33: PSC2 interrupt
	ILP_PSCn(1),	// 34: PSC1 interrupt
	ILP_PSCn(0),	// 35: PSC0 interrupt
	ILP_COMM_TIM,	// 36: Combined interrupts from comm timers
	ILP_SEC,		// 37: SEC interrupt
	ILP_FEC1,	// 38: FEC1 interrupt
	ILP_FEC0,	// 39: FEC0 interrupt
	ILP_I2C,		// 40: I2C interrupt
	ILP_PCI_ARB,	// 41: PCI arbiter interrupt
	ILP_PCI_CB,		// 42: Comm bus PCI interrupt
	ILP_PCI_XLB,	// 43: XLB PCI interrupt
	0,				// 44: Not Used
	0,				// 45: Not Used
	0,				// 46: Not Used
	ILP_XLB_ARB,	// 47: XLBARB to CPU interrupt
	ILP_DMA,		// 48: Multichannel DMA interrupt
	0,				// 49: Not Used
	0,				// 50: Not Used
	0,				// 51: Not Used
	0,				// 52: Not Used
	ILP_SLT1,		// 53: Slice timer 1 interrupt
	ILP_SLT0,		// 54: Slice timer 1 interrupt
	0,				// 55: Not Used
	0,				// 56: Not Used
	0,				// 57: Not Used
	0,				// 58: Not Used
	ILP_GPTn(3),	// 59: GPT3 interrupt
	ILP_GPTn(2),	// 60: GPT2 interrupt
	ILP_GPTn(1),	// 61: GPT1 interrupt
	ILP_GPTn(0),	// 62: GPT0 interrupt
	0,				// 63: Not Used
};
#endif


/*
 * void init_IRQ(void)
 *
 * This function should be called during kernel startup to initialize
 * the IRQ handling routines.
 */
void __init init_IRQ(void)
{
	int i;

#if defined(CONFIG_M5445X)
	for (i = 0; i < SYS_IRQS; i++)
		irq_controller[i] = &m5445x_irq_controller;
#elif defined(CONFIG_M547X_8X)
	for (i = 0; i < SYS_IRQS; i++)
		irq_controller[i] = &m547x_8x_irq_controller;
#endif
}

/*
 * process_int(unsigned long vec, struct pt_regs *fp)
 *
 * Process an interrupt.  Called from entry.S.
 */
asmlinkage void process_int(unsigned long vec, struct pt_regs *fp)
{
	struct pt_regs *old_regs;
	struct irq_node *node;
	old_regs = set_irq_regs(fp);
	kstat_cpu(0).irqs[vec]++;

	node = irq_list[vec];
	if (!node)
		handle_badint(fp);
	else {
		do {
			node->handler(vec, node->dev_id);
			node = node->next;
		} while (node);
	}

	set_irq_regs(old_regs);
}

/*
 * show_interrupts( struct seq_file *p, void *v)
 *
 * Called to show all the current interrupt information.
 */
int show_interrupts(struct seq_file *p, void *v)
{
	struct irq_controller *contr;
	struct irq_node *node;
	int i = *(loff_t *) v;

	if ((i < NR_IRQS) && (irq_list[i])) {
		contr = irq_controller[i];
		node = irq_list[i];
		seq_printf(p, "%-8s %3u: %10u %s", contr->name, i,
			kstat_cpu(0).irqs[i], node->devname);
		while ((node = node->next))
			seq_printf(p, ", %s", node->devname);

		seq_printf(p, "\n");
	}

	return 0;
}

/*
 * get_irq_node(void)
 *
 * Get an irq node from the pool.
 */
struct irq_node *get_irq_node(void)
{
	struct irq_node *p = pool;
	int i;

	for (i = 0; i < POOL_SIZE; i++, p++) {
		if (!p->handler) {
			memset(p, 0, sizeof(struct irq_node));
			return p;
		}
	}
	printk(KERN_INFO "%s(%s:%d): No more irq nodes, I suggest you \
		increase POOL_SIZE", __FUNCTION__, __FILE__, __LINE__);
	return NULL;
}

void init_irq_proc(void)
{
	/* Insert /proc/irq driver here */
}

int setup_irq(unsigned int irq, struct irq_node *node)
{
	struct irq_controller *contr;
	struct irq_node **prev;
	unsigned long flags;

	if (irq >= NR_IRQS || !irq_controller[irq]) {
		printk("%s: Incorrect IRQ %d from %s\n",
		       __FUNCTION__, irq, node->devname);
		return -ENXIO;
	}

	contr = irq_controller[irq];
	spin_lock_irqsave(&contr->lock, flags);

	prev = irq_list + irq;
	if (*prev) {
		/* Can't share interrupts unless both agree to */
		if (!((*prev)->flags & node->flags & IRQF_SHARED)) {
			spin_unlock_irqrestore(&contr->lock, flags);
			printk(KERN_INFO "%s: -BUSY-Incorrect IRQ %d \n",
				__FUNCTION__, irq);
			return -EBUSY;
		}
		while (*prev)
			prev = &(*prev)->next;
	}

	if (!irq_list[irq]) {
		if (contr->startup)
			contr->startup(irq);
		else
			contr->enable(irq);
	}
	node->next = NULL;
	*prev = node;

	spin_unlock_irqrestore(&contr->lock, flags);

	return 0;
}

int request_irq(unsigned int irq,
		irq_handler_t handler,
		unsigned long flags, const char *devname, void *dev_id)
{
	struct irq_node *node = get_irq_node();
	int res;

	if (!node) {
		printk(KERN_INFO "%s:get_irq_node error %x\n",
			__FUNCTION__,(unsigned int) node);
		return -ENOMEM;
	}
	node->handler = handler;
	node->flags   = flags;
	node->dev_id  = dev_id;
	node->devname = devname;

	res = setup_irq(irq, node);
	if (res)
		node->handler = NULL;

	return res;
}
EXPORT_SYMBOL(request_irq);

void free_irq(unsigned int irq, void *dev_id)
{
	struct irq_controller *contr;
	struct irq_node **p, *node;
	unsigned long flags;

	if (irq >= NR_IRQS || !irq_controller[irq]) {
		printk(KERN_DEBUG "%s: Incorrect IRQ %d\n", __FUNCTION__, irq);
		return;
	}

	contr = irq_controller[irq];
	spin_lock_irqsave(&contr->lock, flags);

	p = irq_list + irq;
	while ((node = *p)) {
		if (node->dev_id == dev_id)
			break;
		p = &node->next;
	}

	if (node) {
		*p = node->next;
		node->handler = NULL;
	} else
		printk(KERN_DEBUG "%s: Removing probably wrong IRQ %d\n",
		       __FUNCTION__, irq);

	if (!irq_list[irq]) {
		if (contr->shutdown)
			contr->shutdown(irq);
		else
			contr->disable(irq);
	}

	spin_unlock_irqrestore(&contr->lock, flags);
}
EXPORT_SYMBOL(free_irq);

void enable_irq(unsigned int irq)
{
	struct irq_controller *contr;
	unsigned long flags;

	if (irq >= NR_IRQS || !irq_controller[irq]) {
		printk(KERN_DEBUG "%s: Incorrect IRQ %d\n", __FUNCTION__, irq);
		return;
	}

	contr = irq_controller[irq];
	spin_lock_irqsave(&contr->lock, flags);
	if (irq_depth[irq]) {
		if (!--irq_depth[irq]) {
			if (contr->enable)
				contr->enable(irq);
		}
	} else
		WARN_ON(1);
	spin_unlock_irqrestore(&contr->lock, flags);
}
EXPORT_SYMBOL(enable_irq);

void disable_irq(unsigned int irq)
{
	struct irq_controller *contr;
	unsigned long flags;

	if (irq >= NR_IRQS || !irq_controller[irq]) {
		printk(KERN_DEBUG "%s: Incorrect IRQ %d\n", __FUNCTION__, irq);
		return;
	}

	contr = irq_controller[irq];
	spin_lock_irqsave(&contr->lock, flags);
	if (!irq_depth[irq]++) {
		if (contr->disable)
			contr->disable(irq);
	}
	spin_unlock_irqrestore(&contr->lock, flags);
}
EXPORT_SYMBOL(disable_irq);

void disable_irq_nosync(unsigned int irq) __attribute__((alias("disable_irq")));
EXPORT_SYMBOL(disable_irq_nosync);


unsigned long probe_irq_on(void)
{
	return 0;
}
EXPORT_SYMBOL(probe_irq_on);

int probe_irq_off(unsigned long irqs)
{
	return 0;
}
EXPORT_SYMBOL(probe_irq_off);

asmlinkage void handle_badint(struct pt_regs *regs)
{
	kstat_cpu(0).irqs[0]++;
	num_spurious++;
	printk(KERN_DEBUG "unexpected interrupt from %u\n", regs->vector);
}
EXPORT_SYMBOL(handle_badint);

#ifdef CONFIG_M5445X
/*
 * M5445X Implementation
 */
void m5445x_irq_enable(unsigned int irq)
{
	/* enable the interrupt hardware */
	if (irq < 64)
		return;

	/* adjust past non-hardware ints */
	irq -= 64;

	/* check for eport */
	if ((irq > 0) && (irq < 8)) {
		/* enable eport */
		MCF_EPORT_EPPAR &= ~(3 << (irq*2));	/* level */
		MCF_EPORT_EPDDR &= ~(1 << irq);		/* input */
		MCF_EPORT_EPIER |= 1 << irq;		/* irq enabled */
	}

	if (irq < 64) {
		/* controller 0 */
		MCF_INTC0_ICR(irq) = 0x02;
		MCF_INTC0_CIMR = irq;
	} else {
		/* controller 1 */
		irq -= 64;
		MCF_INTC1_ICR(irq) = 0x02;
		MCF_INTC1_CIMR = irq;
	}
}

void m5445x_irq_disable(unsigned int irq)
{
	/* disable the interrupt hardware */
	if (irq < 64)
		return;

	/* adjust past non-hardware ints */
	irq -= 64;

	/* check for eport */
	if ((irq > 0) && (irq < 8)) {
		/* disable eport */
		MCF_EPORT_EPIER &= ~(1 << irq);
	}

	if (irq < 64) {
		/* controller 0 */
		MCF_INTC0_ICR(irq) = 0x00;
		MCF_INTC0_SIMR = irq;
	} else {
		/* controller 1 */
		irq -= 64;
		MCF_INTC1_ICR(irq) = 0x00;
		MCF_INTC1_SIMR = irq;
	}
}
#elif defined(CONFIG_M547X_8X)
/*
 * M547X_8X Implementation
 */
void m547x_8x_irq_enable(unsigned int irq)
{
	/* enable the interrupt hardware */
	if (irq < 64)
		return;

	/* adjust past non-hardware ints */
	irq -= 64;

	/* check for eport */
	if ((irq > 0) && (irq < 8)) {
		/* enable eport */
		MCF_EPPAR &= ~(3 << (irq*2));	/* level */
		MCF_EPDDR &= ~(1 << irq);	/* input */
		MCF_EPIER |= 1 << irq;		/* irq enabled */

                /* Configure pin as input of external irq */
                switch (irq) {
                case 1:
                        MCF_GPIO_PAR_DMA = (MCF_GPIO_PAR_DMA & ~0x0c) | 0x04;
                        break;

                case 2:
                        MCF_GPIO_PAR_TIMER = (MCF_GPIO_PAR_TIMER & ~0x06) | 0x04;
                        break;

                case 3:
                        MCF_GPIO_PAR_TIMER = (MCF_GPIO_PAR_TIMER & ~0x30) | 0x20;
                        break;

                case 4:
                        MCF_GPIO_PAR_PCIBR = (MCF_GPIO_PAR_PCIBR & ~0x0300) | 0x0200;
                        break;

                case 5:
                        MCF_GPIO_PAR_FECI2CIRQ |= 0x0001;
                        break;

                case 6:
                        MCF_GPIO_PAR_FECI2CIRQ |= 0x0002;
                        break;

                case 7:
                        /* IRQ7 is always enabled */
                        break;
                }
	}

	if (irq < 32) {
		/* *grumble* don't set low bit of IMRL */
		MCF_IMRL &= (~(1 << irq) & ~1);
	}
	else {
		MCF_IMRH &= ~(1 << (irq - 32));
	}
	MCF_ICR(irq) = intc_ilp[irq];
}

void m547x_8x_irq_disable(unsigned int irq)
{
	/* disable the interrupt hardware */
	if (irq < 64)
		return;

	/* adjust past non-hardware ints */
	irq -= 64;

	/* check for eport */
	if ((irq > 0) && (irq < 8)) {
		/* disable eport */
		MCF_EPIER &= ~(1 << irq);

                /* Restore default assignment of external irq pin */
                switch (irq) {
                case 1:
                        MCF_GPIO_PAR_DMA &= ~0x0c;
                        break;

                case 2:
                        MCF_GPIO_PAR_TIMER |= 0x06;
                        break;

                case 3:
                        MCF_GPIO_PAR_TIMER |= 0x30;
                        break;

                case 4:
                        MCF_GPIO_PAR_PCIBR &= ~0x0300;
                        break;

                case 5:
                        MCF_GPIO_PAR_FECI2CIRQ |= 0x0001;
                        break;

                case 6:
                        MCF_GPIO_PAR_FECI2CIRQ |= 0x0002;
                        break;

                case 7:
                        /* IRQ7 is always enabled */
                        break;
                }
	}

	if (irq < 32) {
		MCF_IMRL |= (1 << irq);
	}
	else {
		MCF_IMRH |= (1 << (irq - 32));
	}
	MCF_ICR(irq) = 0;
}
#endif

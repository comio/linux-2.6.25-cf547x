/*
 *
 * coldfire_edma.c - eDMA driver for Coldfire MCF5445x
 *
 * Yaroslav Vinogradov yaroslav.vinogradov@freescale.com
 *
 * Copyright Freescale Semiconductor, Inc. 2007
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <asm/virtconvert.h>
#include <asm/coldfire.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <asm/mcf5445x_edma.h>
#include <asm/mcf5445x_intc.h>
#include <asm/coldfire_edma.h>


/* callback handler data for each TCD */
struct edma_isr_record {
	edma_irq_handler irq_handler;       /* interrupt handler */
	edma_error_handler error_handler;	/* error interrupt handler */
	void* dev;							/* device used for the channel */
	int  allocated;						/* busy flag */
	spinlock_t *lock;					/* spin lock (if needs to be locked in interrupt) */
	const char* device_id;				/* device id string, used in proc file system */
};

/* device structure */
struct coldfire_edma_dev {
	struct cdev cdev; 			/* character device */
	struct edma_isr_record dma_interrupt_handlers[EDMA_CHANNELS]; /* channel handlers */
};

/* allocated major device number */
static int coldfire_dma_major;
/* device driver structure */
static struct coldfire_edma_dev* devp = NULL;

/* device driver file operations */
struct file_operations coldfire_edma_fops = {
	.owner = THIS_MODULE,
};

/* eDMA channel interrupt handler */
static int dmaisr(int irq, void *dev_id)
{
	int channel = irq - EDMA_INT_CONTROLLER_BASE - EDMA_INT_CHANNEL_BASE;
	int result = IRQ_HANDLED;

	if (devp!=NULL && devp->dma_interrupt_handlers[channel].lock) {
		spin_lock(devp->dma_interrupt_handlers[channel].lock);
	}

	if (devp!=NULL && devp->dma_interrupt_handlers[channel].irq_handler) {
		result = devp->dma_interrupt_handlers[channel].irq_handler(channel,
							devp->dma_interrupt_handlers[channel].dev);
	} else {
		confirm_edma_interrupt_handled(channel);
		printk(EDMA_DRIVER_NAME ": No handler for DMA channel %d\n", channel);
	}

   	if (devp!=NULL && devp->dma_interrupt_handlers[channel].lock) {
		spin_unlock(devp->dma_interrupt_handlers[channel].lock);
	}

	return result;
}

/* eDMA error interrupt handler */
static int dma_error_isr(int irq, void* dev_id)
{
	u16 err;
	int i;

	err = MCF_EDMA_ERR;
	for (i=0;i<EDMA_CHANNELS;i++) {
		if (err & (1<<i)) {
			if (devp!=NULL && devp->dma_interrupt_handlers[i].error_handler) {
				devp->dma_interrupt_handlers[i].error_handler(i, devp->dma_interrupt_handlers[i].dev);
			} else {
				printk(KERN_WARNING EDMA_DRIVER_NAME ": DMA error on channel %d\n", i);
			}
		}
	}

	MCF_EDMA_CERR = MCF_EDMA_CERR_CAER;
	return IRQ_HANDLED;
}

/* sets channel parameters */
void set_edma_params(int channel, u32 source, u32 dest,
					 u32 attr, u32 soff, u32 nbytes, u32 slast,
					 u32 citer, u32 biter, u32 doff, u32 dlast_sga,
					 int major_int, int disable_req)
{

	if (channel<0 || channel>EDMA_CHANNELS)
		return;

	MCF_EDMA_TCD_SADDR(channel) = source;
	MCF_EDMA_TCD_DADDR(channel) = dest;
	MCF_EDMA_TCD_ATTR(channel) = attr;
	MCF_EDMA_TCD_SOFF(channel) = MCF_EDMA_TCD_SOFF_SOFF(soff);
	MCF_EDMA_TCD_NBYTES(channel) = MCF_EDMA_TCD_NBYTES_NBYTES(nbytes);
	MCF_EDMA_TCD_SLAST(channel) = MCF_EDMA_TCD_SLAST_SLAST(slast);
	MCF_EDMA_TCD_CITER(channel) = MCF_EDMA_TCD_CITER_CITER(citer);
	MCF_EDMA_TCD_BITER(channel)=MCF_EDMA_TCD_BITER_BITER(biter);
	MCF_EDMA_TCD_DOFF(channel) = MCF_EDMA_TCD_DOFF_DOFF(doff);
	MCF_EDMA_TCD_DLAST_SGA(channel) = MCF_EDMA_TCD_DLAST_SGA_DLAST_SGA(dlast_sga);
	/* interrupt at the end of major loop */
	if (major_int) {
		MCF_EDMA_TCD_CSR(channel) |= MCF_EDMA_TCD_CSR_INT_MAJOR;
	} else {
		MCF_EDMA_TCD_CSR(channel) &= ~MCF_EDMA_TCD_CSR_INT_MAJOR;
	}
	/* disable request at the end of major loop of transfer or not*/
	if (disable_req) {
		MCF_EDMA_TCD_CSR(channel) |= MCF_EDMA_TCD_CSR_D_REQ;
	} else {
		MCF_EDMA_TCD_CSR(channel) &= ~MCF_EDMA_TCD_CSR_D_REQ;
	}

}
EXPORT_SYMBOL(set_edma_params);

/* init eDMA controller */
void init_edma(void)
{
	MCF_EDMA_CR = 0;
}
EXPORT_SYMBOL(init_edma);

/* request eDMA channel */
int request_edma_channel(int channel,
						edma_irq_handler handler,
						edma_error_handler error_handler,
						void* dev,
						spinlock_t *lock,
						const char* device_id )
{
	if (devp!=NULL && channel>=0 && channel<=EDMA_CHANNELS) {
		if (devp->dma_interrupt_handlers[channel].allocated) {
			return -EBUSY;
		}
		devp->dma_interrupt_handlers[channel].allocated = 1;
		devp->dma_interrupt_handlers[channel].irq_handler = handler;
		devp->dma_interrupt_handlers[channel].error_handler = error_handler;
		devp->dma_interrupt_handlers[channel].dev = dev;
		devp->dma_interrupt_handlers[channel].lock = lock;
		devp->dma_interrupt_handlers[channel].device_id = device_id;
		return 0;
	}
	return -EINVAL;
}
EXPORT_SYMBOL(request_edma_channel);

/* free eDMA channel */
int free_edma_channel(int channel, void* dev)
{
	if (devp!=NULL && channel>=0 && channel<=EDMA_CHANNELS) {
		if (devp->dma_interrupt_handlers[channel].allocated) {
			if (devp->dma_interrupt_handlers[channel].dev != dev) {
				return -EBUSY;
			}
			devp->dma_interrupt_handlers[channel].allocated = 0;
			devp->dma_interrupt_handlers[channel].dev = NULL;
			devp->dma_interrupt_handlers[channel].irq_handler = NULL;
			devp->dma_interrupt_handlers[channel].error_handler = NULL;
			devp->dma_interrupt_handlers[channel].lock = NULL;
		}
		return 0;
	}
	return -EINVAL;
}
EXPORT_SYMBOL(free_edma_channel);

/* clean-up device driver allocated resources */
static void coldfire_edma_cleanup(void)
{
	dev_t devno;
	int i;

	/* free interrupts/memory */
	if (devp) {
		for (i=0;i<EDMA_CHANNELS;i++)
		{
			MCF_INTC0_SIMR = EDMA_INT_CHANNEL_BASE+i;
			free_irq(EDMA_INT_CHANNEL_BASE+EDMA_INT_CONTROLLER_BASE+i,	devp);
		}
		MCF_INTC0_SIMR = EDMA_INT_CHANNEL_BASE+EDMA_CHANNELS;
		free_irq(EDMA_INT_CHANNEL_BASE+EDMA_INT_CONTROLLER_BASE+EDMA_CHANNELS, devp);
		cdev_del(&devp->cdev);
		kfree(devp);
	}

	/* unregister character device */
	devno = MKDEV(coldfire_dma_major, 0);
	unregister_chrdev_region(devno, 1);
}

#ifdef CONFIG_PROC_FS
/* proc file system support */

#define FREE_CHANNEL "free"
#define DEVICE_UNKNOWN "device unknown"

static int proc_edma_show(struct seq_file *m, void *v)
{
	int i;

	if (devp==NULL) return 0;

	for (i = 0 ; i < EDMA_CHANNELS ; i++) {
		if (devp->dma_interrupt_handlers[i].allocated) {
			if (devp->dma_interrupt_handlers[i].device_id)
		    	seq_printf(m, "%2d: %s\n", i, devp->dma_interrupt_handlers[i].device_id);
			else
				seq_printf(m, "%2d: %s\n", i, DEVICE_UNKNOWN);
		} else {
			seq_printf(m, "%2d: %s\n", i, FREE_CHANNEL);
		}
	}
	return 0;
}

static int proc_edma_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_edma_show, NULL);
}

static const struct file_operations proc_edma_operations = {
	.open		= proc_edma_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_edma_init(void)
{
	struct proc_dir_entry *e;

	e = create_proc_entry("edma", 0, NULL);
	if (e)
		e->proc_fops = &proc_edma_operations;

	return 0;
}

#endif

/* initializes device driver */
static int __init coldfire_edma_init(void)
{
	dev_t dev;
	int result;
	int i;

	/* allocate free major number */
	result = alloc_chrdev_region(&dev, DMA_DEV_MINOR, 1, EDMA_DRIVER_NAME);
	if (result<0) {
		printk(KERN_WARNING EDMA_DRIVER_NAME": can't get major %d\n", result);
		return result;
	}
	coldfire_dma_major = MAJOR(dev);

	/* allocate device driver structure */
	devp = kmalloc(sizeof(struct coldfire_edma_dev), GFP_KERNEL);
	if (!devp) {
		result = -ENOMEM;
		goto fail;
	}

	/* init handlers (no handlers for beggining) */
   	for (i=0;i<EDMA_CHANNELS;i++) {
		devp->dma_interrupt_handlers[i].irq_handler = NULL;
		devp->dma_interrupt_handlers[i].error_handler = NULL;
		devp->dma_interrupt_handlers[i].dev = NULL;
		devp->dma_interrupt_handlers[i].allocated = 0;
		devp->dma_interrupt_handlers[i].lock = NULL;
		devp->dma_interrupt_handlers[i].device_id = NULL;
	}

    /* register char device */
	cdev_init(&devp->cdev, &coldfire_edma_fops);
	devp->cdev.owner = THIS_MODULE;
	devp->cdev.ops = &coldfire_edma_fops;
	result = cdev_add(&devp->cdev, dev, 1);
	if (result) {
		printk(KERN_NOTICE EDMA_DRIVER_NAME": Error %d adding coldfire-dma device\n", result);
		result = -ENODEV;
		goto fail;
	}

	/* request/enable irq for each eDMA channel */
	for (i=0;i<EDMA_CHANNELS;i++)
	{
		result = request_irq(EDMA_INT_CHANNEL_BASE+EDMA_INT_CONTROLLER_BASE+i,
			dmaisr, SA_INTERRUPT, EDMA_DRIVER_NAME, devp);
		if (result) {
			printk(KERN_WARNING EDMA_DRIVER_NAME": Cannot request irq %d\n",
				EDMA_INT_CHANNEL_BASE+EDMA_INT_CONTROLLER_BASE+i);
			result = -EBUSY;
			goto fail;
		}

		MCF_INTC0_ICR(EDMA_INT_CHANNEL_BASE+i) = EDMA_IRQ_LEVEL;
		MCF_INTC0_CIMR = EDMA_INT_CHANNEL_BASE+i;

	}

    /* request error interrupt */
	result = request_irq(EDMA_INT_CHANNEL_BASE + EDMA_INT_CONTROLLER_BASE + EDMA_CHANNELS,
				dma_error_isr, SA_INTERRUPT, EDMA_DRIVER_NAME, devp);
	if (result) {
		printk(KERN_WARNING EDMA_DRIVER_NAME": Cannot request irq %d\n",
				EDMA_INT_CHANNEL_BASE+EDMA_INT_CONTROLLER_BASE+EDMA_CHANNELS);
		result = -EBUSY;
		goto fail;
	}

	/* enable error interrupt in interrupt controller */
	MCF_INTC0_ICR(EDMA_INT_CHANNEL_BASE+EDMA_CHANNELS) = EDMA_IRQ_LEVEL;
	MCF_INTC0_CIMR = EDMA_INT_CHANNEL_BASE+EDMA_CHANNELS;

#ifdef CONFIG_PROC_FS
	proc_edma_init();
#endif

	printk(EDMA_DRIVER_NAME ": initialized successfully\n");

	return 0;
fail:
	coldfire_edma_cleanup();
	return result;

}

static void __exit coldfire_edma_exit(void)
{
	coldfire_edma_cleanup();
}

module_init(coldfire_edma_init);
module_exit(coldfire_edma_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yaroslav Vinogradov, Freescale Inc.");
MODULE_DESCRIPTION("eDMA library for Coldfire 5445x");

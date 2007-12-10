/*
 * coldfire_edma.h - eDMA driver for Coldfire MCF5445x
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

#ifndef _LINUX_COLDFIRE_DMA_H
#define _LINUX_COLDFIRE_DMA_H

#include <linux/interrupt.h>
#include <asm/mcf5445x_edma.h>

#define EDMA_DRIVER_NAME "ColdFire-eDMA"
#define DMA_DEV_MINOR 1

#ifdef CONFIG_M54455
#define EDMA_INT_CHANNEL_BASE 		8
#define EDMA_INT_CONTROLLER_BASE 	64
#define EDMA_INT_BASE			(EDMA_INT_CHANNEL_BASE + \
					 EDMA_INT_CONTROLLER_BASE)
#define EDMA_CHANNELS			16
#define EDMA_INT_ERR			16	/* edma error interrupt */
#endif /* CONFIG_M54455 */
 
typedef irqreturn_t (*edma_irq_handler)(int, void *);
typedef void (*edma_error_handler)(int, void *);
 
/* Setup transfer control descriptor (TCD)
 *   channel - descriptor number
 *   source  - source address
 *   dest    - destination address
 *   attr    - attributes
 *   soff    - source offset
 *   nbytes  - number of bytes to be transfered in minor loop
 *   slast   - last source address adjustment
 *   citer   - major loop count
 *   biter   - begining minor loop count
 *   doff    - destination offset
 *   dlast_sga - last destination address adjustment
 *   major_int - generate interrupt after each major loop
 *   disable_req - disable DMA request after major loop
 */
void set_edma_params(int channel, u32 source, u32 dest,
		u32 attr, u32 soff, u32 nbytes, u32 slast,
		u32 citer, u32 biter, u32 doff, u32 dlast_sga,
		int major_int, int disable_req);

/* Starts eDMA transfer on specified channel
 *   channel - eDMA TCD number
 */
static inline void  start_edma_transfer(int channel)
{
	MCF_EDMA_SERQ = channel;
	MCF_EDMA_SSRT = channel;
}

/* Stops eDMA transfer
 *   channel - eDMA TCD number
 */
static inline void stop_edma_transfer(int channel)
{
	MCF_EDMA_CINT = channel;
	MCF_EDMA_CERQ = channel;
}


/* Confirm that interrupt has been handled
 *   channel - eDMA TCD number
 */
static inline void confirm_edma_interrupt_handled(int channel)
{
	MCF_EDMA_CINT = channel;
}
 
/* Initialize eDMA controller */
void init_edma(void);
 
/* Request eDMA channel:
 *   channel - eDMA TCD number
 *   handler - channel IRQ callback
 *   error_handler - error interrupt handler callback for channel
 *   dev - device
 *   lock - spinlock to be locked (can be NULL)
 *   device_id - device driver name for proc file system output
 */
int request_edma_channel(int channel,
		edma_irq_handler handler,
		edma_error_handler error_handler,
		void *dev,
		spinlock_t *lock,
		const char *device_id);

/**
 * set_edma_callback - Update the channel callback/arg
 * @channel: channel number
 * @handler: dma handler
 * @error_handler: dma error handler
 * @arg: argument to pass back
 *
 * Returns 0 if success or a negative value if failure
 */
int set_edma_callback(int channel,
		edma_irq_handler handler,
		edma_error_handler error_handler,
		void *arg);
  
/* Free eDMA channel
 *  channel - eDMA TCD number
 *  dev - device
 */
int free_edma_channel(int channel, void *dev);

/*
 * DMA Modes
 */
#define	DMA_MODE_READ		0
#define DMA_MODE_WRITE		1
#endif

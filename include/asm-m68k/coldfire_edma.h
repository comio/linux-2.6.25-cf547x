#ifndef _LINUX_COLDFIRE_DMA_H
#define _LINUX_COLDFIRE_DMA_H

#include <linux/interrupt.h>

#define EDMA_DRIVER_NAME 		"ColdFire-eDMA"
#define DMA_DEV_MINOR 			1

#define EDMA_INT_CHANNEL_BASE 		8
#define EDMA_INT_CONTROLLER_BASE 	64
#define EDMA_CHANNELS			16

#define EDMA_IRQ_LEVEL			5

typedef irqreturn_t (*edma_irq_handler)(int, void *);
typedef void (*edma_error_handler)(int, void *);

void set_edma_params(int channel, u32 source, u32 dest,
	u32 attr, u32 soff, u32 nbytes, u32 slast,
	u32 citer, u32 biter, u32 doff, u32 dlast_sga);

void start_edma_transfer(int channel, int major_int);

void stop_edma_transfer(int channel);

void confirm_edma_interrupt_handled(int channel);

void init_edma(void);

int  request_edma_channel(int channel,
			edma_irq_handler handler,
			edma_error_handler error_handler,
			void *dev,
			spinlock_t *lock,
			const char *device_id);

int free_edma_channel(int channel, void *dev);

#endif

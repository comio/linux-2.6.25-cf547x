#ifndef _M68K_DMA_H
#define _M68K_DMA_H 1

/* it's useless on the m68k, but unfortunately needed by the new
   bootmem allocator (but this should do it for this) */
#define MAX_DMA_ADDRESS PAGE_OFFSET

#ifndef CONFIG_COLDFIRE
#define MAX_DMA_CHANNELS 8

extern int request_dma(unsigned int dmanr, const char * device_id);	/* reserve a DMA channel */
extern void free_dma(unsigned int dmanr);	/* release it again */

#else /* not (defined(CONFIG_MCF5474) || defined(CONFIG_MCF5484) || defined(CONFIG_MCF5475) || defined(CONFIG_MCF5485)) */

/************************************************
 * 	Multichannel DMA definitions		*
 ************************************************/
#ifdef CONFIG_MCD_DMA
#include <asm/MCD_dma.h>
#include <asm/m5485dma.h>

struct scatterlist;

#define MAX_DMA_CHANNELS NCHANNELS
/*
 *  identifiers for each initiator/requestor
 */
#define DMA_ALWAYS      (0)
#define DMA_DSPI_RX     (1)
#define DMA_DSPI_TX     (2)
#define DMA_DREQ0       (3)
#define DMA_PSC0_RX     (4)
#define DMA_PSC0_TX     (5)
#define DMA_USBEP0      (6)
#define DMA_USBEP1      (7)
#define DMA_USBEP2      (8)
#define DMA_USBEP3      (9)
#define DMA_PCI_TX      (10)
#define DMA_PCI_RX      (11)
#define DMA_PSC1_RX     (12)
#define DMA_PSC1_TX     (13)
#define DMA_I2C_RX      (14)
#define DMA_I2C_TX      (15)
#define DMA_FEC0_RX     (16)
#define DMA_FEC0_TX     (17)
#define DMA_FEC1_RX     (18)
#define DMA_FEC1_TX     (19)
#define DMA_DREQ1       (20)
#define DMA_CTM0        (21)
#define DMA_CTM1        (22)
#define DMA_CTM2        (23)
#define DMA_CTM3        (24)
#define DMA_CTM4        (25)
#define DMA_CTM5        (26)
#define DMA_CTM6        (27)
#define DMA_CTM7        (28)
#define DMA_USBEP4      (29)
#define DMA_USBEP5      (30)
#define DMA_USBEP6      (31)
#define DMA_PSC2_RX     (32)
#define DMA_PSC2_TX     (33)
#define DMA_PSC3_RX     (34)
#define DMA_PSC3_TX     (35)
#define DMA_FEC_RX(x)   ((x == 0) ? DMA_FEC0_RX : DMA_FEC1_RX)
#define DMA_FEC_TX(x)   ((x == 0) ? DMA_FEC0_TX : DMA_FEC1_TX)

int  dma_set_initiator(int);
unsigned int dma_get_initiator(int);
void dma_remove_initiator(int);
int dma_set_channel(int);
int dma_get_channel(int);
void dma_remove_channel(int);
int dma_set_channel_fec(int requestor);
int dma_connect(int channel, int address);
int dma_disconnect(int channel);
void dma_remove_channel_by_number(int channel);
int dma_init(void);

#endif /* CONFIG_MCD_DMA */

extern spinlock_t dma_spin_lock;

static __inline__ unsigned long claim_dma_lock(void)
{
	unsigned long flags;
	spin_lock_irqsave(&dma_spin_lock, flags);
	return flags;
}

static __inline__ void release_dma_lock(unsigned long flags)
{
	spin_unlock_irqrestore(&dma_spin_lock, flags);
}


/*
 *  Linux standard DMA stuff
 */
#if 0
int  request_dma(unsigned int channel, const char * device_id);
void free_dma(unsigned int channel);
void enable_dma(unsigned int channel);
void disable_dma(unsigned int channel);
int  dma_channel_active(unsigned int channel);
void set_dma_sg(unsigned int channel, struct scatterlist *sg, int nr_sg);
void set_dma_page(unsigned int channel, char pagenr);
void set_dma_addr(unsigned int channel, unsigned long physaddr);
void set_dma_count(unsigned int channel, unsigned long count);
void set_dma_mode(unsigned int channel, unsigned int mode);
void set_dma_speed(unsigned int channel, int cycle_ns);
int  get_dma_residue(unsigned int channel);
#endif
#define clear_dma_ff(channel)

#endif /* not (defined(CONFIG_MCF5474) || defined(CONFIG_MCF5484) || defined(CONFIG_MCF5475) || defined(CONFIG_MCF5485)) */

#ifdef CONFIG_PCI
extern int isa_dma_bridge_buggy;
#else
#define isa_dma_bridge_buggy    (0)
#endif

#endif /* _M68K_DMA_H */

/*
 * crypto/ocf/talitos/talitos.c
 *
 * An OCF-Linux module that uses Freescale's SEC to do the crypto.
 * Based on crypto/ocf/hifn and crypto/ocf/safe OCF drivers
 *
 * Copyright (c) 2006 Freescale Semiconductor, Inc.
 *
 * This code written by Kim A. B. Phillips <kim.phillips@freescale.com>
 * some code copied from files with the following:
 * Copyright (C) 2004 David McCullough <davidm@snapgear.com>

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ---------------------------------------------------------------------------
 *
 * NOTES:
 *
 * The Freescale SEC (also known as 'talitos') resides on the
 * internal bus, and runs asynchronous to the processor core.  It has
 * a wide gamut of cryptographic acceleration features, including single-
 * pass IPsec (also known as algorithm chaining).  To properly utilize 
 * all of the SEC's performance enhancing features, further reworking 
 * of higher level code (framework, applications) will be necessary.
 *
 * The following table shows which SEC version is present in which devices:
 * 
 * Devices       SEC version
 *
 * 8272, 8248    SEC 1.0
 * 885, 875      SEC 1.2
 * 8555E, 8541E  SEC 2.0
 * 8349E         SEC 2.01
 * 8548E         SEC 2.1
 *
 * The following table shows the features offered by each SEC version:
 *
 * 	                       Max.   chan-
 * version  Bus I/F       Clock  nels  DEU AESU AFEU MDEU PKEU RNG KEU
 *
 * SEC 1.0  internal 64b  100MHz   4     1    1    1    1    1   1   0
 * SEC 1.2  internal 32b   66MHz   1     1    1    0    1    0   0   0
 * SEC 2.0  internal 64b  166MHz   4     1    1    1    1    1   1   0
 * SEC 2.01 internal 64b  166MHz   4     1    1    1    1    1   1   0
 * SEC 2.1  internal 64b  333MHz   4     1    1    1    1    1   1   1
 *
 * Each execution unit in the SEC has two modes of execution; channel and
 * slave/debug.  This driver employs the channel infrastructure in the
 * device for convenience.  Only the RNG is directly accessed due to the
 * convenience of its random fifo pool.  The relationship between the
 * channels and execution units is depicted in the following diagram:
 *
 *    -------   ------------
 * ---| ch0 |---|          |
 *    -------   |          |
 *              |          |------+-------+-------+-------+------------
 *    -------   |          |      |       |       |       |           |
 * ---| ch1 |---|          |      |       |       |       |           |
 *    -------   |          |   ------  ------  ------  ------      ------
 *              |controller|   |DEU |  |AESU|  |MDEU|  |PKEU| ...  |RNG |
 *    -------   |          |   ------  ------  ------  ------      ------
 * ---| ch2 |---|          |      |       |       |       |           |
 *    -------   |          |      |       |       |       |           |
 *              |          |------+-------+-------+-------+------------
 *    -------   |          |
 * ---| ch3 |---|          |
 *    -------   ------------
 *
 * Channel ch0 may drive an aes operation to the aes unit (AESU),
 * and, at the same time, ch1 may drive a message digest operation
 * to the mdeu. Each channel has an input descriptor FIFO, and the 
 * FIFO can contain, e.g. on the 8541E, up to 24 entries, before a
 * a buffer overrun error is triggered. The controller is responsible
 * for fetching the data from descriptor pointers, and passing the 
 * data to the appropriate EUs. The controller also writes the 
 * cryptographic operation's result to memory. The SEC notifies 
 * completion by triggering an interrupt and/or setting the 1st byte 
 * of the hdr field to 0xff.
 *
 * TODO:
 * o support more algorithms
 * o support more versions of the SEC
 * o add support for linux 2.4
 * o scatter-gather (sg) support
 * o add support for public key ops (PKEU)
 * o add statistics
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/random.h>
#include <linux/skbuff.h>
//#include <linux/tracer.h> /* AK : tracing for perf */
#include <asm/scatterlist.h>
#include <linux/dma-mapping.h>  /* dma_map_single() */
#include <linux/moduleparam.h>
#include <linux/uio.h>


#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
#include <linux/platform_device.h>
#endif
#if 0
#include <crypto/cryptodev.h>
#endif
#if 1
#include <opencrypto/crypto.h>
#include <opencrypto/cryptodev.h>
#endif

#define DRV_NAME "talitos" 

#include "talitos_dev.h"
#include "talitos_soft.h"
#include <asm/io.h>
#include <linux/mm.h>
#include <asm/m5485sec.h>
#include <asm/m5485sim.h>
#include <asm/coldfire.h>
#define read_random(p,l) get_random_bytes(p,l)

const char talitos_driver_name[] = "Talitos OCF";
const char talitos_driver_version[] = "0.1";

static int talitos_process(void *, struct cryptop *, int);
static int talitos_newsession(void *, u_int32_t *, struct cryptoini *);
static int talitos_freesession(void *, u_int64_t);
static void dump_talitos_status(struct talitos_softc *sc);
static void skb_copy_bits_back(struct sk_buff *skb, int offset, caddr_t cp,
								 int len);
static int talitos_read_random(void *arg, u_int32_t *buf, int maxwords);
static void talitos_rng_init(struct talitos_softc *sc);
static int talitos_newsession(void *arg, u_int32_t *sidp, 
							struct cryptoini *cri);
static int talitos_submit(struct talitos_softc *sc, struct talitos_desc *td, 
								int chsel);
static int talitos_process(void *arg, struct cryptop *crp, int hint);
static void talitos_init_device(struct talitos_softc *sc);
static void talitos_reset_device_master(struct talitos_softc *sc);
static void talitos_reset_device(struct talitos_softc *sc);


static void talitos_doneprocessing(struct talitos_softc *sc, unsigned long chnum);
static void talitos_errorprocessing(struct talitos_softc *sc, unsigned long chnum);
static int talitos_probe(struct platform_device *pdev);
static int talitos_remove(struct platform_device *pdev);

#ifdef TALITOS_TASKLET
static inline int talitos_inline_polling (unsigned long lock_flags);
static inline void talitos_poll (unsigned long lock_flags);
static ocf_iomem_t sec_base_addr;
static u32 chnum = 0;
static u32 fifo_num = 0;
static void talitos_tasklet (unsigned long data);	  
DECLARE_TASKLET(isr_talitos_tasklet, talitos_tasklet, 0);
static LIST_HEAD(talitos_tasklet_q);
static spinlock_t talitos_tasklet_q_lock;
#define	TALITOS_TASKLET_Q_LOCK() \
			({ \
				spin_lock_irqsave(&talitos_tasklet_q_lock, r_flags); \
				dprintk("%s,%d: RETQ_LOCK\n", __FILE__, __LINE__); \
			 })
#define	TALITOS_TASKLET_Q_UNLOCK() \
			({ \
			 	dprintk("%s,%d: RETQ_UNLOCK\n", __FILE__, __LINE__); \
				spin_unlock_irqrestore(&talitos_tasklet_q_lock, r_flags); \
			 })

#endif /* TALITOS_TASKLET */


static int debug = 1;

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Enable debug");
static int coldfire_debug = 0;

static inline void talitos_write(volatile unsigned *addr, u32 val)
{
#ifdef FSL_SEC11_MCF547X_8X
	*(volatile unsigned long *)((unsigned long)addr) = val;
#else
        out_be32(addr, val);
#endif
}

static inline u32 talitos_read(volatile unsigned *addr)
{
        u32 val;
#ifdef FSL_SEC11_MCF547X_8X
	val = *(volatile unsigned long *)((unsigned long)addr);
#else
        val = in_be32(addr);
#endif
        return val;
}
		
static void dump_talitos_status(struct talitos_softc *sc)
{
	unsigned int v, v_hi, i, *ptr;
#ifdef FSL_SEC11_MCF547X_8X
        v = talitos_read(sc->sc_base_addr + TALITOS_EUACR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_EUACR_HI);
        printk(KERN_INFO DRV_NAME ": EUACR          0x%08x_%08x\n", v, v_hi);
        v = talitos_read(sc->sc_base_addr + TALITOS_EUASR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_EUASR_HI);
        printk(KERN_INFO DRV_NAME ": EUASR          0x%08x_%08x\n", v, v_hi);

        v = talitos_read(sc->sc_base_addr + TALITOS_DEURCR);
        printk(KERN_INFO DRV_NAME ": DEU  Reset 0x%08x\n", v);
        v = talitos_read(sc->sc_base_addr + TALITOS_DEUSR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_DEUSR_HI);
        printk(KERN_INFO DRV_NAME ": DEU  Status 0x%08x_%08x\n", v, v_hi);
        v = talitos_read(sc->sc_base_addr + TALITOS_DEUISR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_DEUISR_HI);
        printk(KERN_INFO DRV_NAME ": DEU  ISR 0x%08x_%08x\n", v, v_hi);
        v = talitos_read(sc->sc_base_addr + TALITOS_DEUICR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_DEUICR_HI);
        printk(KERN_INFO DRV_NAME ": DEU  ICR 0x%08x_%08x\n", v, v_hi);

        v = talitos_read(sc->sc_base_addr + TALITOS_AESURCR);
        printk(KERN_INFO DRV_NAME ": AESU  Reset 0x%08x\n", v);
        v = talitos_read(sc->sc_base_addr + TALITOS_AESUSR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_AESUSR_HI);
        printk(KERN_INFO DRV_NAME ": AESU  Status 0x%08x_%08x\n", v, v_hi);
        v = talitos_read(sc->sc_base_addr + TALITOS_AESUISR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_AESUISR_HI);
        printk(KERN_INFO DRV_NAME ": AESU  ISR 0x%08x_%08x\n", v, v_hi);
        v = talitos_read(sc->sc_base_addr + TALITOS_AESUICR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_AESUICR_HI);
        printk(KERN_INFO DRV_NAME ": AESU  ICR 0x%08x_%08x\n", v, v_hi);

        v = talitos_read(sc->sc_base_addr + TALITOS_MDEURCR);
        printk(KERN_INFO DRV_NAME ": MDEU  Reset 0x%08x\n", v);
        v = talitos_read(sc->sc_base_addr + TALITOS_MDEUSR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_MDEUSR_HI);
        printk(KERN_INFO DRV_NAME ": MDEU  Status 0x%08x_%08x\n", v, v_hi);
        v = talitos_read(sc->sc_base_addr + TALITOS_MDEUISR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_MDEUISR_HI);
        printk(KERN_INFO DRV_NAME ": MDEU  ISR 0x%08x_%08x\n", v, v_hi);
        v = talitos_read(sc->sc_base_addr + TALITOS_MDEUICR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_MDEUICR_HI);
        printk(KERN_INFO DRV_NAME ": MDEU  ICR 0x%08x_%08x\n", v, v_hi);

        v = talitos_read(sc->sc_base_addr + TALITOS_AFEURCR);
        printk(KERN_INFO DRV_NAME ": AFEU  Reset 0x%08x\n", v);
        v = talitos_read(sc->sc_base_addr + TALITOS_AFEUSR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_AFEUSR_HI);
        printk(KERN_INFO DRV_NAME ": AFEU  Status 0x%08x_%08x\n", v, v_hi);
        v = talitos_read(sc->sc_base_addr + TALITOS_AFEUISR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_AFEUISR_HI);
        printk(KERN_INFO DRV_NAME ": AFEU  ISR 0x%08x_%08x\n", v, v_hi);
        v = talitos_read(sc->sc_base_addr + TALITOS_AFEUICR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_AFEUICR_HI);
        printk(KERN_INFO DRV_NAME ": AFEU  ICR 0x%08x_%08x\n", v, v_hi);

        v = talitos_read(sc->sc_base_addr + TALITOS_RNGRCR);
        printk(KERN_INFO DRV_NAME ": RNG  Reset 0x%08x\n", v);
        v = talitos_read(sc->sc_base_addr + TALITOS_RNGSR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_RNGSR_HI);
        printk(KERN_INFO DRV_NAME ": RNG  Status 0x%08x_%08x\n", v, v_hi);
        v = talitos_read(sc->sc_base_addr + TALITOS_RNGISR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_RNGISR_HI);
        printk(KERN_INFO DRV_NAME ": RNG  ISR 0x%08x_%08x\n", v, v_hi);
        v = talitos_read(sc->sc_base_addr + TALITOS_RNGICR);
        v_hi = talitos_read(sc->sc_base_addr + TALITOS_RNGICR_HI);
        printk(KERN_INFO DRV_NAME ": RNG  ICR 0x%08x_%08x\n", v, v_hi);
#endif
	v = talitos_read(sc->sc_base_addr + TALITOS_MCR);
	v_hi = talitos_read(sc->sc_base_addr + TALITOS_MCR_HI);
	printk(KERN_INFO DRV_NAME ": MCR          0x%08x_%08x\n", v, v_hi);
	v = talitos_read(sc->sc_base_addr + TALITOS_IMR);
	v_hi = talitos_read(sc->sc_base_addr + TALITOS_IMR_HI);
	printk(KERN_INFO DRV_NAME ": IMR          0x%08x_%08x\n", v, v_hi);
	v = talitos_read(sc->sc_base_addr + TALITOS_ISR);
	v_hi = talitos_read(sc->sc_base_addr + TALITOS_ISR_HI);
	printk(KERN_INFO DRV_NAME ": ISR          0x%08x_%08x\n", v, v_hi);
	for (i = 0; i < sc->sc_num_channels; i++) { 
		v = talitos_read(sc->sc_base_addr + i*TALITOS_CH_OFFSET + 
			TALITOS_CH_CDPR);
		v_hi = talitos_read(sc->sc_base_addr + i*TALITOS_CH_OFFSET + 
			TALITOS_CH_CDPR_HI);
		printk(KERN_INFO DRV_NAME ": CDPR     ch%d 0x%08x_%08x\n", 
			i, v, v_hi);
	}
	for (i = 0; i < sc->sc_num_channels; i++) { 
		v = talitos_read(sc->sc_base_addr + i*TALITOS_CH_OFFSET + 
			TALITOS_CH_CCPSR);
		v_hi = talitos_read(sc->sc_base_addr + i*TALITOS_CH_OFFSET + 
			TALITOS_CH_CCPSR_HI);
		printk(KERN_INFO DRV_NAME ": CCPSR    ch%d 0x%08x_%08x\n", 
			i, v, v_hi);
	}
	ptr = sc->sc_base_addr + TALITOS_CH_DESCBUF;
	v = talitos_read(ptr++);
	 	printk(KERN_INFO DRV_NAME 
                        ": DESCBUF  ch0 0x%08x (tdp%02d)\n",v);
	for (i = 0; i < 15; i++) { 
		v = talitos_read(ptr++); v_hi = talitos_read(ptr++);
		printk(KERN_INFO DRV_NAME 
			": DESCBUF  ch0 0x%08x_%08x (tdp%02d)\n", 
			v, v_hi, i);
	}
	return;

}


/* taken from crypto/ocf/safe/safe.c driver */
static void
skb_copy_bits_back(struct sk_buff *skb, int offset, caddr_t cp, int len)
{
	int i;
	/*printk("%s\n",__FUNCTION__);*/
	if (offset < skb_headlen(skb)) {
		memcpy(skb->data + offset, cp, 
			min_t(int, skb_headlen(skb), len));
		len -= skb_headlen(skb);
		cp += skb_headlen(skb);
	}
	offset -= skb_headlen(skb);
	for (i = 0; len > 0 && i < skb_shinfo(skb)->nr_frags; i++) {
		if (offset < skb_shinfo(skb)->frags[i].size) {
			memcpy(page_address(skb_shinfo(skb)->frags[i].page) +
					skb_shinfo(skb)->frags[i].page_offset,
					cp, min_t(int, 
					skb_shinfo(skb)->frags[i].size, len));
			len -= skb_shinfo(skb)->frags[i].size;
			cp += skb_shinfo(skb)->frags[i].size;
		}
		offset -= skb_shinfo(skb)->frags[i].size;
	}
}

/* 
 * pull random numbers off the RNG FIFO, not exceeding amount available
 */
static int
talitos_read_random(void *arg, u_int32_t *buf, int maxwords)
{
	struct talitos_softc *sc = (struct talitos_softc *) arg;
	int rc;
	u_int32_t v;

#ifdef FSL_SEC11_MCF547X_8X
	struct talitos_desc *td;
	int out_fifo = 4;
	unsigned long r_flags;
	int maxlen = 0, maxleft = 0;

	td = &sc->sc_chnfifo[0][0].cf_desc;
	memset(td,0, sizeof(*td));
	memset((unsigned char*)buf,0, maxwords*4);

	rc = 0 ;
	if (maxwords <= 0){
        	printk("%s error:len %x < 0\n",__FUNCTION__,maxwords);
		return rc;
	}
	
	maxwords = maxwords * 4;
	if(maxwords % 4 != 0){
		/*printk("%s error:len %x is not mutiple of 4\n",__FUNCTION__,maxwords);*/
		maxleft = maxwords % 4;
		maxlen  = maxwords - maxleft;
		/*
		printk("%s error:len %x is not mutiple of 4. left %x len %x\n"
			,__FUNCTION__,maxwords,maxleft,maxlen); 
		*/
	}
        
        /*
 	* OFL is number of available 64-bit words, 
 	* shift and convert to a 32-bit word count
 	*/
        
	/*printk("%s SR %x. maxwords %x\n",__FUNCTION__,v,maxwords);*/

        td->ptr[out_fifo].ptr = dma_map_single(NULL, (unsigned char *)buf,
                        	maxwords, DMA_TO_DEVICE);
        td->ptr[out_fifo].len = maxwords;
	
	td->hdr = 0x40000010; 
	if (coldfire_debug)
		dump_talitos_status(sc);

	talitos_submit(sc,td,1);
	rc = maxwords;
	if (coldfire_debug)
		dump_talitos_status(sc);
        //#ifdef TALITOS_TASKLET  
        //TALITOS_TASKLET_Q_LOCK();       
        //list_add_tail(&sc->sc_chnfifo[0][0].desc_list , &talitos_tasklet_q);        
        //talitos_inline_polling(r_flags);
        //TALITOS_TASKLET_Q_UNLOCK();
	//while(1){};
	//#endif
#else
	/* check for things like FIFO underflow */
	v = talitos_read(sc->sc_base_addr + TALITOS_RNGISR_HI);
	if (unlikely(v)) {
		printk(KERN_ERR DRV_NAME ": RNGISR_HI error %08x\n", v);
		return 0;
	}
	/*
	 * OFL is number of available 64-bit words, 
	 * shift and convert to a 32-bit word count
	 */
	v = talitos_read(sc->sc_base_addr + TALITOS_RNGSR_HI);
	v = (v & TALITOS_RNGSR_HI_OFL) >> (16 - 1);
	if (maxwords > v)
		maxwords = v;
	for (rc = 0; rc < maxwords; rc++) {
		buf[rc] = talitos_read(sc->sc_base_addr + 
			TALITOS_RNG_FIFO + rc*sizeof(u_int32_t));
	}
	if (maxwords & 1) {
		/* 
		 * RNG will complain with an AE in the RNGISR
		 * if we don't complete the pairs of 32-bit reads
		 * to its 64-bit register based FIFO
		 */
		v = talitos_read(sc->sc_base_addr + 
			TALITOS_RNG_FIFO + rc*sizeof(u_int32_t));
	}
#endif
	return rc;
}

static void
talitos_rng_init(struct talitos_softc *sc)
{
	u_int32_t v;
#ifdef FSL_SEC11_MCF547X_8X
	unsigned long time = jiffies;
#endif
	DPRINTF("%s()\n", __FUNCTION__);
	/* reset RNG EU */
#ifdef FSL_SEC11_MCF547X_8X
        v = talitos_read(sc->sc_base_addr + TALITOS_RNGRCR);
        v |= TALITOS_RNGRCR_SR;
        talitos_write(sc->sc_base_addr + TALITOS_RNGRCR, v);
        while ((talitos_read(sc->sc_base_addr + TALITOS_RNGSR)
                & TALITOS_RNGSR_RD) == 0){
                        /*cpu_relax();*/ /* This causes CPU lockup!!! */
                if(jiffies - time > SEC_INIT_TIMEOUT){
                        printk("%s fail, timeout\n",__FUNCTION__);
                        return;
                }
        }
        /*
	* we tell the RNG to start filling the RNG FIFO
 	* by writing the RNGDSR 
 	*/
        v = talitos_read(sc->sc_base_addr + TALITOS_RNGDSR);
        talitos_write(sc->sc_base_addr + TALITOS_RNGDSR, v);
        /*
 	* 64 bits of data will be pushed onto the FIFO every 
	* 256 SEC cycles until the FIFO is full.  The RNG then 
 	* attempts to keep the FIFO full.
 	*/
        v = talitos_read(sc->sc_base_addr + TALITOS_RNGISR);
        if (v) {
                printk(KERN_ERR DRV_NAME ": RNGISR_HI error %08x\n", v);
                return;
        }

#else
	v = talitos_read(sc->sc_base_addr + TALITOS_RNGRCR_HI);
	v |= TALITOS_RNGRCR_HI_SR;
	talitos_write(sc->sc_base_addr + TALITOS_RNGRCR_HI, v);
	while ((talitos_read(sc->sc_base_addr + TALITOS_RNGSR_HI) 
		& TALITOS_RNGSR_HI_RD) == 0){
			cpu_relax(); /* This causes CPU lockup!!! */
	}
	/*
	 * we tell the RNG to start filling the RNG FIFO
	 * by writing the RNGDSR 
	 */
	v = talitos_read(sc->sc_base_addr + TALITOS_RNGDSR_HI);
	talitos_write(sc->sc_base_addr + TALITOS_RNGDSR_HI, v);
	/*
	 * 64 bits of data will be pushed onto the FIFO every 
	 * 256 SEC cycles until the FIFO is full.  The RNG then 
	 * attempts to keep the FIFO full.
	 */
	v = talitos_read(sc->sc_base_addr + TALITOS_RNGISR_HI);
	if (v) {
		printk(KERN_ERR DRV_NAME ": RNGISR_HI error %08x\n", v);
		return;
	}
#endif
	/*
	 * n.b. we need to add a FIPS test here - if the RNG is going 
	 * to fail, it's going to fail at reset time
	 */
	DPRINTF("%s success\n", __FUNCTION__);
	return;
}

/*
 * Generate a new software session.
 */
static int
talitos_newsession(void *arg, u_int32_t *sidp, struct cryptoini *cri)
{
	struct cryptoini *c, *encini = NULL, *macini = NULL;
	struct talitos_softc *sc = arg;
	struct talitos_session *ses = NULL;
	int sesn;

	DPRINTF("%s()\n", __FUNCTION__);
	if (sidp == NULL || cri == NULL || sc == NULL) {
		DPRINTF("%s,%d - EINVAL\n", __FILE__, __LINE__);
		return EINVAL;
	}

	for (c = cri; c != NULL; c = c->cri_next) {
#ifdef FSL_SEC11_MCF547X_8X
		if(coldfire_debug)
			DPRINTF("%s: c->cri_alg %x\n", 
				__FUNCTION__,c->cri_alg);
#endif
		if (c->cri_alg == CRYPTO_MD5 ||
		    c->cri_alg == CRYPTO_MD5_HMAC ||
		    c->cri_alg == CRYPTO_SHA1 ||
		    c->cri_alg == CRYPTO_SHA1_HMAC ||
		    c->cri_alg == CRYPTO_NULL_HMAC) {
			if (macini)
#ifdef FSL_SEC11_MCF547X_8X
                        {
				if(coldfire_debug)
					DPRINTF("%s: macini =0\n",
                                		__FUNCTION__);
				return EINVAL;
			}
#else
				return EINVAL;
#endif
			macini = c;
		} else if (c->cri_alg == CRYPTO_DES_CBC ||
			c->cri_alg == CRYPTO_3DES_CBC ||
			c->cri_alg == CRYPTO_AES_CBC ||
			c->cri_alg == CRYPTO_NULL_CBC ||
			c->cri_alg == CRYPTO_ARC4) {
			if (encini)
#ifdef FSL_SEC11_MCF547X_8X
                        {
				if(coldfire_debug)
                                	DPRINTF("%s: encini =0\n",
                                		__FUNCTION__);
				return EINVAL;
                        }
#else
				return EINVAL;
#endif
			encini = c;
		} else {
			DPRINTF("UNKNOWN c->cri_alg %d\n", encini->cri_alg);
			return EINVAL;
		}
	}

	if (encini == NULL && macini == NULL)
#ifdef FSL_SEC11_MCF547X_8X
        {      
		if(coldfire_debug)
			DPRINTF("%s encini NULL,macini NULL\n",
				__FUNCTION__);
		return EINVAL;
	}
#else
		return EINVAL;
#endif

	if (encini) {
#ifdef FSL_SEC11_MCF547X_8X
		if(coldfire_debug)
			DPRINTF("%s encini: cri_klen %x,cri_alg %x \n",
				__FUNCTION__,encini->cri_klen,
				encini->cri_alg);
#endif
		/* validate key length */
		switch (encini->cri_alg) {
		case CRYPTO_DES_CBC:
			if (encini->cri_klen != 64)
				return EINVAL;
			break;
		case CRYPTO_3DES_CBC:
			if (encini->cri_klen != 192) {
				return EINVAL;
			}
			break;
		case CRYPTO_AES_CBC:
			if (encini->cri_klen != 128 &&
			    encini->cri_klen != 192 &&
			    encini->cri_klen != 256)
				return EINVAL;
			break;
		case CRYPTO_ARC4:
			if ( ( encini->cri_klen < ARC4_SEC_MIN_KEY_SIZE*8 ) ||
			( encini->cri_klen > ARC4_SEC_MAX_KEY_SIZE*8 ) ) {
				printk("%s: ARC4 do not support the "
					"key size %d\n", __FUNCTION__, encini->cri_klen);
				return EINVAL;
			}
			else {
				/*printk("%s: ARC4 support klen %d", __FUNCTION__, encini->cri_klen);*/
				sc->sc_rc4_first = 1;
			}
			break;
		default:
			DPRINTF("UNKNOWN encini->cri_alg %d\n", 
				encini->cri_alg);
			return EINVAL;
		}
	}

	if (sc->sc_sessions == NULL) {
		ses = sc->sc_sessions = (struct talitos_session *)
			kmalloc(sizeof(struct talitos_session), GFP_ATOMIC);
		if (ses == NULL)
			return ENOMEM;
		memset(ses, 0, sizeof(struct talitos_session));
		sesn = 0;
		sc->sc_nsessions = 1;
	} else {
		for (sesn = 0; sesn < sc->sc_nsessions; sesn++) {
			if (sc->sc_sessions[sesn].ses_used == 0) {
				ses = &sc->sc_sessions[sesn];
				break;
			}
		}

		if (ses == NULL) {
			/* allocating session */
			sesn = sc->sc_nsessions;
			ses = (struct talitos_session *) kmalloc(
				(sesn + 1) * sizeof(struct talitos_session), 
				GFP_ATOMIC);
			if (ses == NULL)
				return ENOMEM;
			memset(ses, 0,
				(sesn + 1) * sizeof(struct talitos_session));
			memcpy(ses, sc->sc_sessions, 
				sesn * sizeof(struct talitos_session));
			memset(sc->sc_sessions, 0,
				sesn * sizeof(struct talitos_session));
			kfree(sc->sc_sessions);
			sc->sc_sessions = ses;
			ses = &sc->sc_sessions[sesn];
			sc->sc_nsessions++;
		}
	}

	ses->ses_used = 1;

	if (encini) {
		/* get an IV */
		/* XXX may read fewer than requested */
		read_random(ses->ses_iv, sizeof(ses->ses_iv));

		ses->ses_klen = (encini->cri_klen + 7) / 8;
#ifdef FSL_SEC11_MCF547X_8X
		if(coldfire_debug)
                	DPRINTF("%s encini: ses_klen %x,cri_klen %x,sizeof(ses->ses_iv) %x \n",
				__FUNCTION__,ses->ses_klen,encini->cri_klen,
				sizeof(ses->ses_iv));
#endif
		memcpy(ses->ses_key, encini->cri_key, ses->ses_klen);
		if (macini) {
			/* doing hash on top of cipher */
			ses->ses_hmac_len = (macini->cri_klen + 7) / 8;
			memcpy(ses->ses_hmac, macini->cri_key,
				ses->ses_hmac_len);
		}
	} else 
	if (macini) {
		/* doing hash */
		ses->ses_klen = (macini->cri_klen + 7) / 8;
		memcpy(ses->ses_key, macini->cri_key, ses->ses_klen);
	}
	/* really should make up a template td here, 
	 * and only fill things like i/o and direction in process() */

	/* assign session ID */
	*sidp = TALITOS_SID(sc->sc_num, sesn);
	if(coldfire_debug)
		DPRINTF("%s over: sc_num %x,sesn %x,*sip %x \n",
			__FUNCTION__,sc->sc_num,sesn,*sidp);
	return 0;
}

/*
 * Deallocate a session.
 */
static int
talitos_freesession(void *arg, u_int64_t tid)
{
	struct talitos_softc *sc = arg;
	int session, ret;
	u_int32_t sid = ((u_int32_t) tid) & 0xffffffff;

	if (sc == NULL)
		return EINVAL;
	session = TALITOS_SESSION(sid);
	if (session < sc->sc_nsessions) {
		memset(&sc->sc_sessions[session], 0,
			sizeof(sc->sc_sessions[session]));
		ret = 0;
	} else
		ret = EINVAL;
	return ret;
}

/*
 * launch device processing - it will come back with done notification 
 * in the form of an interrupt and/or HDR_DONE_BITS in header 
 */
static int 
talitos_submit(
	struct talitos_softc *sc,
	struct talitos_desc *td,
	int chsel)
{
	u_int32_t v;
#ifdef FSL_SEC11_MCF547X_8X
	/*printk("%s channel %x\n",__FUNCTION__,chsel);*/
        v = dma_map_single(NULL, td, sizeof(*td), DMA_TO_DEVICE);
        talitos_write(sc->sc_base_addr + 
                chsel*TALITOS_CH_OFFSET + TALITOS_CH_FF, v);
	if (coldfire_debug)
		dump_talitos_status(sc);
#else
	v = dma_map_single(NULL, td, sizeof(*td), DMA_TO_DEVICE);
	talitos_write(sc->sc_base_addr + 
		chsel*TALITOS_CH_OFFSET + TALITOS_CH_FF, 0);
	talitos_write(sc->sc_base_addr + 
		chsel*TALITOS_CH_OFFSET + TALITOS_CH_FF_HI, v);
#endif
	return 0;
}

static int
talitos_process(void *arg, struct cryptop *crp, int hint)
{
	int i,k, err = 0, ivsize;
	struct talitos_softc *sc = arg;
	struct cryptodesc *crd1, *crd2, *maccrd, *enccrd;
	caddr_t iv;
	struct talitos_session *ses;
	struct talitos_desc *td;
	int chsel; /* AK */
	/* descriptor mappings */
	int hmac_key, hmac_data, cipher_iv, cipher_key, 
		in_fifo, out_fifo, cipher_iv_out;
	unsigned long flags;
	unsigned long r_flags;
	int status; /* AK */
	int rc;
	u32 desc_header = 0 ; /* AK */

	if (crp == NULL || crp->crp_callback == NULL || sc == NULL) {
		return EINVAL;
	}
	crp->crp_etype = 0;
	if (TALITOS_SESSION(crp->crp_sid) >= sc->sc_nsessions) {
		return EINVAL;
	}

	ses = &sc->sc_sessions[TALITOS_SESSION(crp->crp_sid)];
	
  
	#ifdef TALITOS_TASKLET
	/* AK : obtain ch and fifo - TBD */
    	/* enter the channel scheduler */ 
	//spin_lock_irqsave(&sc->sc_chnfifolock[sc->sc_num_channels], flags);
	/* 
	 * Added by Vishnu
	 * Makes the code compatible over different SEC Revisions
	 * Get the Number of channels/FIFO for this SEC Revision
	 */
	chsel = chnum;
	i = fifo_num;
	//chsel = sc->sc_num_channels - 1;
	//i = sc->sc_chfifo_len - 1;
	fifo_num++;
	if (fifo_num > i ){
		fifo_num = 0;
		chnum++;
		if (chnum > chsel)
			chnum = 0;
	}
	
	/* release the channel scheduler lock */ 
	//spin_unlock_irqrestore(&sc->sc_chnfifolock[sc->sc_num_channels], flags);

	/*
	 * now check whether the desc. is free. set the done
	 * notification if free
	 */
	if (sc->sc_chnfifo[chsel][i].cf_desc.hdr == 0) {
		/* TBD - do we really need to clear everything out*/
		memset(&sc->sc_chnfifo[chsel][i].cf_desc,
				0, sizeof(*td));

	} else {
		/* process all backlogs */
		  for(;;) {
			TALITOS_TASKLET_Q_LOCK();	
 			rc = talitos_inline_polling(r_flags);
			TALITOS_TASKLET_Q_UNLOCK();
			printk(KERN_INFO "%s %d times checking Q",__FUNCTION__, k);
			if(rc) {
			  printk(KERN_INFO "empty\n");
				break;
			}
			printk("not empty processed\n");
			chnum = fifo_num = i = chsel = 0;			

		}
		/* check the channel+fifo again */
		if (sc->sc_chnfifo[chsel][i].cf_desc.hdr == 0) {
			memset(&sc->sc_chnfifo[chsel][i].cf_desc,
					0, sizeof(*td));
			sc->sc_chnfifo[chsel][i].cf_desc.hdr |= 
						TALITOS_DONE_NOTIFY;
		} else {
			printk(KERN_INFO "%s panic full - needs debug\n",__FUNCTION__);
		}
	}
       
	#endif /* TALITOS_TASKLET */			

	td = &sc->sc_chnfifo[chsel][i].cf_desc;
	sc->sc_chnfifo[chsel][i].cf_crp = crp;

	crd1 = crp->crp_desc;
	if (crd1 == NULL) {
		err = EINVAL;
		printk(KERN_INFO "%s crd1 null\n",__FUNCTION__);
		goto errout;
	}
	crd2 = crd1->crd_next;
	/*printk("crd1 %x , crd2 %x\n",crd1,crd2);*/
	/* prevent compiler warning */
	hmac_key = 0;
	hmac_data = 0;
	if (crd2 == NULL) {
		if(coldfire_debug)
			DPRINTF("%s crd2 = NULL, alg %x, desc_type %x\n",
				__FUNCTION__,crd1->crd_alg,sc->sc_desc_types);
		//td->hdr |= TD_TYPE_COMMON_NONSNOOP_NO_AFEU;
		desc_header |= TD_TYPE_COMMON_NONSNOOP_NO_AFEU;
		/* assign descriptor dword ptr mappings for this desc. type */
		cipher_iv = 1;
		cipher_key = 2;
		in_fifo = 3;
		cipher_iv_out = 5;
		if (crd1->crd_alg == CRYPTO_MD5_HMAC ||
		    crd1->crd_alg == CRYPTO_SHA1_HMAC ||
		    crd1->crd_alg == CRYPTO_SHA1 ||
		    crd1->crd_alg == CRYPTO_MD5) {
			out_fifo = 5;
			maccrd = crd1;
			enccrd = NULL;
		} else if (crd1->crd_alg == CRYPTO_DES_CBC ||
		    crd1->crd_alg == CRYPTO_3DES_CBC ||
		    crd1->crd_alg == CRYPTO_AES_CBC ||
		    crd1->crd_alg == CRYPTO_ARC4) {
			out_fifo = 4;
			maccrd = NULL;
			enccrd = crd1;
		} else {
			DPRINTF("UNKNOWN crd1->crd_alg %d\n", crd1->crd_alg);
			err = EINVAL;
			goto errout;
		}
	} else {
		DPRINTF("%s crd2 != NULL,alg %x, desc_type %x\n",
			__FUNCTION__,crd1->crd_alg,sc->sc_desc_types);
		if (sc->sc_desc_types & TALITOS_HAS_DT_IPSEC_ESP) {
			//td->hdr |= TD_TYPE_IPSEC_ESP;
			desc_header |=  TD_TYPE_IPSEC_ESP;
		} else {
			DPRINTF("unimplemented: multiple descriptor ipsec\n");
			err = EINVAL;
			goto errout;
		}
		/* assign descriptor dword ptr mappings for this desc. type */
		hmac_key = 0;
		hmac_data = 1;
		cipher_iv = 2;
		cipher_key = 3;
		in_fifo = 4;
		out_fifo = 5;
		cipher_iv_out = 6;
		if ((crd1->crd_alg == CRYPTO_MD5_HMAC ||
                     crd1->crd_alg == CRYPTO_SHA1_HMAC ||
                     crd1->crd_alg == CRYPTO_MD5 ||
                     crd1->crd_alg == CRYPTO_SHA1) &&
		    (crd2->crd_alg == CRYPTO_DES_CBC ||
		     crd2->crd_alg == CRYPTO_3DES_CBC ||
		     crd2->crd_alg == CRYPTO_AES_CBC ||
		     crd2->crd_alg == CRYPTO_ARC4) &&
		    ((crd2->crd_flags & CRD_F_ENCRYPT) == 0)) {
			maccrd = crd1;
			enccrd = crd2;
		} else if ((crd1->crd_alg == CRYPTO_DES_CBC ||
		     crd1->crd_alg == CRYPTO_ARC4 ||
		     crd1->crd_alg == CRYPTO_3DES_CBC ||
		     crd1->crd_alg == CRYPTO_AES_CBC) &&
		    (crd2->crd_alg == CRYPTO_MD5_HMAC ||
                     crd2->crd_alg == CRYPTO_SHA1_HMAC ||
                     crd2->crd_alg == CRYPTO_MD5 ||
                     crd2->crd_alg == CRYPTO_SHA1) &&
		    (crd1->crd_flags & CRD_F_ENCRYPT)) {
			enccrd = crd1;
			maccrd = crd2;
		} else {
			/* We cannot order the SEC as requested */
			printk(DRV_NAME ": cannot do the order\n");
			err = EINVAL;
			goto errout;
		}
	}

	/* assign in_fifo and out_fifo based on input/output struct type */
	if (crp->crp_flags & CRYPTO_F_SKBUF) {
		/* using SKB buffers */
		struct sk_buff *skb = (struct sk_buff *)crp->crp_buf;
		if (skb_shinfo(skb)->nr_frags) {
			printk(DRV_NAME ": skb frags unimplemented\n");
			err = EINVAL;
			goto errout;
		}
		td->ptr[in_fifo].ptr = dma_map_single(NULL, skb->data, 
			skb->len, DMA_TO_DEVICE);
		td->ptr[in_fifo].len = skb->len;
		td->ptr[out_fifo].ptr = dma_map_single(NULL, skb->data, 
			skb->len, DMA_TO_DEVICE);
		td->ptr[out_fifo].len = skb->len;
		td->ptr[hmac_data].ptr = dma_map_single(NULL, skb->data,
			skb->len, DMA_TO_DEVICE);
	} else if (crp->crp_flags & CRYPTO_F_IOV) {
		/* using IOV buffers */
		struct ocf_uio *uiop = (struct uio *)crp->crp_buf;
		if (uiop->uio_iovcnt > 1) {
			printk(DRV_NAME ": iov frags unimplemented\n");
			err = EINVAL;
			goto errout;
		}
		td->ptr[in_fifo].ptr = dma_map_single(NULL,
			uiop->uio_iov->iov_base, crp->crp_ilen, DMA_TO_DEVICE);
		td->ptr[in_fifo].len = crp->crp_ilen;
		if (crp->crp_mac) {
			td->ptr[out_fifo].ptr = dma_map_single(NULL,
				crp->crp_mac, ses->ses_klen, DMA_TO_DEVICE);
			td->ptr[out_fifo].len = ses->ses_klen;
		} else {
			/* crp_olen is never set; always use crp_ilen */
			td->ptr[out_fifo].ptr = dma_map_single(NULL,
				uiop->uio_iov->iov_base,
				crp->crp_ilen, DMA_TO_DEVICE);
			td->ptr[out_fifo].len = crp->crp_ilen;
		}
	} else {
		/* using contig buffers */
		td->ptr[in_fifo].ptr = dma_map_single(NULL,
			crp->crp_buf, crp->crp_ilen, DMA_TO_DEVICE);
		//td->ptr[in_fifo].ptr =	virt_to_phys(crp->crp_buf);
		td->ptr[in_fifo].len = crp->crp_ilen;
		td->ptr[out_fifo].ptr = dma_map_single(NULL,
			crp->crp_buf, crp->crp_ilen, DMA_TO_DEVICE);
		//td->ptr[out_fifo].ptr =  virt_to_phys(crp->crp_buf);
		td->ptr[out_fifo].len = crp->crp_ilen;
	}

	if (enccrd) {
		switch (enccrd->crd_alg) {
		case CRYPTO_3DES_CBC:
		  	//td->hdr |= TALITOS_MODE0_DEU_3DES;
			desc_header |= TALITOS_MODE0_DEU_3DES;
			/* FALLTHROUGH */
		case CRYPTO_DES_CBC:
		  	//td->hdr |= TALITOS_SEL0_DEU
		  	//		|  TALITOS_MODE0_DEU_CBC;
			desc_header |= TALITOS_SEL0_DEU
				|  TALITOS_MODE0_DEU_CBC;
			if (enccrd->crd_flags & CRD_F_ENCRYPT)
				//td->hdr |= TALITOS_MODE0_DEU_ENC;
				desc_header |= TALITOS_MODE0_DEU_ENC;
			ivsize = 2*sizeof(u_int32_t);
			DPRINTF("%cDES ses %d ch %d len %d\n",
				(/*td->hdr*/desc_header & TALITOS_MODE0_DEU_3DES)?'3':'1',
				(u32)TALITOS_SESSION(crp->crp_sid),
				chsel, td->ptr[in_fifo].len);
			break;
		case CRYPTO_AES_CBC:
		  	//td->hdr |= TALITOS_SEL0_AESU
		  	//		|  TALITOS_MODE0_AESU_CBC;
			desc_header |= TALITOS_SEL0_AESU
				|  TALITOS_MODE0_AESU_CBC;
			if (enccrd->crd_flags & CRD_F_ENCRYPT)
			  	//td->hdr |= TALITOS_MODE0_AESU_ENC;
				desc_header |= TALITOS_MODE0_AESU_ENC;
			ivsize = 4*sizeof(u_int32_t);
			DPRINTF("AES  ses %d ch %d len %d\n",
				(u32)TALITOS_SESSION(crp->crp_sid),
				chsel, td->ptr[in_fifo].len);
			break;
		case CRYPTO_ARC4:
			if(coldfire_debug)
				printk("flags %x\n", enccrd->crd_flags);
			if ((enccrd->crd_flags & CRD_ARC4_FIRST) == CRD_ARC4_FIRST) {
				sc->sc_rc4_first = 1;
				if(coldfire_debug)
					printk("ARC4 first\n");
			}
			else {
				sc->sc_rc4_first = 0;
			}

			if (sc->sc_rc4_first == 1) {
				desc_header = SEC_ALG_AFEU_KEY ;
			}
			else if (sc->sc_rc4_first == 0) {
				desc_header = SEC_ALG_AFEU_CONTEXT;
			}
			else {
				printk("%s: the ARC4 error,first 1\n", __FUNCTION__);
			}

			ivsize = ARC4_SEC_CONTEXT_LEN;
			if(coldfire_debug)
                        	DPRINTF("ARC4  ses %d ch %d len %d ivsize %d\n",
					(u32)TALITOS_SESSION(crp->crp_sid),
					chsel, td->ptr[in_fifo].len,ivsize);
			break;
		default:
			printk(DRV_NAME ": unimplemented enccrd->crd_alg %d\n",
				enccrd->crd_alg);
			err = EINVAL;
			goto errout;
		}
		/*
		 * Setup encrypt/decrypt state.  When using basic ops
		 * we can't use an inline IV because hash/crypt offset
		 * must be from the end of the IV to the start of the
		 * crypt data and this leaves out the preceding header
		 * from the hash calculation.  Instead we place the IV
		 * in the state record and set the hash/crypt offset to
		 * copy both the header+IV.
		 */
		if (enccrd->crd_flags & CRD_F_ENCRYPT) {
		  	//td->hdr |= TALITOS_DIR_OUTBOUND; 
		  	/*printk("enccrd->crd_flags & CRD_F_ENCRYPT \n");*/
			desc_header |= TALITOS_DIR_OUTBOUND; 
			if (enccrd->crd_flags & CRD_F_IV_EXPLICIT)
				iv = enccrd->crd_iv;
			else
				iv = (caddr_t) ses->ses_iv;
			if ((enccrd->crd_flags & CRD_F_IV_PRESENT) == 0) {
				if (crp->crp_flags & CRYPTO_F_SKBUF)     {
					skb_copy_bits_back(
						(struct sk_buff *)
						(crp->crp_buf),
						enccrd->crd_inject, 
						iv, ivsize);
				}
				else if (crp->crp_flags & CRYPTO_F_IOV)  {
					cuio_copyback((struct uio *)
						(crp->crp_buf),  
						enccrd->crd_inject, 
						ivsize, iv);
				}
			}
		} else {
			/*printk("enccrd->crd_flags ! CRD_F_ENCRYPT \n");*/
			//td->hdr |= TALITOS_DIR_INBOUND; 
			desc_header |= TALITOS_DIR_INBOUND; 
			if (enccrd->crd_flags & CRD_F_IV_EXPLICIT)
				iv = enccrd->crd_iv;
			else
				iv = (caddr_t) ses->ses_iv;
			if ((enccrd->crd_flags & CRD_F_IV_PRESENT) == 0) {
				if (crp->crp_flags & CRYPTO_F_SKBUF)     {
					skb_copy_bits((struct sk_buff *)
						(crp->crp_buf),
						enccrd->crd_inject, 
						iv, ivsize);
				}
				else if (crp->crp_flags & CRYPTO_F_IOV)  {
					cuio_copyback((struct uio *)
						(crp->crp_buf),
						enccrd->crd_inject,
						ivsize, iv);
				}
			}
		}

		if (enccrd->crd_alg == CRYPTO_ARC4) {
			/*ivsize = ARC4_SEC_CONTEXT_LEN*/
			if (sc->sc_rc4_first == 1) {
                        	td->ptr[cipher_iv].ptr = NULL;
                        	td->ptr[cipher_iv].len = 0;
			}
			else if (sc->sc_rc4_first == 0) {
			        
				td->ptr[cipher_iv].ptr = dma_map_single(NULL, iv, ivsize, 
                                	DMA_TO_DEVICE);
                        	//td->ptr[cipher_iv].ptr = virt_to_phys(iv);
				td->ptr[cipher_iv].len = ivsize;
			}
			else {
				printk("%s: the ARC4 error,first 2\n", __FUNCTION__);	
			}
                        td->ptr[cipher_iv_out].ptr = dma_map_single(NULL, iv, ivsize, 
                        	DMA_TO_DEVICE);
                        //td->ptr[cipher_iv_out].ptr = virt_to_phys(iv);
                        td->ptr[cipher_iv_out].len = ivsize;
		}
		else {
			td->ptr[cipher_iv].ptr = dma_map_single(NULL, iv, ivsize, 
				DMA_TO_DEVICE);
			//td->ptr[cipher_iv].ptr = virt_to_phys(iv);
			td->ptr[cipher_iv].len = ivsize;
			td->ptr[cipher_iv_out].ptr = dma_map_single(NULL, iv, ivsize, 
				DMA_TO_DEVICE);
			//td->ptr[cipher_iv_out].ptr = virt_to_phys(iv);
			td->ptr[cipher_iv_out].len = ivsize;
		}
	}

	if (enccrd && maccrd) {
		//int bypass, coffset, oplen; /* AK commented out, not used */
		/* this is ipsec only for now */
		//td->hdr |= TALITOS_SEL1_MDEU
		//	|  TALITOS_MODE1_MDEU_INIT
		//	|  TALITOS_MODE1_MDEU_PAD;
		desc_header |= TALITOS_SEL1_MDEU
			|  TALITOS_MODE1_MDEU_INIT
			|  TALITOS_MODE1_MDEU_PAD;
		switch (maccrd->crd_alg) {
			case	CRYPTO_MD5:	
			  	//td->hdr |= TALITOS_MODE1_MDEU_MD5;
				desc_header |= TALITOS_MODE1_MDEU_MD5;
				break;
			case	CRYPTO_MD5_HMAC:	
			  	//td->hdr |= TALITOS_MODE1_MDEU_MD5_HMAC;
				desc_header |= TALITOS_MODE1_MDEU_MD5_HMAC;
				break;
			case	CRYPTO_SHA1:	
			  	//td->hdr |= TALITOS_MODE1_MDEU_SHA1;
				desc_header |= TALITOS_MODE1_MDEU_SHA1;
				break;
			case	CRYPTO_SHA1_HMAC:	
			  	//td->hdr |= TALITOS_MODE1_MDEU_SHA1_HMAC;
				desc_header |= TALITOS_MODE1_MDEU_SHA1_HMAC;
				break;
			default:
				/* We cannot order the SEC as requested */
				printk(DRV_NAME ": cannot do the order\n");
				err = EINVAL;
				goto errout;
		}
		if ((maccrd->crd_alg == CRYPTO_MD5_HMAC) ||
		   (maccrd->crd_alg == CRYPTO_SHA1_HMAC)) {
			/*
			 * The offset from hash data to the start of
			 * crypt data is the difference in the skips.
			 */
			/* ipsec only for now */
			td->ptr[hmac_key].ptr = dma_map_single(NULL, 
				ses->ses_hmac, ses->ses_hmac_len, DMA_TO_DEVICE);
			td->ptr[hmac_key].len = ses->ses_hmac_len;
			td->ptr[in_fifo].ptr  += enccrd->crd_skip;
			td->ptr[in_fifo].len  =  enccrd->crd_len;
			td->ptr[out_fifo].ptr += enccrd->crd_skip;
			td->ptr[out_fifo].len =  enccrd->crd_len;
			/* bytes of HMAC to postpend to ciphertext */
			//td->ptr[out_fifo].extent =  12;	/* ipsec */
			td->ptr[hmac_data].ptr += maccrd->crd_skip; 
			td->ptr[hmac_data].len = enccrd->crd_skip - maccrd->crd_skip;
		}
		if (enccrd->crd_flags & CRD_F_KEY_EXPLICIT) {
			printk(DRV_NAME ": CRD_F_KEY_EXPLICIT unimplemented\n");
		}
	}

	if (!enccrd && maccrd) {
		/* single MD5 or SHA */
		//td->hdr |= TALITOS_SEL0_MDEU
		//		|  TALITOS_MODE0_MDEU_INIT
		//		|  TALITOS_MODE0_MDEU_PAD;
		desc_header |= TALITOS_SEL0_MDEU
				|  TALITOS_MODE0_MDEU_INIT
				|  TALITOS_MODE0_MDEU_PAD;
		switch (maccrd->crd_alg) {
			case	CRYPTO_MD5:	
			  	//td->hdr |= TALITOS_MODE0_MDEU_MD5;
				desc_header |= TALITOS_MODE0_MDEU_MD5;
				DPRINTF("MD5  ses %d ch %d len %d\n",
					(u32)TALITOS_SESSION(crp->crp_sid), 
					chsel, td->ptr[in_fifo].len);
				break;
			case	CRYPTO_MD5_HMAC:	
			  	//td->hdr |= TALITOS_MODE0_MDEU_MD5_HMAC;
				desc_header |= TALITOS_MODE0_MDEU_MD5_HMAC;
				break;
			case	CRYPTO_SHA1:	
			  	//td->hdr |= TALITOS_MODE0_MDEU_SHA1;
			  	desc_header |= TALITOS_MODE0_MDEU_SHA1;
				DPRINTF("SHA1 ses %d ch %d len %d\n",
					(u32)TALITOS_SESSION(crp->crp_sid), 
					chsel, td->ptr[in_fifo].len);
				break;
			case	CRYPTO_SHA1_HMAC:	
				//td->hdr |= TALITOS_MODE0_MDEU_SHA1_HMAC;
				desc_header |= TALITOS_MODE0_MDEU_SHA1_HMAC;
				break;
			default:
				/* We cannot order the SEC as requested */
				DPRINTF(DRV_NAME ": cannot do the order\n");
				err = EINVAL;
				goto errout;
		}

		if (crp->crp_flags & CRYPTO_F_IOV)
			if (!crp->crp_mac)
				td->ptr[out_fifo].ptr += maccrd->crd_inject;

		if ((maccrd->crd_alg == CRYPTO_MD5_HMAC) ||
		   (maccrd->crd_alg == CRYPTO_SHA1_HMAC)) {
			td->ptr[hmac_key].ptr = dma_map_single(NULL, 
				ses->ses_hmac, ses->ses_hmac_len, 
				DMA_TO_DEVICE);
			td->ptr[hmac_key].len = ses->ses_hmac_len;
		}
	} 
	else {
		/* using process key (session data has duplicate) */
		if (enccrd->crd_alg == CRYPTO_ARC4) {
                        if (sc->sc_rc4_first == 1) {
				td->ptr[cipher_key].ptr = dma_map_single(NULL, 
                                	enccrd->crd_key, (enccrd->crd_klen + 7) / 8, 
                                	DMA_TO_DEVICE);
				td->ptr[cipher_key].len = (enccrd->crd_klen + 7) / 8;
                        }
                        else if (sc->sc_rc4_first == 0) {
				td->ptr[cipher_key].ptr = NULL;
				td->ptr[cipher_key].len = 0;       
                        }
			else {
				printk("%s: the ARC4 error,first 2\n", __FUNCTION__);   
			}
		}
		else {
			td->ptr[cipher_key].ptr = dma_map_single(NULL, 
				enccrd->crd_key, (enccrd->crd_klen + 7) / 8, 
				DMA_TO_DEVICE);
			//td->ptr[cipher_key].ptr = virt_to_phys(enccrd->crd_key);
			td->ptr[cipher_key].len = (enccrd->crd_klen + 7) / 8;
		}
	}	

	
	if (enccrd->crd_alg != CRYPTO_ARC4) {
		desc_header |= TALITOS_DONE_NOTIFY;
	}
	/* Obtain spinlock */
	spin_lock_irqsave(&sc->sc_chnfifo[chsel][i].desc_lock, flags);

	/* write to header */					
	td->hdr = desc_header;
	/* release spinlock */
	spin_unlock_irqrestore(&sc->sc_chnfifo[chsel][i].desc_lock, flags);


	/* descriptor complete - GO! */
	status = talitos_submit(sc, td, chsel);

	/* AK tracing */
	//trace_log_L1(34,0,0);
	if (enccrd->crd_alg == CRYPTO_ARC4) {
		if (sc->sc_rc4_first == 1) {
			sc->sc_rc4_first = 0;
			printk("%s: ARC4 SETkey over,then do context\n",
				__FUNCTION__);
		}
	}
	/* AK : add this job to the tasklet q */
	#ifdef TALITOS_TASKLET	
	TALITOS_TASKLET_Q_LOCK();	
	list_add_tail(&sc->sc_chnfifo[chsel][i].desc_list , &talitos_tasklet_q);	
	talitos_inline_polling(r_flags);
	TALITOS_TASKLET_Q_UNLOCK();
	#endif /* TALITOS_TASKLET */
	if (coldfire_debug)
		dump_talitos_status(sc);
	
	return status;


errout:
	if (err != ERESTART) {
		crp->crp_etype = err;
		crypto_done(crp);
	} else {
	        sc->sc_needwakeup |= CRYPTO_SYMQ;
        }  
	
	return err;
}


#ifdef TALITOS_TASKLET
/*
 * Inline polling - from program context
 */
static inline int talitos_inline_polling (unsigned long lock_flags)
{
	int rc;
	u32 v;
	unsigned long r_flags = lock_flags;
	unsigned long flags;
	struct cryptop *crpt;
	struct desc_cryptop_pair *descp;
	rc = list_empty(&talitos_tasklet_q);
	if (!rc) {
			descp = list_entry(talitos_tasklet_q.next, typeof(*descp), 			     								desc_list);
			/* check for done notification */
			if ((descp->cf_desc.hdr & TALITOS_HDR_DONE_BITS) 
			    == TALITOS_HDR_DONE_BITS) {
			  //trace_log_L1(44,0,0);
				crpt = descp->cf_crp;
				if(coldfire_debug)
					printk(KERN_INFO "%s 0x%08x,0x%08x\n",
						__FUNCTION__,&descp->cf_desc.hdr,
						descp->cf_desc.hdr);

				/* every single pkt needs to be ack-ed to avoid
			 	* stored done interrupt generation. maybe one
			 	* write would do as we never unmask the cha 
			 	* interrupts
				*/
 				v = 0xffffffff;
				talitos_write(sec_base_addr + TALITOS_ICR, v);
				//talitos_write(sec_base_addr + TALITOS_ICR_HI, v);

				spin_lock_irqsave(&descp->desc_lock, flags);
				/* clear descriptor header */
				descp->cf_desc.hdr = 0;
				spin_unlock_irqrestore(&descp->desc_lock, flags);

				/* remove element from list */
				list_del(&descp->desc_list);

				TALITOS_TASKLET_Q_UNLOCK();
				/* complete post procesing */
       				crypto_done(crpt);

				TALITOS_TASKLET_Q_LOCK();
			}
			
	}
	return rc;
}



/*
 * This routine is called by talitos tasklet
 */
static inline void talitos_poll (unsigned long lock_flags)
{
	struct cryptop *crpt;
	struct desc_cryptop_pair *descp;
	unsigned long r_flags = lock_flags;
	unsigned long flags;
	u32 num_of_times;
	u32 v;
	num_of_times = 0;
	for(;;) {
		descp = NULL;
		if (!list_empty(&talitos_tasklet_q)) {
			descp = list_entry(talitos_tasklet_q.next, typeof(*descp), 														desc_list);
			crpt = descp->cf_crp;
			/* check for done notification */
			if ((descp->cf_desc.hdr & TALITOS_HDR_DONE_BITS) 
			    == TALITOS_HDR_DONE_BITS) {

				/* every single pkt needs to be ack-ed to avoid
			 	* stored done interrupt generation. maybe one
			 	* write would do as we never unmask the cha 
			 	* interrupts
				*/
 				v = 0xffffffff;
				talitos_write(sec_base_addr + TALITOS_ICR, v);
				//talitos_write(sec_base_addr + TALITOS_ICR_HI, v);

				spin_lock_irqsave(&descp->desc_lock, flags);
				/* clear descriptor header */
				descp->cf_desc.hdr = 0;
				spin_unlock_irqrestore(&descp->desc_lock, flags);

				/* remove element from list */
				list_del(&descp->desc_list);

				TALITOS_TASKLET_Q_UNLOCK();
				/* complete post procesing */
				//trace_log_L1(41,0,0);
       				crypto_done(crpt);
				TALITOS_TASKLET_Q_LOCK();
				num_of_times++;
			} else {
				//trace_log_L1(42,0,0);
				/* reschedule tasklet - this may be costly*/
				//tasklet_schedule(&isr_talitos_tasklet);
				
				v = talitos_read(sec_base_addr + TALITOS_IMR);
				v |= TALITOS_IMR_ALL;
				talitos_write(sec_base_addr + TALITOS_IMR, v);
				break;
			}
		
		} else {
			/* since Q empty we need to unmask talitos interrupt
			* so that tasklet can be scheduled again */
		  	//trace_log_L1(43,0,0);			 	

		  	/* before we unmask clear out talitos interrupt sources */
			v = 0xffffffff;
			talitos_write(sec_base_addr + TALITOS_ICR, v);
			talitos_write(sec_base_addr + TALITOS_ICR_HI, v);
		  	/* unmask talitos interrupt */
			v = talitos_read(sec_base_addr + TALITOS_IMR);
			v = 0;//|= TALITOS_IMR_ALL;
			talitos_write(sec_base_addr + TALITOS_IMR, v);
			break;
		}
	}

}


/*
 * this is the bottom half for talitos interrupt
 * processing
 */
static void talitos_tasklet (unsigned long data)
{
	unsigned long r_flags;
	TALITOS_TASKLET_Q_LOCK();
	talitos_poll(r_flags);
	TALITOS_TASKLET_Q_UNLOCK();
       
}
#endif /* TALITOS_TASKLET */



/* go through all channels descriptors, notifying OCF what has 
 * _and_hasn't_ successfully completed and reset the device 
 * (otherwise it's up to decoding desc hdrs!)
 */
static void talitos_errorprocessing(struct talitos_softc *sc, unsigned long chnum)
{
	unsigned long flags;
	int i=0, j=0;

	//printk(KERN_INFO "%s ERROR! we will be scewed if we proceed \n",__FUNCTION__);
	

	/* disable further scheduling until under control */
	//spin_lock_irqsave(&sc->sc_chnfifolock[sc->sc_num_channels], flags);

	if (debug) dump_talitos_status(sc);
	/* go through descriptors, try and salvage those successfully done, 
	 * and EIO those that weren't
	 */
	/* 
	 * Added by: Vishnu
	 * For TALITOS_IS_SEC_2_2_0
	 * Number of Channels is 1
	 * We can avoid this loop
	 */
	/*
	for (i = 0; i < sc->sc_num_channels; i++) {
		//spin_lock_irqsave(&sc->sc_chnfifolock[i], flags);
	*/
		for (j = 0; j < sc->sc_chfifo_len; j++) {
			if (sc->sc_chnfifo[i][j].cf_desc.hdr) {
				if ((sc->sc_chnfifo[i][j].cf_desc.hdr 
					& TALITOS_HDR_DONE_BITS) 
					!= TALITOS_HDR_DONE_BITS) {
					/* this one didn't finish */
					/* signify in crp->etype */
					sc->sc_chnfifo[i][j].cf_crp->crp_etype 
						= EIO;
				}
			} else
				continue; /* free entry */
			/* either way, notify ocf */
			crypto_done(sc->sc_chnfifo[i][j].cf_crp);
			/* and tag it available again */
			sc->sc_chnfifo[i][j].cf_desc.hdr = 0;
		}
	/*
		//spin_unlock_irqrestore(&sc->sc_chnfifolock[i], flags);
	}
	*/
	/* reset and initialize the SEC h/w device */
	talitos_reset_device(sc);
	talitos_init_device(sc);
	if (sc->sc_exec_units & TALITOS_HAS_EU_RNG)
		talitos_rng_init(sc);

	/* Okay. Stand by. */
	//spin_unlock_irqrestore(&sc->sc_chnfifolock[sc->sc_num_channels], flags);

	return;
}

/* go through all channels descriptors, notifying OCF what's been done */
static void talitos_doneprocessing(struct talitos_softc *sc, unsigned long chnum)
{
	unsigned long flags;
	u32 v;

      	/* enter the channel scheduler */ 
	//spin_lock_irqsave(&sc->sc_chnfifolock[sc->sc_num_channels], flags);

	#ifdef TALITOS_TASKLET

	/* mask SEC interrupt */
	v = talitos_read(sc->sc_base_addr + TALITOS_IMR);
	v = 0;//&= ~TALITOS_IMR_ALL;
	talitos_write(sc->sc_base_addr + TALITOS_IMR, v);

	//trace_log_L1(40,0,0);

	/* raise tasklet irq */
	tasklet_schedule(&isr_talitos_tasklet);
	#endif /* TALITOS_TASKLET */

	/* release the channel scheduler lock */ 
	//spin_unlock_irqrestore(&sc->sc_chnfifolock[sc->sc_num_channels], flags);
	return;
}


static irqreturn_t 
talitos_intr(int irq, void *arg)
{
	struct talitos_softc *sc = arg;
	u_int32_t v, v_hi;
	unsigned long chnum = 0;
	
	/* read the status register */
	v = talitos_read(sc->sc_base_addr + TALITOS_ISR);
	v_hi = talitos_read(sc->sc_base_addr + TALITOS_ISR_HI);

	/* determine for which channel we got this interrupt */
#if 0
	if ( v & 0x00000003)	
		chnum = 0;
	else if ( v & 0x0000000c)	
		chnum = 1;
	else if ( v & 0x00000030)	
		chnum = 2;
	else if ( v & 0x000000c0)	
		chnum = 3;
	else
		printk("panic, ISR_low = 0x%8.8x, ISR_hi = 0x%8.8x...\n", v, v_hi);
#endif
	chnum = 0;
	
	/* ack */
	talitos_write(sc->sc_base_addr + TALITOS_ICR, v);
	talitos_write(sc->sc_base_addr + TALITOS_ICR_HI, v_hi);

	if (unlikely(v & TALITOS_ISR_ERROR)) {
		/* Okay, Houston, we've had a problem here. */
		printk(KERN_DEBUG DRV_NAME 
			": got error interrupt - ISR 0x%08x_%08x\n", v, v_hi);
		//trace_set_L1(g,3);
		//trace_log_L1(40, trace_var(g), 300); 
		printk("%s error ...\n", __FUNCTION__);
		while(1){};
		talitos_errorprocessing(sc, chnum);
	} else
	if (v & TALITOS_ISR_DONE) {
		/* now do the done processing */
		printk("%s Done ...\n", __FUNCTION__);
		talitos_doneprocessing(sc, chnum);
	}
	else {
		printk(KERN_INFO "%s Panic : talitos intr. reason unknown \n", __FUNCTION__);
	}
	
	/* AK added */
        if (sc->sc_needwakeup) {                /* XXX check high watermark */
               int wakeup = sc->sc_needwakeup & (CRYPTO_SYMQ|CRYPTO_ASYMQ);
               DPRINTF("%s: wakeup crypto %x\n", __func__,
                        sc->sc_needwakeup);
               sc->sc_needwakeup &= ~wakeup;
               crypto_unblock(sc->sc_cid, wakeup);
        }

	return IRQ_HANDLED;
}

/*
 * set the master reset bit on the device.
 */
static void
talitos_reset_device_master(struct talitos_softc *sc)
{
	u_int32_t v;
#ifdef FSL_SEC11_MCF547X_8X
        unsigned long time = jiffies;
	if(coldfire_debug)
		DPRINTF("%s()\n", __FUNCTION__);
        v = 0;//talitos_read(sc->sc_base_addr + TALITOS_MCR);
        talitos_write(sc->sc_base_addr + TALITOS_MCR, v | TALITOS_MCR_SWR);

        while (talitos_read(sc->sc_base_addr + TALITOS_MCR) & TALITOS_MCR_SWR){
                if (jiffies - time > SEC_INIT_TIMEOUT){
                        printk("%s reset SEC1.1 Master timeout: %x\n",
                                        __FUNCTION__,v);
                        break;
                }
	}
	if(coldfire_debug)
		DPRINTF("%s ok\n", __FUNCTION__);
#else
	/* Reset the device by writing 1 to MCR:SWR and waiting 'til cleared */
	v = talitos_read(sc->sc_base_addr + TALITOS_MCR);
	talitos_write(sc->sc_base_addr + TALITOS_MCR, v | TALITOS_MCR_SWR);

	while (talitos_read(sc->sc_base_addr + TALITOS_MCR) & TALITOS_MCR_SWR)
		cpu_relax();
#endif
	return;
}

#ifdef FSL_SEC11_MCF547X_8X
static void
talitos_init_DESU(struct talitos_softc *sc)
{
        u_int32_t v;
	if(coldfire_debug)
		DPRINTF("%s()\n", __FUNCTION__);
        /* init DESU */
        v = talitos_read(sc->sc_base_addr + TALITOS_DEUICR);
        v = TALITOS_DEUICR_MASK;
        talitos_write(sc->sc_base_addr + TALITOS_DEUICR, v);
      
}

static void
talitos_init_AFEU(struct talitos_softc *sc)
{
        u_int32_t v;
	if(coldfire_debug)
		DPRINTF("%s()\n", __FUNCTION__);
        /* init AFEU */
        v = talitos_read(sc->sc_base_addr + TALITOS_AFEUICR);
        v = TALITOS_AFEUICR_MASK;
        talitos_write(sc->sc_base_addr + TALITOS_AFEUICR, v);
      
}

static void
talitos_init_AESU(struct talitos_softc *sc)
{
        u_int32_t v;
	if(coldfire_debug)
		DPRINTF("%s()\n", __FUNCTION__);
        /* init AESU */
        v = talitos_read(sc->sc_base_addr + TALITOS_AESUICR);
        v = TALITOS_AESUICR_MASK;
        talitos_write(sc->sc_base_addr + TALITOS_AESUICR, v);
      
}

static void
talitos_init_MDEU(struct talitos_softc *sc)
{
        u_int32_t v;
	if(coldfire_debug)
		DPRINTF("%s()\n", __FUNCTION__);
        /* init MDEU */
        v = talitos_read(sc->sc_base_addr + TALITOS_MDEUICR);
        v = TALITOS_MDEUICR_MASK;
        talitos_write(sc->sc_base_addr + TALITOS_MDEUICR, v);
      
}

static void
talitos_init_RNG(struct talitos_softc *sc)
{
        u_int32_t v;
	if(coldfire_debug)
		DPRINTF("%s()\n", __FUNCTION__);
        /* init RNG */
        v = talitos_read(sc->sc_base_addr + TALITOS_RNGICR);
        v = TALITOS_RNGICR_MASK;
        talitos_write(sc->sc_base_addr + TALITOS_RNGICR, v);
      
}

static void
talitos_reset_DESU(struct talitos_softc *sc)
{
        u_int32_t v;
	unsigned long time = jiffies;
	if(coldfire_debug)
		DPRINTF("%s()\n", __FUNCTION__);
        /*
 	* DESU reset
 	*/
	v = talitos_read(sc->sc_base_addr + TALITOS_DEURCR);
	talitos_write(sc->sc_base_addr + TALITOS_DEURCR, 
			v | TALITOS_DEURCR_RESET);

	while(!(talitos_read(sc->sc_base_addr + TALITOS_DEUSR) 
		& TALITOS_DEUSR_RESET)){
		if (jiffies - time > SEC_INIT_TIMEOUT){
			printk("%s reset DES unit timeout: %x\n",
                        		__FUNCTION__,v);
                        break;
		}
	}
	if(coldfire_debug)
		DPRINTF("%s ok\n", __FUNCTION__);
}

static void
talitos_reset_AFEU(struct talitos_softc *sc)
{
        u_int32_t v;
        unsigned long time = jiffies;
	if(coldfire_debug)
		DPRINTF("%s()\n", __FUNCTION__);
        /*
 	* AFEU reset
	*/
        v = talitos_read(sc->sc_base_addr + TALITOS_AFEURCR);
        talitos_write(sc->sc_base_addr + TALITOS_AFEURCR, 
                        v | TALITOS_AFEURCR_RESET);

        while(!(talitos_read(sc->sc_base_addr + TALITOS_AFEUSR) & 
		TALITOS_AFEUSR_RESET)){
                if (jiffies - time > 10*SEC_INIT_TIMEOUT){
                        printk("%s reset AFE unit timeout: %x should be %x\n",
                                        __FUNCTION__,
					talitos_read(sc->sc_base_addr + TALITOS_AFEUSR),
					TALITOS_AFEUSR_RESET);
                        return;
                }
        }
	if(coldfire_debug)
		DPRINTF("%s ok\n", __FUNCTION__);        
}

static void
talitos_reset_AESU(struct talitos_softc *sc)
{
        u_int32_t v;
        unsigned long time = jiffies;

	if(coldfire_debug)
		DPRINTF("%s()\n", __FUNCTION__);
        /*
	* AESU reset
 	*/
        v = talitos_read(sc->sc_base_addr + TALITOS_AESURCR);
        talitos_write(sc->sc_base_addr + TALITOS_AESURCR, 
                        v | TALITOS_AESURCR_RESET);

        while(!(talitos_read(sc->sc_base_addr + TALITOS_AESUSR) 
		& TALITOS_AESUSR_RESET)){
                if (jiffies - time > SEC_INIT_TIMEOUT){
                        printk("%s reset AES unit timeout: %x\n",
                                        __FUNCTION__,v);
                        break;
                }
        }
	if(coldfire_debug)
		DPRINTF("%s ok\n", __FUNCTION__);
}

static void
talitos_reset_MDEU(struct talitos_softc *sc)
{
        u_int32_t v;
        unsigned long time = jiffies;

	if(coldfire_debug)
		DPRINTF("%s()\n", __FUNCTION__);
        /*
	* MDEU reset
	*/
        v = talitos_read(sc->sc_base_addr + TALITOS_MDEURCR);
        talitos_write(sc->sc_base_addr + TALITOS_MDEURCR,
                        v | TALITOS_MDEURCR_RESET);

        while(!(talitos_read(sc->sc_base_addr + TALITOS_MDEUSR)  
		& TALITOS_MDEUSR_RESET)){
                if (jiffies - time > SEC_INIT_TIMEOUT){
                        printk("%s reset MDE unit timeout: %x\n",
                                        __FUNCTION__,v);
                        break;
                }
        }
	if(coldfire_debug)
		DPRINTF("%s ok\n", __FUNCTION__);
}

static void
talitos_reset_RNG(struct talitos_softc *sc)
{
        u_int32_t v;
        unsigned long time = jiffies;
	if(coldfire_debug)
		DPRINTF("%s()\n", __FUNCTION__);
        /*
	* RNG reset
	*/
        v = talitos_read(sc->sc_base_addr + TALITOS_RNGRCR);
        talitos_write(sc->sc_base_addr + TALITOS_RNGRCR,
                        v | TALITOS_RNGRCR_SR);

        while(!(talitos_read(sc->sc_base_addr + TALITOS_RNGSR) 
		& TALITOS_RNGSR_RD)){
                if (jiffies - time > SEC_INIT_TIMEOUT){
                        printk("%s reset RNG unit timeout: %x\n",
                                        __FUNCTION__,v);
                        break;
                }
        }
	if(coldfire_debug)
		DPRINTF("%s ok\n", __FUNCTION__);

}
#endif

/*
 *  * Initialize registers we need to touch only once.
 *   */
static void
talitos_init_device(struct talitos_softc *sc)
{
        u_int32_t v;
        int i;

	if(coldfire_debug)
		DPRINTF("%s()\n", __FUNCTION__);
#ifdef FSL_SEC11_MCF547X_8X
        /* init controller */

        v = talitos_read(sc->sc_base_addr + TALITOS_IMR);
        v = 0;//|= TALITOS_IMR_ALL;
        talitos_write(sc->sc_base_addr + TALITOS_IMR, v);
        v = talitos_read(sc->sc_base_addr + TALITOS_IMR_HI);
        v |= TALITOS_IMR_HI_ALL;
        talitos_write(sc->sc_base_addr + TALITOS_IMR_HI, v);

        /* init all channels */
        for (i = 0; i < sc->sc_num_channels; i++) {
                v = talitos_read(sc->sc_base_addr +
                        i*TALITOS_CH_OFFSET + TALITOS_CH_CCCR);
                v |= TALITOS_CH_CCCR_CDWE
                  |  TALITOS_CH_CCCR_NE
                  |  TALITOS_CH_CCCR_NT     /* Do selective notification */
                  |  TALITOS_CH_CCCR_CDIE;  /* invoke interrupt if done */
                  talitos_write(sc->sc_base_addr +
                        i*TALITOS_CH_OFFSET + TALITOS_CH_CCCR, v);
        }

        talitos_init_DESU(sc);
        talitos_init_AESU(sc);
        talitos_init_AFEU(sc);
        talitos_init_MDEU(sc);
        talitos_init_RNG(sc);

	MCF_ICR(ISC_SEC) = ILP_SEC;

       /* Enable the  SEC interrupt */
        //enable_irq(64 + ISC_SEC);	
	if (coldfire_debug)
		dump_talitos_status(sc);
#else
        /* init all channels */
        for (i = 0; i < sc->sc_num_channels; i++) {
                v = talitos_read(sc->sc_base_addr +
                        i*TALITOS_CH_OFFSET + TALITOS_CH_CCCR_HI);
                v |= TALITOS_CH_CCCR_HI_CDWE
                  //#ifdef TALITOS_INTERRUPT_COALESCE
                  //|  TALITOS_CH_CCCR_HI_NT     /* Do selective notification */
                  //#endif
                  |  TALITOS_CH_CCCR_HI_CDIE;  /* invoke interrupt if done */
                talitos_write(sc->sc_base_addr +
                	i*TALITOS_CH_OFFSET + TALITOS_CH_CCCR_HI, v);
        }
        /* enable all interrupts */
        v = talitos_read(sc->sc_base_addr + TALITOS_IMR);
        v |= TALITOS_IMR_ALL;
        talitos_write(sc->sc_base_addr + TALITOS_IMR, v);                                                                                                            /* AK commented out: we don't want to take interrupt
           or want to get notified when individual cha units
           complete its job. This could really hurt performance
           and we will take two interrupt per packet despite
           the fact that we use one descriptor
         */
        #if 0
        #ifdef TALITOS_BASELINE
        v = talitos_read(sc->sc_base_addr + TALITOS_IMR_HI);
        v |= TALITOS_IMR_HI_ALL;
        talitos_write(sc->sc_base_addr + TALITOS_IMR_HI, v); 
        #endif
        #endif
#endif
	return;
}
/*
 * Resets the device.  Values in the registers are left as is
 * from the reset (i.e. initial values are assigned elsewhere).
 */
static void
talitos_reset_device(struct talitos_softc *sc)
{
	u_int32_t v;
	int i;

	if(coldfire_debug)
		DPRINTF("%s()\n", __FUNCTION__);

	/*
	 * Master reset
	 * errata documentation: warning: certain SEC interrupts 
	 * are not fully cleared by writing the MCR:SWR bit, 
	 * set bit twice to completely reset 
	 */
#ifdef FSL_SEC11_MCF547X_8X
	unsigned long time;
	if (coldfire_debug)
		dump_talitos_status(sc);
	/*MCF_SECSACR = 1;*/
        talitos_reset_device_master(sc); 
#else
	talitos_reset_device_master(sc);	/* once */
	talitos_reset_device_master(sc);	/* and once again */
#endif	
	/* reset all channels */
	for (i = 0; i < sc->sc_num_channels; i++) {
		v = talitos_read(sc->sc_base_addr + i*TALITOS_CH_OFFSET +
			TALITOS_CH_CCCR);
		talitos_write(sc->sc_base_addr + i*TALITOS_CH_OFFSET +
			TALITOS_CH_CCCR, v | TALITOS_CH_CCCR_RESET);
#ifdef FSL_SEC11_MCF547X_8X
			time = jiffies;	
			while(talitos_read(sc->sc_base_addr + i*TALITOS_CH_OFFSET 
				+TALITOS_CH_CCCR) & TALITOS_CH_CCCR_RESET){
				if (jiffies - time > SEC_INIT_TIMEOUT){
					printk("%s reset channel %x timeout: %x,Addr %x\n",
						__FUNCTION__,i,
						talitos_read(sc->sc_base_addr + 
						i*TALITOS_CH_OFFSET +TALITOS_CH_CCCR),
						(sc->sc_base_addr + i*TALITOS_CH_OFFSET +
                                                        TALITOS_CH_CCCR));
					break;
				}
		
			}
#endif
	}

#ifdef FSL_SEC11_MCF547X_8X
	talitos_reset_AFEU(sc);
        talitos_reset_DESU(sc);
	talitos_reset_AESU(sc);
	talitos_reset_MDEU(sc);
	talitos_reset_RNG(sc);
	if (coldfire_debug)
		dump_talitos_status(sc);
#endif

}

/* Set up the crypto device structure, private data,
 * and anything else we need before we start */
static int talitos_probe(struct platform_device *pdev)
{
	struct talitos_softc *sc;
	struct resource *r;
	static int num_chips = 0;
	int rc = 0;
	int i;
	int j;

	if(coldfire_debug)
		DPRINTF("%s()\n", __FUNCTION__);

	sc = (struct talitos_softc *) kmalloc(sizeof(*sc), GFP_KERNEL);
	if (!sc){
		printk("%s kmalloc talitos_softc fail\n",__FUNCTION__);
		return -ENOMEM;
	}
	memset(sc, 0, sizeof(*sc));

	sc->sc_irq = -1;
	sc->sc_cid = -1;
	sc->sc_dev = pdev;
	sc->sc_num = num_chips++;

	platform_set_drvdata(sc->sc_dev, sc);

	/* get the irq line */
	//sc->sc_irq = 64 + platform_get_irq(pdev, 0);
        sc->sc_irq = 64 + platform_get_irq(pdev, 0);
	rc = request_irq(sc->sc_irq, talitos_intr, IRQF_DISABLED, DRV_NAME, sc);
	if (rc) {
		printk(KERN_ERR DRV_NAME ": failed to hook irq %d\n", 
			sc->sc_irq);
		sc->sc_irq = -1;
		goto out;
	}

	/* get a pointer to the register memory */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

#ifdef FSL_SEC11_MCF547X_8X
	sc->sc_base_addr = (ocf_iomem_t) r->start;
	if(coldfire_debug)
		DPRINTF("BaseAddr %x %x %x\n",sc->sc_base_addr,r->start,r->end);
#else
        sc->sc_base_addr = (ocf_iomem_t) ioremap(r->start, (r->end - r->start));
#endif
	#ifdef TALITOS_TASKLET 
	sec_base_addr = sc->sc_base_addr;
	#endif

	if (!sc->sc_base_addr) {
		printk(KERN_ERR DRV_NAME ": failed to ioremap\n");
		goto out;
	}
	
	/* figure out our SEC's properties and capabilities */
#ifdef FSL_SEC11_MCF547X_8X
        sc->sc_chiprev = (u64)talitos_read(sc->sc_base_addr + TALITOS_ID)
                 | talitos_read(sc->sc_base_addr + TALITOS_ID_HI) << 32;
	if(coldfire_debug)
		DPRINTF("sec id %x %x %x\n", talitos_read(sc->sc_base_addr + TALITOS_ID),
                                    talitos_read(sc->sc_base_addr + TALITOS_ID_HI),
                                    SEC_SID);
#else
	sc->sc_chiprev = (u64)talitos_read(sc->sc_base_addr + TALITOS_ID) << 32
		 | talitos_read(sc->sc_base_addr + TALITOS_ID_HI);
#endif
	if(coldfire_debug)
		DPRINTF("sec id 0x%llx\n", sc->sc_chiprev);
	/* bulk should go away with openfirmware flat device tree support */
	if (sc->sc_chiprev & TALITOS_ID_SEC_2_0) {
		sc->sc_num_channels = TALITOS_NCHANNELS_SEC_2_0;
		sc->sc_chfifo_len = TALITOS_CHFIFOLEN_SEC_2_0;
		sc->sc_exec_units = TALITOS_HAS_EUS_SEC_2_0;
		sc->sc_desc_types = TALITOS_HAS_DESCTYPES_SEC_2_0;
	}  
	else if (sc->sc_chiprev & TALITOS_ID_SEC_2_1_2) { /* AK generates compiler warning? */
		sc->sc_num_channels = TALITOS_NCHANNELS_SEC_2_0;
		sc->sc_chfifo_len = TALITOS_CHFIFOLEN_SEC_2_0;
		sc->sc_exec_units = TALITOS_HAS_EUS_SEC_2_0;
		sc->sc_desc_types = TALITOS_HAS_DESCTYPES_SEC_2_0;
	}
	else if (sc->sc_chiprev & TALITOS_ID_SEC_2_2_0) { /* Vishnu */
		sc->sc_num_channels = TALITOS_NCHANNELS_SEC_2_2;
		sc->sc_chfifo_len = TALITOS_CHFIFOLEN_SEC_2_2;
		sc->sc_exec_units = TALITOS_HAS_EUS_SEC_2_2;
		sc->sc_desc_types = TALITOS_HAS_DESCTYPES_SEC_2_2;
	}
#ifdef FSL_SEC11_MCF547X_8X
        else if (sc->sc_chiprev & TALITOS_ID_SEC_1_1) {
                sc->sc_num_channels = TALITOS_NCHANNELS_SEC_1_1;
                sc->sc_chfifo_len = TALITOS_CHFIFOLEN_SEC_1_1;
                sc->sc_exec_units = TALITOS_HAS_EUS_SEC_1_1;
                sc->sc_desc_types = TALITOS_HAS_DESCTYPES_SEC_1_1;
        }
#endif
	else {
		printk(KERN_ERR DRV_NAME ": failed to id device\n");
		goto out;
	}


	#ifdef TALITOS_TASKLET
		spin_lock_init(&talitos_tasklet_q_lock);
	#endif /* TALITOS_TASKLET */

	sc->sc_chnlastalg = (int *) kmalloc(
		sc->sc_num_channels * sizeof(int), GFP_KERNEL);
	if (!sc->sc_chnlastalg)
		goto out;
	memset(sc->sc_chnlastalg, 0, sc->sc_num_channels * sizeof(int));
	sc->sc_chnfifo = (struct desc_cryptop_pair **) kmalloc(
		sc->sc_num_channels * sizeof(struct desc_cryptop_pair *), 
		GFP_KERNEL);
	if (!sc->sc_chnfifo)
		goto out;

	for (i = 0; i < sc->sc_num_channels; i++) {
		sc->sc_chnfifo[i] = (struct desc_cryptop_pair *) kmalloc(
			sc->sc_chfifo_len * sizeof(struct desc_cryptop_pair), 
			GFP_KERNEL);
		if (!sc->sc_chnfifo[i])
			goto out;
		memset(sc->sc_chnfifo[i], 0, 
			sc->sc_chfifo_len * sizeof(struct desc_cryptop_pair));
	}

	/* Initialize header locks */
	for (i = 0; i < sc->sc_num_channels; i++) {
	  for(j = 0; j < sc->sc_chfifo_len; j++) {
		spin_lock_init(&sc->sc_chnfifo[i][j].desc_lock);
	  }
	}


	#ifdef TALITOS_TASKLET
	for (i = 0; i < sc->sc_num_channels; i++) {
		for (j = 0; j < sc->sc_chfifo_len; j++){
			INIT_LIST_HEAD(&sc->sc_chnfifo[i][j].desc_list);
	  	}
	}

	#endif /* TALITOS_TASKLET */

	/* reset and initialize the SEC h/w device */
	talitos_reset_device(sc);
	talitos_init_device(sc);

	sc->sc_cid = crypto_get_driverid(0, "talitos");
	if (sc->sc_cid < 0) {
		printk(KERN_ERR DRV_NAME ": could not get crypto driver id\n");
		goto out;
	}

	/* register algorithms with the framework */
	printk(DRV_NAME ":");

	if (sc->sc_exec_units & TALITOS_HAS_EU_RNG)  {
		printk(" rng");
		//talitos_rng_init(sc);
		crypto_rregister(sc->sc_cid, talitos_read_random, sc);
	}
	if (sc->sc_exec_units & TALITOS_HAS_EU_DEU) {
		printk(" des/3des");
		crypto_register(sc->sc_cid, CRYPTO_3DES_CBC, 0, 0,
			talitos_newsession, talitos_freesession,
			talitos_process, sc);
		crypto_register(sc->sc_cid, CRYPTO_DES_CBC, 0, 0,
			talitos_newsession, talitos_freesession,
			talitos_process, sc);
	}
	if (sc->sc_exec_units & TALITOS_HAS_EU_AESU) {
		printk(" aes");
		crypto_register(sc->sc_cid, CRYPTO_AES_CBC, 0, 0,
			talitos_newsession, talitos_freesession,
			talitos_process, sc);
	}
        if (sc->sc_exec_units & TALITOS_HAS_EU_AFEU) {
                printk(" rc4");
		sc->sc_rc4_first = 0;
                crypto_register(sc->sc_cid, CRYPTO_ARC4, 0, 0,
                        talitos_newsession, talitos_freesession,
                        talitos_process, sc);
        }
	if (sc->sc_exec_units & TALITOS_HAS_EU_MDEU) {
		printk(" md5");
		crypto_register(sc->sc_cid, CRYPTO_MD5, 0, 0,
			talitos_newsession, talitos_freesession,
			talitos_process, sc);
		/* HMAC support only with IPsec for now */
		crypto_register(sc->sc_cid, CRYPTO_MD5_HMAC, 0, 0,
			talitos_newsession, talitos_freesession,
			talitos_process, sc);
		printk(" sha1");
		crypto_register(sc->sc_cid, CRYPTO_SHA1, 0, 0,
			talitos_newsession, talitos_freesession,
			talitos_process, sc);
		/* HMAC support only with IPsec for now */
		crypto_register(sc->sc_cid, CRYPTO_SHA1_HMAC, 0, 0,
			talitos_newsession, talitos_freesession,
			talitos_process, sc);
	}
	printk(" \n");
	return 0;

out:
	talitos_remove(pdev);
	return -ENOMEM;
}



static int talitos_remove(struct platform_device *pdev)
{
	struct talitos_softc *sc = platform_get_drvdata(pdev);
	int i;

	DPRINTF("%s()\n", __FUNCTION__);
	if (sc->sc_chnfifo) {
		for (i = 0; i < sc->sc_num_channels; i++)
			if (sc->sc_chnfifo[i])
				kfree(sc->sc_chnfifo[i]);
		kfree(sc->sc_chnfifo);
	}
	if (sc->sc_chnlastalg)
		kfree(sc->sc_chnlastalg);
	//if (sc->sc_chnfifolock)
	//	kfree(sc->sc_chnfifolock);
	if (sc->sc_cid >= 0)
		crypto_unregister_all(sc->sc_cid);
	if (sc->sc_irq != -1)
		free_irq( sc->sc_irq, sc);
	if (sc->sc_base_addr)
		iounmap((void *) sc->sc_base_addr);
	kfree(sc);
	return 0;
}

/* Structure for a platform device driver */
static struct platform_driver talitos_driver = {
	.probe = talitos_probe,
	.remove = talitos_remove,
	.driver = {
#ifdef FSL_SEC11_MCF547X_8X
		.name = "fsl-sec1",
#else
		.name = "fsl-sec2"
#endif
	}
};

static int __init talitos_init(void)
{
	return platform_driver_register(&talitos_driver);
}

static void __exit talitos_exit(void)
{
	platform_driver_unregister(&talitos_driver);
}

module_init(talitos_init);
module_exit(talitos_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("kim.phillips@freescale.com");
MODULE_DESCRIPTION("OCF driver for Freescale SEC (talitos)");

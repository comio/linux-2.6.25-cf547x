/*
 * Freescale SEC data structures for integration with ocf-linux
 *
 * Copyright (c) 2006 Freescale Semiconductor, Inc.
 *
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
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/random.h>
#include <linux/skbuff.h>
#include <asm/scatterlist.h>
#include <linux/dma-mapping.h>  
#include <linux/moduleparam.h>
#include <linux/uio.h>

#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
#include <linux/platform_device.h>
#endif

//#define TALITOS_SMALL_PACKET_IMPROVE
//#define TALITOS_INTERRUPT_COALESCE 
#define TALITOS_HDR_DONE_BITS	0xff000000
/* This data structure used as "data" for the timer and tasklet functions */
struct talitos_timer_data {
	struct timer_list timer;
	int (*kick) (unsigned long tdelay);
	int (*stop) (void);
        unsigned long prevjiffies;
};

extern struct talitos_timer_data ktimer_data;

#ifdef TALITOS_SMALL_PACKET_IMPROVE
extern volatile unsigned long gPending; 
extern volatile unsigned long gPendingLoopIndex; 
extern volatile struct cryptop		*pCryptopPending[10]; /* assuming there could */
                                                      /* 100 req. outstanding at the max. */
static void talitos_timer_finish_job(void);
#endif

#ifdef TALITOS_INTERRUPT_COALESCE 
static void talitos_timer_coalesce_pending_job (void);
extern int talitos_assign_chnfifo(u32 reset);
extern int talitos_find_channel(int i, u32 reset, int chnum);
struct coalesce_info {
	u32 *pCryptopCoalesce[100]; /* pointer to cryptop struct */
	u32 *pTalitos_hdr[100];     /* pointer to talitos header */
	u32 how_many_pending;
	int chnum;
};
extern struct coalesce_info talitos_coalesce_info;
#endif



/* function prototypes */
static void talitos_timer_fn(unsigned long arg);
static void talitos_kick_timer(unsigned long tdelay);
static int talitos_stop_timer(void);
static int talitos_init_timer(void);


/*
 * setup the timer data structure and initialize
 * the kernel timer
 */
static int talitos_init_timer(void)
{	
	init_timer(&ktimer_data.timer);

	#if 0
	if( ktimer_data.timer.entry.next != NULL)
		printk("%s entry.next is not NULL! panic...\n", __FUNCTION__);
	else
		printk("%s entry.next is NULL, cool ...\n", __FUNCTION__);
	#endif


	/* initialize function pointers */
	ktimer_data.kick = talitos_kick_timer;
	ktimer_data.stop = talitos_stop_timer;
	       
	return 0;
}

/*
 * starting th kernel timer to count tdelay jiffies
 * at the end of which the talitos_timer_fn will
 * be called
 */
static void talitos_kick_timer(unsigned long tdelay)
{
	unsigned long j = jiffies;
	//printk("%s adding the kernel timer ...\n", __FUNCTION__);

	/* fill the data for our timer function */
	ktimer_data.prevjiffies = j;
	
	/* register the timer */
	ktimer_data.timer.data = &ktimer_data;
	ktimer_data.timer.function = talitos_timer_fn;
	ktimer_data.timer.expires = j + tdelay; /* parameter */

	#if 0
	if( ktimer_data.timer.entry.next != NULL)
		printk("%s entry.next is not NULL! panic...\n", __FUNCTION__);
	else
		printk("%s entry.next is NULL, cool ...\n", __FUNCTION__);
	#endif

	add_timer(&ktimer_data.timer);
}

/*
 * this will delete the timer
 */
static int talitos_stop_timer(void)
{
        del_timer(&ktimer_data.timer);
}

/*
 * this is the function that is invoked after 
 * tdelay jiffies. Its responsibility is to cleanup
 * outstanding crypto requests to talitos
 */
static void talitos_timer_fn(unsigned long arg)
{

	/* before doing anything stop the timer first */
	ktimer_data.stop();

        #ifdef TALITOS_SMALL_PACKET_IMPROVE

	/* finish the outstanding job */
	talitos_timer_finish_job();
        #endif

	#ifdef TALITOS_INTERRUPT_COALESCE
	talitos_timer_coalesce_pending_job();

	#endif
       
}

#ifdef TALITOS_SMALL_PACKET_IMPROVE
static void talitos_timer_finish_job(void)
{
	unsigned long i;;
	gPendingLoopIndex = gPending;
	printk("%s complete...\n", __FUNCTION__);
	for (i = 0; i < gPendingLoopIndex; i++) {
	  if (gPending >= 100) {
	    printk("%s panic  %d outstanding req. \n", __FUNCTION__, gPending);
	  }

	  /* AK : do the postprocessing here */
          crypto_done(pCryptopPending[i]);

	  /* AK : update the pending counter */
	  gPending--;
          //printk("%s gPending = %d \n", __FUNCTION__, gPending);
	}

}
#endif

#ifdef TALITOS_INTERRUPT_COALESCE
static void talitos_timer_coalesce_pending_job (void)
{
	u32 num_of_pending = 0;
	u32 num_completed = 0;
	u32 i;
	u32 reset;
	int fifo_num;
	int chnum;

	/* find out how many jobs where originally submitted */

	num_of_pending = talitos_coalesce_info.how_many_pending;

	/* find out the number of jobs completed and for the
	 * completed jobs call crypto_done to finish post
	 * processing 
	*/
	for (i = 0; i < num_of_pending; i++) {
		
		/* check whether the header associated with 
		 * this state indicates SEC completion of 
		 * the job
		*/
		//if((*talitos_coalesce_info.pTalitos_hdr[i]
		//	& TALITOS_HDR_DONE_BITS) 
		//        == TALITOS_HDR_DONE_BITS) {
		num_completed++;
		crypto_done(talitos_coalesce_info.pCryptopCoalesce[i]);
		/* now tag the descriptor as available */
		*talitos_coalesce_info.pTalitos_hdr[i] = 0;
		//}
	}
	
	//printk(KERN_INFO "%s pending=%d, completed=%d \n", __FUNCTION__,
	//       num_of_pending, num_completed);

	/* if SEC completed all jobs then do housekeeping
	 * work here, otherwise, kick the timer again so that 
	 * we can come back here to complete the remainder of 
	 * the jobs
	*/
	if ((num_of_pending - num_completed) == 0) {

		/* do some housekeeping work */

		/* reset the pending counter */
		talitos_coalesce_info.how_many_pending = 0;

		/* try to implement resetting the fifo number, there
		   is some problem in doing this due to static defn. 
		   of fifo number in talitos.c file. FIXME */
		reset = 1;
		fifo_num = talitos_assign_chnfifo(reset);
		//if (fifo_num == -1)
		//	printk(KERN_INFO "%s fifo num reset OK ...\n", __FUNCTION__);
		//else
		//	printk(KERN_INFO "%s fifo num reset not OK ...\n", __FUNCTION__);

		chnum = talitos_find_channel(0, 1, (talitos_coalesce_info.chnum));

	}else {
		/* schedule the timer again so 
		 *   that we can complete the pending
		 *  job
		*/
		ktimer_data.kick(1000);
	}
	//printk(KERN_INFO "%s trace 5 \n", __FUNCTION__);

}
#endif

/*
 * initializes a kernel timer
 */
static int  talitos_ktimer_probe (void) 
{
        talitos_init_timer();    
  
}

/*
 * removes kernel timer
 */
static int  talitos_ktimer_remove (void) 
{
        /* delete kernel timer */
        //del_timer(&data->timer);

        /* now free up the timer struct */
	printk("%s removing ktimer ...\n", __FUNCTION__);
 
}


static int __init talitos_ktimer_init(void)
{
	printk("%s ktimer initialized...\n", __FUNCTION__);
        talitos_ktimer_probe();
	
}

static void __exit talitos_ktimer_exit(void)
{
        talitos_ktimer_remove();

}

module_init(talitos_ktimer_init);
module_exit(talitos_ktimer_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("ahsan.kabir@freescale.com");
MODULE_DESCRIPTION("Timer for talitos driver");





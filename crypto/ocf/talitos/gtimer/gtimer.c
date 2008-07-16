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
#include <linux/dma-mapping.h>  /* dma_map_single() */
#include <linux/moduleparam.h>
#include <linux/uio.h>

#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
#include <linux/platform_device.h>
#endif

#include <crypto/cryptodev.h>
#include "gtimer.h"
#include <asm/io.h>
#define DRV_NAME "gtimer0" 
#define GTIMER0_DEBUG

static void gtimer0_test(void);
static int  gtimer0_remove (struct platform_device *pdev);
static inline void gtimer_write(volatile unsigned *addr, u32 val)
{
        /*out_be32(addr, val);*/
	*(volatile unsigned long *)((unsigned long)addr) = val;
}

static inline u32 gtimer_read(volatile unsigned *addr)
{
        u32 val;
        /*val = in_be32(addr);*/
	val = *(volatile unsigned long *)((unsigned long)addr) ;
        return val;
}


struct gtimer0_dev {
  int (*open) (unsigned long count);
  int (*close) (void);
};

struct gtimer0_dev gtimer_dev;
struct gtimer0_info *tm;


static int gtimer0_open (unsigned long count)
{
  
  unsigned long v;
  int i = 100;

  /* make sure counting is disabled first 
   * and then put down the base cnt value
  */
  v = gtimer_read(tm->tm_base_addr + GTIMER0_GTBCR_OFFSET);
  v |= 0x80000000; /* disable */
  gtimer_write(tm->tm_base_addr + GTIMER0_GTBCR_OFFSET, v);

  v = gtimer_read(tm->tm_base_addr + GTIMER0_GTBCR_OFFSET);
  v &= 0x80000000; /* clear count */
  gtimer_write(tm->tm_base_addr + GTIMER0_GTBCR_OFFSET, v);

  v = gtimer_read(tm->tm_base_addr + GTIMER0_GTBCR_OFFSET);
  v |= count; /* base cnt */
  gtimer_write(tm->tm_base_addr + GTIMER0_GTBCR_OFFSET, v);

  /* clear the count reg just in case */
  gtimer_write(tm->tm_base_addr + GTIMER0_GTCCR_OFFSET, 0);


  /* now enable counting */
  v = gtimer_read(tm->tm_base_addr + GTIMER0_GTBCR_OFFSET);
  v &= 0x7fffffff;
  gtimer_write(tm->tm_base_addr + GTIMER0_GTBCR_OFFSET, v);

  /* poll the current count reg */
  while (1)
    {
      v = gtimer_read(tm->tm_base_addr + GTIMER0_GTCCR_OFFSET);
      //printk("cur. cnt reg = 0x%8.8x \n", v);
      if (v & 0x80000000)
	break;
    }
  printk("%s TOG bit of CCR got set ...\n", __FUNCTION__);

  /* now check whether activity bit got set */
  //#if 0
  while (1)
    {
      v = gtimer_read(tm->tm_base_addr + GTIMER0_GTVPR_OFFSET);
      printk("%s vec/pri reg = 0x%8.8x \n", __FUNCTION__, v);
      if (v & 0x40000000)
	break;
    }
  //#endif
  

}

static int gtimer0_close (void)
{
   unsigned long *pRegRead;
 
  /* make sure counting is disabled first */
  pRegRead = (tm->tm_base_addr + GTIMER0_GTBCR_OFFSET);
  *pRegRead |= 0x80000000; 

  /* clear the base cnt register */
  *pRegRead &= 0x80000000;

}


static irqreturn_t
gtimer0_intr (int irq, void *arg, struct pt_regs *regs)
{
	/* stop counting and reload initial count value */
	/* figure out channel# and the FF# which are outstanding */
	/* access the queue to service those SEC req */
	/* obtain the desc ptr for all processed req and 
         clear their status for reuse */
         printk("%s timer working \n", __FUNCTION__);
  
         gtimer_dev.close();

	return IRQ_HANDLED;
}



static int  gtimer0_probe (struct platform_device *pdev) 
{
 
	struct resource *r;
        int rc;
	int i;
	unsigned long offset;

	tm = (struct gtimer0_info *) kmalloc(sizeof(*tm), GFP_KERNEL);
	if (!tm)
		return -ENOMEM;
	memset(tm, 0, sizeof(*tm));

	tm->tm_irq = -1; 
        tm->tm_dev = pdev;
       
        platform_set_drvdata(tm->tm_dev, tm);

	/* obtain irq for this timer */
	tm->tm_irq = platform_get_irq(pdev, 0);
	printk("%s IRQ number = %d \n", __FUNCTION__, tm->tm_irq);

	rc = request_irq(tm->tm_irq, gtimer0_intr, 0, DRV_NAME, tm);
	if (rc) {
		printk(KERN_ERR DRV_NAME ": failed to hook irq %d\n", 
			tm->tm_irq);
		tm->tm_irq = -1;
		goto out;
	}

        /* get the pointer to timer registers */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        tm->tm_base_addr = (unsigned long) ioremap(r->start, (r->end - r->start));
	printk("%s Immr base = 0x%8.8x \n", __FUNCTION__, tm->tm_base_addr);
	if (!tm->tm_base_addr) {
		printk(KERN_ERR DRV_NAME ": failed to ioremap\n");
		goto out;
	}

	/* initialize function pointers : open, close */
	gtimer_dev.open  = gtimer0_open;
	gtimer_dev.close = gtimer0_close;  

       /* now test the module */
	gtimer0_test();  

	return 0;

out:
	gtimer0_remove(pdev);
	return -ENOMEM;

}



static int  gtimer0_remove (struct platform_device *pdev) 
{
        struct gtimer0_info *tm = platform_get_drvdata(pdev);
	int i;
      
        /* make sure timer0 counting is disabled */
	/* clear any malloc memory */
	/* release IRQ */
	if (tm->tm_irq != -1)
	  free_irq(tm->tm_irq, tm);
	if (tm->tm_base_addr)
       		iounmap((void *) tm->tm_base_addr);

	kfree(tm);
        return 0;

}

static void gtimer0_test(void)
{
  unsigned long count;

  count = 0x7fffffff;  /* 2 sec wall clock */
  printk("%s Kicking timer with count = %d \n", __FUNCTION__, count);

  /* Kick the timer */
  gtimer_dev.open( count);

}


/* Structure for a timer0 device driver */
static struct platform_driver gtimer0_driver = {
	.probe = gtimer0_probe,
	.remove = gtimer0_remove,
	.driver = {
		.name = "fsl-gtimer0",
	}
};

static int __init gtimer0_init(void)
{
	return platform_driver_register(&gtimer0_driver);
}

static void __exit gtimer0_exit(void)
{

	platform_driver_unregister(&gtimer0_driver);
}

module_init(gtimer0_init);
module_exit(gtimer0_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("ahsan.kabir@freescale.com");
MODULE_DESCRIPTION("OpenPIC Global timer driver");




/*
TCR = default value
GTBCR0 = first CI=1 (disable counting), then load BASECNT = 24x3000= 9000 ticks = 0x384
GTVPR0 = taken care of by MPC8548CDS BSP
GTDR0 = default value
*/


//talitos_process ()
//{


	/* if chx and FF0 kick the timer */


//}


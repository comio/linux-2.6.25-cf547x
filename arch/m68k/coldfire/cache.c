/*
 *  linux/arch/m68k/coldifre/cache.c
 *
 *  Matt Waddel Matt.Waddel@freescale.com
 *  Copyright Freescale Semiconductor, Inc. 2007
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */

#include <linux/interrupt.h>
#include <asm/cfcache.h>
#include <asm/coldfire.h>
#include <asm/system.h>

#define _DCACHE_SIZE (2*16384)
#define _ICACHE_SIZE (2*16384)

#define _SET_SHIFT 4

/*
 * Masks for cache sizes.  Programming note: because the set size is a
 * power of two, the mask is also the last address in the set.
 */

#define _DCACHE_SET_MASK ((_DCACHE_SIZE/64-1)<<_SET_SHIFT)
#define _ICACHE_SET_MASK ((_ICACHE_SIZE/64-1)<<_SET_SHIFT)
#define LAST_DCACHE_ADDR _DCACHE_SET_MASK
#define LAST_ICACHE_ADDR _ICACHE_SET_MASK

/************************************************************
 *  Routine to cleanly flush the cache, pushing all lines and
 *  invalidating them.
 *
 *  The is the flash-resident version, used after copying the .text
 *  segment from flash to ram.
 *************************************************************/
void FLASHDcacheFlushInvalidate(void)
	__attribute__ ((section (".text_loader")));

void FLASHDcacheFlushInvalidate()
{
	unsigned long set;
	unsigned long start_set;
	unsigned long end_set;

	start_set = 0;
	end_set = (unsigned long)LAST_DCACHE_ADDR;

	for (set = start_set; set < end_set; set += (0x10 - 3))
		asm volatile("cpushl %%dc,(%0)\n"
			     "\taddq%.l #1,%0\n"
			     "\tcpushl %%dc,(%0)\n"
			     "\taddq%.l #1,%0\n"
			     "\tcpushl %%dc,(%0)\n"
			     "\taddq%.l #1,%0\n"
			     "\tcpushl %%dc,(%0)" : : "a" (set));
}

/************************************************************
 *  Routine to cleanly flush the cache, pushing all lines and
 *  invalidating them.
 *
 *************************************************************/
void DcacheFlushInvalidate()
{
	unsigned long set;
	unsigned long start_set;
	unsigned long end_set;

	start_set = 0;
	end_set = (unsigned long)LAST_DCACHE_ADDR;

	for (set = start_set; set < end_set; set += (0x10 - 3))
		asm volatile("cpushl %%dc,(%0)\n"
			     "\taddq%.l #1,%0\n"
			     "\tcpushl %%dc,(%0)\n"
			     "\taddq%.l #1,%0\n"
			     "\tcpushl %%dc,(%0)\n"
			     "\taddq%.l #1,%0\n"
			     "\tcpushl %%dc,(%0)" : : "a" (set));
}



/******************************************************************************
 *  Routine to cleanly flush the a block of cache, pushing all relevant lines
 *  and invalidating them.
 *
 ******************************************************************************/
void DcacheFlushInvalidateCacheBlock(void *start, unsigned long size)
{
	unsigned long set;
	unsigned long start_set;
	unsigned long end_set;

	/* if size is bigger than the cache can store
	 * set the size to the maximum amount
	 */

	if (size > LAST_DCACHE_ADDR)
		size = LAST_DCACHE_ADDR;

	start_set = ((unsigned long)start) & _DCACHE_SET_MASK;
	end_set = ((unsigned long)(start+size-1)) & _DCACHE_SET_MASK;

	if (start_set > end_set) {
		/* from the begining to the lowest address */
		for (set = 0; set <= end_set; set += (0x10 - 3))
			asm volatile("cpushl %%dc,(%0)\n"
				     "\taddq%.l #1,%0\n"
				     "\tcpushl %%dc,(%0)\n"
				     "\taddq%.l #1,%0\n"
				     "\tcpushl %%dc,(%0)\n"
				     "\taddq%.l #1,%0\n"
				     "\tcpushl %%dc,(%0)" : : "a" (set));

		/* next loop will finish the cache ie pass the hole */
		end_set = LAST_DCACHE_ADDR;
	}
	for (set = start_set; set <= end_set; set += (0x10 - 3))
		asm volatile("cpushl %%dc,(%0)\n"
			     "\taddq%.l #1,%0\n"
			     "\tcpushl %%dc,(%0)\n"
			     "\taddq%.l #1,%0\n"
			     "\tcpushl %%dc,(%0)\n"
			     "\taddq%.l #1,%0\n"
			     "\tcpushl %%dc,(%0)" : : "a" (set));
}


void IcacheInvalidateCacheBlock(void *start, unsigned long size)
{
	unsigned long set;
	unsigned long start_set;
	unsigned long end_set;

	/* if size is bigger than the cache can store
	 * set the size to the maximum ammount
	 */

	if (size > LAST_ICACHE_ADDR)
		size = LAST_ICACHE_ADDR;

	start_set = ((unsigned long)start) & _ICACHE_SET_MASK;
	end_set = ((unsigned long)(start+size-1)) & _ICACHE_SET_MASK;

	if (start_set > end_set) {
		/* from the begining to the lowest address */
		for (set = 0; set <= end_set; set += (0x10 - 3))
			asm volatile("cpushl %%ic,(%0)\n"
				     "\taddq%.l #1,%0\n"
				     "\tcpushl %%ic,(%0)\n"
				     "\taddq%.l #1,%0\n"
				     "\tcpushl %%ic,(%0)\n"
				     "\taddq%.l #1,%0\n"
				     "\tcpushl %%ic,(%0)" : : "a" (set));

		/* next loop will finish the cache ie pass the hole */
		end_set = LAST_ICACHE_ADDR;
	}
	for (set = start_set; set <= end_set; set += (0x10 - 3))
		asm volatile("cpushl %%ic,(%0)\n"
			     "\taddq%.l #1,%0\n"
			     "\tcpushl %%ic,(%0)\n"
			     "\taddq%.l #1,%0\n"
			     "\tcpushl %%ic,(%0)\n"
			     "\taddq%.l #1,%0\n"
			     "\tcpushl %%ic,(%0)" : : "a" (set));
}


/********************************************************************
 *  Disable the data cache completely
 ********************************************************************/
void DcacheDisable(void)
{
	int newValue;
	unsigned long flags;

	local_save_flags(flags);
	local_irq_disable();

	DcacheFlushInvalidate();      /* begin by flushing the cache */
	newValue = CACHE_DISABLE_MODE; /* disable it */
	cacr_set(newValue);
	local_irq_restore(flags);
}

/********************************************************************
 *  Unconditionally enable the data cache
 ********************************************************************/
void DcacheEnable(void)
{
	cacr_set(CACHE_INITIAL_MODE);
}


unsigned long shadow_cacr;

void cacr_set(unsigned long x)
{
	shadow_cacr = x;

	__asm__ __volatile__ ("movec %0, %%cacr"
			      : /* no outputs */
			      : "r" (shadow_cacr));
}

unsigned long cacr_get(void)
{
	return shadow_cacr;
}

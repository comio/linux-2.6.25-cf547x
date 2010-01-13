/*
 * include/asm-m68k/cf_548x_cacheflush.h - Coldfire 547x/548x Cache
 *
 * Based on include/asm-m68k/cacheflush.h
 *
 * Coldfire pieces by:
 *   Kurt Mahan kmahan@freescale.com
 *
 * Copyright Freescale Semiconductor, Inc. 2007, 2008
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef M68K_CF_548x_CACHEFLUSH_H
#define M68K_CF_548x_CACHEFLUSH_H

#include <asm/cfcache.h>
/*
 * Cache handling functions
 */

#define flush_icache()						\
({								\
  unsigned long set;						\
  unsigned long start_set;					\
  unsigned long end_set;					\
								\
  start_set = 0;						\
  end_set = (unsigned long)LAST_ICACHE_ADDR;			\
    								\
  for (set = start_set; set <= end_set; set += (CACHE_LINE_SIZE - 3)) {	\
    asm volatile("cpushl %%ic,(%0)\n"				\
                 "\taddq%.l #1,%0\n"				\
                 "\tcpushl %%ic,(%0)\n"				\
                 "\taddq%.l #1,%0\n"				\
                 "\tcpushl %%ic,(%0)\n"				\
                 "\taddq%.l #1,%0\n"				\
                 "\tcpushl %%ic,(%0)" : "=a" (set) : "0" (set));		\
  }								\
})

#define flush_dcache()						\
({								\
  unsigned long set;						\
  unsigned long start_set;					\
  unsigned long end_set;					\
								\
  start_set = 0;						\
  end_set = (unsigned long)LAST_DCACHE_ADDR;			\
    								\
  for (set = start_set; set <= end_set; set += (CACHE_LINE_SIZE - 3)) {	\
    asm volatile("cpushl %%dc,(%0)\n"				\
                 "\taddq%.l #1,%0\n"				\
                 "\tcpushl %%dc,(%0)\n"				\
                 "\taddq%.l #1,%0\n"				\
                 "\tcpushl %%dc,(%0)\n"				\
                 "\taddq%.l #1,%0\n"				\
                 "\tcpushl %%dc,(%0)" : "=a" (set) : "0" (set));		\
  }								\
})

#define flush_bcache()						\
({								\
  unsigned long set;						\
  unsigned long start_set;					\
  unsigned long end_set;					\
								\
  start_set = 0;						\
  end_set = (unsigned long)LAST_DCACHE_ADDR;			\
    								\
  for (set = start_set; set <= end_set; set += (CACHE_LINE_SIZE - 3)) {	\
    asm volatile("cpushl %%bc,(%0)\n"				\
                 "\taddq%.l #1,%0\n"				\
                 "\tcpushl %%bc,(%0)\n"				\
                 "\taddq%.l #1,%0\n"				\
                 "\tcpushl %%bc,(%0)\n"				\
                 "\taddq%.l #1,%0\n"				\
                 "\tcpushl %%bc,(%0)" : "=a" (set) : "0" (set));		\
  }								\
})

/**
 * cf_cache_flush - Push dirty cache out and invalidate
 * @paddr: starting physical address
 * @len: number of bytes
 *
 * Push the any dirty lines starting at paddr for len bytes and
 * invalidate those lines.
 */
static inline void cf_cache_flush(unsigned long paddr, int len)
{
        unsigned long start_line;
        unsigned long end_line;
        unsigned long sets;
        unsigned long i;

	if (len == 0)
		return;

        if (len > DCACHE_WAY_SIZE) {
                len = DCACHE_WAY_SIZE;
        }
        
        end_line = ((paddr & (DCACHE_WAY_SIZE - 1)) + len + (CACHE_LINE_SIZE - 1)) & ~(CACHE_LINE_SIZE - 1);
        start_line = paddr & _DCACHE_SET_MASK;
        sets = (end_line - start_line) / CACHE_LINE_SIZE;

        for (i = 0; i < sets; i++ , start_line = (start_line + (CACHE_LINE_SIZE - 3)) & _DCACHE_SET_MASK) {
                asm volatile("cpushl %%bc,(%0)\n"
                             "addq%.l #1,%0\n"
                             "cpushl %%bc,(%0)\n"
                             "addq%.l #1,%0\n"
                             "cpushl %%bc,(%0)\n"
                             "addq%.l #1,%0\n"
                             "cpushl %%bc,(%0)" : "=a" (start_line) : "0" (start_line));
        }

#if 0
        unsigned long set;
        unsigned long start_set;
        unsigned long end_set;

        start_set = paddr & _DCACHE_SET_MASK;
        end_set = (start_set + len - 1) & _DCACHE_SET_MASK;

        if (start_set > end_set) {
                /* from the begining to the lowest address */
                for (set = 0; set <= end_set; set += (CACHE_LINE_SIZE - 3)) {
                        asm volatile("cpushl %%bc,(%0)\n"
                                     "addq%.l #1,%0\n"
                                     "cpushl %%bc,(%0)\n"
                                     "addq%.l #1,%0\n"
                                     "cpushl %%bc,(%0)\n"
                                     "addq%.l #1,%0\n"
                                     "cpushl %%bc,(%0)" : "=a" (set) : "0" (set));
                }
                /* next loop will finish the cache ie pass the hole */
                end_set = LAST_DCACHE_ADDR;    
        }

        for (set = start_set; set <= end_set; set += (CACHE_LINE_SIZE - 3)) {
                asm volatile("cpushl %%bc,(%0)\n"
                             "addq%.l #1,%0\n"
                             "cpushl %%bc,(%0)\n"
                             "addq%.l #1,%0\n"
                             "cpushl %%bc,(%0)\n"
                             "addq%.l #1,%0\n"
                             "cpushl %%bc,(%0)" : "=a" (set) : "0" (set));
        }
#endif
}

/**
 * cf_cache_push - Push dirty cache out with no invalidate
 * @paddr: starting physical address
 * @len: number of bytes
 *
 * Push the any dirty lines starting at paddr for len bytes.
 * Those lines are not invalidated.
 */
static inline void cf_cache_push(unsigned long paddr, int len)
{
	asm volatile("nop\n"
		     "move%.l   %0,%%d0\n"
		     "or%.l	%1,%%d0\n"
		     "movec	%%d0,%%cacr\n"
                     : : "r" (shadow_cacr),
			 "i" (CF_CACR_DPI+CF_CACR_IDPI)
		     : "d0");

        cf_cache_flush(paddr , len);

	asm volatile("nop\n"
		     "movec	%0,%%cacr\n"
                     : : "r" (shadow_cacr));

}

/**
 * cf_cache_clear - invalidate cache
 * @paddr: starting physical address
 * @len: number of bytes
 *
 * Invalidate cache lines starting at paddr for len bytes.
 * Those lines are not pushed (a lie for 547x).
 */
static inline void cf_cache_clear(unsigned long paddr, int len)
{
  cf_cache_flush(paddr , len);
}

/**
 * cf_cache_flush_range - Push dirty data/inst cache in range out and invalidate
 * @vstart - starting virtual address
 * @vend: ending virtual address
 */
static inline void cf_cache_flush_range(unsigned long vstart, unsigned long vend)
{
	vend = PAGE_ALIGN(vend);

        cf_cache_flush(__pa(vstart) , vend - vstart);
}

/**
 * cf_dcache_flush_range - Push dirty data cache in range out and invalidate
 * @vstart - starting virtual address
 * @vend: ending virtual address
 */
static inline void cf_dcache_flush_range(unsigned long vstart, unsigned long vend)
{
        unsigned long start_line;
        unsigned long end_line;
        unsigned long sets;
        unsigned long i;
        unsigned long paddr = __pa(vstart);
        unsigned long len = vend - vstart;

	if (len == 0)
		return;

        if (len > DCACHE_WAY_SIZE) {
                len = DCACHE_WAY_SIZE;
        }
        
        end_line = ((paddr & (DCACHE_WAY_SIZE - 1)) + len + (CACHE_LINE_SIZE - 1)) & ~(CACHE_LINE_SIZE - 1);
        start_line = paddr & _DCACHE_SET_MASK;
        sets = (end_line - start_line) / CACHE_LINE_SIZE;

        for (i = 0; i < sets; i++ , start_line = (start_line + (CACHE_LINE_SIZE - 3)) & _DCACHE_SET_MASK) {
                asm volatile("cpushl %%dc,(%0)\n"
                             "addq%.l #1,%0\n"
                             "cpushl %%dc,(%0)\n"
                             "addq%.l #1,%0\n"
                             "cpushl %%dc,(%0)\n"
                             "addq%.l #1,%0\n"
                             "cpushl %%dc,(%0)" : "=a" (start_line) : "0" (start_line));
        }
}

/**
 * cf_icache_flush_range - Push dirty inst cache in range out and invalidate
 * @vstart - starting virtual address
 * @vend: ending virtual address
 *
 * Push the any dirty instr lines starting at paddr for len bytes and
 * invalidate those lines.  This should just be an invalidate since you
 * shouldn't be able to have dirty instruction cache.
 */
static inline void cf_icache_flush_range(unsigned long vstart, unsigned long vend)
{
        unsigned long start_line;
        unsigned long end_line;
        unsigned long sets;
        unsigned long i;
        unsigned long paddr = __pa(vstart);
        unsigned long len = vend - vstart;

	if (len == 0)
		return;

        if (len > ICACHE_WAY_SIZE) {
                len = ICACHE_WAY_SIZE;
        }
        
        end_line = ((paddr & (ICACHE_WAY_SIZE - 1)) + len + (CACHE_LINE_SIZE - 1)) & ~(CACHE_LINE_SIZE - 1);
        start_line = paddr & _ICACHE_SET_MASK;
        sets = (end_line - start_line) / CACHE_LINE_SIZE;

        for (i = 0; i < sets; i++ , start_line = (start_line + (CACHE_LINE_SIZE - 3)) & _ICACHE_SET_MASK) {
                asm volatile("cpushl %%ic,(%0)\n"
                             "addq%.l #1,%0\n"
                             "cpushl %%ic,(%0)\n"
                             "addq%.l #1,%0\n"
                             "cpushl %%ic,(%0)\n"
                             "addq%.l #1,%0\n"
                             "cpushl %%ic,(%0)" : "=a" (start_line) : "0" (start_line));
        }
}

/*
 * invalidate the cache for the specified memory range.
 * It starts at the physical address specified for
 * the given number of bytes.
 */
extern void cache_clear(unsigned long paddr, int len);
/*
 * push any dirty cache in the specified memory range.
 * It starts at the physical address specified for
 * the given number of bytes.
 */
extern void cache_push(unsigned long paddr, int len);

/*
 * push and invalidate pages in the specified user virtual
 * memory range.
 */
extern void cache_push_v(unsigned long vaddr, int len);

/* This is needed whenever the virtual mapping of the current
   process changes.  */

/**
 * flush_cache_mm - Flush an mm_struct
 * @mm: mm_struct to flush
 */
static inline void flush_cache_mm(struct mm_struct *mm)
{
	if (mm == current->mm)
		flush_bcache();
}

#define flush_cache_dup_mm(mm)	flush_cache_mm(mm)

#define flush_cache_all()		flush_bcache()

/**
 * flush_cache_range - Flush a cache range
 * @vma: vma struct
 * @start: Starting address
 * @end: Ending address
 *
 * flush_cache_range must be a macro to avoid a dependency on
 * linux/mm.h which includes this file.
 */
static inline void flush_cache_range(struct vm_area_struct *vma,
	unsigned long start, unsigned long end)
{
	if (vma->vm_mm == current->mm)
                cf_cache_flush_range(start, end);
}

/**
 * flush_cache_page - Flush a page of the cache
 * @vma: vma struct
 * @vmaddr:
 * @pfn: page numer
 *
 * flush_cache_page must be a macro to avoid a dependency on
 * linux/mm.h which includes this file.
 */
static inline void flush_cache_page(struct vm_area_struct *vma,
	unsigned long vmaddr, unsigned long pfn)
{
	if (vma->vm_mm == current->mm)
		cf_cache_flush_range(vmaddr, vmaddr+PAGE_SIZE);
}

/* Push the page at kernel virtual address and clear the icache */
/* RZ: use cpush %bc instead of cpush %dc, cinv %ic */
#define flush_page_to_ram(page) __flush_page_to_ram((void *) page_address(page))
extern inline void __flush_page_to_ram(void *address)
{
        unsigned long addr = (unsigned long)address;

        addr &= ~(PAGE_SIZE - 1); /* round down to page start address */
        
        cf_cache_flush_range(addr , addr + PAGE_SIZE);
}

/* Use __flush_page_to_ram() for flush_dcache_page all values are same - MW */
#define flush_dcache_page(page)			\
	__flush_page_to_ram((void *) page_address(page))
#define flush_icache_page(vma,pg)		\
	__flush_page_to_ram((void *) page_address(pg))
#define flush_icache_user_range(adr,len)	do { } while (0)
/* NL */
#define flush_icache_user_page(vma,page,addr,len)	do { } while (0)

/* Push n pages at kernel virtual address and clear the icache */
extern void flush_icache_range (unsigned long address,
                                unsigned long endaddr);

static inline void copy_to_user_page(struct vm_area_struct *vma,
				     struct page *page, unsigned long vaddr,
				     void *dst, void *src, int len)
{
	memcpy(dst, src, len);
	cf_cache_flush(page_to_phys(page), PAGE_SIZE);
}
static inline void copy_from_user_page(struct vm_area_struct *vma,
				       struct page *page, unsigned long vaddr,
				       void *dst, void *src, int len)
{
	cf_cache_flush(page_to_phys(page), PAGE_SIZE);
	memcpy(dst, src, len);
}

#define flush_cache_vmap(start, end)		flush_cache_all()
#define flush_cache_vunmap(start, end)		flush_cache_all()
#define flush_dcache_mmap_lock(mapping)		do { } while (0)
#define flush_dcache_mmap_unlock(mapping)	do { } while (0)

#endif /* M68K_CF_548x_CACHEFLUSH_H */

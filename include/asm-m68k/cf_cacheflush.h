#ifndef M68K_CF_CACHEFLUSH_H
#define M68K_CF_CACHEFLUSH_H

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
  end_set = (unsigned long)LAST_DCACHE_ADDR;			\
								\
  for (set = start_set; set <= end_set; set += (0x10 - 3)) 	\
    asm volatile("cpushl %%ic,(%0)\n"				\
		 "\taddq%.l #1,%0\n"				\
		 "\tcpushl %%ic,(%0)\n"				\
		 "\taddq%.l #1,%0\n"				\
		 "\tcpushl %%ic,(%0)\n"				\
		 "\taddq%.l #1,%0\n"				\
		 "\tcpushl %%ic,(%0)" : : "a" (set));		\
})

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


#define flush_cache_all()		do { } while (0)
#define flush_cache_mm(mm)		do { } while (0)
#define flush_cache_range(mm, a, b)	do { } while (0)
#define flush_cache_page(vma, vmaddr, pfn)	do { } while (0)

#define flush_dcache_range(paddr, len)	do { } while (0)

/* Push the page at kernel virtual address and clear the icache */
/* use cpush %bc instead of cpush %dc, cinv %ic */
#define flush_page_to_ram(page) __flush_page_to_ram((void *) page_address(page))
extern inline void __flush_page_to_ram(void *address)
{
  unsigned long set;
  unsigned long start_set;
  unsigned long end_set;
  unsigned long addr = (unsigned long) address;

  addr &= ~(PAGE_SIZE - 1); /* round down to page start address */

  start_set = addr & _ICACHE_SET_MASK;
  end_set = (addr + PAGE_SIZE-1) & _ICACHE_SET_MASK;

  if (start_set > end_set) {
    /* from the begining to the lowest address */
    for (set = 0; set <= end_set; set += (0x10 - 3))
      asm volatile("cpushl %%bc,(%0)\n"
		   "\taddq%.l #1,%0\n"
		   "\tcpushl %%bc,(%0)\n"
		   "\taddq%.l #1,%0\n"
		   "\tcpushl %%bc,(%0)\n"
		   "\taddq%.l #1,%0\n"
		   "\tcpushl %%bc,(%0)" : : "a" (set));

    /* next loop will finish the cache ie pass the hole */
    end_set = LAST_ICACHE_ADDR;
  }
  for (set = start_set; set <= end_set; set += (0x10 - 3))
    asm volatile("cpushl %%bc,(%0)\n"
		 "\taddq%.l #1,%0\n"
		 "\tcpushl %%bc,(%0)\n"
		 "\taddq%.l #1,%0\n"
		 "\tcpushl %%bc,(%0)\n"
		 "\taddq%.l #1,%0\n"
		 "\tcpushl %%bc,(%0)" : : "a" (set));
}

#define flush_dcache_page(page)			do { } while (0)
#define flush_icache_page(vma, pg)		do { } while (0)
#define flush_icache_user_range(adr, len)	do { } while (0)
/* NL */
#define flush_icache_user_page(vma, page, addr, len)	do { } while (0)

/* Push n pages at kernel virtual address and clear the icache */
/* use cpush %bc instead of cpush %dc, cinv %ic */
extern inline void flush_icache_range(unsigned long address,
				      unsigned long endaddr)
{
  unsigned long set;
  unsigned long start_set;
  unsigned long end_set;

  start_set = address & _ICACHE_SET_MASK;
  end_set = endaddr & _ICACHE_SET_MASK;

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

static inline void copy_to_user_page(struct vm_area_struct *vma,
				     struct page *page, unsigned long vaddr,
				     void *dst, void *src, int len)
{
	memcpy(dst, src, len);
	flush_icache_user_page(vma, page, vaddr, len);
}
static inline void copy_from_user_page(struct vm_area_struct *vma,
				       struct page *page, unsigned long vaddr,
				       void *dst, void *src, int len)
{
	memcpy(dst, src, len);
}

#define flush_cache_dup_mm(mm)			flush_cache_mm(mm)
#define flush_cache_vmap(start, end)		flush_cache_all()
#define flush_cache_vunmap(start, end)		flush_cache_all()
#define flush_dcache_mmap_lock(mapping)		do { } while (0)
#define flush_dcache_mmap_unlock(mapping)	do { } while (0)

#endif /* M68K_CF_CACHEFLUSH_H */

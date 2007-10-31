/*
 * include/asm-m68k/cfcache.h
 */
#ifndef CF_CFCACHE_H
#define CF_CFCACHE_H

#define CF_CACR_DEC         (0x80000000) /* Data Cache Enable                */
#define CF_CACR_DW          (0x40000000) /* Data default Write-protect       */
#define CF_CACR_DESB        (0x20000000) /* Data Enable Store Buffer         */
#define CF_CACR_DDPI        (0x10000000) /* Data Disable CPUSHL Invalidate   */
#define CF_CACR_DHLCK       (0x08000000) /* 1/2 Data Cache Lock Mode         */
#define CF_CACR_DDCM_00     (0x00000000) /* Cacheable writethrough imprecise */
#define CF_CACR_DDCM_01     (0x02000000) /* Cacheable copyback               */
#define CF_CACR_DDCM_10     (0x04000000) /* Noncacheable precise             */
#define CF_CACR_DDCM_11     (0x06000000) /* Noncacheable imprecise           */
#define CF_CACR_DCINVA      (0x01000000) /* Data Cache Invalidate All        */
#define CF_CACR_IVO         (0x00100000) /* Invalidate only                  */
#define CF_CACR_BEC         (0x00080000) /* Branch Cache Enable              */
#define CF_CACR_BCINVA      (0x00040000) /* Branch Cache Invalidate All      */
#define CF_CACR_IEC         (0x00008000) /* Instruction Cache Enable         */
#define CF_CACR_SPA         (0x00004000) /* Search by Physical Address       */
#define CF_CACR_DNFB        (0x00002000) /* Default cache-inhibited fill buf */
#define CF_CACR_IDPI        (0x00001000) /* Instr Disable CPUSHL Invalidate  */
#define CF_CACR_IHLCK       (0x00000800) /* 1/2 Instruction Cache Lock Mode  */
#define CF_CACR_IDCM        (0x00000400) /* Noncacheable Instr default mode  */
#define CF_CACR_ICINVA      (0x00000100) /* Instr Cache Invalidate All       */
#define CF_CACR_EUSP        (0x00000020) /* Switch stacks in user mode       */

#define DCACHE_LINE_SIZE 0x0010     /* bytes per line        */
#define DCACHE_WAY_SIZE  0x2000     /* words per cache block */
#define CACHE_DISABLE_MODE (CF_CACR_DCINVA+CF_CACR_BCINVA+CF_CACR_ICINVA)
#ifdef CONFIG_M5445X_DISABLE_CACHE
/* disable cache for testing rev0 silicon */
#define CACHE_INITIAL_MODE (CF_CACR_EUSP)
#else
#define CACHE_INITIAL_MODE (CF_CACR_DEC+CF_CACR_BEC+CF_CACR_IEC+CF_CACR_EUSP)
#endif

#define _DCACHE_SIZE (2*16384)
#define _ICACHE_SIZE (2*16384)

#define _SET_SHIFT 4

/*
 * Masks for cache sizes.  Programming note: because the set size is a
 * power of two, the mask is also the last address in the set.
 * This may need to be #ifdef for other Coldfire processors.
 */

#define _DCACHE_SET_MASK ((_DCACHE_SIZE/64-1)<<_SET_SHIFT)
#define _ICACHE_SET_MASK ((_ICACHE_SIZE/64-1)<<_SET_SHIFT)
#define LAST_DCACHE_ADDR _DCACHE_SET_MASK
#define LAST_ICACHE_ADDR _ICACHE_SET_MASK


#ifndef __ASSEMBLY__

extern void DcacheFlushInvalidate(void);

extern void DcacheDisable(void);
extern void DcacheEnable(void);

/******************************************************************************/
/*** Unimplemented Cache functionality                                      ***/
/******************************************************************************/
#define preDcacheInvalidateBlockMark()
#define postDcacheInvalidateBlockMark()
#define DcacheZeroBlock(p, l)           fast_bzero((char *)(p), (long)(l))
#define loadDcacheInvalidateBlock()     ASSERT(!"Not Implemented on V4e")
#define IcacheInvalidateBlock()         ASSERT(!"Not Implemented on V4e")

/******************************************************************************/
/*** Redundant Cache functionality on ColdFire                              ***/
/******************************************************************************/
#define DcacheInvalidateBlock(p, l) DcacheFlushInvalidateCacheBlock(p, l)
#define DcacheFlushCacheBlock(p, l) DcacheFlushInvalidateCacheBlock(p, l)
#define DcacheFlushBlock(p, l)      DcacheFlushInvalidateCacheBlock(p, l)

extern void DcacheFlushInvalidateCacheBlock(void *start, unsigned long size);
extern void FLASHDcacheFlushInvalidate(void);

extern void cacr_set(unsigned long x);

#endif /* !__ASSEMBLY__ */

#endif /* CF_CACHE_H */

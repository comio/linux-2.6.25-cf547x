#ifndef _M68K_DELAY_H
#define _M68K_DELAY_H

#include <asm/param.h>

/*
 * Copyright (C) 1994 Hamish Macdonald
 *
 * Delay routines, using a pre-computed "loops_per_jiffy" value.
 */

static inline void __delay(unsigned long loops)
{
#if defined(CONFIG_COLDFIRE)
	/* The coldfire runs this loop at significantly different speeds
	 * depending upon long word alignment or not.  We'll pad it to
	 * long word alignment which is the faster version.
	 * The 0x4a8e is of course a 'tstl %fp' instruction.  This is better
	 * than using a NOP (0x4e71) instruction because it executes in one
	 * cycle not three and doesn't allow for an arbitary delay waiting
	 * for bus cycles to finish.  Also fp/a6 isn't likely to cause a
	 * stall waiting for the register to become valid if such is added
	 * to the coldfire at some stage.
	 */
	__asm__ __volatile__ (".balignw 4, 0x4a8e\n\t"
			      "1: subql #1, %0\n\t"
			      "jcc 1b"
			      : "=d" (loops) : "0" (loops));
#else
	__asm__ __volatile__ ("1: subql #1,%0; jcc 1b"
		: "=d" (loops) : "0" (loops));
#endif
}

extern void __bad_udelay(void);

/*
 * Use only for very small delays ( < 1 msec).  Should probably use a
 * lookup table, really, as the multiplications take much too long with
 * short delays.  This is a "reasonable" implementation, though (and the
 * first constant multiplications gets optimized away if the delay is
 * a constant)
 */
static inline void __const_udelay(unsigned long xloops)
{
#if defined(CONFIG_COLDFIRE)

	__delay(((((unsigned long long) xloops * loops_per_jiffy))>>32)*HZ);
#else
	unsigned long tmp;

	__asm__ ("mulul %2,%0:%1"
		: "=d" (xloops), "=d" (tmp)
		: "d" (xloops), "1" (loops_per_jiffy));
	__delay(xloops * HZ);
#endif
}

static inline void __udelay(unsigned long usecs)
{
	__const_udelay(usecs * 4295);	/* 2**32 / 1000000 */
}

#define udelay(n) (__builtin_constant_p(n) ? \
	((n) > 20000 ? __bad_udelay() : __const_udelay((n) * 4295)) : \
	__udelay(n))

static inline unsigned long muldiv(unsigned long a, unsigned long b,
				   unsigned long c)
{
#if defined(CONFIG_COLDFIRE)
	return (long)(((unsigned long long)a * b)/c);
#else
	unsigned long tmp;

	__asm__ ("mulul %2,%0:%1; divul %3,%0:%1"
		: "=d" (tmp), "=d" (a)
		: "d" (b), "d" (c), "1" (a));
	return a;
#endif
}

#endif /* defined(_M68K_DELAY_H) */

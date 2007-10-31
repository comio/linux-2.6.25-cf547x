/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#ifndef CONFIG_COLDFIRE
#include <asm/uaccess.h>

unsigned long __generic_copy_from_user(void *to, const void __user *from,
				       unsigned long n)
{
	unsigned long tmp, res;

	asm volatile ("\n"
		"	tst.l	%0\n"
		"	jeq	2f\n"
		"1:	moves.l	(%1)+,%3\n"
		"	move.l	%3,(%2)+\n"
		"	subq.l	#1,%0\n"
		"	jne	1b\n"
		"2:	btst	#1,%5\n"
		"	jeq	4f\n"
		"3:	moves.w	(%1)+,%3\n"
		"	move.w	%3,(%2)+\n"
		"4:	btst	#0,%5\n"
		"	jeq	6f\n"
		"5:	moves.b	(%1)+,%3\n"
		"	move.b  %3,(%2)+\n"
		"6:\n"
		"	.section .fixup,\"ax\"\n"
		"	.even\n"
		"10:	move.l	%0,%3\n"
		"7:	clr.l	(%2)+\n"
		"	subq.l	#1,%3\n"
		"	jne	7b\n"
		"	lsl.l	#2,%0\n"
		"	btst	#1,%5\n"
		"	jeq	8f\n"
		"30:	clr.w	(%2)+\n"
		"	addq.l	#2,%0\n"
		"8:	btst	#0,%5\n"
		"	jeq	6b\n"
		"50:	clr.b	(%2)+\n"
		"	addq.l	#1,%0\n"
		"	jra	6b\n"
		"	.previous\n"
		"\n"
		"	.section __ex_table,\"a\"\n"
		"	.align	4\n"
		"	.long	1b,10b\n"
		"	.long	3b,30b\n"
		"	.long	5b,50b\n"
		"	.previous"
		: "=d" (res), "+a" (from), "+a" (to), "=&r" (tmp)
		: "0" (n / 4), "d" (n & 3));

	return res;
}
EXPORT_SYMBOL(__generic_copy_from_user);

unsigned long __generic_copy_to_user(void __user *to, const void *from,
				     unsigned long n)
{
	unsigned long tmp, res;

	asm volatile ("\n"
		"	tst.l	%0\n"
		"	jeq	4f\n"
		"1:	move.l	(%1)+,%3\n"
		"2:	moves.l	%3,(%2)+\n"
		"3:	subq.l	#1,%0\n"
		"	jne	1b\n"
		"4:	btst	#1,%5\n"
		"	jeq	6f\n"
		"	move.w	(%1)+,%3\n"
		"5:	moves.w	%3,(%2)+\n"
		"6:	btst	#0,%5\n"
		"	jeq	8f\n"
		"	move.b	(%1)+,%3\n"
		"7:	moves.b  %3,(%2)+\n"
		"8:\n"
		"	.section .fixup,\"ax\"\n"
		"	.even\n"
		"20:	lsl.l	#2,%0\n"
		"50:	add.l	%5,%0\n"
		"	jra	8b\n"
		"	.previous\n"
		"\n"
		"	.section __ex_table,\"a\"\n"
		"	.align	4\n"
		"	.long	2b,20b\n"
		"	.long	3b,20b\n"
		"	.long	5b,50b\n"
		"	.long	6b,50b\n"
		"	.long	7b,50b\n"
		"	.long	8b,50b\n"
		"	.previous"
		: "=d" (res), "+a" (from), "+a" (to), "=&r" (tmp)
		: "0" (n / 4), "d" (n & 3));

	return res;
}
EXPORT_SYMBOL(__generic_copy_to_user);

/*
 * Copy a null terminated string from userspace.
 */
long strncpy_from_user(char *dst, const char __user *src, long count)
{
	long res;
	char c;

	if (count <= 0)
		return count;

	asm volatile ("\n"
		"1:	moves.b	(%2)+,%4\n"
		"	move.b	%4,(%1)+\n"
		"	jeq	2f\n"
		"	subq.l	#1,%3\n"
		"	jne	1b\n"
		"2:	sub.l	%3,%0\n"
		"3:\n"
		"	.section .fixup,\"ax\"\n"
		"	.even\n"
		"10:	move.l	%5,%0\n"
		"	jra	3b\n"
		"	.previous\n"
		"\n"
		"	.section __ex_table,\"a\"\n"
		"	.align	4\n"
		"	.long	1b,10b\n"
		"	.previous"
		: "=d" (res), "+a" (dst), "+a" (src), "+r" (count), "=&d" (c)
		: "i" (-EFAULT), "0" (count));

	return res;
}
EXPORT_SYMBOL(strncpy_from_user);

/*
 * Return the size of a string (including the ending 0)
 *
 * Return 0 on exception, a value greater than N if too long
 */
long strnlen_user(const char __user *src, long n)
{
	char c;
	long res;

	asm volatile ("\n"
		"1:	subq.l	#1,%1\n"
		"	jmi	3f\n"
		"2:	moves.b	(%0)+,%2\n"
		"	tst.b	%2\n"
		"	jne	1b\n"
		"	jra	4f\n"
		"\n"
		"3:	addq.l	#1,%0\n"
		"4:	sub.l	%4,%0\n"
		"5:\n"
		"	.section .fixup,\"ax\"\n"
		"	.even\n"
		"20:	sub.l	%0,%0\n"
		"	jra	5b\n"
		"	.previous\n"
		"\n"
		"	.section __ex_table,\"a\"\n"
		"	.align	4\n"
		"	.long	2b,20b\n"
		"	.previous\n"
		: "=&a" (res), "+d" (n), "=&d" (c)
		: "0" (src), "r" (src));

	return res;
}
EXPORT_SYMBOL(strnlen_user);

/*
 * Zero Userspace
 */

unsigned long __clear_user(void __user *to, unsigned long n)
{
	unsigned long res;

	asm volatile ("\n"
		"	tst.l	%0\n"
		"	jeq	3f\n"
		"1:	moves.l	%2,(%1)+\n"
		"2:	subq.l	#1,%0\n"
		"	jne	1b\n"
		"3:	btst	#1,%4\n"
		"	jeq	5f\n"
		"4:	moves.w	%2,(%1)+\n"
		"5:	btst	#0,%4\n"
		"	jeq	7f\n"
		"6:	moves.b	%2,(%1)\n"
		"7:\n"
		"	.section .fixup,\"ax\"\n"
		"	.even\n"
		"10:	lsl.l	#2,%0\n"
		"40:	add.l	%4,%0\n"
		"	jra	7b\n"
		"	.previous\n"
		"\n"
		"	.section __ex_table,\"a\"\n"
		"	.align	4\n"
		"	.long	1b,10b\n"
		"	.long	2b,10b\n"
		"	.long	4b,40b\n"
		"	.long	5b,40b\n"
		"	.long	6b,40b\n"
		"	.long	7b,40b\n"
		"	.previous"
		: "=d" (res), "+a" (to)
		: "r" (0), "0" (n / 4), "d" (n & 3));

    return res;
}
EXPORT_SYMBOL(__clear_user);

#else /* CONFIG_COLDFIRE */

#include <asm/cf_uaccess.h>

unsigned long __generic_copy_from_user(void *to, const void *from,
		unsigned long n)
{
    unsigned long tmp;
    __asm__ __volatile__
	("   tstl %2\n"
	 "   jeq 2f\n"
	 "1: movel (%1)+,%3\n"
	 "   movel %3,(%0)+\n"
	 "   subql #1,%2\n"
	 "   jne 1b\n"
	 "2: movel %4,%2\n"
	 "   bclr #1,%2\n"
	 "   jeq 4f\n"
	 "3: movew (%1)+,%3\n"
	 "   movew %3,(%0)+\n"
	 "4: bclr #0,%2\n"
	 "   jeq 6f\n"
	 "5: moveb (%1)+,%3\n"
	 "   moveb %3,(%0)+\n"
	 "6:\n"
	 ".section .fixup,\"ax\"\n"
	 "   .even\n"
	 "7: movel %2,%%d0\n"
	 "71:clrl (%0)+\n"
	 "   subql #1,%%d0\n"
	 "   jne 71b\n"
	 "   lsll #2,%2\n"
	 "   addl %4,%2\n"
	 "   btst #1,%4\n"
	 "   jne 81f\n"
	 "   btst #0,%4\n"
	 "   jne 91f\n"
	 "   jra 6b\n"
	 "8: addql #2,%2\n"
	 "81:clrw (%0)+\n"
	 "   btst #0,%4\n"
	 "   jne 91f\n"
	 "   jra 6b\n"
	 "9: addql #1,%2\n"
	 "91:clrb (%0)+\n"
	 "   jra 6b\n"
	 ".previous\n"
	 ".section __ex_table,\"a\"\n"
	 "   .align 4\n"
	 "   .long 1b,7b\n"
	 "   .long 3b,8b\n"
	 "   .long 5b,9b\n"
	 ".previous"
	 : "=a"(to), "=a"(from), "=d"(n), "=&d"(tmp)
	 : "d"(n & 3), "0"(to), "1"(from), "2"(n/4)
	 : "d0", "memory");
    return n;
}
EXPORT_SYMBOL(__generic_copy_from_user);


unsigned long __generic_copy_to_user(void *to, const void *from,
		unsigned long n)
{
    unsigned long tmp;
    __asm__ __volatile__
	("   tstl %2\n"
	 "   jeq 3f\n"
	 "1: movel (%1)+,%3\n"
	 "22:movel %3,(%0)+\n"
	 "2: subql #1,%2\n"
	 "   jne 1b\n"
	 "3: movel %4,%2\n"
	 "   bclr #1,%2\n"
	 "   jeq 4f\n"
	 "   movew (%1)+,%3\n"
	 "24:movew %3,(%0)+\n"
	 "4: bclr #0,%2\n"
	 "   jeq 5f\n"
	 "   moveb (%1)+,%3\n"
	 "25:moveb %3,(%0)+\n"
	 "5:\n"
	 ".section .fixup,\"ax\"\n"
	 "   .even\n"
	 "60:addql #1,%2\n"
	 "6: lsll #2,%2\n"
	 "   addl %4,%2\n"
	 "   jra 5b\n"
	 "7: addql #2,%2\n"
	 "   jra 5b\n"
	 "8: addql #1,%2\n"
	 "   jra 5b\n"
	 ".previous\n"
	 ".section __ex_table,\"a\"\n"
	 "   .align 4\n"
	 "   .long 1b,60b\n"
	 "   .long 22b,6b\n"
	 "   .long 2b,6b\n"
	 "   .long 24b,7b\n"
	 "   .long 3b,60b\n"
	 "   .long 4b,7b\n"
	 "   .long 25b,8b\n"
	 "   .long 5b,8b\n"
	 ".previous"
	 : "=a"(to), "=a"(from), "=d"(n), "=&d"(tmp)
	 : "r"(n & 3), "0"(to), "1"(from), "2"(n / 4)
	 : "memory");
    return n;
}
EXPORT_SYMBOL(__generic_copy_to_user);

/*
 * Copy a null terminated string from userspace.
 */

long strncpy_from_user(char *dst, const char *src, long count)
{
	long res = -EFAULT;
	if (!(access_ok(VERIFY_READ, src, 1))) /* --tym-- */
		return res;
    if (count == 0) return count;
    __asm__ __volatile__
	("1: moveb (%2)+,%%d0\n"
	 "12:moveb %%d0,(%1)+\n"
	 "   jeq 2f\n"
	 "   subql #1,%3\n"
	 "   jne 1b\n"
	 "2: subl %3,%0\n"
	 "3:\n"
	 ".section .fixup,\"ax\"\n"
	 "   .even\n"
	 "4: movel %4,%0\n"
	 "   jra 3b\n"
	 ".previous\n"
	 ".section __ex_table,\"a\"\n"
	 "   .align 4\n"
	 "   .long 1b,4b\n"
	 "   .long 12b,4b\n"
	 ".previous"
	 : "=d"(res), "=a"(dst), "=a"(src), "=d"(count)
	 : "i"(-EFAULT), "0"(count), "1"(dst), "2"(src), "3"(count)
	 : "d0", "memory");
    return res;
}
EXPORT_SYMBOL(strncpy_from_user);

/*
 * Return the size of a string (including the ending 0)
 *
 * Return 0 on exception, a value greater than N if too long
 */
long strnlen_user(const char *src, long n)
{
    long res = -EFAULT;
    if (!(access_ok(VERIFY_READ, src, 1))) /* --tym-- */
	return res;

	res = -(long)src;
	__asm__ __volatile__
		("1:\n"
		 "   tstl %2\n"
		 "   jeq 3f\n"
		 "2: moveb (%1)+,%%d0\n"
		 "22:\n"
		 "   subql #1,%2\n"
		 "   tstb %%d0\n"
		 "   jne 1b\n"
		 "   jra 4f\n"
		 "3:\n"
		 "   addql #1,%0\n"
		 "4:\n"
		 "   addl %1,%0\n"
		 "5:\n"
		 ".section .fixup,\"ax\"\n"
		 "   .even\n"
		 "6: moveq %3,%0\n"
		 "   jra 5b\n"
		 ".previous\n"
		 ".section __ex_table,\"a\"\n"
		 "   .align 4\n"
		 "   .long 2b,6b\n"
		 "   .long 22b,6b\n"
		 ".previous"
		 : "=d"(res), "=a"(src), "=d"(n)
		 : "i"(0), "0"(res), "1"(src), "2"(n)
		 : "d0");
	return res;
}
EXPORT_SYMBOL(strnlen_user);


/*
 * Zero Userspace
 */

unsigned long __clear_user(void *to, unsigned long n)
{
    __asm__ __volatile__
	("   tstl %1\n"
	 "   jeq 3f\n"
	 "1: movel %3,(%0)+\n"
	 "2: subql #1,%1\n"
	 "   jne 1b\n"
	 "3: movel %2,%1\n"
	 "   bclr #1,%1\n"
	 "   jeq 4f\n"
	 "24:movew %3,(%0)+\n"
	 "4: bclr #0,%1\n"
	 "   jeq 5f\n"
	 "25:moveb %3,(%0)+\n"
	 "5:\n"
	 ".section .fixup,\"ax\"\n"
	 "   .even\n"
	 "61:addql #1,%1\n"
	 "6: lsll #2,%1\n"
	 "   addl %2,%1\n"
	 "   jra 5b\n"
	 "7: addql #2,%1\n"
	 "   jra 5b\n"
	 "8: addql #1,%1\n"
	 "   jra 5b\n"
	 ".previous\n"
	 ".section __ex_table,\"a\"\n"
	 "   .align 4\n"
	 "   .long 1b,61b\n"
	 "   .long 2b,6b\n"
	 "   .long 3b,61b\n"
	 "   .long 24b,7b\n"
	 "   .long 4b,7b\n"
	 "   .long 25b,8b\n"
	 "   .long 5b,8b\n"
	 ".previous"
	 : "=a"(to), "=d"(n)
	 : "r"(n & 3), "d"(0), "0"(to), "1"(n/4));
    return n;
}
EXPORT_SYMBOL(__clear_user);

#endif /* CONFIG_COLDFIRE */


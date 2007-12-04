/*
 *  linux/arch/m68k/coldfire/cache.c
 *
 *  Matt Waddel Matt.Waddel@freescale.com
 *  Kurt Mahan kmahan@freescale.com
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

/* Cache Control Reg shadow reg */
unsigned long shadow_cacr;

/**
 * cacr_set - Set the Cache Control Register
 * @x Value to set
 */
void cacr_set(unsigned long x)
{
	shadow_cacr = x;

	__asm__ __volatile__ ("movec %0, %%cacr"
			      : /* no outputs */
			      : "r" (shadow_cacr));
}

/**
 * cacr_get - Get the current value of the Cache Control Register
 *
 * @return CACR value
 */
unsigned long cacr_get(void)
{
	return shadow_cacr;
}

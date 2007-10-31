
#ifndef M68K_PGALLOC_H
#define M68K_PGALLOC_H

#include <linux/mm.h>
#include <linux/highmem.h>
#include <asm/setup.h>
#include <asm/virtconvert.h>


#if defined(CONFIG_SUN3)
#include <asm/sun3_pgalloc.h>
#elif defined(CONFIG_COLDFIRE)
#include <asm/cf_pgalloc.h>
#else
#include <asm/motorola_pgalloc.h>
#endif

extern void m68k_setup_node(int node);

#endif /* M68K_PGALLOC_H */

/*
 *  linux/arch/m68k/coldifre/config.c
 *
 *  Matt Waddel Matt.Waddel@freescale.com
 *  Copyright Freescale Semiconductor, Inc. 2007
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/bootmem.h>
#include <linux/mm.h>
#include <asm/bootinfo.h>
#include <asm/machdep.h>
#include <asm/coldfire.h>
#include <asm/cfcache.h>
#include <asm/cacheflush.h>
#include <asm/bootinfo.h>
#include <asm/io.h>
#include <asm/cfmmu.h>
#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/traps.h>
#include <asm/movs.h>
#include <asm/movs.h>
#include <asm/page.h>
#include <asm/pgalloc.h>
#include <asm/mcf5445x_intc.h>
#include <asm/mcf5445x_sdramc.h>
#include <asm/mcf5445x_fbcs.h>
#include <asm/mcf5445x_dtim.h>

/* JKM -- testing */
#include <linux/pfn.h>
/* JKM */

extern int get_irq_list(struct seq_file *p, void *v);
extern char _text, _end;
extern char _etext, _edata, __init_begin, __init_end;
extern struct console mcfrs_console;
extern char m68k_command_line[CL_SIZE];
extern unsigned long availmem;

static int irq_enable[NR_IRQS];
unsigned long num_pages;

void coldfire_sort_memrec(void)
{
	int i, j;

	/* Sort the m68k_memory records by address */
	for (i = 0; i < m68k_num_memory; ++i) {
		for (j = i + 1; j < m68k_num_memory; ++j) {
			if (m68k_memory[i].addr > m68k_memory[j].addr) {
				struct mem_info tmp;
				tmp = m68k_memory[i];
				m68k_memory[i] = m68k_memory[j];
				m68k_memory[j] = tmp;
			}
		}
	}
	/* Trim off discontiguous bits */
	for (i = 1; i < m68k_num_memory; ++i) {
		if ((m68k_memory[i-1].addr + m68k_memory[i-1].size) !=
			m68k_memory[i].addr) {
			printk(KERN_DEBUG "m68k_parse_bootinfo: addr gap between \
				0x%lx & 0x%lx\n",
				m68k_memory[i-1].addr+m68k_memory[i-1].size,
				m68k_memory[i].addr);
			m68k_num_memory = i;
			break;
		}
	}
}

int __init uboot_commandline(char *bootargs)
{
	int len = 0, cmd_line_len;
	static struct uboot_record uboot_info;

	extern unsigned long uboot_info_stk;

	/* Add 0x80000000 to get post-remapped kernel memory location */
	uboot_info.bd_info = (*(u32 *)(uboot_info_stk)) + 0x80000000;
	uboot_info.initrd_start = (*(u32 *)(uboot_info_stk+4)) + 0x80000000;
	uboot_info.initrd_end = (*(u32 *)(uboot_info_stk+8)) + 0x80000000;
	uboot_info.cmd_line_start = (*(u32 *)(uboot_info_stk+12)) + 0x80000000;
	uboot_info.cmd_line_stop = (*(u32 *)(uboot_info_stk+16)) + 0x80000000;

	cmd_line_len = uboot_info.cmd_line_stop - uboot_info.cmd_line_start;
	if ((cmd_line_len > 0) && (cmd_line_len < CL_SIZE-1))
		len = (int)strncpy(bootargs, (char *)uboot_info.cmd_line_start,\
				   cmd_line_len);

	return len;
}

/*
 * This routine does things not done in the bootloader.
 */
#define DEFAULT_COMMAND_LINE "root=/dev/mtdblock1 rw rootfstype=jffs2 ip=none mtdparts=physmap-flash.0:5M(kernel)ro,-(jffs2)"
asmlinkage void __init cf_early_init(void)
{
	struct bi_record *record = (struct bi_record *) &_end;

	extern char _end;

	SET_VBR((void *)MCF_RAMBAR1);

	/* Mask all interrupts */
	MCF_INTC0_IMRL = 0xFFFFFFFF;
	MCF_INTC0_IMRH = 0xFFFFFFFF;
	MCF_INTC1_IMRL = 0xFFFFFFFF;
	MCF_INTC1_IMRH = 0xFFFFFFFF;

#if defined(CONFIG_NOR_FLASH_BASE)
	MCF_FBCS_CSAR(1) = CONFIG_NOR_FLASH_BASE;
#else
	MCF_FBCS_CSAR(1) = 0x00000000;
#endif

#if CONFIG_SDRAM_SIZE > (256*1024*1024)
	/* Init optional SDRAM chip select */
	MCF_SDRAMC_SDCS(1) = (256*1024*1024) | 0x1B;
#endif

	m68k_machtype = MACH_CFMMU;
	m68k_fputype = FPU_CFV4E;
	m68k_mmutype = MMU_CFV4E;
	m68k_cputype = CPU_CFV4E;

	m68k_num_memory = 0;
	m68k_memory[m68k_num_memory].addr = CONFIG_SDRAM_BASE;
	m68k_memory[m68k_num_memory++].size = CONFIG_SDRAM_SIZE;

	if (!uboot_commandline(m68k_command_line)) {
#if defined(CONFIG_BOOTPARAM)
		strncpy(m68k_command_line, CONFIG_BOOTPARAM_STRING, CL_SIZE-1);
#else
		strcpy(m68k_command_line, DEFAULT_COMMAND_LINE);
#endif
	}


#if defined(CONFIG_BLK_DEV_INITRD)
	/* add initrd image */
	record = (struct bi_record *) ((void *)record + record->size);
	record->tag = BI_RAMDISK;
	record->size =  sizeof(record->tag) + sizeof(record->size)
		+ sizeof(record->data[0]) + sizeof(record->data[1]);
#endif

	/* Mark end of tags. */
	record = (struct bi_record *) ((void *) record + record->size);
	record->tag = 0;
	record->data[0] = 0;
	record->data[1] = 0;
	record->size = sizeof(record->tag) + sizeof(record->size)
		+ sizeof(record->data[0]) + sizeof(record->data[1]);

	/* Invalidate caches via CACR */
	flush_bcache();
	cacr_set(CACHE_DISABLE_MODE);

	/* Turn on caches via CACR, enable EUSP */
	cacr_set(CACHE_INITIAL_MODE);
}

void settimericr(unsigned int timer, unsigned int level)
{
	volatile unsigned char *icrp;
	unsigned int icr;
	unsigned char irq;

	if (timer <= 2) {
		switch (timer) {
		case 2:  irq = 33; icr = MCFSIM_ICR_TIMER2; break;
		default: irq = 32; icr = MCFSIM_ICR_TIMER1; break;
		}

		icrp = (volatile unsigned char *) (icr);
		*icrp = level;
		coldfire_enable_irq0(irq);
	}
}

/* Assembler routines */
asmlinkage void buserr(void);
asmlinkage void trap(void);
asmlinkage void system_call(void);
asmlinkage void inthandler(void);

void __init coldfire_trap_init(void)
{
	int i = 0;
	e_vector *vectors;

	vectors = (e_vector *)MCF_RAMBAR1;
	/*
	 * There is a common trap handler and common interrupt
	 * handler that handle almost every vector. We treat
	 * the system call and bus error special, they get their
	 * own first level handlers.
	 */
	for (i = 3; (i <= 23); i++)
		vectors[i] = trap;
	for (i = 33; (i <= 63); i++)
		vectors[i] = trap;
	for (i = 24; (i <= 31); i++)
		vectors[i] = inthandler;
	for (i = 64; (i < 255); i++)
		vectors[i] = inthandler;

	vectors[255] = 0;
	vectors[2] = buserr;
	vectors[32] = system_call;
}

void coldfire_tick(void)
{
	/* Reset the ColdFire timer */
	__raw_writeb(MCF_DTIM_DTER_CAP | MCF_DTIM_DTER_REF, MCF_DTIM0_DTER);
}

void __init coldfire_sched_init(irq_handler_t handler)
{
	unsigned int	mcf_timerlevel = 5;
	unsigned int	mcf_timervector = 64+32;

	__raw_writew(MCF_DTIM_DTMR_RST_RST, MCF_DTIM0_DTMR);
	__raw_writel(((MCF_BUSCLK / 16) / HZ), MCF_DTIM0_DTRR);
	__raw_writew(MCF_DTIM_DTMR_ORRI	| MCF_DTIM_DTMR_CLK_DIV16 |
		     MCF_DTIM_DTMR_FRR	| MCF_DTIM_DTMR_RST_EN, \
		     MCF_DTIM0_DTMR);

	request_irq(mcf_timervector, handler, SA_INTERRUPT, \
		    "timer", (void *)MCF_DTIM0_DTMR);

	settimericr(1, mcf_timerlevel);
}

int timerirqpending(int timer)
{
	unsigned int imr = 0;

	switch (timer) {
	case 1:  imr = 0x1; break;
	case 2:  imr = 0x2; break;
	default: break;
	}

	return (getiprh() & imr);
}

unsigned long coldfire_gettimeoffset(void)
{
	volatile unsigned long trr, tcn, offset;

	tcn = __raw_readw(MCF_DTIM0_DTCN);
	trr = __raw_readl(MCF_DTIM0_DTRR);
	offset = (tcn * (1000000 / HZ)) / trr;

	/* Check if we just wrapped the counters and maybe missed a tick */
	if ((offset < (1000000 / HZ / 2)) && timerirqpending(1))
		offset += 1000000 / HZ;
	return offset;
}

void coldfire_reboot(void)
{
	/* disable interrupts and do a software reset */
	asm("movew #0x2700, %%sr\n\t"
	    "moveb #0x80, %%d0\n\t"
	    "moveb %%d0, 0xfc0a0000\n\t"
	    : : : "%d0");
}

/* int coldfire_hwclk(int i, struct rtc_time *t)
{
	printk ("Real time clock needs porting.\n");
	return 0;
}*/

static void coldfire_get_model(char *model)
{
	sprintf(model, "Version 4 ColdFire");
}

void coldfire_enable_irq(unsigned int vec)
{
	unsigned long flags;

	vec -= 64;

	if (((int)vec < 0) || (vec > 63)) {
		printk(KERN_WARNING "enable_irq %d failed\n", vec);
		return;
	}

	local_irq_save(flags);
	irq_enable[vec]++;
	if (vec < 32)
		MCF_INTC0_IMRL &= ~(1 << vec);
	else
		MCF_INTC0_IMRH &= ~(1 << (vec - 32));
	local_irq_restore(flags);
}

void coldfire_disable_irq(unsigned int vec)
{
	unsigned long flags;

	vec -= 64;

	if (((int)vec < 0) || (vec > 63)) {
		printk(KERN_WARNING "disable_irq %d failed\n", vec);
		return;
	}

	local_irq_save(flags);
	if (--irq_enable[vec] == 0) {
		if (vec < 32)
			MCF_INTC0_IMRL |= (1 << vec);
		else
			MCF_INTC0_IMRH |= (1 << (vec - 32));

	}
	local_irq_restore(flags);
}

static void __init
coldfire_bootmem_alloc(unsigned long memory_start, unsigned long memory_end)
{
	unsigned long base_pfn;

	/* compute total pages in system */
	num_pages = PAGE_ALIGN(memory_end - PAGE_OFFSET) >> PAGE_SHIFT;

	/* align start/end to page boundries */
	memory_start = PAGE_ALIGN(memory_start);
	memory_end = memory_end & PAGE_MASK;

	/* page numbers */
	base_pfn = __pa(PAGE_OFFSET) >> PAGE_SHIFT;
	min_low_pfn = __pa(memory_start) >> PAGE_SHIFT;
	max_low_pfn = __pa(memory_end) >> PAGE_SHIFT;

	high_memory = (void *)memory_end;
	availmem = memory_start;

	/* setup bootmem data */
	m68k_setup_node(0);
	availmem += init_bootmem_node(NODE_DATA(0), min_low_pfn,
		base_pfn, max_low_pfn);
	availmem = PAGE_ALIGN(availmem);
	free_bootmem(__pa(availmem), memory_end - (availmem));
}

void __init config_coldfire(void)
{
	unsigned long endmem, startmem;
	int i;

	/*
	 * Calculate endmem from m68k_memory, assume all are contiguous
	 */
	startmem = ((((int) &_end) + (PAGE_SIZE - 1)) & PAGE_MASK);
	endmem = PAGE_OFFSET;
	for (i = 0; i < m68k_num_memory; ++i)
		endmem += m68k_memory[i].size;

	printk(KERN_INFO "starting up linux startmem 0x%lx, endmem 0x%lx, \
		size %luMB\n", startmem,  endmem, (endmem - startmem) >> 20);

	memset(irq_enable, 0, sizeof(irq_enable));

	/*
	 * Setup coldfire mach-specific handlers
	 */
	mach_max_dma_address 	= 0xffffffff;
	mach_sched_init 	= coldfire_sched_init;
	mach_tick		= coldfire_tick;
	mach_gettimeoffset 	= coldfire_gettimeoffset;
	mach_reset 		= coldfire_reboot;
/*	mach_hwclk 		= coldfire_hwclk; to be done */
	mach_get_model 		= coldfire_get_model;

	coldfire_bootmem_alloc(startmem, endmem);

	/*
	 * initrd setup
	 */
/* #ifdef CONFIG_BLK_DEV_INITRD
	if (m68k_ramdisk.size)  {
		reserve_bootmem (__pa(m68k_ramdisk.addr), m68k_ramdisk.size);
		initrd_start = (unsigned long) m68k_ramdisk.addr;
		initrd_end = initrd_start + m68k_ramdisk.size;
		printk (KERN_DEBUG "initrd: %08lx - %08lx\n", initrd_start,
			initrd_end);
	}
#endif */

#if defined(CONFIG_DUMMY_CONSOLE) || defined(CONFIG_FRAMEBUFFER_CONSOLE)
	conswitchp = &dummy_con;
#endif

#if defined(CONFIG_SERIAL_COLDFIRE)
	/*
	 * This causes trouble when it is re-registered later.
	 * Currently this is fixed by conditionally commenting
	 * out the register_console in mcf_serial.c
	 */
	register_console(&mcfrs_console);
#endif
}

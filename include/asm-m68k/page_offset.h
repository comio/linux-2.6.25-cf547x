
/* This handles the memory map.. */
#if !defined(CONFIG_SUN3) && !defined(CONFIG_COLDFIRE)
#define PAGE_OFFSET_RAW		0x00000000
#elif defined(CONFIG_SUN3)
#define PAGE_OFFSET_RAW		0x0E000000
#else /* CONFIG_COLDFIRE */
#define PAGE_OFFSET_RAW		0xC0000000
#define	PHYS_OFFSET		0x40000000
#endif


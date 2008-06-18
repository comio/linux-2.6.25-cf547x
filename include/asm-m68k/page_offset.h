/*
 * Page and physical memory maps.
 */
#if defined(CONFIG_SUN3)
#define PAGE_OFFSET_RAW		0x0E000000

#elif defined(CONFIG_M5445X) || defined(CONFIG_M547X_8X)
#define	PHYS_OFFSET		CONFIG_SDRAM_BASE
#define PAGE_OFFSET_RAW		(PHYS_OFFSET)
/* #define PAGE_OFFSET_RAW		0xC0000000 */

#else
/* default */
#define PAGE_OFFSET_RAW		0x00000000
#endif

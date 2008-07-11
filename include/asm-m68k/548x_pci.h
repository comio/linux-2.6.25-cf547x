#ifndef _ASM_M68K_548X_PCI_H
#define _ASM_M68K_548X_PCI_H

/*
 * asm-m68k/pci_m68k.h - m68k specific PCI declarations.
 *
 * Written by Wout Klaren.
 */
#include <linux/mm.h>
#include <asm/scatterlist.h>

#include <asm-generic/pci.h>

struct pci_ops;

/*
 * Structure with hardware dependent information and functions of the
 * PCI bus.
 */

struct pci_bus_info
{
	/*
	 * Resources of the PCI bus.
	 */

	struct resource mem_space;
	struct resource io_space;

	/*
	 * System dependent functions.
	 */

	struct pci_ops *m68k_pci_ops;

	void (*fixup)(int pci_modify);
	void (*conf_device)(struct pci_dev *dev);
};

#define pcibios_assign_all_busses()	0
#define pcibios_scan_all_fns(a, b)	0

static inline void pcibios_set_master(struct pci_dev *dev)
{
	/* No special bus mastering setup handling */
}

static inline void pcibios_penalize_isa_irq(int irq)
{
	/* We don't do dynamic PCI IRQ allocation */
}

#ifndef CONFIG_COLDFIRE
/* The PCI address space does equal the physical memory
 * address space.  The networking and block device layers use
 * this boolean for bounce buffer decisions.
 */
#define PCI_DMA_BUS_IS_PHYS	(1)

#define PCIBIOS_MIN_IO 		0x00004000
#define PCIBIOS_MIN_MEM 	0x04000000

#else /* !CONFIG_COLDFIRE */
#include <asm-generic/pci-dma-compat.h>
#define PCI_DMA_BASE 		/*0x40000000*/0	/* PCI-DMA window base 		*/

extern struct pci_bus_info *__init init_coldfire_pci(void);
extern void * pci_alloc_son(struct pci_dev *, size_t,
                            dma_addr_t *, int);
/*
 * The PCI address space equal the virtual memory
 * address space on m547X/m548X.
 */
#define PCI_DMA_BUS_IS_PHYS	(1)

#define PCIBIOS_MIN_IO 		0x00000100
#define PCIBIOS_MIN_MEM 	0x02000000

struct scatterlist;


/* This is always fine. */
#define pci_dac_dma_supported(pci_dev, mask)	(1)


/* These macros should be used after a pci_map_sg call has been done
 * to get bus addresses of each of the SG entries and their lengths.
 * You should only work with the number of sg entries pci_map_sg
 * returns.
 */
#define sg_dma_address(sg)	((sg)->dma_address)
#define sg_dma_len(sg)		((sg)->length)

static inline void pcibios_align_resource(void *data, struct resource *res, unsigned long size, unsigned long align)
{
}

#endif /* !CONFIG_COLDFIRE*/
#endif /* _ASM_M68K_548X_PCI_H */

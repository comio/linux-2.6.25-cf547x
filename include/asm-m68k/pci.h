/*
 * asm-m68k/pci.h - m68k specific PCI declarations.
 *
 * Coldfire Implementation Copyright (c) 2007 Freescale Semiconductor, Inc.
 *	Kurt Mahan <kmahan@freescale.com>
 */
#ifndef _ASM_M68K_PCI_H
#define _ASM_M68K_PCI_H

#ifndef CONFIG_PCI
/*
 * The PCI address space does equal the physical memory
 * address space.  The networking and block device layers use
 * this boolean for bounce buffer decisions.
 */
#define PCI_DMA_BUS_IS_PHYS		(1)
#else
#include <asm-generic/pci-dma-compat.h>

/*
 * The PCI address space does equal the physical memory
 * address space.  The networking and block device layers use
 * this boolean for bounce buffer decisions.
 */
#define PCI_DMA_BUS_IS_PHYS		(1)

#define PCIBIOS_MIN_IO			0x00004000
#define PCIBIOS_MIN_MEM			0x02000000

#define pcibios_assign_all_busses()	0
#define pcibios_scan_all_fns(a, b)	0

static inline void
pcibios_set_master(struct pci_dev *dev)
{
	/* no special bus mastering setup handling */
}

static inline void
pcibios_penalize_isa_irq(int irq, int active)
{
	/* no dynamic PCI IRQ allocation */
}

#if 0
static inline void
pcibios_add_platform_entries(struct pci_dev *dev)
{
	/* no special handling */
}
#endif

static inline void
pcibios_resource_to_bus(struct pci_dev *dev, struct pci_bus_region *region,
			 struct resource *res)
{
#ifdef CONFIG_M54455
	if ((res->start == 0xa0000000) || (res->start == 0xa8000000)) {
		/* HACK!  FIX! kludge to fix bridge mapping */
		region->start = res->start & 0x0fffffff;
		region->end = res->end & 0x0fffffff;
	} else {
		region->start = res->start;
		region->end = res->end;
	}
#else
	region->start = res->start;
	region->end = res->end;
#endif
}

static inline void
pcibios_bus_to_resource(struct pci_dev *dev, struct resource *res,
			struct pci_bus_region *region)
{
	res->start = region->start;
	res->end = region->end;
}

static inline struct resource *
pcibios_select_root(struct pci_dev *pdev, struct resource *res)
{
	struct resource *root = NULL;

	if (res->flags & IORESOURCE_IO)
		root = &ioport_resource;
	if (res->flags & IORESOURCE_MEM)
		root = &iomem_resource;

	return root;
}

#endif /* CONFIG_PCI */
#endif /* _ASM_M68K_PCI_H */

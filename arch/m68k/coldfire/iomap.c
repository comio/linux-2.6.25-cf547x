/*
 * arch/m68k/coldfire/iomap.c
 *
 * Generic coldfire iomap interface
 *
 * Based on the sh64 iomap.c by Paul Mundt.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/pci.h>
#include <asm/io.h>

void __iomem *__attribute__ ((weak))
ioport_map(unsigned long port, unsigned int len)
{
	return (void __iomem *)port;
}
EXPORT_SYMBOL(pci_iomap);

void ioport_unmap(void __iomem *addr)
{
	/* Nothing .. */
}
EXPORT_SYMBOL(pci_iounmap);

void __iomem *pci_iomap(struct pci_dev *dev, int bar, unsigned long max)
{
	unsigned long start = pci_resource_start(dev, bar);
	unsigned long len = pci_resource_len(dev, bar);
	unsigned long flags = pci_resource_flags(dev, bar);
printk(KERN_INFO "PCI_IOMAP: BAR=%d  START=0x%lx  LEN=0x%lx  FLAGS=0x%lx\n",
       bar, start, len, flags);

	if (!len)
		return NULL;
	if (max && len > max)
		len = max;
	if (flags & IORESOURCE_IO)
		return ioport_map(start, len);
	if (flags & IORESOURCE_MEM)
		return (void __iomem *)start;

	/* What? */
	return NULL;
}
EXPORT_SYMBOL(ioport_map);

void pci_iounmap(struct pci_dev *dev, void __iomem *addr)
{
	/* Nothing .. */
}
EXPORT_SYMBOL(ioport_unmap);

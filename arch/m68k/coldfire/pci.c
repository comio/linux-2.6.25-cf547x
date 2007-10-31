/*
 * linux/arch/m68k/coldfire/pci.c
 *
 * PCI initialization for Coldfire architectures.
 *
 * Currently Supported:
 *	M5445x
 *
 * Copyright (c) 2007 Freescale Semiconductor, Inc.
 *	Kurt Mahan <kmahan@freescale.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>

#include <asm/mcfsim.h>
#include <asm/pci.h>

/* pci ops for reading/writing config */
struct pci_raw_ops *raw_pci_ops;

/* pci debug flag */
static int debug_pci;

#ifdef CONFIG_M54455
extern int init_mcf5445x_pci(void);
extern void mcf5445x_conf_device(struct pci_dev *dev);
extern void mcf5445x_pci_dumpregs(void);

extern struct resource pci_ioport_resource;
extern struct resource pci_iomem_resource;
#endif

static int
pci_read(struct pci_bus *bus, unsigned int devfn, int where,
	 int size, u32 *value)
{
	return raw_pci_ops->read(0, bus->number, devfn, where, size, value);
}

static int
pci_write(struct pci_bus *bus, unsigned int devfn, int where,
	  int size, u32 value)
{
	return raw_pci_ops->write(0, bus->number, devfn, where, size, value);
}

struct pci_ops pci_root_ops = {
	.read = pci_read,
	.write = pci_write,
};

/*
 * pcibios_setup(char *)
 *
 * Initialize the pcibios based on cmd line params.
 */
char * __init
pcibios_setup(char *str)
{
	if (!strcmp(str, "debug")) {
		debug_pci = 1;
		return NULL;
	}
	return str;
}

/*
 * We need to avoid collisions with `mirrored' VGA ports
 * and other strange ISA hardware, so we always want the
 * addresses to be allocated in the 0x000-0x0ff region
 * modulo 0x400.
 *
 * Why? Because some silly external IO cards only decode
 * the low 10 bits of the IO address. The 0x00-0xff region
 * is reserved for motherboard devices that decode all 16
 * bits, so it's ok to allocate at, say, 0x2800-0x28ff,
 * but we want to try to avoid allocating at 0x2900-0x2bff
 * which might have be mirrored at 0x0100-0x03ff..
 */
void
pcibios_align_resource(void *data, struct resource *res, resource_size_t size,
		       resource_size_t align)
{
	struct pci_dev *dev = data;

	if (res->flags & IORESOURCE_IO) {
		resource_size_t start = res->start;

		if (size > 0x100)
			printk(KERN_ERR "PCI: I/O Region %s/%d too large"
			       " (%ld bytes)\n", pci_name(dev),
			       dev->resource - res, (long int)size);

		if (start & 0x300) {
			start = (start + 0x3ff) & ~0x3ff;
			res->start = start;
		}
	}
}

/*
 * Swizzle the device pin each time we cross a bridge
 * and return the slot number.
 */
static u8 __devinit
pcibios_swizzle(struct pci_dev *dev, u8 *pin)
{
	return 0;
}

/*
 * Map a slot/pin to an IRQ.
 */
static int
pcibios_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	return 0x43;
}

/*
 * pcibios_update_irq(struct pci_dev *dev, int irq)
 *
 * Update a PCI interrupt.
 */
void __init
pcibios_update_irq(struct pci_dev *dev, int irq)
{
	pci_write_config_byte(dev, PCI_INTERRUPT_LINE, irq);
}

/*
 * pcibios_enable_device(struct pci_dev *dev, int mask)
 *
 * Enable a device on the PCI bus.
 */
int
pcibios_enable_device(struct pci_dev *dev, int mask)
{
	u16 cmd, old_cmd;
	int idx;
	struct resource *r;

	pci_read_config_word(dev, PCI_COMMAND, &cmd);
	old_cmd = cmd;
	for (idx = 0; idx < 6; idx++) {
		r = &dev->resource[idx];
		if (!r->start && r->end) {
			printk(KERN_ERR "PCI: Device %s not available because "
			       "of resource collisions\n", pci_name(dev));
			return -EINVAL;
		}
		if (r->flags & IORESOURCE_IO)
			cmd |= PCI_COMMAND_IO;
		if (r->flags & IORESOURCE_MEM)
			cmd |= PCI_COMMAND_MEMORY;
	}
	if (cmd != old_cmd) {
		printk("PCI: Enabling device %s (%04x -> %04x)\n",
		       pci_name(dev), old_cmd, cmd);
		pci_write_config_word(dev, PCI_COMMAND, cmd);
#ifdef CONFIG_M54455
		mcf5445x_conf_device(dev);
#endif
	}

	return 0;
}

/*
 * pcibios_fixup_bus(struct pci_bus *bus)
 */
void __init
pcibios_fixup_bus(struct pci_bus *bus)
{
	struct pci_dev *dev = bus->self;

	if (!dev) {
		/* Root bus. */
#ifdef CONFIG_M54455
		bus->resource[0] = &pci_ioport_resource;
		bus->resource[1] = &pci_iomem_resource;
#endif
	}
}

/*
 * pcibios_init(void)
 *
 * Allocate/initialize low level pci bus/devices.
 */
static int __init
pcibios_init(void)
{
	struct pci_bus *bus;

	if (!raw_pci_ops) {
		printk(KERN_WARNING "PCIBIOS: FATAL: NO PCI Hardware found\n");
		return 0;
	}

	/* allocate and scan the (only) bus */
	bus = pci_scan_bus_parented(NULL, 0, &pci_root_ops, NULL);

	/* setup everything */
	if (bus) {
		/* compute the bridge window sizes */
		pci_bus_size_bridges(bus);

		/* (re)assign device resources */
		pci_bus_assign_resources(bus);

		/* add the bus to the system */
		pci_bus_add_devices(bus);

		/* fixup irqs */
		pci_fixup_irqs(pcibios_swizzle, pcibios_map_irq);
	}

	return 0;
}

/*
 * pci_init(void)
 *
 * Initialize the PCI Hardware.
 */
static int __init
pci_init(void)
{
#if defined(CONFIG_M54455)
	init_mcf5445x_pci();
#endif
	if (!raw_pci_ops)
		printk(KERN_ERR "PCI: FATAL: NO PCI Detected\n");

	return 0;
}

/* low level hardware (first) */
arch_initcall(pci_init);

/* basic bios init (second) */
subsys_initcall(pcibios_init);

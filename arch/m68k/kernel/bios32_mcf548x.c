/*
 * bios32.c - PCI BIOS functions for m68k systems.
 *
 * Written by Wout Klaren.
 *
 * Based on the DEC Alpha bios32.c by Dave Rusling and David Mosberger.
 */
#include <linux/init.h>
#include <linux/kernel.h>

#if 0
# define DBG_DEVS(args)		printk args
#else
# define DBG_DEVS(args)
#endif

#ifdef CONFIG_PCI

/*
 * PCI support for Linux/m68k. Currently only the Hades is supported.
 *
 * The support for PCI bridges in the DEC Alpha version has
 * been removed in this version.
 */

#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/mm.h>

#include <asm/io.h>
#include <asm/pci.h>
#include <asm/uaccess.h>

#define KB		1024
#define MB		(1024*KB)
#define GB		(1024*MB)

#define MAJOR_REV	0
#define MINOR_REV	5

/*
 * Align VAL to ALIGN, which must be a power of two.
 */

#define MAX(val1, val2)		(((val1) > (val2)) ? val1 : val2)

/*
 * Offsets relative to the I/O and memory base addresses from where resources
 * are allocated.
 */

#ifdef CONFIG_COLDFIRE
#define IO_ALLOC_OFFSET		0x00000100
#define MEM_ALLOC_OFFSET	0x00000000
#else /* CONFIG_COLDFIRE */
#define IO_ALLOC_OFFSET		0x00004000
#define MEM_ALLOC_OFFSET	0x04000000
#endif /* CONFIG_COLDFIRE */

/*
 * Declarations of hardware specific initialisation functions.
 */

extern struct pci_bus_info *init_hades_pci(void);

/*
 * Bus info structure of the PCI bus. A pointer to this structure is
 * put in the sysdata member of the pci_bus structure.
 */

static struct pci_bus_info *bus_info;

static int pci_modify = 1;		/* If set, layout the PCI bus ourself. */
static int skip_vga;			/* If set do not modify base addresses
					   of vga cards.*/
static int disable_pci_burst;		/* If set do not allow PCI bursts. */

static unsigned int io_base;
static unsigned int mem_base;

/*
 * static void disable_dev(struct pci_dev *dev)
 *
 * Disable PCI device DEV so that it does not respond to I/O or memory
 * accesses.
 *
 * Parameters:
 *
 * dev	- device to disable.
 */

static void __init disable_dev(struct pci_dev *dev)
{
	unsigned short cmd;

	if (((dev->class >> 8 == PCI_CLASS_NOT_DEFINED_VGA) ||
	     (dev->class >> 8 == PCI_CLASS_DISPLAY_VGA) ||
	     (dev->class >> 8 == PCI_CLASS_DISPLAY_XGA)) && skip_vga)
		return;

	pci_read_config_word(dev, PCI_COMMAND, &cmd);

	cmd &= (~PCI_COMMAND_IO & ~PCI_COMMAND_MEMORY & ~PCI_COMMAND_MASTER);
	pci_write_config_word(dev, PCI_COMMAND, cmd);
}

/* Stolen from pcibios_enable_resources/i386 */
int pcibios_enable_device(struct pci_dev *dev, int mask)
{
	u16 cmd, old_cmd;
	int idx;
	struct resource *r;

	pci_read_config_word(dev, PCI_COMMAND, &cmd);
	old_cmd = cmd;
	for(idx=0; idx<6; idx++) {
		/* Only set up the requested stuff */
		if (!(mask & (1<<idx)))
			continue;

		r = &dev->resource[idx];
		if (!r->start && r->end) {
			printk("PCI: Device %s not available because"
				" of resource collisions\n", dev->dev.bus_id);
			return -EINVAL;
		}
		if (r->flags & IORESOURCE_IO)
			cmd |= PCI_COMMAND_IO;
		if (r->flags & IORESOURCE_MEM)
			cmd |= PCI_COMMAND_MEMORY;
	}
	if (dev->resource[PCI_ROM_RESOURCE].start)
		cmd |= PCI_COMMAND_MEMORY;
	if (cmd != old_cmd) {
		printk("PCI: Enabling device %s (%04x -> %04x)\n", dev->dev.bus_id, old_cmd, cmd);
		pci_write_config_word(dev, PCI_COMMAND, cmd);
	}
	return 0;
}

/*
 * static void layout_dev(struct pci_dev *dev)
 *
 * Layout memory and I/O for a device.
 *
 * Parameters:
 *
 * device	- device to layout memory and I/O for.
 */

static void __init layout_dev(struct pci_dev *dev)
{
	unsigned short cmd;
	unsigned int base, mask, size, reg;
	unsigned int alignto;
	int i;

	/*
	 * Skip video cards if requested.
	 */
	if (((dev->class >> 8 == PCI_CLASS_NOT_DEFINED_VGA) ||
	     (dev->class >> 8 == PCI_CLASS_DISPLAY_VGA) ||
	     (dev->class >> 8 == PCI_CLASS_DISPLAY_XGA)) && skip_vga){
		printk("%s: VGA\n",__FUNCTION__);
		return;
	}
	pci_read_config_word(dev, PCI_COMMAND, &cmd);

	for (reg = PCI_BASE_ADDRESS_0, i = 0; reg <= PCI_BASE_ADDRESS_5; reg += 4, i++)
	{
		/*
		 * Figure out how much space and of what type this
		 * device wants.
		 */

		pci_write_config_dword(dev, reg, 0xffffffff);
		pci_read_config_dword(dev, reg, &base);
		if (!base)
		{
			/* this base-address register is unused */
			dev->resource[i].start = 0;
			dev->resource[i].end = 0;
			dev->resource[i].flags = 0;
			continue;
		}

		/*
		 * We've read the base address register back after
		 * writing all ones and so now we must decode it.
		 */
	
		if (base & PCI_BASE_ADDRESS_SPACE_IO)
		{
			/*
			 * I/O space base address register.
			 */

			cmd |= PCI_COMMAND_IO;

			base &= PCI_BASE_ADDRESS_IO_MASK;
			mask = (~base << 1) | 0x1;
			size = (mask & base) & 0xffffffff;

			/*
			 * Align to multiple of size of minimum base.
			 */

#ifdef CONFIG_COLDFIRE
			alignto = MAX(PAGE_SIZE, size) ;
#else /* CONFIG_COLDFIRE */
			alignto = MAX(0x040, size) ;
#endif /* CONFIG_COLDFIRE */
			base = ALIGN(io_base, alignto);
			io_base = base + size;
			pci_write_config_dword(dev, reg, base | PCI_BASE_ADDRESS_SPACE_IO);

			dev->resource[i].start = base;
			dev->resource[i].end = dev->resource[i].start + size - 1;
			dev->resource[i].flags = IORESOURCE_IO | PCI_BASE_ADDRESS_SPACE_IO;

			DBG_DEVS(("layout_dev: IO address: %x\n", base));
		}
		else
		{
			unsigned int type;

			/*
			 * Memory space base address register.
			 */

			cmd |= PCI_COMMAND_MEMORY;
			type = base & PCI_BASE_ADDRESS_MEM_TYPE_MASK;
			base &= PCI_BASE_ADDRESS_MEM_MASK;
			mask = (~base << 1) | 0x1;
			size = (mask & base) & 0xffffffff;
			switch (type)
			{
			case PCI_BASE_ADDRESS_MEM_TYPE_32:
			case PCI_BASE_ADDRESS_MEM_TYPE_64:
				break;

			case PCI_BASE_ADDRESS_MEM_TYPE_1M:
				printk("bios32 WARNING: slot %d, function %d "
				       "requests memory below 1MB---don't "
				       "know how to do that.\n",
				       PCI_SLOT(dev->devfn),
				       PCI_FUNC(dev->devfn));
				continue;
			}
			DBG_DEVS(("%s MEM: base %x,type %x,mask %x,size %x\n",
				__FUNCTION__, base, type, mask, size));
			/*
			 * Align to multiple of size of minimum base.
			 */

			alignto = max_t(unsigned int, 0x1000, size);
			base = ALIGN(mem_base, alignto);
			mem_base = base + size;
			pci_write_config_dword(dev, reg, base);

			dev->resource[i].start = base;
			dev->resource[i].end = dev->resource[i].start + size - 1;
			dev->resource[i].flags = IORESOURCE_MEM;
			DBG_DEVS(("%s MEM :base %x,size %x\n",
				__FUNCTION__, base, size));
			if (type == PCI_BASE_ADDRESS_MEM_TYPE_64)
			{
				/*
				 * 64-bit address, set the highest 32 bits
				 * to zero.
				 */

				reg += 4;
				pci_write_config_dword(dev, reg, 0);

				i++;
				dev->resource[i].start = 0;
				dev->resource[i].end = 0;
				dev->resource[i].flags = 0;
				printk("%s:type == 64\n",__FUNCTION__);
			}
		}
	}

	/*
	 * Enable device:
	 */

	if (dev->class >> 8 == PCI_CLASS_NOT_DEFINED ||
	    dev->class >> 8 == PCI_CLASS_NOT_DEFINED_VGA ||
	    dev->class >> 8 == PCI_CLASS_DISPLAY_VGA ||
	    dev->class >> 8 == PCI_CLASS_DISPLAY_XGA)
	{
		/*
		 * All of these (may) have I/O scattered all around
		 * and may not use i/o-base address registers at all.
		 * So we just have to always enable I/O to these
		 * devices.
		 */
		cmd |= PCI_COMMAND_IO;
	}

	pci_write_config_word(dev, PCI_COMMAND, cmd | PCI_COMMAND_MASTER);

	pci_write_config_byte(dev, PCI_LATENCY_TIMER, (disable_pci_burst) ? 0 : 32);

	if (bus_info != NULL)
		bus_info->conf_device(dev);	/* Machine dependent configuration. */

	printk(KERN_INFO "layout_dev: bus %d  slot 0x%x  VID 0x%x  DID 0x%x  class 0x%x\n",
		dev->bus->number, PCI_SLOT(dev->devfn),
		dev->vendor, dev->device, dev->class);
}

/*
 * static void layout_bus(struct pci_bus *bus)
 *
 * Layout memory and I/O for all devices on the given bus.
 *
 * Parameters:
 *
 * bus	- bus.
 */

static void __init layout_bus(struct pci_bus *bus)
{
	unsigned int bio, bmem;
	struct pci_dev *dev;

	DBG_DEVS(("layout_bus: starting bus %d\n", bus->number));

	if (list_empty(&bus->devices) && list_empty(&bus->children))
		return;

	/*
	 * Align the current bases on appropriate boundaries (4K for
	 * IO and 1MB for memory).
	 */

	bio = io_base = ALIGN(io_base, 4*KB);
	bmem = mem_base = ALIGN(mem_base, 1*MB);

	/*
	 * PCI devices might have been setup by a PCI BIOS emulation
	 * running under TOS. In these cases there is a
	 * window during which two devices may have an overlapping
	 * address range. To avoid this causing trouble, we first
	 * turn off the I/O and memory address decoders for all PCI
	 * devices.  They'll be re-enabled only once all address
	 * decoders are programmed consistently.
	 */

	DBG_DEVS(("layout_bus: disable_dev for bus %d\n", bus->number));

#ifdef NL_ORIGINAL
	for (dev = bus->devices; dev; dev = dev->sibling)
#else
	dev = NULL;
	while ((dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, dev)) != NULL)
#endif
	{
		if ((dev->class >> 16 != PCI_BASE_CLASS_BRIDGE) ||
		    (dev->class >> 8 == PCI_CLASS_BRIDGE_PCMCIA))
			disable_dev(dev);
	}

	/*
	 * Allocate space to each device:
	 */

	DBG_DEVS(("layout_bus: starting bus %d devices\n", bus->number));

#ifdef NL_ORIGINAL
	for (dev = bus->devices; dev; dev = dev->sibling)
#else
	dev = NULL;
	while ((dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, dev)) != NULL)
#endif
	{
		if ((dev->class >> 16 != PCI_BASE_CLASS_BRIDGE) ||
		    (dev->class >> 8 == PCI_CLASS_BRIDGE_PCMCIA))
			layout_dev(dev);
	}

	DBG_DEVS(("layout_bus: bus %d finished\n", bus->number));
}

/*
 * static void pcibios_fixup(void)
 *
 * Layout memory and I/O of all devices on the PCI bus if 'pci_modify' is
 * true. This might be necessary because not every m68k machine with a PCI
 * bus has a PCI BIOS. This function should be called right after
 * pci_scan_bus() in pcibios_init().
 */

static void __init pcibios_fixup(void)
{
	DBG_DEVS(("%s\n", __FUNCTION__));
	if (pci_modify)
	{
		/*
		 * Set base addresses for allocation of I/O and memory space.
		 */

		io_base = bus_info->io_space.start + IO_ALLOC_OFFSET;
		mem_base = bus_info->mem_space.start + MEM_ALLOC_OFFSET;

		/*
		 * Scan the tree, allocating PCI memory and I/O space.
		 */

#ifdef NL_ORIGINAL
		layout_bus(pci_bus_b(pci_root.next));
#else
		layout_bus(pci_bus_b(pci_root_buses.next));
#endif
	}

	/*
	 * Fix interrupt assignments, etc.
	 */

	bus_info->fixup(pci_modify);
}

/*
 * static void pcibios_claim_resources(struct pci_bus *bus)
 *
 * Claim all resources that are assigned to devices on the given bus.
 *
 * Parameters:
 *
 * bus	- bus.
 */

static void __init pcibios_claim_resources(struct pci_bus *bus)
{
	struct pci_dev *dev;
	int i;
	DBG_DEVS(("%s\n", __FUNCTION__));
#ifdef NL_ORIGINAL
	while (bus)
#else
	while ((bus = pci_find_next_bus(bus)) != NULL)
#endif
	{

#ifdef NL_ORIGINAL
		for (dev = bus->devices; (dev != NULL); dev = dev->sibling)
#else
		dev = NULL;
		while ((dev = pci_find_device(PCI_ANY_ID, PCI_ANY_ID, dev)) != NULL)
#endif
		{
			for (i = 0; i < PCI_NUM_RESOURCES; i++)
			{
				struct resource *r = &dev->resource[i];
				struct resource *pr;
				struct pci_bus_info *bus_info = 
					(struct pci_bus_info *) dev->sysdata;

				if ((r->start == 0) || (r->parent != NULL))
					continue;

#ifdef CONFIG_COLDFIRE
				if (dev->class >> 16 == PCI_BASE_CLASS_BRIDGE)
					continue;
#endif /* CONFIG_COLDFIRE */
#if 1
				if (r->flags & IORESOURCE_IO)
					pr = &bus_info->io_space;
				else
					pr = &bus_info->mem_space;
#else
				if (r->flags & IORESOURCE_IO)
					pr = &ioport_resource;
				else
					pr = &iomem_resource;
#endif
				if (request_resource(pr, r) < 0)
				{
#ifdef NL_ORIGINAL
					DBG_DEVS(("PCI: Address space collision on "
						"region %d of device %s\n", i, dev->name));
#else
					printk("PCI: Address space collision on region %d of device %s\n", i, dev->dev.bus_id);
#endif
				}
			}
		}

#ifdef NL_ORIGINAL
		if (bus->children)
			pcibios_claim_resources(bus->children);
#else
		if (!list_empty(&bus->children))
			pcibios_claim_resources(pci_bus_b(bus->children.next));
#endif

#ifdef NL_ORIGINAL
		bus = bus->next;
#endif
	}
}

/*
 * int pcibios_assign_resource(struct pci_dev *dev, int i)
 *
 * Assign a new address to a PCI resource.
 *
 * Parameters:
 *
 * dev	- device.
 * i	- resource.
 *
 * Result: 0 if successful.
 */

int __init pcibios_assign_resource(struct pci_dev *dev, int i)
{
	struct resource *r = &dev->resource[i];
	struct resource *pr = pci_find_parent_resource(dev, r);
	unsigned long size = r->end + 1;
	DBG_DEVS(("%s:IO_ALLOC_OFFSET %x\n", __FUNCTION__, IO_ALLOC_OFFSET));
	if (!pr)
		return -EINVAL;

	if (r->flags & IORESOURCE_IO)
	{
		DBG_DEVS(("%s:IORESOURCE_IO:start %x, size %lx\n",
			__FUNCTION__, bus_info->io_space.start, size));
		if (size > 0x100)
			return -EFBIG;

#ifdef NL_ORIGINAL
		if (allocate_resource(pr, r, size, bus_info->io_space.start +
				      IO_ALLOC_OFFSET,  bus_info->io_space.end, 1024))
#else
		if (allocate_resource(pr, r, size, bus_info->io_space.start +
				   IO_ALLOC_OFFSET,  bus_info->io_space.end, 1024, NULL, NULL))
#endif
			return -EBUSY;
	}
	else
	{
		DBG_DEVS(("%s:IORESOURCE_MEM:start %x, size %lx\n",
				__FUNCTION__, bus_info->mem_space.start, size));
#ifdef NL_ORIGINAL
		if (allocate_resource(pr, r, size, bus_info->mem_space.start +
				      MEM_ALLOC_OFFSET, bus_info->mem_space.end, size))
#else
		if (allocate_resource(pr, r, size, bus_info->io_space.start +
				   IO_ALLOC_OFFSET,  bus_info->io_space.end, 1024, NULL, NULL))
#endif
			return -EBUSY;
	}

	if (i < 6)
		pci_write_config_dword(dev, PCI_BASE_ADDRESS_0 + 4 * i, r->start);

	return 0;
}

void pcibios_fixup_bus(struct pci_bus *bus)
{
	struct pci_dev *dev;
	void *sysdata;

	sysdata = (bus->parent) ? bus->parent->sysdata : bus->sysdata;

#ifdef NL_ORIGINAL
	for (dev = bus->devices; (dev != NULL); dev = dev->sibling)
#else
	dev = NULL;
	while ((dev = pci_find_device(PCI_ANY_ID, PCI_ANY_ID, dev)) != NULL)
#endif
		dev->sysdata = sysdata;
}

int __init pcibios_init(void)
{
	printk("Linux/m68k PCI BIOS32 revision %x.%02x\n", MAJOR_REV, MINOR_REV);

	bus_info = NULL;
#ifdef CONFIG_COLDFIRE
	bus_info = init_coldfire_pci();
#endif /* CONFIG_COLDFIRE */
#ifdef CONFIG_HADES
	if (MACH_IS_HADES)
		bus_info = init_hades_pci();
#endif
	if (bus_info != NULL)
	{
		printk("PCI: Probing PCI hardware\n");
		pci_scan_bus(0, bus_info->m68k_pci_ops, bus_info);
		pcibios_fixup();
#ifdef NL_ORIGINAL
		pcibios_claim_resources(pci_root);
#else
		pcibios_claim_resources(pci_bus_b(pci_root_buses.next));
#endif
	}
	else
		printk("PCI: No PCI bus detected\n");
	return 0;
}

subsys_initcall(pcibios_init);

char * pcibios_setup(char *str)
{
	if (!strcmp(str, "nomodify"))
	{
		pci_modify = 0;
		return NULL;
	}
	else if (!strcmp(str, "skipvga"))
	{
		skip_vga = 1;
		return NULL;
	}
	else if (!strcmp(str, "noburst"))
	{
		disable_pci_burst = 1;
		return NULL;
	}

	return str;
}
#endif /* CONFIG_PCI */

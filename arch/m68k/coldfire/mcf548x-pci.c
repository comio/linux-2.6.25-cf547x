/*
 * 	ColdFire 547x/548x PCI Host Controller functions
 */
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/interrupt.h>

#include <linux/dma-mapping.h>
#include <asm/coldfire.h>
#include <asm/io.h>
#include <asm/m5485sim.h>
#include <asm/m5485pci.h>
#include <asm/irq.h>
#include <asm/pci.h>
#include <asm/virtconvert.h>


#undef DEBUG
//#define DEBUG

#ifdef DEBUG
//#define DBG(x...) printk(KERN_DEBUG x)
#define DBG(x...) printk(x)
#else
#define DBG(x...)
#endif

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,5,75))
# define irqreturn_t void
# define IRQ_HANDLED
# define IRQ_NONE
#endif

/*
 *  Bridge configration dafaults
 */
#define PCI_RETRIES	0
#define PCI_CACHE_LINE 	8
#define PCI_MINGNT	1
#define PCI_MAXLAT	42


/*
 *  Initiator windows setting
 */
#define HOST_MEM_BASE    	0xD0000000	/* ColdFire Memory window base 	*/
#define PCI_MEM_BASE    	0xD0000000	/* PCI Memory window base	*/
#define PCI_MEM_SIZE    	0x08000000	/* Memory window size (128M)	*/
#define HOST_IO_BASE     	0xD8000000	/* ColdFire I/O window base 	*/
#define PCI_IO_BASE_ADDR     	0x00000000	/* PCI I/O window base 		*/
#define PCI_IO_SIZE     	0x00010000	/* I/O window size (64K) 	*/
#define HOST_CFG_BASE    	0xD8000000	/* ColdFire config window base 	*/
#define HOST_DMA_BASE 	 CONFIG_SDRAM_BASE	/* ColdFire PCI-DMA window base */
#define PCI_HDR_BASE    	(MCF_MBAR+0xB00)/* ColdFire config registers    */

#define PCI_MEM_MASK		(PCI_MEM_SIZE-1)
#define PCI_IO_MASK		(PCI_IO_SIZE-1)

/* Macro to set initiator window */
#define WxBAR(host_address, pci_address, size) 	\
	(((host_address)  & 0xff000000)      |	\
    	((((size)-1) & 0xff000000) >> 8)     | 	\
	((pci_address) & 0xff000000) >> 16)

/*
 *  BIOS internal data
 */
static u8 revision;		/* controller revision */

/*
 * 	Board specific setting
 */
const unsigned int irq_lines[] = { 5, 7 };

#define N_SLOTS      	(sizeof(board_info) / sizeof(board_info[0]))
#define N_IRQS		(sizeof(irq_lines)  / sizeof(irq_lines[0]))
#define BRIDGE_SLOT  	0

const struct slotinfo {
	unsigned char idsel;	/* device number     */
	unsigned char irq;	/* external IRQ      */
	unsigned char req;	/* REQ line number   */
	unsigned char gnt;	/* GNT line number   */
} board_info[] = {
	{0,  0, 0, 0},		/* Bridge      */
	{17, 5, 1, 1},		/* Slot #1     */
	{18, 5, 2, 2},		/* Slot #2     */	
	{20, 7, 3, 3},		/* Slot #3     */
	{21, 7, 4, 4},		/* Slot #4     */
};

/************************************************************************/

/*
 * static int mk_conf_addr()
 *
 * Return type0 or type1 configuration address
 * by the means of device address and PCI dword location
 * 0 - for not existing slots
 */
static int mk_conf_addr(/*struct pci_dev *dev*/struct pci_bus *bus, unsigned int devfn, int where)
{
	int slot, func, address, idsel, dev_fn;

	if (bus->number) {
		address = MCF_PCICAR_E | (bus->number << 16) |
		    (devfn << 8) | (where & 0xfc);
	} else {
		slot = PCI_SLOT(devfn);
		if (slot > N_SLOTS || slot == BRIDGE_SLOT)
			return 0;
		else {
			func = PCI_FUNC(devfn);
			idsel = board_info[slot].idsel;

			dev_fn = PCI_DEVFN(idsel, func);
			address = MCF_PCICAR_E | (bus->number << 16) |
			    (dev_fn << 8) | (where & 0xfc);
		}
	}

	return (address);
}

/*
 * static int read_config_byte()
 *
 * Read a byte from configuration space of specified device
 */
static int read_config_byte(/*struct pci_dev *dev*/struct pci_bus *bus, unsigned int devfn, int where, u8 *value)
{
	int slot;
	int address;
	int result;

	*value = 0xff;
	result = PCIBIOS_SUCCESSFUL;

	slot = PCI_SLOT(devfn);
	if (slot == BRIDGE_SLOT) {
		if (where <= 0x40)
			*value = *(volatile u8 *) (PCI_HDR_BASE + (where ^ 3));
		else
			*value = 0;
	} else {
		address = mk_conf_addr(bus, devfn, where);
		if (!address)
			result = PCIBIOS_DEVICE_NOT_FOUND;
		else {
			MCF_PCICAR = address;
			*value = *(volatile u8 *) (HOST_CFG_BASE + (where & 3));
		}
	}
	__asm__ __volatile__("nop");
	__asm__ __volatile__("nop");
	MCF_PCICAR &= ~MCF_PCICAR_E;

	DBG("PCI: read_config_byte bus=%d, devfn=%d, addr=0x%02X, val=0x%02X, ret=%02X\n",
	    bus->number, devfn, where, *value, result);

	return (result);
}

/*
 * static int read_config_word()
 *
 * Read a word from configuration space of specified device
 */
static int read_config_word(/*struct pci_dev *dev*/struct pci_bus *bus, unsigned int devfn, int where, u16 *value)
{
	int slot;
	int address;
	int result;

	*value = 0xffff;
	result = PCIBIOS_SUCCESSFUL;

	if (where & 0x1)
		result = PCIBIOS_BAD_REGISTER_NUMBER;
	else {
		slot = PCI_SLOT(devfn);
		if (slot == BRIDGE_SLOT) {
			if (where <= 0x3f)
				*value =
				    *(volatile u16 *) (PCI_HDR_BASE +
						       (where ^ 2));
			else
				*value = 0;
		} else {
			address = mk_conf_addr(bus, devfn, where);
			if (!address)
				result = PCIBIOS_DEVICE_NOT_FOUND;
			else {
				MCF_PCICAR = address;
				*value = le16_to_cpu(*(volatile u16 *)
						     (HOST_CFG_BASE +
						      (where & 2)));
			}
		}
	}
        __asm__ __volatile__("nop");
        __asm__ __volatile__("nop");
	MCF_PCICAR &= ~MCF_PCICAR_E;

	DBG("PCI: read_config_word bus=%d, devfn=%d, addr=0x%02X, val=0x%04X ret=%02X\n",
	    bus->number, devfn, where, *value, result);

	return (result);
}

/*
 * static int read_config_dword()
 *
 * Read a long word from configuration space of specified device
 */
static int read_config_dword(/*struct pci_dev *dev*/struct pci_bus *bus, unsigned int devfn, int where, u32 *value)
{
	int slot;
	int address;
	int result;

	*value = 0xffffffff;
	result = PCIBIOS_SUCCESSFUL;

	if (where & 0x3)
		result = PCIBIOS_BAD_REGISTER_NUMBER;
	else {
		slot = PCI_SLOT(devfn);
		if (slot == BRIDGE_SLOT) {
			if (where <= 0x3d)
				*value =
				    *(volatile u32 *) (PCI_HDR_BASE + where);
			else
				*value = 0;
			__asm__ __volatile__("nop");
        		__asm__ __volatile__("nop");
		} else {
			address = mk_conf_addr(bus, devfn, where);
			if (!address)
				result = PCIBIOS_DEVICE_NOT_FOUND;
			else {
				MCF_PCICAR = address;
				*value = le32_to_cpu(*(volatile u32 *)
						     (HOST_CFG_BASE));
				__asm__ __volatile__("nop");
				__asm__ __volatile__("nop");
				if (bus->number != 0 && revision < 1) {
					volatile u32 temp;

					MCF_PCICAR |= 0xff0000;
					temp = *(volatile u32 *) (HOST_CFG_BASE);
				}
			}
		}
	}

	MCF_PCICAR &= ~MCF_PCICAR_E;

	DBG("PCI: read_config_dword bus=%d, devfn=%d, addr=0x%02X, value=0x%08X ret=%02X\n",
	     bus->number, devfn, where, *value, result);

	return (result);
}

/*
 * static int write_config_byte()
 *
 * Write a byte to configuration space of specified device
 */
static int write_config_byte(/*struct pci_dev *dev*/struct pci_bus *bus, unsigned int devfn, int where, u8 value)
{
	int slot;
	int address;
	int result;

	result = PCIBIOS_SUCCESSFUL;

	slot = PCI_SLOT(devfn);
	if (slot == BRIDGE_SLOT) {
		if (where <= 0x40)
			*(volatile u8 *) (PCI_HDR_BASE + (where ^ 3)) = value;
	} else {
		address = mk_conf_addr(bus, devfn, where);
		if (!address)
			result = PCIBIOS_DEVICE_NOT_FOUND;
		else {
			MCF_PCICAR = address;
			*(volatile u8 *) (HOST_CFG_BASE + (where & 3)) = value;
		}
	}
        __asm__ __volatile__("nop");
        __asm__ __volatile__("nop");
	MCF_PCICAR &= ~MCF_PCICAR_E;

	DBG("PCI: write_config_byte bus=%d, devfn=%d, addr=0x%02X, value=0x%02X ret=%02X\n",
	     bus->number, devfn, where, value, result);

	return (result);
}

/*
 * static int write_config_word()
 *
 * Write a word to configuration space of specified device
 */
static int write_config_word(/*struct pci_dev *dev*/struct pci_bus *bus, unsigned int devfn, int where, u16 value)
{
	int slot;
	int address;
	int result;

	result = PCIBIOS_SUCCESSFUL;

	if (where & 0x1)
		result = PCIBIOS_BAD_REGISTER_NUMBER;
	else {
		slot = PCI_SLOT(devfn);
		if (slot == BRIDGE_SLOT) {
			if (where <= 0x3f)
				*(volatile u16 *) (PCI_HDR_BASE + (where ^ 2)) =
				    value;
		} else {
			address = mk_conf_addr(bus, devfn, where);
			if (!address)
				result = PCIBIOS_DEVICE_NOT_FOUND;
			else {
				MCF_PCICAR = address;
				*(volatile u16 *) (HOST_CFG_BASE + (where & 2)) =
				    cpu_to_le16(value);
			}
		}
	}
	__asm__ __volatile__("nop");
        __asm__ __volatile__("nop");
	MCF_PCICAR &= ~MCF_PCICAR_E;

	DBG("PCI: write_config_word bus=%d, devfn=%d, addr=0x%02X, value=0x%04X ret=%02X\n",
	     bus->number, devfn, where, value, result);

	return (result);
}

/*
 * static int write_config_dword()
 *
 * Write a long word to configuration space of specified device
 */
static int write_config_dword(/*struct pci_dev *dev*/struct pci_bus *bus, unsigned int devfn, int where, u32 value)
{
	int slot;
	int address;
	int result;

	result = PCIBIOS_SUCCESSFUL;

	if (where & 0x3)
		result = PCIBIOS_BAD_REGISTER_NUMBER;
	else {
		slot = PCI_SLOT(devfn);
		if (slot == BRIDGE_SLOT) {
			if (where <= 0x3d)
				*(volatile u32 *) (PCI_HDR_BASE + where) =
				    value;
		} else {
			address = mk_conf_addr(bus, devfn, where);
			if (!address)
				result = PCIBIOS_DEVICE_NOT_FOUND;
			else {
				MCF_PCICAR = address;
				*(volatile u32 *) (HOST_CFG_BASE) =
				    cpu_to_le32(value);
			}
		}
	}
	__asm__ __volatile__("nop");
        __asm__ __volatile__("nop");
	MCF_PCICAR &= ~MCF_PCICAR_E;

	DBG("PCI: write_config_dword dev=%d, fn=%d, addr=0x%02X, value=0x%08X ret=%02X\n",
	    PCI_SLOT(devfn), PCI_FUNC(devfn), where,  value, result);

	return (result);
}

static int config_read(struct pci_bus *bus, unsigned int devfn,
		       int where, int size, u32 * val)
{
	switch (size) {
	case 1:
		return read_config_byte(bus, devfn, where, (u8 *) val);
	case 2:
		return read_config_word(bus, devfn, where, (u16 *) val);
	default:
		return read_config_dword(bus, devfn, where, val);
	}
}

static int config_write(struct pci_bus *bus, unsigned int devfn,
			int where, int size, u32 val)
{
	switch (size) {
	case 1:
		return write_config_byte(bus, devfn, where, (u8) val);
	case 2:
		return write_config_word(bus, devfn, where, (u16) val);
	default:
		return write_config_dword(bus, devfn, where, val);
	}
}

/*
 *  configuration routines entry points
 */
static struct pci_ops bus_ops = {
      read:		config_read,
      write:		config_write
};

/************************************************************************/

/*
 * u8 pci_inb()
 *
 * Read a byte at specified address from I/O space
 */
unsigned char pci_inb(long addr)
{
	char value;

	value = *(volatile unsigned char *) (HOST_IO_BASE | (addr & PCI_IO_MASK));
	DBG("PCI: inb addr=0x%08X, value=0x%02X\n", addr, value);

	return (unsigned char) value;
}


/*
 * u16 pci_inw()
 *
 * Read a word at specified address from I/O space
 */
unsigned short pci_inw(long addr)
{
	short value;
	volatile unsigned short *ptr;

	ptr = (volatile unsigned short *) (HOST_IO_BASE | (addr & PCI_IO_MASK));
	value = le16_to_cpu(*ptr);

	DBG("PCI: inw addr=0x%08X, value=0x%04X\n",  addr, value);
	return (unsigned short) value;
}

/*
 * u16 pci_raw_inw()
 *
 * Read a raw word at specified address from I/O space
 */
unsigned short pci_raw_inw(long addr)
{
	short value;
	volatile unsigned short *ptr;

	ptr = (volatile unsigned short *) (HOST_IO_BASE | (addr & PCI_IO_MASK));
	value = *ptr;

	DBG("PCI: raw_inw addr=0x%08X, value=0x%04X\n",  addr, value);
	return (unsigned short) value;
}

/*
 * u32 pci_inl()
 *
 * Read a dword at specified address from I/O space
 */
unsigned long pci_inl(long addr)
{
	long value;
	volatile unsigned long *ptr;

	ptr = (volatile unsigned long *) (HOST_IO_BASE | (addr & PCI_IO_MASK));
	value = le32_to_cpu(*ptr);

	DBG("PCI: inl addr=0x%08X, value=0x%08X\n",  addr, value);
	return (unsigned long) value;
}

/*
 * u32 pci_raw_inl()
 *
 * Read a raw dword at specified address from I/O space
 */
unsigned long pci_raw_inl(long addr)
{
	long value;
	volatile unsigned long *ptr;

	ptr = (volatile unsigned long *) (HOST_IO_BASE | (addr & PCI_IO_MASK));
	value = *ptr;

	DBG("PCI: raw_inl addr=0x%08X, value=0x%08X\n",  addr, value);
	return (unsigned long) value;
}

/*
 * void pci_outb()
 *
 * Write a byte value at specified address to I/O space
 */
void pci_outb( unsigned char value,  long addr)
{

	*(volatile unsigned char *) (HOST_IO_BASE | (addr & PCI_IO_MASK)) = value;
	DBG("PCI: outb addr=0x%08X, value=0x%02X\n",  addr, value);
}


/*
 * void pci_outw()
 *
 * Write a word value at specified address to I/O space
 */
void pci_outw(volatile unsigned short value, volatile  long addr)
{
	volatile unsigned short *ptr;

	ptr = (volatile unsigned short *) (HOST_IO_BASE | (addr & PCI_IO_MASK));
	*ptr = cpu_to_le16(value);
	DBG("PCI: outw addr=0x%08X, value=0x%04X\n",  addr, value);
}

/*
 * void pci_raw_outw()
 *
 * Write a raw word value at specified address to I/O space
 */
void pci_raw_outw(volatile unsigned short value, volatile  long addr)
{
	volatile unsigned short *ptr;

	ptr = (volatile unsigned short *) (HOST_IO_BASE | (addr & PCI_IO_MASK));
	*ptr = value;
	DBG("PCI: raw_outw addr=0x%08X, value=0x%04X\n",  addr, value);
}

/*
 * void pci_outl()
 *
 * Write a long word value at specified address to I/O space
 */
void pci_outl(volatile unsigned long value, volatile long addr)
{
	volatile unsigned long *ptr;

	ptr = (volatile unsigned long *)(HOST_IO_BASE | (addr & PCI_IO_MASK));
	*ptr = cpu_to_le32(value);
	DBG("PCI: outl addr=0x%08X, value=0x%08X\n", addr, value);
}

/*
 * void pci_raw_outl()
 *
 * Write a raw long word value at specified address to I/O space
 */
void pci_raw_outl(volatile unsigned long value, volatile long addr)
{
	volatile unsigned long *ptr;

	ptr = (volatile unsigned long *)(HOST_IO_BASE | (addr & PCI_IO_MASK));
	*ptr = value;
	DBG("PCI: raw_outl addr=0x%08X, value=0x%08X\n", addr, value);
}

/*
 * void pci_insb()
 *
 * Read several byte values from specified I/O port
 */
void pci_insb(volatile unsigned char *addr, unsigned char *buf, int len)
{
	for (; len--; buf++)
		*buf = pci_inb((unsigned long)addr);
	DBG("PCI: pci_insb addr=0x%08X, buf=%p, len=%d\n", addr, buf, len);
}


/*
 * void pci_insw()
 *
 * Read several word values from specified I/O port
 */
void pci_insw(volatile unsigned short *addr, unsigned short *buf, int len)
{
	for (; len--; buf++)
		*buf = pci_inw((unsigned long)addr);
	DBG("PCI: pci_insw addr=0x%08X, buf=%p, len=%d\n", addr, buf, len);
}

/*
 * void pci_insl()
 *
 * Read several dword values from specified I/O port
 */
void pci_insl(volatile unsigned long *addr, unsigned long *buf, int len)
{
	for (; len--; buf++)
		*buf = pci_inl((unsigned long)addr);
	DBG("PCI: pci_insl addr=0x%08X, buf=%p, len=%d\n", addr, buf, len);
}

/*
 * void pci_outsb()
 *
 * Write several byte values to specified I/O port
 */
void pci_outsb(volatile unsigned char *addr, const unsigned char *buf, int len)
{
	for (; len--; buf++)
		pci_outb((unsigned long)addr, *buf);
	DBG("PCI: pci_outsb addr=0x%08X, buf=%p, len=%d\n", addr, buf, len);
}

/*
 * void pci_outsw()
 *
 * Write several word values to specified I/O port
 */
void pci_outsw(volatile unsigned short *addr, const unsigned short *buf, int len)
{
	for (; len--; buf++)
		pci_outw((unsigned long)addr, *buf);
	DBG("PCI: pci_outsw addr=0x%08X, buf=%p, len=%d\n", addr, buf, len);
}

/*
 * void pci_outsl()
 *
 * Write several dword values to specified I/O port
 */
void pci_outsl(volatile unsigned long *addr, const unsigned long *buf, int len)
{
	for (; len--; buf++)
		pci_outl((unsigned long)addr, *buf);
	DBG("PCI: pci_outsl addr=0x%08X, buf=%p, len=%d\n", addr, buf, len);
}

/*
 * void pci_xlb_handler()
 *
 * PCI XLB interrupt handler
 */
irqreturn_t xlb_interrupt(int irq, void *dev)
{
	volatile int xlb_error = MCF_PCIISR;

	/* Acknowlege interrupt */
	MCF_PCIISR = xlb_error;

	/* Dump interrupt reason */
	if (xlb_error & MCF_PCIISR_RE)
		DBG("PCI: Retry Error Received\n");
		
	if (xlb_error & MCF_PCIISR_IA)
		DBG("PCI: Initiator Abort Received\n");
		
	if (xlb_error & MCF_PCIISR_TA)
		DBG("PCI: Target Abort Received\n");

	return IRQ_HANDLED;
}


/*
 * void pci_arbiter_handler()
 *
 * PCI arbiter interrupt handler
 */
irqreturn_t arb_interrupt(int irq, void *dev)
{
	volatile unsigned long arb_error = MCF_PCIARB_PASR;

	/* Acknowlege interrupt */
	printk("%s\n",__FUNCTION__);
	MCF_PCIARB_PASR = arb_error;

	if (arb_error & MCF_PCIARB_PASR_ITLMBK) {
		DBG("PCI: coldfire master time-out\n");				
				
		/* Set infinite  number of retries */			
	        MCF_PCIICR &= ~0xFF;		    		
	}
		
	if (arb_error & MCF_PCIARB_PASR_EXTMBK(0x1F)) {
		arb_error >>= 17;
		DBG("PCI: external master time-out (mask = 0x%X)\n", arb_error);
		
	        /* raise arbitration priority level */		
		MCF_PCIARB_PACR = MCF_PCIARB_PACR_EXTMPRI(arb_error);
	}

	return IRQ_HANDLED;
}


/*
 * void pci_eint_handler()
 *
 * Eport interrupt handler
 */
irqreturn_t eint_handler(int irq, void *dev)
{
	/* Just acknowlege interrupt and exit */
	MCF_EPFR = 0x1 << (irq - 64);
	return IRQ_HANDLED;
}


/*
 * void __init coldfire_fixup(int pci_modify)
 *
 * Assign IRQ numbers as used by Linux to the interrupt pins
 * of the PCI cards.
 */
static void __init coldfire_fixup(int pci_modify)
{
	struct pci_dev *dev;
	unsigned char slot, pin;

	DBG("%s\n",__FUNCTION__);
#ifdef NL_ORIGINAL
	pci_for_each_dev(dev) {
#else
	dev = NULL;
	while ((dev = pci_find_device(PCI_ANY_ID, PCI_ANY_ID, dev)) != NULL) {
#endif
		if (dev->class >> 16 != PCI_BASE_CLASS_BRIDGE) {
			slot = PCI_SLOT(dev->devfn);
			dev->irq = 64 + board_info[slot].irq;

			/* Check if device needs interrupt */
#ifdef NL_ORIGINAL
			pcibios_read_config_byte(
				    dev->bus->number, dev->devfn,
    				    PCI_INTERRUPT_PIN, &pin);

			if ( pin ) {
				pcibios_write_config_byte(
					dev->bus->number, dev->devfn,
					PCI_INTERRUPT_LINE, dev->irq);
			}
#else
			pci_read_config_byte(dev,
    				    PCI_INTERRUPT_PIN, &pin);

			if ( pin ) {
				pci_write_config_byte(dev,
					PCI_INTERRUPT_LINE, dev->irq);
			}
#endif
		}
	}
}

static void __init configure_device(struct pci_dev *dev)
{
	/* TODO: This should depend from disable_pci_burst setting */
	DBG("%s\n",__FUNCTION__);
#ifdef NL_ORIGINAL
	pcibios_write_config_byte(bus, devfn, PCI_CACHE_LINE_SIZE, PCI_CACHE_LINE);
#else
	pci_write_config_byte(dev, PCI_CACHE_LINE_SIZE, PCI_CACHE_LINE);
#endif
}

struct pci_bus_info *__init init_coldfire_pci(void)
{
	static struct pci_bus_info bus;
	int i;
	int pci_mem_va;
	static char irq_name[N_IRQS][15];

	/* Get controller revision */
	revision = MCF_PCICCRIR;
	printk("ColdFire PCI Host Bridge (Rev. %d) detected:"
		"MEMBase %x,MEMLen %x,IOBase %x,IOLen %x\n",
		revision, HOST_MEM_BASE, PCI_MEM_SIZE - 1, 0, PCI_IO_SIZE - 1);

	/* Setup bus info structure. */
	memset(&bus, 0, sizeof (struct pci_bus_info));

	/* Request intiator memory resource */
	bus.mem_space.start = PCI_MEM_BASE;//HOST_MEM_BASE;
	bus.mem_space.end = bus.mem_space.start + PCI_MEM_SIZE - 1;
	bus.mem_space.name = "PCI Bus #0";
	if (request_resource(&iomem_resource, &bus.mem_space) != 0)
	{
		printk("Failed to request bridge iomem resource\n");
		return NULL;
	}

	/* Request intiator memory resource */
	bus.io_space.start = 0;
	bus.io_space.end = bus.io_space.start + PCI_IO_SIZE - 1;
	bus.io_space.name =  "PCI Bus #0";
	if (request_resource(&ioport_resource, &bus.io_space) != 0)
	{
		printk("Failed to request bridge ioport resource\n");
		return NULL;
	}

	/* Set up the arbiter */
	MCF_PCIARB_PACR = 0 /*MCF_PCIARB_PACR_PKMD*/
			| MCF_PCIARB_PACR_INTMPRI 
			| MCF_PCIARB_PACR_INTMINTEN 
			| MCF_PCIARB_PACR_EXTMPRI(0x1F) 
			| MCF_PCIARB_PACR_EXTMINTEN(0x1F);

	/* GNT and REQ */
	MCF_PAR_PCIBG = 0x3FF;
	MCF_PAR_PCIBR = 0x3FF;

	/* Enable bus mastering, memory access and MWI */
	MCF_PCISCR = MCF_PCISCR_B | MCF_PCISCR_M | MCF_PCISCR_MW;

	/* Setup burst parameters */
	MCF_PCICR1 = MCF_PCICR1_LATTIMER(32) |
	    	     MCF_PCICR1_CACHELINESIZE(PCI_CACHE_LINE);

	MCF_PCICR2 = 0;
	/*MCF_PCICR2_MINGNT(PCI_MINGNT) |
		MCF_PCICR2_MAXLAT(PCI_MAXLAT);
	*/
	/* Turn on error signaling */
	MCF_PCIICR = MCF_PCIICR_TAE | MCF_PCIICR_IAE | PCI_RETRIES;
	MCF_PCIGSCR |= MCF_PCIGSCR_SEE;
	/*
	 * Configure Initiator Windows
	 * Window 0: 128M PCI Memory @ HOST_MEM_BASE, 1:1 mapping
	 * Window 1: 64K  I/O Memory @ HOST_IO_BASE,  1:0 mapping
	 */
	MCF_PCIIW0BTAR = WxBAR(HOST_MEM_BASE, PCI_MEM_BASE, PCI_MEM_SIZE);
	MCF_PCIIW1BTAR = WxBAR(HOST_IO_BASE,  PCI_IO_BASE_ADDR,  PCI_IO_SIZE);

	MCF_PCIIWCR = MCF_PCIIWCR_WINCTRL1_IO |
	    	      MCF_PCIIWCR_WINCTRL0_MEMRDLINE;

	/* Target PCI DMA Windows */
	MCF_PCIBAR1   = PCI_DMA_BASE;
	MCF_PCITBATR1 = HOST_DMA_BASE | MCF_PCITBATR1_EN;
        MCF_PCIBAR0   = MCF_RAMBAR0;;
        MCF_PCITBATR0 = MCF_RAMBAR0 | MCF_PCITBATR0_EN;
        DBG("PCI TCR %x,MCF_PCIBAR1 %x,MCF_PCITBATR1 %x."
		"MCF_PCIBAR0 %x,MCF_PCITBATR9 %x\n", MCF_PCITCR, MCF_PCIBAR1,
		MCF_PCITBATR1, MCF_PCIBAR0, MCF_PCITBATR0);
	/* Enable internal PCI controller interrupts */
	MCF_ICR(ISC_PCI_XLB) = ILP_PCI_XLB;
	/*request_irq(64+ISC_PCI_XLB, xlb_interrupt, 
			SA_INTERRUPT, "PCI XL Bus", (void*)-1);
	enable_irq (64+ISC_PCI_XLB);
	*/
        if(request_irq(64+ISC_PCI_XLB, xlb_interrupt, 
                         IRQF_DISABLED, "PCI XL Bus", (void*)-1)){
		printk("Cannot allocate ISC_PCI_XLB  IRQ\n");
                return (struct pci_bus_info *)-EBUSY;
	}

	MCF_ICR(ISC_PCI_ARB) = ILP_PCI_ARB;
	/*request_irq(64+ISC_PCI_ARB, arb_interrupt, 
			SA_INTERRUPT, "PCI Arbiter", (void*)-1);
	enable_irq (64+ISC_PCI_ARB);
	*/
        if(request_irq(64+ISC_PCI_ARB, arb_interrupt,
                        IRQF_DISABLED, "PCI Arbiter", (void*)-1)){
                printk("Cannot allocate ISC_PCI_ARB  IRQ\n");
                return (struct pci_bus_info *)-EBUSY;
        }

	/* Set slots interrupt setting */
	for (i = 0; i < N_IRQS; i++) 
	{	
		/* Set trailing edge for PCI interrupts */
		MCF_EPPAR &= ~MCF_EPPAR_EPPA(irq_lines[i], 0x3);
		if (irq_lines[i] == 5)
			MCF_EPPAR |= MCF_EPPAR_EPPA(irq_lines[i],  MCF_EPPAR_EPPAx_FALLING);
		else
                        MCF_EPPAR |= MCF_EPPAR_EPPA(irq_lines[i],  0/*MCF_EPPAR_EPPAx_FALLING*/);
		/* Turn on irq line in eport */
		MCF_EPIER |= MCF_EPIER_EPIE(irq_lines[i]);

		/* Enable irq in gpio */
		if (irq_lines[i] == 5)
		    MCF_PAR_FECI2CIRQ |= 1;
		    
		if (irq_lines[i] == 6)
		    MCF_PAR_FECI2CIRQ |= 2;   

		/* Register external interrupt handlers */
		sprintf(irq_name[i], "PCI IRQ%d", irq_lines[i]);
		/*request_irq(64 + irq_lines[i], eint_handler, 
			    SA_SHIRQ, irq_name[i], (void*)-1);
		enable_irq(64 + irq_lines[i]);*/
        	if(request_irq(64 + irq_lines[i], eint_handler, 
				IRQF_SHARED, irq_name[i], (void*)-1)){
                	printk("Cannot allocate irq_lines[%d] IRQ\n", irq_lines[i]);
                	return (struct pci_bus_info *)-EBUSY;
        	}
	}

	/* Clear PCI Reset and wait for devices to reset */
	MCF_PCIGSCR &= ~MCF_PCIGSCR_PR;
	schedule_timeout((5 * HZ) / 10);
	/* Remap initiator windows (should be 1:1 to the physical memory) */
	pci_mem_va = (int) ioremap_nocache(HOST_MEM_BASE, PCI_MEM_SIZE + PCI_IO_SIZE);
#if 1
	printk("%s: MEMBase_phy %x, Virt %x, len %x\n",__FUNCTION__,
		HOST_MEM_BASE,pci_mem_va,PCI_MEM_SIZE + PCI_IO_SIZE);
#endif
	BUG_ON(pci_mem_va != HOST_MEM_BASE);

	/* Setup bios32 and pci bus driver callbacks */
	bus.m68k_pci_ops = &bus_ops;
	bus.fixup = coldfire_fixup;
	bus.conf_device = configure_device;

	return &bus;
}


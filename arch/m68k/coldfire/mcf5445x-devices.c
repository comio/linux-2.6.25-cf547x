/*
 * arch/m68k/coldfire/mcf5445x-devices.c
 *
 * Coldfire M5445x Platform Device Configuration
 *
 * Based on the Freescale MXC devices.c
 *
 * Copyright (c) 2007 Freescale Semiconductor, Inc.
 *	Kurt Mahan <kmahan@freescale.com>
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>

#include <asm/coldfire.h>
#include <asm/mcfsim.h>

/* ATA Interrupt */
#define IRQ_ATA		(64 + 64 + 54)

/* ATA Base */
#define	BASE_IO_ATA	0x90000000

#define ATA_IER		MCF_REG08(BASE_IO_ATA+0x2c)	/* int enable reg */
#define ATA_ICR		MCF_REG08(BASE_IO_ATA+0x30)	/* int clear reg */

/*
 * On-chip PATA
 */
#if defined(CONFIG_PATA_FSL) || defined(CONFIG_PATA_FSL_MODULE)
static int ata_init(struct platform_device *pdev)
{
	/* clear ints */
	ATA_IER = 0x00;
	ATA_ICR = 0xff;

	/* setup shared pins */
	MCF_GPIO_PAR_FEC = (MCF_GPIO_PAR_FEC & MCF_GPIO_PAR_FEC_FEC1_MASK) |
			   MCF_GPIO_PAR_FEC_FEC1_ATA;

	MCF_GPIO_PAR_FECI2C = (MCF_GPIO_PAR_FECI2C &
	  		      (MCF_GPIO_PAR_FECI2C_MDC1_MASK &
			      MCF_GPIO_PAR_FECI2C_MDIO1_MASK)) |
	  		      MCF_GPIO_PAR_FECI2C_MDC1_ATA_DIOR |
	  		      MCF_GPIO_PAR_FECI2C_MDIO1_ATA_DIOW;

	MCF_GPIO_PAR_ATA = MCF_GPIO_PAR_ATA_BUFEN |
			   MCF_GPIO_PAR_ATA_CS1 |
			   MCF_GPIO_PAR_ATA_CS0 |
			   MCF_GPIO_PAR_ATA_DA2 |
			   MCF_GPIO_PAR_ATA_DA1 |
			   MCF_GPIO_PAR_ATA_DA0 |
			   MCF_GPIO_PAR_ATA_RESET_RESET |
			   MCF_GPIO_PAR_ATA_DMARQ_DMARQ |
			   MCF_GPIO_PAR_ATA_IORDY_IORDY;

	MCF_GPIO_PAR_PCI = (MCF_GPIO_PAR_PCI &
			     (MCF_GPIO_PAR_PCI_GNT3_MASK &
			      MCF_GPIO_PAR_PCI_REQ3_MASK)) |
			   MCF_GPIO_PAR_PCI_GNT3_ATA_DMACK |
			   MCF_GPIO_PAR_PCI_REQ3_ATA_INTRQ;

	return 0;
}

static void ata_exit(void)
{
	printk(KERN_INFO "** ata_exit\n");
}

static int ata_get_clk_rate(void)
{
	return MCF_BUSCLK;
}

/* JKM -- move these to a header file */
#define MCF_IDE_DMA_WATERMARK	32	/* DMA watermark level in bytes */
#define MCF_IDE_DMA_BD_NR	(512/3/4) /* number of BDs per channel */

static struct fsl_ata_platform_data ata_data = {
	.init             = ata_init,
	.exit             = ata_exit,
	.get_clk_rate     = ata_get_clk_rate,
#ifdef CONFIG_PATA_FSL_USE_DMA
        .udma_mask        = 0x0F, /* the board handles up to UDMA3 */
        .fifo_alarm       = MCF_IDE_DMA_WATERMARK / 2,
        .max_sg           = MCF_IDE_DMA_BD_NR,
#endif
};

static struct resource pata_fsl_resources[] = {
	[0] = {		/* I/O */
		.start		= BASE_IO_ATA,
		.end		= BASE_IO_ATA + 0x000000d8,
		.flags		= IORESOURCE_MEM,
	},
	[2] = {		/* IRQ */
		.start		= IRQ_ATA,
		.end		= IRQ_ATA,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device pata_fsl_device = {
	.name			= "pata_fsl",
	.id			= -1,
	.num_resources		= ARRAY_SIZE(pata_fsl_resources),
	.resource		= pata_fsl_resources,
	.dev			= {
		.platform_data	= &ata_data,
		.coherent_dma_mask = ~0,	/* $$$ REVISIT */
	},
};

static inline void mcf5445x_init_pata(void)
{
	(void)platform_device_register(&pata_fsl_device);
}
#else
static inline void mcf5445x_init_pata(void)
{
}
#endif

static int __init mcf5445x_init_devices(void)
{
	printk(KERN_INFO "MCF5445x INIT_DEVICES\n");
#if 0
	mcf5445x_init_pata();
#endif

	return 0;
}
arch_initcall(mcf5445x_init_devices);

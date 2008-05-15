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
#include <linux/mtd/physmap.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>

#include <asm/coldfire.h>
#include <asm/mcfsim.h>

static struct resource coldfire_i2c_resources[] = {
	[0] = {		/* I/O */
		.start		= MCF_MBAR + 0x008F00,
		.end		= MCF_MBAR + 0x008F20,
		.flags		= IORESOURCE_MEM,
	},
	[2] = {		/* IRQ */
		.start		= 40,
		.end		= 40,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device coldfire_i2c_device = {
	.name			= "MCF548X-i2c",
	.id			= -1,
	.num_resources		= ARRAY_SIZE(coldfire_i2c_resources),
	.resource		= coldfire_i2c_resources,
};

static struct resource coldfire_sec_resources[] = {
	[0] = {         /* I/O */
		.start          = MCF_MBAR + 0x00020000,
		.end            = MCF_MBAR + 0x00033000,
		.flags          = IORESOURCE_MEM,
	},
	[2] = {         /* IRQ */
		.start          = ISC_SEC,
		.end            = ISC_SEC,
		.flags          = IORESOURCE_IRQ,
	},
};

static struct platform_device coldfire_sec_device = {
	.name                   = "fsl-sec1",
	.id                     = -1,
	.num_resources          = ARRAY_SIZE(coldfire_sec_resources),
	.resource               = coldfire_sec_resources,
};

#if defined(CONFIG_MTD_PHYSMAP)
static struct physmap_flash_data mcf5485_flash_data = {
	.width          = 2,
};

static struct resource mcf5485_flash_resource = {
	.start          = 0xf8000000,
	.end            = 0xf80fffff,
	.flags          = IORESOURCE_MEM,
};

static struct platform_device mcf5485_flash_device = {
	.name           = "physmap-flash",
	.id             = 0,
	.dev            = {
		.platform_data  = &mcf5485_flash_data,
	},
	.num_resources  = 1,
	.resource       = &mcf5485_flash_resource,
};
#endif

static int __init mcf5485_init_devices(void)
{
	printk(KERN_INFO "MCF5485x INIT_DEVICES\n");

	platform_device_register(&coldfire_i2c_device);
	platform_device_register(&coldfire_sec_device);
/*#if defined(CONFIG_MTD_PHYSMAP)
	platform_device_register(&mcf5485_flash_device);
#endif*/
	return 0;
}
arch_initcall(mcf5485_init_devices);

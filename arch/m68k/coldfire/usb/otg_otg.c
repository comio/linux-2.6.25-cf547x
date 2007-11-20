/*
 * Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * platform_device registration for ULPI OTG device
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>

#include <asm/mcfsim.h>

#define USB_OTGREGS_BASE MCF_REG32(0xFC0B0000)
#define INT_USB		(64 + 64 + 47)	/* INTC1:47 16.2.9.1 */
#define INT_UOCSR	(64 + 64 + 53)	/* INTC1:53 16.2.9.1 */

extern int usbotg_init(struct platform_device *pdev);
extern void usbotg_uninit(struct fsl_usb2_platform_data *pdata);
extern struct fsl_usb2_platform_data mxc_otg_config;

static void otg_otg_release(struct device *dev)
{
	/* normally not freed */
}

/* *INDENT-OFF* */
static struct resource otg_otg_resources[] = {
	[0] = {
		.start = (u32) (&USB_OTGREGS_BASE),
		.end   = (u32) (&USB_OTGREGS_BASE + 0x1ff),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_USB,
		.flags = IORESOURCE_IRQ,
	},
};

/*!
 * OTG device
 */
static u64 otg_otg_dmamask = ~(u32) 0;
static struct platform_device otg_otg_device = {
	.name = "fsl-usb2-otg",
	.id   = -1,
	.dev  = {
		.release           = otg_otg_release,
		.dma_mask          = &otg_otg_dmamask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data     = &mxc_otg_config,
	},
	.resource = otg_otg_resources,
	.num_resources = ARRAY_SIZE(otg_otg_resources),
};
/* *INDENT-ON* */

static int __init mx31_otg_otg_init(void)
{
	int rc = 0;

	pr_debug("register OTG otg res=0x%p, size=%d\n",
		 otg_otg_device.resource, otg_otg_device.num_resources);

	rc = platform_device_register(&otg_otg_device);
	if (rc) {
		pr_debug("can't register ULPI OTG dvc, %d\n", rc);
	} else {
		printk(KERN_INFO "usb: OTG ULPI transceiver registered\n");
		pr_debug("otg_otg_device=0x%p  resources=0x%p.\n",
			 &otg_otg_device, otg_otg_device.resource);
	}

	return rc;
}

subsys_initcall(mx31_otg_otg_init);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ULPI OTG device registration");
MODULE_LICENSE("GPL");

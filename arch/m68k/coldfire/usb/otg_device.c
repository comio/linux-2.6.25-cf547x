/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/usb/otg.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/usb/fsl_xcvr.h>

#include <asm/system.h>
#include <asm/coldfire.h>

#define USB_OTGREGS_BASE MCF_REG32(0xFC0B0000)
#define INT_USB		(64 + 64 + 47)	/* INTC1:47 16.2.9.1 */
#define INT_UOCSR	(64 + 64 + 53)	/* INTC1:53 16.2.9.1 */

extern int usbotg_init(struct platform_device *pdev);
extern void usbotg_uninit(struct fsl_usb2_platform_data *pdata);
extern struct fsl_usb2_platform_data mxc_otg_config;

struct platform_device otg_udc_device;

/*!
 * OTG Gadget device
 */

static void usb_release(struct device *dev)
{
	/* normally not freed */
}

static u64 udc_dmamask = ~(u32) 0;
static struct resource otg_udc_resources[] = {
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


struct platform_device otg_udc_device = {
	.name = "fsl-usb2-udc",
	.id   = -1,
	.dev  = {
		.release           = usb_release,
		.dma_mask          = &udc_dmamask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data     = &mxc_otg_config,
		},
	.resource = otg_udc_resources,
	.num_resources = ARRAY_SIZE(otg_udc_resources),
};

static int __init udc_init(void)
{
	int rc __attribute((unused));

	rc = platform_device_register(&otg_udc_device);
	if (rc)
		printk(KERN_ERR "usb: can't register OTG Gadget, rc=%d\n", rc);
	else
		printk(KERN_INFO "usb: OTG Gadget registered\n");
	return rc;
}

subsys_initcall(udc_init);

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
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/usb/fsl_xcvr.h>

#include <asm/system.h>
#include <asm/mcfsim.h>

#define USB_OTGREGS_BASE MCF_REG32(0xFC0B0000)
#define INT_USB		(64 + 64 + 47)	/* INTC1:47 16.2.9.1 */
#define INT_UOCSR	(64 + 64 + 53)	/* INTC1:53 16.2.9.1 */

struct platform_device *otg_host_device;

extern struct platform_device *host_pdev_register(struct resource *res,
						  int n_res,
						  struct fsl_usb2_platform_data
						  *config);

extern int usbotg_init(struct platform_device *pdev);
extern void usbotg_uninit(struct fsl_usb2_platform_data *pdata);
extern struct fsl_usb2_platform_data mxc_otg_config;

/*!
 * OTG host config
 */
static struct resource otg_host_resources[] = {
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

static int __init otg_host_init(void)
{
	otg_host_device = host_pdev_register(otg_host_resources,
					     ARRAY_SIZE(otg_host_resources),
					     &mxc_otg_config);
	return 0;
}

subsys_initcall(otg_host_init);

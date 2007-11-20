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
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/usb/fsl_xcvr.h>

#include <asm/system.h>
#include <asm/coldfire.h>

extern void fsl_usb_enable_clk(void);
extern void fsl_usb_disable_clk(void);
extern int fsl_usb_mem_init(struct platform_device *pdev);

extern struct fsl_xcvr_ops *xc_ops[];

static int otg_used;

int usbotg_init(struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata;
	struct fsl_xcvr_ops *xops = xc_ops[USB_CTRLR_OTG];
	int rc;

	pdata = (struct fsl_usb2_platform_data *)pdev->dev.platform_data;

	pr_debug("%s: pdev=0x%p  pdata=0x%p\n", __FUNCTION__, pdev, pdata);

	if (!xops) {
		printk(KERN_ERR "OTG transceiver ops missing\n");
		return -EINVAL;
	}
	pdata->xcvr_ops = xops;
	pdata->xcvr_type = xops->xcvr_type;

	if (!otg_used) {
		/* request_mem_region and ioremap registers */
		rc = fsl_usb_mem_init(pdev);
		if (rc)
			return rc;

		fsl_usb_enable_clk();

		if (xops->init)
			xops->init(pdev);
	}

	otg_used++;
	pr_debug("%s: success\n", __FUNCTION__);
	return 0;
}

void usbotg_uninit(struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata;
	pdata = (struct fsl_usb2_platform_data *)pdev->dev.platform_data;

	pr_debug("%s\n", __FUNCTION__);

	otg_used--;
	if (!otg_used) {
		if (pdata->xcvr_ops && pdata->xcvr_ops->uninit)
			pdata->xcvr_ops->uninit(pdev);

		iounmap(pdata->regs);
		release_mem_region(pdata->r_start, pdata->r_len);

		pdata->regs = NULL;
		pdata->r_start = pdata->r_len = 0;

		fsl_usb_disable_clk();
	}
}

struct fsl_usb2_platform_data mxc_otg_config = {
	.name            = "OTG",
	.platform_init   = usbotg_init,
	.platform_uninit = usbotg_uninit,
	.es              = 1,
	.big_endian_mmio = 1,
	.big_endian_desc = 1,
	.le_setup_buf    = 1,
	.does_otg        = 1,
	.power_budget    = 500,		/* 500 mA max power */
	.max_ep_nr       = 4,		/* DDD read from a register ? */
	.phy_mode	 = FSL_USB2_PHY_ULPI, /* DDD redundant with xcvr_type */
};

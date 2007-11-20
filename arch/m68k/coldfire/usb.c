/*
 *
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *	otg_{get,set}_transceiver() are from arm/plat-omap/usb.c.
 *	which is Copyright (C) 2004 Texas Instruments, Inc.
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
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/usb/otg.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/usb/fsl_xcvr.h>


/* The dmamask must be set for EHCI to work */
static u64 ehci_dmamask = ~(u32) 0;

struct fsl_xcvr_ops *xc_ops[3] = { NULL };

void fsl_usb_enable_clk(void)
{
}
EXPORT_SYMBOL(fsl_usb_enable_clk);

void fsl_usb_disable_clk(void)
{
}
EXPORT_SYMBOL(fsl_usb_disable_clk);

void fsl_usb_xcvr_register(struct fsl_xcvr_ops *xcvr_ops)
{
	pr_debug("%s ctrlr=%d\n", __FUNCTION__, xcvr_ops->ctrlr);
	xc_ops[xcvr_ops->ctrlr] = xcvr_ops;

}
EXPORT_SYMBOL(fsl_usb_xcvr_register);

void fsl_usb_xcvr_unregister(enum fsl_usb_ctrlr ctrlr)
{
	pr_debug("%s ctrlr=%d\n", __FUNCTION__, ctrlr);
	xc_ops[ctrlr] = NULL;
}
EXPORT_SYMBOL(fsl_usb_xcvr_unregister);

/*!
 * Register an instance of a USB host platform device.
 *
 * @param	res:	resource pointer
 * @param       n_res:	number of resources
 * @param       config: config pointer
 *
 * @return      newly-registered platform_device
 *
 * DDD fix this comment:
 * The USB controller supports 3 host interfaces, and the
 * kernel can be configured to support some number of them.
 * Each supported host interface is registered as an instance
 * of the "fsl-ehci" device.  Call this function multiple times
 * to register each host interface.
 */
static int instance_id;
struct platform_device *host_pdev_register(struct resource *res, int n_res,
					  struct fsl_usb2_platform_data *config)
{
	struct platform_device *pdev;

	pr_debug("register host res=0x%p, size=%d\n", res, n_res);

	pdev = platform_device_register_simple("fsl-ehci",
					       instance_id, res, n_res);
	if (IS_ERR(pdev)) {
		printk(KERN_ERR "usb: can't register %s Host, %ld\n",
		       config->name, PTR_ERR(pdev));
		return NULL;
	}

	pdev->dev.coherent_dma_mask = 0xffffffff;
	pdev->dev.dma_mask = &ehci_dmamask;

	/*
	 * platform_device_add_data() makes a copy of
	 * the platform_data passed in.  That makes it
	 * impossible to share the same config struct for
	 * all OTG devices (host,gadget,otg).  So, just
	 * set the platform_data pointer ourselves.
	 */
	pdev->dev.platform_data = config;

	printk(KERN_INFO "usb: %s Host registered\n", config->name);
	pr_debug("pdev=0x%p  dev=0x%p  resources=0x%p  pdata=0x%p\n",
		 pdev, &pdev->dev, pdev->resource, pdev->dev.platform_data);

	instance_id++;

	return pdev;
}


int fsl_usb_mem_init(struct platform_device *pdev)
{
	struct resource *res;
	struct fsl_usb2_platform_data *pdata;

	pdata = (struct fsl_usb2_platform_data *)pdev->dev.platform_data;

	pr_debug("%s: pdev=0x%p  pdata=0x%p\n", __FUNCTION__, pdev, pdata);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no MEM resource.\n");
		return -ENODEV;
	}

	pdata->r_start = res->start;
	pdata->r_len = res->end - res->start + 1;
	pr_debug("%s: MEM resource start=0x%x  len=0x%x\n", pdata->name,
		 res->start, pdata->r_len);

	if (!request_mem_region(pdata->r_start, pdata->r_len, "OTG")) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		return -EBUSY;
	}
	pdata->regs = ioremap(pdata->r_start, pdata->r_len);
	pr_debug("ioremapped to 0x%p\n", pdata->regs);

	if (pdata->regs == NULL) {
		dev_err(&pdev->dev, "ioremap failed\n");
		release_mem_region(pdata->r_start, pdata->r_len);
		return -EFAULT;
	}

	pr_debug("%s: success\n", __FUNCTION__);
	return 0;
}


#if defined(CONFIG_USB_OTG)
static struct otg_transceiver *xceiv;

/**
 * otg_get_transceiver - find the (single) OTG transceiver driver
 *
 * Returns the transceiver driver, after getting a refcount to it; or
 * null if there is no such transceiver.  The caller is responsible for
 * releasing that count.
 */
struct otg_transceiver *otg_get_transceiver(void)
{
	pr_debug("%s xceiv=0x%p\n", __FUNCTION__, xceiv);
	if (xceiv)
		get_device(xceiv->dev);
	return xceiv;
}
EXPORT_SYMBOL(otg_get_transceiver);

int otg_set_transceiver(struct otg_transceiver *x)
{
	pr_debug("%s xceiv=0x%p  x=0x%p\n", __FUNCTION__, xceiv, x);
	if (xceiv && x)
		return -EBUSY;
	xceiv = x;
	return 0;
}
EXPORT_SYMBOL(otg_set_transceiver);
#endif

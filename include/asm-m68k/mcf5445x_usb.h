/*
 * Duck Schmid duck@freescale.com
 *
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

#include <asm/mcfsim.h>

/* ehci_arc_hc_driver.flags value */
#define FSL_PLATFORM_HC_FLAGS (HCD_USB2 | HCD_MEMORY)

static inline int fsl_platform_verify(struct platform_device *pdev)
{
	return 0;
}

static inline void fsl_platform_usb_setup(struct usb_hcd *hcd)
{
}

static inline void fsl_platform_set_host_mode(struct usb_hcd *hcd)
{
	unsigned int temp;
	struct fsl_usb2_platform_data *pdata;
	struct fsl_usb_host_regs *regs;

	pdata = hcd->self.controller->platform_data;
	regs = pdata->regs;

	if (pdata->xcvr_ops && pdata->xcvr_ops->set_host)
		pdata->xcvr_ops->set_host();

	/* set host mode and select "big endian" */
	temp = fsl_readl(&regs->usbmode);
	fsl_writel(temp | USBMODE_CM_HOST | USBMODE_ES, &regs->usbmode);

	pr_debug("%s: set usbmode to 0x%x\n\n", __FUNCTION__,
		fsl_readl(&regs->usbmode));

}

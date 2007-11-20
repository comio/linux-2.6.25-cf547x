/*
 * drivers/usb/host/ehci-arc.c
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

/* Note: this file is #included by ehci-hcd.c */

#include <linux/platform_device.h>
#include <linux/usb/fsl_usb2.h>
#include <linux/fsl_devices.h>
#include <linux/usb/otg.h>

#include "ehci-fsl.h"


/* FIXME: Power Managment is un-ported so temporarily disable it */
#undef CONFIG_PM

/* PCI-based HCs are common, but plenty of non-PCI HCs are used too */

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

/**
 * usb_hcd_fsl_probe - initialize FSL-based HCDs
 * @drvier: Driver to be used for this HCD
 * @pdev: USB Host Controller being probed
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 */
static int usb_hcd_fsl_probe(const struct hc_driver *driver,
			     struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;
	struct usb_hcd *hcd;
	struct resource *res;
	int irq;
	int retval;

	pr_debug("initializing FSL-SOC USB Controller\n");

	/* Need platform data for setup */
	if (!pdata) {
		dev_err(&pdev->dev,
			"No platform data for %s.\n", pdev->dev.bus_id);
		return -ENODEV;
	}

	retval = fsl_platform_verify(pdev);
	if (retval)
		return retval;

	/*
	 * do platform specific init: check the clock, grab/config pins, etc.
	 */
	if (pdata->platform_init && pdata->platform_init(pdev)) {
		retval = -ENODEV;
		goto err1;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			pdev->dev.bus_id);
		return -ENODEV;
	}
	irq = res->start;

	fsl_platform_set_vbus_power(pdev, 1);

	hcd = usb_create_hcd(driver, &pdev->dev, pdev->dev.bus_id);
	if (!hcd) {
		retval = -ENOMEM;
		goto err1;
	}

	if (pdata->regs) {
		pr_debug("REGS: using pdata->regs (0x%p)\n", pdata->regs);
		hcd->regs = pdata->regs;
		hcd->rsrc_start = pdata->r_start;
		hcd->rsrc_len = pdata->r_len;
	} else {
		pr_debug("REGS: NO pdata->regs\n");
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			dev_err(&pdev->dev, "Found HC with no register addr. "
				"Check %s setup!\n", pdev->dev.bus_id);
			retval = -ENODEV;
			goto err2;
		}
		hcd->rsrc_start = res->start;
		hcd->rsrc_len = res->end - res->start + 1;

		/*
		printk("DDD %s(): rsrc_start=0x%llx  rsrc_len=0x%llx  "
			"pdata=0x%p\n", __FUNCTION__,
			hcd->rsrc_start, hcd->rsrc_len, pdata);
		*/

		if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
					driver->description)) {
			dev_dbg(&pdev->dev, "request_mem_region failed\n");
			retval = -EBUSY;
			goto err2;
		}
		hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);

		if (hcd->regs == NULL) {
			dev_dbg(&pdev->dev, "error mapping memory\n");
			retval = -EFAULT;
			goto err3;
		}
	}
	hcd->power_budget = pdata->power_budget;

	/* DDD
	 * the following must be done by this point, otherwise the OTG
	 * host port doesn't make it thru initializtion.
	 * ehci_halt(), called by ehci_fsl_setup() returns -ETIMEDOUT
	 */
	fsl_platform_set_host_mode(hcd);

	retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (retval != 0)
		goto err4;

#if defined(CONFIG_USB_OTG)
	if (pdata->does_otg) {
		struct ehci_hcd *ehci = hcd_to_ehci(hcd);

		ehci->transceiver = otg_get_transceiver();

		if (ehci->transceiver) {
			retval = otg_set_host(ehci->transceiver,
					      &ehci_to_hcd(ehci)->self);
			if (retval) {
				if (ehci->transceiver)
					put_device(ehci->transceiver->dev);
				goto err3;
			}
		} else {
			printk(KERN_ERR "can't find transceiver\n");
			retval = -ENODEV;
			goto err3;
		}
	}
#endif

	return retval;

err4:
	/* DDD only if we did the iomap() iounmap(hcd->regs); */
err3:
	/* DDD only if we did a request_
	 * release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	 */
err2:
	usb_put_hcd(hcd);
err1:
	dev_err(&pdev->dev, "init %s fail, %d\n", pdev->dev.bus_id, retval);
	if (pdata->platform_uninit)
		pdata->platform_uninit(pdev);
	return retval;
}

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_fsl_remove - shutdown processing for FSL-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_fsl_probe().
 *
 */
static void usb_hcd_fsl_remove(struct usb_hcd *hcd,
			       struct platform_device *pdev)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	/* DDD shouldn't we turn off the power here? */
	fsl_platform_set_vbus_power(pdev, 0);

	usb_remove_hcd(hcd);

	if (ehci->transceiver) {
		(void)otg_set_host(ehci->transceiver, 0);
		put_device(ehci->transceiver->dev);
	}
	usb_put_hcd(hcd);

	/*
	 * do platform specific un-initialization:
	 * release iomux pins, etc.
	 */
	if (pdata->platform_uninit)
		pdata->platform_uninit(pdev);
}

/* called after powerup, by probe or system-pm "wakeup" */
static int ehci_fsl_reinit(struct ehci_hcd *ehci)
{
	fsl_platform_usb_setup(ehci_to_hcd(ehci));
	ehci_port_power(ehci, 0);

	return 0;
}

/* called during probe() after chip reset completes */
static int ehci_fsl_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;
	struct fsl_usb2_platform_data *pdata;
	pdata = hcd->self.controller-> platform_data;

	ehci->big_endian_desc = pdata->big_endian_desc;
	ehci->big_endian_mmio = pdata->big_endian_mmio;

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + 0x100;
	ehci->regs = hcd->regs + 0x100 +
	    HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));

	pr_debug("%s(): ehci->caps=0x%p  ehci->regs=0x%p\n", __FUNCTION__,
		 ehci->caps, ehci->regs);

	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	retval = ehci_halt(ehci);
	if (retval)
		return retval;

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

	ehci->is_tdi_rh_tt = 1;

	ehci->sbrn = 0x20;

	ehci_reset(ehci);

	retval = ehci_fsl_reinit(ehci);
	return retval;
}

static const struct hc_driver ehci_fsl_hc_driver = {
	.description = hcd_name,
	.product_desc = "Freescale On-Chip EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = FSL_PLATFORM_HC_FLAGS,

	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_fsl_setup,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
};

#ifdef CONFIG_USB_OTG
/*
 * Holding pen for all the EHCI registers except port_status,
 * which is a zero element array and hence takes no space.
 * The port_status register is saved in usb_ehci_portsc.
 */
volatile static struct ehci_regs usb_ehci_regs;
static u32 usb_ehci_portsc;

/* suspend/resume, section 4.3 */

/* These routines rely on the bus (pci, platform, etc)
 * to handle powerdown and wakeup, and currently also on
 * transceivers that don't need any software attention to set up
 * the right sort of wakeup.
 *
 * They're also used for turning on/off the port when doing OTG.
 */
static int ehci_fsl_drv_suspend(struct platform_device *pdev,
				pm_message_t message)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	u32 tmp;

	pr_debug("%s pdev=0x%p  ehci=0x%p  hcd=0x%p\n",
		 __FUNCTION__, pdev, ehci, hcd);
	pr_debug("%s ehci->regs=0x%p  hcd->regs=0x%p  hcd->state=%d\n",
		 __FUNCTION__, ehci->regs, hcd->regs, hcd->state);

	hcd->state = HC_STATE_SUSPENDED;
	pdev->dev.power.power_state = PMSG_SUSPEND;

	if (hcd->driver->suspend)
		return hcd->driver->suspend(hcd, message);

	/* ignore non-host interrupts */
	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	tmp = ehci_readl(ehci, &ehci->regs->command);
	tmp &= ~CMD_RUN;
	ehci_writel(ehci, tmp, &ehci->regs->command);

	memcpy((void *)&usb_ehci_regs, ehci->regs, sizeof(struct ehci_regs));
	usb_ehci_portsc = ehci_readl(ehci, &ehci->regs->port_status[0]);

	/* clear the W1C bits */
	usb_ehci_portsc &= cpu_to_hc32(ehci, ~PORT_RWC_BITS);

	fsl_platform_set_vbus_power(pdev, 0);
	/* clear PP to cut power to the port */
	tmp = ehci_readl(ehci, &ehci->regs->port_status[0]);
	tmp &= ~PORT_POWER;
	ehci_writel(ehci, tmp, &ehci->regs->port_status[0]);

	return 0;
}

static int ehci_fsl_drv_resume(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	u32 tmp;
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	pr_debug("%s pdev=0x%p  pdata=0x%p  ehci=0x%p  hcd=0x%p\n",
		 __FUNCTION__, pdev, pdata, ehci, hcd);

	pr_debug("%s ehci->regs=0x%p  hcd->regs=0x%p",
		 __FUNCTION__, ehci->regs, hcd->regs);

	memcpy(ehci->regs, (void *)&usb_ehci_regs, sizeof(struct ehci_regs));
	ehci_writel(ehci, usb_ehci_portsc, &ehci->regs->port_status[0]);

	tmp = USBMODE_CM_HOST | (pdata->es ? USBMODE_ES : 0);
	ehci_writel(ehci, tmp, hcd->regs + FSL_SOC_USB_USBMODE);

	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	hcd->state = HC_STATE_RUNNING;
	pdev->dev.power.power_state = PMSG_ON;

	tmp = ehci_readl(ehci, &ehci->regs->command);
	tmp |= CMD_RUN;
	ehci_writel(ehci, tmp, &ehci->regs->command);

	fsl_platform_set_vbus_power(pdev, 1);

	usb_hcd_resume_root_hub(hcd);

	return 0;
}
#endif				/* CONFIG_USB_OTG */

static int ehci_fsl_drv_probe(struct platform_device *pdev)
{
	if (usb_disabled())
		return -ENODEV;

	return usb_hcd_fsl_probe(&ehci_fsl_hc_driver, pdev);
}

static int ehci_fsl_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_hcd_fsl_remove(hcd, pdev);

	return 0;
}

MODULE_ALIAS("fsl-ehci");

static struct platform_driver ehci_fsl_driver = {
	.probe = ehci_fsl_drv_probe,
	.remove = ehci_fsl_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
#ifdef CONFIG_USB_OTG
	.suspend = ehci_fsl_drv_suspend,
	.resume  = ehci_fsl_drv_resume,
#endif
	.driver = {
		   .name = "fsl-ehci",
		   },
};

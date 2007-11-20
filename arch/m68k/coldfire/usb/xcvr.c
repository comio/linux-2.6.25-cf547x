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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/usb/fsl_xcvr.h>
#include <linux/usb/fsl_usb2.h>

#include <asm/mcfsim.h>

extern void fsl_usb_xcvr_register(struct fsl_xcvr_ops *xcvr_ops);
extern void fsl_usb_xcvr_unregister(enum fsl_usb_ctrlr ctrlr);

#define MCF_SCM_BCR		MCF_REG32(0xFC040024)
#define MCF_SCM_BCR_GBR		(1 << 9)	/* global bursts for read */
#define MCF_SCM_BCR_GBW		(1 << 8)	/* global bursts for write */
#define MCF_SCM_BCR_SBE_ALL	(0xff << 0)	/* slave burst enable */


#ifdef ULPI_DEBUG
void print_ulpi_regs(void)
{
	pr_debug("MCF_SCM_BCR=0x%08lx  MCF_CCM_MISCCR=0x%08x  "
		 "MCF_GPIO_PAR_DMA=0x%08x  MCF_GPIO_PAR_USB=08%08x  "
		 "MCF_GPIO_PAR_FEC=08%08x\n",
		 MCF_SCM_BCR, MCF_CCM_MISCCR, MCF_GPIO_PAR_DMA,
		 MCF_GPIO_PAR_USB, MCF_GPIO_PAR_FEC);
}
EXPORT_SYMBOL(print_ulpi_regs);
#endif


static void xcvr_init(struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;
	struct fsl_xcvr_ops *this = pdata->xcvr_ops;
	struct fsl_usb_host_regs *regs = pdata->regs;

	pr_debug("%s: ctrlr=%d  pdata=0x%p  regs=0x%p\n", __FUNCTION__,
		 this->ctrlr, pdata, pdata->regs);

	/* enable USB read, write and slave bursts */
	MCF_SCM_BCR = MCF_SCM_BCR_GBR | MCF_SCM_BCR_GBW | MCF_SCM_BCR_SBE_ALL;

	/* Use external clock source if PLL isn't a multiple of 60MHz */
	MCF_CCM_MISCCR &= ~MCF_CCM_MISCCR_USBSRC;

	/* Initialize the USB Clock: use USB input clock */
	MCF_GPIO_PAR_DMA = (MCF_GPIO_PAR_DMA & MCF_GPIO_PAR_DMA_DREQ1_MASK) |
			   MCF_GPIO_PAR_DMA_DREQ1_USB_CLKIN;

	switch (this->xcvr_type) {
	case PORTSCX_PTS_ULPI:
		/* Enable the required ULPI signals */
		MCF_GPIO_PAR_DMA = (MCF_GPIO_PAR_DMA &
				    MCF_GPIO_PAR_DMA_DACK1_MASK) |
				    MCF_GPIO_PAR_DMA_DACK1_ULPI_DIR;

		MCF_GPIO_PAR_USB = MCF_GPIO_PAR_USB_VBUSEN_ULPI_NXT |
				   MCF_GPIO_PAR_USB_VBUSOC_ULPI_STP;

		MCF_GPIO_PAR_FEC = (MCF_GPIO_PAR_FEC &
				    MCF_GPIO_PAR_FEC_FEC0_MASK) |
				    MCF_GPIO_PAR_FEC_FEC0_RMII_ULPI;
		break;
	case PORTSCX_PTS_ONCHIP:
		/* Enable VBUS_EN and VBUS_OC signals */
		MCF_GPIO_PAR_USB = MCF_GPIO_PAR_USB_VBUSEN_VBUSEN |
				   MCF_GPIO_PAR_USB_VBUSOC_VBUSOC;

		/* Setup USB_VBUS_OC signal to be active-low */
		MCF_CCM_MISCCR |= MCF_CCM_MISCCR_USBOC;

		break;
	}

	pr_debug("&regs->portsc1=0x%p  old portsc1=0x%x \n", &regs->portsc1,
		 regs->portsc1);

	regs->portsc1 &= ~PORTSCX_PTS_MASK;
	regs->portsc1 |= this->xcvr_type;

	/*
	 * need to reset the controller here so that the ID pin
	 * is correctly detected.
	 */
	regs->usbcmd |= USB_CMD_CTRL_RESET;

	/*
	 * allow controller to reset, and leave time for
	 * the ULPI transceiver to reset too.
	 */
	mdelay(10);

	pr_debug("DDD %s: done.  portsc1=0x%x\n", __FUNCTION__, regs->portsc1);
}

static void xcvr_uninit(struct platform_device *pdev)
{
	pr_debug("%s: pdev=0x%p\n", __FUNCTION__, pdev);
}


struct fsl_xcvr_ops xcvr_ops_otg = {
	.ctrlr          = USB_CTRLR_OTG,
	.init           = xcvr_init,
	.uninit         = xcvr_uninit,

#ifdef CONFIG_USB_M5445X_ULPI
	.xcvr_type      = PORTSCX_PTS_ULPI,
#elif defined CONFIG_USB_M5445X_FSLS
	.xcvr_type      = PORTSCX_PTS_ONCHIP,
#else
#error "Invalid USB transceiver selection."
#endif
};

static int __init usb_xcvr_init(void)
{
	pr_debug("%s\n", __FUNCTION__);

	fsl_usb_xcvr_register(&xcvr_ops_otg);

	pr_debug("%s done\n", __FUNCTION__);
	return 0;
}

static void __exit usb_xcvr_exit(void)
{
	fsl_usb_xcvr_unregister(USB_CTRLR_OTG);
}

module_init(usb_xcvr_init);
module_exit(usb_xcvr_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("External ULPI xcvr driver");
MODULE_LICENSE("GPL");


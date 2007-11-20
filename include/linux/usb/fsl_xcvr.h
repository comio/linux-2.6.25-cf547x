/*
 * Copyright 2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

enum fsl_usb_ctrlr {
	USB_CTRLR_H1 = 0,
	USB_CTRLR_H2 = 1,
	USB_CTRLR_OTG = 2,
};


/**
 * struct fsl_xcvr_ops - USB transceiver operations
 *
 * @xcvr_type: one of PORTSCX_PTS_{UTMI,SERIAL,ULPI}
 * @init: transceiver- and board-specific initialization function
 * @uninit: transceiver- and board-specific uninitialization function
 * @set_host:
 * @set_device:
 *
 */
struct fsl_xcvr_ops {
	enum fsl_usb_ctrlr ctrlr;	/* H1, H2, OTG */
	u32 xcvr_type;

	void (*init)(struct platform_device *pdev);
	void (*uninit)(struct platform_device *pdev);
	void (*set_host)(void);	/* DDD combine set_host and _device ? */
	void (*set_device)(void);
	void (*set_vbus_power)(struct fsl_usb2_platform_data *pdata, int on);
};



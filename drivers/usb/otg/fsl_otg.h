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

#ifndef FSL_OTG_H
#define FSL_OTG_H
#include <linux/usb/otg.h>
#include "otg_fsm.h"
#include <linux/ioctl.h>

#define ERR(format, arg...) \
printk(KERN_ERR "%s:%s: " format "\n" , __FILE__,  __FUNCTION__ , ## arg)

/*
 *  A-DEVICE timing  constants
 */

/* Wait for VBUS Rise  */
#define TA_WAIT_VRISE	(100)	/* a_wait_vrise 100 ms, section: 6.6.5.1 */

/* Wait for B-Connect */
#define TA_WAIT_BCON	(10000)	/* a_wait_bcon > 1 sec, section: 6.6.5.2
				 * This is only used to get out of
				 * OTG_STATE_A_WAIT_BCON state if there was
				 * no connection for these many milliseconds
				 */

/* A-Idle to B-Disconnect */
/* It is necessary for this timer to be more than 750 ms because of a bug in OPT
 * test 5.4 in which B OPT disconnects after 750 ms instead of 75ms as stated
 * in the test description
 */
#define TA_AIDL_BDIS	(5000)	/* a_suspend minimum 200 ms, section: 6.6.5.3 */

/* B-Idle to A-Disconnect */
#define TA_BIDL_ADIS	(12)	/* 3 to 200 ms */

/* B-device timing constants */

/* Data-Line Pulse Time*/
#define TB_DATA_PLS	(10)	/* b_srp_init,continue 5~10ms, section:5.3.3 */
#define TB_DATA_PLS_MIN	(5)	/* minimum 5 ms */
#define TB_DATA_PLS_MAX	(10)	/* maximum 10 ms */

/* SRP Initiate Time  */
#define TB_SRP_INIT	(100)	/* b_srp_init,maximum 100 ms, section:5.3.8 */

/* SRP Fail Time  */
#define TB_SRP_FAIL	(7000)	/* b_srp_init,Fail time 5~30s, section:6.8.2.2 */

/* SRP result wait time */
#define TB_SRP_WAIT	(60)

/* VBus time */
#define TB_VBUS_PLS	(30)	/* time to keep vbus pulsing asserted */

/* Discharge time */
/* This time should be less than 10ms. It varies from system to system. */
#define TB_VBUS_DSCHRG	(8)

/* A-SE0 to B-Reset  */
#define TB_ASE0_BRST	(20)	/* b_wait_acon, mini 3.125 ms,section:6.8.2.4 */

/* A bus suspend timer before we can switch to b_wait_aconn */
#define TB_A_SUSPEND	(7)
#define TB_BUS_RESUME	(12)

/* SE0 Time Before SRP */
#define TB_SE0_SRP	(2)	/* b_idle,minimum 2 ms, section:5.3.2 */

#define SET_OTG_STATE(otg_ptr, newstate)	((otg_ptr)->state = newstate)


struct fsl_otg_timer {
	unsigned long expires;	/* Number of count increase to timeout */
	unsigned long count;	/* Tick counter */
	void (*function) (unsigned long);	/* Timeout function */
	unsigned long data;	/* Data passed to function */
	struct list_head list;
};

struct fsl_otg_timer inline *otg_timer_initializer
    (void (*function) (unsigned long), unsigned long expires,
     unsigned long data) {
	struct fsl_otg_timer *timer;
	timer = kmalloc(sizeof(struct fsl_otg_timer), GFP_KERNEL);
	if (timer == NULL)
		return NULL;
	timer->function = function;
	timer->expires = expires;
	timer->data = data;
	return timer;
}

struct fsl_otg {
	struct otg_transceiver otg;
	struct otg_fsm fsm;
	struct fsl_usb_device_regs *dr_mem_map;
	struct delayed_work otg_event;

	/*used for usb host */
	u8 host_working;
	u8 on_off;

	int irq;
};

struct fsl_otg_config {
	u8 otg_port;
};

/*For SRP and HNP handle*/
#define FSL_OTG_MAJOR	66
#define FSL_OTG_NAME	"fsl-otg"
/*Command to OTG driver(ioctl)*/
#define OTG_IOCTL_MAGIC		FSL_OTG_MAJOR
/*if otg work as host,it should return 1,otherwise it return 0*/
#define GET_OTG_STATUS		_IOR(OTG_IOCTL_MAGIC, 1, int)
#define SET_A_SUSPEND_REQ	_IOW(OTG_IOCTL_MAGIC, 2, int)
#define SET_A_BUS_DROP		_IOW(OTG_IOCTL_MAGIC, 3, int)
#define SET_A_BUS_REQ		_IOW(OTG_IOCTL_MAGIC, 4, int)
#define SET_B_BUS_REQ		_IOW(OTG_IOCTL_MAGIC, 5, int)
#define GET_A_SUSPEND_REQ	_IOR(OTG_IOCTL_MAGIC, 6, int)
#define GET_A_BUS_DROP		_IOR(OTG_IOCTL_MAGIC, 7, int)
#define GET_A_BUS_REQ		_IOR(OTG_IOCTL_MAGIC, 8, int)
#define GET_B_BUS_REQ		_IOR(OTG_IOCTL_MAGIC, 9, int)


/********************************************************************/
#endif

/*****************************************************************************

  Simple GPIO driver for Freescale CooldFire 547x/548x (PSC1PSC0 port)
  (C) 2007 Industrie Dial Face S.p.A.
  
   Author Luigi 'Comio' Mantellini <luigi.mantellini@idf-hit.com>
   
*****************************************************************************/

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <asm/coldfire.h>
#include <asm/m5485gpio.h>

#include "mcf_gpio.h"

MODULE_AUTHOR("Luigi 'Comio' Mantellini ");
MODULE_DESCRIPTION("Simple GPIO driver for MCF547x/548x (PSC1PSC0 port)");
MODULE_LICENSE("GPL");

static int major = 0;		/* default to dynamic major */
module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major device number");

static u8 masks[8]={
	0x04, // PSC0
	0x08,
	0x30,
	0xC0,
	0x04, // PSC1
	0x08,
	0x30,
	0xC0
};

int mcf_gpio_PSC1PSC0_enable (struct _gpio_port_descr_t *port, void *parm, u8 pin) {
	u8 mask;
	u8 reg;
	if (!port) {
		return -EINVAL;
	}
	mask=(1<<pin) & port->pinmask;
	if (!mask) {
		return -EINVAL;
	}

	if (mask&0x0F) {
		// PSC0
		reg=MCF_GPIO_PAR_PSC0&~masks[pin];
		MCF_GPIO_PAR_PSC0=reg;
	} else {
		// PSC1
		reg=MCF_GPIO_PAR_PSC1&~masks[pin];
		MCF_GPIO_PAR_PSC1=reg;
	} 
		
	return 0;
}

int mcf_gpio_PSC1PSC0_get_status(struct _gpio_port_descr_t *port, void *parm, u8 pin, unsigned int *s) {
	u8 *masks=(u8 *)parm;
	u8 reg;
	u8 mask;

	if (!port) {
		MCF_GPIO_DEB3(printk(KERN_DEBUG MCF_GPIO_PREFIX "generic get status16 using null port.\n"));
		return -EINVAL;
	}
	mask=(1<<pin) & port->pinmask;
	if (!mask) {
		return -EINVAL;
	}

	*s=0; 
	// Check if enable
	if (mask&0x0F) {
		// PSC0
		reg=MCF_GPIO_PAR_PSC0;
	} else {
		// PSC1
		reg=MCF_GPIO_PAR_PSC1;
	} 

	if ((reg&masks[pin])==0) {
		*s|=STATUS_ENABLE;
	}

	// Check direction
	reg=*REG08(port->PDDR);
	if ((reg&mask)!=0) { 
		*s|=STATUS_OUTPUT;
	}
	
	// Check if set
	reg=*REG08(port->POD);
	if (reg&mask) {
		*s|=STATUS_SET;
	}

	return 0;
}

gpio_port_descr_t mcf_gpio_psc1psc0 = {
	.name    = "PSC1PSC0",               
	.POD     = (void*)&MCF_GPIO_PODR_PSC1PSC0,
	.PDDR    = (void*)&MCF_GPIO_PDDR_PSC1PSC0,
	.PPDSDR  = (void*)&MCF_GPIO_PPDSDR_PSC1PSC0,
	.PCLRR   = (void*)&MCF_GPIO_PCLRR_PSC1PSC0,
	.PAR     = NULL,
	.pinmask = 0xFF,
	
	.enable  = &mcf_gpio_PSC1PSC0_enable, .enable_parm=masks,
	.get_status = &mcf_gpio_PSC1PSC0_get_status, .get_status_parm=masks
};

static int __init mcf_gpio_psc1psc0_init(void)
{
	mcf_gpio_add(&mcf_gpio_psc1psc0, major);
	return 0;
}

static void __exit mcf_gpio_psc1psc0_cleanup(void)
{
	mcf_gpio_del(&mcf_gpio_psc1psc0);
}

module_init(mcf_gpio_psc1psc0_init);
module_exit(mcf_gpio_psc1psc0_cleanup);

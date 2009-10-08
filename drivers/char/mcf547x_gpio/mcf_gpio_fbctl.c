/*****************************************************************************

  Simple GPIO driver for Freescale CooldFire 547x/548x (FBCTL port)
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
MODULE_DESCRIPTION("Simple GPIO driver for MCF547x/548x (FBCTL port)");
MODULE_LICENSE("GPL");

static int major = 0;		/* default to dynamic major */
module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major device number");

static u16 masks[8]={
	0x0003,
	0x0004,
	0x0030,
	0x0040,
	0x0100,
	0x0400,
	0x1000,
	0x4000
};

gpio_port_descr_t mcf_gpio_fbctl = {
	.name    = "FBCTL",               
	.POD     = (void*)&MCF_GPIO_PODR_FBCTL,
	.PDDR    = (void*)&MCF_GPIO_PDDR_FBCTL,
	.PPDSDR  = (void*)&MCF_GPIO_PPDSDR_FBCTL,
	.PCLRR   = (void*)&MCF_GPIO_PCLRR_FBCTL,
	.PAR     = (void*)&MCF_GPIO_PAR_FBCTL,
	.pinmask = 0xFF, 
	
	.enable  = &mcf_gpio_generic_enable_16, .enable_parm=masks,
	.disable = &mcf_gpio_generic_disable_16, .disable_parm=masks,
	.get_status =&mcf_gpio_generic_get_status_16, .get_status_parm=masks
};

static int __init mcf_gpio_fbctl_init(void)
{
	mcf_gpio_add(&mcf_gpio_fbctl, major);
	return 0;
}

static void __exit mcf_gpio_fbctl_cleanup(void)
{
	mcf_gpio_del(&mcf_gpio_fbctl);
}

module_init(mcf_gpio_fbctl_init);
module_exit(mcf_gpio_fbctl_cleanup);
	 

/*****************************************************************************

  Simple GPIO driver for Freescale CooldFire 547x/548x (FEC0L port)
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
MODULE_DESCRIPTION("Simple GPIO driver for MCF547x/548x (FEC0L port)");
MODULE_LICENSE("GPL");

static int major = 0;		/* default to dynamic major */
module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major device number");

static u16 masks[8]={
		0x4000,
		0x4000,
		0x4000,
		0x4000,
		0x4000,
		0x4000,
		0x4000,
		0x4000
};

gpio_port_descr_t mcf_gpio_fec0l = {
	.name    = "FEC0L",
	.POD     = (void*)&MCF_GPIO_PODR_FEC0L,
	.PDDR    = (void*)&MCF_GPIO_PDDR_FEC0L,
	.PPDSDR  = (void*)&MCF_GPIO_PPDSDR_FEC0L,
	.PCLRR   = (void*)&MCF_GPIO_PCLRR_FEC0L,
	.PAR     = (void*)&MCF_GPIO_PAR_FECI2CIRQ,
	.pinmask = 0xFF,
	
	.enable  = &mcf_gpio_generic_enable_16, .enable_parm=masks,
	.disable = &mcf_gpio_generic_disable_16, .disable_parm=masks,
	.get_status =&mcf_gpio_generic_get_status_16, .get_status_parm=masks
};

static int __init mcf_gpio_fec0l_init(void)
{
	mcf_gpio_add(&mcf_gpio_fec0l, major);
	return 0;
}

static void __exit mcf_gpio_fec0l_cleanup(void)
{
	mcf_gpio_del(&mcf_gpio_fec0l);
}

module_init(mcf_gpio_fec0l_init);
module_exit(mcf_gpio_fec0l_cleanup);

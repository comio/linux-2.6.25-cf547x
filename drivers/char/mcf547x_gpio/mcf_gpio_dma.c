/*****************************************************************************

  Simple GPIO driver for Freescale CooldFire 547x/548x (DMA port)
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
MODULE_DESCRIPTION("Simple GPIO driver for MCF547x/548x (DMA port)");
MODULE_LICENSE("GPL");

static int major = 0;		/* default to dynamic major */
module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major device number");

static u8 masks[8]={
		0x03,
		0x0C,
		0x30,
		0xC0,
		0x00,  // Not used
		0x00,  // Not used
		0x00,  // Not used
		0x00   // Not used
};

gpio_port_descr_t mcf_gpio_dma = {
	.name    = "DMA",               
	.POD     = (void*)&MCF_GPIO_PODR_DMA,
	.PDDR    = (void*)&MCF_GPIO_PDDR_DMA,
	.PPDSDR  = (void*)&MCF_GPIO_PPDSDR_DMA,
	.PCLRR   = (void*)&MCF_GPIO_PCLRR_DMA,
	.PAR     = (void*)&MCF_GPIO_PAR_DMA,
	.pinmask = 0x0F, 
	
	.enable  = &mcf_gpio_generic_enable_8, .enable_parm=masks,
	.disable = &mcf_gpio_generic_disable_8, .disable_parm=masks,
	.get_status =&mcf_gpio_generic_get_status_8, .get_status_parm=masks
};

static int __init mcf_gpio_dma_init(void)
{
	mcf_gpio_add(&mcf_gpio_dma, major);
	return 0;
}

static void __exit mcf_gpio_dma_cleanup(void)
{
	mcf_gpio_del(&mcf_gpio_dma);
}

module_init(mcf_gpio_dma_init);
module_exit(mcf_gpio_dma_cleanup);

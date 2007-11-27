/***************************************************************************/
/*
 *	linux/arch/m68k/coldfire/spi-m5445x.c
 *
 *	Sub-architcture dependant initialization code for the Freescale
 *	5445x SPI module
 *
 *	Yaroslav Vinogradov yaroslav.vinogradov@freescale.com
 *	Copyright Freescale Semiconductor, Inc 2007
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or 
 *	(at your option) any later version.
 */
/***************************************************************************/


#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#include <asm/dma.h>
#include <asm/traps.h>
#include <asm/machdep.h>
#include <asm/coldfire.h>
#include <asm/mcfsim.h>
#include <asm/mcfqspi.h>
#include <asm/mcf5445x_gpio.h>

#define SPI_NUM_CHIPSELECTS 	0x10
#define SPI_PAR_VAL  (0 | MCF_GPIO_PAR_DSPI_PCS5_PCS5 | MCF_GPIO_PAR_DSPI_PCS2_PCS2 \
		| MCF_GPIO_PAR_DSPI_PCS1_PCS1 | MCF_GPIO_PAR_DSPI_PCS0_PCS0 | MCF_GPIO_PAR_DSPI_SIN_SIN \
		| MCF_GPIO_PAR_DSPI_SOUT_SOUT | MCF_GPIO_PAR_DSPI_SCK_SCK)

#define MCF5445x_DSPI_IRQ_SOURCE	(31)
#define MCF5445x_DSPI_IRQ_VECTOR	(64 + MCF5445x_DSPI_IRQ_SOURCE)

#define MCF5445x_DSPI_PAR	(0xFC0A4063)
#define MCF5445x_DSPI_MCR	(0xFC05C000)
#define MCF5445x_INTC0_ICR	(0xFC048040)
#define MCF5445x_INTC0_IMRL	(0xFC04800C)


#define M5445x_AUDIO_IRQ_SOURCE 49
#define M5445x_AUDIO_IRQ_VECTOR (128+M5445x_AUDIO_IRQ_SOURCE)
#define M5445x_AUDIO_IRQ_LEVEL	4

void coldfire_qspi_cs_control(u8 cs, u8 command)
{
}

#if defined(CONFIG_SPI_COLDFIRE_SSI_AUDIO)
static struct coldfire_spi_chip ssi_audio_chip_info = {
	.mode = SPI_MODE_0,
	.bits_per_word = 16,
	.del_cs_to_clk = 16,
	.del_after_trans = 16,
	.void_write_data = 0
};

#endif

static struct spi_board_info spi_board_info[] = {

#if defined(CONFIG_SPI_COLDFIRE_SSI_AUDIO)
	{
		.modalias = "ssi_audio",
		.max_speed_hz = 300000,
		.bus_num = 1,
		.chip_select = 5,
		.irq = M5445x_AUDIO_IRQ_VECTOR,
		.platform_data = NULL,
		.controller_data = &ssi_audio_chip_info
	}
#endif

};

static struct coldfire_spi_master coldfire_master_info = {
	.bus_num = 1,
	.num_chipselect = SPI_NUM_CHIPSELECTS,
	.irq_source = MCF5445x_DSPI_IRQ_SOURCE,
	.irq_vector = MCF5445x_DSPI_IRQ_VECTOR,
	.irq_mask = (0x01 << MCF5445x_DSPI_IRQ_SOURCE),
	.irq_lp = 0x2,  /* Level */
	.par_val = SPI_PAR_VAL,
//	.par_val16 = SPI_PAR_VAL,
	.cs_control = coldfire_qspi_cs_control,
};

static struct resource coldfire_spi_resources[] = {
	[0] = {
		.name = "qspi-par",
		.start = MCF5445x_DSPI_PAR,
		.end = MCF5445x_DSPI_PAR,
		.flags = IORESOURCE_MEM
	},

	[1] = {
		.name = "qspi-module",
		.start = MCF5445x_DSPI_MCR,
		.end = MCF5445x_DSPI_MCR + 0xB8,
		.flags = IORESOURCE_MEM
	},

	[2] = {
		.name = "qspi-int-level",
		.start = MCF5445x_INTC0_ICR + MCF5445x_DSPI_IRQ_SOURCE,
		.end = MCF5445x_INTC0_ICR + MCF5445x_DSPI_IRQ_SOURCE,
		.flags = IORESOURCE_MEM
	},

	[3] = {
		.name = "qspi-int-mask",
		.start = MCF5445x_INTC0_IMRL,
		.end = MCF5445x_INTC0_IMRL,
		.flags = IORESOURCE_MEM
	}
};

static struct platform_device coldfire_spi = {
	.name = "spi_coldfire", //"coldfire-qspi",
	.id = -1,
	.resource = coldfire_spi_resources,
	.num_resources = ARRAY_SIZE(coldfire_spi_resources),
	.dev = {
		.platform_data = &coldfire_master_info,
	}
};

static int __init spi_dev_init(void)
{
	int retval = 0;
	
	retval = platform_device_register(&coldfire_spi);

	if (retval < 0) {
		printk(KERN_ERR "SPI-m5445x: platform_device_register failed with code=%d\n", retval);
		goto out;
	}

	if (ARRAY_SIZE(spi_board_info))
		retval = spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));


out:
	return retval;
}

arch_initcall(spi_dev_init);

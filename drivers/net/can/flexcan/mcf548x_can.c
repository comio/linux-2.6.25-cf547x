/*
 * DESCRIPTION:
 *  CAN bus driver for the Freescale MCF548x embedded CPU.
 *
 * AUTHOR:
 *  Andrey Volkov <avolkov@varma-el.com>
 *
 * COPYRIGHT:
 *  2004-2005, Varma Electronics Oy
 *
 * LICENCE:
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * HISTORY:
 * 	2008-06-23 support for MCF548x's FlexCAN
 * 		Huan, Wang  <b18965@freescale.com>
 * 	2005-02-03 created
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/io.h>

#include "flexcan.h"
#include <asm/coldfire.h>
#include <asm/m5485sim.h>
#include <linux/can/version.h>	/* for RCSID. Removed by mkpatch script */

RCSID("$Id$");

#define PDEV_MAX 2

struct platform_device *pdev[PDEV_MAX];

static int __devinit mcf548x_can_probe(struct platform_device *pdev)
{
	struct resource *mem;
	struct net_device *dev;
	struct flexcan_platform_data *pdata = pdev->dev.platform_data;
	struct can_priv *can;
	u32 mem_size;
	int ret = -ENODEV;

	if (!pdata)
		return ret;

	dev = alloc_flexcandev();
	if (!dev)
		return -ENOMEM;
	can = netdev_priv(dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	dev->irq = platform_get_irq(pdev, 0);
	if (!mem || !dev->irq)
		goto req_error;

	mem_size = mem->end - mem->start + 1;
	if (!request_mem_region(mem->start, mem_size, pdev->dev.driver->name)) {
		dev_err(&pdev->dev, "resource unavailable\n");
		goto req_error;
	}
	SET_NETDEV_DEV(dev, &pdev->dev);

	dev->base_addr = (unsigned long)ioremap_nocache(mem->start, mem_size);
	if (!dev->base_addr) {
		dev_err(&pdev->dev, "failed to map can port\n");
		ret = -ENOMEM;
		goto fail_map;
	}
	can->can_sys_clock = pdata->clock_frq;
	platform_set_drvdata(pdev, dev);
	ret = register_flexcandev(dev, pdata->clock_src);
	if (ret >= 0) {
		dev_info(&pdev->dev, "probe for port 0x%lX done\n",
			 dev->base_addr);
		return ret;
	}

	iounmap((unsigned long *)dev->base_addr);
fail_map:
	release_mem_region(mem->start, mem_size);
req_error:
	free_candev(dev);
	dev_err(&pdev->dev, "probe failed\n");
	return ret;
}

static int __devexit mcf548x_can_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct resource *mem;

	platform_set_drvdata(pdev, NULL);
	unregister_flexcandev(dev);
	iounmap((unsigned long *)dev->base_addr);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, mem->end - mem->start + 1);
	free_candev(dev);
	return 0;
}

static struct platform_driver mcf548x_can_driver = {
	.driver = {
		   .name = "mcf548x-flexcan",
		   },
	.probe = mcf548x_can_probe,
	.remove = __devexit_p(mcf548x_can_remove),
};

static struct resource mcf548x_can0_resources[] = {
	[0] = {
		.start 		= MCF_MBAR + 0x0000A000,
		.end		= MCF_MBAR + 0x0000A7FF,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= 49,
		.end		= 49,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct resource mcf548x_can1_resources[] = {
	[0] = {
		.start 		= MCF_MBAR + 0x0000A800,
		.end		= MCF_MBAR + 0x0000AFFF,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= 55,
		.end		= 55,
		.flags		= IORESOURCE_IRQ,
	},
};


static int __init mcf548x_of_to_pdev(void)
{
	unsigned int i;
	int err = -ENODEV;
	struct flexcan_platform_data pdata;

	pdev[0] = platform_device_register_simple("mcf548x-flexcan", 0,
			mcf548x_can0_resources, 2);
	if (IS_ERR(pdev[0])) {
		err = PTR_ERR(pdev[0]);
		return err;
	}
	pdev[1] = platform_device_register_simple("mcf548x-flexcan", 1,
			mcf548x_can1_resources, 2);
	if (IS_ERR(pdev[1])) {
		err = PTR_ERR(pdev[1]);
		return err;
	}

	/* FlexCAN clock */
	pdata.clock_frq = 100000000;

	for (i = 0; i < PDEV_MAX; i++) {
		err = platform_device_add_data(pdev[i], &pdata, sizeof(pdata));
		if (err)
			return err;
	}
	return err;
}

int __init mcf548x_can_init(void)
{
	int err = mcf548x_of_to_pdev();

	if (err) {
		printk(KERN_ERR "%s init failed with err=%d\n",
			mcf548x_can_driver.driver.name, err);
		return err;
	}

	return platform_driver_register(&mcf548x_can_driver);
}

void __exit mcf548x_can_exit(void)
{
	int i;
	platform_driver_unregister(&mcf548x_can_driver);
	for (i = 0; i < PDEV_MAX; i++)
		platform_device_unregister(pdev[i]);
}

module_init(mcf548x_can_init);
module_exit(mcf548x_can_exit);

MODULE_AUTHOR("Andrey Volkov <avolkov@varma-el.com>");
MODULE_DESCRIPTION("Freescale MCF548x CAN driver");
MODULE_LICENSE("GPL v2");

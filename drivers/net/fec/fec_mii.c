/*****************************************************************************

MII frontend for Freescale CooldFire 547x/548x FEC Driver
(C) 2010 Industrie Dial Face S.p.A.

Author Luigi 'Comio' Mantellini <luigi.mantellini@idf-hit.com>

*****************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/phy.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <asm/mcfsim.h>

#include "fec.h"

int fec_mdio_write(struct mii_bus *bus, int mii_id, int regnum, u16 value)
{
	struct net_device *netdev;
	
	if (!bus||!bus->priv)
		return -EINVAL;

	MCF_GPIO_PAR_FECI2CIRQ |= 0xFFC0;

	netdev = (struct net_device *)bus->priv;

	fec_write_mii(netdev->base_addr, mii_id, regnum, value);

	return 0;
}

int fec_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	int  value = 0xffff;
	struct net_device *netdev;

	if (!bus||!bus->priv)
		return -EINVAL;

	MCF_GPIO_PAR_FECI2CIRQ |= 0xFFC0;

	netdev = (struct net_device *)bus->priv;
	
	fec_read_mii(netdev->base_addr, mii_id, regnum, &value);
	
	return value;
}

int fec_mdio_reset(struct mii_bus *bus)
{
	struct net_device *netdev;

	if (!bus||!bus->priv)
		return -EINVAL;

	netdev = (struct net_device *)bus->priv;

	fec_reset_mii(netdev->base_addr);

	return 0;
}

struct mii_bus *fec_create_miibus(struct net_device *netdev)
{
	struct mii_bus *new_bus;
	static int busid = 0;
	int phyid;

	if (!netdev) {
		return NULL;
	}

	new_bus = kzalloc(sizeof(struct mii_bus), GFP_KERNEL);

	if (!new_bus)
		return NULL;

	new_bus->read = &fec_mdio_read,
	new_bus->write = &fec_mdio_write,
	new_bus->reset = &fec_mdio_reset,
	new_bus->id = busid++;
	new_bus->name = kzalloc(sizeof(char)*32, GFP_KERNEL);
	sprintf(new_bus->name, "fec_mdio#%d", new_bus->id);
	printk("bus %s\n", new_bus->name);
	new_bus->priv = netdev;
	new_bus->phy_mask = 0xfffffffe;
	new_bus->irq = kzalloc(sizeof(new_bus->irq[0])*PHY_MAX_ADDR, GFP_KERNEL);

	for (phyid = 0; phyid < PHY_MAX_ADDR; phyid++) {
		new_bus->irq[phyid] = PHY_POLL;
		new_bus->phy_map[phyid] = NULL;
	}

	new_bus->dev = &netdev->dev;

	return new_bus;
}

int fec_mdio_setup(struct net_device *dev)
{
	int err;
	struct fec_priv *priv;
	struct phy_device *phy;
	char phy_id[BUS_ID_SIZE];

	if (!dev)
		return -EINVAL;
	priv = (struct fec_priv *)dev->priv;

	if (!dev)
		return -EINVAL;

	printk(KERN_INFO "Enabling mdio bus @ %p\n", (void *)priv->netdev->base_addr);
	
	priv->mdio = fec_create_miibus(dev);

	printk("1\n");

	if (priv->mdio) {
		err = mdiobus_register(priv->mdio);
	} else {
		err = -ENOMEM;
		goto nomem;
	}

	if (err) {
		printk (KERN_ERR "%s: Cannot register as MDIO bus\n",
				priv->mdio->name);
		goto bus_register_fail;
	}

	phy = get_phy_device(priv->mdio, 0);

	snprintf(phy_id, BUS_ID_SIZE, PHY_ID_FMT, priv->mdio->id, 0);

	priv->phy = NULL;

	if (phy) {
		phy = phy_connect(dev, phy_id, &fec_adjust_link, 0, PHY_INTERFACE_MODE_MII);

		if (phy) {
			phy->supported &= (SUPPORTED_10baseT_Half
			| SUPPORTED_10baseT_Full
			| SUPPORTED_100baseT_Half
			| SUPPORTED_100baseT_Full
			| SUPPORTED_Autoneg
			| SUPPORTED_Pause | SUPPORTED_Asym_Pause
			| SUPPORTED_MII
			| SUPPORTED_TP);
			phy->advertising = phy->supported;
		}

		phy_start(phy);

		priv->phy = phy;
	}

	return 0;

bus_register_fail:
	kfree(priv->mdio);
	priv->mdio = NULL;
nomem:
	return err;
}

int fec_mdio_remove(struct net_device *dev)
{
	struct fec_priv *priv;

	if (!dev)
		return -EINVAL;

	priv = (struct fec_priv *)dev->priv;
	if (!priv||!priv->mdio)
		return -EINVAL;

	mdiobus_unregister(priv->mdio);
	if (priv->mdio&&priv->mdio->name)
		kfree(priv->mdio->name);
	kfree(priv->mdio);
	priv->mdio = NULL;

	return 0;
}

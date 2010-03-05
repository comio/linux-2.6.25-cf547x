/*****************************************************************************

Ethtool frontend for Freescale CooldFire 547x/548x FEC Driver
(C) 2010 Industrie Dial Face S.p.A.

Author Luigi 'Comio' Mantellini <luigi.mantellini@idf-hit.com>

*****************************************************************************/

#include <linux/kernel.h>
#include <linux/ethtool.h>
#include <linux/netdevice.h>

#include "fec.h"

static void	fec_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strcpy (info->driver, FEC_DRV_NAME);
	strcpy (info->version, FEC_DRV_VERSION);
	strcpy (info->bus_info, "DMA");
}

const static struct ethtool_ops fec_ethtool_ops = {
	.get_drvinfo = fec_get_drvinfo,
	.get_link = ethtool_op_get_link,
};

const struct ethtool_ops *fec_set_ethtool(struct net_device *dev)
{
	if (dev) {
		dev->ethtool_ops = &fec_ethtool_ops;
		return dev->ethtool_ops;
	}
	return NULL;
}


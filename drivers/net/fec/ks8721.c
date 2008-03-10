#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/bitops.h>

#if 0
#include <linux/config.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <asm/coldfire.h>
#endif
#include <asm/coldfire.h>
#include <asm/mcfsim.h>

#include   "fec.h"
#include   "ks8721.h"

#ifdef   CONFIG_FEC_548x_AUTO_NEGOTIATION
#define   KS8721_AUTO_NEGOTIATION_ENABLE
#endif

/************************************************************************
* +NAME: ks8721_init_transceiver
*
* DESCRIPTION: This function initializes the transceiver
*
* RETURNS: If no error occurs, this function returns zero.
*          Otherwise, it returns 1
*************************************************************************/

int ks8721_init_transceiver(unsigned long base_addr, int *fduplex)
{

	int data;
	unsigned long time;
	int flag = 1;

	int result;

	// Set the frequency of MII
	FEC_MSCR(base_addr) = FEC_MII_SPEED;

	// Reset
	if ((result = fec_write_mii(base_addr, FEC_PHY_ADDR, KS8721_CTRL, KS8721_CTRL_RESET)))
		return result;

	// Read back    
	if ((result = fec_read_mii(base_addr, FEC_PHY_ADDR, KS8721_CTRL, &data)) != 0)
		return result;

	// If reset bit is set, return
	if (data & KS8721_CTRL_RESET)
		return -ETIME;

#ifdef KS8721_AUTO_NEGOTIATION_ENABLE

	// Disable  the auto-negotiation 
	if ((result = fec_write_mii(base_addr, FEC_PHY_ADDR, KS8721_CTRL, 0)) != 0)
		return result;

	// Set the auto-negotiation advertisement register 
	if ((result = fec_write_mii(base_addr, FEC_PHY_ADDR, KS8721_ANADV, KS8721_ANADV_ADV_ALL)) != 0)
		return result;

	// Enable the auto-negotiation
	if ((result = fec_write_mii(base_addr, FEC_PHY_ADDR, KS8721_CTRL, KS8721_CTRL_AN_ENABLE)) != 0)
		return result;

	// Read PHY status register
	if ((result = fec_read_mii(base_addr, FEC_PHY_ADDR, KS8721_STAT, &data)) != 0)
		return result;
	// Set the current time
	time = jiffies;

	// Wait for the auto-negotiation completion
	while (!(data & KS8721_STAT_ANCOMPLETE))
	{

		if (jiffies - time > KS8721_TIMEOUT * HZ)
		{
			flag = 0;
			break;
		}

		schedule();

		// Read PHY status register
		if ((result = fec_read_mii(base_addr, FEC_PHY_ADDR, KS8721_STAT, &data)) != 0)
			return result;
	}

	if (flag)
	{
		// Set the duplex flag     
		if (data & KS8721_STAT_FDUPLEX)
			*fduplex = 1;
		else
			*fduplex = 0;

		return 0;
	}

#endif

	// Set the default mode (Full duplex, 100 Mbps) 
	if ((result = fec_write_mii(base_addr, FEC_PHY_ADDR, KS8721_CTRL, KS8721_CTRL_DEFAULT_MODE)) != 0)
		return result;
	*fduplex = KS8721_CTRL_DEFAULT_MODE & 0x100;

	return 0;

}

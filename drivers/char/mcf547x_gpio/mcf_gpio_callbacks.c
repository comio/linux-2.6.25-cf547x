/*****************************************************************************

  Simple GPIO driver for Freescale CooldFire 547x/548x (generic callbacks)
  (C) 2007 Industrie Dial Face S.p.A.
  
   Author Luigi 'Comio' Mantellini <luigi.mantellini@idf-hit.com>
   
*****************************************************************************/

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <asm/coldfire.h>
#include <asm/m5485gpio.h>

#include "mcf_gpio.h"

int mcf_gpio_generic_init(struct _gpio_port_descr_t *port, void *parm) {
	if (!port) {
		MCF_GPIO_DEB3(printk(KERN_DEBUG MCF_GPIO_PREFIX "generic init using null port.\n"));
		return -EINVAL;
	}
	MCF_GPIO_DEBPROTO(printk(KERN_DEBUG MCF_GPIO_PREFIX "%s generic init.\n", port->name));

	return 0;
}

int mcf_gpio_generic_done(struct _gpio_port_descr_t *port, void *parm) {
	if (!port) { 
		MCF_GPIO_DEB3(printk(KERN_DEBUG MCF_GPIO_PREFIX "generic done using null port.\n"));
		return -EINVAL;
	}
	MCF_GPIO_DEBPROTO(printk(KERN_DEBUG MCF_GPIO_PREFIX "%s generic done.\n", port->name));

	return 0;
}

int mcf_gpio_generic_set(struct _gpio_port_descr_t *port, void *parm, u8 pin, unsigned int value) {
	u8 mask;
	u8 reg;
	if (!port) {
		MCF_GPIO_DEB3(printk(KERN_DEBUG MCF_GPIO_PREFIX "generic set using null port.\n"));
		return -EINVAL;
	}
	mask =(1<<pin) & (port->pinmask);
	if (!mask) {
		return -EINVAL;
	}
	reg=*REG08(port->POD);
	reg=(reg&~mask);
	if (value) {
		reg|=mask;
	}
	*REG08(port->POD)=reg;
	return 0;
}

int mcf_gpio_generic_get(struct _gpio_port_descr_t *port, void *parm, u8 pin, unsigned int *value) {
	u8 mask;
	u8 reg;
	if (!port) {
		MCF_GPIO_DEB3(printk(KERN_DEBUG MCF_GPIO_PREFIX "generic get using null port.\n"));
		return -EINVAL;
	}
	mask=(1<<pin) & (port->pinmask);
	if (!mask) {
		return -EINVAL;
	}
	reg=*REG08(port->PPDSDR);
	*value = (reg & mask)?1:0;
	return 0;
}

int mcf_gpio_generic_dir(struct _gpio_port_descr_t *port, void *parm, u8 pin, direction_t d) {
	u8 mask;
	u8 reg;
	if (!port) {
		MCF_GPIO_DEB3(printk(KERN_DEBUG MCF_GPIO_PREFIX "generic dir using null port.\n"));
		return -EINVAL;
	}
	mask=(1<<pin) & (port->pinmask);
	if (!mask) {
		return -EINVAL;
	}
	reg=*REG08(port->PDDR);
	reg&=~mask;
	if (d==INPUT) { 
                reg|=0;
	} else if (d==OUTPUT) {
		reg|=mask;
	} else {
		return -EINVAL;
	}
	*REG08(port->PDDR)=reg;
	return 0;	
}

int mcf_gpio_generic_enable(struct _gpio_port_descr_t *port, void *parm, u8 pin) {
	return -EINVAL; // always fail
}

int mcf_gpio_generic_enable_8  (struct _gpio_port_descr_t *port, void *parm, u8 pin) {
	u8 *masks=(u8 *)parm;
	u8 reg;
	u8 mask;
	if (!port) {
		MCF_GPIO_DEB3(printk(KERN_DEBUG MCF_GPIO_PREFIX "generic enable8 using null port.\n"));
		return -EINVAL;
	}
	mask=(1<<pin) & port->pinmask;
	if (!mask) {
		return -EINVAL;
	}

	reg=*REG08(port->PAR);
	reg&=~masks[pin];	
	*REG08(port->PAR)=reg;
	return 0;
}

int mcf_gpio_generic_enable_16 (struct _gpio_port_descr_t *port, void *parm, u8 pin) {
	u16 *masks=(u16 *)parm;
	u16 reg;
	u8 mask;
	if (!port) {
		MCF_GPIO_DEB3(printk(KERN_DEBUG MCF_GPIO_PREFIX "generic enable16 using null port.\n"));	
		return -EINVAL;
	}
	mask=(1<<pin) & port->pinmask;
	if (!mask) {
		return -EINVAL;
	}

	reg=*REG16(port->PAR);
	reg&=~masks[pin];	
	*REG16(port->PAR)=reg;
	return 0;
}

int mcf_gpio_generic_disable_8  (struct _gpio_port_descr_t *port, void *parm, u8 pin) {
	u8 *masks=(u8 *)parm;
	u8 reg;
	u8 mask;
	if (!port) {
		MCF_GPIO_DEB3(printk(KERN_DEBUG MCF_GPIO_PREFIX "generic disable8 using null port.\n"));
		return -EINVAL;
	}
	mask=(1<<pin) & port->pinmask;
	if (!mask) {
		return -EINVAL;
	}

	reg=*REG08(port->PAR);
	reg|=masks[pin];	
	*REG08(port->PAR)=reg;
	return 0;
}

int mcf_gpio_generic_disable_16 (struct _gpio_port_descr_t *port, void *parm, u8 pin) {
	u16 *masks=(u16 *)parm;
	u16 reg;
	u8 mask;
	if (!port) {
		MCF_GPIO_DEB3(printk(KERN_DEBUG MCF_GPIO_PREFIX "generic disable16 using null port.\n"));	
		return -EINVAL;
	}
	mask=(1<<pin) & port->pinmask;
	if (!mask) {
		return -EINVAL;
	}

	reg=*REG16(port->PAR);
	reg|=masks[pin];	
	*REG16(port->PAR)=reg;
	return 0;
}


int mcf_gpio_generic_disable(struct _gpio_port_descr_t *port, void *parm, u8 pin) {
	return -EINVAL; // always fail
}

int mcf_gpio_generic_set_status(struct _gpio_port_descr_t *port, void *parm, u8 pin, unsigned int s, unsigned int mask) {
	u8 mask_pin;

	if (!port) {
		MCF_GPIO_DEB3(printk(KERN_DEBUG MCF_GPIO_PREFIX "generic set status using null port.\n"));
		return -EINVAL;
	}
	mask_pin=(1<<pin) & port->pinmask;
	if (!mask_pin) {
		return -EINVAL;
	}

	if (BIT_IS_1(mask, STATUS_ENABLE)) {
		if (BIT_IS_1(s, STATUS_ENABLE)) {
			// Enable GPIO port
			MCF_CALLBACK(port, enable)(port, port->enable_parm, pin);
		} else {
			// Disable GPIO port
			MCF_CALLBACK(port, disable)(port, port->disable_parm, pin);
		}
	}

	if (BIT_IS_1(mask, STATUS_OUTPUT)) {
		if (BIT_IS_1(s, STATUS_OUTPUT)) {
			// Set as output
			MCF_CALLBACK(port, dir)(port, port->dir_parm, pin, OUTPUT);
		} else {
			// Set as input
			MCF_CALLBACK(port, dir)(port, port->dir_parm, pin, INPUT);
		}
	}

	if (BIT_IS_1(mask, STATUS_SET)) {
		if (BIT_IS_1(s, STATUS_SET)) {
			// Set bit
			MCF_CALLBACK(port, set)(port, port->set_parm, pin, 1);
		} else {
			// Clear bit
			MCF_CALLBACK(port, set)(port, port->set_parm, pin, 0);
		}
	}
	
	return 0;
}

int mcf_gpio_generic_get_status(struct _gpio_port_descr_t *port, void *parm, u8 pin, unsigned int *s) {
	return -EINVAL; // always fail
}

int mcf_gpio_generic_get_status_8(struct _gpio_port_descr_t *port, void *parm, u8 pin, unsigned int *s) {
	u8 *masks=(u8 *)parm;
	u8 reg;
	u8 mask;

	*s=0;
	
	if (!port) {
		MCF_GPIO_DEB3(printk(KERN_DEBUG MCF_GPIO_PREFIX "generic get status8 using null port.\n"));
		return -EINVAL;
	}
	mask=(1<<pin) & port->pinmask;
	if (!mask) {
		return -EINVAL;
	}

	// Check if enable
	reg=*REG08(port->PAR);
	if ((reg&masks[pin])==0) {
		*s|=STATUS_ENABLE;
	}

	// Check direction
	reg=*REG08(port->PDDR);
	if ((reg&mask)==0) { 
		*s|=STATUS_OUTPUT;
	}
	
	// Check if set
	reg=*REG08(port->POD);
	if (reg&mask) {
		*s|=STATUS_SET;
	}

	return 0;
}

int mcf_gpio_generic_get_status_16(struct _gpio_port_descr_t *port, void *parm, u8 pin, unsigned int *s) {
	u16 *masks=(u16 *)parm;
	u8 reg;
	u16 reg16;
	u8 mask;

	*s=0;
	
	if (!port) {
		MCF_GPIO_DEB3(printk(KERN_DEBUG MCF_GPIO_PREFIX "generic get status16 using null port.\n"));
		return -EINVAL;
	}
	mask=(1<<pin) & port->pinmask;
	if (!mask) {
		return -EINVAL;
	}

 
	// Check if enable
	reg16=*REG16(port->PAR);
	if ((reg16 & masks[pin])==0) {
		*s|=STATUS_ENABLE;
	}

	// Check direction
	reg=*REG08(port->PDDR);
	if ((reg & mask)==0) { 
		*s|=STATUS_OUTPUT;
	}
	
	// Check if set
	reg=*REG08(port->POD);
	if (reg&mask) {
		*s|=STATUS_SET;
	}

	return 0;
}

EXPORT_SYMBOL(mcf_gpio_generic_init);
EXPORT_SYMBOL(mcf_gpio_generic_done);
EXPORT_SYMBOL(mcf_gpio_generic_set);
EXPORT_SYMBOL(mcf_gpio_generic_get);
EXPORT_SYMBOL(mcf_gpio_generic_dir);
EXPORT_SYMBOL(mcf_gpio_generic_enable);
EXPORT_SYMBOL(mcf_gpio_generic_enable_8);
EXPORT_SYMBOL(mcf_gpio_generic_enable_16);
EXPORT_SYMBOL(mcf_gpio_generic_disable);
EXPORT_SYMBOL(mcf_gpio_generic_disable_8);
EXPORT_SYMBOL(mcf_gpio_generic_disable_16);
EXPORT_SYMBOL(mcf_gpio_generic_set_status);
EXPORT_SYMBOL(mcf_gpio_generic_get_status);
EXPORT_SYMBOL(mcf_gpio_generic_get_status_8);
EXPORT_SYMBOL(mcf_gpio_generic_get_status_16);

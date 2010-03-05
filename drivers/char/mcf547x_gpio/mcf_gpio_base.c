/*****************************************************************************

  Simple GPIO driver for Freescale CooldFire 547x/548x (Low level)
  (C) 2007 Industrie Dial Face S.p.A.
  
  Author Luigi 'Comio' Mantellini <luigi.mantellini@idf-hit.com>
   
*****************************************************************************/

/**
 * \file   mcf_gpio_base.c
 * \author Luigi 'Comio' Mantellini
 * \date   2008-01-14
 * \brief  Low-level code to provide a flexyble and extensible gpio
 *         infrastructure.
 */

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <asm/coldfire.h>
#include <asm/m5485gpio.h>

#include "mcf_gpio.h"
	
MODULE_AUTHOR("Luigi 'Comio' Mantellini ");
MODULE_DESCRIPTION("Simple GPIO driver for MCF547x/548x");
MODULE_LICENSE("GPL");

int mcf_dlevel = 3;
module_param(mcf_dlevel, int, 0);
MODULE_PARM_DESC(mcf_dlevel, "Debug level (0=none, ..., 9=proto");
EXPORT_SYMBOL(mcf_dlevel);

#define MCF_KERN_DEBUG KERN_DEBUG

/**
 * \brief gpio_port_root keep the gpios-list
 */
struct _gpio_port_t *gpio_port_root=NULL;

/**
 * \brief Provide a standard write method to meet the Linux infrastrucure.
          mcf_gpio_write() calls the opportune registred callback to perform the
          operation.
 * \param file output file descriptor.
 * \param data user data buffer.
 * \param len buffer lenght.
 * \param ppos (ignored)
 * \return Number of write bytes.
 */
static ssize_t mcf_gpio_write(struct file *file, const char __user *data,
		       size_t len, loff_t *ppos) {
		       
        /* Pin # = Minor # */
	unsigned m = iminor(file->f_dentry->d_inode);
	gpio_port_t *port = (gpio_port_t *)file->private_data;
	unsigned int value;
	
	size_t i;
	int err = 0;

	MCF_GPIO_DEB3(
		if (!port) {
			printk(KERN_ERR MCF_GPIO_PREFIX "write on null port.\n");
			return -EFAULT;
		}
	);

        /* Parse char-by-char the entire user buffer */
	for (i = 0; i < len; ++i) {
		char c;
		if (get_user(c, data + i)) {
			return -EFAULT;
		}
			
		MCF_GPIO_DEBPROTO(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "Write %d from %s pin #%d.\n", c, port->port->name, m));

		switch (c) {
		case '0':
		        /* Write "0" on m-th pin */
		        MCF_CALLBACK(port->port,set)(port->port, port->port->set_parm, m, 0);
			break;
		case '1':
		        /* Write "1" on m-th pin */
		        MCF_CALLBACK(port->port,set)(port->port, port->port->set_parm, m, 1);
			break;
		case 'O':
		case 'o':
		        /* Set the m-th pin as Output pin */
			MCF_CALLBACK(port->port,dir)(port->port, port->port->dir_parm, m, OUTPUT);
			break;
		case 'I':
		case 'i':
		        /* Set the m-th pin as Input pin */
		        MCF_CALLBACK(port->port,dir)(port->port, port->port->dir_parm, m, INPUT);
        		break;
        	case 'E':
		case 'e':
       		        /* Enable the m-th pin */
			MCF_CALLBACK(port->port,enable)(port->port, port->port->enable_parm, m);
			break;
		case 'D':
		case 'd':
      		        /* Disable the m-th pin */
			MCF_CALLBACK(port->port,disable)(port->port, port->port->disable_parm, m);
			break;
		case 'X':
		case 'x':
		        /* Negate the m-th pin */
			MCF_CALLBACK(port->port,get)(port->port, port->port->get_parm, m, &value);
		        MCF_CALLBACK(port->port,set)(port->port, port->port->set_parm, m, value?0:1);
			break;
		case '\n':
			/* end of settings string, do nothing */
			break;
		default:
			printk(KERN_WARNING MCF_GPIO_PREFIX "'%c' unknow command.\n", c);
			err++;
		}
	}
	if (err)
		return -EINVAL;	/* full string handled, report error */

	return len;
}

/**
 * \brief Provide a standard read method to meet the Linux infrastrucure
 * \param file input file descriptor.
 * \param data user data buffer.
 * \param len buffer lenght.
 * \param ppos (ignored)
 * \return Number of read bytes.
 *
 * mcf_gpio_write() calls the opportune get method registred by a specific
 * gpio module.
 */
static ssize_t mcf_gpio_read(struct file *file, char __user * buf,
		      size_t len, loff_t * ppos) {
        /* Pin # = Minor # */
        unsigned int m = iminor(file->f_dentry->d_inode);
	unsigned int value;
	gpio_port_t *port = (gpio_port_t *)file->private_data;

	MCF_GPIO_DEB3(
		if (!port) {
			printk(KERN_ERR MCF_GPIO_PREFIX "read on null port.\n");
			return -EFAULT;
		}
	);

        /* Get the char from GPIO pin */
        MCF_CALLBACK(port->port,get)(port->port, port->port->get_parm, m, &value);

	MCF_GPIO_DEBPROTO(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "Read %d from %s pin #%d.\n", value, port->port->name, m));

	if (put_user(value ? '1' : '0', buf))
		return -EFAULT;
	return 1;
}

/**
 * \brief Provide a standard open method to meet the Linux infrastrucure
 * \param inode opened i-node
 * \param file file descriptor
 * \return Error code (O if OK, <0 if KO)
 */
static int mcf_gpio_open(struct inode *inode, struct file *file) {
        /* Indentify the minor and major numbers */
        /*    major # = GPIO block */
        /*    minor # = pin # into GPIO block*/
	unsigned minor = iminor(inode);
	unsigned major = imajor(inode);
	
	gpio_port_t *port=gpio_port_root;

        /* Searching for the port descriotor associated to major number */
	while (port) {
		if (port->major==major) {
			// Found!
			break;
		}
		port=port->next;
	}
	
	if (port) {
	        /* We identified a valid port descriptor */
		MCF_GPIO_DEBPROTO(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "open on %s (major %d) pin #%d.\n", port->port->name, major, minor));
                /* We pass the port descriptor by means of the opaque private_data pointer */
		file->private_data = port;
		
		if (port->port->pinmask & (1<<minor)) {
		        /* The GPIO block has the $minor pin */
			return nonseekable_open(inode, file);
		} else {
		        /* Argh! The GPIO block hasn't the $minor pin */
		        MCF_GPIO_DEB3(printk(KERN_ERR MCF_GPIO_PREFIX "port %s (major %d) hasn't the pin #%d.\n", port->port->name, major, minor));
			return -EINVAL;
		}
	} else {
	        /* Argh! No valid port descriptor identified :S */
		printk(KERN_ERR MCF_GPIO_PREFIX "unable to find the device (major=%d)\n", major);
		return -EINVAL;
	}
}

/**
 * \brief Provide a standard release method to meet the Linux infrastrucure.
          This method doesn't perform anything and returns always OK (=0).
 * \param inode opened i-node
 * \param file opened file descriptor
 * \return Error code (Always ok...)
 */
static int mcf_gpio_release(struct inode *inode, struct file *file) {
	return 0;		
}

/**
 * \brief File operation descriptor.
 */
static struct file_operations mcf_gpio_fileops = {
        .owner   = THIS_MODULE,
        .write   = mcf_gpio_write,
        .read    = mcf_gpio_read,
        .open    = mcf_gpio_open,
        .release = mcf_gpio_release
};

/**
 * \brief Initialize the GPIO blocks list (implemented as a simple-linked list).
 */
static void gpio_list_init(void) {
	gpio_port_root=NULL;
}

/**
 * \brief Destroy the GPIO blocks list (implemented as a simple-linked list).
 */
static void gpio_list_done(void) {
	while (gpio_port_root) {
		struct _gpio_port_t *tmp=gpio_port_root->next;
		MCF_CALLBACK(gpio_port_root->port,done)(gpio_port_root->port, gpio_port_root->port->done_parm);
		cdev_del(&gpio_port_root->cdev);
		unregister_chrdev_region(MKDEV(gpio_port_root->major, 0), MAXPINS(gpio_port_root->port->pinmask));		
		kfree(gpio_port_root);
		gpio_port_root=tmp;
	}
	gpio_port_root=NULL;
}

int mcf_gpio_add(gpio_port_descr_t *port, int major) {
	int 	    rc;
	dev_t       devid;
	gpio_port_t *newport;

	if (!port) {
		MCF_GPIO_DEB3(printk(KERN_ERR MCF_GPIO_PREFIX "add using a null port.\n"));
		return -EINVAL;
	}
	
	MCF_GPIO_DEB3(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX " adding %s\n", port->name));
	MCF_GPIO_DEB3(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "   .POD    = %8p\n", port->POD));
	MCF_GPIO_DEB3(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "   .PDDR   = %8p\n", port->PDDR));
	MCF_GPIO_DEB3(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "   .PPDSDR = %8p\n", port->PPDSDR));
	MCF_GPIO_DEB3(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "   .PCLRR  = %8p\n", port->PCLRR));
	MCF_GPIO_DEB3(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "   .PAR    = %8p\n", port->PAR));

    	newport=kmalloc(sizeof(*newport), GFP_KERNEL);
	if (!newport) {
		MCF_GPIO_DEB3(printk(KERN_ERR MCF_GPIO_PREFIX "unable to allocate memory.\n"));
		rc = -ENOMEM;
		goto quit;
	}

	newport->pdev = platform_device_alloc(port->name, -1);
	if (!newport->pdev) {
		MCF_GPIO_DEB3(printk(KERN_ERR MCF_GPIO_PREFIX "unable to allocate memory.\n"));
		rc = -ENOMEM;
		goto undo_malloc;
	}
	
	printk(KERN_NOTICE MCF_GPIO_PREFIX "port %s initializing\n", port->name);
	
	if (major) {
        	devid = MKDEV(major, 0);
		rc = register_chrdev_region(devid,  MAXPINS(port->pinmask), port->name);
	} else {
        	rc = alloc_chrdev_region(&devid, 0, MAXPINS(port->pinmask), port->name);
        	major = MAJOR(devid);
	}
	MCF_GPIO_DEB2(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "%s has major %d.\n", port->name, major));

	if (rc < 0) {
      		printk(KERN_ERR MCF_GPIO_PREFIX "error alloc_chrdev_region %d\n.", rc);
        	goto undo_malloc3;
    	}

	cdev_init(&newport->cdev, &mcf_gpio_fileops);
    	rc = cdev_add (&newport->cdev, devid, MAXPINS(port->pinmask));
    	if (rc < 0) {
    		printk(KERN_ERR MCF_GPIO_PREFIX "unable to add device %s.\n", port->name);
    		goto undo_malloc2;
    	}

	// Fill the struct...
	newport->owner = THIS_MODULE;	
	newport->port  = port;
	newport->major = major;

	// Stupid Double Linked List...
	newport->next=gpio_port_root;
	newport->prev=NULL;	
	if (gpio_port_root) {
		gpio_port_root->prev=newport;
	}
	gpio_port_root=newport;
	
MCF_GPIO_DEBPROTO(
	printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "%s %s init method.\n", port->name, port->init?"has":"hasn't");
	printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "%s %s done method.\n", port->name, port->done?"has":"hasn't");
	printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "%s %s set method.\n", port->name, port->set?"has":"hasn't");
	printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "%s %s get method.\n", port->name, port->get?"has":"hasn't");
	printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "%s %s dir method.\n", port->name, port->dir?"has":"hasn't");
	printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "%s %s enable method.\n", port->name, port->enable?"has":"hasn't");
	printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "%s %s disable method.\n", port->name, port->disable?"has":"hasn't");
	printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "%s %s get_status method.\n", port->name, port->get_status?"has":"hasn't");	
	printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "%s %s set_status method.\n", port->name, port->set_status?"has":"hasn't");	
)
	MCF_CALLBACK(newport->port,init)(newport->port, newport->port->init_parm);

#ifdef CONFIG_DEVFS_FS
	for (i=0; i<MAXPINS(newport->port->pinmask); i++) {
	    if ( (1<<i) & newport->port->pinmask) {
		devfs_mk_cdev(MKDEV(newport->major, i), S_IFCHR|S_IRUSR|S_IWUSR, "%s%d", newport->port->name, i);
	    }
	}
#endif

#ifdef CONFIG_MCF_GPIO_PROCFS
	mcf_gpio_proc_register_port(newport);
#endif

	printk(KERN_NOTICE MCF_GPIO_PREFIX "port %s added using major %d.\n", newport->port->name, newport->major);
	return 0; /* succeed */
	
undo_malloc3:
	kfree(newport->pdev);
undo_malloc2:
undo_malloc:
        // platform_device_del(pdev);
        kfree(newport);
quit:
	return rc;
}

int mcf_gpio_del(gpio_port_descr_t *port) {
	gpio_port_t *t=gpio_port_root;

	if (!port) {
		MCF_GPIO_DEB3(printk(KERN_ERR MCF_GPIO_PREFIX "del using a null port.\n"));
		return -EINVAL;
	}

	MCF_GPIO_DEB3(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX " deleting %s\n", port->name));
	MCF_GPIO_DEB3(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "   .POD    = %8p\n", port->POD));
	MCF_GPIO_DEB3(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "   .PDDR   = %8p\n", port->PDDR));
	MCF_GPIO_DEB3(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "   .PPDSDR = %8p\n", port->PPDSDR));
	MCF_GPIO_DEB3(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "   .PCLRR  = %8p\n", port->PCLRR));
	MCF_GPIO_DEB3(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "   .PAR    = %8p\n", port->PAR));

	while (t && t->port!=port) {
		t = t->next; // Next...
	}

	if (t) {
		gpio_port_t *prev=t->prev;
		gpio_port_t *next=t->next;
		
		MCF_GPIO_DEB3(printk(MCF_KERN_DEBUG MCF_GPIO_PREFIX "port %s is present.\n", port->name));
		MCF_CALLBACK(t->port, done)(t->port, t->port->done_parm);

#ifdef CONFIG_DEVFS_FS
		for (i=0; i<MAXPINS(t->port->pinmask); i++) {
		    if ( (1<<i) & t->port->pinmask) {
			devfs_remove("%s%d", t->port->name, i);
		    }
		}
#endif

#ifdef CONFIG_MCF_GPIO_PROCFS
		mcf_gpio_proc_unregister_port(t);
#endif

		cdev_del(&t->cdev);
		unregister_chrdev_region(MKDEV(t->major, 0), MAXPINS(t->port->pinmask));
		platform_device_unregister(t->pdev);
		
		kfree(t->pdev);
		
		if (prev) {
			prev->next=next;
		}
		if (next) {
			next->prev=prev;
		}
		kfree(t);
		printk(KERN_NOTICE MCF_GPIO_PREFIX "port %s removed.\n", port->name);
		return 0; // Ok
	} else {
		printk(KERN_ERR MCF_GPIO_PREFIX "port %s is not present.\n", port->name);
		return -EINVAL;
	}
}

static int __init mcf_gpio_init(void) {
	gpio_list_init();
	printk(KERN_INFO MCF_GPIO_NAME ": " MCF_GPIO_NAME " v" MCF_GPIO_VER " loaded.\n");
#ifdef CONFIG_MCF_GPIO_PROCFS
	mcf_gpio_proc_register();
#endif
	return 0;
}

static void __exit mcf_gpio_cleanup(void)
{
#ifdef CONFIG_MCF_GPIO_PROCFS
	mcf_gpio_proc_unregister();
#endif
	gpio_list_done();
	printk(KERN_INFO MCF_GPIO_NAME ": unloaded.\n");
}

module_init(mcf_gpio_init);
module_exit(mcf_gpio_cleanup);

EXPORT_SYMBOL(mcf_gpio_add);
EXPORT_SYMBOL(mcf_gpio_del);

/*****************************************************************************

  Simple GPIO driver for Freescale CooldFire 547x/548x (ProcFS interface)
  (C) 2007 Industrie Dial Face S.p.A.
  
  Author Luigi 'Comio' Mantellini <luigi.mantellini@idf-hit.com>

  Synopsis:
     This module provides useful functions to support the procfs access to
     gpio interfaces.

     mcf_gpio_proc_register        - Registers main procfs files
     mcf_gpio_proc_unregister      - Unregisters main procfs files
     mcf_gpio_proc_register_port   - Register a port dedicated procfs file
     mcf_gpio_proc_unregister_port - Unregister a port dedicated procfs file

  Changelog:
     2007-12-17 First version - readonly access
   
*****************************************************************************/
#include <linux/kernel.h>

#ifdef CONFIG_MCF_GPIO_PROCFS

#include <linux/proc_fs.h>

#include "mcf_gpio.h"

#define PROC_PRINT_VARS                                 \
    off_t pos = 0;                                      \
    off_t begin = 0;                                    \
    int len = 0;
    
#define PROC_PRINT(fmt,args...)                         \
    len += sprintf(page + len , fmt, ##args);           \
    pos += len;                                         \
    if(pos < off) {                                     \
        len = 0;                                        \
        begin = pos;                                    \
    }                                                   \
    if(pos > off + count)                               \
        goto done;


#define PROC_PRINT_DONE                                 \
        *eof = 1;                                       \
    done:                                               \
        *start = page + (off - begin);                  \
        len -= (off - begin);                           \
        if(len > count)                                 \
            len = count;                                \
        if(len < 0)                                     \
            len = 0;                                    \
        return len 

static struct proc_dir_entry *mcf_gpio_proc_dir     = NULL;
static struct proc_dir_entry *mcf_gpio_proc_summary = NULL;

static int mcf_gpio_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data) {
	int i;
	struct _gpio_port_t *t;
	unsigned int value;
	int ret;
	
	PROC_PRINT_VARS;

	PROC_PRINT(MCF_GPIO_NAME ": \n\n");
	
	if (!gpio_port_root) {
		PROC_PRINT("No gpio devices found!\n");
		goto gout;
	}
	
	t=gpio_port_root;
	while (t) {
		PROC_PRINT("%s (major %d)\n", t->port->name, t->major);

		for (i=WORDSIZE(t->port->pinmask)-1; i>=0; i--) {
			PROC_PRINT("%d", i);
		}
	        PROC_PRINT("\n");

		// Print Status Flag: - = Disable, I = Input, O = Output
		for (i=WORDSIZE(t->port->pinmask)-1; i>=0; i--) {
			ret=MCF_CALLBACK(t->port, get_status)(t->port, t->port->get_status_parm, i, &value);
			if ((ret==0) && BIT_IS_1(value, STATUS_ENABLE)) {
				if (BIT_IS_1(value, STATUS_OUTPUT)) {
					PROC_PRINT("o");
				} else {
					PROC_PRINT("i");
				}
			} else {
				PROC_PRINT("-");
			}
		}
		PROC_PRINT("\n");

		for (i=WORDSIZE(t->port->pinmask)-1; i>=0; i--) {
			ret=MCF_CALLBACK(t->port, get_status)(t->port, t->port->get_status_parm, i, &value);
			if ((ret==0) && BIT_IS_1(value, STATUS_ENABLE)) {
				MCF_CALLBACK(t->port, get)(t->port, t->port->get_parm, i, &value);
				PROC_PRINT("%d", value);
			} else {
				PROC_PRINT("-");
			}
		}
		PROC_PRINT("\n\n");
		t=t->next;
	}

gout:	
	PROC_PRINT_DONE;
}

static int mcf_gpio_proc_read_port(char *page, char **start, off_t off, int count, int *eof, void *data) {
	int i;
	unsigned int value;
	struct _gpio_port_t *port=(struct _gpio_port_t *)data;
	int ret;
	
	PROC_PRINT_VARS;

	if (!data) {
		PROC_PRINT("gpio driver error...\n");
		goto gout;
	}

	PROC_PRINT("%s: \n\n", port->port->name);
	
	PROC_PRINT("%s (major %d)\n\n", port->port->name, port->major);
	for (i=WORDSIZE(port->port->pinmask)-1; i>=0; i--) {
		PROC_PRINT("%d", i);
	}
        PROC_PRINT("\n");

	// Print Status Flag: - = Disable, I = Input, O = Output
	for (i=WORDSIZE(port->port->pinmask)-1; i>=0; i--) {
		ret=MCF_CALLBACK(port->port, get_status)(port->port, port->port->get_status_parm, i, &value);
		if ((ret==0) && BIT_IS_1(value, STATUS_ENABLE)) {
			if (BIT_IS_1(value, STATUS_OUTPUT)) {
				PROC_PRINT("o");
			} else {
				PROC_PRINT("i");
			}
		} else {
			PROC_PRINT("-");
		}
	}
        PROC_PRINT("\n");

	for (i=WORDSIZE(port->port->pinmask)-1; i>=0; i--) {
		ret=MCF_CALLBACK(port->port, get_status)(port->port, port->port->get_status_parm, i, &value);
		if ((ret==0) && BIT_IS_1(value, STATUS_ENABLE)) {
			MCF_CALLBACK(port->port, get)(port->port, port->port->get_parm, i, &value);
			PROC_PRINT("%d", value);
                } else {
			PROC_PRINT("-");
		}
	}
	PROC_PRINT("\n\n");

gout:	
	PROC_PRINT_DONE;
}

int mcf_gpio_proc_register(void) {
	int rv = 0; // OK...

	mcf_gpio_proc_dir = proc_mkdir(MCF_GPIO_PROCDIR, NULL);
	if(!mcf_gpio_proc_dir) {
		rv = -ENOMEM;
		goto out;
	}

	// Summary file
        mcf_gpio_proc_summary = create_proc_read_entry( MCF_GPIO_PROCENTRY,
							S_IFREG|S_IRUGO|S_IWUSR,
							mcf_gpio_proc_dir,
							mcf_gpio_proc_read,
							NULL);
        if (!mcf_gpio_proc_summary) {
                rv = -ENOMEM;
                goto removedir;
        }

        return 0;
removedir:
	remove_proc_entry(MCF_GPIO_PROCDIR, NULL);
out:
	return rv;
}

void mcf_gpio_proc_unregister(void) {
	remove_proc_entry(MCF_GPIO_PROCENTRY, mcf_gpio_proc_dir);
        remove_proc_entry(MCF_GPIO_PROCDIR,   NULL);
}

int mcf_gpio_proc_register_port(struct _gpio_port_t *port) {
	if (!port || !port->port) {
		return -EINVAL;
	}
	
	port->proc_entry= create_proc_read_entry(
				port->port->name,
				S_IFREG|S_IRUGO|S_IWUSR,
				mcf_gpio_proc_dir,
				mcf_gpio_proc_read_port,
				port);
	if (!port->proc_entry) {
		printk(KERN_ERR MCF_GPIO_PREFIX "unable to register procfs for port %s.\n", port->port->name);
		return -ENOMEM;	
	} else {
		printk(KERN_NOTICE MCF_GPIO_PREFIX "procfs for port %s registered.\n", port->port->name);
		return 0;
	}
}

void mcf_gpio_proc_unregister_port(struct _gpio_port_t *port) {
	if (port && port->proc_entry) {
		remove_proc_entry(port->port->name, mcf_gpio_proc_dir);
		printk(KERN_NOTICE MCF_GPIO_PREFIX "procfs for port %s unregistered.\n", port->port->name);
	}
}

#endif

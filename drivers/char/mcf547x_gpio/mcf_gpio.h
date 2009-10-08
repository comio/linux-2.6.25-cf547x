/*****************************************************************************

  Simple GPIO driver for Freescale CooldFire 547x/548x
  (C) 2007 Industrie Dial Face S.p.A.
  
  Author Luigi 'Comio' Mantellini <luigi.mantellini@idf-hit.com>
   
*****************************************************************************/

#ifndef __MCF_GPIO_H__
#define __MCF_GPIO_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cdev.h>

#ifdef CONFIG_MCF_GPIO_PROCFS
#include <linux/proc_fs.h>
#endif

#define MCF_GPIO_NAME "mcf-gpio"
#define MCF_GPIO_VER  "0.0.2"
#define MCF_GPIO_PREFIX MCF_GPIO_NAME ": "

/**
 * \brief Module Debug Level.
 *
 * Module Debug Level:
 *  0 - No debug
 * ...
 *  9 - Very verbose
 */
extern int mcf_dlevel;

#ifdef CONFIG_MCF_GPIO_DEBUG
#define MCF_GPIO_DEB1(x)     if (mcf_dlevel>=1)   { x; }
#define MCF_GPIO_DEB2(x)     if (mcf_dlevel>=2)   { x; }
#define MCF_GPIO_DEB3(x)     if (mcf_dlevel>=3)   { x; }
#define MCF_GPIO_DEB4(x)     if (mcf_dlevel>=4)   { x; }
#define MCF_GPIO_DEB5(x)     if (mcf_dlevel>=5)   { x; }
#define MCF_GPIO_DEB6(x)     if (mcf_dlevel>=6)   { x; }
#define MCF_GPIO_DEB7(x)     if (mcf_dlevel>=7)   { x; }
#define MCF_GPIO_DEB8(x)     if (mcf_dlevel>=8)   { x; }
#define MCF_GPIO_DEBPROTO(x) if (mcf_dlevel>=9)   { x; }
#else
#define MCF_GPIO_DEB1(x)
#define MCF_GPIO_DEB2(x)
#define MCF_GPIO_DEB3(x)
#define MCF_GPIO_DEB4(x)
#define MCF_GPIO_DEB5(x)
#define MCF_GPIO_DEB6(x)
#define MCF_GPIO_DEB7(x)
#define MCF_GPIO_DEB8(x)
#define MCF_GPIO_DEBPROTO(x)
#endif

/**
 * \brief Returns the Most Significant Bit Number.
 */
#define MAXPINS(mask)    \
	(mask&0x8000?16: \
	(mask&0x4000?15: \
	(mask&0x2000?14: \
	(mask&0x1000?13: \
	(mask&0x0800?12: \
	(mask&0x0400?11: \
	(mask&0x0200?10: \
	(mask&0x0100? 9: \
	(mask&0x0080? 8: \
	(mask&0x0040? 7: \
	(mask&0x0020? 6: \
	(mask&0x0010? 5: \
	(mask&0x0008? 4: \
	(mask&0x0004? 3: \
	(mask&0x0002? 2: \
	(mask&0x0001? 1: \
 	              0))))))))))))))))
/**
 * \brief Returns the size (In bit) of the smallest word that can contain the value.
 */
#define WORDSIZE(mask)    \
        (mask&0xFF00?16: \
        (mask&0x00FF? 8: \
                      0))

/**
 * \brief Callback hack.
 *
 * Call p->x() if it exists, otherwise call a generic function (appending the
 * "mcf_gpio_generic_" prefix to the name. 
 */
#define MCF_CALLBACK(p, x) \
	(p->x?p->x:mcf_gpio_generic_##x)
	
#define STATUS_ENABLE 0x0001
#define STATUS_OUTPUT 0x0002
#define STATUS_SET    0x0004

#define BIT_IS_1(v,m) ((v&m)==m)
#define BIT_IS_0(v,m) ((v&m)==0)

/**
 * \brief GPIO direction enum
 *
 */
typedef enum _direction_t {
	INPUT  = 0,
	OUTPUT = 1
} direction_t;

struct _gpio_port_descr_t;

/**
 * \brief Callbacks hook types definition.
 *
 * These types are used to define hooks into the gpio
 * descriptor structure and they are called as needed.
 * Each type indicates which paramters are need to perform
 * the specific call. All functions must accept as first 
 * parameter the port descriptor and as second parameter
 * a opaque pointer to an optional function specific parameter.
 * The following paramters are defined respect to the function
 * requirements.
 *
 */
/*@{*/
/**
 * \brief gpio_init_t is called to initialize a gpio block.
 * \param port GPIO port descritor
 * \param parm Optional paramter
 * \return 0 if Ok, !0 otherwise
 */
typedef int (*gpio_init_t)       (struct _gpio_port_descr_t* port, void *parm);
/**
 * \brief gpio_doen_t is called to release a gpio block.
 * \param port GPIO port descritor
 * \param parm Optional paramter
 * \return 0 if Ok, !0 otherwise
 */
typedef int (*gpio_done_t)       (struct _gpio_port_descr_t* port, void *parm);
/**
 * \brief gpio_set_t is called to write a bit on a gpio block.
 * \param port GPIO port descritor
 * \param parm Optional paramter
 * \return 0 if Ok, !0 otherwise
 */
typedef int (*gpio_set_t)        (struct _gpio_port_descr_t* port, void *parm, u8 pin, unsigned int value);
/**
 * \brief gpio_get_t is called to read a bit on a gpio block.
 * \param port GPIO port descritor
 * \param parm Optional paramter
 * \param pin Pin number
 * \param value value to set
 * \return 0 if Ok, !0 otherwise
 */
typedef int (*gpio_get_t)        (struct _gpio_port_descr_t* port, void *parm, u8 pin, unsigned int *value);
/**
 * \brief gpio_dir_t is called to change the bit direcion on a gpio block.
 * \param port GPIO port descritor
 * \param parm Optional paramter
 * \param pin Pin number
 * \param value returning value
 * \return 0 if Ok, !0 otherwise
 */
typedef int (*gpio_dir_t)        (struct _gpio_port_descr_t* port, void *parm, u8 pin, direction_t d);
/**
 * \brief gpio_enable_t is called to enabe a gpio block.
 * \param port GPIO port descritor
 * \param parm Optional paramter
 * \param pin Pin number
 * \param d Direction (INPUT or OUTPUT)
 * \return 0 if Ok, !0 otherwise
 */
typedef int (*gpio_enable_t)     (struct _gpio_port_descr_t* port, void *parm, u8 pin);
/**
 * \brief gpio_disable_t is called to disable a gpio block.
 * \param port GPIO port descritor
 * \param parm Optional paramter
 * \param pin Pin number
 * \return 0 if Ok, !0 otherwise
 */
typedef int (*gpio_disable_t)    (struct _gpio_port_descr_t* port, void *parm, u8 pin);
/**
 * \brief gpio_set_status_t is called to change the status of a pin on a gpio block.
 * \param port GPIO port descritor
 * \param parm Optional paramter
 * \param pin Pin number
 * \param s status
 * \param mask status mask
 * \return 0 if Ok, !0 otherwise
 */
typedef int (*gpio_set_status_t) (struct _gpio_port_descr_t* port, void *parm, u8 pin, unsigned int s, unsigned int mask);
/**
 * \brief gpio_get_status_t is called to get the status of a pin on a gpio block.
 * \param port GPIO port descritor
 * \param parm Optional paramter
 * \param pin Pin number
 * \param s Actual pin status
 * \return 0 if Ok, !0 otherwise
 */
typedef int (*gpio_get_status_t) (struct _gpio_port_descr_t* port, void *parm, u8 pin, unsigned int *s);
/*@}*/ 

/**
 * \brief GPIO Port descriptor structure
 *
 */
typedef struct _gpio_port_descr_t {
	char name[16];   /**< Symbolic Name */
	void *POD;       /**< Port Output Data Register */
	void *PDDR;      /**< Port Data Direction Register */ 
	void *PPDSDR;    /**< Port Pin Data/Set Data Register */
	void *PCLRR;     /**< Port Clear Output Register */
	void *PAR;       /**< Pin Assginment Register (may be casted) */
	
	u16   pinmask;   /**< Usable pins */
	
	// Callbacks and params pointer
	gpio_init_t    init;    void *init_parm;
	gpio_done_t    done;    void *done_parm;
	gpio_set_t     set;     void *set_parm;
	gpio_get_t     get;     void *get_parm;
	gpio_dir_t     dir;     void *dir_parm;
	gpio_enable_t  enable;  void *enable_parm;
	gpio_disable_t disable; void *disable_parm;
	gpio_set_status_t  set_status;  void *set_status_parm;
	gpio_get_status_t  get_status;  void *get_status_parm;

	// Private data... the driver doesn't touch them
	void *data;
} gpio_port_descr_t;

// GPIO list structure
typedef struct _gpio_port_t {
	struct module		*owner; // Owner
	struct cdev		cdev;   // Associated cdev device
	struct platform_device	*pdev;  // Associated pdev device
	gpio_port_descr_t	*port;  // GPIO Port descriptor
	int			major;  // Associated major number
	struct _gpio_port_t 	*next;
	struct _gpio_port_t 	*prev;

#ifdef CONFIG_MCF_GPIO_PROCFS
	// Procfs entry (touched by driver)
	struct proc_dir_entry 	*proc_entry;
#endif

} gpio_port_t;

extern struct _gpio_port_t *gpio_port_root;

extern int mcf_gpio_add(gpio_port_descr_t *port, int major);
extern int mcf_gpio_del(gpio_port_descr_t *port);

/**
 * \brief Generic callbacks (see mcf_gpio_callbacks.c file).
 *
 * These methods implement dummy or standard (and safe) operations
 * on ColdFire GPIO blocks.
 * The _8 and _16 suffixes indicate if the operation is performed
 * on 8bit or 16bit register.
 * To understand these functions you need to consult the ColdFire
 * User Manual and their stub implementation (file
 * mcf_gpio_callbacks.c).
 */
extern int mcf_gpio_generic_init          (struct _gpio_port_descr_t *port, void *parm);
extern int mcf_gpio_generic_done          (struct _gpio_port_descr_t *port, void *parm);
extern int mcf_gpio_generic_set           (struct _gpio_port_descr_t *port, void *parm, u8 pin, unsigned int value);
extern int mcf_gpio_generic_get           (struct _gpio_port_descr_t *port, void *parm, u8 pin, unsigned int *value);
extern int mcf_gpio_generic_dir           (struct _gpio_port_descr_t *port, void *parm, u8 pin, direction_t d);
extern int mcf_gpio_generic_enable        (struct _gpio_port_descr_t *port, void *parm, u8 pin);
extern int mcf_gpio_generic_enable_8      (struct _gpio_port_descr_t *port, void *parm, u8 pin);
extern int mcf_gpio_generic_enable_16     (struct _gpio_port_descr_t *port, void *parm, u8 pin);
extern int mcf_gpio_generic_disable       (struct _gpio_port_descr_t *port, void *parm, u8 pin);
extern int mcf_gpio_generic_disable_8     (struct _gpio_port_descr_t *port, void *parm, u8 pin);
extern int mcf_gpio_generic_disable_16    (struct _gpio_port_descr_t *port, void *parm, u8 pin);
extern int mcf_gpio_generic_set_status    (struct _gpio_port_descr_t *port, void *parm, u8 pin, unsigned int s, unsigned int mask);
extern int mcf_gpio_generic_get_status    (struct _gpio_port_descr_t *port, void *parm, u8 pin, unsigned int *s);
extern int mcf_gpio_generic_get_status_8  (struct _gpio_port_descr_t *port, void *parm, u8 pin, unsigned int *s);
extern int mcf_gpio_generic_get_status_16 (struct _gpio_port_descr_t *port, void *parm, u8 pin, unsigned int *s);

#ifdef CONFIG_MCF_GPIO_PROCFS

#include <linux/proc_fs.h>

#define MCF_GPIO_PROCDIR MCF_GPIO_NAME
#define MCF_GPIO_PROCENTRY "gpios"

extern int  mcf_gpio_proc_register(void);
extern void mcf_gpio_proc_unregister(void);
extern int  mcf_gpio_proc_register_port(struct _gpio_port_t *port);
extern void mcf_gpio_proc_unregister_port(struct _gpio_port_t *port);
#endif

#endif


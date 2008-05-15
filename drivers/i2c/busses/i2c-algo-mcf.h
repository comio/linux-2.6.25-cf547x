#ifndef I2C_ALGO_MCF_H
#define I2C_ALGO_MCF_H 1

/* --- Defines for pcf-adapters ---------------------------------------	*/
#include <linux/i2c.h>

struct i2c_algo_mcf_data {
	void *data;		/* private data for lolevel routines	*/
	void (*setmcf) (void *data, int ctl, int val);
	int (*getmcf) (void *data, int ctl);
	int (*getown) (void *data);
	int (*getclock) (void *data);
	void (*waitforpin) (void);
	/* local settings */
	int udelay;
	int mdelay;
	int timeout;
};

int i2c_mcf_add_bus(struct i2c_adapter *);
int i2c_mcf_del_bus(struct i2c_adapter *);

#endif /* I2C_ALGO_MCF_H */

#ifndef _COLDFIRE_H_
#define _COLDFIRE_H_

#if defined(CONFIG_M5445X)
#define MCF_MBAR	0x0
/*
 * Even though RAMBAR1 should be in the 0x8xxxxxxx range there
 * is a problem that needs to be resolved.  Currently head.S
 * disables SRAM/RAMBAR1.
 */
#define MCF_RAMBAR1 0x40000000
#define MCF_SRAM	0x40000000
#define MCF_VBASE	(MCF_RAMBAR1)
#elif defined(CONFIG_M547X_8X)
#define MCF_MBAR	(0xE0000000)
#define MCF_MMUBAR	(MCF_MBAR + 0x01000000)
#define MCF_RAMBAR0	(MCF_MBAR + 0x03000000)
#define MCF_RAMBAR1	(MCF_MBAR + 0x03001000)
#define MCF_VBASE	(MCF_RAMBAR0)
#endif

#define MCF_CLK     	CONFIG_MCFCLK
#define MCF_BUSCLK	(CONFIG_MCFCLK/2)

#ifdef __ASSEMBLY__
#define REG32
#define REG16
#define REG08
#else  /* __ASSEMBLY__ */
#define REG32(x) ((volatile unsigned long  *)(x))
#define REG16(x) ((volatile unsigned short *)(x))
#define REG08(x) ((volatile unsigned char  *)(x))

#define MCF_REG32(x) *(volatile unsigned long  *)(MCF_MBAR+(x))
#define MCF_REG16(x) *(volatile unsigned short *)(MCF_MBAR+(x))
#define MCF_REG08(x) *(volatile unsigned char  *)(MCF_MBAR+(x))

void cacr_set(unsigned long);
unsigned long cacr_get(void);

#if defined(CONFIG_M5445X)
#define coldfire_enable_irq0(irq)	MCF_INTC0_CIMR = (irq);

#define coldfire_enable_irq1(irq)	MCF_INTC1_CIMR = (irq);

#define coldfire_disable_irq0(irq)	MCF_INTC0_SIMR = (irq);

#define coldfire_disable_irq1(irq)	MCF_INTC1_SIMR = (irq);
#elif defined(CONFIG_M547X_8X)
#endif
#define getiprh()			MCF_INTC0_IPRH

#endif /* __ASSEMBLY__ */

#endif  /* _COLDFIRE_H_  */

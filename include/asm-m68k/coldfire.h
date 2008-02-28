#ifndef _COLDFIRE_H_
#define _COLDFIRE_H_

#if defined(CONFIG_M54455)
#define MCF_MBAR	0x0
#define MCF_RAMBAR1 	0x40000000
#define MCF_SRAM	0x80000000
#elif defined(CONFIG_M547X_8X)
#define MCF_MBAR	0xE0000000
#define MCF_RAMBAR0	0xE3000000
#define MCF_RAMBAR1	0xE3001000
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

#define coldfire_enable_irq0(irq)	MCF_INTC0_CIMR = (irq);

#define coldfire_enable_irq1(irq)	MCF_INTC1_CIMR = (irq);

#define coldfire_disable_irq0(irq)	MCF_INTC0_SIMR = (irq);

#define coldfire_disable_irq1(irq)	MCF_INTC1_SIMR = (irq);

#define getiprh()			MCF_INTC0_IPRH

#endif /* __ASSEMBLY__ */

#endif  /* _COLDFIRE_H_  */

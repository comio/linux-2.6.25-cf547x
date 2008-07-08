/*
 *	m5485sim.h -- ColdFire 547x/548x System Integration Unit support.
 */
 
#ifndef	m5485sim_h
#define	m5485sim_h


/*
 *      System Integration Unit Registers
 */
#define MCF_SDRAMDS	MCF_REG32(0x000004)	/* SDRAM Drive Strength 	*/
#define MCF_SBCR       	MCF_REG32(0x000010)	/* System Breakpoint Control 	*/
#define MCF_CSnCFG(x)	MCF_REG32(0x000020+(x*4))/* SDRAM Chip Select X		*/
#define MCF_SECSACR     MCF_REG32(0x000038)	/* Sequential Access Control 	*/	
#define MCF_RSR         MCF_REG32(0x000044)	/* Reset Status 		*/
#define MCF_JTAGID      MCF_REG32(0x000050)	/* JTAG Device Identification 	*/

/*
 *      FlexBus Chip Selects Registers
 */
#define MCF_CSARn(x)    MCF_REG32(0x000500+(x*0xC))
#define MCF_CSMRn(x)    MCF_REG32(0x000504+(x*0xC))
#define MCF_CSCRn(x)    MCF_REG32(0x000508+(x*0xC))

/*
 *      Interrupt Controller Registers
 */
#define MCF_IPRH      	MCF_REG32(0x000700)
#define MCF_IPRL      	MCF_REG32(0x000704)
#define MCF_IMRH      	MCF_REG32(0x000708)
#define MCF_IMRL      	MCF_REG32(0x00070C)
#define MCF_INTFRCH   	MCF_REG32(0x000710)
#define MCF_INTFRCL   	MCF_REG32(0x000714)
#define MCF_IRLR      	MCF_REG08(0x000718)
#define MCF_IACKLPR   	MCF_REG08(0x000719)
#define MCF_SWIACK    	MCF_REG08(0x0007E0)
#define MCF_LnIACK(x) 	MCF_REG08(0x0007E4+((x)*0x004))
#define MCF_ICR(x)    	MCF_REG08(0x000740+((x)*0x001))

/*
 *	Slice Timers Registers
 */
#define MCF_SLTCNT(x)   MCF_REG32(0x000900+((x)*0x010))
#define MCF_SCR(x)      MCF_REG32(0x000904+((x)*0x010))
#define MCF_SCNT(x)     MCF_REG32(0x000908+((x)*0x010))
#define MCF_SSR(x)      MCF_REG32(0x00090C+((x)*0x010))

/*
 *	Interrupt sources
 */
#define ISC_EPORT_Fn(x)		(x) 		/* EPORT Interrupts 	*/
#define ISC_USB_EPn(x)		(15+(x))	/* USB Endopint 	*/
#define ISC_USB_ISR		(22) 		/* USB General source 	*/
#define ISC_USB_AISR		(22) 		/* USB core source 	*/
#define ISC_DSPI_OVRFW		(25)		/* DSPI overflow 	*/
#define ISC_DSPI_RFOF		(26)		
#define ISC_DSPI_RFDF		(27)		
#define ISC_DSPI_TFUF		(28)		
#define ISC_DSPI_TCF		(29)		
#define ISC_DSPI_TFFF		(30)		
#define ISC_DSPI_EOQF		(31)		
#define ISC_PSCn(x)		(35-(x))		
#define ISC_COMM_TIM		(36)		
#define ISC_SEC			(37)		
#define ISC_FEC1		(38)		
#define ISC_FEC0		(39)		
#define ISC_I2C			(40)		
#define ISC_PCI_ARB		(41)		
#define ISC_PCI_CB		(42)		
#define ISC_PCI_XLB		(43)		
#define ISC_DMA			(48)		
#define ISC_CANn_ERR(x)		(49+(6*(x)))		
#define ISC_CANn_BUSOFF(x)	(50+(6*(x)))		
#define ISC_CANn_MBOR(x)	(51+(6*(x)))		
#define ISC_CAN0_WAKEIN		(52)		
#define ISC_SLTn(x)		(54-(x))		
#define ISC_GPTn(x)		(62-(x))		

/*
 *	Interrupt level and priorities
 */
#define ILP_TOP			(MCF_ICR_IL(5) | MCF_ICR_IP(3))
#define ILP_SLT0		(MCF_ICR_IL(5) | MCF_ICR_IP(2))
#define ILP_SLT1		(MCF_ICR_IL(5) | MCF_ICR_IP(1))
#define ILP_DMA			(MCF_ICR_IL(5) | MCF_ICR_IP(0))
#define ILP_SEC			(MCF_ICR_IL(4) | MCF_ICR_IP(7))
#define ILP_FEC0		(MCF_ICR_IL(4) | MCF_ICR_IP(6))
#define ILP_FEC1		(MCF_ICR_IL(4) | MCF_ICR_IP(5))
#define ILP_PCI_XLB		(MCF_ICR_IL(4) | MCF_ICR_IP(4))
#define ILP_PCI_ARB		(MCF_ICR_IL(4) | MCF_ICR_IP(3))
#define ILP_PCI_CB		(MCF_ICR_IL(4) | MCF_ICR_IP(2))
#define ILP_I2C			(MCF_ICR_IL(4) | MCF_ICR_IP(1))

#define ILP_USB_EPn(x)		(MCF_ICR_IL(3) | MCF_ICR_IP(7-(x)))
#define ILP_USB_EP0		(MCF_ICR_IL(3) | MCF_ICR_IP(7))
#define ILP_USB_EP1		(MCF_ICR_IL(3) | MCF_ICR_IP(6))
#define ILP_USB_EP2		(MCF_ICR_IL(3) | MCF_ICR_IP(5))
#define ILP_USB_EP3		(MCF_ICR_IL(3) | MCF_ICR_IP(4))
#define ILP_USB_EP4		(MCF_ICR_IL(3) | MCF_ICR_IP(3))
#define ILP_USB_EP5		(MCF_ICR_IL(3) | MCF_ICR_IP(2))
#define ILP_USB_EP6		(MCF_ICR_IL(3) | MCF_ICR_IP(1))
#define ILP_USB_ISR		(MCF_ICR_IL(3) | MCF_ICR_IP(0))

#define ILP_USB_AISR		(MCF_ICR_IL(2) | MCF_ICR_IP(7))
#define ILP_DSPI_OVRFW		(MCF_ICR_IL(2) | MCF_ICR_IP(6))
#define ILP_DSPI_RFOF		(MCF_ICR_IL(2) | MCF_ICR_IP(5))
#define ILP_DSPI_RFDF		(MCF_ICR_IL(2) | MCF_ICR_IP(4))
#define ILP_DSPI_TFUF		(MCF_ICR_IL(2) | MCF_ICR_IP(3))
#define ILP_DSPI_TCF		(MCF_ICR_IL(2) | MCF_ICR_IP(2))
#define ILP_DSPI_TFFF		(MCF_ICR_IL(2) | MCF_ICR_IP(1))
#define ILP_DSPI_EOQF		(MCF_ICR_IL(2) | MCF_ICR_IP(0))

#define ILP_COMM_TIM		(MCF_ICR_IL(1) | MCF_ICR_IP(7))
#define ILP_PSCn(x)		(MCF_ICR_IL(1) | MCF_ICR_IP(3-((x)&3)))
#define ILP_PSC0		(MCF_ICR_IL(1) | MCF_ICR_IP(3))
#define ILP_PSC1		(MCF_ICR_IL(1) | MCF_ICR_IP(2))
#define ILP_PSC2		(MCF_ICR_IL(1) | MCF_ICR_IP(1))
#define ILP_PSC3		(MCF_ICR_IL(1) | MCF_ICR_IP(0))





/********************************************************************/

/*
 *      System Integration Unit Bitfields
 */
 
/* SBCR */
#define MCF_SBCR_PIN2DSPI       (0x08000000)
#define MCF_SBCR_DMA2CPU        (0x10000000)
#define MCF_SBCR_CPU2DMA        (0x20000000)
#define MCF_SBCR_PIN2DMA        (0x40000000)
#define MCF_SBCR_PIN2CPU        (0x80000000)

/* SECSACR */
#define MCF_SECSACR_SEQEN       (0x00000001)

/* RSR */
#define MCF_RSR_RST             (0x00000001)
#define MCF_RSR_RSTWD           (0x00000002)
#define MCF_RSR_RSTJTG          (0x00000008)

/* JTAGID */
#define MCF_JTAGID_REV          (0xF0000000)
#define MCF_JTAGID_PROCESSOR    (0x0FFFFFFF)
#define MCF_JTAGID_MCF5485      (0x0800C01D)
#define MCF_JTAGID_MCF5484      (0x0800D01D)
#define MCF_JTAGID_MCF5483      (0x0800E01D)
#define MCF_JTAGID_MCF5482      (0x0800F01D)
#define MCF_JTAGID_MCF5481      (0x0801001D)
#define MCF_JTAGID_MCF5480      (0x0801101D)
#define MCF_JTAGID_MCF5475      (0x0801201D)
#define MCF_JTAGID_MCF5474      (0x0801301D)
#define MCF_JTAGID_MCF5473      (0x0801401D)
#define MCF_JTAGID_MCF5472      (0x0801501D)
#define MCF_JTAGID_MCF5471      (0x0801601D)
#define MCF_JTAGID_MCF5470      (0x0801701D)


/*
 *      Interrupt Controller Bitfields
 */
#define MCF_IRLR_IRQ(x)         (((x)&0x7F)<<1)
#define MCF_IACKLPR_PRI(x)      (((x)&0x0F)<<0)
#define MCF_IACKLPR_LEVEL(x)    (((x)&0x07)<<4)
#define MCF_ICR_IP(x)           (((x)&0x07)<<0)
#define MCF_ICR_IL(x)           (((x)&0x07)<<3)

/*
 *      Slice Timers Bitfields
 */
#define MCF_SCR_TEN    		(0x01000000)
#define MCF_SCR_IEN    		(0x02000000)
#define MCF_SCR_RUN    		(0x04000000)
#define MCF_SSR_ST     		(0x01000000)
#define MCF_SSR_BE     		(0x02000000)


/*
 * Some needed coldfire registers
 */
#define MCF_PAR_PCIBG 		   MCF_REG16(0x000A48)
#define MCF_PAR_PCIBR   	   MCF_REG16(0x000A4A)
#define MCF_PAR_PSCn(x)            MCF_REG08(0x000A4F-((x)&0x3))
#define MCF_PAR_FECI2CIRQ   	   MCF_REG16(0x000A44)
#define MCF_PAR_DSPI               MCF_REG16(0x000A50)
#define MCF_PAR_TIMER              MCF_REG08(0X000A52)
#define MCF_EPPAR       	   MCF_REG16(0x000F00)
#define MCF_EPIER       	   MCF_REG08(0x000F05)
#define MCF_EPFR       	   	   MCF_REG08(0x000F0C)

/*
 * Some GPIO bitfields
 */
#define MCF_PAR_SDA			(0x0008)
#define MCF_PAR_SCL			(0x0004)
#define MCF_PAR_PSC_TXD             	(0x04)
#define MCF_PAR_PSC_RXD             	(0x08)
#define MCF_PAR_PSC_RTS(x)          	(((x)&0x03)<<4)
#define MCF_PAR_PSC_CTS(x)          	(((x)&0x03)<<6)
#define MCF_PAR_PSC_CTS_GPIO        	(0x00)
#define MCF_PAR_PSC_CTS_BCLK        	(0x80)
#define MCF_PAR_PSC_CTS_CTS         	(0xC0)
#define MCF_PAR_PSC_RTS_GPIO        	(0x00)
#define MCF_PAR_PSC_RTS_FSYNC       	(0x20)
#define MCF_PAR_PSC_RTS_RTS         	(0x30)
#define MCF_PAR_PSC_CANRX		(0x40)


/*
 * Some used coldfire values
 */
#define MCF_EPIER_EPIE(x)          (0x01 << (x))
#define MCF_EPPAR_EPPAx_FALLING    (2)
#define MCF_EPPAR_EPPA(n,x)        (((x)&0x0003) << (2*n))


#endif	/* m5485sim_h */

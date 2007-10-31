/*
 * Matt Waddel Matt.Waddel@freescale.com
 *
 * Copyright Freescale Semiconductor, Inc. 2007
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __MCF5445X_GPIO_H__
#define __MCF5445X_GPIO_H__

/*********************************************************************
*
* General Purpose I/O Module (GPIO)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_GPIO_PODR_FEC0H	MCF_REG08(0xFC0A4000)	/* FEC0 High Port Output Data Register */
#define MCF_GPIO_PODR_FEC0L	MCF_REG08(0xFC0A4001)	/* FEC0 Low Port Output Data Register */
#define MCF_GPIO_PODR_SSI	MCF_REG08(0xFC0A4002)	/* SSI Port Output Data Register */
#define MCF_GPIO_PODR_FBCTL	MCF_REG08(0xFC0A4003)	/* Flexbus Control Port Output Data Register */
#define MCF_GPIO_PODR_BE	MCF_REG08(0xFC0A4004)	/* Flexbus Byte Enable Port Output Data Register */
#define MCF_GPIO_PODR_CS	MCF_REG08(0xFC0A4005)	/* Flexbus Chip-Select Port Output Data Register */
#define MCF_GPIO_PODR_DMA	MCF_REG08(0xFC0A4006)	/* DMA Port Output Data Register */
#define MCF_GPIO_PODR_FECI2C	MCF_REG08(0xFC0A4007)	/* FEC1 / I2C Port Output Data Register */
#define MCF_GPIO_PODR_UART	MCF_REG08(0xFC0A4009)	/* UART Port Output Data Register */
#define MCF_GPIO_PODR_DSPI	MCF_REG08(0xFC0A400A)	/* DSPI Port Output Data Register */
#define MCF_GPIO_PODR_TIMER	MCF_REG08(0xFC0A400B)	/* Timer Port Output Data Register */
#define MCF_GPIO_PODR_PCI	MCF_REG08(0xFC0A400C)	/* PCI Port Output Data Register */
#define MCF_GPIO_PODR_USB	MCF_REG08(0xFC0A400D)	/* USB Port Output Data Register */
#define MCF_GPIO_PODR_ATAH	MCF_REG08(0xFC0A400E)	/* ATA High Port Output Data Register */
#define MCF_GPIO_PODR_ATAL	MCF_REG08(0xFC0A400F)	/* ATA Low Port Output Data Register */
#define MCF_GPIO_PODR_FEC1H	MCF_REG08(0xFC0A4010)	/* FEC1 High Port Output Data Register */
#define MCF_GPIO_PODR_FEC1L	MCF_REG08(0xFC0A4011)	/* FEC1 Low Port Output Data Register */
#define MCF_GPIO_PODR_FBADH	MCF_REG08(0xFC0A4014)	/* Flexbus AD High Port Output Data Register */
#define MCF_GPIO_PODR_FBADMH	MCF_REG08(0xFC0A4015)	/* Flexbus AD Med-High Port Output Data Register */
#define MCF_GPIO_PODR_FBADML	MCF_REG08(0xFC0A4016)	/* Flexbus AD Med-Low Port Output Data Register */
#define MCF_GPIO_PODR_FBADL	MCF_REG08(0xFC0A4017)	/* Flexbus AD Low Port Output Data Register */
#define MCF_GPIO_PDDR_FEC0H	MCF_REG08(0xFC0A4018)	/* FEC0 High Port Data Direction Register */
#define MCF_GPIO_PDDR_FEC0L	MCF_REG08(0xFC0A4019)	/* FEC0 Low Port Data Direction Register */
#define MCF_GPIO_PDDR_SSI	MCF_REG08(0xFC0A401A)	/* SSI Port Data Direction Register */
#define MCF_GPIO_PDDR_FBCTL	MCF_REG08(0xFC0A401B)	/* Flexbus Control Port Data Direction Register */
#define MCF_GPIO_PDDR_BE	MCF_REG08(0xFC0A401C)	/* Flexbus Byte Enable Port Data Direction Register */
#define MCF_GPIO_PDDR_CS	MCF_REG08(0xFC0A401D)	/* Flexbus Chip-Select Port Data Direction Register */
#define MCF_GPIO_PDDR_DMA	MCF_REG08(0xFC0A401E)	/* DMA Port Data Direction Register */
#define MCF_GPIO_PDDR_FECI2C	MCF_REG08(0xFC0A401F)	/* FEC1 / I2C Port Data Direction Register */
#define MCF_GPIO_PDDR_UART	MCF_REG08(0xFC0A4021)	/* UART Port Data Direction Register */
#define MCF_GPIO_PDDR_DSPI	MCF_REG08(0xFC0A4022)	/* DSPI Port Data Direction Register */
#define MCF_GPIO_PDDR_TIMER	MCF_REG08(0xFC0A4023)	/* Timer Port Data Direction Register */
#define MCF_GPIO_PDDR_PCI	MCF_REG08(0xFC0A4024)	/* PCI Port Data Direction Register */
#define MCF_GPIO_PDDR_USB	MCF_REG08(0xFC0A4025)	/* USB Port Data Direction Register */
#define MCF_GPIO_PDDR_ATAH	MCF_REG08(0xFC0A4026)	/* ATA High Port Data Direction Register */
#define MCF_GPIO_PDDR_ATAL	MCF_REG08(0xFC0A4027)	/* ATA Low Port Data Direction Register */
#define MCF_GPIO_PDDR_FEC1H	MCF_REG08(0xFC0A4028)	/* FEC1 High Port Data Direction Register */
#define MCF_GPIO_PDDR_FEC1L	MCF_REG08(0xFC0A4029)	/* FEC1 Low Port Data Direction Register */
#define MCF_GPIO_PDDR_FBADH	MCF_REG08(0xFC0A402C)	/* Flexbus AD High Port Data Direction Register */
#define MCF_GPIO_PDDR_FBADMH	MCF_REG08(0xFC0A402D)	/* Flexbus AD Med-High Port Data Direction Register */
#define MCF_GPIO_PDDR_FBADML	MCF_REG08(0xFC0A402E)	/* Flexbus AD Med-Low Port Data Direction Register */
#define MCF_GPIO_PDDR_FBADL	MCF_REG08(0xFC0A402F)	/* Flexbus AD Low Port Data Direction Register */
#define MCF_GPIO_PPDSDR_FEC0H	MCF_REG08(0xFC0A4030)	/* FEC0 High Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_FEC0L	MCF_REG08(0xFC0A4031)	/* FEC0 Low Port Clear Output Data Register */
#define MCF_GPIO_PPDSDR_SSI	MCF_REG08(0xFC0A4032)	/* SSI Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_FBCTL	MCF_REG08(0xFC0A4033)	/* Flexbus Control Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_BE	MCF_REG08(0xFC0A4034)	/* Flexbus Byte Enable Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_CS	MCF_REG08(0xFC0A4035)	/* Flexbus Chip-Select Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_DMA	MCF_REG08(0xFC0A4036)	/* DMA Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_FECI2C	MCF_REG08(0xFC0A4037)	/* FEC1 / I2C Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_UART	MCF_REG08(0xFC0A4039)	/* UART Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_DSPI	MCF_REG08(0xFC0A403A)	/* DSPI Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_TIMER	MCF_REG08(0xFC0A403B)	/* FTimer Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_PCI	MCF_REG08(0xFC0A403C)	/* PCI Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_USB	MCF_REG08(0xFC0A403D)	/* USB Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_ATAH	MCF_REG08(0xFC0A403E)	/* ATA High Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_ATAL	MCF_REG08(0xFC0A403F)	/* ATA Low Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_FEC1H	MCF_REG08(0xFC0A4040)	/* FEC1 High Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_FEC1L	MCF_REG08(0xFC0A4041)	/* FEC1 Low Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_FBADH	MCF_REG08(0xFC0A4044)	/* Flexbus AD High Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_FBADMH	MCF_REG08(0xFC0A4045)	/* Flexbus AD Med-High Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_FBADML	MCF_REG08(0xFC0A4046)	/* Flexbus AD Med-Low Port Pin Data/Set Data Register */
#define MCF_GPIO_PPDSDR_FBADL	MCF_REG08(0xFC0A4047)	/* Flexbus AD Low Port Pin Data/Set Data Register */
#define MCF_GPIO_PCLRR_FEC0H	MCF_REG08(0xFC0A4048)	/* FEC0 High Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_FEC0L	MCF_REG08(0xFC0A4049)	/* FEC0 Low Port Pin Data/Set Data Register */
#define MCF_GPIO_PCLRR_SSI	MCF_REG08(0xFC0A404A)	/* SSI Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_FBCTL	MCF_REG08(0xFC0A404B)	/* Flexbus Control Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_BE	MCF_REG08(0xFC0A404C)	/* Flexbus Byte Enable Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_CS	MCF_REG08(0xFC0A404D)	/* Flexbus Chip-Select Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_DMA	MCF_REG08(0xFC0A404E)	/* DMA Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_FECI2C	MCF_REG08(0xFC0A404F)	/* FEC1 / I2C Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_UART	MCF_REG08(0xFC0A4051)	/* UART Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_DSPI	MCF_REG08(0xFC0A4052)	/* DSPI Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_TIMER	MCF_REG08(0xFC0A4053)	/* Timer Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_PCI	MCF_REG08(0xFC0A4054)	/* PCI Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_USB	MCF_REG08(0xFC0A4055)	/* USB Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_ATAH	MCF_REG08(0xFC0A4056)	/* ATA High Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_ATAL	MCF_REG08(0xFC0A4057)	/* ATA Low Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_FEC1H	MCF_REG08(0xFC0A4058)	/* FEC1 High Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_FEC1L	MCF_REG08(0xFC0A4059)	/* FEC1 Low Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_FBADH	MCF_REG08(0xFC0A405C)	/* Flexbus AD High Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_FBADMH	MCF_REG08(0xFC0A405D)	/* Flexbus AD Med-High Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_FBADML	MCF_REG08(0xFC0A405E)	/* Flexbus AD Med-Low Port Clear Output Data Register */
#define MCF_GPIO_PCLRR_FBADL	MCF_REG08(0xFC0A405F)	/* Flexbus AD Low Port Clear Output Data Register */
#define MCF_GPIO_PAR_FEC	MCF_REG08(0xFC0A4060)	/* FEC Pin Assignment Register */
#define MCF_GPIO_PAR_DMA	MCF_REG08(0xFC0A4061)	/* DMA Pin Assignment Register */
#define MCF_GPIO_PAR_FBCTL	MCF_REG08(0xFC0A4062)	/* Flexbus Control Pin Assignment Register */
#define MCF_GPIO_PAR_DSPI	MCF_REG08(0xFC0A4063)	/* DSPI Pin Assignment Register */
#define MCF_GPIO_PAR_BE		MCF_REG08(0xFC0A4064)	/* Flexbus Byte-Enable Pin Assignment Register */
#define MCF_GPIO_PAR_CS		MCF_REG08(0xFC0A4065)	/* Flexbus Chip-Select Pin Assignment Register */
#define MCF_GPIO_PAR_TIMER	MCF_REG08(0xFC0A4066)	/* Time Pin Assignment Register */
#define MCF_GPIO_PAR_USB	MCF_REG08(0xFC0A4067)	/* USB Pin Assignment Register */
#define MCF_GPIO_PAR_UART	MCF_REG08(0xFC0A4069)	/* UART Pin Assignment Register */
#define MCF_GPIO_PAR_FECI2C	MCF_REG16(0xFC0A406A)	/* FEC / I2C Pin Assignment Register */
#define MCF_GPIO_PAR_SSI	MCF_REG16(0xFC0A406C)	/* SSI Pin Assignment Register */
#define MCF_GPIO_PAR_ATA	MCF_REG16(0xFC0A406E)	/* ATA Pin Assignment Register */
#define MCF_GPIO_PAR_IRQ	MCF_REG08(0xFC0A4070)	/* IRQ Pin Assignment Register */
#define MCF_GPIO_PAR_PCI	MCF_REG16(0xFC0A4072)	/* PCI Pin Assignment Register */
#define MCF_GPIO_MSCR_SDRAM	MCF_REG08(0xFC0A4074)	/* SDRAM Mode Select Control Register */
#define MCF_GPIO_MSCR_PCI	MCF_REG08(0xFC0A4075)	/* PCI Mode Select Control Register */
#define MCF_GPIO_DSCR_I2C	MCF_REG08(0xFC0A4078)	/* I2C Drive Strength Control Register */
#define MCF_GPIO_DSCR_FLEXBUS	MCF_REG08(0xFC0A4079)	/* FLEXBUS Drive Strength Control Register */
#define MCF_GPIO_DSCR_FEC	MCF_REG08(0xFC0A407A)	/* FEC Drive Strength Control Register */
#define MCF_GPIO_DSCR_UART	MCF_REG08(0xFC0A407B)	/* UART Drive Strength Control Register */
#define MCF_GPIO_DSCR_DSPI	MCF_REG08(0xFC0A407C)	/* DSPI Drive Strength Control Register */
#define MCF_GPIO_DSCR_TIMER	MCF_REG08(0xFC0A407D)	/* TIMER Drive Strength Control Register */
#define MCF_GPIO_DSCR_SSI	MCF_REG08(0xFC0A407E)	/* SSI Drive Strength Control Register */
#define MCF_GPIO_DSCR_DMA	MCF_REG08(0xFC0A407F)	/* DMA Drive Strength Control Register */
#define MCF_GPIO_DSCR_DEBUG	MCF_REG08(0xFC0A4080)	/* DEBUG Drive Strength Control Register */
#define MCF_GPIO_DSCR_RESET	MCF_REG08(0xFC0A4081)	/* RESET Drive Strength Control Register */
#define MCF_GPIO_DSCR_IRQ	MCF_REG08(0xFC0A4082)	/* IRQ Drive Strength Control Register */
#define MCF_GPIO_DSCR_USB	MCF_REG08(0xFC0A4083)	/* USB Drive Strength Control Register */
#define MCF_GPIO_DSCR_ATA	MCF_REG08(0xFC0A4084)	/* ATA Drive Strength Control Register */

/* Bit definitions and macros for PODR_FEC0H */
#define MCF_GPIO_PODR_FEC0H_PODR0	(0x01)	/* FEC0_CRS / ULPI_DATA6 */
#define MCF_GPIO_PODR_FEC0H_PODR1	(0x02)	/* FEC0_RXD0 / FEC0_RMII_RXD0 */
#define MCF_GPIO_PODR_FEC0H_PODR2	(0x04)	/* FEC0_RXDV/FEC0_RMII_CRS_DV */
#define MCF_GPIO_PODR_FEC0H_PODR3	(0x08)	/* FEC0_RXCLK / ULPI_DATA1 */
#define MCF_GPIO_PODR_FEC0H_PODR4	(0x10)	/* FEC0_COL / ULPI_DATA7 */
#define MCF_GPIO_PODR_FEC0H_PODR5	(0x20)	/* FEC0_TXD0 / FEC0_RMII_TXD0 */
#define MCF_GPIO_PODR_FEC0H_PODR6	(0x40)	/* FEC0_TXEN / FEC0_RMII_TXEN */
#define MCF_GPIO_PODR_FEC0H_PODR7	(0x80)	/* FEC0_TXCLK / FEC0_RMII_REF_CLK */

/* Bit definitions and macros for PODR_FEC0L */
#define MCF_GPIO_PODR_FEC0L_PODR0	(0x01)	/* FEC0_RXER / FEC0_RMII_RXER */
#define MCF_GPIO_PODR_FEC0L_PODR1	(0x02)	/* FEC0_RXD1 / FEC0_RMII_RXD1 */
#define MCF_GPIO_PODR_FEC0L_PODR2	(0x04)	/* FEC0_RXD2 / ULPI_DATA4 */
#define MCF_GPIO_PODR_FEC0L_PODR3	(0x08)	/* FEC0_RXD3 / ULPI_DATA5 */
#define MCF_GPIO_PODR_FEC0L_PODR4	(0x10)	/* FEC0_TXER / ULPI_DATA0 */
#define MCF_GPIO_PODR_FEC0L_PODR5	(0x20)	/* FEC0_TXD1 / FEC0_RMII_TXD1 */
#define MCF_GPIO_PODR_FEC0L_PODR6	(0x40)	/* FEC0_TXD2 / ULPI_DATA2 */
#define MCF_GPIO_PODR_FEC0L_PODR7	(0x80)	/* FEC0_TXD3 / ULPI_DATA3 */

/* Bit definitions and macros for PODR_SSI */
#define MCF_GPIO_PODR_SSI_PODR0		(0x01)	/* SSI_TXD / U1TXD */
#define MCF_GPIO_PODR_SSI_PODR1		(0x02)	/* SSI_RXD / U1RXD */
#define MCF_GPIO_PODR_SSI_PODR2		(0x04)	/* SSI_FS / U1RTS */
#define MCF_GPIO_PODR_SSI_PODR3		(0x08)	/* SSI_BCLK / U1CTS */
#define MCF_GPIO_PODR_SSI_PODR4		(0x10)	/* SSI_MCLK */

/* Bit definitions and macros for PODR_FBCTL */
#define MCF_GPIO_PODR_FBCTL_PODR0	(0x01)	/* FB_TS / FB_ALE / FB_TBST */
#define MCF_GPIO_PODR_FBCTL_PODR1	(0x02)	/* FB_RW */
#define MCF_GPIO_PODR_FBCTL_PODR2	(0x04)	/* FB_TA */
#define MCF_GPIO_PODR_FBCTL_PODR3	(0x08)	/* FB_OE */

/* Bit definitions and macros for PODR_BE */
#define MCF_GPIO_PODR_BE_PODR0		(0x01)	/* FB_BE/BWE0 / FB_SIZ0 */
#define MCF_GPIO_PODR_BE_PODR1		(0x02)	/* FB_BE/BWE1 / FB_SIZ1 */
#define MCF_GPIO_PODR_BE_PODR2		(0x04)	/* FB_BE/BWE2 / FB_SIZ2 */
#define MCF_GPIO_PODR_BE_PODR3		(0x08)	/* FB_BE/BWE3 / FB_SIZ3 */

/* Bit definitions and macros for PODR_CS */
#define MCF_GPIO_PODR_CS_PODR1		(0x02)	/* FB_CS1 */
#define MCF_GPIO_PODR_CS_PODR2		(0x04)	/* FB_CS2 */
#define MCF_GPIO_PODR_CS_PODR3		(0x08)	/* FB_CS3 */

/* Bit definitions and macros for PODR_DMA */
#define MCF_GPIO_PODR_DMA_PODR0		(0x01)	/* DREQ0 */
#define MCF_GPIO_PODR_DMA_PODR1		(0x02)	/* DACK0 / DSPI_PCS3 */
#define MCF_GPIO_PODR_DMA_PODR2		(0x04)	/* DREQ1 / USB_CLKIN */
#define MCF_GPIO_PODR_DMA_PODR3		(0x08)	/* DACK1 / ULPI_DIR */

/* Bit definitions and macros for PODR_FECI2C */
#define MCF_GPIO_PODR_FECI2C_PODR0	(0x01)	/* I2C_SDA / U2RXD */
#define MCF_GPIO_PODR_FECI2C_PODR1	(0x02)	/* I2C_SCL / U2TXD */
#define MCF_GPIO_PODR_FECI2C_PODR2	(0x04)	/* FEC0_MDIO */
#define MCF_GPIO_PODR_FECI2C_PODR3	(0x08)	/* FEC0_MDC */
#define MCF_GPIO_PODR_FECI2C_PODR4	(0x10)	/* FEC1_MDIO / ATA_DIOW */
#define MCF_GPIO_PODR_FECI2C_PODR5	(0x20)	/* FEC1_MDC / ATA_DIOR */

/* Bit definitions and macros for PODR_UART */
#define MCF_GPIO_PODR_UART_PODR0	(0x01)	/* U1TXD */
#define MCF_GPIO_PODR_UART_PODR1	(0x02)	/* U1RXD */
#define MCF_GPIO_PODR_UART_PODR2	(0x04)	/* U1RTS */
#define MCF_GPIO_PODR_UART_PODR3	(0x08)	/* U0CTS */
#define MCF_GPIO_PODR_UART_PODR4	(0x10)	/* U1TXD */
#define MCF_GPIO_PODR_UART_PODR5	(0x20)	/* U1RXD */
#define MCF_GPIO_PODR_UART_PODR6	(0x40)	/* U1RTS */
#define MCF_GPIO_PODR_UART_PODR7	(0x80)	/* U1CTS */

/* Bit definitions and macros for PODR_DSPI */
#define MCF_GPIO_PODR_DSPI_PODR0	(0x01)	/* DSPI_SOUT / SBF_DO */
#define MCF_GPIO_PODR_DSPI_PODR1	(0x02)	/* DSPI_SIN / SBF_DI */
#define MCF_GPIO_PODR_DSPI_PODR2	(0x04)	/* DSPI_SCK / SBF_CK */
#define MCF_GPIO_PODR_DSPI_PODR3	(0x08)	/* DSPI_PCS0/SS */
#define MCF_GPIO_PODR_DSPI_PODR4	(0x10)	/* DSPI_PCS1 / SBF_CS */
#define MCF_GPIO_PODR_DSPI_PODR5	(0x20)	/* DSPI_PCS2 */
#define MCF_GPIO_PODR_DSPI_PODR6	(0x40)	/* DSPI_PCS5/SS */

/* Bit definitions and macros for PODR_TIMER */
#define MCF_GPIO_PODR_TIMER_PODR0	(0x01)	/* T0IN / T0OUT / U2RTS */
#define MCF_GPIO_PODR_TIMER_PODR1	(0x02)	/* T1IN / T1OUT / U2CTS */
#define MCF_GPIO_PODR_TIMER_PODR2	(0x04)	/* T2IN / T2OUT / U2TXD */
#define MCF_GPIO_PODR_TIMER_PODR3	(0x08)	/* T3IN / T3OUT / U2RXD */

/* Bit definitions and macros for PODR_PCI */
#define MCF_GPIO_PODR_PCI_PODR0		(0x01)	/* PCI_REQ0/PCI_EXTGNT */
#define MCF_GPIO_PODR_PCI_PODR1		(0x02)	/* PCI_REQ1 */
#define MCF_GPIO_PODR_PCI_PODR2		(0x04)	/* PCI_REQ2 */
#define MCF_GPIO_PODR_PCI_PODR3		(0x08)	/* PCI_REQ3 / ATA_INTR */
#define MCF_GPIO_PODR_PCI_PODR4		(0x10)	/* PCI_GNT0/PCI_EXTREQ */
#define MCF_GPIO_PODR_PCI_PODR5		(0x20)	/* PCI_GNT1 /  */
#define MCF_GPIO_PODR_PCI_PODR6		(0x40)	/* PCI_GNT2 /  */
#define MCF_GPIO_PODR_PCI_PODR7		(0x80)	/* PCI_GNT3 / ATA_DMACK */

/* Bit definitions and macros for PODR_USB */
#define MCF_GPIO_PODR_USB_PODR0		(0x01)	/* USB_VBUS_OC / ULPI_STP */
#define MCF_GPIO_PODR_USB_PODR1		(0x02)	/* USB_VBUS_EN / USB_PULLUP / ULPI_NXT */

/* Bit definitions and macros for PODR_ATAH */
#define MCF_GPIO_PODR_ATAH_PODR0	(0x01)	/* ATA_DA0 */
#define MCF_GPIO_PODR_ATAH_PODR1	(0x02)	/* ATA_DA1 */
#define MCF_GPIO_PODR_ATAH_PODR2	(0x04)	/* ATA_DA2 */
#define MCF_GPIO_PODR_ATAH_PODR3	(0x08)	/* ATA_CS0 */
#define MCF_GPIO_PODR_ATAH_PODR4	(0x10)	/* ATA_CS1 */
#define MCF_GPIO_PODR_ATAH_PODR5	(0x20)	/* ATA_BUFFER_EN */

/* Bit definitions and macros for PODR_ATAL */
#define MCF_GPIO_PODR_ATAL_PODR0	(0x01)	/* ATA_IORDY */
#define MCF_GPIO_PODR_ATAL_PODR1	(0x02)	/* ATA_DMARQ */
#define MCF_GPIO_PODR_ATAL_PODR2	(0x04)	/* ATA_RESET */

/* Bit definitions and macros for PODR_FEC1H */
#define MCF_GPIO_PODR_FEC1H_PODR0	(0x01)	/* FEC1_CRS / ATA_DATA6 */
#define MCF_GPIO_PODR_FEC1H_PODR1	(0x02)	/* FEC1_RXD0 / FEC1_RMII_RXD0 / ATA_DATA13 */
#define MCF_GPIO_PODR_FEC1H_PODR2	(0x04)	/* FEC1_RXDV / FEC1_RMII_CRS_DV / ATA_DATA15 */
#define MCF_GPIO_PODR_FEC1H_PODR3	(0x08)	/* FEC1_RXCLK / ATA_DATA5 */
#define MCF_GPIO_PODR_FEC1H_PODR4	(0x10)	/* FEC1_COL / ATA_DATA7 */
#define MCF_GPIO_PODR_FEC1H_PODR5	(0x20)	/* FEC1_TXD0 / FEC1_RMII_TXD0 / ATA_DATA9 */
#define MCF_GPIO_PODR_FEC1H_PODR6	(0x40)	/* FEC1_TXEN / FEC1_RMII_TXEN / ATA_DATA8 */
#define MCF_GPIO_PODR_FEC1H_PODR7	(0x80)	/* FEC1_TXCLK / FEC1_RMII_REF_CLK / ATA_DATA11 */

/* Bit definitions and macros for PODR_FEC1L */
#define MCF_GPIO_PODR_FEC1L_PODR0	(0x01)	/* FEC1_RXER / FEC1_RMII_RXER / ATA_DATA12 */
#define MCF_GPIO_PODR_FEC1L_PODR1	(0x02)	/* FEC1_RXD1 / FEC1_RMII_RXD1 / ATA_DATA14 */
#define MCF_GPIO_PODR_FEC1L_PODR2	(0x04)	/* FEC1_RXD2 / ATA_DATA3 */
#define MCF_GPIO_PODR_FEC1L_PODR3	(0x08)	/* FEC1_RXD3 / ATA_DATA4 */
#define MCF_GPIO_PODR_FEC1L_PODR4	(0x10)	/* FEC1_TXER / ATA_DATA0 */
#define MCF_GPIO_PODR_FEC1L_PODR5	(0x20)	/* FEC1_TXD1 / FEC1_RMII_TXD1 / ATA_DATA10 */
#define MCF_GPIO_PODR_FEC1L_PODR6	(0x40)	/* FEC1_TXD2 / ATA_DATA1 */
#define MCF_GPIO_PODR_FEC1L_PODR7	(0x80)	/* FEC1_TXD3 / ATA_DATA2 */

/* Bit definitions and macros for PODR_FBADH */
#define MCF_GPIO_PODR_FBADH_PODR0	(0x01)	/* FB_AD24 */
#define MCF_GPIO_PODR_FBADH_PODR1	(0x02)	/* FB_AD25 */
#define MCF_GPIO_PODR_FBADH_PODR2	(0x04)	/* FB_AD26 */
#define MCF_GPIO_PODR_FBADH_PODR3	(0x08)	/* FB_AD27 */
#define MCF_GPIO_PODR_FBADH_PODR4	(0x10)	/* FB_AD28 */
#define MCF_GPIO_PODR_FBADH_PODR5	(0x20)	/* FB_AD29 */
#define MCF_GPIO_PODR_FBADH_PODR6	(0x40)	/* FB_AD30 */
#define MCF_GPIO_PODR_FBADH_PODR7	(0x80)	/* FB_AD31 */

/* Bit definitions and macros for PODR_FBADMH */
#define MCF_GPIO_PODR_FBADMH_PODR0	(0x01)	/* FB_AD16 */
#define MCF_GPIO_PODR_FBADMH_PODR1	(0x02)	/* FB_AD17 */
#define MCF_GPIO_PODR_FBADMH_PODR2	(0x04)	/* FB_AD18 */
#define MCF_GPIO_PODR_FBADMH_PODR3	(0x08)	/* FB_AD19 */
#define MCF_GPIO_PODR_FBADMH_PODR4	(0x10)	/* FB_AD20 */
#define MCF_GPIO_PODR_FBADMH_PODR5	(0x20)	/* FB_AD21 */
#define MCF_GPIO_PODR_FBADMH_PODR6	(0x40)	/* FB_AD22 */
#define MCF_GPIO_PODR_FBADMH_PODR7	(0x80)	/* FB_AD23 */

/* Bit definitions and macros for PODR_FBADML */
#define MCF_GPIO_PODR_FBADML_PODR0	(0x01)	/* FB_AD8 */
#define MCF_GPIO_PODR_FBADML_PODR1	(0x02)	/* FB_AD9 */
#define MCF_GPIO_PODR_FBADML_PODR2	(0x04)	/* FB_AD10 */
#define MCF_GPIO_PODR_FBADML_PODR3	(0x08)	/* FB_AD11 */
#define MCF_GPIO_PODR_FBADML_PODR4	(0x10)	/* FB_AD12 */
#define MCF_GPIO_PODR_FBADML_PODR5	(0x20)	/* FB_AD13 */
#define MCF_GPIO_PODR_FBADML_PODR6	(0x40)	/* FB_AD14 */
#define MCF_GPIO_PODR_FBADML_PODR7	(0x80)	/* FB_AD15 */

/* Bit definitions and macros for PODR_FBADL */
#define MCF_GPIO_PODR_FBADL_PODR0	(0x01)	/* FB_AD0 */
#define MCF_GPIO_PODR_FBADL_PODR1	(0x02)	/* FB_AD1 */
#define MCF_GPIO_PODR_FBADL_PODR2	(0x04)	/* FB_AD2 */
#define MCF_GPIO_PODR_FBADL_PODR3	(0x08)	/* FB_AD3 */
#define MCF_GPIO_PODR_FBADL_PODR4	(0x10)	/* FB_AD4 */
#define MCF_GPIO_PODR_FBADL_PODR5	(0x20)	/* FB_AD5 */
#define MCF_GPIO_PODR_FBADL_PODR6	(0x40)	/* FB_AD6 */
#define MCF_GPIO_PODR_FBADL_PODR7	(0x80)	/* FB_AD7 */

/* Bit definitions and macros for PDDR_FEC0H */
#define MCF_GPIO_PDDR_FEC0H_PDDR0	(0x01)	/* FEC0_CRS / ULPI_DATA6 */
#define MCF_GPIO_PDDR_FEC0H_PDDR1	(0x02)	/* FEC0_RXD0 / FEC0_RMII_RXD0 */
#define MCF_GPIO_PDDR_FEC0H_PDDR2	(0x04)	/* FEC0_RXDV/FEC0_RMII_CRS_DV */
#define MCF_GPIO_PDDR_FEC0H_PDDR3	(0x08)	/* FEC0_RXCLK / ULPI_DATA1 */
#define MCF_GPIO_PDDR_FEC0H_PDDR4	(0x10)	/* FEC0_COL / ULPI_DATA7 */
#define MCF_GPIO_PDDR_FEC0H_PDDR5	(0x20)	/* FEC0_TXD0 / FEC0_RMII_TXD0 */
#define MCF_GPIO_PDDR_FEC0H_PDDR6	(0x40)	/* FEC0_TXEN / FEC0_RMII_TXEN */
#define MCF_GPIO_PDDR_FEC0H_PDDR7	(0x80)	/* FEC0_TXCLK / FEC0_RMII_REF_CLK */

/* Bit definitions and macros for PDDR_FEC0L */
#define MCF_GPIO_PDDR_FEC0L_PDDR0	(0x01)	/* FEC0_RXER / FEC0_RMII_RXER */
#define MCF_GPIO_PDDR_FEC0L_PDDR1	(0x02)	/* FEC0_RXD1 / FEC0_RMII_RXD1 */
#define MCF_GPIO_PDDR_FEC0L_PDDR2	(0x04)	/* FEC0_RXD2 / ULPI_DATA4 */
#define MCF_GPIO_PDDR_FEC0L_PDDR3	(0x08)	/* FEC0_RXD3 / ULPI_DATA5 */
#define MCF_GPIO_PDDR_FEC0L_PDDR4	(0x10)	/* FEC0_TXER / ULPI_DATA0 */
#define MCF_GPIO_PDDR_FEC0L_PDDR5	(0x20)	/* FEC0_TXD1 / FEC0_RMII_TXD1 */
#define MCF_GPIO_PDDR_FEC0L_PDDR6	(0x40)	/* FEC0_TXD2 / ULPI_DATA2 */
#define MCF_GPIO_PDDR_FEC0L_PDDR7	(0x80)	/* FEC0_TXD3 / ULPI_DATA3 */

/* Bit definitions and macros for PDDR_SSI */
#define MCF_GPIO_PDDR_SSI_PDDR0		(0x01)	/* SSI_TXD / U1TXD */
#define MCF_GPIO_PDDR_SSI_PDDR1		(0x02)	/* SSI_RXD / U1RXD */
#define MCF_GPIO_PDDR_SSI_PDDR2		(0x04)	/* SSI_FS / U1RTS */
#define MCF_GPIO_PDDR_SSI_PDDR3		(0x08)	/* SSI_BCLK / U1CTS */
#define MCF_GPIO_PDDR_SSI_PDDR4		(0x10)	/* SSI_MCLK */

/* Bit definitions and macros for PDDR_FBCTL */
#define MCF_GPIO_PDDR_FBCTL_PDDR0	(0x01)	/* FB_TS / FB_ALE / FB_TBST */
#define MCF_GPIO_PDDR_FBCTL_PDDR1	(0x02)	/* FB_RW */
#define MCF_GPIO_PDDR_FBCTL_PDDR2	(0x04)	/* FB_TA */
#define MCF_GPIO_PDDR_FBCTL_PDDR3	(0x08)	/* FB_OE */

/* Bit definitions and macros for PDDR_BE */
#define MCF_GPIO_PDDR_BE_PDDR0		(0x01)	/* FB_BE/BWE0 / FB_SIZ0 */
#define MCF_GPIO_PDDR_BE_PDDR1		(0x02)	/* FB_BE/BWE1 / FB_SIZ1 */
#define MCF_GPIO_PDDR_BE_PDDR2		(0x04)	/* FB_BE/BWE2 / FB_SIZ2 */
#define MCF_GPIO_PDDR_BE_PDDR3		(0x08)	/* FB_BE/BWE3 / FB_SIZ3 */

/* Bit definitions and macros for PDDR_CS */
#define MCF_GPIO_PDDR_CS_PDDR1		(0x02)	/* FB_CS1 */
#define MCF_GPIO_PDDR_CS_PDDR2		(0x04)	/* FB_CS2 */
#define MCF_GPIO_PDDR_CS_PDDR3		(0x08)	/* FB_CS3 */

/* Bit definitions and macros for PDDR_DMA */
#define MCF_GPIO_PDDR_DMA_PDDR0		(0x01)	/* DREQ0 */
#define MCF_GPIO_PDDR_DMA_PDDR1		(0x02)	/* DACK0 / DSPI_PCS3 */
#define MCF_GPIO_PDDR_DMA_PDDR2		(0x04)	/* DREQ1 / USB_CLKIN */
#define MCF_GPIO_PDDR_DMA_PDDR3		(0x08)	/* DACK1 / ULPI_DIR */

/* Bit definitions and macros for PDDR_FECI2C */
#define MCF_GPIO_PDDR_FECI2C_PDDR0	(0x01)	/* I2C_SDA / U2RXD */
#define MCF_GPIO_PDDR_FECI2C_PDDR1	(0x02)	/* I2C_SCL / U2TXD */
#define MCF_GPIO_PDDR_FECI2C_PDDR2	(0x04)	/* FEC0_MDIO */
#define MCF_GPIO_PDDR_FECI2C_PDDR3	(0x08)	/* FEC0_MDC */
#define MCF_GPIO_PDDR_FECI2C_PDDR4	(0x10)	/* FEC1_MDIO / ATA_DIOW */
#define MCF_GPIO_PDDR_FECI2C_PDDR5	(0x20)	/* FEC1_MDC / ATA_DIOR */

/* Bit definitions and macros for PDDR_UART */
#define MCF_GPIO_PDDR_UART_PDDR0	(0x01)	/* U1TXD */
#define MCF_GPIO_PDDR_UART_PDDR1	(0x02)	/* U1RXD */
#define MCF_GPIO_PDDR_UART_PDDR2	(0x04)	/* U1RTS */
#define MCF_GPIO_PDDR_UART_PDDR3	(0x08)	/* U0CTS */
#define MCF_GPIO_PDDR_UART_PDDR4	(0x10)	/* U1TXD */
#define MCF_GPIO_PDDR_UART_PDDR5	(0x20)	/* U1RXD */
#define MCF_GPIO_PDDR_UART_PDDR6	(0x40)	/* U1RTS */
#define MCF_GPIO_PDDR_UART_PDDR7	(0x80)	/* U1CTS */

/* Bit definitions and macros for PDDR_DSPI */
#define MCF_GPIO_PDDR_DSPI_PDDR0	(0x01)	/* DSPI_SOUT / SBF_DO */
#define MCF_GPIO_PDDR_DSPI_PDDR1	(0x02)	/* DSPI_SIN / SBF_DI */
#define MCF_GPIO_PDDR_DSPI_PDDR2	(0x04)	/* DSPI_SCK / SBF_CK */
#define MCF_GPIO_PDDR_DSPI_PDDR3	(0x08)	/* DSPI_PCS0/SS */
#define MCF_GPIO_PDDR_DSPI_PDDR4	(0x10)	/* DSPI_PCS1 / SBF_CS */
#define MCF_GPIO_PDDR_DSPI_PDDR5	(0x20)	/* DSPI_PCS2 */
#define MCF_GPIO_PDDR_DSPI_PDDR6	(0x40)	/* DSPI_PCS5/SS */

/* Bit definitions and macros for PDDR_TIMER */
#define MCF_GPIO_PDDR_TIMER_PDDR0	(0x01)	/* T0IN / T0OUT / U2RTS */
#define MCF_GPIO_PDDR_TIMER_PDDR1	(0x02)	/* T1IN / T1OUT / U2CTS */
#define MCF_GPIO_PDDR_TIMER_PDDR2	(0x04)	/* T2IN / T2OUT / U2TXD */
#define MCF_GPIO_PDDR_TIMER_PDDR3	(0x08)	/* T3IN / T3OUT / U2RXD */

/* Bit definitions and macros for PDDR_PCI */
#define MCF_GPIO_PDDR_PCI_PDDR0		(0x01)	/* PCI_REQ0/PCI_EXTGNT */
#define MCF_GPIO_PDDR_PCI_PDDR1		(0x02)	/* PCI_REQ1 */
#define MCF_GPIO_PDDR_PCI_PDDR2		(0x04)	/* PCI_REQ2 */
#define MCF_GPIO_PDDR_PCI_PDDR3		(0x08)	/* PCI_REQ3 / ATA_INTR */
#define MCF_GPIO_PDDR_PCI_PDDR4		(0x10)	/* PCI_GNT0/PCI_EXTREQ */
#define MCF_GPIO_PDDR_PCI_PDDR5		(0x20)	/* PCI_GNT1 /  */
#define MCF_GPIO_PDDR_PCI_PDDR6		(0x40)	/* PCI_GNT2 /  */
#define MCF_GPIO_PDDR_PCI_PDDR7		(0x80)	/* PCI_GNT3 / ATA_DMACK */

/* Bit definitions and macros for PDDR_USB */
#define MCF_GPIO_PDDR_USB_PDDR0		(0x01)	/* USB_VBUS_OC / ULPI_STP */
#define MCF_GPIO_PDDR_USB_PDDR1		(0x02)	/* USB_VBUS_EN / USB_PULLUP / ULPI_NXT */

/* Bit definitions and macros for PDDR_ATAH */
#define MCF_GPIO_PDDR_ATAH_PDDR0	(0x01)	/* ATA_DA0 */
#define MCF_GPIO_PDDR_ATAH_PDDR1	(0x02)	/* ATA_DA1 */
#define MCF_GPIO_PDDR_ATAH_PDDR2	(0x04)	/* ATA_DA2 */
#define MCF_GPIO_PDDR_ATAH_PDDR3	(0x08)	/* ATA_CS0 */
#define MCF_GPIO_PDDR_ATAH_PDDR4	(0x10)	/* ATA_CS1 */
#define MCF_GPIO_PDDR_ATAH_PDDR5	(0x20)	/* ATA_BUFFER_EN */

/* Bit definitions and macros for PDDR_ATAL */
#define MCF_GPIO_PDDR_ATAL_PDDR0	(0x01)	/* ATA_IORDY */
#define MCF_GPIO_PDDR_ATAL_PDDR1	(0x02)	/* ATA_DMARQ */
#define MCF_GPIO_PDDR_ATAL_PDDR2	(0x04)	/* ATA_RESET */

/* Bit definitions and macros for PDDR_FEC1H */
#define MCF_GPIO_PDDR_FEC1H_PDDR0	(0x01)	/* FEC1_CRS / ATA_DATA6 */
#define MCF_GPIO_PDDR_FEC1H_PDDR1	(0x02)	/* FEC1_RXD0 / FEC1_RMII_RXD0 / ATA_DATA13 */
#define MCF_GPIO_PDDR_FEC1H_PDDR2	(0x04)	/* FEC1_RXDV / FEC1_RMII_CRS_DV / ATA_DATA15 */
#define MCF_GPIO_PDDR_FEC1H_PDDR3	(0x08)	/* FEC1_RXCLK / ATA_DATA5 */
#define MCF_GPIO_PDDR_FEC1H_PDDR4	(0x10)	/* FEC1_COL / ATA_DATA7 */
#define MCF_GPIO_PDDR_FEC1H_PDDR5	(0x20)	/* FEC1_TXD0 / FEC1_RMII_TXD0 / ATA_DATA9 */
#define MCF_GPIO_PDDR_FEC1H_PDDR6	(0x40)	/* FEC1_TXEN / FEC1_RMII_TXEN / ATA_DATA8 */
#define MCF_GPIO_PDDR_FEC1H_PDDR7	(0x80)	/* FEC1_TXCLK / FEC1_RMII_REF_CLK / ATA_DATA11 */

/* Bit definitions and macros for PDDR_FEC1L */
#define MCF_GPIO_PDDR_FEC1L_PDDR0	(0x01)	/* FEC1_RXER / FEC1_RMII_RXER / ATA_DATA12 */
#define MCF_GPIO_PDDR_FEC1L_PDDR1	(0x02)	/* FEC1_RXD1 / FEC1_RMII_RXD1 / ATA_DATA14 */
#define MCF_GPIO_PDDR_FEC1L_PDDR2	(0x04)	/* FEC1_RXD2 / ATA_DATA3 */
#define MCF_GPIO_PDDR_FEC1L_PDDR3	(0x08)	/* FEC1_RXD3 / ATA_DATA4 */
#define MCF_GPIO_PDDR_FEC1L_PDDR4	(0x10)	/* FEC1_TXER / ATA_DATA0 */
#define MCF_GPIO_PDDR_FEC1L_PDDR5	(0x20)	/* FEC1_TXD1 / FEC1_RMII_TXD1 / ATA_DATA10 */
#define MCF_GPIO_PDDR_FEC1L_PDDR6	(0x40)	/* FEC1_TXD2 / ATA_DATA1 */
#define MCF_GPIO_PDDR_FEC1L_PDDR7	(0x80)	/* FEC1_TXD3 / ATA_DATA2 */

/* Bit definitions and macros for PDDR_FBADH */
#define MCF_GPIO_PDDR_FBADH_PDDR0	(0x01)	/* FB_AD24 */
#define MCF_GPIO_PDDR_FBADH_PDDR1	(0x02)	/* FB_AD25 */
#define MCF_GPIO_PDDR_FBADH_PDDR2	(0x04)	/* FB_AD26 */
#define MCF_GPIO_PDDR_FBADH_PDDR3	(0x08)	/* FB_AD27 */
#define MCF_GPIO_PDDR_FBADH_PDDR4	(0x10)	/* FB_AD28 */
#define MCF_GPIO_PDDR_FBADH_PDDR5	(0x20)	/* FB_AD29 */
#define MCF_GPIO_PDDR_FBADH_PDDR6	(0x40)	/* FB_AD30 */
#define MCF_GPIO_PDDR_FBADH_PDDR7	(0x80)	/* FB_AD31 */

/* Bit definitions and macros for PDDR_FBADMH */
#define MCF_GPIO_PDDR_FBADMH_PDDR0	(0x01)	/* FB_AD16 */
#define MCF_GPIO_PDDR_FBADMH_PDDR1	(0x02)	/* FB_AD17 */
#define MCF_GPIO_PDDR_FBADMH_PDDR2	(0x04)	/* FB_AD18 */
#define MCF_GPIO_PDDR_FBADMH_PDDR3	(0x08)	/* FB_AD19 */
#define MCF_GPIO_PDDR_FBADMH_PDDR4	(0x10)	/* FB_AD20 */
#define MCF_GPIO_PDDR_FBADMH_PDDR5	(0x20)	/* FB_AD21 */
#define MCF_GPIO_PDDR_FBADMH_PDDR6	(0x40)	/* FB_AD22 */
#define MCF_GPIO_PDDR_FBADMH_PDDR7	(0x80)	/* FB_AD23 */

/* Bit definitions and macros for PDDR_FBADML */
#define MCF_GPIO_PDDR_FBADML_PDDR0	(0x01)	/* FB_AD8 */
#define MCF_GPIO_PDDR_FBADML_PDDR1	(0x02)	/* FB_AD9 */
#define MCF_GPIO_PDDR_FBADML_PDDR2	(0x04)	/* FB_AD10 */
#define MCF_GPIO_PDDR_FBADML_PDDR3	(0x08)	/* FB_AD11 */
#define MCF_GPIO_PDDR_FBADML_PDDR4	(0x10)	/* FB_AD12 */
#define MCF_GPIO_PDDR_FBADML_PDDR5	(0x20)	/* FB_AD13 */
#define MCF_GPIO_PDDR_FBADML_PDDR6	(0x40)	/* FB_AD14 */
#define MCF_GPIO_PDDR_FBADML_PDDR7	(0x80)	/* FB_AD15 */

/* Bit definitions and macros for PDDR_FBADL */
#define MCF_GPIO_PDDR_FBADL_PDDR0	(0x01)	/* FB_AD0 */
#define MCF_GPIO_PDDR_FBADL_PDDR1	(0x02)	/* FB_AD1 */
#define MCF_GPIO_PDDR_FBADL_PDDR2	(0x04)	/* FB_AD2 */
#define MCF_GPIO_PDDR_FBADL_PDDR3	(0x08)	/* FB_AD3 */
#define MCF_GPIO_PDDR_FBADL_PDDR4	(0x10)	/* FB_AD4 */
#define MCF_GPIO_PDDR_FBADL_PDDR5	(0x20)	/* FB_AD5 */
#define MCF_GPIO_PDDR_FBADL_PDDR6	(0x40)	/* FB_AD6 */
#define MCF_GPIO_PDDR_FBADL_PDDR7	(0x80)	/* FB_AD7 */

/* Bit definitions and macros for PPDSDR_FEC0H */
#define MCF_GPIO_PPDSDR_FEC0H_PPDR0	(0x01)	/* FEC0_CRS / ULPI_DATA6 */
#define MCF_GPIO_PPDSDR_FEC0H_PPDR1	(0x02)	/* FEC0_RXD0 / FEC0_RMII_RXD0 */
#define MCF_GPIO_PPDSDR_FEC0H_PPDR2	(0x04)	/* FEC0_RXDV / FEC0_RMII_CRS_DV */
#define MCF_GPIO_PPDSDR_FEC0H_PPDR3	(0x08)	/* FEC0_RXCLK / ULPI_DATA1 */
#define MCF_GPIO_PPDSDR_FEC0H_PPDR4	(0x10)	/* FEC0_COL / ULPI_DATA7 */
#define MCF_GPIO_PPDSDR_FEC0H_PPDR5	(0x20)	/* FEC0_TXD0 / FEC0_RMII_TXD0 */
#define MCF_GPIO_PPDSDR_FEC0H_PPDR6	(0x40)	/* FEC0_TXEN / FEC0_RMII_TXEN */
#define MCF_GPIO_PPDSDR_FEC0H_PPDR7	(0x80)	/* FEC0_TXCLK / FEC0_RMII_REF_CLK */

/* Bit definitions and macros for PPDSDR_FEC0L */
#define MCF_GPIO_PPDSDR_FEC0L_PCLRR0	(0x01)	/* FEC0_RXER / FEC0_RMII_RXER */
#define MCF_GPIO_PPDSDR_FEC0L_PCLRR1	(0x02)	/* FEC0_RXD1 / FEC0_RMII_RXD1 */
#define MCF_GPIO_PPDSDR_FEC0L_PCLRR2	(0x04)	/* FEC0_RXD2 / ULPI_DATA4 */
#define MCF_GPIO_PPDSDR_FEC0L_PCLRR3	(0x08)	/* FEC0_RXD3 / ULPI_DATA5 */
#define MCF_GPIO_PPDSDR_FEC0L_PCLRR4	(0x10)	/* FEC0_TXER / ULPI_DATA0 */
#define MCF_GPIO_PPDSDR_FEC0L_PCLRR5	(0x20)	/* FEC0_TXD1 / FEC0_RMII_TXD1 */
#define MCF_GPIO_PPDSDR_FEC0L_PCLRR6	(0x40)	/* FEC0_TXD2 / ULPI_DATA2 */
#define MCF_GPIO_PPDSDR_FEC0L_PCLRR7	(0x80)	/* FEC0_TXD3 / ULPI_DATA3 */

/* Bit definitions and macros for PPDSDR_SSI */
#define MCF_GPIO_PPDSDR_SSI_PPDR0	(0x01)	/* SSI_TXD / U1TXD */
#define MCF_GPIO_PPDSDR_SSI_PPDR1	(0x02)	/* SSI_RXD / U1RXD */
#define MCF_GPIO_PPDSDR_SSI_PPDR2	(0x04)	/* SSI_FS / U1RTS */
#define MCF_GPIO_PPDSDR_SSI_PPDR3	(0x08)	/* SSI_BCLK / U1CTS */
#define MCF_GPIO_PPDSDR_SSI_PPDR4	(0x10)	/* SSI_MCLK */

/* Bit definitions and macros for PPDSDR_FBCTL */
#define MCF_GPIO_PPDSDR_FBCTL_PPDR0	(0x01)	/* FB_TS / FB_ALE / FB_TBST */
#define MCF_GPIO_PPDSDR_FBCTL_PPDR1	(0x02)	/* FB_RW */
#define MCF_GPIO_PPDSDR_FBCTL_PPDR2	(0x04)	/* FB_TA */
#define MCF_GPIO_PPDSDR_FBCTL_PPDR3	(0x08)	/* FB_OE */

/* Bit definitions and macros for PPDSDR_BE */
#define MCF_GPIO_PPDSDR_BE_PPDR0	(0x01)	/* FB_BE/BWE0 / FB_SIZ0 */
#define MCF_GPIO_PPDSDR_BE_PPDR1	(0x02)	/* FB_BE/BWE1 / FB_SIZ1 */
#define MCF_GPIO_PPDSDR_BE_PPDR2	(0x04)	/* FB_BE/BWE2 / FB_SIZ2 */
#define MCF_GPIO_PPDSDR_BE_PPDR3	(0x08)	/* FB_BE/BWE3 / FB_SIZ3 */

/* Bit definitions and macros for PPDSDR_CS */
#define MCF_GPIO_PPDSDR_CS_PPDR1	(0x02)	/* FB_CS1 */
#define MCF_GPIO_PPDSDR_CS_PPDR2	(0x04)	/* FB_CS2 */
#define MCF_GPIO_PPDSDR_CS_PPDR3	(0x08)	/* FB_CS3 */

/* Bit definitions and macros for PPDSDR_DMA */
#define MCF_GPIO_PPDSDR_DMA_PPDR0	(0x01)	/* DREQ0 */
#define MCF_GPIO_PPDSDR_DMA_PPDR1	(0x02)	/* DACK0 / DSPI_PCS3 */
#define MCF_GPIO_PPDSDR_DMA_PPDR2	(0x04)	/* DREQ1 / USB_CLKIN */
#define MCF_GPIO_PPDSDR_DMA_PPDR3	(0x08)	/* DACK1 / ULPI_DIR */

/* Bit definitions and macros for PPDSDR_FECI2C */
#define MCF_GPIO_PPDSDR_FECI2C_PPDR0	(0x01)	/* I2C_SDA / U2RXD */
#define MCF_GPIO_PPDSDR_FECI2C_PPDR1	(0x02)	/* I2C_SCL / U2TXD */
#define MCF_GPIO_PPDSDR_FECI2C_PPDR2	(0x04)	/* FEC0_MDIO */
#define MCF_GPIO_PPDSDR_FECI2C_PPDR3	(0x08)	/* FEC0_MDC */
#define MCF_GPIO_PPDSDR_FECI2C_PPDR4	(0x10)	/* FEC1_MDIO / ATA_DIOW */
#define MCF_GPIO_PPDSDR_FECI2C_PPDR5	(0x20)	/* FEC1_MDC / ATA_DIOR */

/* Bit definitions and macros for PPDSDR_UART */
#define MCF_GPIO_PPDSDR_UART_PPDR0	(0x01)	/* U1TXD */
#define MCF_GPIO_PPDSDR_UART_PPDR1	(0x02)	/* U1RXD */
#define MCF_GPIO_PPDSDR_UART_PPDR2	(0x04)	/* U1RTS */
#define MCF_GPIO_PPDSDR_UART_PPDR3	(0x08)	/* U0CTS */
#define MCF_GPIO_PPDSDR_UART_PPDR4	(0x10)	/* U1TXD */
#define MCF_GPIO_PPDSDR_UART_PPDR5	(0x20)	/* U1RXD */
#define MCF_GPIO_PPDSDR_UART_PPDR6	(0x40)	/* U1RTS */
#define MCF_GPIO_PPDSDR_UART_PPDR7	(0x80)	/* U1CTS */

/* Bit definitions and macros for PPDSDR_DSPI */
#define MCF_GPIO_PPDSDR_DSPI_PPDR0	(0x01)	/* DSPI_SOUT / SBF_DO */
#define MCF_GPIO_PPDSDR_DSPI_PPDR1	(0x02)	/* DSPI_SIN / SBF_DI */
#define MCF_GPIO_PPDSDR_DSPI_PPDR2	(0x04)	/* DSPI_SCK / SBF_CK */
#define MCF_GPIO_PPDSDR_DSPI_PPDR3	(0x08)	/* DSPI_PCS0/SS */
#define MCF_GPIO_PPDSDR_DSPI_PPDR4	(0x10)	/* DSPI_PCS1 / SBF_CS */
#define MCF_GPIO_PPDSDR_DSPI_PPDR5	(0x20)	/* DSPI_PCS2 */
#define MCF_GPIO_PPDSDR_DSPI_PPDR6	(0x40)	/* DSPI_PCS5/SS */

/* Bit definitions and macros for PPDSDR_TIMER */
#define MCF_GPIO_PPDSDR_TIMER_PPDR0	(0x01)	/* T0IN / T0OUT / U2RTS */
#define MCF_GPIO_PPDSDR_TIMER_PPDR1	(0x02)	/* T1IN / T1OUT / U2CTS */
#define MCF_GPIO_PPDSDR_TIMER_PPDR2	(0x04)	/* T2IN / T2OUT / U2TXD */
#define MCF_GPIO_PPDSDR_TIMER_PPDR3	(0x08)	/* T3IN / T3OUT / U2RXD */

/* Bit definitions and macros for PPDSDR_PCI */
#define MCF_GPIO_PPDSDR_PCI_PPDR0	(0x01)	/* PCI_REQ0/PCI_EXTGNT */
#define MCF_GPIO_PPDSDR_PCI_PPDR1	(0x02)	/* PCI_REQ1 */
#define MCF_GPIO_PPDSDR_PCI_PPDR2	(0x04)	/* PCI_REQ2 */
#define MCF_GPIO_PPDSDR_PCI_PPDR3	(0x08)	/* PCI_REQ3 / ATA_INTR */
#define MCF_GPIO_PPDSDR_PCI_PPDR4	(0x10)	/* PCI_GNT0/PCI_EXTREQ */
#define MCF_GPIO_PPDSDR_PCI_PPDR5	(0x20)	/* PCI_GNT1 /  */
#define MCF_GPIO_PPDSDR_PCI_PPDR6	(0x40)	/* PCI_GNT2 /  */
#define MCF_GPIO_PPDSDR_PCI_PPDR7	(0x80)	/* PCI_GNT3 / ATA_DMACK */

/* Bit definitions and macros for PPDSDR_USB */
#define MCF_GPIO_PPDSDR_USB_PPDR0	(0x01)	/* USB_VBUS_OC / ULPI_STP */
#define MCF_GPIO_PPDSDR_USB_PPDR1	(0x02)	/* USB_VBUS_EN / USB_PULLUP / ULPI_NXT */

/* Bit definitions and macros for PPDSDR_ATAH */
#define MCF_GPIO_PPDSDR_ATAH_PPDR0	(0x01)	/* ATA_DA0 */
#define MCF_GPIO_PPDSDR_ATAH_PPDR1	(0x02)	/* ATA_DA1 */
#define MCF_GPIO_PPDSDR_ATAH_PPDR2	(0x04)	/* ATA_DA2 */
#define MCF_GPIO_PPDSDR_ATAH_PPDR3	(0x08)	/* ATA_CS0 */
#define MCF_GPIO_PPDSDR_ATAH_PPDR4	(0x10)	/* ATA_CS1 */
#define MCF_GPIO_PPDSDR_ATAH_PPDR5	(0x20)	/* ATA_BUFFER_EN */

/* Bit definitions and macros for PPDSDR_ATAL */
#define MCF_GPIO_PPDSDR_ATAL_PPDR0	(0x01)	/* ATA_IORDY */
#define MCF_GPIO_PPDSDR_ATAL_PPDR1	(0x02)	/* ATA_DMARQ */
#define MCF_GPIO_PPDSDR_ATAL_PPDR2	(0x04)	/* ATA_RESET */

/* Bit definitions and macros for PPDSDR_FEC1H */
#define MCF_GPIO_PPDSDR_FEC1H_PPDR0	(0x01)	/* FEC1_CRS / ATA_DATA6 */
#define MCF_GPIO_PPDSDR_FEC1H_PPDR1	(0x02)	/* FEC1_RXD0 / FEC1_RMII_RXD0 / ATA_DATA13 */
#define MCF_GPIO_PPDSDR_FEC1H_PPDR2	(0x04)	/* FEC1_RXDV / FEC1_RMII_CRS_DV / ATA_DATA15 */
#define MCF_GPIO_PPDSDR_FEC1H_PPDR3	(0x08)	/* FEC1_RXCLK / ATA_DATA5 */
#define MCF_GPIO_PPDSDR_FEC1H_PPDR4	(0x10)	/* FEC1_COL / ATA_DATA7 */
#define MCF_GPIO_PPDSDR_FEC1H_PPDR5	(0x20)	/* FEC1_TXD0 / FEC1_RMII_TXD0 / ATA_DATA9 */
#define MCF_GPIO_PPDSDR_FEC1H_PPDR6	(0x40)	/* FEC1_TXEN / FEC1_RMII_TXEN / ATA_DATA8 */
#define MCF_GPIO_PPDSDR_FEC1H_PPDR7	(0x80)	/* FEC1_TXCLK / FEC1_RMII_REF_CLK / ATA_DATA11 */

/* Bit definitions and macros for PPDSDR_FEC1L */
#define MCF_GPIO_PPDSDR_FEC1L_PPDR0	(0x01)	/* FEC1_RXER / FEC1_RMII_RXER / ATA_DATA12 */
#define MCF_GPIO_PPDSDR_FEC1L_PPDR1	(0x02)	/* FEC1_RXD1 / FEC1_RMII_RXD1 / ATA_DATA14 */
#define MCF_GPIO_PPDSDR_FEC1L_PPDR2	(0x04)	/* FEC1_RXD2 / ATA_DATA3 */
#define MCF_GPIO_PPDSDR_FEC1L_PPDR3	(0x08)	/* FEC1_RXD3 / ATA_DATA4 */
#define MCF_GPIO_PPDSDR_FEC1L_PPDR4	(0x10)	/* FEC1_TXER / ATA_DATA0 */
#define MCF_GPIO_PPDSDR_FEC1L_PPDR5	(0x20)	/* FEC1_TXD1 / FEC1_RMII_TXD1 / ATA_DATA10 */
#define MCF_GPIO_PPDSDR_FEC1L_PPDR6	(0x40)	/* FEC1_TXD2 / ATA_DATA1 */
#define MCF_GPIO_PPDSDR_FEC1L_PPDR7	(0x80)	/* FEC1_TXD3 / ATA_DATA2 */

/* Bit definitions and macros for PPDSDR_FBADH */
#define MCF_GPIO_PPDSDR_FBADH_PPDR0	(0x01)	/* FB_AD24 */
#define MCF_GPIO_PPDSDR_FBADH_PPDR1	(0x02)	/* FB_AD25 */
#define MCF_GPIO_PPDSDR_FBADH_PPDR2	(0x04)	/* FB_AD26 */
#define MCF_GPIO_PPDSDR_FBADH_PPDR3	(0x08)	/* FB_AD27 */
#define MCF_GPIO_PPDSDR_FBADH_PPDR4	(0x10)	/* FB_AD28 */
#define MCF_GPIO_PPDSDR_FBADH_PPDR5	(0x20)	/* FB_AD29 */
#define MCF_GPIO_PPDSDR_FBADH_PPDR6	(0x40)	/* FB_AD30 */
#define MCF_GPIO_PPDSDR_FBADH_PPDR7	(0x80)	/* FB_AD31 */

/* Bit definitions and macros for PPDSDR_FBADMH */
#define MCF_GPIO_PPDSDR_FBADMH_PPDR0	(0x01)	/* FB_AD16 */
#define MCF_GPIO_PPDSDR_FBADMH_PPDR1	(0x02)	/* FB_AD17 */
#define MCF_GPIO_PPDSDR_FBADMH_PPDR2	(0x04)	/* FB_AD18 */
#define MCF_GPIO_PPDSDR_FBADMH_PPDR3	(0x08)	/* FB_AD19 */
#define MCF_GPIO_PPDSDR_FBADMH_PPDR4	(0x10)	/* FB_AD20 */
#define MCF_GPIO_PPDSDR_FBADMH_PPDR5	(0x20)	/* FB_AD21 */
#define MCF_GPIO_PPDSDR_FBADMH_PPDR6	(0x40)	/* FB_AD22 */
#define MCF_GPIO_PPDSDR_FBADMH_PPDR7	(0x80)	/* FB_AD23 */

/* Bit definitions and macros for PPDSDR_FBADML */
#define MCF_GPIO_PPDSDR_FBADML_PPDR0	(0x01)	/* FB_AD8 */
#define MCF_GPIO_PPDSDR_FBADML_PPDR1	(0x02)	/* FB_AD9 */
#define MCF_GPIO_PPDSDR_FBADML_PPDR2	(0x04)	/* FB_AD10 */
#define MCF_GPIO_PPDSDR_FBADML_PPDR3	(0x08)	/* FB_AD11 */
#define MCF_GPIO_PPDSDR_FBADML_PPDR4	(0x10)	/* FB_AD12 */
#define MCF_GPIO_PPDSDR_FBADML_PPDR5	(0x20)	/* FB_AD13 */
#define MCF_GPIO_PPDSDR_FBADML_PPDR6	(0x40)	/* FB_AD14 */
#define MCF_GPIO_PPDSDR_FBADML_PPDR7	(0x80)	/* FB_AD15 */

/* Bit definitions and macros for PPDSDR_FBADL */
#define MCF_GPIO_PPDSDR_FBADL_PPDR0	(0x01)	/* FB_AD0 */
#define MCF_GPIO_PPDSDR_FBADL_PPDR1	(0x02)	/* FB_AD1 */
#define MCF_GPIO_PPDSDR_FBADL_PPDR2	(0x04)	/* FB_AD2 */
#define MCF_GPIO_PPDSDR_FBADL_PPDR3	(0x08)	/* FB_AD3 */
#define MCF_GPIO_PPDSDR_FBADL_PPDR4	(0x10)	/* FB_AD4 */
#define MCF_GPIO_PPDSDR_FBADL_PPDR5	(0x20)	/* FB_AD5 */
#define MCF_GPIO_PPDSDR_FBADL_PPDR6	(0x40)	/* FB_AD6 */
#define MCF_GPIO_PPDSDR_FBADL_PPDR7	(0x80)	/* FB_AD7 */

/* Bit definitions and macros for PCLRR_FEC0H */
#define MCF_GPIO_PCLRR_FEC0H_PCLRR0	(0x01)	/* FEC0_CRS / ULPI_DATA6 */
#define MCF_GPIO_PCLRR_FEC0H_PCLRR1	(0x02)	/* FEC0_RXD0 / FEC0_RMII_RXD0 */
#define MCF_GPIO_PCLRR_FEC0H_PCLRR2	(0x04)	/* FEC0_RXDV/FEC0_RMII_CRS_DV */
#define MCF_GPIO_PCLRR_FEC0H_PCLRR3	(0x08)	/* FEC0_RXCLK / ULPI_DATA1 */
#define MCF_GPIO_PCLRR_FEC0H_PCLRR4	(0x10)	/* FEC0_COL / ULPI_DATA7 */
#define MCF_GPIO_PCLRR_FEC0H_PCLRR5	(0x20)	/* FEC0_TXD0 / FEC0_RMII_TXD0 */
#define MCF_GPIO_PCLRR_FEC0H_PCLRR6	(0x40)	/* FEC0_TXEN / FEC0_RMII_TXEN */
#define MCF_GPIO_PCLRR_FEC0H_PCLRR7	(0x80)	/* FEC0_TXCLK / FEC0_RMII_REF_CLK */

/* Bit definitions and macros for PCLRR_FEC0L */
#define MCF_GPIO_PCLRR_FEC0L_PPDR0	(0x01)	/* FEC0_RXER / FEC0_RMII_RXER */
#define MCF_GPIO_PCLRR_FEC0L_PPDR1	(0x02)	/* FEC0_RXD1 / FEC0_RMII_RXD1 */
#define MCF_GPIO_PCLRR_FEC0L_PPDR2	(0x04)	/* FEC0_RXD2 / ULPI_DATA4 */
#define MCF_GPIO_PCLRR_FEC0L_PPDR3	(0x08)	/* FEC0_RXD3 / ULPI_DATA5 */
#define MCF_GPIO_PCLRR_FEC0L_PPDR4	(0x10)	/* FEC0_TXER / ULPI_DATA0 */
#define MCF_GPIO_PCLRR_FEC0L_PPDR5	(0x20)	/* FEC0_TXD1 / FEC0_RMII_TXD1 */
#define MCF_GPIO_PCLRR_FEC0L_PPDR6	(0x40)	/* FEC0_TXD2 / ULPI_DATA2 */
#define MCF_GPIO_PCLRR_FEC0L_PPDR7	(0x80)	/* FEC0_TXD3 / ULPI_DATA3 */

/* Bit definitions and macros for PCLRR_SSI */
#define MCF_GPIO_PCLRR_SSI_PCLRR0	(0x01)	/* SSI_TXD / U1TXD */
#define MCF_GPIO_PCLRR_SSI_PCLRR1	(0x02)	/* SSI_RXD / U1RXD */
#define MCF_GPIO_PCLRR_SSI_PCLRR2	(0x04)	/* SSI_FS / U1RTS */
#define MCF_GPIO_PCLRR_SSI_PCLRR3	(0x08)	/* SSI_BCLK / U1CTS */
#define MCF_GPIO_PCLRR_SSI_PCLRR4	(0x10)	/* SSI_MCLK */

/* Bit definitions and macros for PCLRR_FBCTL */
#define MCF_GPIO_PCLRR_FBCTL_PCLRR0	(0x01)	/* FB_TS / FB_ALE / FB_TBST */
#define MCF_GPIO_PCLRR_FBCTL_PCLRR1	(0x02)	/* FB_RW */
#define MCF_GPIO_PCLRR_FBCTL_PCLRR2	(0x04)	/* FB_TA */
#define MCF_GPIO_PCLRR_FBCTL_PCLRR3	(0x08)	/* FB_OE */

/* Bit definitions and macros for PCLRR_BE */
#define MCF_GPIO_PCLRR_BE_PCLRR0	(0x01)	/* FB_BE/BWE0 / FB_SIZ0 */
#define MCF_GPIO_PCLRR_BE_PCLRR1	(0x02)	/* FB_BE/BWE1 / FB_SIZ1 */
#define MCF_GPIO_PCLRR_BE_PCLRR2	(0x04)	/* FB_BE/BWE2 / FB_SIZ2 */
#define MCF_GPIO_PCLRR_BE_PCLRR3	(0x08)	/* FB_BE/BWE3 / FB_SIZ3 */

/* Bit definitions and macros for PCLRR_CS */
#define MCF_GPIO_PCLRR_CS_PCLRR1	(0x02)	/* FB_CS1 */
#define MCF_GPIO_PCLRR_CS_PCLRR2	(0x04)	/* FB_CS2 */
#define MCF_GPIO_PCLRR_CS_PCLRR3	(0x08)	/* FB_CS3 */

/* Bit definitions and macros for PCLRR_DMA */
#define MCF_GPIO_PCLRR_DMA_PCLRR0	(0x01)	/* DREQ0 */
#define MCF_GPIO_PCLRR_DMA_PCLRR1	(0x02)	/* DACK0 / DSPI_PCS3 */
#define MCF_GPIO_PCLRR_DMA_PCLRR2	(0x04)	/* DREQ1 / USB_CLKIN */
#define MCF_GPIO_PCLRR_DMA_PCLRR3	(0x08)	/* DACK1 / ULPI_DIR */

/* Bit definitions and macros for PCLRR_FECI2C */
#define MCF_GPIO_PCLRR_FECI2C_PCLRR0	(0x01)	/* I2C_SDA / U2RXD */
#define MCF_GPIO_PCLRR_FECI2C_PCLRR1	(0x02)	/* I2C_SCL / U2TXD */
#define MCF_GPIO_PCLRR_FECI2C_PCLRR2	(0x04)	/* FEC0_MDIO */
#define MCF_GPIO_PCLRR_FECI2C_PCLRR3	(0x08)	/* FEC0_MDC */
#define MCF_GPIO_PCLRR_FECI2C_PCLRR4	(0x10)	/* FEC1_MDIO / ATA_DIOW */
#define MCF_GPIO_PCLRR_FECI2C_PCLRR5	(0x20)	/* FEC1_MDC / ATA_DIOR */

/* Bit definitions and macros for PCLRR_UART */
#define MCF_GPIO_PCLRR_UART_PCLRR0	(0x01)	/* U1TXD */
#define MCF_GPIO_PCLRR_UART_PCLRR1	(0x02)	/* U1RXD */
#define MCF_GPIO_PCLRR_UART_PCLRR2	(0x04)	/* U1RTS */
#define MCF_GPIO_PCLRR_UART_PCLRR3	(0x08)	/* U0CTS */
#define MCF_GPIO_PCLRR_UART_PCLRR4	(0x10)	/* U1TXD */
#define MCF_GPIO_PCLRR_UART_PCLRR5	(0x20)	/* U1RXD */
#define MCF_GPIO_PCLRR_UART_PCLRR6	(0x40)	/* U1RTS */
#define MCF_GPIO_PCLRR_UART_PCLRR7	(0x80)	/* U1CTS */

/* Bit definitions and macros for PCLRR_DSPI */
#define MCF_GPIO_PCLRR_DSPI_PCLRR0	(0x01)	/* DSPI_SOUT / SBF_DO */
#define MCF_GPIO_PCLRR_DSPI_PCLRR1	(0x02)	/* DSPI_SIN / SBF_DI */
#define MCF_GPIO_PCLRR_DSPI_PCLRR2	(0x04)	/* DSPI_SCK / SBF_CK */
#define MCF_GPIO_PCLRR_DSPI_PCLRR3	(0x08)	/* DSPI_PCS0/SS */
#define MCF_GPIO_PCLRR_DSPI_PCLRR4	(0x10)	/* DSPI_PCS1 / SBF_CS */
#define MCF_GPIO_PCLRR_DSPI_PCLRR5	(0x20)	/* DSPI_PCS2 */
#define MCF_GPIO_PCLRR_DSPI_PCLRR6	(0x40)	/* DSPI_PCS5/SS */

/* Bit definitions and macros for PCLRR_TIMER */
#define MCF_GPIO_PCLRR_TIMER_PCLRR0	(0x01)	/* T0IN / T0OUT / U2RTS */
#define MCF_GPIO_PCLRR_TIMER_PCLRR1	(0x02)	/* T1IN / T1OUT / U2CTS */
#define MCF_GPIO_PCLRR_TIMER_PCLRR2	(0x04)	/* T2IN / T2OUT / U2TXD */
#define MCF_GPIO_PCLRR_TIMER_PCLRR3	(0x08)	/* T3IN / T3OUT / U2RXD */

/* Bit definitions and macros for PCLRR_PCI */
#define MCF_GPIO_PCLRR_PCI_PCLRR0	(0x01)	/* PCI_REQ0/PCI_EXTGNT */
#define MCF_GPIO_PCLRR_PCI_PCLRR1	(0x02)	/* PCI_REQ1 */
#define MCF_GPIO_PCLRR_PCI_PCLRR2	(0x04)	/* PCI_REQ2 */
#define MCF_GPIO_PCLRR_PCI_PCLRR3	(0x08)	/* PCI_REQ3 / ATA_INTR */
#define MCF_GPIO_PCLRR_PCI_PCLRR4	(0x10)	/* PCI_GNT0/PCI_EXTREQ */
#define MCF_GPIO_PCLRR_PCI_PCLRR5	(0x20)	/* PCI_GNT1 /  */
#define MCF_GPIO_PCLRR_PCI_PCLRR6	(0x40)	/* PCI_GNT2 /  */
#define MCF_GPIO_PCLRR_PCI_PCLRR7	(0x80)	/* PCI_GNT3 / ATA_DMACK */

/* Bit definitions and macros for PCLRR_USB */
#define MCF_GPIO_PCLRR_USB_PCLRR0	(0x01)	/* USB_VBUS_OC / ULPI_STP */
#define MCF_GPIO_PCLRR_USB_PCLRR1	(0x02)	/* USB_VBUS_EN / USB_PULLUP / ULPI_NXT */

/* Bit definitions and macros for PCLRR_ATAH */
#define MCF_GPIO_PCLRR_ATAH_PCLRR0	(0x01)	/* ATA_DA0 */
#define MCF_GPIO_PCLRR_ATAH_PCLRR1	(0x02)	/* ATA_DA1 */
#define MCF_GPIO_PCLRR_ATAH_PCLRR2	(0x04)	/* ATA_DA2 */
#define MCF_GPIO_PCLRR_ATAH_PCLRR3	(0x08)	/* ATA_CS0 */
#define MCF_GPIO_PCLRR_ATAH_PCLRR4	(0x10)	/* ATA_CS1 */
#define MCF_GPIO_PCLRR_ATAH_PCLRR5	(0x20)	/* ATA_BUFFER_EN */

/* Bit definitions and macros for PCLRR_ATAL */
#define MCF_GPIO_PCLRR_ATAL_PCLRR0	(0x01)	/* ATA_IORDY */
#define MCF_GPIO_PCLRR_ATAL_PCLRR1	(0x02)	/* ATA_DMARQ */
#define MCF_GPIO_PCLRR_ATAL_PCLRR2	(0x04)	/* ATA_RESET */

/* Bit definitions and macros for PCLRR_FEC1H */
#define MCF_GPIO_PCLRR_FEC1H_PCLRR0	(0x01)	/* FEC1_CRS / ATA_DATA6 */
#define MCF_GPIO_PCLRR_FEC1H_PCLRR1	(0x02)	/* FEC1_RXD0 / FEC1_RMII_RXD0 / ATA_DATA13 */
#define MCF_GPIO_PCLRR_FEC1H_PCLRR2	(0x04)	/* FEC1_RXDV / FEC1_RMII_CRS_DV / ATA_DATA15 */
#define MCF_GPIO_PCLRR_FEC1H_PCLRR3	(0x08)	/* FEC1_RXCLK / ATA_DATA5 */
#define MCF_GPIO_PCLRR_FEC1H_PCLRR4	(0x10)	/* FEC1_COL / ATA_DATA7 */
#define MCF_GPIO_PCLRR_FEC1H_PCLRR5	(0x20)	/* FEC1_TXD0 / FEC1_RMII_TXD0 / ATA_DATA9 */
#define MCF_GPIO_PCLRR_FEC1H_PCLRR6	(0x40)	/* FEC1_TXEN / FEC1_RMII_TXEN / ATA_DATA8 */
#define MCF_GPIO_PCLRR_FEC1H_PCLRR7	(0x80)	/* FEC1_TXCLK / FEC1_RMII_REF_CLK / ATA_DATA11 */

/* Bit definitions and macros for PCLRR_FEC1L */
#define MCF_GPIO_PCLRR_FEC1L_PCLRR0	(0x01)	/* FEC1_RXER / FEC1_RMII_RXER / ATA_DATA12 */
#define MCF_GPIO_PCLRR_FEC1L_PCLRR1	(0x02)	/* FEC1_RXD1 / FEC1_RMII_RXD1 / ATA_DATA14 */
#define MCF_GPIO_PCLRR_FEC1L_PCLRR2	(0x04)	/* FEC1_RXD2 / ATA_DATA3 */
#define MCF_GPIO_PCLRR_FEC1L_PCLRR3	(0x08)	/* FEC1_RXD3 / ATA_DATA4 */
#define MCF_GPIO_PCLRR_FEC1L_PCLRR4	(0x10)	/* FEC1_TXER / ATA_DATA0 */
#define MCF_GPIO_PCLRR_FEC1L_PCLRR5	(0x20)	/* FEC1_TXD1 / FEC1_RMII_TXD1 / ATA_DATA10 */
#define MCF_GPIO_PCLRR_FEC1L_PCLRR6	(0x40)	/* FEC1_TXD2 / ATA_DATA1 */
#define MCF_GPIO_PCLRR_FEC1L_PCLRR7	(0x80)	/* FEC1_TXD3 / ATA_DATA2 */

/* Bit definitions and macros for PCLRR_FBADH */
#define MCF_GPIO_PCLRR_FBADH_PCLRR0	(0x01)	/* FB_AD24 */
#define MCF_GPIO_PCLRR_FBADH_PCLRR1	(0x02)	/* FB_AD25 */
#define MCF_GPIO_PCLRR_FBADH_PCLRR2	(0x04)	/* FB_AD26 */
#define MCF_GPIO_PCLRR_FBADH_PCLRR3	(0x08)	/* FB_AD27 */
#define MCF_GPIO_PCLRR_FBADH_PCLRR4	(0x10)	/* FB_AD28 */
#define MCF_GPIO_PCLRR_FBADH_PCLRR5	(0x20)	/* FB_AD29 */
#define MCF_GPIO_PCLRR_FBADH_PCLRR6	(0x40)	/* FB_AD30 */
#define MCF_GPIO_PCLRR_FBADH_PCLRR7	(0x80)	/* FB_AD31 */

/* Bit definitions and macros for PCLRR_FBADMH */
#define MCF_GPIO_PCLRR_FBADMH_PCLRR0	(0x01)	/* FB_AD16 */
#define MCF_GPIO_PCLRR_FBADMH_PCLRR1	(0x02)	/* FB_AD17 */
#define MCF_GPIO_PCLRR_FBADMH_PCLRR2	(0x04)	/* FB_AD18 */
#define MCF_GPIO_PCLRR_FBADMH_PCLRR3	(0x08)	/* FB_AD19 */
#define MCF_GPIO_PCLRR_FBADMH_PCLRR4	(0x10)	/* FB_AD20 */
#define MCF_GPIO_PCLRR_FBADMH_PCLRR5	(0x20)	/* FB_AD21 */
#define MCF_GPIO_PCLRR_FBADMH_PCLRR6	(0x40)	/* FB_AD22 */
#define MCF_GPIO_PCLRR_FBADMH_PCLRR7	(0x80)	/* FB_AD23 */

/* Bit definitions and macros for PCLRR_FBADML */
#define MCF_GPIO_PCLRR_FBADML_PCLRR0	(0x01)	/* FB_AD8 */
#define MCF_GPIO_PCLRR_FBADML_PCLRR1	(0x02)	/* FB_AD9 */
#define MCF_GPIO_PCLRR_FBADML_PCLRR2	(0x04)	/* FB_AD10 */
#define MCF_GPIO_PCLRR_FBADML_PCLRR3	(0x08)	/* FB_AD11 */
#define MCF_GPIO_PCLRR_FBADML_PCLRR4	(0x10)	/* FB_AD12 */
#define MCF_GPIO_PCLRR_FBADML_PCLRR5	(0x20)	/* FB_AD13 */
#define MCF_GPIO_PCLRR_FBADML_PCLRR6	(0x40)	/* FB_AD14 */
#define MCF_GPIO_PCLRR_FBADML_PCLRR7	(0x80)	/* FB_AD15 */

/* Bit definitions and macros for PCLRR_FBADL */
#define MCF_GPIO_PCLRR_FBADL_PCLRR0	(0x01)	/* FB_AD0 */
#define MCF_GPIO_PCLRR_FBADL_PCLRR1	(0x02)	/* FB_AD1 */
#define MCF_GPIO_PCLRR_FBADL_PCLRR2	(0x04)	/* FB_AD2 */
#define MCF_GPIO_PCLRR_FBADL_PCLRR3	(0x08)	/* FB_AD3 */
#define MCF_GPIO_PCLRR_FBADL_PCLRR4	(0x10)	/* FB_AD4 */
#define MCF_GPIO_PCLRR_FBADL_PCLRR5	(0x20)	/* FB_AD5 */
#define MCF_GPIO_PCLRR_FBADL_PCLRR6	(0x40)	/* FB_AD6 */
#define MCF_GPIO_PCLRR_FBADL_PCLRR7	(0x80)	/* FB_AD7 */

/* Bit definitions and macros for PAR_FEC */
#define MCF_GPIO_PAR_FEC_FEC0(x)	(((x)&0x07))
#define MCF_GPIO_PAR_FEC_FEC1(x)	(((x)&0x07)<<4)
#define MCF_GPIO_PAR_FEC_FEC1_MASK	(0x8F)
#define MCF_GPIO_PAR_FEC_FEC1_MII	(0x70)
#define MCF_GPIO_PAR_FEC_FEC1_RMII_GPIO	(0x30)
#define MCF_GPIO_PAR_FEC_FEC1_RMII_ATA	(0x20)
#define MCF_GPIO_PAR_FEC_FEC1_ATA	(0x10)
#define MCF_GPIO_PAR_FEC_FEC1_GPIO	(0x00)
#define MCF_GPIO_PAR_FEC_FEC0_MASK	(0xF8)
#define MCF_GPIO_PAR_FEC_FEC0_MII	(0x07)
#define MCF_GPIO_PAR_FEC_FEC0_RMII_GPIO	(0x03)
#define MCF_GPIO_PAR_FEC_FEC0_RMII_ULPI	(0x02)
#define MCF_GPIO_PAR_FEC_FEC0_ULPI	(0x01)
#define MCF_GPIO_PAR_FEC_FEC0_GPIO	(0x00)

/* Bit definitions and macros for PAR_DMA */
#define MCF_GPIO_PAR_DMA_DREQ0		(0x01)
#define MCF_GPIO_PAR_DMA_DACK0(x)	(((x)&0x03)<<2)
#define MCF_GPIO_PAR_DMA_DREQ1(x)	(((x)&0x03)<<4)
#define MCF_GPIO_PAR_DMA_DACK1(x)	(((x)&0x03)<<6)
#define MCF_GPIO_PAR_DMA_DACK1_MASK	(0x3F)
#define MCF_GPIO_PAR_DMA_DACK1_DACK1	(0xC0)
#define MCF_GPIO_PAR_DMA_DACK1_ULPI_DIR	(0x40)
#define MCF_GPIO_PAR_DMA_DACK1_GPIO	(0x00)
#define MCF_GPIO_PAR_DMA_DREQ1_MASK	(0xCF)
#define MCF_GPIO_PAR_DMA_DREQ1_DREQ1	(0x30)
#define MCF_GPIO_PAR_DMA_DREQ1_USB_CLKIN (0x10)
#define MCF_GPIO_PAR_DMA_DREQ1_GPIO	(0x00)
#define MCF_GPIO_PAR_DMA_DACK0_MASK	(0xF3)
#define MCF_GPIO_PAR_DMA_DACK0_DACK1	(0x0C)
#define MCF_GPIO_PAR_DMA_DACK0_ULPI_DIR	(0x04)
#define MCF_GPIO_PAR_DMA_DACK0_GPIO	(0x00)
#define MCF_GPIO_PAR_DMA_DREQ0_DREQ0	(0x01)
#define MCF_GPIO_PAR_DMA_DREQ0_GPIO	(0x00)

/* Bit definitions and macros for PAR_FBCTL */
#define MCF_GPIO_PAR_FBCTL_TS(x)	(((x)&0x03)<<3)
#define MCF_GPIO_PAR_FBCTL_RW		(0x20)
#define MCF_GPIO_PAR_FBCTL_TA		(0x40)
#define MCF_GPIO_PAR_FBCTL_OE		(0x80)
#define MCF_GPIO_PAR_FBCTL_OE_OE	(0x80)
#define MCF_GPIO_PAR_FBCTL_OE_GPIO	(0x00)
#define MCF_GPIO_PAR_FBCTL_TA_TA	(0x40)
#define MCF_GPIO_PAR_FBCTL_TA_GPIO	(0x00)
#define MCF_GPIO_PAR_FBCTL_RW_RW	(0x20)
#define MCF_GPIO_PAR_FBCTL_RW_GPIO	(0x00)
#define MCF_GPIO_PAR_FBCTL_TS_MASK	(0xE7)
#define MCF_GPIO_PAR_FBCTL_TS_TS	(0x18)
#define MCF_GPIO_PAR_FBCTL_TS_ALE	(0x10)
#define MCF_GPIO_PAR_FBCTL_TS_TBST	(0x08)
#define MCF_GPIO_PAR_FBCTL_TS_GPIO	(0x80)

/* Bit definitions and macros for PAR_DSPI */
#define MCF_GPIO_PAR_DSPI_SCK		(0x01)
#define MCF_GPIO_PAR_DSPI_SOUT		(0x02)
#define MCF_GPIO_PAR_DSPI_SIN		(0x04)
#define MCF_GPIO_PAR_DSPI_PCS0		(0x08)
#define MCF_GPIO_PAR_DSPI_PCS1		(0x10)
#define MCF_GPIO_PAR_DSPI_PCS2		(0x20)
#define MCF_GPIO_PAR_DSPI_PCS5		(0x40)
#define MCF_GPIO_PAR_DSPI_PCS5_PCS5	(0x40)
#define MCF_GPIO_PAR_DSPI_PCS5_GPIO	(0x00)
#define MCF_GPIO_PAR_DSPI_PCS2_PCS2	(0x20)
#define MCF_GPIO_PAR_DSPI_PCS2_GPIO	(0x00)
#define MCF_GPIO_PAR_DSPI_PCS1_PCS1	(0x10)
#define MCF_GPIO_PAR_DSPI_PCS1_GPIO	(0x00)
#define MCF_GPIO_PAR_DSPI_PCS0_PCS0	(0x08)
#define MCF_GPIO_PAR_DSPI_PCS0_GPIO	(0x00)
#define MCF_GPIO_PAR_DSPI_SIN_SIN	(0x04)
#define MCF_GPIO_PAR_DSPI_SIN_GPIO	(0x00)
#define MCF_GPIO_PAR_DSPI_SOUT_SOUT	(0x02)
#define MCF_GPIO_PAR_DSPI_SOUT_GPIO	(0x00)
#define MCF_GPIO_PAR_DSPI_SCK_SCK	(0x01)
#define MCF_GPIO_PAR_DSPI_SCK_GPIO	(0x00)

/* Bit definitions and macros for PAR_BE */
#define MCF_GPIO_PAR_BE_BE0		(0x01)
#define MCF_GPIO_PAR_BE_BE1		(0x04)
#define MCF_GPIO_PAR_BE_BE2(x)		(((x)&0x03)<<4)
#define MCF_GPIO_PAR_BE_BE3(x)		(((x)&0x03)<<6)
#define MCF_GPIO_PAR_BE_BE3_MASK	(0x3F)
#define MCF_GPIO_PAR_BE_BE3_BE3		(0xC0)
#define MCF_GPIO_PAR_BE_BE3_TSIZ1	(0x80)
#define MCF_GPIO_PAR_BE_BE3_GPIO	(0x00)
#define MCF_GPIO_PAR_BE_BE2_MASK	(0xCF)
#define MCF_GPIO_PAR_BE_BE2_BE2		(0x30)
#define MCF_GPIO_PAR_BE_BE2_TSIZ0	(0x20)
#define MCF_GPIO_PAR_BE_BE2_GPIO	(0x00)
#define MCF_GPIO_PAR_BE_BE1_BE1		(0x04)
#define MCF_GPIO_PAR_BE_BE1_GPIO	(0x00)
#define MCF_GPIO_PAR_BE_BE0_BE0		(0x01)
#define MCF_GPIO_PAR_BE_BE0_GPIO	(0x00)

/* Bit definitions and macros for PAR_CS */
#define MCF_GPIO_PAR_CS_CS1		(0x02)
#define MCF_GPIO_PAR_CS_CS2		(0x04)
#define MCF_GPIO_PAR_CS_CS3		(0x08)
#define MCF_GPIO_PAR_CS_CS3_CS3		(0x08)
#define MCF_GPIO_PAR_CS_CS3_GPIO	(0x00)
#define MCF_GPIO_PAR_CS_CS2_CS2		(0x04)
#define MCF_GPIO_PAR_CS_CS2_GPIO	(0x00)
#define MCF_GPIO_PAR_CS_CS1_CS1		(0x02)
#define MCF_GPIO_PAR_CS_CS1_GPIO	(0x00)

/* Bit definitions and macros for PAR_TIMER */
#define MCF_GPIO_PAR_TIMER_T0IN(x)	(((x)&0x03))
#define MCF_GPIO_PAR_TIMER_T1IN(x)	(((x)&0x03)<<2)
#define MCF_GPIO_PAR_TIMER_T2IN(x)	(((x)&0x03)<<4)
#define MCF_GPIO_PAR_TIMER_T3IN(x)	(((x)&0x03)<<6)
#define MCF_GPIO_PAR_TIMER_T3IN_MASK	(0x3F)
#define MCF_GPIO_PAR_TIMER_T3IN_T3IN	(0xC0)
#define MCF_GPIO_PAR_TIMER_T3IN_T3OUT	(0x80)
#define MCF_GPIO_PAR_TIMER_T3IN_U2RXD	(0x40)
#define MCF_GPIO_PAR_TIMER_T3IN_GPIO	(0x00)
#define MCF_GPIO_PAR_TIMER_T2IN_MASK	(0xCF)
#define MCF_GPIO_PAR_TIMER_T2IN_T2IN	(0x30)
#define MCF_GPIO_PAR_TIMER_T2IN_T2OUT	(0x20)
#define MCF_GPIO_PAR_TIMER_T2IN_U2TXD	(0x10)
#define MCF_GPIO_PAR_TIMER_T2IN_GPIO	(0x00)
#define MCF_GPIO_PAR_TIMER_T1IN_MASK	(0xF3)
#define MCF_GPIO_PAR_TIMER_T1IN_T1IN	(0x0C)
#define MCF_GPIO_PAR_TIMER_T1IN_T1OUT	(0x08)
#define MCF_GPIO_PAR_TIMER_T1IN_U2CTS	(0x04)
#define MCF_GPIO_PAR_TIMER_T1IN_GPIO	(0x00)
#define MCF_GPIO_PAR_TIMER_T0IN_MASK	(0xFC)
#define MCF_GPIO_PAR_TIMER_T0IN_T0IN	(0x03)
#define MCF_GPIO_PAR_TIMER_T0IN_T0OUT	(0x02)
#define MCF_GPIO_PAR_TIMER_T0IN_U2RTS	(0x01)
#define MCF_GPIO_PAR_TIMER_T0IN_GPIO	(0x00)

/* Bit definitions and macros for PAR_USB */
#define MCF_GPIO_PAR_USB_VBUSOC(x)		(((x)&0x03))
#define MCF_GPIO_PAR_USB_VBUSEN(x)		(((x)&0x03)<<2)
#define MCF_GPIO_PAR_USB_VBUSEN_MASK		(0xF3)
#define MCF_GPIO_PAR_USB_VBUSEN_VBUSEN		(0x0C)
#define MCF_GPIO_PAR_USB_VBUSEN_USBPULLUP	(0x08)
#define MCF_GPIO_PAR_USB_VBUSEN_ULPI_NXT	(0x04)
#define MCF_GPIO_PAR_USB_VBUSEN_GPIO		(0x00)
#define MCF_GPIO_PAR_USB_VBUSOC_MASK		(0xFC)
#define MCF_GPIO_PAR_USB_VBUSOC_VBUSOC		(0x03)
#define MCF_GPIO_PAR_USB_VBUSOC_ULPI_STP	(0x01)
#define MCF_GPIO_PAR_USB_VBUSOC_GPIO		(0x00)

/* Bit definitions and macros for PAR_UART */
#define MCF_GPIO_PAR_UART_U0TXD		(0x01)
#define MCF_GPIO_PAR_UART_U0RXD		(0x02)
#define MCF_GPIO_PAR_UART_U0RTS		(0x04)
#define MCF_GPIO_PAR_UART_U0CTS		(0x08)
#define MCF_GPIO_PAR_UART_U1TXD		(0x10)
#define MCF_GPIO_PAR_UART_U1RXD		(0x20)
#define MCF_GPIO_PAR_UART_U1RTS		(0x40)
#define MCF_GPIO_PAR_UART_U1CTS		(0x80)
#define MCF_GPIO_PAR_UART_U1CTS_U1CTS	(0x80)
#define MCF_GPIO_PAR_UART_U1CTS_GPIO	(0x00)
#define MCF_GPIO_PAR_UART_U1RTS_U1RTS	(0x40)
#define MCF_GPIO_PAR_UART_U1RTS_GPIO	(0x00)
#define MCF_GPIO_PAR_UART_U1RXD_U1RXD	(0x20)
#define MCF_GPIO_PAR_UART_U1RXD_GPIO	(0x00)
#define MCF_GPIO_PAR_UART_U1TXD_U1TXD	(0x10)
#define MCF_GPIO_PAR_UART_U1TXD_GPIO	(0x00)
#define MCF_GPIO_PAR_UART_U0CTS_U0CTS	(0x08)
#define MCF_GPIO_PAR_UART_U0CTS_GPIO	(0x00)
#define MCF_GPIO_PAR_UART_U0RTS_U0RTS	(0x04)
#define MCF_GPIO_PAR_UART_U0RTS_GPIO	(0x00)
#define MCF_GPIO_PAR_UART_U0RXD_U0RXD	(0x02)
#define MCF_GPIO_PAR_UART_U0RXD_GPIO	(0x00)
#define MCF_GPIO_PAR_UART_U0TXD_U0TXD	(0x01)
#define MCF_GPIO_PAR_UART_U0TXD_GPIO	(0x00)

/* Bit definitions and macros for PAR_FECI2C */
#define MCF_GPIO_PAR_FECI2C_SDA(x)	(((x)&0x0003))
#define MCF_GPIO_PAR_FECI2C_SCL(x)	(((x)&0x0003)<<2)
#define MCF_GPIO_PAR_FECI2C_PAR_SDA(x)	(((x)&0x0003))
#define MCF_GPIO_PAR_FECI2C_PAR_SCL(x)	(((x)&0x0003)<<2)
#define MCF_GPIO_PAR_FECI2C_MDIO0	(0x0010)
#define MCF_GPIO_PAR_FECI2C_MDC0	(0x0040)
#define MCF_GPIO_PAR_FECI2C_MDIO1(x)	(((x)&0x0003)<<8)
#define MCF_GPIO_PAR_FECI2C_MDC1(x)	(((x)&0x0003)<<10)
#define MCF_GPIO_PAR_FECI2C_MDC1_MASK	(0xF3FF)
#define MCF_GPIO_PAR_FECI2C_MDC1_MDC1	(0x0C00)
#define MCF_GPIO_PAR_FECI2C_MDC1_ATA_DIOR (0x0800)
#define MCF_GPIO_PAR_FECI2C_MDC1_GPIO	(0x0000)
#define MCF_GPIO_PAR_FECI2C_MDIO1_MASK	(0xFCFF)
#define MCF_GPIO_PAR_FECI2C_MDIO1_MDIO1	(0x0300)
#define MCF_GPIO_PAR_FECI2C_MDIO1_ATA_DIOW (0x0200)
#define MCF_GPIO_PAR_FECI2C_MDIO1_GPIO	(0x0000)
#define MCF_GPIO_PAR_FECI2C_MDC0_MDC0	(0x0040)
#define MCF_GPIO_PAR_FECI2C_MDC0_GPIO	(0x0000)
#define MCF_GPIO_PAR_FECI2C_MDIO0_MDIO0	(0x0010)
#define MCF_GPIO_PAR_FECI2C_MDIO0_GPIO	(0x0000)
#define MCF_GPIO_PAR_FECI2C_SCL_MASK	(0xFFF3)
#define MCF_GPIO_PAR_FECI2C_SCL_SCL	(0x000C)
#define MCF_GPIO_PAR_FECI2C_SCL_U2TXD	(0x0004)
#define MCF_GPIO_PAR_FECI2C_SCL_GPIO	(0x0000)
#define MCF_GPIO_PAR_FECI2C_SDA_MASK	(0xFFFC)
#define MCF_GPIO_PAR_FECI2C_SDA_SDA	(0x0003)
#define MCF_GPIO_PAR_FECI2C_SDA_U2RXD	(0x0001)
#define MCF_GPIO_PAR_FECI2C_SDA_GPIO	(0x0000)

/* Bit definitions and macros for PAR_SSI */
#define MCF_GPIO_PAR_SSI_MCLK		(0x0001)
#define MCF_GPIO_PAR_SSI_STXD(x)	(((x)&0x0003)<<2)
#define MCF_GPIO_PAR_SSI_SRXD(x)	(((x)&0x0003)<<4)
#define MCF_GPIO_PAR_SSI_FS(x)		(((x)&0x0003)<<6)
#define MCF_GPIO_PAR_SSI_BCLK(x)	(((x)&0x0003)<<8)
#define MCF_GPIO_PAR_SSI_BCLK_MASK	(0xFCFF)
#define MCF_GPIO_PAR_SSI_BCLK_BCLK	(0x0300)
#define MCF_GPIO_PAR_SSI_BCLK_U1CTS	(0x0200)
#define MCF_GPIO_PAR_SSI_BCLK_GPIO	(0x0000)
#define MCF_GPIO_PAR_SSI_FS_MASK	(0xFF3F)
#define MCF_GPIO_PAR_SSI_FS_FS		(0x00C0)
#define MCF_GPIO_PAR_SSI_FS_U1RTS	(0x0080)
#define MCF_GPIO_PAR_SSI_FS_GPIO	(0x0000)
#define MCF_GPIO_PAR_SSI_SRXD_MASK	(0xFFCF)
#define MCF_GPIO_PAR_SSI_SRXD_SRXD	(0x0030)
#define MCF_GPIO_PAR_SSI_SRXD_U1RXD	(0x0020)
#define MCF_GPIO_PAR_SSI_SRXD_GPIO	(0x0000)
#define MCF_GPIO_PAR_SSI_STXD_MASK	(0xFFF3)
#define MCF_GPIO_PAR_SSI_STXD_STXD	(0x000C)
#define MCF_GPIO_PAR_SSI_STXD_U1TXD	(0x0008)
#define MCF_GPIO_PAR_SSI_STXD_GPIO	(0x0000)
#define MCF_GPIO_PAR_SSI_MCLK_MCLK	(0x0001)
#define MCF_GPIO_PAR_SSI_MCLK_GPIO	(0x0000)

/* Bit definitions and macros for PAR_ATA */
#define MCF_GPIO_PAR_ATA_IORDY		(0x0001)
#define MCF_GPIO_PAR_ATA_DMARQ		(0x0002)
#define MCF_GPIO_PAR_ATA_RESET		(0x0004)
#define MCF_GPIO_PAR_ATA_DA0		(0x0020)
#define MCF_GPIO_PAR_ATA_DA1		(0x0040)
#define MCF_GPIO_PAR_ATA_DA2		(0x0080)
#define MCF_GPIO_PAR_ATA_CS0		(0x0100)
#define MCF_GPIO_PAR_ATA_CS1		(0x0200)
#define MCF_GPIO_PAR_ATA_BUFEN		(0x0400)
#define MCF_GPIO_PAR_ATA_BUFEN_BUFEN	(0x0400)
#define MCF_GPIO_PAR_ATA_BUFEN_GPIO	(0x0000)
#define MCF_GPIO_PAR_ATA_CS1_CS1	(0x0200)
#define MCF_GPIO_PAR_ATA_CS1_GPIO	(0x0000)
#define MCF_GPIO_PAR_ATA_CS0_CS0	(0x0100)
#define MCF_GPIO_PAR_ATA_CS0_GPIO	(0x0000)
#define MCF_GPIO_PAR_ATA_DA2_DA2	(0x0080)
#define MCF_GPIO_PAR_ATA_DA2_GPIO	(0x0000)
#define MCF_GPIO_PAR_ATA_DA1_DA1	(0x0040)
#define MCF_GPIO_PAR_ATA_DA1_GPIO	(0x0000)
#define MCF_GPIO_PAR_ATA_DA0_DA0	(0x0020)
#define MCF_GPIO_PAR_ATA_DA0_GPIO	(0x0000)
#define MCF_GPIO_PAR_ATA_RESET_RESET	(0x0004)
#define MCF_GPIO_PAR_ATA_RESET_GPIO	(0x0000)
#define MCF_GPIO_PAR_ATA_DMARQ_DMARQ	(0x0002)
#define MCF_GPIO_PAR_ATA_DMARQ_GPIO	(0x0000)
#define MCF_GPIO_PAR_ATA_IORDY_IORDY	(0x0001)
#define MCF_GPIO_PAR_ATA_IORDY_GPIO	(0x0000)

/* Bit definitions and macros for PAR_IRQ */
#define MCF_GPIO_PAR_IRQ_IRQ1		(0x02)
#define MCF_GPIO_PAR_IRQ_IRQ4		(0x10)
#define MCF_GPIO_PAR_IRQ_IRQ4_IRQ4	(0x10)
#define MCF_GPIO_PAR_IRQ_IRQ4_GPIO	(0x00)
#define MCF_GPIO_PAR_IRQ_IRQ1_IRQ1	(0x02)
#define MCF_GPIO_PAR_IRQ_IRQ1_GPIO	(0x00)

/* Bit definitions and macros for PAR_PCI */
#define MCF_GPIO_PAR_PCI_REQ0		(0x0001)
#define MCF_GPIO_PAR_PCI_REQ1		(0x0004)
#define MCF_GPIO_PAR_PCI_REQ2		(0x0010)
#define MCF_GPIO_PAR_PCI_REQ3(x)	(((x)&0x0003)<<6)
#define MCF_GPIO_PAR_PCI_GNT0		(0x0100)
#define MCF_GPIO_PAR_PCI_GNT1		(0x0400)
#define MCF_GPIO_PAR_PCI_GNT2		(0x1000)
#define MCF_GPIO_PAR_PCI_GNT3(x)	(((x)&0x0003)<<14)
#define MCF_GPIO_PAR_PCI_GNT3_MASK	(0x3FFF)
#define MCF_GPIO_PAR_PCI_GNT3_GNT3	(0xC000)
#define MCF_GPIO_PAR_PCI_GNT3_ATA_DMACK	(0x8000)
#define MCF_GPIO_PAR_PCI_GNT3_GPIO	(0x0000)
#define MCF_GPIO_PAR_PCI_GNT2_GNT2	(0x1000)
#define MCF_GPIO_PAR_PCI_GNT2_GPIO	(0x0000)
#define MCF_GPIO_PAR_PCI_GNT1_GNT1	(0x0400)
#define MCF_GPIO_PAR_PCI_GNT1_GPIO	(0x0000)
#define MCF_GPIO_PAR_PCI_GNT0_GNT0	(0x0100)
#define MCF_GPIO_PAR_PCI_GNT0_GPIO	(0x0000)
#define MCF_GPIO_PAR_PCI_REQ3_MASK	(0xFF3F)
#define MCF_GPIO_PAR_PCI_REQ3_REQ3	(0x00C0)
#define MCF_GPIO_PAR_PCI_REQ3_ATA_INTRQ	(0x0080)
#define MCF_GPIO_PAR_PCI_REQ3_GPIO	(0x0000)
#define MCF_GPIO_PAR_PCI_REQ2_REQ2	(0x0010)
#define MCF_GPIO_PAR_PCI_REQ2_GPIO	(0x0000)
#define MCF_GPIO_PAR_PCI_REQ1_REQ1	(0x0040)
#define MCF_GPIO_PAR_PCI_REQ1_GPIO	(0x0000)
#define MCF_GPIO_PAR_PCI_REQ0_REQ0	(0x0001)
#define MCF_GPIO_PAR_PCI_REQ0_GPIO	(0x0000)

/* Bit definitions and macros for MSCR_SDRAM */
#define MCF_GPIO_MSCR_SDRAM_SDCTL(x)		(((x)&0x03))
#define MCF_GPIO_MSCR_SDRAM_SDCLK(x)		(((x)&0x03)<<2)
#define MCF_GPIO_MSCR_SDRAM_SDDQS(x)		(((x)&0x03)<<4)
#define MCF_GPIO_MSCR_SDRAM_SDDATA(x)		(((x)&0x03)<<6)
#define MCF_GPIO_MSCR_SDRAM_SDDATA_MASK		(0x3F)
#define MCF_GPIO_MSCR_SDRAM_SDDATA_DDR1		(0xC0)
#define MCF_GPIO_MSCR_SDRAM_SDDATA_DDR2		(0x80)
#define MCF_GPIO_MSCR_SDRAM_SDDATA_FS_LPDDR	(0x40)
#define MCF_GPIO_MSCR_SDRAM_SDDATA_HS_LPDDR	(0x00)
#define MCF_GPIO_MSCR_SDRAM_SDDQS_MASK		(0xCF)
#define MCF_GPIO_MSCR_SDRAM_SDDQS_DDR1		(0x30)
#define MCF_GPIO_MSCR_SDRAM_SDDQS_DDR2		(0x20)
#define MCF_GPIO_MSCR_SDRAM_SDDQS_FS_LPDDR	(0x10)
#define MCF_GPIO_MSCR_SDRAM_SDDQS_HS_LPDDR	(0x00)
#define MCF_GPIO_MSCR_SDRAM_SDCLK_MASK		(0xF3)
#define MCF_GPIO_MSCR_SDRAM_SDCLK_DDR1		(0x0C)
#define MCF_GPIO_MSCR_SDRAM_SDCLK_DDR2		(0x08)
#define MCF_GPIO_MSCR_SDRAM_SDCLK_FS_LPDDR	(0x04)
#define MCF_GPIO_MSCR_SDRAM_SDCLK_HS_LPDDR	(0x00)
#define MCF_GPIO_MSCR_SDRAM_SDCTL_MASK		(0xFC)
#define MCF_GPIO_MSCR_SDRAM_SDCTL_DDR1		(0x03)
#define MCF_GPIO_MSCR_SDRAM_SDCTL_DDR2		(0x02)
#define MCF_GPIO_MSCR_SDRAM_SDCTL_FS_LPDDR	(0x01)
#define MCF_GPIO_MSCR_SDRAM_SDCTL_HS_LPDDR	(0x00)

/* Bit definitions and macros for MSCR_PCI */
#define MCF_GPIO_MSCR_PCI_PCI		(0x01)
#define MCF_GPIO_MSCR_PCI_PCI_HI_66MHZ	(0x01)
#define MCF_GPIO_MSCR_PCI_PCI_LO_33MHZ	(0x00)

/* Bit definitions and macros for DSCR_I2C */
#define MCF_GPIO_DSCR_I2C_I2C(x)	(((x)&0x03))
#define MCF_GPIO_DSCR_I2C_I2C_LOAD_50PF	(0x03)
#define MCF_GPIO_DSCR_I2C_I2C_LOAD_30PF	(0x02)
#define MCF_GPIO_DSCR_I2C_I2C_LOAD_20PF	(0x01)
#define MCF_GPIO_DSCR_I2C_I2C_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_FLEXBUS */
#define MCF_GPIO_DSCR_FLEXBUS_FBADL(x)		(((x)&0x03))
#define MCF_GPIO_DSCR_FLEXBUS_FBADH(x)		(((x)&0x03)<<2)
#define MCF_GPIO_DSCR_FLEXBUS_FBCTL(x)		(((x)&0x03)<<4)
#define MCF_GPIO_DSCR_FLEXBUS_FBCLK(x)		(((x)&0x03)<<6)
#define MCF_GPIO_DSCR_FLEXBUS_FBCLK_LOAD_50PF	(0xC0)
#define MCF_GPIO_DSCR_FLEXBUS_FBCLK_LOAD_30P	(0x80)
#define MCF_GPIO_DSCR_FLEXBUS_FBCLK_LOAD_20PF	(0x40)
#define MCF_GPIO_DSCR_FLEXBUS_FBCLK_LOAD_10PF	(0x00)
#define MCF_GPIO_DSCR_FLEXBUS_FBCTL_LOAD_50PF	(0x30)
#define MCF_GPIO_DSCR_FLEXBUS_FBCTL_LOAD_30PF	(0x20)
#define MCF_GPIO_DSCR_FLEXBUS_FBCTL_LOAD_20PF	(0x10)
#define MCF_GPIO_DSCR_FLEXBUS_FBCTL_LOAD_10PF	(0x00)
#define MCF_GPIO_DSCR_FLEXBUS_FBADH_LOAD_50PF	(0x0C)
#define MCF_GPIO_DSCR_FLEXBUS_FBADH_LOAD_30PF	(0x08)
#define MCF_GPIO_DSCR_FLEXBUS_FBADH_LOAD_20PF	(0x04)
#define MCF_GPIO_DSCR_FLEXBUS_FBADH_LOAD_10PF	(0x00)
#define MCF_GPIO_DSCR_FLEXBUS_FBADL_LOAD_50PF	(0x03)
#define MCF_GPIO_DSCR_FLEXBUS_FBADL_LOAD_30PF	(0x02)
#define MCF_GPIO_DSCR_FLEXBUS_FBADL_LOAD_20PF	(0x01)
#define MCF_GPIO_DSCR_FLEXBUS_FBADL_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_FEC */
#define MCF_GPIO_DSCR_FEC_FEC0(x)		(((x)&0x03))
#define MCF_GPIO_DSCR_FEC_FEC1(x)		(((x)&0x03)<<2)
#define MCF_GPIO_DSCR_FEC_FEC1_LOAD_50PF	(0x0C)
#define MCF_GPIO_DSCR_FEC_FEC1_LOAD_30PF	(0x08)
#define MCF_GPIO_DSCR_FEC_FEC1_LOAD_20PF	(0x04)
#define MCF_GPIO_DSCR_FEC_FEC1_LOAD_10PF	(0x00)
#define MCF_GPIO_DSCR_FEC_FEC0_LOAD_50PF	(0x03)
#define MCF_GPIO_DSCR_FEC_FEC0_LOAD_30PF	(0x02)
#define MCF_GPIO_DSCR_FEC_FEC0_LOAD_20PF	(0x01)
#define MCF_GPIO_DSCR_FEC_FEC0_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_UART */
#define MCF_GPIO_DSCR_UART_UART0(x)		(((x)&0x03))
#define MCF_GPIO_DSCR_UART_UART1(x)		(((x)&0x03)<<2)
#define MCF_GPIO_DSCR_UART_UART1_LOAD_50PF	(0x0C)
#define MCF_GPIO_DSCR_UART_UART1_LOAD_30PF	(0x08)
#define MCF_GPIO_DSCR_UART_UART1_LOAD_20PF	(0x04)
#define MCF_GPIO_DSCR_UART_UART1_LOAD_10PF	(0x00)
#define MCF_GPIO_DSCR_UART_UART0_LOAD_50PF	(0x03)
#define MCF_GPIO_DSCR_UART_UART0_LOAD_30PF	(0x02)
#define MCF_GPIO_DSCR_UART_UART0_LOAD_20PF	(0x01)
#define MCF_GPIO_DSCR_UART_UART0_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_DSPI */
#define MCF_GPIO_DSCR_DSPI_DSPI(x)		(((x)&0x03))
#define MCF_GPIO_DSCR_DSPI_DSPI_LOAD_50PF	(0x03)
#define MCF_GPIO_DSCR_DSPI_DSPI_LOAD_30PF	(0x02)
#define MCF_GPIO_DSCR_DSPI_DSPI_LOAD_20PF	(0x01)
#define MCF_GPIO_DSCR_DSPI_DSPI_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_TIMER */
#define MCF_GPIO_DSCR_TIMER_TIMER(x)		(((x)&0x03))
#define MCF_GPIO_DSCR_TIMER_TIMER_LOAD_50PF	(0x03)
#define MCF_GPIO_DSCR_TIMER_TIMER_LOAD_30PF	(0x02)
#define MCF_GPIO_DSCR_TIMER_TIMER_LOAD_20PF	(0x01)
#define MCF_GPIO_DSCR_TIMER_TIMER_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_SSI */
#define MCF_GPIO_DSCR_SSI_SSI(x)	(((x)&0x03))
#define MCF_GPIO_DSCR_SSI_SSI_LOAD_50PF	(0x03)
#define MCF_GPIO_DSCR_SSI_SSI_LOAD_30PF	(0x02)
#define MCF_GPIO_DSCR_SSI_SSI_LOAD_20PF (0x01)
#define MCF_GPIO_DSCR_SSI_SSI_LOAD_10PF (0x00)

/* Bit definitions and macros for DSCR_DMA */
#define MCF_GPIO_DSCR_DMA_DMA(x)	(((x)&0x03))
#define MCF_GPIO_DSCR_DMA_DMA_LOAD_50PF	(0x03)
#define MCF_GPIO_DSCR_DMA_DMA_LOAD_30PF	(0x02)
#define MCF_GPIO_DSCR_DMA_DMA_LOAD_20PF	(0x01)
#define MCF_GPIO_DSCR_DMA_DMA_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_DEBUG */
#define MCF_GPIO_DSCR_DEBUG_DEBUG(x)		(((x)&0x03))
#define MCF_GPIO_DSCR_DEBUG_DEBUG_LOAD_50PF	(0x03)
#define MCF_GPIO_DSCR_DEBUG_DEBUG_LOAD_30PF	(0x02)
#define MCF_GPIO_DSCR_DEBUG_DEBUG_LOAD_20PF	(0x01)
#define MCF_GPIO_DSCR_DEBUG_DEBUG_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_RESET */
#define MCF_GPIO_DSCR_RESET_RESET(x)		(((x)&0x03))
#define MCF_GPIO_DSCR_RESET_RESET_LOAD_50PF	(0x03)
#define MCF_GPIO_DSCR_RESET_RESET_LOAD_30PF	(0x02)
#define MCF_GPIO_DSCR_RESET_RESET_LOAD_20PF	(0x01)
#define MCF_GPIO_DSCR_RESET_RESET_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_IRQ */
#define MCF_GPIO_DSCR_IRQ_IRQ(x)	(((x)&0x03))
#define MCF_GPIO_DSCR_IRQ_IRQ_LOAD_50PF	(0x03)
#define MCF_GPIO_DSCR_IRQ_IRQ_LOAD_30PF	(0x02)
#define MCF_GPIO_DSCR_IRQ_IRQ_LOAD_20PF	(0x01)
#define MCF_GPIO_DSCR_IRQ_IRQ_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_USB */
#define MCF_GPIO_DSCR_USB_USB(x)	(((x)&0x03))
#define MCF_GPIO_DSCR_USB_USB_LOAD_50PF	(0x03)
#define MCF_GPIO_DSCR_USB_USB_LOAD_30PF	(0x02)
#define MCF_GPIO_DSCR_USB_USB_LOAD_20PF	(0x01)
#define MCF_GPIO_DSCR_USB_USB_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_ATA */
#define MCF_GPIO_DSCR_ATA_ATA(x)	(((x)&0x03))
#define MCF_GPIO_DSCR_ATA_ATA_LOAD_50PF	(0x03)
#define MCF_GPIO_DSCR_ATA_ATA_LOAD_30PF	(0x02)
#define MCF_GPIO_DSCR_ATA_ATA_LOAD_20PF	(0x01)
#define MCF_GPIO_DSCR_ATA_ATA_LOAD_10PF	(0x00)

/********************************************************************/

#endif /* __MCF5445X_GPIO_H__ */

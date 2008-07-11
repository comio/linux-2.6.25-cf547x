/***************************************************************************
			smifb.h  -  SiliconMotion LynxEM+ frame buffer device
                             -------------------
    begin                : Thu Aug 9 2001
    copyright            : (C) 2001 by Szu-Tao Huang
    email                : johuang@siliconmotion.com
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#define smi_mmiowb(dat,reg)	writeb(dat, SMIRegs + reg)
#define smi_mmioww(dat,reg)	writew(dat, SMIRegs + reg)
#define smi_mmiowl(dat,reg)	writel(dat, SMIRegs + reg)

#define smi_mmiorb(reg)	        readb(SMIRegs + reg)
#define smi_mmiorw(reg)	        readw(SMIRegs + reg)
#define smi_mmiorl(reg)	        readl(SMIRegs + reg)

#define SIZE_SR00_SR04      (0x04 - 0x00 + 1)
#define SIZE_SR10_SR24      (0x24 - 0x10 + 1)
#define SIZE_SR30_SR75      (0x75 - 0x30 + 1)
#define SIZE_SR80_SR93      (0x93 - 0x80 + 1)
#define SIZE_SRA0_SRAF      (0xAF - 0xA0 + 1)
#define SIZE_GR00_GR08      (0x08 - 0x00 + 1)
#define SIZE_AR00_AR14      (0x14 - 0x00 + 1)
#define SIZE_CR00_CR18      (0x18 - 0x00 + 1)
#define SIZE_CR30_CR4D      (0x4D - 0x30 + 1)
#define SIZE_CR90_CRA7      (0xA7 - 0x90 + 1)
#define SIZE_VPR                        (0x6C + 1)
#define SIZE_DPR			(0x44 + 1)

#define numVGAModes			6
#define numChipIDs			3

#define NR_PALETTE	256
#define NR_RGB          2

/*
 * Minimum X and Y resolutions
 */
#define MIN_XRES	640
#define MIN_YRES	480

static inline void smi_crtcw(int reg, int val)
{
        smi_mmiowb(reg, 0x3d4);
        smi_mmiowb(val, 0x3d5);
}

static inline unsigned int smi_crtcr(int reg)
{
        smi_mmiowb(reg, 0x3d4);
        return smi_mmiorb(0x3d5);
}

static inline void smi_grphw(int reg, int val)
{
        smi_mmiowb(reg, 0x3ce);
        smi_mmiowb(val, 0x3cf);
}

static inline unsigned int smi_grphr(int reg)
{
        smi_mmiowb(reg, 0x3ce);
        return smi_mmiorb(0x3cf);
}

static inline void smi_attrw(int reg, int val)
{
        smi_mmiorb(0x3da);
        smi_mmiowb(reg, 0x3c0);
        smi_mmiorb(0x3c1);
        smi_mmiowb(val, 0x3c0);
}

static inline void smi_seqw(int reg, int val)
{
        smi_mmiowb(reg, 0x3c4);
        smi_mmiowb(val, 0x3c5);
}

static inline unsigned int smi_seqr(int reg)
{
        smi_mmiowb(reg, 0x3c4);
        return smi_mmiorb(0x3c5);
}
/*
* Private structure
*/
struct smifb_info {
        /*
        * The following is a pointer to be passed into the
        * functions below.  The modules outside the main
        * smifb.c driver have no knowledge as to what
        * is within this structure.
        */
        struct fb_info          fb;
        struct display_switch   *dispsw;
        struct pci_dev	        *dev;
        signed int              currcon;

        struct {
                u8 red, green, blue;
        } palette[NR_RGB];

        u_int                   palette_size;
};

struct par_info {
	/*
	 * Hardware
	 */
	u16		chipID;
	char	*m_pLFB;
	char	*m_pMMIO;
	char	*m_pDPR;
	char	*m_pVPR;

	u_int	width;
	u_int	height;
	u_int	hz;
};

// The next structure holds all information relevant for a specific video mode.
struct ModeInit
{
	int			  mmSizeX;
	int			  mmSizeY;
	int			  bpp;
	int			  hz;
	unsigned char Init_MISC;
	unsigned char Init_SR00_SR04[SIZE_SR00_SR04];
	unsigned char Init_SR10_SR24[SIZE_SR10_SR24];
	unsigned char Init_SR30_SR75[SIZE_SR30_SR75];
	unsigned char Init_SR80_SR93[SIZE_SR80_SR93];
	unsigned char Init_SRA0_SRAF[SIZE_SRA0_SRAF];
	unsigned char Init_GR00_GR08[SIZE_GR00_GR08];
	unsigned char Init_AR00_AR14[SIZE_AR00_AR14];
	unsigned char Init_CR00_CR18[SIZE_CR00_CR18];
	unsigned char Init_CR30_CR4D[SIZE_CR30_CR4D];
	unsigned char Init_CR90_CRA7[SIZE_CR90_CRA7];
};

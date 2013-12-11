/*
 * Freescale Vybrid MVF Video Driver
 *
 * Copyright (C) 2013 Device Solutions Ltd
 *
 * Based off mxsfb.c
 * Copyright (C) 2011-2013 Marek Vasut <marex@denx.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <malloc.h>
#include <video_fb.h>

#include <asm/arch/vybrid-regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/errno.h>
#include <asm/io.h>

#include "videomodes.h"

#define	PS2KHZ(ps)	(1000000000UL / (ps))

static GraphicDevice panel;

/*
 * DENX M28EVK:
 * setenv videomode
 * video=ctfb:x:800,y:480,depth:18,mode:0,pclk:30066,
 *       le:0,ri:256,up:0,lo:45,hs:1,vs:1,sync:100663296,vmode:0
 *
 * Freescale mx23evk/mx28evk with a Seiko 4.3'' WVGA panel:
 * setenv videomode
 * video=ctfb:x:800,y:480,depth:24,mode:0,pclk:29851,
 * 	 le:89,ri:164,up:23,lo:10,hs:10,vs:10,sync:0,vmode:0
 */

static struct clkctl *ccm = (struct clkctl *)CCM_BASE_ADDR;
static struct tconctl *tcon0 = (struct tconctl *)TCON0_BASE_ADDR;

static void mvf_lcd_init(GraphicDevice *panel, struct ctfb_res_modes *mode, int bpp)
{
	struct dcu_regs *dcu0 = (struct dcu_regs *)DCU0_BASE_ADDR;
	uint32_t dcu_update_mode = 0;
	int i = 0;

	// Setup GPIO for the display - should happen elsewhere...
	// Hack - write GPIO numbers here!
	writel( (1<<25), 0x400FF004 );	//GPIO0->PSOR = PIN(25);
	writel( (1<<22) | (1<<12), 0x400FF0C4 ); //GPIO3->PSOR = PIN(20); GPIO3->PSOR = PIN(12);

/*
	printf("x:0x%08x y:0x%08x\r\n", mode->xres, mode->yres );
	printf("CCGR2 = 0x%08x\r\n", ccm->ccgr2 );
	printf("Buffer = 0x%08x\r\n", panel->frameAdrs );
*/

	/* Enable DCU */
	ccm->ccgr3 |= 0x00030000; /* CCM_CCGR3_CG8(3) */
	ccm->cscdr3 |= 0x000b0000; /* CCM_CSCDR3_DCU0_DIV(5) | CCM_CSCDR3_DCU0_EN_MASK */

	/* Enable TCON and put in BYPASS mode */
	ccm->ccgr1 |= 0x000000c0; /*CCM_CCGR1_CG13(3) */
	tcon0->ctrl1 |= 0x20000000; /* TCON_CTRL1_TCON_BYPASS_MASK */

	//dcu0->disp_size = ((mode->xres/16) << 16) | mode->yres;
	//dcu0->hsyn_para = (mode->left_margin << 22)   | (mode->hsync_len << 11) | mode->right_margin;
	//dcu0->vsyn_para = (mode->upper_margin << 22)  | (mode->vsync_len << 11) | mode->lower_margin;
	//dcu0->div_ratio = 3;

//	writel( ((1024/16) | (768 << 16)), &dcu0->disp_size );		// LVDS
//	writel( ((800/16) | (480 << 16)), &dcu0->disp_size );		// 7 inch
//	writel( (40 << 22)   | (48 << 11) | 40, &dcu0->hsyn_para );
//	writel( (13 << 22)  | (3 << 11) | 29, &dcu0->vsyn_para );
//	writel( 0, &dcu0->div_ratio );

	writel( ((mode->xres/16) | (mode->yres << 16)), &dcu0->disp_size );		// 7 inch
	writel( (mode->left_margin << 22)   | (mode->hsync_len << 11) | mode->right_margin, &dcu0->hsyn_para );
	writel( (mode->upper_margin << 22)  | (mode->vsync_len << 11) | mode->lower_margin, &dcu0->vsyn_para );
	//writel( (48/(mode->pixclock))-1, &dcu0->div_ratio );
	writel( 3, &dcu0->div_ratio );

	writel(  (0x00004000 | mode->vmode), &dcu0->dcu_mode ); 

	// First disable all layers
	for( i = 0; i < 64; i++ )
		dcu0->layer[i].ctrldescl_4 = 0;

	dcu0->layer[0].ctrldescl_1 = (mode->yres << 16) | mode->xres; // size
	dcu0->layer[0].ctrldescl_2 = 0x00000000; // position
	dcu0->layer[0].ctrldescl_3 = panel->frameAdrs; // address - needs to be 64-bit aligned...
	dcu0->layer[0].ctrldescl_4 = 0x80060000; // Enable | bpp ; 

	// These registers are not required - zero to ensure nothing stupid happens
	dcu0->layer[0].ctrldescl_5 = 0x00000000; // chroma key max
	dcu0->layer[0].ctrldescl_6 = 0x00000000; // chroma key min
	dcu0->layer[0].ctrldescl_7 = 0x00000000; // tile
	dcu0->layer[0].ctrldescl_8 = 0x00000000; // foreground color when using transparency
	dcu0->layer[0].ctrldescl_9 = 0x00000000; // background color when using transparency

	// Update mode
	writel( 0x40000000, &dcu0->update_mode );
    do
	{
		dcu_update_mode = dcu0->update_mode;
		udelay(200);
		//printf("waiting for update\r\n");
	} while( (dcu_update_mode & 0x40000000) != 0 );
	
	dcu0->update_mode = 0x40000000;
	
	//printf("disp_size = 0x%08x at 0x%08x \r\n", dcu0->disp_size, &(dcu0->disp_size));

}

void *video_hw_init(void)
{
	int bpp = -1;
	char *penv;
	void *fb;
	struct ctfb_res_modes mode;

	puts("Video: ");

	/* Suck display configuration from "videomode" variable */
	penv = getenv("videomode");
	if (!penv) {
		printf("MVFFB: 'videomode' variable not set!\r\n");
		return NULL;
	}

	bpp = video_get_params(&mode, penv);

	/* fill in Graphic device struct */
	sprintf(panel.modeIdent, "%dx%dx%d", mode.xres, mode.yres, bpp);

	panel.winSizeX = mode.xres;
	panel.winSizeY = mode.yres;
	panel.plnSizeX = mode.xres;
	panel.plnSizeY = mode.yres;

	switch (bpp) {
	case 24:
	case 18:
		panel.gdfBytesPP = 4;
		panel.gdfIndex = GDF_32BIT_X888RGB;
		break;
	case 16:
		panel.gdfBytesPP = 2;
		panel.gdfIndex = GDF_16BIT_565RGB;
		break;
	case 8:
		panel.gdfBytesPP = 1;
		panel.gdfIndex = GDF__8BIT_INDEX;
		break;
	default:
		printf("MXSFB: Invalid BPP specified! (bpp = %i)\n", bpp);
		return NULL;
	}

	panel.memSize = mode.xres * mode.yres * panel.gdfBytesPP;

	/* Allocate framebuffer */
	fb = malloc(panel.memSize);
	//fb = (u32*)0x81000000;
	if (!fb) {
		printf("MXSFB: Error allocating framebuffer!\n");
		return NULL;
	}
	else
		printf("MXSFB: Alllocated %d bytes at 0x%08x\r\n", panel.memSize, fb );

	/* Wipe framebuffer */
	memset(fb, 0, panel.memSize);

	panel.frameAdrs = (u32)fb;

	printf("%s\n", panel.modeIdent);

	/* Start framebuffer */
	mvf_lcd_init(&panel, &mode, bpp);

	return (void *)&panel;

}

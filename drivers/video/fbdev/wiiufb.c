/*
 * wiiufb - Wii U framebuffer driver
 * Based on xilinxfb
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <stdarg.h>
#include <asm/cacheflush.h>
#include <asm/io.h>

#include "wiiufb_regs.h"

#define DRIVER_NAME		"wiiufb"

/*	TODO:
 *	There's some kind of caching issue going on here.
 *	Seems it might be something in copyarea or imageblit?
 */

//Useful for reading stuff out from the GX2's MMIO
static int leak_mmio = 0xDEADCAFE;
module_param(leak_mmio, int, 0644);
MODULE_PARM_DESC(leak_mmio, "oops");

/*
 * In default fb mode we only support a single mode: 1280x720 32 bit true color.
 * Each pixel gets a word (32 bits) of memory, organized as RGBA8888
 */

// 16 bit
#define BYTES_PER_PIXEL	2
#define BITS_PER_PIXEL	(BYTES_PER_PIXEL * 8)

// RGB565
#define OFFS_RED		11
#define OFFS_GREEN		5
#define OFFS_BLUE		0

// Use 16 palettes
#define MAX_PALETTES	16

struct wiiufb_platform_data {
	u32 width, height;         /* resolution of screen in pixels */
};

// Default wiiufb configuration
static struct wiiufb_platform_data wiiu_fb_default_pdata = {
	.width = 1280,
	.height = 720,
};

static struct fb_fix_screeninfo wiiu_fb_fix = {
	.id =		"wiiufb",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.accel =	FB_ACCEL_NONE
};

static struct fb_var_screeninfo wiiu_fb_var = {
	.bits_per_pixel =	BITS_PER_PIXEL,

	.red =		{ OFFS_RED, 5, 0 },
	.green =	{ OFFS_GREEN, 6, 0 },
	.blue =		{ OFFS_BLUE, 5, 0 },
	.transp =	{ 0, 0, 0 },

	.activate =	FB_ACTIVATE_NOW
};

struct wiiufb_drvdata {
	struct fb_info info;								/* FB driver info record */
	void __iomem *regs;									/* virt. address of the control registers */
	phys_addr_t	  regs_phys;
	void         *fb_virt;								/* virt. address of the frame buffer */
	dma_addr_t    fb_phys;								/* phys. address of the frame buffer */
	u32 	      pseudo_palette[MAX_PALETTES];			/* Fake palette of 16 colors */
};
	
static int wiiufb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp, struct fb_info *info) {
	u32 *pal = info->pseudo_palette;
	
	if (regno >= MAX_PALETTES)
		return -EINVAL;

	// convert RGB to grayscale
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green + 7471 * blue) >> 16;

	// 16 bit RGB565
	pal[regno] = (red & 0xf800) | ((green & 0xfc00) >> 5) | ((blue & 0xf800) >> 11);
	return 0;
}

static struct fb_ops wiiufb_ops = {
	.owner				= THIS_MODULE,
	.fb_setcolreg		= wiiufb_setcolreg,
	.fb_fillrect		= cfb_fillrect,
	.fb_copyarea		= cfb_copyarea,
	.fb_imageblit		= cfb_imageblit,
};


#include <linux/delay.h>
static int wiiufb_assign(struct platform_device *pdev, struct wiiufb_drvdata *drvdata, struct wiiufb_platform_data *pdata) {
	int rc;
	struct device *dev = &pdev->dev;
	int fbsize = pdata->width * pdata->height * BYTES_PER_PIXEL;
	struct resource* res;
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drvdata->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(drvdata->regs)) {
		dev_err(dev, "Failed to map registers!\n");
		return -ENOMEM;
	}
	
	drvdata->regs_phys = res->start;
	drvdata->fb_virt = dma_zalloc_coherent(dev, PAGE_ALIGN(fbsize), &drvdata->fb_phys, GFP_KERNEL);
	
	if (!drvdata->fb_virt) {
		dev_err(dev, "Could not allocate framebuffer!\n");
		return -ENOMEM;
	}

	writereg(D1GRPH_ENABLE, 0);
	writereg(D1GRPH_CONTROL, 0);
	writereg(D1GRPH_PRIMARY_SURFACE_ADDRESS, 0);
	writereg(D1GRPH_PITCH, 0);
	setreg(D1GRPH_ENABLE, D1GRPH_ENABLE_REG, 1);
	setreg(D1GRPH_CONTROL, D1GRPH_DEPTH, D1GRPH_DEPTH_16BPP);
	setreg(D1GRPH_CONTROL, D1GRPH_FORMAT, D1GRPH_FORMAT_16BPP_RGB565);
	setreg(D1GRPH_CONTROL, D1GRPH_ADDRESS_TRANSLATION, D1GRPH_ADDRESS_TRANSLATION_PHYS);
	setreg(D1GRPH_CONTROL, D1GRPH_PRIVILEGED_ACCESS, D1GRPH_PRIVILEGED_ACCESS_DISABLE);
	setreg(D1GRPH_CONTROL, D1GRPH_ARRAY_MODE, D1GRPH_ARRAY_LINEAR_ALIGNED);
	setreg(D1GRPH_PRIMARY_SURFACE_ADDRESS, D1GRPH_PRIMARY_SURFACE_ADDR, drvdata->fb_phys);
	setreg(D1GRPH_PITCH, D1GRPH_PITCH_VAL, pdata->width);
	leak_mmio = readreg(D1GRPH_CONTROL);
	
	setreg(D1GRPH_SWAP_CNTL, D1GRPH_ENDIAN_SWAP, D1GRPH_ENDIAN_SWAP_16);
	setreg(D1GRPH_SWAP_CNTL, D1GRPH_RED_CROSSBAR, D1GRPH_RED_CROSSBAR_RED);
	setreg(D1GRPH_SWAP_CNTL, D1GRPH_GREEN_CROSSBAR, D1GRPH_GREEN_CROSSBAR_GREEN);
	setreg(D1GRPH_SWAP_CNTL, D1GRPH_BLUE_CROSSBAR, D1GRPH_BLUE_CROSSBAR_BLUE);
	setreg(D1GRPH_SWAP_CNTL, D1GRPH_ALPHA_CROSSBAR, D1GRPH_ALPHA_CROSSBAR_ALPHA);

	/* Fill struct fb_info */
	drvdata->info.device = dev;
	drvdata->info.screen_base = drvdata->fb_virt;
	drvdata->info.screen_size = fbsize;
	drvdata->info.fbops = &wiiufb_ops;
	drvdata->info.fix = wiiu_fb_fix;
	drvdata->info.fix.smem_start = drvdata->fb_phys;
	drvdata->info.fix.smem_len = fbsize;
	drvdata->info.fix.line_length = pdata->width * BYTES_PER_PIXEL;

	drvdata->info.pseudo_palette = drvdata->pseudo_palette;
	drvdata->info.flags = FBINFO_DEFAULT;
	drvdata->info.var = wiiu_fb_var;
	drvdata->info.var.xres = pdata->width;
	drvdata->info.var.yres = pdata->height;
	drvdata->info.var.xres_virtual = pdata->width;
	drvdata->info.var.yres_virtual = pdata->height;

	/* Allocate a colour map */
	rc = fb_alloc_cmap(&drvdata->info.cmap, MAX_PALETTES, 0);
	if (rc) {
		return rc;
	}

	/* Register new frame buffer */
	rc = register_framebuffer(&drvdata->info);
	if (rc) {
		fb_dealloc_cmap(&drvdata->info.cmap);
		return rc;
	}
		
	return 0;
}

static int wiiufb_release(struct device *dev) {
	struct wiiufb_drvdata *drvdata = dev_get_drvdata(dev);

	unregister_framebuffer(&drvdata->info);
	fb_dealloc_cmap(&drvdata->info.cmap);

	return 0;
}

static int wiiufb_probe(struct platform_device *pdev) {
	struct wiiufb_platform_data pdata;
	struct wiiufb_drvdata *drvdata;

	pdata = wiiu_fb_default_pdata;

	/* Allocate the driver data region */
	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata) {
		return -ENOMEM;
	}

	dev_set_drvdata(&pdev->dev, drvdata);
	return wiiufb_assign(pdev, drvdata, &pdata);
}

static int wiiufb_remove(struct platform_device *op) {
	return wiiufb_release(&op->dev);
}

/* Match table for of_platform binding */
static struct of_device_id wiiufb_match[] = {
	{ .compatible = "nintendo,wiiufb", },
	{},
};
MODULE_DEVICE_TABLE(of, wiiufb_match);

static struct platform_driver wiiufb_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = wiiufb_match,
	},
	.probe  = wiiufb_probe,
	.remove = wiiufb_remove,
};
module_platform_driver(wiiufb_driver);

MODULE_DESCRIPTION("Wii U framebuffer driver");
MODULE_LICENSE("GPL");

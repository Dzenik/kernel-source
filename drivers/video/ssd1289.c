/*
 * SSD1289 Framebuffer
 *
 *
 * Original: Copyright (c) 2009 Jean-Christian de Rivaz
 *
 * SPI mods, console support, 320x240 instead of 240x320:
 * Copyright (c) 2012 Jeroen Domburg <jeroen@spritesmods.com>
 *
 * Bits and pieces borrowed from the fsl-ssd1289.c:
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 * Author: Alison Wang <b18965@freescale.com>
 *         Jason Jin <Jason.jin@freescale.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * The Solomon Systech SSD1289 chip drive TFT screen up to 240x320. 
 *
 * For direct I/O-mode:
 *
 * This driver expect the SSD1286 to be connected to a 16 bits local bus
 * and to be set in the 16 bits parallel interface mode. To use it you must
 * define in your board file a struct platform_device with a name set to
 * "ssd1289" and a struct resource array with two IORESOURCE_MEM: the first
 * for the control register; the second for the data register.
 *
 * For SPI mode:
 *
 * A SPI port with modalias 'ssd1289' should exist. The LCD should be in 16-bit
 * mode (which they usually are hardwired to be...) and connected to three
 * CD4094s and a CD4020. For schematic, check out 
 * http://spritesmods.com/?art=spitft
 *
 * LCDs in their own, native SPI mode aren't supported yet, mostly because I 
 * can't get my hands on a cheap one.
 */

#define DEBUG

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/fb.h>
#include <asm/io.h>
#include <linux/spi/spi.h>

#define SSD1289_REG_OSCILLATION      0x00
#define SSD1289_REG_DRIVER_OUT_CTRL  0x01
#define SSD1289_REG_LCD_DRIVE_AC     0x02
#define SSD1289_REG_POWER_CTRL_1     0x03
#define SSD1289_REG_DISPLAY_CTRL     0x07
#define SSD1289_REG_FRAME_CYCLE      0x0b
#define SSD1289_REG_POWER_CTRL_2     0x0c
#define SSD1289_REG_POWER_CTRL_3     0x0d
#define SSD1289_REG_POWER_CTRL_4     0x0e
#define SSD1289_REG_GATE_SCAN_START  0x0f
#define SSD1289_REG_SLEEP_MODE       0x10
#define SSD1289_REG_ENTRY_MODE       0x11
#define SSD1289_REG_POWER_CTRL_5     0x1e
#define SSD1289_REG_GDDRAM_DATA      0x22
#define SSD1289_REG_WR_DATA_MASK_1   0x23
#define SSD1289_REG_WR_DATA_MASK_2   0x24
#define SSD1289_REG_FRAME_FREQUENCY  0x25
#define SSD1289_REG_GAMMA_CTRL_1     0x30
#define SSD1289_REG_GAMME_CTRL_2     0x31
#define SSD1289_REG_GAMMA_CTRL_3     0x32
#define SSD1289_REG_GAMMA_CTRL_4     0x33
#define SSD1289_REG_GAMMA_CTRL_5     0x34
#define SSD1289_REG_GAMMA_CTRL_6     0x35
#define SSD1289_REG_GAMMA_CTRL_7     0x36
#define SSD1289_REG_GAMMA_CTRL_8     0x37
#define SSD1289_REG_GAMMA_CTRL_9     0x3a
#define SSD1289_REG_GAMMA_CTRL_10    0x3b
#define SSD1289_REG_V_SCROLL_CTRL_1  0x41
#define SSD1289_REG_V_SCROLL_CTRL_2  0x42
#define SSD1289_REG_H_RAM_ADR_POS    0x44
#define SSD1289_REG_V_RAM_ADR_START  0x45
#define SSD1289_REG_V_RAM_ADR_END    0x46
#define SSD1289_REG_FIRST_WIN_START  0x48
#define SSD1289_REG_FIRST_WIN_END    0x49
#define SSD1289_REG_SECND_WIN_START  0x4a
#define SSD1289_REG_SECND_WIN_END    0x4b
#define SSD1289_REG_GDDRAM_X_ADDR    0x4e
#define SSD1289_REG_GDDRAM_Y_ADDR    0x4f

static const char *dev_spi = "/dev/spidev0.0";
module_param(dev_spi, charp, 0000);
MODULE_PARM_DESC(dev_spi, "SPI device to use (default=/dev/spidev0.0)");

struct ssd1289_page {
	unsigned short x;
	unsigned short y;
	unsigned short *buffer;
	unsigned short len;
	int must_update;
};

struct ssd1289 {
	struct device *dev;
#if defined(CONFIG_SSD1289_DIRECTIO_MODE)
	volatile unsigned short *ctrl_io;
	volatile unsigned short *data_io;
#elif defined(CONFIG_SSD1289_SPI_MODE)
	struct spi_device *spidev;
#endif
	struct fb_info *info;
	unsigned int pages_count;
	struct ssd1289_page *pages;
	unsigned long pseudo_palette[17];
	int backlight;
};


#if defined(CONFIG_SSD1289_SPI_MODE)
/*
hese two routines will write to the SPI-port, shifting data into the 4094 shift
registers.
*/
#define CD4094_RD (1<<0)
#define CD4094_WR (1<<1)
#define CD4094_BCNT (1<<2)
#define CD4094_RST (1<<3)
#define CD4094_DC (1<<4)
#define CD4094_INVERT (CD4094_RD|CD4094_WR|CD4094_RST)

/*
This routine will write a single 16-bit value, either as data or as a command 
(depends on isdata). The LCD will clock this in because the SPIs /CS goes high.
*/
static int ssd1289_spi_write(struct ssd1289 *item, unsigned short value, 
				unsigned int isdata) 
{

    // item->spidev->max_speed_hz=32000000;  
    // item->spidev->master->setup(item->spidev);

	u8 buf[3];
	buf[0]=((isdata?CD4094_DC:0)|CD4094_WR|(item->backlight?CD4094_BCNT:0))^CD4094_INVERT;
	buf[1]=(value>>8)&0xff;
	buf[2]=(value)&0xff;
	spi_write(item->spidev, buf, 3);
	return 0;
}

#define BLOCKLEN (4096)
static u8 ssd1289_spiblock[BLOCKLEN*4];

/*
This routine will clock in len words of data. The LCD will clock this in every 4th byte written
because a 4020 will pull down the /cs at that moment.
*/
static int ssd1289_spi_write_datablock(struct ssd1289 *item, 
					unsigned short *block, int len) 
{
	int x;
	unsigned short value;

    // item->spidev->max_speed_hz=32000000;  
    // item->spidev->master->setup(item->spidev);

	//ToDo: send in parts if needed
	if (len>BLOCKLEN) {
		dev_err(item->dev, "%s: len > blocklen (%i > %i)\n",
			__func__, len, BLOCKLEN);
		len=BLOCKLEN;
	}

	for (x=0; x<len; x++) {
		value=block[x];
		ssd1289_spiblock[(x*4)]=0; //dummy
		ssd1289_spiblock[(x*4)+1]=(CD4094_DC|CD4094_WR|(item->backlight?CD4094_BCNT:0))^CD4094_INVERT;
		ssd1289_spiblock[(x*4)+2]=(value>>8)&0xff;
		ssd1289_spiblock[(x*4)+3]=(value)&0xff;
	}
	spi_write(item->spidev, ssd1289_spiblock, len*4);
	return 0;
}
#endif


static inline void ssd1289_reg_set(struct ssd1289 *item, unsigned char reg,
				   unsigned short value)
{
#if defined(CONFIG_SSD1289_DIRECTIO_MODE)
	unsigned short ctrl = reg & 0x00ff;
	writew(ctrl, item->ctrl_io);
	writew(value, item->data_io);
#elif defined(CONFIG_SSD1289_SPI_MODE)
	ssd1289_spi_write(item, reg&0xff, 0);
	ssd1289_spi_write(item, value, 1);
#endif
}

static void ssd1289_copy(struct ssd1289 *item, unsigned int index)
{
	unsigned short x;
	unsigned short y;
	unsigned short *buffer;
	unsigned int len;
#if defined(CONFIG_SSD1289_DIRECTIO_MODE)
	unsigned int count;
#endif
	x = item->pages[index].x;
	y = item->pages[index].y;
	buffer = item->pages[index].buffer;
	len = item->pages[index].len;
	dev_dbg(item->dev,
		"%s: page[%u]: x=%3hu y=%3hu buffer=0x%p len=%3hu\n",
		__func__, index, x, y, buffer, len);

	ssd1289_reg_set(item, SSD1289_REG_GDDRAM_X_ADDR, (item->info->var.yres - 1)-y);
	ssd1289_reg_set(item, SSD1289_REG_GDDRAM_Y_ADDR, x);
#if defined(CONFIG_SSD1289_DIRECTIO_MODE)
	writew(SSD1289_REG_GDDRAM_DATA, item->ctrl_io);
	for (count = 0; count < len; count++) {
		writew(buffer[count], item->data_io);
	}
#elif defined(CONFIG_SSD1289_SPI_MODE)
	ssd1289_spi_write(item, SSD1289_REG_GDDRAM_DATA, 0);

	ssd1289_spi_write_datablock(item, buffer, len);
/*
//The write_datablock can also be exchanged with this code, which is slower but 
//doesn't require the CD4020 counter IC.
	for (count = 0; count < len; count++) {
		ssd1289_spi_write(item, buffer[count], 1);
	}
*/
#endif
}

static void ssd1289_update_all(struct ssd1289 *item)
{
	unsigned short i;
	struct fb_deferred_io *fbdefio = item->info->fbdefio;
	for (i = 0; i < item->pages_count; i++) {
		item->pages[i].must_update=1;
	}
	schedule_delayed_work(&item->info->deferred_work, fbdefio->delay);
}

static void ssd1289_update(struct fb_info *info, struct list_head *pagelist)
{
	struct ssd1289 *item = (struct ssd1289 *)info->par;
	struct page *page;
	int i;

	//We can be called because of pagefaults (mmap'ed framebuffer, pages
	//returned in *pagelist) or because of kernel activity 
	//(pages[i]/must_update!=0). Add the former to the list of the latter.
	list_for_each_entry(page, pagelist, lru) {
		item->pages[page->index].must_update=1;
	}

	//Copy changed pages.
	for (i=0; i<item->pages_count; i++) {
	//ToDo: Small race here between checking and setting must_update, 
		//maybe lock?
		if (item->pages[i].must_update) {
			item->pages[i].must_update=0;
			ssd1289_copy(item, i);
		}
	}

}


static void __init ssd1289_setup(struct ssd1289 *item)
{
	dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);
	// OSCEN=1
	ssd1289_reg_set(item, SSD1289_REG_OSCILLATION, 0x0001);
	// DCT=b1010=fosc/4 BT=b001=VGH:+6,VGL:-4
	// DC=b1010=fosc/4 AP=b010=small to medium
	ssd1289_reg_set(item, SSD1289_REG_POWER_CTRL_1, 0xa2a4);
	// VRC=b100:5.5V
	ssd1289_reg_set(item, SSD1289_REG_POWER_CTRL_2, 0x0004);
	// VRH=b1000:Vref*2.165
	ssd1289_reg_set(item, SSD1289_REG_POWER_CTRL_3, 0x0308);
	// VCOMG=1 VDV=b1000:VLCD63*1.05
	ssd1289_reg_set(item, SSD1289_REG_POWER_CTRL_4, 0x3000);
	// nOTP=1 VCM=0x2a:VLCD63*0.77
	ssd1289_reg_set(item, SSD1289_REG_POWER_CTRL_5, 0x00ea);
	// RL=0 REV=1 CAD=0 BGR=1 SM=0 TB=1 MUX=0x13f=319
	ssd1289_reg_set(item, SSD1289_REG_DRIVER_OUT_CTRL, 0x2b3f);
	// FLD=0 ENWS=0 D/C=1 EOR=1 WSMD=0 NW=0x00=0
	ssd1289_reg_set(item, SSD1289_REG_LCD_DRIVE_AC, 0x0600);
	// SLP=0
	ssd1289_reg_set(item, SSD1289_REG_SLEEP_MODE, 0x0000);
	// VSMode=0 DFM=3:65k TRAMS=0 OEDef=0 WMode=0 Dmode=0
	// TY=0 ID=2 AM=1 LG=0 (orig: ID=3, AM=0)
//	ssd1289_reg_set(item, SSD1289_REG_ENTRY_MODE, 0x6030);
	ssd1289_reg_set(item, SSD1289_REG_ENTRY_MODE, 0x6028);
	// PT=0 VLE=1 SPT=0 GON=1 DTE=1 CM=0 D=3
	ssd1289_reg_set(item, SSD1289_REG_DISPLAY_CTRL, 0x0233);
	// NO=0 SDT=0 EQ=0 DIV=0 SDIV=1 SRTN=1 RTN=9:25 clock
	ssd1289_reg_set(item, SSD1289_REG_FRAME_CYCLE, 0x0039);
	// SCN=0
	ssd1289_reg_set(item, SSD1289_REG_GATE_SCAN_START, 0x0000);

	// PKP1=7 PKP0=7
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_1, 0x0707);
	// PKP3=2 PKP2=4
	ssd1289_reg_set(item, SSD1289_REG_GAMME_CTRL_2, 0x0204);
	// PKP5=2 PKP4=2
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_3, 0x0204);
	// PRP1=5 PRP0=2
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_4, 0x0502);
	// PKN1=5 PKN0=7
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_5, 0x0507);
	// PKN3=2 PNN2=4
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_6, 0x0204);
	// PKN5=2 PKN4=4
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_7, 0x0204);
	// PRN1=5 PRN0=2
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_8, 0x0502);
	// VRP1=3 VRP0=2
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_9, 0x0302);
	// VRN1=3 VRN0=2
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_10, 0x0302);

	// WMR=0 WMG=0
	ssd1289_reg_set(item, SSD1289_REG_WR_DATA_MASK_1, 0x0000);
	// WMB=0
	ssd1289_reg_set(item, SSD1289_REG_WR_DATA_MASK_2, 0x0000);
	// OSC=b1010:548k
	ssd1289_reg_set(item, SSD1289_REG_FRAME_FREQUENCY, 0xa000);
	// SS1=0
	ssd1289_reg_set(item, SSD1289_REG_FIRST_WIN_START, 0x0000);
	// SE1=319
	ssd1289_reg_set(item, SSD1289_REG_FIRST_WIN_END,
			(item->info->var.xres - 1));
	// SS2=0
	ssd1289_reg_set(item, SSD1289_REG_SECND_WIN_START, 0x0000);
	// SE2=0
	ssd1289_reg_set(item, SSD1289_REG_SECND_WIN_END, 0x0000);
	// VL1=0
	ssd1289_reg_set(item, SSD1289_REG_V_SCROLL_CTRL_1, 0x0000);
	// VL2=0
	ssd1289_reg_set(item, SSD1289_REG_V_SCROLL_CTRL_2, 0x0000);
	// HEA=0xef=239 HSA=0
	ssd1289_reg_set(item, SSD1289_REG_H_RAM_ADR_POS,
			(item->info->var.yres - 1) << 8);
	// VSA=0
	ssd1289_reg_set(item, SSD1289_REG_V_RAM_ADR_START, 0x0000);
	// VEA=0x13f=319
	ssd1289_reg_set(item, SSD1289_REG_V_RAM_ADR_END,
			(item->info->var.xres - 1));
}

//This routine will allocate the buffer for the complete framebuffer. This
//is one continuous chunk of 16-bit pixel values; userspace programs
//will write here.
static int __init ssd1289_video_alloc(struct ssd1289 *item)
{
	unsigned int frame_size;

	dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

	frame_size = item->info->fix.line_length * item->info->var.yres;
	dev_dbg(item->dev, "%s: item=0x%p frame_size=%u\n",
		__func__, (void *)item, frame_size);

	item->pages_count = frame_size / PAGE_SIZE;
	if ((item->pages_count * PAGE_SIZE) < frame_size) {
		item->pages_count++;
	}
	dev_dbg(item->dev, "%s: item=0x%p pages_count=%u\n",
		__func__, (void *)item, item->pages_count);

	item->info->fix.smem_len = item->pages_count * PAGE_SIZE;
	item->info->fix.smem_start =
	    (unsigned long)vmalloc(item->info->fix.smem_len);
	if (!item->info->fix.smem_start) {
		dev_err(item->dev, "%s: unable to vmalloc\n", __func__);
		return -ENOMEM;
	}
	memset((void *)item->info->fix.smem_start, 0, item->info->fix.smem_len);

	return 0;
}

static void ssd1289_video_free(struct ssd1289 *item)
{
	dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

	// kfree((void *)item->info->fix.smem_start);
	vfree((void *)item->info->fix.smem_start);
}

//This routine will allocate a ssd1289_page struct for each vm page in the
//main framebuffer memory. Each struct will contain a pointer to the page
//start, an x- and y-offset, and the length of the pagebuffer which is in the framebuffer.
static int __init ssd1289_pages_alloc(struct ssd1289 *item)
{
	unsigned short pixels_per_page;
	unsigned short yoffset_per_page;
	unsigned short xoffset_per_page;
	unsigned short index;
	unsigned short x = 0;
	unsigned short y = 0;
	unsigned short *buffer;
	unsigned int len;

	dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

	item->pages = kmalloc(item->pages_count * sizeof(struct ssd1289_page),
			      GFP_KERNEL);
	if (!item->pages) {
		dev_err(item->dev, "%s: unable to kmalloc for ssd1289_page\n",
			__func__);
		return -ENOMEM;
	}

	pixels_per_page = PAGE_SIZE / (item->info->var.bits_per_pixel / 8);
	yoffset_per_page = pixels_per_page / item->info->var.xres;
	xoffset_per_page = pixels_per_page -
	    (yoffset_per_page * item->info->var.xres);
	dev_dbg(item->dev, "%s: item=0x%p pixels_per_page=%hu "
		"yoffset_per_page=%hu xoffset_per_page=%hu\n",
		__func__, (void *)item, pixels_per_page,
		yoffset_per_page, xoffset_per_page);

	buffer = (unsigned short *)item->info->fix.smem_start;
	for (index = 0; index < item->pages_count; index++) {
		len = (item->info->var.xres * item->info->var.yres) -
		    (index * pixels_per_page);
		if (len > pixels_per_page) {
			len = pixels_per_page;
		}
		dev_dbg(item->dev,
			"%s: page[%d]: x=%3hu y=%3hu buffer=0x%p len=%3hu\n",
			__func__, index, x, y, buffer, len);
		item->pages[index].x = x;
		item->pages[index].y = y;
		item->pages[index].buffer = buffer;
		item->pages[index].len = len;

		x += xoffset_per_page;
		if (x >= item->info->var.xres) {
			y++;
			x -= item->info->var.xres;
		}
		y += yoffset_per_page;
		buffer += pixels_per_page;
	}

	return 0;
}

static void ssd1289_pages_free(struct ssd1289 *item)
{
	dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

	// kfree(item->pages);
	vfree(item->pages);
}

static inline __u32 CNVT_TOHW(__u32 val, __u32 width)
{
	return ((val<<width) + 0x7FFF - val)>>16;
}

//This routine is needed because the console driver won't work without it.
static int ssd1289_setcolreg(unsigned regno,
			       unsigned red, unsigned green, unsigned blue,
			       unsigned transp, struct fb_info *info)
{
	int ret = 1;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				      7471 * blue) >> 16;
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;
			u32 value;

			red = CNVT_TOHW(red, info->var.red.length);
			green = CNVT_TOHW(green, info->var.green.length);
			blue = CNVT_TOHW(blue, info->var.blue.length);
			transp = CNVT_TOHW(transp, info->var.transp.length);

			value = (red << info->var.red.offset) |
				(green << info->var.green.offset) |
				(blue << info->var.blue.offset) |
				(transp << info->var.transp.offset);

			pal[regno] = value;
			ret = 0;
		}
		break;
	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}
	return ret;
}

static int ssd1289_blank(int blank_mode, struct fb_info *info)
{
	struct ssd1289 *item = (struct ssd1289 *)info->par;
	if (blank_mode == FB_BLANK_UNBLANK)
		item->backlight=1;
	else
		item->backlight=0;
	//Item->backlight won't take effect until the LCD is written to. Force that
	//by dirty'ing a page.
	item->pages[0].must_update=1;
	schedule_delayed_work(&info->deferred_work, 0);
	return 0;
}

static void ssd1289_touch(struct fb_info *info, int x, int y, int w, int h) 
{
	struct fb_deferred_io *fbdefio = info->fbdefio;
	struct ssd1289 *item = (struct ssd1289 *)info->par;
	int i, ystart, yend;
	if (fbdefio) {
		//Touch the pages the y-range hits, so the deferred io will update them.
		for (i=0; i<item->pages_count; i++) {
			ystart=item->pages[i].y;
			yend=item->pages[i].y+(item->pages[i].len/info->fix.line_length)+1;
			if (!((y+h)<ystart || y>yend)) {
				item->pages[i].must_update=1;
			}
		}
		//Schedule the deferred IO to kick in after a delay.
		schedule_delayed_work(&info->deferred_work, fbdefio->delay);
	}
}

static void ssd1289_fillrect(struct fb_info *p, const struct fb_fillrect *rect) 
{
	sys_fillrect(p, rect);
	ssd1289_touch(p, rect->dx, rect->dy, rect->width, rect->height);
}

static void ssd1289_imageblit(struct fb_info *p, const struct fb_image *image) 
{
	sys_imageblit(p, image);
	ssd1289_touch(p, image->dx, image->dy, image->width, image->height);
}

static void ssd1289_copyarea(struct fb_info *p, const struct fb_copyarea *area) 
{
	sys_copyarea(p, area);
	ssd1289_touch(p, area->dx, area->dy, area->width, area->height);
}

static ssize_t ssd1289_write(struct fb_info *p, const char __user *buf, 
				size_t count, loff_t *ppos) 
{
	ssize_t res;
	res = fb_sys_write(p, buf, count, ppos);
	ssd1289_touch(p, 0, 0, p->var.xres, p->var.yres);
	return res;
}

static struct fb_ops ssd1289_fbops = {
	.owner        = THIS_MODULE,
	.fb_read      = fb_sys_read,
	.fb_write     = ssd1289_write,
	.fb_fillrect  = ssd1289_fillrect,
	.fb_copyarea  = ssd1289_copyarea,
	.fb_imageblit = ssd1289_imageblit,
	.fb_setcolreg	= ssd1289_setcolreg,
	.fb_blank	= ssd1289_blank,
};

static struct fb_fix_screeninfo ssd1289_fix __initdata = {
	.id          = "SSD1289",
	.type        = FB_TYPE_PACKED_PIXELS,
	.visual      = FB_VISUAL_TRUECOLOR,
	.accel       = FB_ACCEL_NONE,
	.line_length = 320 * 2,
};

static struct fb_var_screeninfo ssd1289_var __initdata = {
	.xres		= 320,
	.yres		= 240,
	.xres_virtual	= 320,
	.yres_virtual	= 240,
	.width		= 320,
	.height		= 240,
	.bits_per_pixel	= 16,
	.red		= {11, 5, 0},
	.green		= {5, 6, 0},
	.blue		= {0, 5, 0},
	.activate	= FB_ACTIVATE_NOW,
	.vmode		= FB_VMODE_NONINTERLACED,
};

static struct fb_deferred_io ssd1289_defio = {
        .delay          = HZ / 20,
        .deferred_io    = &ssd1289_update,
};

#if defined(CONFIG_SSD1289_DIRECTIO_MODE)
// static int __init ssd1289_probe(struct platform_device *dev)
static int __devinit ssd1289_probe(struct platform_device *dev)
#elif defined(CONFIG_SSD1289_SPI_MODE)
// static int __init ssd1289_probe(struct spi_device *dev)
static int __devinit ssd1289_probe(struct spi_device *dev)
#endif
{
	int ret = 0;
	struct ssd1289 *item;
#if defined(CONFIG_SSD1289_DIRECTIO_MODE)
	struct resource *ctrl_res;
	struct resource *data_res;
	unsigned int ctrl_res_size;
	unsigned int data_res_size;
	struct resource *ctrl_req;
	struct resource *data_req;
	unsigned short signature;
#endif
	struct fb_info *info;

	dev_dbg(&dev->dev, "%s\n", __func__);

	item = kzalloc(sizeof(struct ssd1289), GFP_KERNEL);
	if (!item) {
		dev_err(&dev->dev,
			"%s: unable to kzalloc for ssd1289\n", __func__);
		ret = -ENOMEM;
		goto out;
	}
	item->dev = &dev->dev;
	dev_set_drvdata(&dev->dev, item);
	item->backlight=1;

#if defined(CONFIG_SSD1289_DIRECTIO_MODE)
	ctrl_res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!ctrl_res) {
		dev_err(&dev->dev,
			"%s: unable to platform_get_resource for ctrl_res\n",
			__func__);
		ret = -ENOENT;
		goto out_item;
	}
	ctrl_res_size = ctrl_res->end - ctrl_res->start + 1;
	ctrl_req = request_mem_region(ctrl_res->start, ctrl_res_size,
				      dev->name);
	if (!ctrl_req) {
		dev_err(&dev->dev,
			"%s: unable to request_mem_region for ctrl_req\n",
			__func__);
		ret = -EIO;
		goto out_item;
	}
	item->ctrl_io = ioremap(ctrl_res->start, ctrl_res_size);
	if (!item->ctrl_io) {
		ret = -EINVAL;
		dev_err(&dev->dev,
			"%s: unable to ioremap for ctrl_io\n", __func__);
		goto out_item;
	}

	data_res = platform_get_resource(dev, IORESOURCE_MEM, 1);
	if (!data_res) {
		dev_err(&dev->dev,
			"%s: unable to platform_get_resource for data_res\n",
			__func__);
		ret = -ENOENT;
		goto out_item;
	}
	data_res_size = data_res->end - data_res->start + 1;
	data_req = request_mem_region(data_res->start,
				      data_res_size, dev->name);
	if (!data_req) {
		dev_err(&dev->dev,
			"%s: unable to request_mem_region for data_req\n",
			__func__);
		ret = -EIO;
		goto out_item;
	}
	item->data_io = ioremap(data_res->start, data_res_size);
	if (!item->data_io) {
		ret = -EINVAL;
		dev_err(&dev->dev,
			"%s: unable to ioremap for data_io\n", __func__);
		goto out_item;
	}

	dev_dbg(&dev->dev, "%s: ctrl_io=%p data_io=%p\n",
		__func__, item->ctrl_io, item->data_io);

	signature = readw(item->ctrl_io);
	dev_dbg(&dev->dev, "%s: signature=0x%04x\n", __func__, signature);
	if (signature != 0x8989) {
		ret = -ENODEV;
		dev_err(&dev->dev,
			"%s: unknown signature 0x%04x\n", __func__, signature);
		goto out_item;
	}

	dev_info(&dev->dev, "item=0x%p ctrl=0x%p data=0x%p\n", (void *)item,
		 (void *)ctrl_res->start, (void *)data_res->start);
#elif defined(CONFIG_SSD1289_SPI_MODE)
	item->spidev=dev;
	item->dev=&dev->dev;
	dev_set_drvdata(&dev->dev, item);
	dev_info(&dev->dev, "spi registered, item=0x%p\n", (void *)item);
#endif

	info = framebuffer_alloc(sizeof(struct ssd1289), &dev->dev);
	if (!info) {
		ret = -ENOMEM;
		dev_err(&dev->dev,
			"%s: unable to framebuffer_alloc\n", __func__);
		goto out_item;
	}
	info->pseudo_palette = &item->pseudo_palette;
	item->info = info;
	info->par = item;
	info->dev = &dev->dev;
	info->fbops = &ssd1289_fbops;
	info->flags = FBINFO_FLAG_DEFAULT|FBINFO_VIRTFB;
	info->fix = ssd1289_fix;
	info->var = ssd1289_var;

	ret = ssd1289_video_alloc(item);
	if (ret) {
		dev_err(&dev->dev,
			"%s: unable to ssd1289_video_alloc\n", __func__);
		goto out_info;
	}
	info->screen_base = (char __iomem *)item->info->fix.smem_start;

	ret = ssd1289_pages_alloc(item);
	if (ret < 0) {
		dev_err(&dev->dev,
			"%s: unable to ssd1289_pages_init\n", __func__);
		goto out_video;
	}

	info->fbdefio = &ssd1289_defio;
	fb_deferred_io_init(info);

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&dev->dev,
			"%s: unable to register_frambuffer\n", __func__);
		goto out_pages;
	}

	ssd1289_setup(item);
	ssd1289_update_all(item);

	return ret;

out_pages:
	ssd1289_pages_free(item);
out_video:
	ssd1289_video_free(item);
out_info:
	framebuffer_release(info);
out_item:
	// kfree(item);
	vfree(item);
out:
	return ret;
}


// static int ssd1289_remove(struct spi_device *spi)
// {
	// struct fb_info *info = dev_get_drvdata(&spi->dev);
	// struct ssd1289 *item = (struct ssd1289 *)info->par;
#if defined(CONFIG_SSD1289_DIRECTIO_MODE)
	//ToDo: directio-mode: shouldn't those resources be free()'ed too?
	static int __devexit ssd1289_remove(struct platform_device *dev)
#elif defined(CONFIG_SSD1289_SPI_MODE)
	// dev_set_drvdata(&spi->dev, NULL);
	static int __devexit ssd1289_remove(struct spi_device *dev)
#endif
	// unregister_framebuffer(info);
	// ssd1289_pages_free(item);
	// ssd1289_video_free(item);
	// framebuffer_release(info);
	// kfree(item);
{
   struct ssd1289 *item = dev_get_drvdata(&dev->dev);
   struct fb_info *info;

   dev_dbg(&dev->dev, "%s\n", __func__);

   dev_set_drvdata(&dev->dev, NULL);
   if (item) {
           info = item->info;
           if (info)
                   unregister_framebuffer(info);
           ssd1289_pages_free(item);
           ssd1289_video_free(item);
           // kfree(item);
           vfree(item);
           if (info)
                   framebuffer_release(info);
   }
	return 0;
}

#ifdef CONFIG_PM
static int ssd1289_suspend(struct spi_device *spi, pm_message_t state)
{
	struct fb_info *info = dev_get_drvdata(&spi->dev);
	struct ssd1289 *item = (struct ssd1289 *)info->par;
	/* enter into sleep mode */
	ssd1289_reg_set(item, SSD1289_REG_SLEEP_MODE, 0x0001);
	return 0;
}

static int ssd1289_resume(struct spi_device *spi)
{
	struct fb_info *info = dev_get_drvdata(&spi->dev);
	struct ssd1289 *item = (struct ssd1289 *)info->par;
	/* leave sleep mode */
	ssd1289_reg_set(item, SSD1289_REG_SLEEP_MODE, 0x0000);
	return 0;
}
#else
#define ssd1289_suspend NULL
#define ssd1289_resume NULL
#endif

#if defined(CONFIG_SSD1289_DIRECTIO_MODE)
static struct platform_driver ssd1289_driver = {
	.probe = ssd1289_probe,
	.driver = {
		   .name = "ssd1289",
		   },
};
#elif defined(CONFIG_SSD1289_SPI_MODE)
static struct spi_driver spi_ssd1289_driver = {
	.driver = {
		.name	= "spi-ssd1289",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe = ssd1289_probe,
	.remove = ssd1289_remove,
	.suspend = ssd1289_suspend,
	.resume = ssd1289_resume,
};
#endif

static int __init ssd1289_init(void)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

#if defined(CONFIG_SSD1289_DIRECTIO_MODE)
	ret = platform_driver_register(&ssd1289_driver);
#elif defined(CONFIG_SSD1289_SPI_MODE)
	ret = spi_register_driver(&spi_ssd1289_driver);
#endif
	if (ret) {
		pr_err("%s: unable to platform_driver_register\n", __func__);
	}

	return ret;
}
static void __exit ssd1289_exit(void)
{
       pr_debug("%s\n", __func__);

#if defined(CONFIG_SSD1289_DIRECTIO_MODE)
       platform_driver_unregister(&ssd1289_driver);
#elif defined(CONFIG_SSD1289_SPI_MODE)
       spi_unregister_driver(&spi_ssd1289_driver);
#endif

}

module_init(ssd1289_init);
module_exit(ssd1289_exit);

MODULE_DESCRIPTION("SSD1289 LCD Driver");
MODULE_AUTHOR("Jeroen Domburg <jeroen@spritesmods.com>");
MODULE_LICENSE("GPL");

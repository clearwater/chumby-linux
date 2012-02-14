/*
 * Freescale STMP378X Samsung LMS350 LCD panel initialization
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#include <linux/regulator/consumer.h>

#include <mach/regs-lcdif.h>
#include <mach/regs-lradc.h>
#include <mach/regs-pinctrl.h>
#include <mach/regs-clkctrl.h>
#include <mach/regs-pwm.h>
#include <mach/regs-apbh.h>
#include <mach/gpio.h>
#include <mach/pins.h>
#include <mach/lcdif.h>
#include <mach/cpu.h>
#include <mach/stmp3xxx.h>

#include "common.h"

#define DOTCLK_H_ACTIVE  320
#define DOTCLK_H_PULSE_WIDTH 69
#define DOTCLK_HF_PORCH  5
#define DOTCLK_HB_PORCH  6
#define DOTCLK_H_WAIT_CNT  (DOTCLK_H_PULSE_WIDTH + DOTCLK_HB_PORCH)
#define DOTCLK_H_PERIOD (DOTCLK_H_WAIT_CNT + DOTCLK_HF_PORCH + DOTCLK_H_ACTIVE)

#define DOTCLK_V_PULSE_WIDTH  15
#define DOTCLK_V_ACTIVE  240
#define DOTCLK_VF_PORCH  3
#define DOTCLK_VB_PORCH  24
#define DOTCLK_V_WAIT_CNT (DOTCLK_V_PULSE_WIDTH + DOTCLK_VB_PORCH)
#define DOTCLK_V_PERIOD (DOTCLK_VF_PORCH + DOTCLK_V_ACTIVE + DOTCLK_V_WAIT_CNT)

static struct stmp3xxx_platform_bl_data bl_data;
static struct pin_group *lcd_pins;
static unsigned *lcd_spi_pins;

static void spi_write(u32 val)
{
	u32 mask;

	gpio_set_value(lcd_spi_pins[SPI_MOSI], 0);
	gpio_set_value(lcd_spi_pins[SPI_SCLK], 1);
	gpio_set_value(lcd_spi_pins[SPI_CS], 0);

	for (mask = 0x00800000; mask != 0; mask >>= 1) {
		gpio_set_value(lcd_spi_pins[SPI_SCLK], 0);
		if (val & mask)
			gpio_set_value(lcd_spi_pins[SPI_MOSI], 1);
		else
			gpio_set_value(lcd_spi_pins[SPI_MOSI], 0);

		gpio_set_value(lcd_spi_pins[SPI_SCLK], 1);
	}

	udelay(10);
	gpio_set_value(lcd_spi_pins[SPI_MOSI], 1);
	gpio_set_value(lcd_spi_pins[SPI_SCLK], 1);
	gpio_set_value(lcd_spi_pins[SPI_CS], 1);
}

static void write_reg(u16 reg, u16 val)
{
#if 0
	pr_debug("%s: writing %x to %x\n", __func__, reg, val);
	spi_write(0x00740000 | reg);
	spi_write(0x00760000 | val);
#else
	printk( "write_reg not supported for Nanovision LCD implementation\n" );
#endif

}

static const unsigned short pon_seq[] = {
	/* power on */
	0x07, 0x0000,  20,
	0x12, 0x1618,   0,
	0x11, 0x222f,   0,
	0x13, 0x40ca,   0,
	0x10, 0x3108, 300,
	0x12, 0x1658, 250,
	0x01, 0x2b1d,   0,
	0x02, 0x0300,   0,
	0x03, 0xD040,   0,
	0x08, (DOTCLK_VB_PORCH + DOTCLK_V_PULSE_WIDTH) - 2,   0,
	0x09, ((DOTCLK_H_PULSE_WIDTH / 3) + DOTCLK_HB_PORCH) - 2,   0,
	0x76, 0x2213,   0,
	0x0b, 0x33e1,   0,
	0x0c, 0x0020,   0,
	0x76, 0x0000,   0,
	0x0d, 0x0000,   0,
	0x0e, 0x0000,   0,
	0x14, 0x0000,   0,
	0x15, 0x0803,   0,
	0x16, 0x0000,   0,
	0x30, 0x0209,   0,
	0x31, 0x0404,   0,
	0x32, 0x0e07,   0,
	0x33, 0x0602,   0,
	0x34, 0x0707,   0,
	0x35, 0x0707,   0,
	0x36, 0x0707,   0,
	0x37, 0x0206,   0,
	0x38, 0x0f06,   0,
	0x39, 0x0611,  20,
};

static const unsigned short don_seq[] = {
	/* display on */
	0x07, 0x0001, 150,
	0x07, 0x0101, 150,
	0x76, 0x2213,   0,
	0x1c, 0x6650,   0,
	0x0b, 0x33e0,   0,
	0x76, 0x0000,   0,
	0x07, 0x0103,   0,
};


static const unsigned short doff_seq[] = {
	/* display off */
	0x0b, 0x33e1,   0,
	0x07, 0x0102, 150,
	0x07, 0x0100, 150,
	0x12, 0x0000,   0,
	0x10, 0x0000,   0,
};

static const unsigned short poff_seq[] = {
	/* power off */
	/* called after display off */
	0x07, 0x0000,    0,
	0x10, 0x0000,    0,
	0x11, 0x0000,    0,
};

static const unsigned short sby_seq[] = {
	/* standby */
	/* called after display off */
	0x10, 0x0001,     0
};

static const unsigned short csby_seq[] = {
	/* cancel standby */
	/* called after display on */
	0x10, 0x0000,     0
};

static void display_off(void)
{
	int i;
	const unsigned short *seq;

#if 0
	seq = doff_seq;
	for (i = 0; i < ARRAY_SIZE(doff_seq); i += 3) {
		write_reg(seq[i], seq[i + 1]);
		if (seq[i + 2])
			udelay(seq[i + 2]);
	}
#else
	printk( "unsupported display_off call\n" );
#endif
}

static void display_on(void)
{
#if 0
	int i;
	const unsigned short *seq;

	seq = don_seq;
	for (i = 0; i < ARRAY_SIZE(don_seq); i += 3) {
		write_reg(seq[i], seq[i + 1]);
		if (seq[i + 2])
			udelay(seq[i + 2]);
	}
#else
	printk( "unsupported display_on call\n" );
#endif
}

static void power_off(void)
{
	int i;
	const unsigned short *seq;

#if 0
	seq = poff_seq;
	for (i = 0; i < ARRAY_SIZE(poff_seq); i += 3) {
		write_reg(seq[i], seq[i + 1]);
		if (seq[i + 2])
			udelay(seq[i + 2]);
	}
#else
	printk( "unsupported power_off call\n" );
#endif

}

static void init_panel_hw(void)
{
	int i;
	const unsigned short *seq;

#if 0
	seq = pon_seq;
	for (i = 0; i < ARRAY_SIZE(pon_seq); i += 3) {
		write_reg(seq[i], seq[i + 1]);
		if (seq[i + 2])
			udelay(seq[i + 2]);
	}
#else
	printk( "Unsupported init_panel_hw call\n" );
#endif
	display_on();
}

static int init_pinmux(void)
{
	int ret = -EINVAL;

	if (cpu_is_stmp37xx())
		lcd_pins = &stmp37xx_lcd_pins;
	else if (cpu_is_stmp378x())
		lcd_pins = &stmp378x_lcd_pins;
	else
		goto out;
	ret = stmp3xxx_request_pin_group(lcd_pins, "lcd_lms350");

out:
	return ret;
}

static int init_pinmux_spi(void)
{
	int ret = -EINVAL;

	if (cpu_is_stmp37xx())
		lcd_spi_pins = stmp37xx_lcd_spi_pins;
	else if (cpu_is_stmp378x())
		lcd_spi_pins = stmp378x_lcd_spi_pins;
	else
		goto out_1;

	ret = gpio_request(lcd_spi_pins[SPI_MOSI], "lcd_lms350");
	if (ret)
		goto out_1;

	ret = gpio_request(lcd_spi_pins[SPI_SCLK], "lcd_lms350");
	if (ret)
		goto out_2;
	ret = gpio_request(lcd_spi_pins[SPI_CS], "lcd_lms350");
	if (ret)
		goto out_3;

	/* Enable these pins as outputs */
	gpio_direction_output(lcd_spi_pins[SPI_MOSI], 1);
	gpio_direction_output(lcd_spi_pins[SPI_SCLK], 1);
	gpio_direction_output(lcd_spi_pins[SPI_CS], 1);

	return 0;
out_3:
	gpio_free(lcd_spi_pins[SPI_SCLK]);
out_2:
	gpio_free(lcd_spi_pins[SPI_MOSI]);
out_1:
	return ret;
}

static void uninit_pinmux(void)
{
	stmp3xxx_release_pin_group(lcd_pins, "lcd_lms350");
}

static void uninit_pinmux_spi(void)
{
	gpio_free(lcd_spi_pins[SPI_MOSI]);
	gpio_free(lcd_spi_pins[SPI_SCLK]);
	gpio_free(lcd_spi_pins[SPI_CS]);
}

static struct clk *lcd_clk;

static int init_panel(struct device *dev, dma_addr_t phys, int memsize,
		struct stmp3xxx_platform_fb_entry *pentry)
{
	int ret = 0;

	lcd_clk = clk_get(dev, "lcdif");
	if (IS_ERR(lcd_clk)) {
		ret = PTR_ERR(lcd_clk);
		goto out_1;
	}
	ret = clk_enable(lcd_clk);
	if (ret) {
		clk_put(lcd_clk);
		goto out_1;
	}
	ret = clk_set_rate(lcd_clk, 1000000/pentry->cycle_time_ns); /* kHz */
	if (ret) {
		clk_disable(lcd_clk);
		clk_put(lcd_clk);
		goto out_1;
	}

    // Code disabled, as the panel is already initialized in the
    // bootloader.
#if 0
	/*
	 * Make sure we do a high-to-low transition to reset the panel.
	 * First make it low for 100 msec, hi for 10 msec, low for 10 msec,
	 * then hi.
	 */
	HW_LCDIF_CTRL1_CLR(BM_LCDIF_CTRL1_RESET); /* low */
	mdelay(100);
	HW_LCDIF_CTRL1_SET(BM_LCDIF_CTRL1_RESET); /* high */
	mdelay(10);
	HW_LCDIF_CTRL1_CLR(BM_LCDIF_CTRL1_RESET); /* low */

	/* For the Samsung, Reset must be held low at least 30 uSec
	 * Therefore, we'll hold it low for about 10 mSec just to be sure.
	 * Then we'll wait 1 mSec afterwards.
	 */
	mdelay(10);
	HW_LCDIF_CTRL1_SET(BM_LCDIF_CTRL1_RESET); /* high */
	mdelay(1);
#endif

	ret = init_pinmux();
	if (ret)
		goto out_1;

	setup_dotclk_panel(DOTCLK_V_PULSE_WIDTH, DOTCLK_V_PERIOD,
			DOTCLK_V_WAIT_CNT, DOTCLK_V_ACTIVE,
			DOTCLK_H_PULSE_WIDTH, DOTCLK_H_PERIOD,
			DOTCLK_H_WAIT_CNT, DOTCLK_H_ACTIVE, 1);

	ret = stmp3xxx_lcdif_dma_init(dev, phys, memsize, 1);
	if (ret)
		goto out_3;

	stmp3xxx_lcd_set_bl_pdata(pentry->bl_data);
	stmp3xxx_lcdif_notify_clients(STMP3XXX_LCDIF_PANEL_INIT, pentry);
	HW_LCDIF_CTRL_SET(BM_LCDIF_CTRL_RUN);

	printk( "Panel init finished.\n" );
	return 0;
out_3:
	//	uninit_pinmux_spi();
out_2:
	uninit_pinmux();
out_1:
	return ret;
}

static void release_panel(struct device *dev,
			  struct stmp3xxx_platform_fb_entry *pentry)
{
	stmp3xxx_lcdif_notify_clients(STMP3XXX_LCDIF_PANEL_RELEASE, pentry);
	display_off();
	power_off();
	//	uninit_pinmux_spi();
	uninit_pinmux();
	release_dotclk_panel();
	stmp3xxx_lcdif_dma_release();
	clk_disable(lcd_clk);
	clk_put(lcd_clk);
}

static int blank_panel(int blank)
{
	int ret = 0;

	switch (blank) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
	  printk( "blanking panel...\n" );
//		HW_LCDIF_CTRL_CLR(BM_LCDIF_CTRL_RUN);
		break;

	case FB_BLANK_UNBLANK:
	  printk( "unblacking panel...\n" );
//		HW_LCDIF_CTRL_SET(BM_LCDIF_CTRL_RUN);
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}

static void stop_panel(void)
{
	stmp3xxx_lcdif_stop();
	display_off();
}

static void run_panel(void)
{
	display_on();
	stmp3xxx_lcdif_run();
}

static struct stmp3xxx_platform_fb_entry fb_entry = {
	.name		= "lms350",
	.x_res		= 320,
	.y_res		= 240,
	.bpp		= 16,
	.cycle_time_ns	= 154,
//	.cycle_time_ns	= 303, // SMC changed to higher value for a lower frequency
	.lcd_type	= STMP3XXX_LCD_PANEL_DOTCLK,
	.init_panel	= init_panel,
	.release_panel	= release_panel,
	.blank_panel	= NULL,//blank_panel,
	.run_panel	= run_panel,
	.stop_panel	= stop_panel,
	.pan_display	= stmp3xxx_lcdif_pan_display,
	.bl_data	= &bl_data,
};

static struct clk *pwm_clk;

static int init_bl(struct stmp3xxx_platform_bl_data *data)
{
	int ret = 0;

	printk( "init_bl\n" );
	pwm_clk = clk_get(NULL, "pwm");
	if (IS_ERR(pwm_clk)) {
		ret = PTR_ERR(pwm_clk);
		goto out;
	}
	clk_enable(pwm_clk);
	stmp3xxx_reset_block(REGS_PWM_BASE, 1);

	ret = stmp3xxx_request_pin(PINID_PWM2, PIN_FUN1, "lcd_lms350");
	if (ret)
		goto out_mux;

	stmp3xxx_pin_voltage(PINID_PWM2, PIN_4MA, "lcd_lms350");
	stmp3xxx_pin_strength(PINID_PWM2, PIN_3_3V, "lcd_lms350");

    // The backlight needs to be run from 60Hz to 700Hz, but a good middle
    // frequency is 100Hz.
    // Since the PWM is slaved off of the 24 MHz crystal, we'll divide it
    // by 64 (by settin ghte CDIV to 6), then set the period to 3750.
    // Then then active- and inactive-states can be measured as percentages.
	HW_PWM_ACTIVEn_WR(2, BF_PWM_ACTIVEn_INACTIVE(100) | // full bright, original code had at 0 probably for battery issues (bunnie)
				BF_PWM_ACTIVEn_ACTIVE(0));
	HW_PWM_PERIODn_WR(2,
			BF_PWM_PERIODn_CDIV(6) | /* divide by 64 */
			BF_PWM_PERIODn_INACTIVE_STATE(2) | /* low */
			BF_PWM_PERIODn_ACTIVE_STATE(3) | /* high */
			BF_PWM_PERIODn_PERIOD(1337));
	HW_PWM_CTRL_SET(BM_PWM_CTRL_PWM2_ENABLE);

	printk( "init_bl finished\n" );

	return 0;

out_mux:
	clk_put(pwm_clk);
out:
	return ret;
}

static void free_bl(struct stmp3xxx_platform_bl_data *data)
{
	HW_PWM_ACTIVEn_WR(2, BF_PWM_ACTIVEn_INACTIVE(0) |
				BF_PWM_ACTIVEn_ACTIVE(0));
	HW_PWM_PERIODn_WR(2,
		BF_PWM_PERIODn_CDIV(6) | /* divide by 64 */
		BF_PWM_PERIODn_INACTIVE_STATE(2) | /* low */
		BF_PWM_PERIODn_ACTIVE_STATE(3) | /* high */
		BF_PWM_PERIODn_PERIOD(1337));
	HW_PWM_CTRL_CLR(BM_PWM_CTRL_PWM2_ENABLE);
	stmp3xxx_pin_voltage(PINID_PWM2, PIN_4MA, "lcd_lms350");
	stmp3xxx_pin_strength(PINID_PWM2, PIN_3_3V, "lcd_lms350");

	stmp3xxx_release_pin(PINID_PWM2, "lcd_lms350");
	clk_disable(pwm_clk);
	clk_put(pwm_clk);
}

static int values[] = { 6, 9, 12, 15, 19, 24, 30, 40, 55, 75, 100 };
static int power[] = {
	0, 1500, 3600, 6100, 10300,
	15500, 74200, 114200, 155200,
	190100, 191000
};

static int  bl_to_power(int br)
{
	int base;
	int rem;

	if (br > 100)
		br = 100;
	base = power[br/10];
	rem = br % 10;
	if (!rem)
		return base;
	else
		return base + (rem * (power[br/10 + 1]) - base) / 10;
}

static int set_bl_intensity(struct stmp3xxx_platform_bl_data *data,
			struct backlight_device *bd, int suspended)
{
	int intensity     = bd->props.brightness;
    int max_intensity = bd->props.max_brightness;
	int scaled_int;
	printk( "set_bl_intensity with %d\n", intensity );
    if(max_intensity < 100) {
        intensity = (intensity * max_intensity) / 100;
        printk("But limiting brightness to %d\n", max_intensity);
    }

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
    /* Disable framebuffer blanking
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
    */
	if (suspended)
		intensity = 0;

	printk( "intensity after modifiers: %d\n", intensity );

	/*
	 * This is not too cool but what can we do?
	 * Luminance changes non-linearly...
	 */
	// don't check power limits for now -- bunnie revisit
	//	if (reg_set_current(data->regulator, bl_to_power(intensity))) {
	//	  printk( "power limit exceeded.\n" );
	//		return -EBUSY;
	//	}

    // Clamp intensity to (0, 100).
    if(intensity > 100)
        intensity = 100;
    if(intensity < 0)
        intensity = 0;

    // Perform a brightness transformation function.
	if(chumby_revision() == 7)
		// On BBY hardware, make lower values brighter.
		intensity = 100-((intensity-100)*(intensity-100))/100;
	else
		// On other hardware, make lower values darker.
		intensity = (intensity*intensity)/100;


    // This 1337 number is a magic number. This should at least be recoded with a #define
    scaled_int = intensity * 1337;
    scaled_int = scaled_int / 100;

	printk( "setting with parameters %d\n", scaled_int );
    // The waveform will begin as being Low, and will bebecome Inactive after
    // the scaled period.
	HW_PWM_ACTIVEn_WR(2,
		BF_PWM_ACTIVEn_INACTIVE(scaled_int) |
		BF_PWM_ACTIVEn_ACTIVE(0));
	HW_PWM_PERIODn_WR(2,
		BF_PWM_PERIODn_CDIV(6) | /* divide by 64 */
		BF_PWM_PERIODn_INACTIVE_STATE(2) | /* low */
		BF_PWM_PERIODn_ACTIVE_STATE(3) | /* high */
		BF_PWM_PERIODn_PERIOD(1337));
	printk( "done.\n" );
	return 0;
}

static struct stmp3xxx_platform_bl_data bl_data = {
	.bl_max_intensity	= 100,
	.bl_default_intensity	= 100, // 50
	.bl_cons_intensity      = 100, // 50
	.init_bl		= init_bl,
	.free_bl		= free_bl,
	.set_bl_intensity	= set_bl_intensity,
};

static int __init register_devices(void)
{
	stmp3xxx_lcd_register_entry(&fb_entry,
				    stmp3xxx_framebuffer.dev.platform_data);
	return 0;
}
subsys_initcall(register_devices);

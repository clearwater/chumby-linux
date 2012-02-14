/*
 * Freescale STMP3XXX Rotary Encoder Driver
 *
 * Author: Drew Benedetti <drewb@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input-polldev.h>
#include <mach/regs-timrot.h>
#include <mach/rotdec.h>

static int absolute = 0;
static unsigned int poll_interval = 50;
static int previous_state = 0;
static int accumulated_counts = 0;
static int last_direction = 0;
static int output_divider = 2;

void stmp3xxx_rotdec_flush(struct input_polled_dev *dev)
{
	/* in relative mode, reading the counter resets it */
	if (!absolute)
		HW_TIMROT_ROTCOUNT_RD();
}

void stmp3xxx_rotdec_poll(struct input_polled_dev *dev)
{
	unsigned int new_state = (HW_TIMROT_ROTCTRL_RD()&BM_TIMROT_ROTCTRL_STATE)>>BP_TIMROT_ROTCTRL_STATE;
	s16 cnt;
	
	// hardware only counts full rotations
	cnt = HW_TIMROT_ROTCOUNT_RD() & BM_TIMROT_ROTCOUNT_UPDOWN;
	
	// upward counting state order is 0 1 3 2, counts up at 0
	// downward counting state order is 0 2 3 1, counts doown at 0
	
	if (!absolute && !((previous_state | new_state)&~3)) {
	    switch((previous_state<<4)|new_state) {
			case 0x00:
				// if we weren't already moving, more likely edge glitching
				// than a rotation by 4
				if (cnt>0)
					cnt = (cnt-1)*4;
				else if (cnt<0)
					cnt = (cnt+1)*4;
				else
					cnt = cnt*4;
				break;

			case 0x11:
				if (cnt<0)
					cnt = (cnt+1)*4;
				else
					cnt = cnt*4;
				break;
			case 0x33:
				cnt = cnt*4;
				break;
			case 0x22:
				if (cnt>0)
					cnt = (cnt-1)*4;
				else
					cnt = cnt*4;
				break;

			/* Counting up */
			case 0x01:
			case 0x13:
			case 0x32:
				cnt = cnt*4+1;
				break;
			case 0x20:
				if (cnt > 0)
					cnt = cnt*4-3;
				else
					cnt = cnt*4+1;
				break;

			/* Counting down */
			case 0x02:
			case 0x23:
			case 0x31:
				cnt = cnt*4-1;
				break;
			case 0x10:
				if (cnt < 0)
					cnt = cnt*4+3;
				else
					cnt = cnt*4-1;
				break;

			/* Other counts of some sort */
			case 0x03:
				// no way to tell which direction if we weren't already moving
				// or moved more than one full rotation
				if (cnt < 0)
					cnt = cnt*4 - 2;
				else if (cnt > 0)
					cnt = cnt*4 + 2;
				else
					cnt = cnt*4 + last_direction*2;
				break;
			case 0x12:
				cnt = cnt*4+2;
				break;
			case 0x30:
				if (cnt < 0)
					cnt = cnt*4+2;
				else
					cnt = cnt*4-2;
				break;
			case 0x21:
				cnt = cnt*4-2;
				break;
	    }

	    if (cnt > 0)
			last_direction = 1;
	    else if (cnt < 0)
			last_direction = -1;
	    else
			last_direction = 0;
	    accumulated_counts += cnt;
	    
		if (accumulated_counts < 0) {
			cnt = -(-accumulated_counts/output_divider);
		} else {
			cnt = accumulated_counts/output_divider;
		}
		accumulated_counts -= output_divider*cnt;
	}
	previous_state = new_state;
	if (!absolute)
		input_report_rel(dev->input, REL_WHEEL, cnt);
	else
		input_report_abs(dev->input, ABS_WHEEL, cnt);
}

struct input_polled_dev *rotdec;
static u32 rotctrl;

static int stmp3xxx_rotdec_probe(struct platform_device *pdev)
{
	int rc = 0;

	/* save original state of HW_TIMROT_ROTCTRL */
	rotctrl = HW_TIMROT_ROTCTRL_RD();

	if (!(rotctrl & BM_TIMROT_ROTCTRL_ROTARY_PRESENT)) {
		dev_info(&pdev->dev, "No rotary decoder present\n");
		rc = -ENODEV;
		goto err_rotdec_present;
	} else {
		/* I had to add some extra line breaks in here
		 * to avoid lines >80 chars wide
		 */
		HW_TIMROT_ROTCTRL_WR(
		 //BF_TIMROT_ROTCTRL_DIVIDER(0x0) | /* 32kHz divider - 1 */
		 BF_TIMROT_ROTCTRL_DIVIDER(0x0F) | /* 32/(15+1) = 2kHz sampling */
		 BF_TIMROT_ROTCTRL_OVERSAMPLE(
			BV_TIMROT_ROTCTRL_OVERSAMPLE__2X) |
		 BF_TIMROT_ROTCTRL_SELECT_B(
			BV_TIMROT_ROTCTRL_SELECT_B__ROTARYB) |
		 BF_TIMROT_ROTCTRL_SELECT_A(
			BV_TIMROT_ROTCTRL_SELECT_A__ROTARYA)
		);
		HW_TIMROT_ROTCTRL_CLR(
		 BM_TIMROT_ROTCTRL_POLARITY_B |
		 BM_TIMROT_ROTCTRL_POLARITY_A
		);

		if (!absolute)
			HW_TIMROT_ROTCTRL_SET(BM_TIMROT_ROTCTRL_RELATIVE);
		else
			HW_TIMROT_ROTCTRL_CLR(BM_TIMROT_ROTCTRL_RELATIVE);

		rc = rotdec_pinmux_request();
		if (rc) {
			dev_err(&pdev->dev,
				"Pin request failed (err=%d)\n", rc);
			goto err_pinmux;
		}

		/* set up input_polled_dev */
		rotdec = input_allocate_polled_device();
		if (!rotdec) {
			dev_err(&pdev->dev,
				"Unable to allocate polled device\n");
			rc = -ENOMEM;
			goto err_alloc_polldev;
		}
		rotdec->flush = stmp3xxx_rotdec_flush;
		rotdec->poll = stmp3xxx_rotdec_poll;
		rotdec->poll_interval = poll_interval; /* msec */

		rotdec->input->name = "stmp3xxx-rotdec";
		if (!absolute)
			input_set_capability(rotdec->input, EV_REL, REL_WHEEL);
		else {
			input_set_capability(rotdec->input, EV_ABS, ABS_WHEEL);
			input_set_abs_params(rotdec->input, ABS_WHEEL,
					-32768, 32767, 0, 0);
		}

		rc = input_register_polled_device(rotdec);
		if (rc) {
			dev_err(&pdev->dev,
				"Unable to register rotary decoder (err=%d)\n",
				rc);
			goto err_reg_polldev;
		}
		previous_state = (HW_TIMROT_ROTCTRL_RD()&BM_TIMROT_ROTCTRL_STATE)>>BP_TIMROT_ROTCTRL_STATE;
	}

	return 0;

err_reg_polldev:
	input_free_polled_device(rotdec);
err_alloc_polldev:
	rotdec_pinmux_free();
err_pinmux:
	/* restore original register state */
	HW_TIMROT_ROTCTRL_WR(rotctrl);

err_rotdec_present:
	return rc;
}

static int stmp3xxx_rotdec_remove(struct platform_device *pdev)
{
	input_unregister_polled_device(rotdec);
	input_free_polled_device(rotdec);

	rotdec_pinmux_free();

	/* restore original register state */
	HW_TIMROT_ROTCTRL_WR(rotctrl);

	return 0;
}

static struct platform_driver stmp3xxx_rotdec_driver = {
	.probe	= stmp3xxx_rotdec_probe,
	.remove	= stmp3xxx_rotdec_remove,
	.driver	= {
		.name	= "stmp3xxx-rotdec",
	},
};

static int __init stmp3xxx_rotdec_init(void)
{
	return platform_driver_register(&stmp3xxx_rotdec_driver);
}

static void __exit stmp3xxx_rotdec_exit(void)
{
	platform_driver_unregister(&stmp3xxx_rotdec_driver);
}

module_init(stmp3xxx_rotdec_init);
module_exit(stmp3xxx_rotdec_exit);

module_param(absolute, bool, 0600);
module_param(poll_interval, uint, 0600);
module_param(output_divider, uint, 0600);

MODULE_AUTHOR("Drew Benedetti <drewb@embeddedalley.com>");
MODULE_DESCRIPTION("STMP3xxx rotary decoder driver");
MODULE_LICENSE("GPL");

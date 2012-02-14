/*
 * Freesclae STMP37XX/STMP378X Touchscreen driver
 *
 * Author: Vitaly Wool <vital@embeddedalley.com>
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
/* #define DEBUG*/

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/input.h>
#include <linux/interrupt.h>

#include <mach/lradc.h>
#include <mach/hardware.h>
#include <mach/regs-lradc.h>
#include <mach/cpu.h>

#define TOUCH_DEBOUNCE_TOLERANCE	100



// Boards after revision 6 had their Y axis flipped.
static int should_flip_y_axis(void)
{
	return chumby_revision() > 6;
}


static int scaled_touchscreen = 0;
#define X_MIN 3930
#define X_MAX 125
#define Y_MIN 130
#define Y_MAX 4000
#define SCREEN_W 320
#define SCREEN_H 240
static inline int axis_to_screen(int raw, int scale, int min, int max) {
    int flip = 0;
    if(min > max) {
        int tmp;
        tmp  = max;
        max  = min;
        min  = tmp;
        flip = 1;
    }

    raw -= min;
    raw = raw / (max / scale);

    if(!flip)
        raw = scale - raw;

    return raw;
}



struct stmp3xxx_ts_info {
	int touch_irq;
	int device_irq;
	struct input_dev *idev;
	enum {
		TS_STATE_DISABLED,
		TS_STATE_TOUCH_DETECT,
		TS_STATE_TOUCH_VERIFY,
		TS_STATE_X_PLANE,
		TS_STATE_Y_PLANE,
	} state;
	u16 x;
	u16 y;
	int sample_count;
};

static inline void enter_state_touch_detect(struct stmp3xxx_ts_info *info)
{
	HW_LRADC_CHn_CLR(2, 0xFFFFFFFF);
	HW_LRADC_CHn_CLR(3, 0xFFFFFFFF);
	HW_LRADC_CHn_CLR(4, 0xFFFFFFFF);
	HW_LRADC_CHn_CLR(5, 0xFFFFFFFF);
	HW_LRADC_CTRL1_CLR(BM_LRADC_CTRL1_LRADC5_IRQ);
	HW_LRADC_CTRL1_CLR(BM_LRADC_CTRL1_TOUCH_DETECT_IRQ);
	/*
	 * turn off the yplus and yminus pullup and pulldown, and turn off touch
	 * detect (enables yminus, and xplus through a resistor.On a press,
	 * xplus is pulled down)
	 */
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_YMINUS_ENABLE);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_YPLUS_ENABLE);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_XMINUS_ENABLE);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_XPLUS_ENABLE);
	HW_LRADC_CTRL0_SET(BM_LRADC_CTRL0_TOUCH_DETECT_ENABLE);

	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_TOUCHSCREEN, 0);
	info->state = TS_STATE_TOUCH_DETECT;
	info->sample_count = 0;
}

static inline void enter_state_disabled(struct stmp3xxx_ts_info *info)
{
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_YMINUS_ENABLE);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_YPLUS_ENABLE);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_XMINUS_ENABLE);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_XPLUS_ENABLE);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_TOUCH_DETECT_ENABLE);

	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_TOUCHSCREEN, 0);
	info->state = TS_STATE_DISABLED;
	info->sample_count = 0;
}


static inline void enter_state_x_plane(struct stmp3xxx_ts_info *info)
{
	HW_LRADC_CTRL0_SET(BM_LRADC_CTRL0_YMINUS_ENABLE);
	HW_LRADC_CTRL0_SET(BM_LRADC_CTRL0_YPLUS_ENABLE);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_XMINUS_ENABLE);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_XPLUS_ENABLE);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_TOUCH_DETECT_ENABLE);

	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);

	info->state = TS_STATE_X_PLANE;
	info->sample_count = 0;
}

static inline void enter_state_y_plane(struct stmp3xxx_ts_info *info)
{
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_YMINUS_ENABLE);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_YPLUS_ENABLE);
	HW_LRADC_CTRL0_SET(BM_LRADC_CTRL0_XMINUS_ENABLE);
	HW_LRADC_CTRL0_SET(BM_LRADC_CTRL0_XPLUS_ENABLE);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_TOUCH_DETECT_ENABLE);

	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);
	info->state = TS_STATE_Y_PLANE;
	info->sample_count = 0;
}

static inline void enter_state_touch_verify(struct stmp3xxx_ts_info *info)
{
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_YMINUS_ENABLE);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_YPLUS_ENABLE);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_XMINUS_ENABLE);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_XPLUS_ENABLE);
	HW_LRADC_CTRL0_SET(BM_LRADC_CTRL0_TOUCH_DETECT_ENABLE);

	info->state = TS_STATE_TOUCH_VERIFY;
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);
	info->sample_count = 0;
}

static void process_lradc(struct stmp3xxx_ts_info *info, u16 x, u16 y,
			int pressure)
{
	switch (info->state) {
	case TS_STATE_X_PLANE:
		pr_debug("%s: x plane state, sample_count %d\n", __func__,
				info->sample_count);
		if (info->sample_count < 2) {
			info->x = x;
			info->sample_count++;
		} else {
			if (abs(info->x - x) > TOUCH_DEBOUNCE_TOLERANCE)
				info->sample_count = 1;
			else {
				u16 x_c = info->x * (info->sample_count - 1);
				info->x = (x_c + x) / info->sample_count;
				info->sample_count++;
			}
		}
		if (info->sample_count > 4)
			enter_state_y_plane(info);
		else
			hw_lradc_set_delay_trigger_kick(
					LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);
		break;

	case TS_STATE_Y_PLANE:
		pr_debug("%s: y plane state, sample_count %d\n", __func__,
				info->sample_count);
		if (info->sample_count < 2) {
			info->y = y;
			info->sample_count++;
		} else {
			if (abs(info->y - y) > TOUCH_DEBOUNCE_TOLERANCE)
				info->sample_count = 1;
			else {
				u16 y_c = info->y * (info->sample_count - 1);
				info->y = (y_c + y) / info->sample_count;
				info->sample_count++;
			}
		}
		if (info->sample_count > 4)
			enter_state_touch_verify(info);
		else
			hw_lradc_set_delay_trigger_kick(
					LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);
		break;

	case TS_STATE_TOUCH_VERIFY:
		{
			int temp;
			temp = info->y;
			info->y = info->x;
			info->x = temp;
		}

		pr_debug("%s: touch verify state, sample_count %d\n", __func__,
				info->sample_count);
		pr_debug("%s: x %d, y %d\n", __func__, info->x, info->y);


		if(should_flip_y_axis())
			info->y = 4096-info->y;
        if(scaled_touchscreen) {
            input_report_abs(info->idev, ABS_X,
                    axis_to_screen(info->x, SCREEN_W, X_MIN, X_MAX));
            input_report_abs(info->idev, ABS_Y,
                    axis_to_screen(info->y, SCREEN_H, Y_MIN, Y_MAX));
        }
        else {
            input_report_abs(info->idev, ABS_X, info->x);
            input_report_abs(info->idev, ABS_Y, info->y);
        }
		input_report_abs(info->idev, ABS_PRESSURE, pressure);
        input_report_key(info->idev, BTN_TOUCH, pressure);
		input_sync(info->idev);
		/* fall through */
	case TS_STATE_TOUCH_DETECT:
		pr_debug("%s: touch detect state, sample_count %d\n", __func__,
				info->sample_count);
		if (pressure) {
			input_report_abs(info->idev, ABS_PRESSURE, pressure);
			enter_state_x_plane(info);
			hw_lradc_set_delay_trigger_kick(
					LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);
		} else
			enter_state_touch_detect(info);
		break;

	default:
		printk(KERN_ERR "%s: unknown touchscreen state %d\n", __func__,
				info->state);
	}
}

static irqreturn_t ts_handler(int irq, void *dev_id)
{
	struct stmp3xxx_ts_info *info = dev_id;
	u16 x_plus, y_plus;
	int pressure = 0;

	if (irq == info->touch_irq)
		HW_LRADC_CTRL1_CLR(BM_LRADC_CTRL1_TOUCH_DETECT_IRQ);
	else if (irq == info->device_irq)
		HW_LRADC_CTRL1_CLR(BM_LRADC_CTRL1_LRADC5_IRQ);

	/* get x, y values */
	x_plus = HW_LRADC_CHn_RD(LRADC_TOUCH_X_PLUS) & BM_LRADC_CHn_VALUE;
	y_plus = HW_LRADC_CHn_RD(LRADC_TOUCH_Y_PLUS) & BM_LRADC_CHn_VALUE;

	/* pressed? */
	if (HW_LRADC_STATUS_RD() & BM_LRADC_STATUS_TOUCH_DETECT_RAW)
		pressure = 1;

	pr_debug("%s: irq %d, x_plus %d, y_plus %d, pressure %d\n",
			__func__, irq, x_plus, y_plus, pressure);

	process_lradc(info, x_plus, y_plus, pressure);

	return IRQ_HANDLED;
}

static int stmp3xxx_ts_probe(struct platform_device *pdev)
{
	struct input_dev *idev;
	struct stmp3xxx_ts_info *info;
	int ret = 0;
	struct resource *res;

	idev = input_allocate_device();
	info = kzalloc(sizeof(struct stmp3xxx_ts_info), GFP_KERNEL);
	if (idev == NULL || info == NULL) {
		ret = -ENOMEM;
		goto out_nomem;
	}

	idev->name = "STMP3XXX touchscreen";
	idev->phys = "stmp3xxx_ts/input0";
	idev->id.bustype = BUS_ISA;
	idev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	idev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);


    if(scaled_touchscreen) {
        input_set_abs_params(idev, ABS_X, 0, SCREEN_W, 0, 0);
        input_set_abs_params(idev, ABS_Y, 0, SCREEN_H, 0, 0);
    }
    else {
        input_set_abs_params(idev, ABS_X, 0, 0xFFF, 0, 0);
        input_set_abs_params(idev, ABS_Y, 0, 0xFFF, 0, 0);
    }
	input_set_abs_params(idev, ABS_PRESSURE, 0, 1, 0, 0);

	ret = input_register_device(idev);
	if (ret)
		goto out_nomem;

	info->idev = idev;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		printk(KERN_ERR "%s: couldn't get IRQ resource\n", __func__);
		ret = -ENODEV;
		goto out_nodev;
	}
	info->touch_irq = res->start;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!res) {
		printk(KERN_ERR "%s: couldn't get IRQ resource\n", __func__);
		ret = -ENODEV;
		goto out_nodev;
	}
	info->device_irq = res->start;

	ret = request_irq(info->touch_irq, ts_handler, IRQF_DISABLED,
				"stmp3xxx_ts_touch", info);
	if (ret)
		goto out_nodev;

	ret = request_irq(info->device_irq, ts_handler, IRQF_DISABLED,
				"stmp3xxx_ts_dev", info);
	if (ret) {
		free_irq(info->touch_irq, info);
		goto out_nodev;
	}
	enter_state_touch_detect(info);

	hw_lradc_use_channel(LRADC_CH2);
	hw_lradc_use_channel(LRADC_CH3);
	hw_lradc_use_channel(LRADC_CH5);
	hw_lradc_configure_channel(LRADC_CH2, 0, 0, 0);
	hw_lradc_configure_channel(LRADC_CH3, 0, 0, 0);
	hw_lradc_configure_channel(LRADC_CH5, 0, 0, 0);

	/* Clear the accumulator & NUM_SAMPLES for the channels */
	HW_LRADC_CHn_CLR(LRADC_CH2, 0xFFFFFFFF);
	HW_LRADC_CHn_CLR(LRADC_CH3, 0xFFFFFFFF);
	HW_LRADC_CHn_CLR(LRADC_CH5, 0xFFFFFFFF);

	hw_lradc_set_delay_trigger(LRADC_DELAY_TRIGGER_TOUCHSCREEN,
			0x3c, 0, 0, 8);
	HW_LRADC_CTRL1_CLR(BM_LRADC_CTRL1_LRADC5_IRQ);
	HW_LRADC_CTRL1_CLR(BM_LRADC_CTRL1_TOUCH_DETECT_IRQ);

	HW_LRADC_CTRL1_SET(BM_LRADC_CTRL1_LRADC5_IRQ_EN);
	HW_LRADC_CTRL1_SET(BM_LRADC_CTRL1_TOUCH_DETECT_IRQ_EN);

	platform_set_drvdata(pdev, info);
	device_init_wakeup(&pdev->dev, 1);
	goto out;

out_nodev:
	input_free_device(idev);
out_nomem:
	kfree(idev);
	kfree(info);
out:
	return ret;
}

static int stmp3xxx_ts_remove(struct platform_device *pdev)
{
	struct stmp3xxx_ts_info *info = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	hw_lradc_unuse_channel(LRADC_CH2);
	hw_lradc_unuse_channel(LRADC_CH3);
	hw_lradc_unuse_channel(LRADC_CH5);
	HW_LRADC_CTRL1_CLR(BM_LRADC_CTRL1_LRADC5_IRQ_EN);
	HW_LRADC_CTRL1_CLR(BM_LRADC_CTRL1_TOUCH_DETECT_IRQ_EN);

	free_irq(info->device_irq, info);
	free_irq(info->touch_irq, info);
	input_free_device(info->idev);

	enter_state_disabled(info);
	kfree(info->idev);
	kfree(info);
	return 0;
}

static int stmp3xxx_ts_suspend(struct platform_device *pdev,
				pm_message_t state)
{
#ifdef CONFIG_PM
	if (!device_may_wakeup(&pdev->dev)) {
		hw_lradc_unuse_channel(LRADC_CH2);
		hw_lradc_unuse_channel(LRADC_CH3);
		hw_lradc_unuse_channel(LRADC_CH5);
	}
#endif
	return 0;
}

static int stmp3xxx_ts_resume(struct platform_device *pdev)
{
#ifdef CONFIG_PM
	if (!device_may_wakeup(&pdev->dev)) {
		hw_lradc_use_channel(LRADC_CH2);
		hw_lradc_use_channel(LRADC_CH3);
		hw_lradc_use_channel(LRADC_CH5);
	}
#endif
	return 0;
}

static struct platform_driver stmp3xxx_ts_driver = {
	.probe		= stmp3xxx_ts_probe,
	.remove		= stmp3xxx_ts_remove,
	.suspend	= stmp3xxx_ts_suspend,
	.resume		= stmp3xxx_ts_resume,
	.driver		= {
		.name		= "stmp3xxx_ts",
		.owner		= THIS_MODULE,
	},
};

static int __init stmp3xxx_ts_init(void)
{
	return platform_driver_register(&stmp3xxx_ts_driver);
}

static void __exit stmp3xxx_ts_exit(void)
{
	platform_driver_unregister(&stmp3xxx_ts_driver);
}

module_init(stmp3xxx_ts_init);
module_exit(stmp3xxx_ts_exit);
module_param(scaled_touchscreen, bool, 0644);
MODULE_PARM_DESC(scaled_touchscreen, "true if the touchscreen should report pre-scaled values");
MODULE_LICENSE("GPL v2");
MODULE_SUPPORTED_DEVICE("input/ts");
MODULE_ALIAS("platform:stmp3xxx_ts");

/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file sdram_autogating.c
 *
 * @brief Enable auto clock gating of the EMI_FAST clock using M4IF.
 *
 * The APIs are for enabling and disabling automatic clock gating of EMI_FAST.
 *
 * @ingroup PM
 */
#include <asm/io.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include <mach/mxc_dvfs.h>
#include "crm_regs.h"

int sdram_autogating_is_active;
static struct device *sdram_autogating_dev;
#define M4IF_CNTL_REG0		0x8c
#define M4IF_CNTL_REG1		0x90

void enable_sdram_autogating(void);
void disable_sdram_autogating(void);

void enable_sdram_autogating()
{
	u32 reg;

	/* Set the Fast arbitration Power saving timer */
	reg = __raw_readl((IO_ADDRESS(M4IF_BASE_ADDR) + M4IF_CNTL_REG1));
	reg &= ~0xFF;
	reg |= 0x09;
	__raw_writel(reg, (IO_ADDRESS(M4IF_BASE_ADDR) + M4IF_CNTL_REG1));
	/*Allow for automatic gating of the EMI internal clock.
	 * If this is done, emi_intr CCGR bits should be set to 11.
	 */
	reg = __raw_readl((IO_ADDRESS(M4IF_BASE_ADDR) + M4IF_CNTL_REG0));
	reg &= ~0x5;
	__raw_writel(reg, (IO_ADDRESS(M4IF_BASE_ADDR) + M4IF_CNTL_REG0));

	sdram_autogating_is_active = 1;
}

void disable_sdram_autogating()
{
	u32 reg;

	reg = __raw_readl((IO_ADDRESS(M4IF_BASE_ADDR) + M4IF_CNTL_REG0));
	reg |= 0x4;
	__raw_writel(reg, (IO_ADDRESS(M4IF_BASE_ADDR) + M4IF_CNTL_REG0));
	sdram_autogating_is_active = 0;
}

static ssize_t sdram_autogating_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (sdram_autogating_is_active)
		return sprintf(buf,
			"M4IF autoclock gating for EMI_FAST enabled\n");
	else
		return sprintf(buf,
			"M4IF autoclock gating for EMI_FAST disabled\n");
}

static ssize_t sdram_autogating_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	if (strstr(buf, "1") != NULL)
		enable_sdram_autogating();
	else if (strstr(buf, "0") != NULL) {
		if (sdram_autogating_is_active)
			disable_sdram_autogating();
	}
	return size;
}

static DEVICE_ATTR(enable, 0644, sdram_autogating_enable_show,
			sdram_autogating_enable_store);

/*!
 * This is the probe routine for the auto clockgating of sdram driver.
 *
 * @param   pdev   The platform device structure
 *
 * @return         The function returns 0 on success
 *
 */
static int __devinit sdram_autogating_probe(struct platform_device *pdev)
{
	int err = 0;

	sdram_autogating_dev = &pdev->dev;

	err = sysfs_create_file(&sdram_autogating_dev->kobj,
							&dev_attr_enable.attr);
	if (err) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for sdram_autogating");
		return err;
	}

	sdram_autogating_is_active = 0;
	if (mxc_cpu_is_rev(CHIP_REV_3_0) >= 1)
		enable_sdram_autogating();
	return 0;
}

static struct platform_driver sdram_autogating_driver = {
	.driver = {
		   .name = "sdram_autogating",
		},
	.probe = sdram_autogating_probe,
};

/*!
 * Initialise the sdram_autogating_driver.
 *
 * @return  The function always returns 0.
 */

static int __init sdram_autogating_init(void)
{
	if (platform_driver_register(&sdram_autogating_driver) != 0) {
		printk(KERN_ERR "sdram_autogating_driver register failed\n");
		return -ENODEV;
	}

	printk(KERN_INFO "sdram autogating driver module loaded\n");
	return 0;
}

static void __exit sdram_autogating_cleanup(void)
{
	sysfs_remove_file(&sdram_autogating_dev->kobj, &dev_attr_enable.attr);

	/* Unregister the device structure */
	platform_driver_unregister(&sdram_autogating_driver);
}

module_init(sdram_autogating_init);
module_exit(sdram_autogating_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("sdram_autogating driver");
MODULE_LICENSE("GPL");


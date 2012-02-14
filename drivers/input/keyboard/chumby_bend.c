/*
    chumby_bend.c
    bunnie -- June 2006 -- 1.4 -- linux 2.4.20
    bunnie -- March 2007 -- 2.0 -- port to Ironforge linux 2.6.16
    bunnie -- December 2007 -- 2.1 -- port to v3.8 and inclusion of watchdog timer feature, reset reason reporting
    sean   -- June 2009 -- 2.2 -- port to Falconwing, and removal of everything but bend sensor functionality.
    sean   -- August 2009 -- 2.3 -- Rewrote entirely and made a keyboard device

    This file is part of the chumby sensor suite driver in the linux kernel.
    Copyright (c) Chumby Industries, 2009

    The sensor suite driver is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License as published
    by the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    The sensor suite driver is distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with the Chumby; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#define FW_BEND_VERSION "2.3-Falconwing"

#include <linux/moduleparam.h>

#include <linux/kernel.h>   /* printk() */
#include <linux/slab.h>     /* kmalloc() */
#include <linux/errno.h>    /* error codes */
#include <linux/types.h>    /* size_t */
#include <linux/fcntl.h>    /* O_ACCMODE */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>

#include <mach/regs-pinctrl.h>
#include <mach/pinmux.h>



// Begin legacy switch mode
static int legacy_open(struct inode *inode, struct file *file);
static int legacy_release(struct inode *inode, struct file *file);
static ssize_t legacy_read(struct file *file, char *buf,
                           size_t count, loff_t * ppos);


// True or false depending on whether the "bend" key is pressed or not.
static int bend_pressed = 0;


#include <linux/cdev.h>
#include <asm/uaccess.h>    /* copy_*_user */
static struct cdev *legacy_switch;
static int sense1_major, sense1_minor;
dev_t legacy_dev;
static struct file_operations legacy_fops = {
    .owner =    THIS_MODULE,
    .read =     legacy_read,
    .open =     legacy_open,
    .release =  legacy_release,
};

// End legacy switch mode





#if 0
#define CHLOG(format, arg...)            \
    printk("chumby_bend.c - %s():%d - " format, __func__, __LINE__, ## arg)
#else
#define CHLOG(...)
#endif




// Do some debouncing.  Figure that events must happen 5 jiffies apart.
static int keys[5];
#define CKEY_UP 0
#define CKEY_RIGHT 1
#define CKEY_LEFT 2
#define CKEY_DOWN 3
#define CKEY_BEND 4

static int timer_queued;

static void update_key_polarities(int *new_keys) {
    // Set the key to fire when the polarity of the switch flips.
    if(new_keys[CKEY_UP])
        HW_PINCTRL_IRQPOL0_SET(1<<24);
    else
        HW_PINCTRL_IRQPOL0_CLR(1<<24);

	if(new_keys[CKEY_RIGHT])
        HW_PINCTRL_IRQPOL0_SET(1<<25);
    else
        HW_PINCTRL_IRQPOL0_CLR(1<<25);

	if(new_keys[CKEY_LEFT])
        HW_PINCTRL_IRQPOL2_SET(1<<28);
    else
        HW_PINCTRL_IRQPOL2_CLR(1<<28);

	if(new_keys[CKEY_DOWN])
        HW_PINCTRL_IRQPOL0_SET(1<<23);
    else
        HW_PINCTRL_IRQPOL0_CLR(1<<23);

	stmp3xxx_configure_irq(PINID_GPMI_WRN,  new_keys[CKEY_UP]?IRQ_TYPE_EDGE_RISING:IRQ_TYPE_EDGE_FALLING);
	stmp3xxx_configure_irq(PINID_GPMI_RDN, new_keys[CKEY_RIGHT]?IRQ_TYPE_EDGE_RISING:IRQ_TYPE_EDGE_FALLING);
	stmp3xxx_configure_irq(PINID_GPMI_CE0N, new_keys[CKEY_LEFT]?IRQ_TYPE_EDGE_RISING:IRQ_TYPE_EDGE_FALLING);
	stmp3xxx_configure_irq(PINID_GPMI_WPN,  new_keys[CKEY_DOWN]?IRQ_TYPE_EDGE_RISING:IRQ_TYPE_EDGE_FALLING);
	stmp3xxx_configure_irq(PINID_PWM4,      new_keys[CKEY_BEND]?IRQ_TYPE_EDGE_RISING:IRQ_TYPE_EDGE_FALLING);

	if(new_keys[CKEY_BEND])
        HW_PINCTRL_IRQPOL1_SET(1<<30);
    else
        HW_PINCTRL_IRQPOL1_CLR(1<<30);
}

static void read_key_values(int *new_keys) {
	int bank0, bank1, bank2;

	/* Read all keys at once.  We'll see chich changed later. */
	bank0 = HW_PINCTRL_DIN0_RD();
	bank1 = HW_PINCTRL_DIN1_RD();
	bank2 = HW_PINCTRL_DIN2_RD();

	/* Pull out the current key values */
	new_keys[CKEY_UP]    = !(bank0&(1<<24));
	new_keys[CKEY_RIGHT] = !(bank0&(1<<25));
	new_keys[CKEY_LEFT]  = !(bank2&(1<<28));
	new_keys[CKEY_DOWN]  = !(bank0&(1<<23));
	new_keys[CKEY_BEND]  = !(bank1&(1<<30));
}

static void update_keys(unsigned long arg) {
	struct input_dev *input_dev = (struct input_dev *)arg;
	int new_keys[5];

	read_key_values(new_keys);

	CHLOG("Compare UP:    %d/%d\n", keys[CKEY_UP], new_keys[CKEY_UP]);
	CHLOG("Compare DOWN:  %d/%d\n", keys[CKEY_DOWN], new_keys[CKEY_DOWN]);
	CHLOG("Compare LEFT:  %d/%d\n", keys[CKEY_LEFT], new_keys[CKEY_LEFT]);
	CHLOG("Compare RIGHT: %d/%d\n", keys[CKEY_RIGHT], new_keys[CKEY_RIGHT]);
	CHLOG("Compare BEND:  %d/%d\n", keys[CKEY_BEND], new_keys[CKEY_BEND]);

	if(new_keys[CKEY_UP] != keys[CKEY_UP])
        input_event(input_dev, EV_KEY, KEY_UP, new_keys[CKEY_UP]);
	if(new_keys[CKEY_RIGHT] != keys[CKEY_RIGHT])
        input_event(input_dev, EV_KEY, KEY_RIGHT, new_keys[CKEY_RIGHT]);
	if(new_keys[CKEY_LEFT] != keys[CKEY_LEFT])
        input_event(input_dev, EV_KEY, KEY_LEFT, new_keys[CKEY_LEFT]);
	if(new_keys[CKEY_DOWN] != keys[CKEY_DOWN])
        input_event(input_dev, EV_KEY, KEY_DOWN, new_keys[CKEY_DOWN]);
	if(new_keys[CKEY_BEND] != keys[CKEY_BEND]) {
		bend_pressed = new_keys[CKEY_BEND];
        input_event(input_dev, EV_KEY, KEY_ENTER, new_keys[CKEY_BEND]);
	}

	memcpy(keys, new_keys, sizeof(keys));
	update_key_polarities(keys);
	timer_queued = 0;
}

static void update_keys_after(int msecs, void *data) {
	static struct timer_list timer;
	if(!timer_queued) {
		CHLOG("Updating keyboard in %d msecs\n", msecs);
		timer_queued=1;

		init_timer(&timer);
		timer.data = (unsigned long)data;
		timer.function = update_keys;
		timer.expires = jiffies + (HZ/1000)*msecs;
		add_timer(&timer);
	}
	else
		CHLOG("Not updating keys, as there was already a request pending\n");
}

static irqreturn_t chumby_key_pressed(int irq, void *arg)
{

	/* Actually update the keys after a delay.  To debounce. */
	CHLOG("Someone pressed a key!!!\n");
	update_keys_after(2, arg);
    return IRQ_HANDLED;
}





// Legacy code.  Note that since the value will only be updated during an
// interrupt context, we don't need locking around these commands.
static int legacy_open(struct inode *inode, struct file *file) {
    return 0;
}
static int legacy_release(struct inode *inode, struct file *file) {
    return 0;
}
static ssize_t legacy_read(struct file *file, char *buf,
                           size_t count, loff_t * ppos) {
    char retval = !!bend_pressed;
    if(copy_to_user(buf, &retval, sizeof(retval)))
        return -EFAULT;
    return sizeof(retval);
}




static int chumby_bend_probe(struct platform_device *pdev)
{
    int ret;
    struct input_dev *input_dev;

    // Let people know all about this lovely driver.
    printk("Chumby bend sensor driver version %s initializing "
           "(scross@chumby.com)...\n",
           FW_BEND_VERSION);


    // Allocate input-dev required structures.
    input_dev = input_allocate_device();
    platform_set_drvdata(pdev, input_dev);


	/* Grab the pins used for this keyboard */
	stmp3xxx_request_pin(PINID_PWM4, PIN_GPIO, "bend sensor");
	stmp3xxx_configure_irq(PINID_PWM4, IRQ_TYPE_EDGE_RISING);
	stmp3xxx_configure_irq_handler(PINID_PWM4, chumby_key_pressed, input_dev);

	stmp3xxx_request_pin(PINID_GPMI_WRN, PIN_GPIO, "up key");
	stmp3xxx_configure_irq(PINID_GPMI_WRN, IRQ_TYPE_EDGE_RISING);
	stmp3xxx_configure_irq_handler(PINID_GPMI_WRN, chumby_key_pressed, input_dev);

	stmp3xxx_request_pin(PINID_GPMI_RDN, PIN_GPIO, "right key");
	stmp3xxx_configure_irq(PINID_GPMI_RDN, IRQ_TYPE_EDGE_RISING);
	stmp3xxx_configure_irq_handler(PINID_GPMI_RDN, chumby_key_pressed, input_dev);

	stmp3xxx_request_pin(PINID_GPMI_CE0N, PIN_GPIO, "left key");
	stmp3xxx_configure_irq(PINID_GPMI_CE0N, IRQ_TYPE_EDGE_RISING);
	stmp3xxx_configure_irq_handler(PINID_GPMI_CE0N, chumby_key_pressed, input_dev);

	stmp3xxx_request_pin(PINID_GPMI_WPN, PIN_GPIO, "down key");
	stmp3xxx_configure_irq(PINID_GPMI_WPN, IRQ_TYPE_EDGE_RISING);
	stmp3xxx_configure_irq_handler(PINID_GPMI_WPN, chumby_key_pressed, input_dev);


	/* Read the current key state, then set the appropriate IRQ polarities */
	read_key_values(keys);
	update_key_polarities(keys);


    // Indicate that we send key events.
    set_bit(EV_KEY, input_dev->evbit);
    bitmap_fill(input_dev->keybit, KEY_MAX);
    //input_set_capability(input, EV_KEY, KEY_ENTER);


    input_dev->name = pdev->name;
    input_dev->phys = "chumby_bend/input0";
    input_dev->dev.parent = &pdev->dev;

    input_dev->id.bustype = BUS_HOST;
    input_dev->id.vendor  = 0x0001;
    input_dev->id.product = 0x0001;
    input_dev->id.version = 0x0100;


    ret = input_register_device(input_dev);
    if(ret) {
        CHLOG("Unable to register device\n");
        goto fail;
    }




    // Backwards-compatibility mode:
    // Older software polls /dev/switch and obtains a binary value to
    // determine whether it's pressed or not.
    legacy_switch = cdev_alloc();
    if(!legacy_switch) {
        CHLOG("Error adding legacy switch mode\n");
        goto fail;
    }

    // Allocate device numbes for the legacy switch device.
    ret = alloc_chrdev_region(&legacy_dev, sense1_minor, 1, "switch");
    if(ret<0) {
        printk(KERN_WARNING 
               "bend: Can't get major number for compatibility mode: %d\n",
               ret);
        goto fail;
    }
    sense1_major = MAJOR(legacy_dev);


    // Create the /dev entry with all of the necessary file options.
    cdev_init(legacy_switch, &legacy_fops);
    legacy_switch->owner = THIS_MODULE;
    legacy_switch->ops   = &legacy_fops;
    ret = cdev_add(legacy_switch, legacy_dev, 1);
    if(ret) {
        printk(KERN_NOTICE "Error %d adding switch device\n", ret);
        goto fail;
    }





    return 0;

fail:
    CHLOG(KERN_NOTICE "Error %d adding switch device\n", ret);
    input_free_device(input_dev);
    return -EINVAL;
}

static int __devexit chumby_bend_remove(struct platform_device *pdev)
{
    struct input_dev *input_dev = platform_get_drvdata(pdev);

    // Legacy mode
    dev_t devno = MKDEV(sense1_major, sense1_minor);
    cdev_del(legacy_switch);
    unregister_chrdev_region(devno, 1);


    CHLOG("Removing bend sensor\n");
    input_unregister_device(input_dev);
    platform_set_drvdata(pdev, NULL);
    return 0;
}


static int chumby_bend_suspend(struct platform_device *pdev, pm_message_t state)
{
    return 0;
}

static int chumby_bend_resume(struct platform_device *pdev)
{
    return 0;
}



static struct platform_driver chumby_bend_driver = {
    .probe  = chumby_bend_probe,
    .remove = __devexit_p(chumby_bend_remove),
    .suspend    = chumby_bend_suspend,
    .resume     = chumby_bend_resume,
    .driver = {
        .name  = "bend-sensor",
        .owner = THIS_MODULE,
    },
};



static int __devinit chumby_bend_init(void)
{
    CHLOG("Initializing driver\n");
    return platform_driver_register(&chumby_bend_driver);
}

static void __exit chumby_bend_exit(void) {
    platform_driver_unregister(&chumby_bend_driver);
}

// entry and exit mappings
module_init(chumby_bend_init);
module_exit(chumby_bend_exit);



MODULE_AUTHOR("scross@chumby.com");
MODULE_DESCRIPTION("chumby bend sensor driver with fake keyboard emulation");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.3");
MODULE_ALIAS("platform:bend-sensor");

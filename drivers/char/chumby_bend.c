/*
    chumby_bend.c
    bunnie -- June 2006 -- 1.4 -- linux 2.4.20
    bunnie -- March 2007 -- 2.0 -- port to Ironforge linux 2.6.16
    bunnie -- December 2007 -- 2.1 -- port to v3.8 and inclusion of watchdog timer feature, reset reason reporting
    sean   -- June 2009 -- 2.2 -- port to Falconwing, and removal of everything but bend sensor functionality.

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

#define FW_BEND_VERSION "1.0-Falconwing"

//#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>   /* printk() */
#include <linux/slab.h>     /* kmalloc() */
#include <linux/fs.h>       /* everything... */
#include <linux/errno.h>    /* error codes */
#include <linux/types.h>    /* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>    /* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/cdev.h>

#include <asm/io.h>
#include <asm/system.h>     /* cli(), *_flags */
#include <asm/uaccess.h>    /* copy_*_user */

#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/poll.h>
#include <linux/sysctl.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/rtc.h>

#include <mach/regs-pinctrl.h>

#include "chumby_bend.h"

/*
 * basic parameters
 */
int bend_major   = 0;       // dynamic allocation
int bend_minor   = 0;
int bend_nr_devs = 1;

//module_param(version038, int, S_IRUGO|S_IWUSR);
module_param(bend_major, int, S_IRUGO);
module_param(bend_minor, int, S_IRUGO);
module_param(bend_nr_devs, int, S_IRUGO);

MODULE_AUTHOR("scross@chumby.com");
MODULE_LICENSE("GPL");

// static data
static int gDone = 0;
static unsigned long fw_bend_status = 0;    /* bitmapped status byte.       */

/*
 * Sense1 sensor data logs, tracked by tasks that are scheduled by the
 * task scheduler
 */

#define FIXEDPOINT_NORM 1000

struct sense1data {
    unsigned char bent;
    struct cdev *bend_cdev;
} sense1task_data;

#define USE_SYSCTL  1

#define SENSE1_BLOCKING 0   // turn off blocking read on sense1 sensor

static int gDebugTrace = 0;

#if 1
#   if USE_SYSCTL
#       define CHUMSENSE1_DEBUG( flag, fmt, args... ) do { if ( gDebug ## flag ) printk( "%s: " fmt, __FUNCTION__ , ## args ); } while (0)
#   else
#       define CHUMSENSE1_DEBUG( flag, fmt, args... ) printk( "%s: " fmt, __FUNCTION__ , ## args )
#   endif
#else
#   define CHUMSENSE1_DEBUG( flag, fmt, args... )
#endif

// function protos
static int chumby_bend_open(struct inode *inode, struct file *file);
static int chumby_bend_release(struct inode *inode, struct file *file);
static int bend_read_proc(char *page, char **start, off_t off,
                int count, int *eof, void *data);
static ssize_t chumby_bend_read(struct file *file, char *buf,
                  size_t count, loff_t * ppos);
//static ssize_t chumby_bend_write(struct file *file, const unsigned char *buf,
//                               size_t count, loff_t *ppos );
//static unsigned int chumby_bend_poll(struct file * filp, struct poll_table_struct * wait);



// map into the generic driver infrastructure
static struct file_operations bend_fops = {
    .owner =    THIS_MODULE,
//  .llseek =   chumby_bend_llseek,
    .read =     chumby_bend_read,
//  .poll =     chumby_bend_poll,
//  .ioctl =    chumby_bend_ioctl,
    .open =     chumby_bend_open,
    .release =  chumby_bend_release,
//  .write =    chumby_bend_write,
//  .fasync =   chubmy_bend_fasync,
};

///////////// code /////////////

static int get_bend_value(void) {
    return !(HW_PINCTRL_DIN1_RD()&0x40000000);
}

static int chumby_bend_release(struct inode *inode, struct file *file)
{
    CHUMSENSE1_DEBUG(Trace, "Top of release.\n");
    fw_bend_status &= ~FW_BEND_IS_OPEN;
    return 0;
}

static int bend_proc_output(char *buf)
{
    int printlen = 0;

    // insert proc debugging output here
    printlen =
        sprintf(buf,
            "Chumby bend driver for Falconwing, version %s (bunnie@chumby.com)\n",
            FW_BEND_VERSION);
    buf += printlen;
//    if (gDebugTrace)
//        printlen += sprintf(buf, "  Debug trace is enabled.\n");
//    else
//        printlen += sprintf(buf, "  Debug trace is disabled.\n");

    printlen += sprintf(buf, "Bend value: %d\n", 
            get_bend_value());/*,
            BF_PINCTRL_DIN1_DIN(30),
            BF_PINCTRL_DIN1_DIN(31),
            BF_PINCTRL_DIN1_DIN(29),
            BF_PINCTRL_DIN1_DIN(28)
            );*/

    // the code below is handy for debugging the watchdog timer, it crashes the kernel
    //  printk( "WSR: %04X, WRSR: %04X, WCR: %04X\n", WSR, WRSR, WCR );
    //  { // force a hard crash of the kernel if possible....
    //    int i;
    //    for( i = 0xC0000000; i < 0xFFFFFF; i++ ) {
    //      *(char *) i = 0x00;
    //    }
    //  }
    return printlen;
}

static int bend_read_proc(char *page, char **start, off_t off,
                int count, int *eof, void *data)
{
    int len = bend_proc_output(page);
    if (len <= off + count)
        *eof = 1;
    *start = page + off;
    len -= off;
    if (len > count)
        len = count;
    if (len < 0)
        len = 0;

    return len;
}


static int chumby_bend_open(struct inode *inode, struct file *file)
{
    // make sure we're not opened twice
    if (fw_bend_status & FW_BEND_IS_OPEN)
        return -EBUSY;

    fw_bend_status |= FW_BEND_IS_OPEN;
    return 0;
}




/*
  this driver handles the following sensors:

  * dimming level
  * speaker muting
  * DC voltage reading
  * battery voltage reading
  * headphone insertion status
  * bend sensor ('switch')

*/
static int __init chumby_bend_init(void)
{
    dev_t dev = 0;
    int result, err;

    sense1task_data.bend_cdev = cdev_alloc();

    // insert all device specific hardware initializations here
    printk("Chumby bend sensor driver version %s initializing "
           "(scross@chumby.com)...\n",
           FW_BEND_VERSION);

    /*
     * Get a range of minor numbers to work with, asking for a dynamic
     * major unless directed otherwise at load time.
     */
    if (bend_major) {
        dev = MKDEV(bend_major, bend_minor);
        result = register_chrdev_region(dev, bend_nr_devs, "switch");
    } else {
        result = alloc_chrdev_region(&dev, bend_minor, bend_nr_devs, "switch");
        bend_major = MAJOR(dev);
    }
    if (result < 0) {
        printk(KERN_WARNING "switch: can't get major %d\n", bend_major);
        return result;
    }

    create_proc_read_entry("bend", 0, 0, bend_read_proc, NULL);

    cdev_init(sense1task_data.bend_cdev, &bend_fops);
    sense1task_data.bend_cdev->owner = THIS_MODULE;
    sense1task_data.bend_cdev->ops = &bend_fops;
    err = cdev_add(sense1task_data.bend_cdev, dev, 1);


    // Set up the bend sensor to be a GPIO.
    HW_PINCTRL_MUXSEL3_SET(0x30000000);


    // Fail gracefully if need be.
    if (err)
        printk(KERN_NOTICE "Error %d adding switch device\n", err);

    return 0;
}

static ssize_t chumby_bend_read(struct file *file, char *buf,
                  size_t count, loff_t * ppos)
{
    char retval = 0;
    size_t retlen = 1;

    retval = get_bend_value();

    CHUMSENSE1_DEBUG(Trace, "sense1 read exit with: %d\n", retval);
    copy_to_user(buf, &retval, retlen);

    return retlen;
}

static void __exit chumby_bend_exit(void)
{
    dev_t devno = MKDEV(bend_major, bend_minor);

    CHUMSENSE1_DEBUG(Trace, "Top of exit.\n");
    // set global flag that we're out of here and force a call to remove ourselves
    gDone = 1;


    // wait to dequeue self; if kernel panics on rmmod try
    // adding 100 to this value
    mdelay(200);

    // insert all cleanup stuff here
    cdev_del(sense1task_data.bend_cdev);
    remove_proc_entry("bend", NULL);
    unregister_chrdev_region(devno, bend_nr_devs);

}

// entry and exit mappings
module_init(chumby_bend_init);
module_exit(chumby_bend_exit);

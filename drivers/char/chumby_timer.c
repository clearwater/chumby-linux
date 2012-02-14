/*
       chumby_timerx.c
       bunnie    -- August 2006 -- 1.0 -- linux 2.4.20
       bunnie    -- April 2007  -- 2.0 -- port to Ironforge linux 2.6.16
       ghutchins -- Nov. 2007   -- 2.1 -- added minor 1, returns msecs

       This file is part of the chumby sensor suite driver in the linux kernel.
       Copyright (c) Chumby Industries, 2007

       The sensor suite driver is free software; you can redistribute it and/or modify
       it under the terms of the GNU General Public License as published by
       the Free Software Foundation; either version 2 of the License, or
       (at your option) any later version.

       The sensor suite driver is distributed in the hope that it will be useful,
       but WITHOUT ANY WARRANTY; without even the implied warranty of
       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
       GNU General Public License for more details.

       You should have received a copy of the GNU General Public License
       along with the Chumby; if not, write to the Free Software
       Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#define TIMERX_VERSION "3.0-Falconwing"
#define CNPLATFORM_falconwing

#include <linux/slab.h>         /* kmalloc() */
#include <linux/fs.h>           /* everything... */
#include <linux/errno.h>        /* error codes */
#include <linux/types.h>        /* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>        /* O_ACCMODE */
#include <linux/cdev.h>
#include <linux/delay.h>
#include <asm/uaccess.h>        /* copy_*_user */

#include <linux/timer.h>
#include <mach/regs-rtc.h>


/*
 * basic parameters
 */

int timerx_major = 0; // dynamic allocation
int timerx_minor = 0;
int timerx_nr_devs = 2;

module_param(timerx_major, int, S_IRUGO);
module_param(timerx_minor, int, S_IRUGO);
module_param(timerx_nr_devs, int, S_IRUGO);

MODULE_AUTHOR("bunnie@chumby.com");
MODULE_DESCRIPTION("Expose jiffies and msecs of uptime");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.1");

// static data
static unsigned long timerx_active = 0;	/* bitmapped status byte.	*/
static unsigned long previous_timestamp = 0;    // minor==1 only!

struct timerxdata {
  struct cdev *timerx_cdev;
} timerxtask_data;

#if 0
#   define CHUMTIMERX_DEBUG( flag, fmt, args... ) do { printk( "%s: " fmt, __FUNCTION__ , ## args ); } while (0)
#else
#   define CHUMTIMERX_DEBUG( flag, fmt, args... )
#endif

// function protos
static int chumby_timerx_open(struct inode *inode, struct file *file);
static int chumby_timerx_release(struct inode *inode, struct file *file);
static int timerx_read_proc(char *page, char **start, off_t off,
                         int count, int *eof, void *data);
static ssize_t chumby_timerx_read(struct file *file, char *buf,
				size_t count, loff_t *ppos);


// map into the generic driver infrastructure
static struct file_operations timerx_fops = {
	owner:		THIS_MODULE,
	read:		chumby_timerx_read,
	open:		chumby_timerx_open,
	release:	chumby_timerx_release,
};


///////////// code /////////////

static int chumby_timerx_release(struct inode *inode, struct file *file) {
    int minor = iminor(inode);
    CHUMTIMERX_DEBUG( Trace, "Top of release(minor=%d).\n", minor );
    timerx_active &= ~(1<<minor);
    return 0;
}


static int timerx_proc_output(char *buf) {
    int printlen = 0;
    
    // insert proc debugging output here
    printlen = sprintf(buf, 
                        "Chumby timerx driver version %s (bunnie@chumby.com)\n"
                        "The current time is: %08lX\n", 
                        TIMERX_VERSION, jiffies );
  
    return(printlen);
}


static int timerx_read_proc(char *page, char **start, off_t off,
                         int count, int *eof, void *data)
{
        int len = timerx_proc_output (page);
        if (len <= off+count) *eof = 1;
        *start = page + off;
        len -= off;
        if (len>count) len = count;
        if (len<0) len = 0;
        return len;
}


static int chumby_timerx_open(struct inode *inode, struct file *file) {
    int minor = iminor(inode);
    
    // make sure we're not opened twice
    if (timerx_active & (1<<minor))
        return -EBUSY;
    
    timerx_active |= (1<<minor);
    file->private_data = (void*) minor;
    if (minor == 1)
        previous_timestamp = 0;
    return(0);
}


static int __init chumby_timerx_init(void) {
  dev_t dev = 0;
  int result, err;

  timerxtask_data.timerx_cdev = cdev_alloc();

  if (timerx_nr_devs > 32)
      timerx_nr_devs = 32;

    // insert all device specific hardware initializations here
    printk("Chumby timerx[%d] driver version %s initializing "
           "(bunnie@chumby.com)... show me your jiffies!!!\n", 
           timerx_nr_devs, TIMERX_VERSION);

    /*
     * Get a range of minor numbers to work with, asking for a dynamic
     * major unless directed otherwise at load time.
     */
    if (timerx_major) {
        dev = MKDEV(timerx_major, timerx_minor);
        result = register_chrdev_region(dev, timerx_nr_devs, "timerx");
    }
    else {
        result = alloc_chrdev_region(&dev, timerx_minor,
                                     timerx_nr_devs, "timerx");
        timerx_major = MAJOR(dev);
    }
    if (result < 0) {
        printk(KERN_WARNING "timerx: can't get major %d\n", timerx_major);
        return result;
    }

    create_proc_read_entry ("timerx", 0, 0, timerx_read_proc, NULL);


    cdev_init(timerxtask_data.timerx_cdev, &timerx_fops);
    timerxtask_data.timerx_cdev->owner = THIS_MODULE;
    timerxtask_data.timerx_cdev->ops = &timerx_fops;
    err = cdev_add (timerxtask_data.timerx_cdev, dev, timerx_nr_devs);
    /* Fail gracefully if need be */
    if (err)
        printk(KERN_NOTICE "Error %d adding timerx device\n", err);

    return (0);
}


static ssize_t chumby_timerx_read(struct file   *file, 
                                  char          *buf,
                                  size_t        count, 
                                  loff_t        *ppos) {
    int minor = (int) file->private_data;
    unsigned long timestamp;
    
    CHUMTIMERX_DEBUG( Trace, "Top of read(minor=%d).\n", minor );
    
    if (minor != 1)
        timestamp = jiffies;
    else {
#ifndef CNPLATFORM_falconwing
        // if minor == 1 we return milliseconds instead of jiffies, note this
        // code is *NOT* portable and assumes that system_timer is exported
        // and system_timer->offset() returns microseconds -- reading of
        // jiffies and usecs was borrowed from do_gettimeofday()
        unsigned long flags;
        unsigned long seq;
        
            timestamp = jiffies_to_msecs(jiffies) + 
                        0;

        // timestamp *MUST* be monotonically increasing, unless 32-bit wrap
        // has occurred -- every 29 days timestamp will wrap
        if (timestamp < previous_timestamp) {
            if (timestamp > 100 && previous_timestamp < 0xFFFFFF00) {
                printk( KERN_ERR "%s(): time travel, %lu -> %lu!\n",
                        __FUNCTION__, previous_timestamp, timestamp );
            }
        }
        previous_timestamp = timestamp;
#else
        timestamp = HW_RTC_MILLISECONDS_RD();
#endif
    }

    copy_to_user(buf, &timestamp, sizeof(timestamp));
    return sizeof(timestamp);
}

static void __exit chumby_timerx_exit(void) {
  dev_t devno = MKDEV(timerx_major, timerx_minor);

  CHUMTIMERX_DEBUG( Trace, "Top of exit.\n" );

  // Wait to dequeue self.
  // If kernel panics on rmmod try adding 100 to this value
  mdelay(200);
  
  // Insert all cleanup stuff here
  cdev_del( timerxtask_data.timerx_cdev );
  remove_proc_entry( "timerx", NULL );
  unregister_chrdev_region(devno, timerx_nr_devs);

}

// entry and exit mappings
module_init(chumby_timerx_init);
module_exit(chumby_timerx_exit);


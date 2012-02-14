/*
 * Freescale STMP378X I2C bus driver
 *
 * Author: Dmitrij Frasenyak <sed@embeddedalley.com>
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
/* #define DEBUG */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <mach/regs-i2c.h>
#include <mach/regs-apbx.h>
#include <mach/i2c.h>

static DECLARE_WAIT_QUEUE_HEAD(cmd_wq);



static void reset_i2c_module(void)
{
	int count;
	count = 1000;
	HW_I2C_CTRL0_SET(BM_I2C_CTRL0_SFTRST);
	udelay(10); /* Reseting the module can take multiple clocks.*/
	while (--count && (!(HW_I2C_CTRL0_RD() & BM_I2C_CTRL0_CLKGATE)))
		udelay(1);

	if (!count) {
		printk(KERN_ERR "timeout reseting the module\n");
		BUG();
	}

	/* take controller out of reset */
	HW_I2C_CTRL0_CLR(BM_I2C_CTRL0_SFTRST | BM_I2C_CTRL0_CLKGATE);
	udelay(10);
	HW_I2C_CTRL1_SET(0x0000FF00); /* Wil catch all error (IRQ mask) */

}

/*
 * Low level master read/write transaction.
 */
static int stmp378x_i2c_xfer_msg(struct i2c_adapter *adap,
				 struct i2c_msg *msg, int stop)
{
	struct stmp378x_i2c_dev *dev = i2c_get_adapdata(adap);
	int err;

//	init_completion(&dev->cmd_complete);
	dev->cmd_err = 0;

	dev_dbg(dev->dev, " Start XFER ===>\n");
	dev_dbg(dev->dev, "addr: 0x%04x, len: %d, flags: 0x%x, stop: %d\n",
		msg->addr, msg->len, msg->flags, stop);

	if ((msg->len == 0) || (msg->len > (PAGE_SIZE - 1)))
		return -EINVAL;

// CHUMBY_radio_fix
// We need to do this special case because of how the FM radio works.
// Basically, you need to send a "Read" message, but write the register
// you want to read as the first packet.
// Therefore, we make a special case for write-then-read when we don't have
// an I2C_M_RD flag but DO have an I2C address with the "read" bit set.
    if((!msg->flags & I2C_M_RD) && (msg->addr & 1)) {
        dev->packet_count = 2;
        hw_i2c_setup_writeread(msg->addr,
                msg->buf,
                msg->len,
                stop ? BM_I2C_CTRL0_POST_SEND_STOP : 0);
		hw_i2c_run(2); /* read */
    }
    else if (msg->flags & I2C_M_RD) {
        dev->packet_count = 1;
		hw_i2c_setup_read(msg->addr ,
				  msg->buf ,
				  msg->len,
				  stop ? BM_I2C_CTRL0_POST_SEND_STOP : 0);

		hw_i2c_run(1); /* read */
	} else {
        dev->packet_count = 1;
		hw_i2c_setup_write(msg->addr ,
				   msg->buf ,
				   msg->len,
				   stop ? BM_I2C_CTRL0_POST_SEND_STOP : 0);

		hw_i2c_run(0); /* write */
	}

    dev->finished = 0;
    err = wait_event_interruptible_timeout(cmd_wq, dev->finished == 1, msecs_to_jiffies(50));
//	err = wait_for_completion_interruptible_timeout(
//		&dev->cmd_complete,
//		msecs_to_jiffies(1)
//		);
//    mdelay(1);
    

	if (err < 0) {
		dev_dbg(dev->dev, "controler is timed out\n");
		return -ETIMEDOUT;
	}
	if ((!dev->cmd_err) && (msg->flags & I2C_M_RD))
		hw_i2c_finish_read(msg->buf, msg->len);
    else if ((!dev->cmd_err) && !(msg->flags & I2C_M_RD) && (msg->addr & 1))
		hw_i2c_finish_writeread(msg->buf, msg->len);
//    printk("Okay, the value in the buffer is now: 0x%08x\n", *((int *)msg->buf));

	dev_dbg(dev->dev, "<============= Done with err=%d\n", dev->cmd_err);


	return dev->cmd_err;
}


static int
stmp378x_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	int i;
	int err;

	if (!msgs->len)
		return -EINVAL;

	for (i = 0; i < num; i++) {
//		printk("Calling stmp378x_i2c_xfer_msg(%p, %p, (%d == (%d - 1)):%d)",
//                adap, &msgs[i], i, num, (i == (num - 1)));
		err = stmp378x_i2c_xfer_msg(adap, &msgs[i], (i == (num - 1)));
		if (err)
			break;
	}

	if (err == 0)
		err = num;

	return err;
}

static u32
stmp378x_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

/*
 * Debug. Don't need dma_irq for the final version
 */

static irqreturn_t
stmp378x_i2c_dma_isr(int this_irq, void *dev_id)
{
	struct stmp378x_i2c_dev *dev = dev_id;
    int cmd_value = HW_APBX_CHn_CMD_RD(3);
	hw_i2c_clear_dma_interrupt();

    // Indicate completion of the transfer(s) if the DMA engine has no more
    // data to transfer.
//    printk(">>> Got a completion IRQ: %p %p (%p)\n", HW_APBX_CHn_NXTCMDAR_RD(3), cmd_value, cmd_value&0xffff0000);
//    if(!HW_APBX_CHn_NXTCMDAR_RD(3))
    if(!(cmd_value&0xffff0000)) { //HW_APBX_CHn_NXTCMDAR_RD(3))
//        complete(&dev->cmd_complete);
        dev->finished = 1;
        wake_up_interruptible(&cmd_wq);
    }
//    else
//        printk(">>> Looks like it's not the last transfer, either.\n");
	return IRQ_HANDLED;

}

#define I2C_IRQ_MASK 0x000000FF

static irqreturn_t
stmp378x_i2c_isr(int this_irq, void *dev_id)
{
	struct stmp378x_i2c_dev *dev = dev_id;
	u32 stat;
//	u32 done_mask =
//		BM_I2C_CTRL1_DATA_ENGINE_CMPLT_IRQ  |
//		BM_I2C_CTRL1_BUS_FREE_IRQ           |
//        0;

	stat = HW_I2C_CTRL1_RD() & I2C_IRQ_MASK;
	if (!stat)
		return IRQ_NONE;

	if (stat & BM_I2C_CTRL1_NO_SLAVE_ACK_IRQ) {
		dev->cmd_err = -EREMOTEIO;

		/*
		 * Stop DMA
		 * Clear NAK
		 */
		HW_I2C_CTRL1_SET(BM_I2C_CTRL1_CLR_GOT_A_NAK);
		hw_i2c_reset_dma();
		reset_i2c_module();

//		complete(&dev->cmd_complete);
        wake_up_interruptible(&cmd_wq);

		goto done;
	}

/* Don't care about  BM_I2C_CTRL1_OVERSIZE_XFER_TERM_IRQ */
	if (stat & (
		    BM_I2C_CTRL1_EARLY_TERM_IRQ |
		    BM_I2C_CTRL1_MASTER_LOSS_IRQ |
		    BM_I2C_CTRL1_SLAVE_STOP_IRQ |
		    BM_I2C_CTRL1_SLAVE_IRQ
		    )) {
		dev->cmd_err = -EIO;
//		complete(&dev->cmd_complete);
        wake_up_interruptible(&cmd_wq);
		goto done;
	}

    // Happy case: The I2C transfer completed without error.
    // This will be handled in the DMA completion IRQ.
//    printk(">>> Got a completion IRQ (packet_count %d -> %d ; stat: 0x%08x / 0x%08x)\n",
//            dev->packet_count, dev->packet_count-1, stat, stat & done_mask);
//    dev->packet_count--;
//	if (((stat & done_mask) == done_mask) && (dev->packet_count == 0)) {
//        printk(">>>     And it was the last packet!\n");
//		complete(&dev->cmd_complete);
//        wake_up_interruptible(&cmd_wq);
//    }


done:
	HW_I2C_CTRL1_CLR(stat);
	return IRQ_HANDLED;
}

static const struct i2c_algorithm stmp378x_i2c_algo = {
	.master_xfer	= stmp378x_i2c_xfer,
	.functionality	= stmp378x_i2c_func,
};


static int
stmp378x_i2c_probe(struct platform_device *pdev)
{
	struct stmp378x_i2c_dev	*dev;
	struct i2c_adapter	*adap;
	struct resource		*irq;
	int err = 0;

	/* NOTE: driver uses the static register mapping */
	dev = kzalloc(sizeof(struct stmp378x_i2c_dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "no mem \n");
		return -ENOMEM;
	}

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0); /* Error */
	if (!irq) {
		dev_err(&pdev->dev, "no err_irq resource\n");
		err = -ENODEV;
		goto nores;
	}
	dev->irq_err = irq->start;

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 1); /* DMA */
	if (!irq) {
		dev_err(&pdev->dev, "no dma_irq resource\n");
		err = -ENODEV;
		goto nores;
	}

	dev->irq_dma = irq->start;
	dev->dev = &pdev->dev;

	err = request_irq(dev->irq_err, stmp378x_i2c_isr, 0, pdev->name, dev);
	if (err) {
		dev_err(&pdev->dev, "Can't get IRQ\n");
		goto no_err_irq;
	}

	err = request_irq(dev->irq_dma, stmp378x_i2c_dma_isr, 0, pdev->name, dev);
	if (err) {
		dev_err(&pdev->dev, "Can't get IRQ\n");
		goto no_dma_irq;
	}

	err = hw_i2c_init(&pdev->dev);
	if (err) {
		dev_err(&pdev->dev, "HW Init failed\n");
		goto init_failed;
	}

	HW_I2C_CTRL1_SET(0x0000FF00); /* Will catch all error (IRQ mask) */

	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	strncpy(adap->name, "378x I2C adapter", sizeof(adap->name));
	adap->algo = &stmp378x_i2c_algo;
	adap->dev.parent = &pdev->dev;

	adap->nr = pdev->id;
	err = i2c_add_numbered_adapter(adap);
	if (err) {
		dev_err(&pdev->dev, "Failed to add adapter\n");
		goto no_i2c_adapter;

	}


    // Init the waitqueue we'll use to wait for the IRQ to fire when we
    // submit data to the I2C controller.
//    init_waitqueue_head(&dev->cmd_wq);

    platform_set_drvdata(pdev, dev);


	return 0;

no_i2c_adapter:
	hw_i2c_stop(dev->dev);
init_failed:
	free_irq(dev->irq_dma, dev);
no_dma_irq:
	free_irq(dev->irq_err, dev);
no_err_irq:
nores:
	kfree(dev);
	return err;
}

static int
stmp378x_i2c_remove(struct platform_device *pdev)
{
	struct stmp378x_i2c_dev	*dev = platform_get_drvdata(pdev);
	int res;

    if(!pdev) {
        printk("Platform device specified was NULL!\n");
        return -EBUSY;
    }
    if(!dev) {
        printk("Decoded device was NULL!\n");
        return -EBUSY;
    }
    /*
    if(!dev->adapter->nr) {
        printk("Adapter number portion of device was NULL!\n");
        return -EBUSY;
    }
    */

    platform_set_drvdata(pdev, NULL);

    printk("Unloading adapter adap->nr: %d\n", dev->adapter.nr);


	res = i2c_del_adapter(&dev->adapter);
	if (res)
		return -EBUSY;

	hw_i2c_stop(dev->dev);

	platform_set_drvdata(pdev, NULL);

	free_irq(dev->irq_err, dev);
	free_irq(dev->irq_dma, dev);

	kfree(dev);
	return 0;
}

static struct platform_driver stmp378x_i2c_driver = {
	.probe		= stmp378x_i2c_probe,
	.remove		= __devexit_p(stmp378x_i2c_remove),
	.driver		= {
		.name	= "i2c_stmp",
		.owner	= THIS_MODULE,
	},
};

/* I2C may be needed to bring up other drivers */

static int __init stmp378x_i2c_init_driver(void)
{
	return platform_driver_register(&stmp378x_i2c_driver);
}
subsys_initcall(stmp378x_i2c_init_driver);

static void __exit stmp378x_i2c_exit_driver(void)
{
	platform_driver_unregister(&stmp378x_i2c_driver);
}
module_exit(stmp378x_i2c_exit_driver);

MODULE_AUTHOR("old_chap@embeddedalley.com");
MODULE_DESCRIPTION("IIC for Freescale STMP378x");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:i2c_stmp");

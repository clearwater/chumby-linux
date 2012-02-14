/*
 * Freescale STMP378X I2C low-level/dma functions
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <linux/dma-mapping.h>
#include <mach/hardware.h>
#include <mach/regs-i2c.h>
#include <mach/regs-apbx.h>
#include <mach/dma.h>
#include <mach/i2c.h>

#include "common.h"

static unsigned int dma_channel =
	STMP3xxx_DMA(STMP378X_APBX_I2C, STMP3XXX_BUS_APBX);


static struct stmp3xxx_dma_descriptor i2c_dma_writeread[2];
static struct stmp3xxx_dma_descriptor i2c_dma_read[2];
static struct stmp3xxx_dma_descriptor i2c_dma_write;
static dma_addr_t i2c_buf_phys;
static u8 *i2c_buf_virt;


/*
 * Select device to read from
 */

u32 cmd_i2c_select[4] = {
	0, /* Chain to i2c_read */

	(BF_APBX_CHn_CMD_XFER_COUNT(1) |
	 /*     BM_APBX_CHn_CMD_SEMAPHORE     | */
	 BF_APBX_CHn_CMD_CMDWORDS(1)   |
	 BM_APBX_CHn_CMD_WAIT4ENDCMD   |
	 BM_APBX_CHn_CMD_CHAIN	 |
	 BM_APBX_CHn_CMD_IRQONCMPLT    | /* For debug*/
	 BF_APBX_CHn_CMD_COMMAND(BV_APBX_CHn_CMD_COMMAND__DMA_READ)),

	0, /* dma handler */

	BM_I2C_CTRL0_RETAIN_CLOCK   |
	BM_I2C_CTRL0_PRE_SEND_START |
	BM_I2C_CTRL0_MASTER_MODE    |
	BM_I2C_CTRL0_DIRECTION      |
	BF_I2C_CTRL0_XFER_COUNT(1)

};

u32 cmd_i2c_write[4] = {
	0,

	(BM_APBX_CHn_CMD_SEMAPHORE     |
	 BF_APBX_CHn_CMD_CMDWORDS(1)   |
	 BM_APBX_CHn_CMD_WAIT4ENDCMD   |
	 BM_APBX_CHn_CMD_IRQONCMPLT    |
	 BF_APBX_CHn_CMD_COMMAND(BV_APBX_CHn_CMD_COMMAND__DMA_READ)),

	0, /* dma handler */

	BM_I2C_CTRL0_PRE_SEND_START |
	BM_I2C_CTRL0_MASTER_MODE    |
/*      BM_I2C_CTRL0_POST_SEND_STOP | */
	BM_I2C_CTRL0_DIRECTION

};

u32 cmd_i2c_writeread[4] = {
    // Set the next_command_address to 0, as it will get set when we go to
    // use this DMA chain object.
	0,



    // Set up the DMA options.
    
    // The first transfer moves two bytes.
	(BF_APBX_CHn_CMD_XFER_COUNT(2)  | 

     // Send only one command word, to HW_I2C_CTRL0.
	 BF_APBX_CHn_CMD_CMDWORDS(1)    |  

     // Wait until the transfer is completed before starting another.
	 BM_APBX_CHn_CMD_WAIT4ENDCMD    |

     // This isn't the last packet; we have another one after it.
	 BM_APBX_CHn_CMD_CHAIN	        |

     // signal us when the transfer is done.
	 BM_APBX_CHn_CMD_IRQONCMPLT     |

     // Indicate we're transferring from memory to the I2C device.
	 BF_APBX_CHn_CMD_COMMAND(BV_APBX_CHn_CMD_COMMAND__DMA_READ)),


    // Set the buffer pointer to 0, as it will also get set when we go to
    // use this chain object.
	0, /* dma handler */


    // Set up the flags to write to the I2C control register.
	BM_I2C_CTRL0_RETAIN_CLOCK       |
	BM_I2C_CTRL0_PRE_SEND_START     |
	BM_I2C_CTRL0_MASTER_MODE        |
/*      BM_I2C_CTRL0_POST_SEND_STOP | */
	BM_I2C_CTRL0_DIRECTION          |
    BM_I2C_CTRL0_SEND_NAK_ON_LAST   |
	BF_I2C_CTRL0_XFER_COUNT(2)
};


u32 cmd_i2c_read[4] = {
	0,

	(BM_APBX_CHn_CMD_SEMAPHORE       |
	 BF_APBX_CHn_CMD_CMDWORDS(1)     |
	 BM_APBX_CHn_CMD_WAIT4ENDCMD     |
	 BM_APBX_CHn_CMD_IRQONCMPLT    |
	 BF_APBX_CHn_CMD_COMMAND(BV_APBX_CHn_CMD_COMMAND__DMA_WRITE)),

	0, /* dma handler */

	BM_I2C_CTRL0_SEND_NAK_ON_LAST |
/*    BM_I2C_CTRL0_POST_SEND_STOP  | */
	BM_I2C_CTRL0_MASTER_MODE	  |
	0   /*BF_I2C_CTRL0_DIRECTION(BV_I2C_CTRL0_DIRECTION__RECEIVE)*/
};


int hw_i2c_init_dma(struct device *dev)
{
	int ret;

	ret = stmp3xxx_dma_request(dma_channel,	dev, "i2c");
	if (ret) {
		dev_err(dev, "stmp3xxx_dma_request failed: error %d\n", ret);
		return ret;
	}

	i2c_buf_virt =
		dma_alloc_coherent(
			dev,
			PAGE_SIZE,
			&i2c_buf_phys,
			GFP_KERNEL);

	if (i2c_buf_virt == NULL)
		return -ENOMEM;


	stmp3xxx_dma_allocate_command(
		dma_channel,
		&i2c_dma_read[0]);

	stmp3xxx_dma_allocate_command(
		dma_channel,
		&i2c_dma_read[1]);

	stmp3xxx_dma_allocate_command(
		dma_channel,
		&i2c_dma_writeread[0]);

	stmp3xxx_dma_allocate_command(
		dma_channel,
		&i2c_dma_writeread[1]);

	stmp3xxx_dma_allocate_command(
		dma_channel,
		&i2c_dma_write);

	stmp3xxx_dma_reset_channel(dma_channel);
	stmp3xxx_dma_clear_interrupt(dma_channel);
	stmp3xxx_dma_enable_interrupt(dma_channel);
	return 0;
};

void hw_i2c_free_dma(struct device *dev)
{
	stmp3xxx_dma_free_command(
		dma_channel,
		&i2c_dma_write);

	stmp3xxx_dma_free_command(
		dma_channel,
		&i2c_dma_writeread[1]);

	stmp3xxx_dma_free_command(
		dma_channel,
		&i2c_dma_writeread[0]);

    /*
	stmp3xxx_dma_free_command(
		dma_channel,
		&i2c_dma_read[2]);
    */

	stmp3xxx_dma_free_command(
		dma_channel,
		&i2c_dma_read[1]);

	stmp3xxx_dma_free_command(
		dma_channel,
		&i2c_dma_read[0]);

	dma_free_coherent(
		dev,
		PAGE_SIZE,
		i2c_buf_virt,
		i2c_buf_phys);

	stmp3xxx_dma_release(dma_channel);
}

void hw_i2c_clear_dma_interrupt(void)
{
	stmp3xxx_dma_clear_interrupt(dma_channel);
}
EXPORT_SYMBOL(hw_i2c_clear_dma_interrupt);

void hw_i2c_setup_write(u8 addr, void *buff, int len, int flags)
{

	memcpy(i2c_dma_write.command, &cmd_i2c_write, sizeof(cmd_i2c_write));

	i2c_dma_write.command->cmd |=
		BF_APBX_CHn_CMD_XFER_COUNT(len+1);

	i2c_dma_write.command->pio_words[0] |=
		BF_I2C_CTRL0_XFER_COUNT(len+1) | flags;

	i2c_dma_write.command->buf_ptr = i2c_buf_phys;
	i2c_buf_virt[0] = addr | I2C_WRITE ;
	memcpy(&i2c_buf_virt[1], buff, len);
}
EXPORT_SYMBOL(hw_i2c_setup_write);

void hw_i2c_finish_read(void *buff, int len)
{
	memcpy(buff, &i2c_buf_virt[1], len);

}
EXPORT_SYMBOL(hw_i2c_finish_read);

void hw_i2c_finish_writeread(void *buff_v, int len)
{
    char *buff = (char *)buff_v;
//    printk("Copying result: 0x%08x\n", *((long *)i2c_buf_virt));
	memcpy(buff+1, &i2c_buf_virt[2], len-1);
}
EXPORT_SYMBOL(hw_i2c_finish_writeread);

void hw_i2c_setup_read(u8 addr, void *buff, int len, int flags)
{

	if (len > (PAGE_SIZE - 4))
		BUG();

	memcpy(i2c_dma_read[0].command,
	       &cmd_i2c_select,
	       sizeof(cmd_i2c_select));

	memcpy(i2c_dma_read[1].command,
	       &cmd_i2c_read,
	       sizeof(cmd_i2c_read));

	i2c_dma_read[0].command->next = i2c_dma_read[1].handle;
	i2c_dma_read[0].command->buf_ptr = i2c_buf_phys ;
	i2c_buf_virt[0] = addr | I2C_READ ;

	i2c_dma_read[1].command->cmd |=	BF_APBX_CHn_CMD_XFER_COUNT(len);

	i2c_dma_read[1].command->pio_words[0] |=
		BF_I2C_CTRL0_XFER_COUNT(len) | flags;

	i2c_dma_read[1].command->buf_ptr = (u32)i2c_buf_phys + 1 ;
	memcpy(&i2c_buf_virt[1], buff, len);

}
EXPORT_SYMBOL(hw_i2c_setup_read);

void hw_i2c_setup_writeread(u8 addr, void *buff, int len, int flags) {
	if (len > (PAGE_SIZE - 4))
		BUG();

	memcpy(i2c_dma_writeread[0].command,
	       &cmd_i2c_writeread,
	       sizeof(cmd_i2c_writeread));

	memcpy(i2c_dma_writeread[1].command,
	       &cmd_i2c_read,
	       sizeof(cmd_i2c_read));

    // Point the next DMA transfer at the second DMA operation.
	i2c_dma_writeread[0].command->next = i2c_dma_writeread[1].handle;
	i2c_dma_writeread[0].command->buf_ptr = i2c_buf_phys;

    // Ensure we're doing a read operation.  We're here, so the address
    // should already be set, but make sure anyway.
	i2c_buf_virt[0] = addr | I2C_READ;

    // Copy the address we're reading from as the second byte to send.
	memcpy(&i2c_buf_virt[1], buff, 1);

    // We'll be sending two bytes on this operation: The address of the I2C
    // device, and the register we want to select.
	i2c_dma_writeread[0].command->cmd |= BF_APBX_CHn_CMD_XFER_COUNT(2);

	i2c_dma_writeread[0].command->pio_words[0] |=
		BF_I2C_CTRL0_XFER_COUNT(2) | flags;



    // Set up the read command to read 1 byte.
	i2c_dma_writeread[1].command->cmd |=	BF_APBX_CHn_CMD_XFER_COUNT(len-1);
	i2c_dma_writeread[1].command->pio_words[0] |=
		BF_I2C_CTRL0_XFER_COUNT(len-1) | flags;

    // Point the "read" buffer at the next available free spot in memory.
    // We've used up two bytes above for the i2c address and register
    // address.
	i2c_dma_writeread[1].command->buf_ptr = (u32)i2c_buf_phys + 2;
}
EXPORT_SYMBOL(hw_i2c_setup_writeread);

void hw_i2c_run(int dir)
{
	if (dir == I2C_WRITE)
		stmp3xxx_dma_go(dma_channel, &i2c_dma_write, 1);
	else if(dir == I2C_READ)
		stmp3xxx_dma_go(dma_channel, &i2c_dma_read[0], 1);
    else
		stmp3xxx_dma_go(dma_channel, &i2c_dma_writeread[0], 1);
}
EXPORT_SYMBOL(hw_i2c_run);

void hw_i2c_reset_dma(void)
{
	stmp3xxx_dma_reset_channel(dma_channel);
	stmp3xxx_dma_clear_interrupt(dma_channel);
}
EXPORT_SYMBOL(hw_i2c_reset_dma);


int hw_i2c_init(struct device *dev)
{
	if (stmp3xxx_request_pin_group(&i2c_pins, "i2c"))
		return -1;


	/* Take controller out of reset */
	HW_I2C_CTRL0_CLR(BM_I2C_CTRL0_SFTRST | BM_I2C_CTRL0_CLKGATE);
	udelay(10);

/*       * Set timing
	 * High time = 120 clks; read bit at 48 for 95Khz/24mhz
	 * Low  time = 128 clks; write bit at 48 for 95khz/24mhz
*/

/*
	Don't set 400khz by default; stfm1000 needs 100khz at the start.
	HW_I2C_TIMING0_WR(0x00780030);
	HW_I2C_TIMING1_WR(0x001F000F);
	HW_I2C_TIMING2_WR(0x0015000D);
*/
    /* 9/30/2009 SMC - Yes, but we're not using the STFM1000, so there. */
    /*
	HW_I2C_TIMING0_WR(0x000F0007);
	HW_I2C_TIMING1_WR(0x001F000F);
	HW_I2C_TIMING2_WR(0x0015000D);
    */

	dev_dbg(dev, "I2C module version %x\n ", HW_I2C_VERSION_RD());
	hw_i2c_init_dma(dev);
	return 0;
}
EXPORT_SYMBOL(hw_i2c_init);

void hw_i2c_stop(struct device *dev)
{
	HW_I2C_CTRL0_SET(BM_I2C_CTRL0_SFTRST);
	hw_i2c_reset_dma();
	hw_i2c_free_dma(dev);
	stmp3xxx_release_pin_group(&i2c_pins, "i2c");
}
EXPORT_SYMBOL(hw_i2c_stop);

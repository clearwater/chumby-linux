/*
 * Linux glue to STMP3xxx battery state machine.
 *
 * Author: Steve Longerbeam <stevel@embeddedalley.com>
 *
 * Copyright (C) 2008 EmbeddedAlley Solutions Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include "ddi_bc_internal.h"
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <mach/ddi_bc.h>
#include <mach/regulator.h>
#include <mach/regs-power.h>
#include <mach/regs-usbphy.h>
#include <mach/regs-pinctrl.h>
#include <mach/regs-clkctrl.h>
#include <mach/regs-pwm.h>
#include <mach/regs-pxp.h>
#include <mach/regs-rtc.h>
#include <mach/cpu.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/reboot.h>

#include <linux/interrupt.h>
#include <linux/fs.h>


#define TEMP_UPPER_BOUND 80
#define TEMP_LOWER_BOUND 75
#define NEED_TO_CHANGE_STATE_CYCLES 2
#define BROWNOUT_VOLTAGE 2880000

// How many times through the loop should we allow the battery to be under
// alarm before shutting the system down.
#define MAX_BATTERY_ALARMS 5

#define CHLOG(format, arg...)            \
        printk("power/linux.c - %s():%d - " format, __func__, __LINE__, ## arg)


#define POWER_STATE_ON 1
#define POWER_STATE_OFF 0
static int power_state = POWER_STATE_ON;
struct task_struct *th = NULL;
static struct stmp3xxx_info *g_info;


struct stmp3xxx_info {
	struct device *dev;
	struct regulator *regulator;

	struct power_supply bat;
	struct power_supply ac;
	struct power_supply usb;

	ddi_bc_Cfg_t *sm_cfg;
	struct mutex sm_lock;
	struct timer_list sm_timer;
	struct work_struct sm_work;
	struct work_struct park_work;
	struct resource *vdd5v_irq;
	int is_ac_online;
#define USB_ONLINE      0x01
#define USB_REG_SET     0x02
#define USB_SM_RESTART  0x04
#define USB_SHUTDOWN    0x08
#define USB_N_SEND      0x10
#define AC_ONLINE      0x01
#define AC_REG_SET     0x02
#define AC_SM_RESTART  0x04
#define AC_SHUTDOWN    0x08
#define AC_N_SEND      0x10
	int is_usb_online;
	struct mutex freeze_lock;

    int current_limit;
    int current_wait_cycles;

    int need_to_change_state;
	
// Used for voltage smoothing.
	int voltage_offset;
	unsigned int voltages[16];
	unsigned int voltage;

// Set to true if park_work should shut down the system after it's done.
    unsigned int park_should_shutdown;

// Keeps track of how many times the battery has been under alarm.
    int battery_alarm_count;

// Only check for a failed DCDC every so often
	unsigned int last_dcdc_validate_time;
};

#define to_stmp3xxx_info(x) container_of((x), struct stmp3xxx_info, bat)
static void power_switch_pressed(void);

/* The chumby device doesn't support USB power, so if 5V is present, it has
 * to be AC.  This is backwards from the way the chip is designed, which is
 * to run primarily off of USB power.
 */
#define is_ac_online()       ddi_power_Get5vPresentFlag()
#define is_usb_online()	     0
#define is_usb_plugged()     0
/* A battery reading of 3190 means the battery is freezing.  3200 means
 * it's even colder.  If the battery is that cold, /and/ the voltage is
 * less than 2000, it means we don't have a battery.
 */
static inline int get_battery_temp(void) {
    unsigned short battery_temp;
    ddi_bc_hwGetBatteryTemp(&battery_temp);
    return battery_temp;
}
#define is_battery_present() ((chumby_revision()!=0)&&(chumby_revision()!=7)&&(get_battery_temp()<3200))
//#define is_battery_present() ((ddi_power_GetBattery()>=2000))
/*
#define is_usb_plugged()(HW_USBPHY_STATUS_RD() & \
		BM_USBPHY_STATUS_DEVPLUGIN_STATUS)
#define is_ac_online()	\
		(ddi_power_Get5vPresentFlag() ? (!is_usb_plugged()) : 0)
#define is_usb_online()	\
		(ddi_power_Get5vPresentFlag() ? (!!is_usb_plugged()) : 0)
*/



/*
 * Power properties
 */
static enum power_supply_property stmp3xxx_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TEMP,
};

static int stmp3xxx_power_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct stmp3xxx_info *info;
	int16_t temp_lo, temp_hi;
    if(psy->type == POWER_SUPPLY_TYPE_MAINS)
        info = container_of((psy), struct stmp3xxx_info, ac);
    else
        info = container_of((psy), struct stmp3xxx_info, usb);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			/* ac online */
			val->intval = is_ac_online();
		else
			/* usb online */
			val->intval = is_usb_online();
		break;

    case POWER_SUPPLY_PROP_TEMP:
		mutex_lock(&info->sm_lock);
        ddi_power_GetDieTemp(&temp_lo, &temp_hi);
		mutex_unlock(&info->sm_lock);
		val->intval = temp_lo + (temp_hi - temp_lo) / 2;
        break;

	default:
		return -EINVAL;
	}

	return 0;
}
/*
 * Battery properties
 */
static enum power_supply_property stmp3xxx_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_CAPACITY, ///// Need to add! /////
};






static int stmp3xxx_bat_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct stmp3xxx_info *info = to_stmp3xxx_info(psy);
	ddi_bc_State_t state;
	ddi_bc_BrokenReason_t reason;
	int temp_alarm;
    uint16_t batt_temp;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		state = ddi_bc_GetState();

		switch (state) {

		case DDI_BC_STATE_CONDITIONING:
		case DDI_BC_STATE_CHARGING:
		case DDI_BC_STATE_TOPPING_OFF:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;

		case DDI_BC_STATE_DISABLED:
		case DDI_BC_STATE_BROKEN:
		case DDI_BC_STATE_UNINITIALIZED:
			val->intval = (ddi_power_Get5vPresentFlag()
				? POWER_SUPPLY_STATUS_NOT_CHARGING
				: POWER_SUPPLY_STATUS_DISCHARGING);
			break;

        case DDI_BC_STATE_WAITING_TO_CHARGE:
            // If the voltage is greater-than-or-equal-to 4.1V, say it's
            // full.  Otherwise, we're just not charging for some reason.
            val->intval = (info->voltage>=4100000
                ? POWER_SUPPLY_STATUS_FULL
                : POWER_SUPPLY_STATUS_NOT_CHARGING);
            break;

		default:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		}

		// Catch-all to report DISCHARGING if we're not plugged in.
		if(!ddi_power_Get5vPresentFlag())
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;

		break;


	case POWER_SUPPLY_PROP_PRESENT:
		/* is battery present */
        val->intval = is_battery_present();
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		temp_alarm = ddi_bc_RampGetDieTempAlarm();
		if (temp_alarm) {
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		} else {
			state = ddi_bc_GetState();
			switch (state) {
			case DDI_BC_STATE_BROKEN:
				reason = ddi_bc_GetBrokenReason();
				val->intval =
				   (reason == DDI_BC_BROKEN_CHARGING_TIMEOUT) ?
					POWER_SUPPLY_HEALTH_DEAD :
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
				break;
			case DDI_BC_STATE_UNINITIALIZED:
				val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
				break;
			default:
				val->intval = POWER_SUPPLY_HEALTH_GOOD;
				break;
			}
		}
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		/* uV */
		val->intval = info->voltage;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* uV */
		val->intval = ddi_power_GetBattery()*1000;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* uA */
		val->intval = ddi_power_GetMaxBatteryChargeCurrent() * 1000;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		mutex_lock(&info->sm_lock);
        ddi_bc_hwGetBatteryTemp(&batt_temp);
		val->intval = batt_temp;
		mutex_unlock(&info->sm_lock);
		break;

    case POWER_SUPPLY_PROP_CAPACITY:
        /* XXX We need to come up with a value here! */
        /* The range goes 2900 - 4200, so scale it down to a percentage */
        /* Actually, we'll make the range 3000-3700.  */
        val->intval = ((info->voltage/1000) - 3000) / 7;
		if(val->intval < 0)
			val->intval = 0;
		if(val->intval > 100)
			val->intval = 100;
        break;

	default:
		return -EINVAL;
	}

	return 0;
}

static void state_machine_timer(unsigned long data)
{
	struct stmp3xxx_info *info = (struct stmp3xxx_info *)data;
	ddi_bc_Cfg_t *cfg = info->sm_cfg;
	int ret;

	/* schedule next call to state machine */
	mod_timer(&info->sm_timer,
		  jiffies + msecs_to_jiffies(cfg->u32StateMachinePeriod));

	ret = schedule_work(&info->sm_work);
	if (!ret)
		dev_dbg(info->dev, "state machine failed to schedule\n");

}


/*
static int get_5v_present() {
    int val = HW_POWER_STS_RD();
    //printk("Determining if 5V is present: 0x%08x\n", val);
    val = !!(val & BM_POWER_STS_VBUSVALID_STATUS);
    ddi_power_Set5vPresentFlag(val);
    return val;
}
*/



// This code taken from sysrq.c
static void send_sig_all(int sig)
{
    struct task_struct *p;

    for_each_process(p) {
        if (p->mm && !is_global_init(p))
            /* Not swapper, init nor kernel thread */
            force_sig(sig, p);
    }
}



/* "park" the system by shutting down all processes and unmounting
 * all unnecessary drives
 */
/* Shuts down the system.  Not much more to say, really. */
static void do_shutdown(void) {
//    orderly_poweroff(0);
//    mdelay(10);
//    orderly_poweroff(1);
//    mdelay(10);
//    machine_power_off();
//
//    /* It shouldn't ever get to this machine-specific shutdown call. */
    HW_POWER_RESET_WR(0x3e770001);
}


static void do_park(int shutdown) {
    int rtc_bits  = 0x00000010; // Disables watchdog.
    static int rtc_state;
    int result;

    /* Kill all processes.  Later on we'll write code that actually
     * manages a sleep state, but for now just send SIGTERM then
     * SIGKILL and hope they clean up after themselves.
     */
    send_sig_all(SIGTERM);
    mdelay(1000);
    send_sig_all(SIGKILL);
    mdelay(100);


    /* Now that all processes are frozen or dead, including watchdog,
     * disable the watchdog timer.  Otherwise it will reboot us, and
     * we don't want that.
     */
    rtc_state = HW_RTC_CTRL_RD()&rtc_bits;
    if(rtc_state)
        HW_RTC_CTRL_CLR(rtc_bits);


    /* Remount filesystems RO, in case we lose power.
     * Also remount them for when we actually power off.
     */
    if((result=do_mount(NULL, "/mnt/storage", NULL, MS_RDONLY | MS_REMOUNT, NULL)))
        printk("Error unmounting /mnt/storage: %d\n", result);
    if((result=do_mount(NULL, "/psp", NULL, MS_RDONLY | MS_REMOUNT, NULL)))
        printk("Error unmounting /psp: %d\n", result);
    mdelay(100);
    sync_filesystems(10);



    /* Turn LOCAL_5V_ON off.  This shouldn't have any effect when
     * running on AC, but on battery it will disable the wifi and USB
     * cards.  So this will also unmount most USB drives, if the
     * voltage is low enough.
     */
    HW_PINCTRL_DOUT0_CLR(0x00001000);

    if(shutdown)
        do_shutdown();

    return;
}

/* Allow code to call do_park() from an interrupt context. */
static void do_park_work(struct work_struct *work) {
	struct stmp3xxx_info *info =
		container_of(work, struct stmp3xxx_info, park_work);
    do_park(info->park_should_shutdown);
}



// Power the system down.  If we're on AC, just leave the system in the
// powered-up-but-appears-off state.  If we're on battery, shut it down.
void stmp3xxx_power_down(void) {
    struct stmp3xxx_info *info = g_info;
    /* These indicate which bits we're interested in flipping. */
    int xtal_bits = 0xfc000000; // 31, 30, 29, 28, 27, 26
    int pwm_bits  = 0x0000001f; // Bits controlling the state of the 5 pwms.
    int pix_bits  = 0x80000000; // Gates off LCDIF.
    int pxp_bits  = 0x80000000; // Gates off PXP.
    static int xtal_state;
    static int pwm_state;
    static int pix_state;
    static int pxp_state;


    printk(">>> Detected power switch pressed.  Turning off.\n");
    printk(">>> A note to those of you watching at home:\n");
    printk(">>> We're going to kill all processes and unmount all filesystems.\n");
    printk(">>> Then we'll sit here charging the battery.\n");
    printk(">>> If you press the power button again, we'll reboot.\n");
    printk(">>> If you unplug this device, we'll power off for real.\n");
    printk(">>> But for now, we'll just make the device look like it's off.\n");

    /* Turn the LCD off */
    HW_PINCTRL_MUXSEL3_SET(0x03000000);
    HW_PINCTRL_DOUT1_CLR(0x10000000);
    HW_PINCTRL_DOE1_SET(0x10000000);

    /* Disable USB power, and bring USB RESET high. */
    HW_PINCTRL_DOUT0_CLR(0x24000000);

    pix_state = HW_CLKCTRL_PIX_RD();
    HW_CLKCTRL_PIX_WR(pix_state|pix_bits);

    pwm_state = HW_PWM_CTRL_RD();
    HW_PWM_CTRL_CLR(pwm_bits);

    pxp_state = HW_PXP_CTRL_RD();
    HW_PXP_CTRL_CLR(pxp_bits);

    xtal_state = HW_CLKCTRL_XTAL_RD();
    HW_CLKCTRL_XTAL_CLR(xtal_bits);

    power_state = POWER_STATE_OFF;

    info->park_should_shutdown = !is_ac_online();
    schedule_work(&info->park_work);
}


// Ensures the device is "powered up".  If power_state is OFF, will power
// off, then allow the chip to power itself up.
void stmp3xxx_ensure_power(void) {
    CHLOG("Ensuring power.  Will%s shutdown.\n", 
            POWER_STATE_OFF==power_state?"":" not");
    if(POWER_STATE_OFF==power_state) {
        do_shutdown();
//        g_info->park_should_shutdown = 1;
//        schedule_work(&g_info->park_work);
    }
}
EXPORT_SYMBOL(stmp3xxx_ensure_power);



/* Updates the voltage value reported to the system, which is averaged
 * across 16 or so samples.
 */
void validate_voltage(struct stmp3xxx_info *info) {
    static int count;
	int voltage_count = 16;

	// Grab the current voltage.  We're going to remove the oldest voltage
	// from the running average.
	unsigned int average_voltage = 0;
    int i;


    /* Only run this loop occasionally. */
    if(count++ < 5)
        return;
    else
        count = 0;

	info->voltages[info->voltage_offset] = ddi_power_GetBattery();


    /* Perform the averaging.
     * XXX Note that this really ought to be improved.
     */
    average_voltage = 0;
    for(i=0; i<voltage_count; i++)
        average_voltage += info->voltages[i];


    /* Note that the voltage reported to the system is 1000 times greater
     * than the voltage reported by the battery.  This conversion needs to
     * happen whenever reporting the voltage to the system.
     */
    average_voltage /= voltage_count;
    average_voltage *= 1000;



//    CHLOG("Average voltage: %d  Voltage offset: %d  Instant voltage: %d\n",
//            average_voltage, info->voltage_offset, voltage*1000);

	/* Deal with incrementing and wrapping the current voltage pointer. */
    info->voltage_offset++;
    if(info->voltage_offset >= voltage_count)
		info->voltage_offset = 0;


	/* Update the voltage in the structure that everyone will use. */
	info->voltage = average_voltage;


	/* Now, determine if the new voltage is too low, necessitating we shut down.
     * Note that this doesn't apply if we're on AC, obviously.
     */
	if(!is_ac_online() && average_voltage <= BROWNOUT_VOLTAGE) {
		printk("Voltage really, really low (%d vs %d), so powering off\n",
               average_voltage, BROWNOUT_VOLTAGE);
        do_park(1);
	}
}




/* Ensure that the current temperature is within a sane range.  If it
 * isn't, either due to loud music or a lot of current going to the
 * battery, then lower the charging rate.
 */
extern uint16_t ddi_bc_hwGetMaxCurrent(void);
void validate_temperature(struct stmp3xxx_info *info) {
	int16_t temp_lo, temp_hi, temp;

    ddi_power_GetDieTemp(&temp_lo, &temp_hi);
    temp = temp_lo + (temp_hi - temp_lo) / 2;

#if 0
    CHLOG("Still workin'.  Temp: %d  Max current: %d  Voltage: %d\n",
          temp, ddi_bc_hwGetMaxCurrent(), ddi_power_GetBattery()*1000);
#endif

    if(temp >= TEMP_UPPER_BOUND) {
#ifdef CONFIG_POWER_SUPPLY_DEBUG
//        CHLOG("!!! Warning: Temperature very hot: %d.\n", temp);
#endif
        // We can't lower the current to 0, because then the battery
        // charger will think we're done charging and want to go top off
        // the battery.
        // We may also want to adjust this value if the state is one of:
        //      DDI_BC_STATE_CONDITIONING
        //      DDI_BC_STATE_TOPPING_OFF
        if(is_ac_online() 
                && info->current_limit > 10
                && ddi_bc_GetState() == DDI_BC_STATE_CHARGING
                && !info->current_wait_cycles) {
            int hw_max = ddi_bc_hwGetMaxCurrent();


            /* If the hardware is currently set far less than our
             * internal current limit, say by 50 mA, set our limit to be
             * equal to that of the current hardware max.
             * This will prevent us from going beyond the limits set by the
             * state machine.
             */
            if(info->current_limit - hw_max <= 50)
                info->current_limit = hw_max;


            /* We want to be at least 20, because we'll subtract 10 from
             * this later, and we want to always be at least 10 mA after
             * we're done adjusting the current.
             */
            if(info->current_limit < 20)
                info->current_limit = 20;

            /* Knock 10 mA off of the charging current.  Eventually we'll
             * be within range of something that doesn't cause the chip to
             * overheat.
             */
            info->current_limit -= 10;
#ifdef CONFIG_POWER_SUPPLY_DEBUG
            CHLOG("!!! Temperature sufficiently hot: %d.  "
                  "Raising charging current: %d\n", temp, info->current_limit);
#endif
            ddi_bc_SetCurrentLimit(info->current_limit); //mA
            if(info->regulator)
                regulator_set_current_limit(info->regulator,
                                            info->current_limit*1000,
                                            info->current_limit*1000);
            info->current_wait_cycles = 2;
        }
        else if(info->current_wait_cycles) {
#ifdef CONFIG_POWER_SUPPLY_DEBUG
//            CHLOG("!!! Will lower current to %d in %d cycles\n", 
//                    info->current_limit-10, info->current_wait_cycles);
#endif
            info->current_wait_cycles--;
        }
    }

    else if(temp <= TEMP_LOWER_BOUND) {
        if(is_ac_online()) {
            if(info->current_limit < 600
                && ddi_bc_GetState() == DDI_BC_STATE_CHARGING
                && !info->current_wait_cycles) {
            info->current_limit += 10;
#ifdef CONFIG_POWER_SUPPLY_DEBUG
                CHLOG(">>> Temperature sufficiently low: %d.  "
                      "Raising charging current: %d\n",
                      temp, info->current_limit);
#endif
                ddi_bc_SetCurrentLimit(info->current_limit); //mA
                if(info->regulator)
                    regulator_set_current_limit(info->regulator,
                                                info->current_limit*1000,
                                                info->current_limit*1000);
                info->current_wait_cycles = 10;
            }
            else if(info->current_wait_cycles) {
#ifdef CONFIG_POWER_SUPPLY_DEBUG
//                CHLOG("!!! Temperature sufficiently low: %d.  "
//                      "Raising charging current to %d in %d cycles\n",
//                      temp, info->current_limit, info->current_wait_cycles);
#endif
                info->current_wait_cycles--;
            }
        }
    }
}



/*
 * Validates that the DCDC is up and running.  Sometimes it can die due to
 * brownout, mostly when moving to AC after being on a low battery.
 */
static void validate_dcdc4p2(struct stmp3xxx_info *info) {
	int state;

	/* Only perform this check if we're not on battery. */
	if(!is_ac_online())
		return;

	/*
	 * Also, if there is no battery, then the reset procedure will fail.
	 * In fact, we shouldn't even be executing code if the DCDC is off.
	 */
	if(!is_battery_present())
		return;


	/* If the battery isn't charging, well, we can't check then. */
	state = ddi_bc_GetState();
	if(state != DDI_BC_STATE_CONDITIONING
	&& state != DDI_BC_STATE_CHARGING
	&& state != DDI_BC_STATE_TOPPING_OFF)
		return;

	/* If the voltage is sufficiently high, then the DCDC is probably on. */
	if(info->voltage>=3300000)
		return;

	/* Only check once per second. */
	if(jiffies_to_msecs(jiffies) < info->last_dcdc_validate_time+1000)
		return;
	info->last_dcdc_validate_time = jiffies_to_msecs(jiffies);

	/* 
	 * By this point, it seems as though the DCDC is not running.
	 * Re-enable the charger and dcdc.  Work around a hardware bug
	 * by disabling it and then re-enabling it.
	 */
	CHLOG("The DCDC appears to be off.  Restarting it...\n");
	HW_POWER_5VCTRL_SET(BM_POWER_5VCTRL_PWD_CHARGE_4P2);
	udelay(100);
	HW_POWER_5VCTRL_CLR(BM_POWER_5VCTRL_PWD_CHARGE_4P2);
	udelay(100);
	return;
}



/*
 * Assumtion:
 * AC power can't be switched to USB w/o system reboot
 * and vice-versa
 */
static void state_machine_work(struct work_struct *work) {
	struct stmp3xxx_info *info =
		container_of(work, struct stmp3xxx_info, sm_work);
    ddi_bc_State_t starting_state = ddi_bc_GetState();
    int something_changed = 0;

	mutex_lock(&info->sm_lock);

    /* Tell the state machine the battery is back, if it's detected one
     * isn't there and we can see a battery present now.
     */
    if(starting_state == DDI_BC_STATE_BROKEN 
        && ddi_bc_GetBrokenReason() == DDI_BC_BROKEN_NO_BATTERY_DETECTED
        && is_battery_present()) {
        ddi_bc_SetFixed();
        ddi_bc_SetEnable();
    }

    /* If we need to change state (defined as need_to_change_state-1==0),
     * perform the handoffs.
     */
    if(info->need_to_change_state && !(--info->need_to_change_state)) {
        something_changed = 1;
        if (ddi_power_Get5vPresentFlag() && is_battery_present()) {
            dev_info(info->dev, "5v present, reenable state machine\n");

            ddi_bc_SetEnable();

            /*
            * We only ack/negate the interrupt here,
            * as we can't decide yet if we really can
            * switch to 5V (USB bits not ready)
            */
            CHLOG("Handing off from battery to 5v\n");
            ddi_power_execute_battery_to_5v_handoff();
            CHLOG("Setting up for 5v to battery handoff\n");
            ddi_power_enable_5v_to_battery_handoff();

            /* Re-enable the charger and dcdc.  Work around a hardware bug
             * by disabling it and then re-enabling it. */
            HW_POWER_5VCTRL_SET(BM_POWER_5VCTRL_PWD_CHARGE_4P2);
            udelay(100);
            HW_POWER_5VCTRL_CLR(BM_POWER_5VCTRL_PWD_CHARGE_4P2);
            udelay(100);
        }
        else if(is_battery_present()) {
            dev_info(info->dev, "5v went away, disabling state machine\n");

            ddi_bc_SetDisable();

            info->is_ac_online = 0;
            if (info->is_usb_online)
                info->is_usb_online = USB_SHUTDOWN;
            if (info->is_ac_online)
                info->is_ac_online = AC_SHUTDOWN;

            ddi_power_execute_5v_to_battery_handoff();
            ddi_power_enable_battery_to_5v_handoff();
            // Since the power went away, VBUSVALID isn't good anymore.
            // Clear it.
            HW_POWER_5VCTRL_CLR(0x00000020);
            mod_timer(&info->sm_timer, jiffies + 1);

        }
        else {
            dev_info(info->dev, "power switch pressed, ac present, "
                    "but no battery.  Just going to idle.\n");
        }

        //// If we've changed something, let userspace know.
        //kobject_uevent(&info->dev->kobj, KOBJ_CHANGE);
    }


    /* If AC is online, and we have a battery, and we've just changed
     * states, then AC_REG_SET will be set.  Set up the regulators.
     */
    if(is_battery_present() 
         && is_ac_online()
         && (!(info->is_ac_online&AC_REG_SET)) ) {

        info->is_usb_online = 0;
        info->is_ac_online |= AC_ONLINE;

        if (!info->regulator) {
            CHLOG("Allocating regulator\n");
            info->regulator = regulator_get(NULL, "charger-1");
            if (!info->regulator || IS_ERR(info->regulator)) {
                dev_err(info->dev, "%s: failed to get regulator\n", __func__);
                info->regulator = NULL;
                ddi_bc_SetCurrentLimit(600 /*mA*/);
                info->current_limit = 0;
                ddi_power_execute_battery_to_5v_handoff();
                ddi_power_enable_5v_to_battery_handoff();
                ddi_bc_StateMachine();
                goto out;
            }
            else
                regulator_set_mode(info->regulator,
                        REGULATOR_MODE_FAST);
        }
        else
            CHLOG("Looks like the regulator was already allocated: %p\n",
                    info->regulator);


        /* Attempt to set the regulator current to 1500 mA.
         * I suspect this is a holdover from when this code used to be part
         * of the USB charging mechanism.
         */
        if (regulator_set_current_limit(info->regulator, 150000, 150000)) {
            dev_err(info->dev, "reg_set_current(150000) failed\n");
            ddi_bc_SetCurrentLimit(0 /*mA*/);
            info->current_limit = 0;
            dev_dbg(info->dev, "charge current set to 0\n");
            mod_timer(&info->sm_timer, jiffies + msecs_to_jiffies(1000));
            ddi_bc_StateMachine();
            goto out;
        }

        dev_dbg(info->dev, "%s: charge current set to 600mA\n", __func__);
        info->current_limit = 600;
        ddi_bc_SetCurrentLimit(info->current_limit);
        regulator_set_current_limit(info->regulator,
                                    info->current_limit*1000,
                                    info->current_limit*1000);


        /* Restart the state machine if that bit is set */
        if (info->is_ac_online & AC_SM_RESTART) {
            info->is_ac_online &= ~AC_SM_RESTART;
            ddi_bc_SetEnable();
        }

        info->is_ac_online |= AC_REG_SET;

        dev_info(info->dev, "changed power connection to ac/5v present\n");

        ddi_bc_StateMachine();
	}

    else if(is_battery_present() && is_ac_online()) {
        /* Set charge current to 0 mA if the system requests AC be shut down. */
        if(info->is_ac_online & AC_SHUTDOWN) {
            info->is_ac_online = 0;
            info->current_limit = 0;
            if(info->regulator)
                regulator_set_current_limit(info->regulator,
                                            info->current_limit,
                                            info->current_limit);
        }
        else
            ddi_bc_StateMachine();
    }

    // Let the state machine run one last time to disable itself.
    else if(!is_battery_present()
         &&  is_ac_online()
         && ddi_bc_GetState() != DDI_BC_STATE_DISABLED
         && ddi_bc_GetState() != DDI_BC_STATE_UNINITIALIZED )
        ddi_bc_StateMachine();


out:

	//validate_temperature(info);
	validate_voltage(info);
	validate_dcdc4p2(info);


    // If the battery has been under alarm for too long, power the system
    // down to avoid it catching on fire.
    if(ddi_bc_RampGetBatteryTempAlarm())
        info->battery_alarm_count++;
    else
        info->battery_alarm_count=0;
    if(info->battery_alarm_count>=MAX_BATTERY_ALARMS) {
        CHLOG("!!! WARNING !!! Detected the battery running at %d degrees."
              "  Shutting down to prevent fire\n",
              get_battery_temp());
        stmp3xxx_power_down();
    }



    // If we've changed something, let userspace know.
    if(starting_state != ddi_bc_GetState() || something_changed)
        kobject_uevent(&info->dev->kobj, KOBJ_CHANGE);


	mutex_unlock(&info->sm_lock);
}


static int bc_sm_restart(struct stmp3xxx_info *info)
{
	ddi_bc_Status_t bcret;
	int ret = 0;

	mutex_lock(&info->sm_lock);

	/* ungate power clk */
	ddi_power_SetPowerClkGate(0);

	/*
	 * config battery charger state machine and move it to the Disabled
	 * state. This must be done before starting the state machine.
	 */
	bcret = ddi_bc_Init(info->sm_cfg);
	if (bcret != DDI_BC_STATUS_SUCCESS) {
		dev_err(info->dev, "state machine init failed: %d\n", bcret);
		ret = -EIO;
		goto out;
	}

	/*
	 * Check what power supply options we have right now. If
	 * we're able to do any battery charging, then set the
	 * appropriate current limit and enable. Otherwise, leave
	 * the battery charger disabled.
	 */
	if (is_ac_online() && is_battery_present()) {
		/* ac supply connected */
		dev_info(info->dev, "ac/5v present, enabling state machine\n");

		info->is_ac_online = AC_ONLINE;

        /* Tell the AC that it's now online, and that it should restart the
         * state machine.
         */
		info->is_usb_online = AC_ONLINE | AC_SM_RESTART;
        CHLOG("Setting current limit to 600 mA\n");
        info->current_limit = 600;
		ddi_bc_SetCurrentLimit(info->current_limit);

	}
    else if(!is_ac_online()) {
		/* not powered */
		dev_info(info->dev, "%s: 5v not present\n", __func__);

		info->is_ac_online = 0;
		info->is_usb_online = 0;
		ddi_bc_SetDisable();
	}
    else {
        dev_info(info->dev, "%s: no battery present\n", __func__);
		info->is_ac_online = AC_ONLINE;
		ddi_bc_SetDisable();
    }

	/* schedule first call to state machine */
	mod_timer(&info->sm_timer, msecs_to_jiffies(5000) + jiffies + 1);
    CHLOG("Returning from sm_restart()\n");

out:
	mutex_unlock(&info->sm_lock);
	return ret;
}





/* Either freeze thaw all the processes in the system.
 * This would probably be better served by the kernel's power management
 * state code, but it's what we'll use for now.
 */
static int power_freeze_thaw_processes(void *info_v) {
    struct stmp3xxx_info *info = (struct stmp3xxx_info *)info_v;

    mutex_lock(&info->freeze_lock);

    /* Create another thread, so this process can be re-run on-demand. */
    th = kthread_create(power_freeze_thaw_processes, info_v, "freeze-thaw");
    if (IS_ERR(th)) {
        CHLOG("Unable to start the process-freezing thread\n");
        th = NULL;
    }


    /* If we're currently in the "off" state, hibernate */
    if(POWER_STATE_OFF==power_state) {

        /* Freeze all processes. */
        /* msleep(500); */
        /* freeze_processes(); */


        /* "park" the system by shutting down all processes and unmounting
         * all unnecessary drives.  Shut down if we're not on AC power.
         */
        do_park(!is_ac_online());




        /* If the power state has changed while we're freezing processes,
         * re-run this function.
         */
        if(POWER_STATE_OFF!=power_state)
            wake_up_process(th);
    }

    #if 0
    /* Given our current setup, the "thaw" portion of this process will
     * likely never run.  As soon as we set up the system with different
     * stages of standby, this will mean something again.
     */
    else {
        HW_PINCTRL_DOUT0_SET(0x00001000);
        thaw_processes();

        // If the watchdog timer was enabled before, reset it to max
        // and re-enable it.  This /ought/ to be taken care of by the
        // watchdog process, but we do it here just to be safe.
        if(rtc_state) {
            HW_RTC_WATCHDOG_WR(0xffffffff);
            HW_RTC_CTRL_TOG(rtc_bits);
        }

        // If the state of the power button has changed while executing
        // this function, re-run it.
        if(POWER_STATE_ON!=power_state)
            wake_up_process(th);
    }
    #endif

    mutex_unlock(&info->freeze_lock);

    do_exit(0);
}



static void power_switch_pressed(void) {

    // If we're plugged in and they press the power button, go to
    // "sleep".
    if(POWER_STATE_ON == power_state) {
        stmp3xxx_power_down();
    }
    else {
        printk(">>> Detected power switch pressed.  "
                "Rebooting or powering off.\n");

        /* Power down.  If we're plugged in, the CPU will automatically
         * power back on, effecting a reboot.
         */
        do_shutdown();

        /* The thawing code, which seems to work, is currently disabled
         * as we reboot instead.  May suffer now from bit rot.
         */
        /*
        // Toggle the bits we're interested in back, from 1 to 0.
        HW_CLKCTRL_XTAL_WR(xtal_state);
        HW_PXP_CTRL_WR(pxp_state);
        HW_PWM_CTRL_WR(pwm_state);
        HW_CLKCTRL_PIX_WR(pix_state);

        // Enable USB power, and disable USB reset.
        HW_PINCTRL_DOUT0_SET(0x24000000);

        
        power_state = POWER_STATE_ON;
        */
    }
}



static irqreturn_t stmp3xxx_vdd5v_irq(int irq, void *cookie)
{
    //int val = HW_POWER_STS_RD();
	struct stmp3xxx_info *info = (struct stmp3xxx_info *)cookie;
    int power_ctrl = HW_POWER_CTRL_RD();

    //printk(">>> Power state changed.  Register: 0x%08x 0x%08x\n", val,
    //power_ctrl );

    // Unplugged: >>> Power state changed.  Register: 0x200301ee 0x00073add
    // Unplugged: >>> Power state changed.  Register: 0x200301ee 0x00473add
    // Unplugged: >>> Power state changed.  Register: 0x200311de
    // Unplugged: >>> Power state changed.  Register: 0x210381ee 0x0007badd
    // Plugged:   >>> Power state changed.  Register: 0x200391ee
    // Plugged:   >>> Power state changed.  Register: 0x200391ee
    // Plugged:   >>> Power state changed.  Register: 0x200381ee
    // Plugged:   >>> Power state changed.  Register: 0x200313fe
    // Plugged:   >>> Power state changed.  Register: 0x200383ee 0x0047bacf
    // Plugged:   >>> Power state changed.  Register: 0x200383ee 0x0047baff
    //
    // Set the 5V persent flag.


    /* Determine which IRQ fired, and then handle it accordingly. */
    if(power_ctrl & BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ
    || power_ctrl & BM_POWER_CTRL_DC_OK_IRQ
    || power_ctrl & BM_POWER_CTRL_VBUSVALID_IRQ) {


        /* Sometime in the near future, have the main power thread prepare
         * for either moving to battery, or moving to AC.
         */
        info->need_to_change_state = NEED_TO_CHANGE_STATE_CYCLES;



        /* We need to set these registers ASAP.  So go ahead and do that.
         * Ideally this would be run in an FIQ out of OCRAM, but that's a
         * problem for another day.
         */
        if(!is_ac_online()) {

            /* Disable the DCDC (on 5V), and disable the 4P2.  These both
             * should be off, but due to a hardware bug they're not.
             */
            HW_POWER_DCDC4P2_WR(HW_POWER_DCDC4P2_RD()&0xff3fffff);

            /* Force the CHARGE and 4P2 off.  There's a hardware bug here
             * somewhere, in that we need to set it twice.  I think.
             */
            HW_POWER_5VCTRL_CLR(BM_POWER_5VCTRL_PWD_CHARGE_4P2);
            udelay(100);
            HW_POWER_5VCTRL_SET(BM_POWER_5VCTRL_PWD_CHARGE_4P2);
            udelay(100);
        }

        /* Only do this when the battery is present.  This IRQ can
         * sometimes get called when the power button is pressed, and if we
         * disable 4P2 when no battery is present, then Very Bad Things can
         * happen.
         */
        else if(is_battery_present()) {
            /* Re-enable the charger and dcdc. */
            HW_POWER_5VCTRL_SET(BM_POWER_5VCTRL_PWD_CHARGE_4P2);
            udelay(100);
            HW_POWER_5VCTRL_CLR(BM_POWER_5VCTRL_PWD_CHARGE_4P2);
            udelay(100);

            /* Re-enable the DCDC and 4P2 bits. */
            HW_POWER_DCDC4P2_WR(HW_POWER_DCDC4P2_RD()|0x00c00000);
        }


        /* Reboot, if we're "asleep" and the cable is either unplugged or
         * plugged in.
         */
        if(POWER_STATE_OFF==power_state)
            do_shutdown();

        HW_POWER_CTRL_CLR(BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ);
        HW_POWER_CTRL_CLR(BM_POWER_CTRL_VBUSVALID_IRQ);
        HW_POWER_CTRL_CLR(BM_POWER_CTRL_DC_OK_IRQ);

    }

    /* Power switch handling.
     * If we're "on", turn blocks off and schedule the freeze task to put
     * the system into a mode where it's charging the battery but appears
     * off.
     */
	else if(HW_POWER_CTRL_RD()&BM_POWER_CTRL_PSWITCH_IRQ) {

        power_switch_pressed();

        if(!th)
            CHLOG(">>> Warning: th is NULL!");
        else
            wake_up_process(th);


        /* Now we're done with the IRQ, clear it. */
        HW_POWER_CTRL_CLR(BM_POWER_CTRL_PSWITCH_IRQ);
    }

    else {
        printk("!!! Unknown power IRQ\n");
    }

	return IRQ_HANDLED;
}





static int stmp3xxx_bat_probe(struct platform_device *pdev)
{
	struct stmp3xxx_info *info;
	int ret = 0;
    int start_voltage = ddi_power_GetBattery();
	int i;


	if (!pdev->dev.platform_data) {
		printk(KERN_ERR "%s: missing platform data\n", __func__);
		return -ENODEV;
	}

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	g_info = info;

	// Disable all brownout detectors while we initialize the power supply
	HW_POWER_MINPWR_SET(0x00001000);

	info->vdd5v_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (info->vdd5v_irq == NULL) {
		printk(KERN_ERR "%s: failed to get irq resouce\n", __func__);
		goto free_info;
	}

	platform_set_drvdata(pdev, info);

	info->dev           = &pdev->dev;
	info->sm_cfg        = pdev->dev.platform_data;
    info->regulator     = NULL;
    info->is_ac_online  = is_ac_online();


    /* Mutexes must be initialized prior to setting up sys calls */
	mutex_init(&info->sm_lock);
	mutex_init(&info->freeze_lock);
	INIT_WORK(&info->sm_work,   state_machine_work);
    INIT_WORK(&info->park_work, do_park_work);


	/* initialize bat power_supply struct */
	info->bat.name           = "battery";
	info->bat.type           = POWER_SUPPLY_TYPE_BATTERY;
	info->bat.properties     = stmp3xxx_bat_props;
	info->bat.num_properties = ARRAY_SIZE(stmp3xxx_bat_props);
	info->bat.get_property   = stmp3xxx_bat_get_property;

	/* initialize ac power_supply struct */
	info->ac.name           = "ac";
	info->ac.type           = POWER_SUPPLY_TYPE_MAINS;
	info->ac.properties     = stmp3xxx_power_props;
	info->ac.num_properties = ARRAY_SIZE(stmp3xxx_power_props);
	info->ac.get_property   = stmp3xxx_power_get_property;

	/* initialize usb power_supply struct */
	info->usb.name           = "usb";
	info->usb.type           = POWER_SUPPLY_TYPE_USB;
	info->usb.properties     = stmp3xxx_power_props;
	info->usb.num_properties = ARRAY_SIZE(stmp3xxx_power_props);
	info->usb.get_property   = stmp3xxx_power_get_property;

	init_timer(&info->sm_timer);
	info->sm_timer.data = (unsigned long)info;
	info->sm_timer.function = state_machine_timer;


	/*
	 * Enable battery temperature reading on HW 10.8 units.
	 * There's a switch to go between measuring the temperature and
	 * measuring the 5V line.
	 */
	HW_PINCTRL_MUXSEL1_SET(0x000000ff);
	HW_PINCTRL_DOUT0_SET(0x00010000);
	HW_PINCTRL_DOE0_SET(0x00010000);


	/* init LRADC channels to measure battery voltage and die temp */
	ddi_power_init_battery();
	HW_POWER_5VCTRL_CLR(BM_POWER_5VCTRL_ENABLE_LINREG_ILIMIT);


	/* Fill in the voltage levels for the averaging code.
     * This needs to be done before the state machine is run!
     */
	for(i=0; i<16; i++)
		info->voltages[i] = start_voltage;
    info->voltage = start_voltage*1000;



	ret = request_irq(info->vdd5v_irq->start,
			stmp3xxx_vdd5v_irq, IRQF_DISABLED,
			pdev->name, info);
	if (ret) {
		dev_err(info->dev, "failed to request irq\n");
		goto stop_sm;
	}



	ret = power_supply_register(&pdev->dev, &info->bat);
	if (ret) {
		dev_err(info->dev, "failed to register battery\n");
		goto free_irq;
	}

	ret = power_supply_register(&pdev->dev, &info->ac);
	if (ret) {
		dev_err(info->dev, "failed to register ac power supply\n");
		goto unregister_bat;
	}

	ret = power_supply_register(&pdev->dev, &info->usb);
	if (ret) {
		dev_err(info->dev, "failed to register usb power supply\n");
		goto unregister_ac;
	}


	/* enable usb device presence detection */
	HW_USBPHY_CTRL_SET(BM_USBPHY_CTRL_ENDEVPLUGINDETECT);


    /* Add our own attributes.  This is mostly for Android support, but it
     * can be useful for other reasons, too.
     */
    //chumby_bat_create_extended_properties(&pdev->dev);


// CHUMBY_local_5v_on
    /* Turn on LOCAL_5V_ON, which powers the USB ports, among other things.
     * LOCAL_5V_ON is located on bank 2, pin 12.
     */
    HW_PINCTRL_MUXSEL0_SET(0x03000000);
    HW_PINCTRL_DOUT0_SET(0x00001000);
    HW_PINCTRL_DOE0_SET(0x00001000);
    HW_PINCTRL_DOUT0_SET(0x00001000);
// !CHUMBY_local_5v_on


    /* Set the power-switch IRQ to fire when the button is pressed. */
    HW_POWER_CTRL_SET(BM_POWER_CTRL_POLARITY_PSWITCH);


    /* Enable power-switch IRQ. */
    HW_POWER_CTRL_CLR(BM_POWER_CTRL_PSWITCH_IRQ);
	HW_POWER_CTRL_SET(BM_POWER_CTRL_ENIRQ_PSWITCH);


    /* Re-enable the brownout detector for the battery, and set it to 2.52V.
	 * This is really super-low, because hopefully we'll be dealing with
	 * undervoltage in the kernel way before that.
     */
	mdelay(10);
    HW_POWER_BATTMONITOR_WR(0x00000603);
    HW_POWER_MINPWR_CLR(0x00001000);


    // Disable DOUBLE_FETS (?).  This'll reduce consumption by a little bit.
    //HW_POWER_MINPWR_CLR(0x00000040);



    if(!th)
        th = kthread_create(power_freeze_thaw_processes, info, "freeze-thaw");

    if (IS_ERR(th)) {
        CHLOG("Unable to start the process-freezing thread\n");
        th = NULL;
    }


	// Run the state machine for the first time.
	ret = bc_sm_restart(info);
	if (ret)
		goto free_info;

    /* Transition to an enabled state. */
    if(is_battery_present())
        ddi_bc_SetEnable();


    /* We have a beefy power supply, so we'll never need the DCDC to
     * source current from the battery.  Turn this ability off.
     */
    HW_POWER_DCDC4P2_WR(HW_POWER_DCDC4P2_RD()&0x0fffffff);


    /* Set the global "halt" command to be our shutdown command. */
    pm_power_off = stmp3xxx_power_down;


	return 0;

unregister_ac:
	power_supply_unregister(&info->ac);
unregister_bat:
	power_supply_unregister(&info->bat);
free_irq:
	free_irq(info->vdd5v_irq->start, pdev);
stop_sm:
	ddi_bc_ShutDown();
free_info:
	kfree(info);
	g_info = NULL;

	// Re-enable brownout detectors
    HW_POWER_MINPWR_CLR(0x00001000);

	return ret;
}



static int stmp3xxx_bat_remove(struct platform_device *pdev)
{
	struct stmp3xxx_info *info = platform_get_drvdata(pdev);

	if (info->regulator)
		regulator_put(info->regulator);
	free_irq(info->vdd5v_irq->start, pdev);
	ddi_bc_ShutDown();
	power_supply_unregister(&info->usb);
	power_supply_unregister(&info->ac);
	power_supply_unregister(&info->bat);
	return 0;
}



static void stmp3xxx_bat_shutdown(struct platform_device *pdev)
{
    CHLOG("Not shutting down battery.  Letting the battery charger run.\n");
    return;
	//ddi_bc_ShutDown();
}


#ifdef CONFIG_PM

static int stmp3xxx_bat_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct stmp3xxx_info *info = platform_get_drvdata(pdev);

	mutex_lock(&info->sm_lock);

	/* disable 5v irq */
	//HW_POWER_CTRL_CLR(BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO);
	HW_POWER_CTRL_CLR(BM_POWER_CTRL_ENIRQ_VBUS_VALID);
	//HW_POWER_CTRL_CLR(BM_POWER_CTRL_ENIRQ_DC_OK);

    /* Also disable power switch IRQ */
	HW_POWER_CTRL_CLR(BM_POWER_CTRL_ENIRQ_PSWITCH);

	ddi_bc_SetDisable();
	/* cancel state machine timer */
	del_timer_sync(&info->sm_timer);

	mutex_unlock(&info->sm_lock);
	return 0;
}

static int stmp3xxx_bat_resume(struct platform_device *pdev)
{
	struct stmp3xxx_info *info = platform_get_drvdata(pdev);
	ddi_bc_Cfg_t *cfg = info->sm_cfg;

	mutex_lock(&info->sm_lock);

	if (is_ac_online()) {
		/* ac supply connected */
		dev_info(info->dev, "ac/5v present, enabling state machine\n");
		info->is_ac_online  = AC_ONLINE;

        CHLOG("Setting current limit to 600 mA\n");
        info->current_limit = 600;
		ddi_bc_SetCurrentLimit(info->current_limit);

        CHLOG("Calling bc_SetEnable\n");
		ddi_bc_SetEnable();
	}
    else {
		/* not powered */
		dev_info(info->dev, "%s: 5v not present\n", __func__);
		info->is_ac_online = 0;
	}
    /*
    ddi_bc_SetEnable();
    bc_sm_restart(info);
    */


    CHLOG("Enabling 5v-removal IRQ\n");

	/* enable 5v irq */
	//HW_POWER_CTRL_SET(BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO);
	HW_POWER_CTRL_SET(BM_POWER_CTRL_ENIRQ_VBUS_VALID);
	//HW_POWER_CTRL_SET(BM_POWER_CTRL_ENIRQ_DC_OK);

	/* reschedule calls to state machine */
	mod_timer(&info->sm_timer,
		  jiffies + msecs_to_jiffies(cfg->u32StateMachinePeriod));

    CHLOG("Returning from call\n");

	mutex_unlock(&info->sm_lock);
	return 0;
}

#else
#define stmp3xxx_bat_suspend NULL
#define stmp3xxx_bat_resume  NULL
#endif

static struct platform_driver stmp3xxx_batdrv = {
	.probe		= stmp3xxx_bat_probe,
	.remove		= stmp3xxx_bat_remove,
	.shutdown       = stmp3xxx_bat_shutdown,
	.suspend	= stmp3xxx_bat_suspend,
	.resume		= stmp3xxx_bat_resume,
	.driver		= {
		.name	= "stmp3xxx-battery",
		.owner	= THIS_MODULE,
	},
};

static int __init stmp3xxx_bat_init(void)
{
	return platform_driver_register(&stmp3xxx_batdrv);
}

static void __exit stmp3xxx_bat_exit(void)
{
	platform_driver_unregister(&stmp3xxx_batdrv);
}

module_init(stmp3xxx_bat_init);
module_exit(stmp3xxx_bat_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Steve Longerbeam <stevel@embeddedalley.com>");
MODULE_DESCRIPTION("Linux glue to STMP3xxx battery state machine");

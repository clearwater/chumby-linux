/*
 *  CPU frequency scaling for Freescale STMP37XX/STMP378X
 *
 *  Author: Vitaly Wool <vital@embeddedalley.com>
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
/* #define DEBUG */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
//#include <linux/regulator/regulator-drv.h>
#include <linux/notifier.h>

#include <mach/hardware.h>
#include <linux/io.h>
#include <asm/system.h>
#include <mach/regulator.h>
#include <mach/power.h>
#include <mach/regs-digctl.h>

#define VERY_HI_RATE		2000000000

static struct profile {
	int cpu;
	int ahb;
	int emi;
	int ss;
	int vddd;
	int vddd_bo;
	int cur;
} profiles[] = {
	{  64000,  64000,  48000, 3, 1000000,  925000, 150000 },
	{ 261820, 130910, 130910, 0, 1225000, 1125000, 173000 },
	{ 360000, 120000, 120000, 0, 1350000, 1250000, 200000 },
	{ 392730, 130910, 130910, 0, 1400000, 1300000, 225000 },
//	{ 454740, 151580, 151580, 0, 1550000, 1450000, 355000 },
	{ 454740, 227370, 130910, 0, 1550000, 1450000, 355000 },
//	{ 454740, 227370, 151580, 0, 1550000, 1450000, 355000 },
};

static struct stmp3xxx_cpufreq {
	struct cpufreq_policy policy;
	struct regulator *regulator;
	struct notifier_block nb;
	struct notifier_block init_nb;
	int freq_id;
	int next_freq_id;
	spinlock_t lock;
} cpufreq_bdata;

static int reg_callback(struct notifier_block *, unsigned long, void *);
static int init_reg_callback(struct notifier_block *, unsigned long, void *);

static inline void __set_new_policy(struct cpufreq_policy *policy)
{
	spin_lock(&cpufreq_bdata.lock);

	if (policy)
		cpufreq_bdata.policy = *policy;
	else
		memzero(&cpufreq_bdata.policy, sizeof(cpufreq_bdata.policy));

	if (cpufreq_bdata.regulator)
		goto out;

	cpufreq_bdata.regulator = regulator_get(NULL, "cpufreq-1");
	if (!cpufreq_bdata.regulator || IS_ERR(cpufreq_bdata.regulator))
		cpufreq_bdata.regulator = NULL;
	else {
		regulator_set_mode(cpufreq_bdata.regulator,
				   REGULATOR_MODE_FAST);
		if (cpufreq_bdata.regulator)
			regulator_register_notifier(
					cpufreq_bdata.regulator,
					&cpufreq_bdata.nb);
	}

out:
	spin_unlock(&cpufreq_bdata.lock);
}

static int stmp3xxx_verify_speed(struct cpufreq_policy *policy)
{
	struct clk *cpu_clk;

	pr_debug("%s: entered, policy %p\n", __func__, policy);

	__set_new_policy(policy);

	if (policy->cpu)
		return -EINVAL;

	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);
	cpu_clk = clk_get(NULL, "cpu");
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);

	pr_debug("%s: policy->min %d, policy->max %d\n",
		__func__, policy->min, policy->max);
	policy->min = clk_round_rate(cpu_clk, policy->min);
	policy->max = clk_round_rate(cpu_clk, policy->max);
	pr_debug("%s: after rounding rate: policy->min %d, policy->max %d\n",
		__func__, policy->min, policy->max);
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);
	clk_put(cpu_clk);

	return 0;
}

static unsigned int stmp3xxx_getspeed(unsigned int cpu)
{
	struct clk *cpu_clk;
	unsigned long rate;

	pr_debug("%s: entered\n", __func__);
	if (cpu)
		return 0;

	cpu_clk = clk_get(NULL, "cpu");
	if (IS_ERR(cpu_clk))
		return 0;
	rate = clk_get_rate(cpu_clk);
	pr_debug("%s: got cpu speed %ld\n", __func__, rate);
	clk_put(cpu_clk);

	return rate;
}

static int set_op(unsigned int target_freq)
{
	struct clk *cpu_clk, *ahb_clk, *emi_clk;
	struct regulator *vddd, *vdddbo, *cur_limit;
	struct cpufreq_freqs freqs;
	int ret = 0, i;
    static int is_first_run = 1;

	cur_limit = cpufreq_bdata.regulator;
	pr_debug("%s: entered\n", __func__);
	cpu_clk = clk_get(NULL, "cpu");
	if (IS_ERR(cpu_clk)) {
		ret = PTR_ERR(cpu_clk);
		goto out_cpu;
	}
	ahb_clk = clk_get(NULL, "hclk");
	if (IS_ERR(ahb_clk)) {
		ret = PTR_ERR(ahb_clk);
		goto out_ahb;
	}
	emi_clk = clk_get(NULL, "emi");
	if (IS_ERR(emi_clk)) {
		ret = PTR_ERR(emi_clk);
		goto out_emi;
	}

	vddd = regulator_get(NULL, "vddd");
	vdddbo = regulator_get(NULL, "vddd_bo");
	if (IS_ERR(vdddbo))
		vdddbo = NULL;

	freqs.old = clk_get_rate(cpu_clk);
	freqs.cpu = 0;
	for (i = 0; i < ARRAY_SIZE(profiles); i++) {
		// If the target frequency is bigger than the CPU frequency at the
		// current slot, but less than the frequency at the next slot, pick
		// the current frequency to use.
		if (profiles[i].cpu <= target_freq 
			&& (i+1 >= ARRAY_SIZE(profiles)
			|| profiles[i + 1].cpu > target_freq)) {
			freqs.new = profiles[i].cpu;
			cpufreq_bdata.next_freq_id = i;
			break;
		}
		if (!vddd && profiles[i].cpu > freqs.old) {
			/* can't safely set more than now */
			freqs.new = profiles[i - 1].cpu;
			break;
		}
	}

	pr_debug("target_freq %d, new %d\n", target_freq, freqs.new);
/*
	if (i == ARRAY_SIZE(profiles) - 1) {
		freqs.new = profiles[i].cpu;
		cpufreq_bdata.next_freq_id = i;
	}
*/

	if (IS_ERR(vddd)) {
		ret = PTR_ERR(vddd);
		if (!cpufreq_bdata.init_nb.notifier_call) {
			/* we only register once */
			cpufreq_bdata.init_nb.notifier_call = init_reg_callback;
			bus_register_notifier(&platform_bus_type,
					      &cpufreq_bdata.init_nb);
		}
		goto out_vddd;
	}

	pr_debug("i %d: freqs.old %d, freqs.new %d\n",
		i, freqs.old, freqs.new);

	spin_lock(&cpufreq_bdata.lock);
	if (cur_limit && (freqs.old < freqs.new)) {
		ret = regulator_set_current_limit(cur_limit, profiles[i].cur, profiles[i].cur);
		if (ret)
			goto out_cur;
	}

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	if (freqs.old > freqs.new || is_first_run) {
		int ss = profiles[i].ss;
        is_first_run = 0;
		clk_set_rate(cpu_clk, profiles[i].cpu);
		clk_set_rate(ahb_clk, profiles[i].ahb);
		clk_set_rate(emi_clk, profiles[i].emi);
		HW_DIGCTL_ARMCACHE_WR(BF_DIGCTL_ARMCACHE_VALID_SS(ss) |
				      BF_DIGCTL_ARMCACHE_DRTY_SS(ss) |
				      BF_DIGCTL_ARMCACHE_CACHE_SS(ss) |
				      BF_DIGCTL_ARMCACHE_DTAG_SS(ss) |
				      BF_DIGCTL_ARMCACHE_ITAG_SS(ss));
		if (vddd && vdddbo) {
			ret = regulator_set_voltage(vddd, profiles[i].vddd, profiles[i].vddd);
			if (ret)
				ret = regulator_set_voltage(vddd,
							    profiles[i].vddd,
							    profiles[i].vddd);
			regulator_set_voltage(vdddbo, profiles[i].vddd_bo, profiles[i].vddd_bo);
		}
	} else if(freqs.old < freqs.new) {
		int ss = profiles[i].ss;
		if (vddd && vdddbo) {
			ret = regulator_set_voltage(vddd, profiles[i].vddd, profiles[i].vddd);
			if (ret)
				ret = regulator_set_voltage(vddd,
							    profiles[i].vddd,
							    profiles[i].vddd);
			regulator_set_voltage(vdddbo, profiles[i].vddd_bo, profiles[i].vddd_bo);
		}
		HW_DIGCTL_ARMCACHE_WR(BF_DIGCTL_ARMCACHE_VALID_SS(ss) |
				      BF_DIGCTL_ARMCACHE_DRTY_SS(ss) |
				      BF_DIGCTL_ARMCACHE_CACHE_SS(ss) |
				      BF_DIGCTL_ARMCACHE_DTAG_SS(ss) |
				      BF_DIGCTL_ARMCACHE_ITAG_SS(ss));
		clk_set_rate(cpu_clk, profiles[i].cpu);
		clk_set_rate(ahb_clk, profiles[i].ahb);
		clk_set_rate(emi_clk, profiles[i].emi);
	}
	udelay(100);

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	if (cur_limit && (freqs.old > freqs.new))	/* will not fail */
		regulator_set_current_limit(cur_limit, profiles[i].cur, profiles[i].cur);

	cpufreq_bdata.freq_id = i;

out_cur:
	spin_unlock(&cpufreq_bdata.lock);
	if (vddd)
		regulator_put(vddd);
out_vddd:
	clk_put(emi_clk);
out_emi:
	clk_put(ahb_clk);
out_ahb:
	clk_put(cpu_clk);
out_cpu:

	return ret;
}

static int stmp3xxx_target(struct cpufreq_policy *policy,
			  unsigned int target_freq,
			  unsigned int relation)
{
	return set_op(target_freq);
}

static int reg_callback(struct notifier_block *self, unsigned long event,
			void *data)
{
	struct stmp3xxx_cpufreq *_temp =
		container_of(self, struct stmp3xxx_cpufreq, nb);
	struct cpufreq_policy *policy = &_temp->policy;
	int max_prof = ARRAY_SIZE(profiles) - 1;
	int ret = -EINVAL;

	pr_debug("%s: entered, _temp %p, policy %p, cpu %d, freq_id %d\n",
		 __func__, _temp, policy, policy->cpu, _temp->freq_id);

	if (policy)
		policy = cpufreq_cpu_get(policy->cpu);
	if (!policy) {
		printk(KERN_ERR "%s: couldn't get cpufreq policy\n", __func__);
		goto out;
	}

	/* FIXME: Need a lock: set policy by user VS async USB event */
	switch (event) {
	case STMP3XXX_REG5V_IS_USB:
		pr_debug("%s: limiting max_freq to %d\n", __func__,
			 profiles[max_prof - 1].cpu);
		policy->user_policy.min = profiles[0].cpu;
		policy->user_policy.max = profiles[max_prof - 1].cpu;
		if (_temp->freq_id > max_prof - 1)
			set_op(profiles[max_prof - 1].cpu);
		break;

	case STMP3XXX_REG5V_NOT_USB:
		pr_debug("%s: undo limiting max_freq to %d\n", __func__,
			 profiles[max_prof - 1].cpu);
		policy->user_policy.min = profiles[0].cpu;
		policy->user_policy.max = profiles[max_prof].cpu;
		break;

	default:
		pr_info("%s: unknown event %ld\n", __func__, event);
		break;
	}
	cpufreq_cpu_put(policy);
	ret = cpufreq_update_policy(policy->cpu);
out:
	return ret;
}

static int init_reg_callback(struct notifier_block *self,
			     unsigned long event, void *data)
{
	int ret;
	struct stmp3xxx_cpufreq *_temp =
		container_of(self, struct stmp3xxx_cpufreq, init_nb);

	ret = set_op(profiles[_temp->next_freq_id].cpu);
	if (ret == 0)
		bus_unregister_notifier(&platform_bus_type,
				&cpufreq_bdata.init_nb);
	return ret;
}

static int __init stmp3xxx_cpu_init(struct cpufreq_policy *policy)
{
	struct clk *cpu_clk = clk_get(NULL, "cpu");

	pr_debug("%s: entered\n", __func__);
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);

	if (policy->cpu != 0)
		return -EINVAL;

	policy->cur = policy->min = policy->max = clk_get_rate(cpu_clk);

	pr_debug("got CPU clock rate %d\n", policy->cur);
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;
	policy->cpuinfo.min_freq = profiles[0].cpu;
	policy->cpuinfo.max_freq = profiles[ARRAY_SIZE(profiles) - 1].cpu;
	policy->cpuinfo.transition_latency = 1000000; /* 1 ms, assumed */
	clk_put(cpu_clk);

	return 0;
}

static struct cpufreq_driver stmp3xxx_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= stmp3xxx_verify_speed,
	.target		= stmp3xxx_target,
	.get		= stmp3xxx_getspeed,
	.init		= stmp3xxx_cpu_init,
	.name		= "stmp3xxx",
};

static int __init stmp3xxx_cpufreq_init(void)
{
	spin_lock_init(&cpufreq_bdata.lock);
	cpufreq_bdata.nb.notifier_call = reg_callback;
	return cpufreq_register_driver(&stmp3xxx_driver);
}

static int __init stmp3xxx_reg_init(void)
{
	pr_debug("%s: enter\n", __func__);
	if (!cpufreq_bdata.regulator)
		__set_new_policy(&cpufreq_bdata.policy);

	if (cpufreq_bdata.regulator)
		regulator_set_current_limit(cpufreq_bdata.regulator,
					    profiles[cpufreq_bdata.freq_id].cur,
					    profiles[cpufreq_bdata.freq_id].cur);
	return 0 ;
}

arch_initcall(stmp3xxx_cpufreq_init);
late_initcall(stmp3xxx_reg_init);

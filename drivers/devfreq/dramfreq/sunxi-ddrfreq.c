/*
 * drivers/devfreq/dramfreq/sunxi-ddrfreq.c
 *
 * Copyright(c) 2013-2015 Allwinnertech Co., Ltd.
 *
 * Author: Pan Nan <pannan@allwinnertech.com>
 *
 * SUNXI ddr frequency dynamic scaling driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <linux/clk.h>
#include <linux/clk/sunxi_name.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#if defined(CONFIG_ARCH_SUN8IW7P1) && defined(CONFIG_CPU_BUDGET_THERMAL)
#include <linux/cpu_budget_cooling.h>
#endif
#include <mach/sys_config.h>
#include <mach/platform.h>
#include "sunxi-mdfs.h"

#if defined(CONFIG_ARCH_SUN9IW1P1) || defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW7P1)
extern char __sram_start, __sram_end, __sram_text_start, __sram_data_end;
#endif

enum {
	DEBUG_FREQ = 1U << 0,
	DEBUG_SUSPEND = 1U << 1,
};
static int debug_mask = DEBUG_FREQ | DEBUG_SUSPEND;
module_param(debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);
#define DDRFREQ_DBG(mask,format,args...) \
	do { if (mask & debug_mask) printk("[ddrfreq] "format,##args); } while (0)
#define DDRFREQ_ERR(format,args...) \
	printk(KERN_ERR "[ddrfreq] ERR:"format,##args)

#define MDFS_RETRIES            (10)

static DEFINE_MUTEX(ddrfreq_lock);
static unsigned int ddrfreq_enable = 0;
static __dram_para_t dram_para;
static struct devfreq *this_df = NULL;
static unsigned long long setfreq_time_usecs = 0;
static unsigned long long getfreq_time_usecs = 0;
unsigned int sunxi_ddrfreq_max = 0;
unsigned int sunxi_ddrfreq_min = 0;

#ifdef CONFIG_SMP
static struct cpumask ipi_mask;
#endif

#if defined(CONFIG_ARCH_SUN9IW1P1)
static struct clk *clk_pll4;
static struct clk *clk_pll6;
#elif defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW8P1)
static struct clk *clk_pll_ddr0;
static struct clk *clk_pll_ddr1;
#elif defined(CONFIG_ARCH_SUN8IW6P1)
#define SUNXI_DDRFREQ_MAXFREQ_MIN		(600000)
#define SUNXI_DDRFREQ_MINFREQ_MIN		(168000)
static struct clk *clk_pll_ddr0;
static unsigned int ddrfreq_odt_disable = 0;
#elif defined(CONFIG_ARCH_SUN8IW7P1)
static struct clk *clk_pll_ddr0;
#endif

#ifdef CONFIG_ARCH_SUN9IW1P1
static int dram_freq_table[][2] = {
	{ 672 * 1000, 1 },
	{ 480 * 1000, 0 },
	{ 336 * 1000, 2 },
	{ 240 * 1000, 0 },
	{ 168 * 1000, 4 },
	{          0, 0 },
};

static unsigned int dram_freq_adjust = 0;
#endif

#if defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW8P1)
unsigned int mdfs_in_cfs = 0;
#endif

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_LOW_POWER_SW
/*
 * 0: normal; 1:enter low-power music; 2:exit low-power music
 */
static unsigned int ddrfreq_system_state = 0;
#endif

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_IN_VSYNC
#define NEXT_VB_TIME_RETRIES    (2)
#define NEXT_VB_TIME_WAIT_US    (2000)

struct ddrfreq_vb_time_ops {
	int (*get_vb_time) (void);
	int (*get_next_vb_time) (void);
	int (*is_in_vb) (void);
};

struct ddrfreq_vb_time_ops vbtime_ops;

int ddrfreq_set_vb_time_ops(struct ddrfreq_vb_time_ops *ops)
{
	vbtime_ops.get_vb_time = ops->get_vb_time;
	vbtime_ops.get_next_vb_time = ops->get_next_vb_time;
	vbtime_ops.is_in_vb = ops->is_in_vb;

	return 0;
}
EXPORT_SYMBOL(ddrfreq_set_vb_time_ops);
#endif

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ
static DEFINE_MUTEX(busfreq_lock);

struct busfreq_table {
	char *name;
	struct clk *bus_clk;
	unsigned long normal_freq;
	unsigned long idle_freq;
};

#if defined(CONFIG_ARCH_SUN9IW1P1)
static struct busfreq_table busfreq_tbl[] = {
	{ .name = "gtbus",  .normal_freq = 400000000, .idle_freq = 300000000 },
	{ .name = "cci400", .normal_freq = 480000000, .idle_freq = 240000000 },
	{ .name = "ahb0",   .normal_freq = 120000000, .idle_freq =  75000000 },
	{ .name = "ahb1",   .normal_freq = 240000000, .idle_freq = 120000000 },
	{ .name = "ahb2",   .normal_freq = 120000000, .idle_freq =  75000000 },
};

static struct clk *gtbus;
static struct clk *cci400;
static struct clk *ahb0;
static struct clk *ahb1;
static struct clk *ahb2;

#elif defined(CONFIG_ARCH_SUN8IW5P1)
static struct busfreq_table busfreq_tbl[] = {
	{ .name = "ahb1",   .normal_freq = 200000000, .idle_freq =  50000000 },
};

static struct clk *ahb1;

#elif defined(CONFIG_ARCH_SUN8IW6P1)
static struct busfreq_table busfreq_tbl[] = {
	{ .name = "cci400", .normal_freq = 480000000, .idle_freq = 240000000 },
	{ .name = "ahb1",   .normal_freq = 200000000, .idle_freq = 100000000 },
};

static struct clk *cci400;
static struct clk *ahb1;
#endif

#endif /* CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ */

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_VDDSYS
#define TABLE_LENGTH (8)
static unsigned int table_length_syscfg = 0;
static unsigned int last_vdd = 0;
static struct regulator *vdd_sys = NULL;
struct ddrfreq_dvfs {
	unsigned int freq;   /* ddr frequency, based on KHz */
	unsigned int volt;   /* voltage for the frequency, based on mv */
};
static struct ddrfreq_dvfs dvfs_table_syscfg[TABLE_LENGTH];
#endif

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ
static int __set_bus_freq(const char *name, struct clk *clk,
					unsigned long target_freq)
{
	unsigned long cur_freq;

	mutex_lock(&busfreq_lock);

	if (clk_prepare_enable(clk)) {
		DDRFREQ_ERR("try to enable %s output failed!\n", name);
		goto err;
	}

	cur_freq = clk_get_rate(clk);
	if (cur_freq == target_freq) {
		mutex_unlock(&busfreq_lock);
		return 0;
	}

#if defined(CONFIG_ARCH_SUN9IW1P1)
	if ((!strcmp(name, "ahb0")) || (!strcmp(name, "ahb2"))) {
		if (target_freq == 75000000) {
			if (clk_set_parent(clk, gtbus)) {
				DDRFREQ_ERR("try to set %s parent failed!\n", name);
				goto err;
			}
		} else if (target_freq == 120000000) {
			if (clk_set_parent(clk, clk_pll4)) {
				DDRFREQ_ERR("try to set %s parent failed!\n", name);
				goto err;
			}
		}

		if (clk_set_rate(clk, target_freq)) {
			DDRFREQ_ERR("try to set %s rate to %lu failed!\n", name, target_freq);
			goto err;
		}
	} else {
		if (clk_set_rate(clk, target_freq)) {
			DDRFREQ_ERR("try to set %s rate to %lu failed!\n", name, target_freq);
			goto err;
		}
	}

#elif defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1)
	if (clk_set_rate(clk, target_freq)) {
		DDRFREQ_ERR("try to set %s rate to %lu failed!\n", name, target_freq);
		goto err;
	}
#endif

	cur_freq = clk_get_rate(clk);
	if (cur_freq != target_freq) {
		DDRFREQ_ERR("%s: %lu != %lu\n", name, cur_freq, target_freq);
		goto err;
	}

	mutex_unlock(&busfreq_lock);
	return 0;

err:
	mutex_unlock(&busfreq_lock);
	return -1;
}
#endif /* CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ */

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_VDDSYS
static int __init_vftable_syscfg(void)
{
	int i, ret = 0;
	char name[16] = {0};
	script_item_u val;
	script_item_value_type_e type;

	type = script_get_item("dram_dvfs_table", "LV_count", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch LV_count from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("LV_count value is %d\n", val.val);
	table_length_syscfg = val.val;

	/* table_length_syscfg must be < TABLE_LENGTH */
	if(table_length_syscfg > TABLE_LENGTH){
		DDRFREQ_ERR("LV_count from sysconfig is out of bounder\n");
		ret = -1;
		goto fail;
	}

	for (i = 1; i <= table_length_syscfg; i++){
		sprintf(name, "LV%d_freq", i);
		type = script_get_item("dram_dvfs_table", name, &val);
		if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
			DDRFREQ_ERR("get LV%d_freq from sysconfig failed\n", i);
			return -ENODEV;
		}
		dvfs_table_syscfg[i-1].freq = val.val / 1000;

		sprintf(name, "LV%d_volt", i);
		type = script_get_item("dram_dvfs_table", name, &val);
		if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
			DDRFREQ_ERR("get LV%d_volt from sysconfig failed\n", i);
			return -ENODEV;
		}
		dvfs_table_syscfg[i-1].volt = val.val;
	}

fail:
	return ret;
}

static void __vftable_show(void)
{
	int i;

	pr_debug("---------------Dram V-F Table---------------\n");
	for(i = 0; i < table_length_syscfg; i++){
		pr_debug("voltage = %4dmv \tfrequency = %4dKHz\n",
			dvfs_table_syscfg[i].volt, dvfs_table_syscfg[i].freq);
	}
	pr_debug("--------------------------------------------\n");
}

static unsigned int __get_vddsys_value(unsigned int freq)
{
	struct ddrfreq_dvfs *dvfs_inf = NULL;
	dvfs_inf = &dvfs_table_syscfg[0];

	while((dvfs_inf+1)->freq >= freq)
		dvfs_inf++;

	return dvfs_inf->volt;
}
#endif /* CONFIG_DEVFREQ_DRAM_FREQ_VDDSYS */

#ifdef CONFIG_ARCH_SUN9IW1P1
static unsigned long __ddrfreq_get(void)
{
	unsigned int pll4_rate, pll6_rate, dram_clk_rate, dram_freq = 0;
	int value;

	value = (readl(SUNXI_CCM_MOD_VBASE + 0x84) >> 12) & 0xF;
	if (value == 0x3) {
		pll6_rate = clk_get_rate(clk_pll6) / 1000;
		// DDRFREQ_DBG(DEBUG_FREQ, "pll6_rate: %d\n", pll6_rate);

		value = readl(SUNXI_CCM_MOD_VBASE + 0x84) >> 8;
		value &= 0xF;
		dram_clk_rate = pll6_rate / (value + 1);
		// DDRFREQ_DBG(DEBUG_FREQ, "dram_clk_rate: %d\n", dram_clk_rate);

		if ((readl(SUNXI_DRAMPHY0_VBASE + 0x04) >> 17) & 0x1)    //pll bypass mode
			dram_freq = dram_clk_rate >> 1;
		else                                                     //pll normal mode
			dram_freq = dram_clk_rate << 1;
		// DDRFREQ_DBG(DEBUG_FREQ, "dram_freq: %d\n", dram_freq);

	} else if (value == 0x0) {
		pll4_rate = clk_get_rate(clk_pll4) / 1000;
		// DDRFREQ_DBG(DEBUG_FREQ, "pll4_rate: %d\n", pll4_rate);

		value = readl(SUNXI_CCM_MOD_VBASE + 0x84) >> 8;
		value &= 0xF;
		dram_clk_rate = pll4_rate / (value + 1);
		// DDRFREQ_DBG(DEBUG_FREQ, "dram_clk_rate: %d\n", dram_clk_rate);

		dram_freq = dram_clk_rate >> 1;
		// DDRFREQ_DBG(DEBUG_FREQ, "dram_freq: %d\n", dram_freq);
	}

	return dram_freq;
}

#elif defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW8P1)
static unsigned long __ddrfreq_get(void)
{
	unsigned long pll_ddr_rate, dram_freq = 0;
	unsigned int dram_div_m;

	if ((readl(SUNXI_CCM_VBASE + 0xF8) >> 16) & 0x1)
		pll_ddr_rate = clk_get_rate(clk_pll_ddr1) / 1000;
	else
		pll_ddr_rate = clk_get_rate(clk_pll_ddr0) / 1000;

	if (mdfs_in_cfs == 1) {
		if (readl(SUNXI_CCM_VBASE + 0xF4) & 0xF)  //pll normal mode
			dram_freq = pll_ddr_rate;
		else                                      //pll bypass mode
			dram_freq = pll_ddr_rate / 4;
	} else if (mdfs_in_cfs == 0) {
		dram_div_m = (readl(SUNXI_CCM_VBASE + 0xF4) & 0xF) + 1;
#ifdef CONFIG_DEVFREQ_DRAM_FREQ_LOW_POWER_SW
		if (ddrfreq_system_state == 1) {
			dram_freq = pll_ddr_rate / 4 / dram_div_m;
		} else {
			dram_freq = pll_ddr_rate * 2 / dram_div_m;
		}
#else
		dram_freq = pll_ddr_rate * 2 / dram_div_m;
#endif

		if ((dram_freq != sunxi_ddrfreq_max) && (dram_freq != sunxi_ddrfreq_min))
			dram_freq = dram_freq >= 360000 ? 360000 : 240000;
	}

	return dram_freq;
}

#elif defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW7P1)
static unsigned long __ddrfreq_get(void)
{
	unsigned long pll_ddr_rate, dram_freq = 0;
	unsigned int dram_div_m;

	pll_ddr_rate = clk_get_rate(clk_pll_ddr0) / 1000;
	dram_div_m = (readl(CCM_DRAM_CFG_REG) & 0xF) + 1;
	dram_freq = pll_ddr_rate / 2 / dram_div_m;

	return dram_freq;
}
#endif

/**
 * dramfreq_get - get the current DRAM frequency (in KHz)
 *
 */
unsigned long dramfreq_get(void)
{
	unsigned long freq;
	ktime_t calltime, delta, rettime;

	calltime = ktime_get();
	freq = __ddrfreq_get();
	rettime = ktime_get();
	delta = ktime_sub(rettime, calltime);
	getfreq_time_usecs = ktime_to_ns(delta) >> 10;

	return freq;
}
EXPORT_SYMBOL_GPL(dramfreq_get);

#ifdef CONFIG_ARCH_SUN9IW1P1
static unsigned int __get_pll6_para(unsigned int pll_clk)
{
	unsigned int n = 0, div1 = 0, div2 = 0, rval = 0;

	if ((pll_clk % 24) == 0) {
		n = pll_clk / 24;
	} else if ((pll_clk % 12) == 0) {
		n = pll_clk / 12;
		div1 = 1;
	} else if((pll_clk % 6) == 0) {
		n = pll_clk / 6;
		div1 = 1;
		div2 = 1;
	}

	rval = (1U << 31) | (div2 << 18) | (div1 << 16) | (n << 8);

	return rval;
}

static void __update_target_freq(unsigned long *freq, unsigned int *div)
{
	int i = 0;

	if (dram_freq_adjust) {
		if ((*freq == 480000) || (*freq == 240000))
			*freq += 1;
	}

	while (dram_freq_table[i][0] != 0) {
		if (dram_freq_table[i+1][0] >= *freq)
			i++;
		else
			break;
	}

	*freq = dram_freq_table[i][0];
	*div  = dram_freq_table[i][1];
}

#elif defined(CONFIG_ARCH_SUN8IW5P1)
static unsigned int __get_pll_ddr_para_in_cfs(unsigned int pll_clk)
{
	u32 rval = 0;
	u32 div;

	div = pll_clk / 24;
	if (div < 12) //no less than 288M
		div=12;

	rval |= (((div - 1) << 8) | (0x1U << 31));

	return rval;
}

static unsigned int __get_pll_div_para(unsigned int pll_clk)
{
	u32 pll_div, present_clk, pll_source;

	pll_source = (readl(_CCM_PLL_DDR_CFG_REG) >> 16) & 0x1;
	if (pll_source)//pll_ddr1
		present_clk = clk_get_rate(clk_pll_ddr1) / 1000000;
	else
		present_clk = clk_get_rate(clk_pll_ddr0) / 1000000;

	pll_div = present_clk * 2 / pll_clk;
	if (pll_div >= 16)
		pll_div = 15;

	return pll_div;
}

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_LOW_POWER_SW
static unsigned int __get_pll_ddr0_para(unsigned int pll_clk)
{
	unsigned int n, k, m = 1, rval, div;

	div = pll_clk / 24;
	m = 2;
	k = 2;
	n = div;


	rval = readl(_CCM_PLL_DDR0_REG);
	rval &= ~((0x1f << 8) | (0x3 << 4) | (0x3 << 0));
	rval = (1U << 31) | ((n - 1) << 8) | ((k - 1) << 4) | (m - 1);

	return rval;
}

static unsigned int __get_pll_ddr1_para(unsigned int pll_clk)
{
	u32 rval, div;

	div = pll_clk / 24;
	if (div < 12) //no less than 288M
		div=12;

	rval = readl(_CCM_PLL_DDR1_REG);
	rval &= ~(0x3f << 8);
	rval |= (((div - 1) << 8) | (0x1U << 31));

	return rval;
}

static unsigned int __get_pll_ddrx_para(unsigned int pll_clk)
{
	u32 para, pll_source;

	pll_source = (readl(_CCM_PLL_DDR_CFG_REG) >> 16) & 0x1;
	if (pll_source)//pll_ddr1
		para = __get_pll_ddr1_para(pll_clk << 1);
	else
		para = __get_pll_ddr0_para(pll_clk);

	return para;
}
#endif /* CONFIG_DEVFREQ_DRAM_FREQ_LOW_POWER_SW */

#elif defined(CONFIG_ARCH_SUN8IW6P1)
static unsigned int __get_pll_ddr_para_in_cfs(unsigned int pll_clk)
{
	unsigned int n = pll_clk / 24;
	unsigned int div1 = 0, div2 = 0, rval = 0;

	rval = (1U << 31) | (div2 << 18) | (div1 << 16) | ((n - 1) << 8);

	return rval;
}

static unsigned int __get_pll_div_para(unsigned int pll_clk)
{
	u32 pll_div, present_clk;

	present_clk = clk_get_rate(clk_pll_ddr0) / 1000000;
	pll_div = present_clk / 2 / pll_clk;

	if (pll_div >= 16)
		pll_div = 15;

	return pll_div;
}

#elif defined(CONFIG_ARCH_SUN8IW7P1)
static unsigned int __get_pll_ddr_para(unsigned int pll_clk)
{
	unsigned int n, k = 1, m = 1, rval;
	unsigned int div;

	div = pll_clk / 24;
	if (div <= 32) {
		n = div;
		k = 1;
	} else {
		if (div <= 64) {
			k = 2;
		} else {
			if (div%3 == 0) {
				k = 3;
			} else if(div%4 == 0) {
				k = 4;
			} else if(div%5 == 0) {
				k = 5;
			}
		}
		n = div / k;
	}

	rval = readl(_CCM_PLL_DDR_REG);
	rval &= ~((0x1f << 8) | (0x3 << 4) | (0x3 << 0));
	rval = (1U << 31) | ((n - 1) << 8) | ((k - 1) << 4) | (m-1);

	return rval;
}

#elif defined(CONFIG_ARCH_SUN8IW8P1)
static unsigned int __get_pll_ddr_para_in_cfs(unsigned int pll_clk)
{
	return 0;
}

static unsigned int __get_pll_div_para(unsigned int pll_clk)
{
	return 0;
}
#endif

#ifdef CONFIG_SMP
volatile bool __sramdata cpu_pause[NR_CPUS];
volatile bool __sramdata pause_flag;
bool __sram is_paused(void)
{
	smp_rmb();
	return pause_flag;
}
void __sram set_paused(bool pause)
{
	pause_flag = pause;
	smp_wmb();
}
bool __sram is_cpuX_paused(unsigned int cpu)
{
	smp_rmb();
	return cpu_pause[cpu];
}
void __sram set_cpuX_paused(unsigned int cpu, bool pause)
{
	cpu_pause[cpu] = pause;
	smp_wmb();
}
void __sram mdfs_pause_cpu(void *info)
{
	unsigned int cpu = raw_smp_processor_id();
	dsb();
	isb();
	set_cpuX_paused(cpu, true);
	while (is_paused());
	set_cpuX_paused(cpu, false);
}
static void mdfs_wait_cpu(void *info)
{
}
#endif /* CONFIG_SMP */

#if defined(CONFIG_ARCH_SUN8IW5P1)
static int mdfs_cfs(unsigned int freq_jump, __dram_para_t *para,
				unsigned int pll_ddr_para)
{
	unsigned int reg_val;

	//bit0 must be 0 for new MDFS process
	while(readl(MC_MDFSCR) & 0x1);

	//setting ODT configuration,enable before adjust
	if (para->dram_clk > 400) {
		writel(0x00000201, ODTMAP);

		if (para->dram_odt_en) {
			reg_val = readl(DXnGCR0(0));
			reg_val |= (0x3 << 9);
			writel(reg_val, DXnGCR0(0));

			reg_val = readl(DXnGCR0(1));
			reg_val |= (0x3 << 9);
			writel(reg_val, DXnGCR0(1));
		}
	}

	//set pll ddr configuration
	reg_val = readl(_CCM_PLL_DDR_CFG_REG);
	reg_val |= ((0x1 << 12) | (0x9 << 0));   //continuously
	writel(reg_val, _CCM_PLL_DDR_CFG_REG);
	//set PLL-DDR1 without update
	writel(pll_ddr_para, _CCM_PLL_DDR1_REG);

	//setting MDFS configuration
	reg_val = readl(MC_MDFSCR);
	reg_val &= ~(0x1 << 2);
	reg_val |= ((freq_jump & 0x1) << 2);    //1: increase  0:decrease
	reg_val |= (0x1 << 1);   //CFS mode
	writel(reg_val, MC_MDFSCR);

	//start mdfs
	writel(((reg_val) | 0x1), MC_MDFSCR);

	//wait for process finished, bit0 must be 0 for new MDFS process
	while(readl(MC_MDFSCR) & 0x1);

	//setting ODT configuration, disable after adjust
	if (para->dram_clk < 400) {
		writel(0x0, ODTMAP);

		reg_val = readl(DXnGCR0(0));
		reg_val &= ~(0x3 << 9);
		writel(reg_val, DXnGCR0(0));

		reg_val = readl(DXnGCR0(1));
		reg_val &= ~(0x3 << 9);
		writel(reg_val, DXnGCR0(1));
	}

	return 0;
}

static int mdfs_dfs(unsigned int freq_jump, __dram_para_t *para,
				unsigned int pll_div)
{
	unsigned int reg_val, rank_num, odt_freq;
	unsigned int trefi = 0;
	unsigned int trfc = 0;

	//bit0 must be 0 for new MDFS process
	while(readl(MC_MDFSCR) & 0x1);

	//set new refresh timing
	trefi = (39 * para->dram_clk) / 320 ;
	trfc = (7 * para->dram_clk) / 40 ;
	//masked master ready
	writel(0xffffffff, MC_MDFSMRMR);

	//set pll lock time
	writel(0x4e200960, PTR1);

	//set ODT register buffer for power save
	reg_val = readl(MC_MDFSCR);
	reg_val |=(0x3U << 14);
	writel(reg_val, MC_MDFSCR);

	reg_val = 0;
	reg_val = (trefi << 16) | (trfc << 0);
	writel(reg_val, RFSHTMG);

	rank_num = readl(MC_WORK_MODE) & 0x1;
	odt_freq = rank_num == 1 ? 360 : 400;
	//according to frequency set register buffer value
	if (para->dram_clk >= odt_freq) {
		//turn on DRAMC odt
		reg_val = readl(DXnGCR0(0));
		reg_val |= (0x3 << 9);
		writel(reg_val, DXnGCR0(0));

		reg_val = readl(DXnGCR0(1));
		reg_val |= (0x3 << 9);
		writel(reg_val, DXnGCR0(1));

		//turn on DRAM ODT
		if (rank_num) {
			writel(0x00000303, ODTMAP);
		} else {
			writel(0x00000201, ODTMAP);
		}
	} else {
		 //turn off DRAMC ODT
		reg_val = readl(DXnGCR0(0));
		reg_val &= ~(0x3 << 9);
		writel(reg_val, DXnGCR0(0));

		reg_val = readl(DXnGCR0(1));
		reg_val &= ~(0x3 << 9);
		writel(reg_val, DXnGCR0(1));

		//turn off DRAM ODT
		writel(0x0, ODTMAP);
	}

	//set the DRAM_CFG_REG divider in CCMU
	reg_val = readl(CCM_DRAM_CFG_REG);
	reg_val &= ~(0xf << 0);
	reg_val |= ((pll_div - 1) << 0);
	writel(reg_val, CCM_DRAM_CFG_REG);

	//set MDFS register
	reg_val = readl(MC_MDFSCR);
	reg_val |= (0x1U << 13);  //enable pad hold in the process
	reg_val |= ( (freq_jump & 0x1) << 2 );    //increase or decrease
	reg_val |= (0x0U << 1);   //DFS mode
	reg_val |= (0x1U << 0);   //start mdfs
	writel(reg_val, MC_MDFSCR);

	//wait for process finished
	while(readl(MC_MDFSCR) & 0x1);  //bit0 must be 0 for new MDFS process

	//close ODT register buffer
	reg_val = readl(MC_MDFSCR);
	reg_val &=~(0x3U << 14);
	writel(reg_val, MC_MDFSCR);

	return 0;
}

#elif defined(CONFIG_ARCH_SUN8IW6P1)
static int mdfs_cfs(unsigned int freq_jump, __dram_para_t *para,
				unsigned int pll_ddr_para)
{
	unsigned int reg_val;
	unsigned int trefi;
	unsigned int trfc;
	unsigned int ctrl_freq;

	//bit0 must be 0 for new MDFS process
	while(readl(MC_MDFSCR) & 0x1);

	//calculate new timing
	ctrl_freq = para->dram_clk / 2;
	trefi = (3900 * ctrl_freq) / 1000 / 32;
	trfc = (210 * ctrl_freq) / 1000;
	//enable timing double buffer
	reg_val = readl(MC_MDFSCR);
	reg_val |= (0x1U << 15);
	writel(reg_val, MC_MDFSCR);
	//set new timing into buffer
	reg_val = 0;
	reg_val = (trefi << 16) | (trfc << 0);
	writel(reg_val, RFSHTMG);
	//set pll-pattern control register
	reg_val = readl(_CCM_PLL_DDR_PATTERN_REG);
	reg_val |=(0x7U << 29);
	writel(reg_val, _CCM_PLL_DDR_PATTERN_REG);

	//set pll ddr configuration
	reg_val = readl(_CCM_PLL_DDR_CFG_REG);
	reg_val |= ((0x1 << 12) | (0x9 << 0));   //continuously
	writel(reg_val, _CCM_PLL_DDR_CFG_REG);
	//set PLL-DDR1 without update
	writel(pll_ddr_para, _CCM_PLL_DDR_REG);

	//setting MDFS configuration
	reg_val = readl(MC_MDFSCR);
	reg_val &= ~(0x1 << 2);
	reg_val |= ((freq_jump & 0x1) << 2);    //1: increase  0:decrease
	reg_val |= (0x1 << 1);   //CFS mode
	writel(reg_val, MC_MDFSCR);

	//start mdfs
	writel(((reg_val) | 0x1), MC_MDFSCR);

	//wait for process finished, bit0 must be 0 for new MDFS process
	while(readl(MC_MDFSCR) & 0x1);

	//set pll-pattern control register
	reg_val = readl(_CCM_PLL_DDR_PATTERN_REG);
	reg_val &= ~(0x7U << 29);
	writel(reg_val, _CCM_PLL_DDR_PATTERN_REG);

	//disable timing double buffer
	reg_val = readl(MC_MDFSCR);
	reg_val &= ~(0x1U << 15);
	writel(reg_val, MC_MDFSCR);

	return 0;
}

unsigned int mdfs_dfs(unsigned int freq_jump, __dram_para_t *para,
				unsigned int pll_div)
{
	unsigned int reg_val, rank_num;
	unsigned int i = 0;
	unsigned int odt_type = 0;

	//bit0 must be 0 for new MDFS process
	while(readl(MC_MDFSCR) & 0x1);

	//masked master ready
	writel(0xffffffff, MC_MDFSMRMR);

	//set ODT register buffer for power save
	reg_val = readl(MC_MDFSCR);
	reg_val |=(0x3U << 14);
	writel(reg_val, MC_MDFSCR);

	//according to frequency set register buffer value
	if (!((para->dram_tpr13 >> 12) & 0x1)) {
		if (para->dram_clk > 400) {
			//turn on DRAMC odt
			if (para->dram_odt_en & 0x1) {
				odt_type = (para->dram_odt_en >> 1) & 0x1;
				for (i = 0; i < 11; i++) {
					//byte 0
					reg_val = readl(DATX0IOCR(i));
					reg_val &= ~(0x3U<<24);
					reg_val |=(odt_type<<24);
					writel(reg_val, DATX0IOCR(i));
					//byte 1
					reg_val = readl(DATX1IOCR(i));
					reg_val &= ~(0x3U<<24);
					reg_val |=(odt_type<<24);
					writel(reg_val, DATX1IOCR(i));
					//byte 2
					reg_val = readl(DATX2IOCR(i));
					reg_val &= ~(0x3U<<24);
					reg_val |=(odt_type<<24);
					writel(reg_val, DATX2IOCR(i));
					//byte 3
					reg_val = readl(DATX3IOCR(i));
					reg_val &= ~(0x3U<<24);
					reg_val |=(odt_type<<24);
					writel(reg_val, DATX3IOCR(i));
				}
			}
			//turn on DRAM ODT
			rank_num = readl(MC_WORK_MODE) & 0x1;
			if (rank_num) {
				writel(0x00000303, ODTMAP);
			} else {
				writel(0x00000201, ODTMAP);
			}
		} else {
			if (para->dram_odt_en & 0x1) {
				//for dqs/dqs#,odt always on
				for (i = 0; i < 11; i++) {
					//byte 0
					reg_val = readl(DATX0IOCR(i));
					reg_val &= ~(0x3U<<24);
					reg_val |= (0x2U<<24);
					writel(reg_val, DATX0IOCR(i));
					//byte 1
					reg_val = readl(DATX1IOCR(i));
					reg_val &= ~(0x3U<<24);
					reg_val |= (0x2U<<24);
					writel(reg_val, DATX1IOCR(i));
					//byte 2
					reg_val = readl(DATX2IOCR(i));
					reg_val &= ~(0x3U<<24);
					reg_val |= (0x2U<<24);
					writel(reg_val, DATX2IOCR(i));
					//byte 3
					reg_val = readl(DATX3IOCR(i));
					reg_val &= ~(0x3U<<24);
					reg_val |= (0x2U<<24);
					writel(reg_val, DATX3IOCR(i));
				}
			}
			//turn off DRAM ODT
			writel(0x0, ODTMAP);
		}
	}

	//set the DRAM_CFG_REG divider in CCMU
	reg_val = readl(CCM_DRAM_CFG_REG);
	reg_val &= ~(0xf << 0);
	reg_val |= ((pll_div - 1) << 0);
	writel(reg_val, CCM_DRAM_CFG_REG);

	//set MDFS register
	reg_val = readl(MC_MDFSCR);
	reg_val |= (0x1U << 13);  //enable pad hold in the process
	reg_val |= (0x1<<4);
	reg_val |= ( (freq_jump & 0x1) << 2 );    //increase or decrease
	reg_val |= (0x0U << 1);   //DFS mode
	reg_val |= (0x1U << 0);   //start mdfs
	writel(reg_val, MC_MDFSCR);

	//wait for process finished
	while(readl(MC_MDFSCR) & 0x1);  //bit0 must be 0 for new MDFS process

	//close ODT register buffer
	reg_val = readl(MC_MDFSCR);
	reg_val &=~(0x3U << 14);
	writel(reg_val, MC_MDFSCR);

	return 0;
}

#elif defined(CONFIG_ARCH_SUN8IW8P1)
static int mdfs_cfs(unsigned int freq_jump, __dram_para_t *para,
				unsigned int pll_ddr_para)
{
	return 0;
}

static int mdfs_dfs(unsigned int freq_jump, __dram_para_t *para,
				unsigned int pll_div)
{
	return 0;
}
#endif

/**
 *  freq_target: target frequency
 *  df: devfreq
 */
#ifdef CONFIG_ARCH_SUN9IW1P1
static int __ddrfreq_set(unsigned int jump, struct devfreq *df,
					unsigned int freq_target, unsigned int div)
#else
static int __ddrfreq_set(unsigned int jump, struct devfreq *df,
					unsigned int freq_target)
#endif
{
	unsigned int pll_para_to_mdfs = 0;
	int ret = 0;
	ktime_t calltime, delta, rettime;
#ifdef CONFIG_CPU_FREQ
	struct cpufreq_policy policy;
#endif

#ifdef CONFIG_SMP
	u32 timeout = 0;
	unsigned int cpu, this_cpu = raw_smp_processor_id();

	cpumask_clear(&ipi_mask);
	cpumask_copy(&ipi_mask, cpu_online_mask);
	cpumask_clear_cpu(this_cpu, &ipi_mask);

	DDRFREQ_DBG(DEBUG_FREQ, "current cpu is cpu%u\n", this_cpu);
#endif

#if defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW8P1)
	if (mdfs_in_cfs == 0) {
#endif

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_IN_VSYNC
		if (!vbtime_ops.is_in_vb) {
			DDRFREQ_ERR("is_in_vb is not initialized\n");
			return -1;
		}
#endif

#if defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW8P1)
	}
#endif

#ifdef CONFIG_CPU_FREQ
	if (cpufreq_get_policy(&policy, 0)) {
		DDRFREQ_ERR("can not get cpu policy\n");
		return -1;
	}
#endif

	dram_para.dram_clk = freq_target / 1000;

#ifdef CONFIG_ARCH_SUN9IW1P1
	if (dram_para.dram_clk <= 800)
		pll_para_to_mdfs = __get_pll6_para(dram_para.dram_clk << 1);
	else
		pll_para_to_mdfs = __get_pll6_para(dram_para.dram_clk >> 1);

#elif defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW8P1)
	if (mdfs_in_cfs == 1) {
		DDRFREQ_DBG(DEBUG_FREQ, "mdfs in cfs mode\n");
		pll_para_to_mdfs = __get_pll_ddr_para_in_cfs(dram_para.dram_clk << 1);
	} else if (mdfs_in_cfs == 0) {
		DDRFREQ_DBG(DEBUG_FREQ, "mdfs in dfs mode\n");
#ifdef CONFIG_DEVFREQ_DRAM_FREQ_LOW_POWER_SW
		if (ddrfreq_system_state != 0)
			pll_para_to_mdfs = __get_pll_ddrx_para(dram_para.dram_clk);
		else
			pll_para_to_mdfs = __get_pll_div_para(dram_para.dram_clk);
#else
		pll_para_to_mdfs = __get_pll_div_para(dram_para.dram_clk);
#endif
	}

#elif defined(CONFIG_ARCH_SUN8IW7P1)
	pll_para_to_mdfs = __get_pll_ddr_para(dram_para.dram_clk << 1);
#endif

#if defined(CONFIG_ARCH_SUN9IW1P1) || defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW7P1)
	memcpy(&__sram_text_start, &__sram_start, (&__sram_end - &__sram_start));
#endif

#ifdef CONFIG_CPU_FREQ
	if (policy.cur < policy.max) {
		__cpufreq_driver_target(&policy, policy.max, CPUFREQ_RELATION_H);
	}
#endif

#if defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW8P1)
	if (mdfs_in_cfs == 0) {
#endif

#if defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW8P1)
	}
#endif

#ifdef CONFIG_SMP
	local_bh_disable();
	set_paused(true);

	preempt_disable();
	smp_call_function_many(&ipi_mask, (smp_call_func_t)mdfs_pause_cpu, NULL, 0);
	preempt_enable();

	dsb();
	isb();

	for_each_online_cpu(cpu) {
		if (cpu == this_cpu)
			continue;
		while (!is_cpuX_paused(cpu) && timeout < 100) {
			udelay(100);
			timeout++;
		}

		if (timeout >= 100) {
			DDRFREQ_ERR("pause cpu%d timeout\n", cpu);
			ret = -2;
			goto out;
		}
	}
#endif

	local_irq_disable();
#ifdef CONFIG_DEVFREQ_DRAM_FREQ_IN_VSYNC
	while(!vbtime_ops.is_in_vb());
#endif
	calltime = ktime_get();
	flush_tlb_all();
	dmb();
	dsb();
	isb();

#if defined(CONFIG_ARCH_SUN9IW1P1)
	mdfs_main(jump, &dram_para, pll_para_to_mdfs, div);

#elif defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW8P1)
	if (mdfs_in_cfs == 1) {
#ifdef CONFIG_DEVFREQ_DRAM_FREQ_CFS_SW
		if (!ddrfreq_odt_disable) {
			if (dram_para.dram_clk > 500) {
				mdfs_main(jump, &dram_para, 0, 0);
			}
		}
#endif

		mdfs_cfs(jump, &dram_para, pll_para_to_mdfs);

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_CFS_SW
		if (!ddrfreq_odt_disable) {
			if (dram_para.dram_clk < 500) {
				mdfs_main(jump, &dram_para, 0, 0);
			}
		}
#endif
	} else if (mdfs_in_cfs == 0) {
#ifdef CONFIG_DEVFREQ_DRAM_FREQ_LOW_POWER_SW
		if (ddrfreq_system_state == 0) {
			mdfs_dfs(jump, &dram_para, pll_para_to_mdfs);
		} else {
			mdfs_main(jump, &dram_para, pll_para_to_mdfs, 0);
		}
#else
		mdfs_dfs(jump, &dram_para, pll_para_to_mdfs);
#endif
	}

#elif defined(CONFIG_ARCH_SUN8IW7P1)
	mdfs_main(jump, &dram_para, pll_para_to_mdfs, 0);
#endif

	rettime = ktime_get();
	delta = ktime_sub(rettime, calltime);
	setfreq_time_usecs = ktime_to_ns(delta) >> 10;
	local_irq_enable();

	DDRFREQ_DBG(DEBUG_FREQ, "elapsed: %lluus\n", setfreq_time_usecs);

#ifdef CONFIG_SMP
out:
	set_paused(false);
	local_bh_enable();
	preempt_disable();
	smp_call_function_many(&ipi_mask, mdfs_wait_cpu, NULL, true);
	preempt_enable();
#endif

	return ret;
}

static int ddrfreq_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct devfreq *df = platform_get_drvdata(pdev);
	int ret = 0, mdfs_retries = 0;
	int jump = 0;
	int cur_freq = 0;
#ifdef CONFIG_ARCH_SUN9IW1P1
	unsigned int div = 0;
#endif
#ifdef CONFIG_DEVFREQ_DRAM_FREQ_IN_VSYNC
	int next_vb_time_us = 0, next_vb_retries = 0;
#endif
#ifdef CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ
	int i;
#endif
#ifdef CONFIG_DEVFREQ_DRAM_FREQ_VDDSYS
	unsigned int new_vdd;
#endif

	mutex_lock(&ddrfreq_lock);
	get_online_cpus();

	if (!ddrfreq_enable) {
		goto unlock;
	}

	if (df == NULL) {
		ret = -1;
		goto unlock;
	}

#ifdef CONFIG_ARCH_SUN9IW1P1
	__update_target_freq(freq, &div);
#endif

	cur_freq = __ddrfreq_get();
	if (df->previous_freq != cur_freq)
		df->previous_freq = cur_freq;

	if (*freq == df->previous_freq) {
		ret = -1;
		goto unlock;
	}

	jump = (*freq > df->previous_freq) ? 1 : 0;

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_LOW_POWER_SW
	if ((df->previous_freq == df->max_freq) && (*freq == df->min_freq)) {
		ddrfreq_system_state = 1;
	} else if ((df->previous_freq == df->min_freq) && (*freq == df->max_freq)) {
		ddrfreq_system_state = 2;
	} else {
		ddrfreq_system_state = 0;
	}
#endif

	DDRFREQ_DBG(DEBUG_FREQ, "DDR: %luKHz->%luKHz start\n", df->previous_freq, *freq);

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_VDDSYS
#if defined(CONFIG_ARCH_SUN9IW1P1)
	vdd_sys = regulator_get(NULL, "axp22_dcdc4");
#elif defined(CONFIG_ARCH_SUN8IW5P1)
	vdd_sys = regulator_get(NULL, "axp22_dcdc2");
#endif

	if (IS_ERR(vdd_sys)) {
		DDRFREQ_ERR("some error happen, fail to get regulator!");
		vdd_sys = NULL;
	}

	new_vdd = __get_vddsys_value(*freq);
	if ((df->previous_freq == df->min_freq) && (*freq == df->max_freq)) {
		if (vdd_sys && (new_vdd > last_vdd)) {
			ret = regulator_set_voltage(vdd_sys, new_vdd * 1000, new_vdd * 1000);
			if(ret != 0) {
				DDRFREQ_ERR("fail to set regulator voltage!\n");
				regulator_put(vdd_sys);
				goto unlock;
			}
			DDRFREQ_DBG(DEBUG_FREQ, "VDD_SYS: %dmv->%dmv ok!\n", last_vdd, new_vdd);
		}
	}
#endif

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ
	if ((df->previous_freq == df->min_freq) && (*freq == df->max_freq)) {
		for (i = ARRAY_SIZE(busfreq_tbl) - 1; i >= 0; i--) {
			__set_bus_freq(busfreq_tbl[i].name,
					busfreq_tbl[i].bus_clk, busfreq_tbl[i].normal_freq);
		}

#if defined(CONFIG_ARCH_SUN9IW1P1)
		DDRFREQ_DBG(DEBUG_FREQ, "GTBUS:%lu, CCI400:%lu, AHB0:%lu, AHB1:%lu, "
				"AHB2:%lu\n", clk_get_rate(gtbus) / 1000000,
				clk_get_rate(cci400) / 1000000, clk_get_rate(ahb0) / 1000000,
				clk_get_rate(ahb1)   / 1000000, clk_get_rate(ahb2) / 1000000);
#elif defined(CONFIG_ARCH_SUN8IW5P1)
		DDRFREQ_DBG(DEBUG_FREQ, "AHB1:%lu\n", clk_get_rate(ahb1) / 1000000);
#elif defined(CONFIG_ARCH_SUN8IW6P1)
		DDRFREQ_DBG(DEBUG_FREQ, "CCI400:%lu, AHB1:%lu\n",
				clk_get_rate(cci400) / 1000000, clk_get_rate(ahb1) / 1000000);
#endif

	}
#endif

#if defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW8P1)
	if (mdfs_in_cfs == 0) {
#endif

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_IN_VSYNC
		if (!vbtime_ops.get_next_vb_time) {
			DDRFREQ_ERR("get_next_vb_time is not initialized\n");
			ret = -1;
			goto unlock;
		}

		do {
			next_vb_time_us = vbtime_ops.get_next_vb_time();
			if (next_vb_time_us < NEXT_VB_TIME_WAIT_US) {
				next_vb_retries++;
				DDRFREQ_DBG(DEBUG_FREQ, "next_vb_time_us: %dus\n", next_vb_time_us);
			} else {
				break;
			}
		} while (next_vb_retries < NEXT_VB_TIME_RETRIES);

		if (next_vb_retries >= NEXT_VB_TIME_RETRIES) {
			DDRFREQ_ERR("next vb retrying failed, next time\n");
			ret = -1;
			goto unlock;
		} else {
			usleep_range(next_vb_time_us - NEXT_VB_TIME_WAIT_US,
							next_vb_time_us - NEXT_VB_TIME_WAIT_US);
		}
#endif /* CONFIG_DEVFREQ_DRAM_FREQ_IN_VSYNC */

#if defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW8P1)
	}
#endif

	do {
#ifdef CONFIG_ARCH_SUN9IW1P1
		ret = __ddrfreq_set(jump, df, *freq, div);
#else
		ret = __ddrfreq_set(jump, df, *freq);
#endif
		if (ret == -1) {
			goto unlock;
		} else if (ret == -2) {
			mdfs_retries++;
		} else if (ret == 0) {
			break;
		}
	} while (mdfs_retries < MDFS_RETRIES);

	if (mdfs_retries >= MDFS_RETRIES) {
		DDRFREQ_ERR("mdfs retrying failed, next time\n");
		ret = -1;
		goto unlock;
	}

	DDRFREQ_DBG(DEBUG_FREQ, "DDR: %luKHz->%luKHz ok!\n",
					df->previous_freq, __ddrfreq_get());

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ
	if ((df->previous_freq == df->max_freq) && (*freq == df->min_freq)) {
		for (i = 0; i < ARRAY_SIZE(busfreq_tbl); i++) {
			__set_bus_freq(busfreq_tbl[i].name,
					busfreq_tbl[i].bus_clk, busfreq_tbl[i].idle_freq);
		}

#if defined(CONFIG_ARCH_SUN9IW1P1)
		DDRFREQ_DBG(DEBUG_FREQ, "GTBUS:%lu, CCI400:%lu, AHB0:%lu, AHB1:%lu, "
				"AHB2:%lu\n", clk_get_rate(gtbus) / 1000000,
				clk_get_rate(cci400) / 1000000, clk_get_rate(ahb0) / 1000000,
				clk_get_rate(ahb1)   / 1000000, clk_get_rate(ahb2) / 1000000);
#elif defined(CONFIG_ARCH_SUN8IW5P1)
		DDRFREQ_DBG(DEBUG_FREQ, "AHB1:%lu\n", clk_get_rate(ahb1) / 1000000);
#elif defined(CONFIG_ARCH_SUN8IW6P1)
		DDRFREQ_DBG(DEBUG_FREQ, "CCI400:%lu, AHB1:%lu\n",
				clk_get_rate(cci400) / 1000000, clk_get_rate(ahb1) / 1000000);
#endif

	}
#endif

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_VDDSYS
	if ((df->previous_freq == df->max_freq) && (*freq == df->min_freq)) {
		if (vdd_sys && (new_vdd < last_vdd)) {
			ret = regulator_set_voltage(vdd_sys, new_vdd * 1000, new_vdd * 1000);
			if (ret != 0) {
				//without care, ddr freq scaling is ok
				ret = 0;
				DDRFREQ_ERR("fail to set regulator voltage!\n");
				regulator_put(vdd_sys);
				goto unlock;
			}
			DDRFREQ_DBG(DEBUG_FREQ, "VDD_SYS: %dmv->%dmv ok!\n", last_vdd, new_vdd);
		}
	}
	last_vdd = new_vdd;

	if (vdd_sys) {
		regulator_put(vdd_sys);
		vdd_sys = NULL;
	}
#endif

unlock:
	put_online_cpus();
	mutex_unlock(&ddrfreq_lock);

	return ret;
}

static int ddrfreq_get_dev_status(struct device *dev,
						struct devfreq_dev_status *stat)
{
	return 0;
}

static struct devfreq_dev_profile ddrfreq_profile = {
	.target         = ddrfreq_target,
	.get_dev_status = ddrfreq_get_dev_status,
};

#ifdef CONFIG_ARCH_SUN8IW5P1
#if defined(CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ) && defined(CONFIG_CPU_FREQ)
#if defined(CONFIG_DEVFREQ_GOV_DSM) && defined(CONFIG_EARLYSUSPEND)
extern atomic_t ddrfreq_dsm_suspend_lock;

static int ddrfreq_cpu_freq_transition(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct cpufreq_freqs *freqs = (struct cpufreq_freqs *)data;

	switch (val) {
	case CPUFREQ_POSTCHANGE:
		if (!atomic_read(&ddrfreq_dsm_suspend_lock)) {
			if (freqs->new < 100000) {
				if (__set_bus_freq(busfreq_tbl[0].name, busfreq_tbl[0].bus_clk,
							100000000)) {
					DDRFREQ_ERR("set %s to 100MHz failed!\n", busfreq_tbl[0].name);
				}
			} else {
				if (__set_bus_freq(busfreq_tbl[0].name, busfreq_tbl[0].bus_clk,
							200000000)) {
					DDRFREQ_ERR("set %s to 200MHz failed!\n", busfreq_tbl[0].name);
				}
			}
		}

		break;
	}

	return 0;
}

static struct notifier_block ddrfreq_cpu_freq_transition_notifier = {
	.notifier_call = ddrfreq_cpu_freq_transition,
};
#endif
#endif
#endif /* CONFIG_ARCH_SUN8IW5P1 */

#if defined(CONFIG_ARCH_SUN8IW7P1) && defined(CONFIG_CPU_BUDGET_THERMAL)
struct platform_device sunxi_ddrfreq_device;
extern int ths_read_data(int value);

static int ddrfreq_budget_cooling_notifier_call(struct notifier_block *nfb, unsigned long mode, void *cmd)
{
	int temperature;

	temperature = ths_read_data(4);
	if (temperature >= 100) {
		DDRFREQ_DBG(DEBUG_FREQ, "temperature=%d C, ddr freq down\n", temperature);
		ddrfreq_target(&sunxi_ddrfreq_device.dev, &this_df->min_freq, 0);
		goto out;
	} else if (temperature < 90) {
		DDRFREQ_DBG(DEBUG_FREQ, "temperature=%d C, ddr freq up\n", temperature);
		ddrfreq_target(&sunxi_ddrfreq_device.dev, &this_df->max_freq, 0);
		goto out;
	} else {
		return NOTIFY_DONE;
	}

out:
	return NOTIFY_OK;
}

static struct notifier_block ddrfreq_budget_cooling_notifier = {
	.notifier_call = ddrfreq_budget_cooling_notifier_call,
};
#endif

static __devinit int sunxi_ddrfreq_probe(struct platform_device *pdev)
{
	int err = 0;
	script_item_u val;
	script_item_value_type_e type;

	type = script_get_item("dram_para", "dram_clk", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_clk from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_clk value is %u\n", val.val);
	dram_para.dram_clk = val.val;
	sunxi_ddrfreq_max = dram_para.dram_clk * 1000;
#if defined(CONFIG_ARCH_SUN8IW6P1)
	if (sunxi_ddrfreq_max < SUNXI_DDRFREQ_MAXFREQ_MIN) {
		printk("[ddrfreq] warning: dram clk is too low to support\n");
		return -ENODEV;
	}
#endif
	pr_debug("sunxi_ddrfreq_max=%u\n", sunxi_ddrfreq_max);

	type = script_get_item("dram_para", "dram_type", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_type from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_type value is %u\n", val.val);
	dram_para.dram_type = val.val;

	type = script_get_item("dram_para", "dram_zq", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_zq from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_zq value is 0x%x\n", val.val);
	dram_para.dram_zq = val.val;

	type = script_get_item("dram_para", "dram_odt_en", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_odt_en from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_odt_en value is %u\n", val.val);
	dram_para.dram_odt_en = val.val;

	type = script_get_item("dram_para", "dram_para1", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_para1 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_para1 value is 0x%x\n", val.val);
	dram_para.dram_para1 = val.val;

	type = script_get_item("dram_para", "dram_para2", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_para2 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_para2 value is 0x%x\n", val.val);
	dram_para.dram_para2 = val.val;

	type = script_get_item("dram_para", "dram_mr0", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_mr0 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_mr0 value is 0x%x\n", val.val);
	dram_para.dram_mr0 = val.val;

	type = script_get_item("dram_para", "dram_mr1", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_mr1 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_mr1 value is 0x%x\n", val.val);
	dram_para.dram_mr1 = val.val;

	type = script_get_item("dram_para", "dram_mr2", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_mr2 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_mr2 value is 0x%x\n", val.val);
	dram_para.dram_mr2 = val.val;

	type = script_get_item("dram_para", "dram_mr3", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_mr3 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_mr3 value is 0x%x\n", val.val);
	dram_para.dram_mr3 = val.val;

	type = script_get_item("dram_para", "dram_tpr0", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_tpr0 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_tpr0 value is 0x%x\n", val.val);
	dram_para.dram_tpr0 = val.val;

	type = script_get_item("dram_para", "dram_tpr1", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_tpr1 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_tpr1 value is 0x%x\n", val.val);
	dram_para.dram_tpr1 = val.val;

	type = script_get_item("dram_para", "dram_tpr2", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_tpr2 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_tpr2 value is 0x%x\n", val.val);
	dram_para.dram_tpr2 = val.val;

	type = script_get_item("dram_para", "dram_tpr3", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_tpr3 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_tpr3 value is 0x%x\n", val.val);
	dram_para.dram_tpr3 = val.val;

	type = script_get_item("dram_para", "dram_tpr4", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_tpr4 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_tpr4 value is 0x%x\n", val.val);
	dram_para.dram_tpr4 = val.val;

	type = script_get_item("dram_para", "dram_tpr5", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_tpr5 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_tpr5 value is 0x%x\n", val.val);
	dram_para.dram_tpr5 = val.val;

	type = script_get_item("dram_para", "dram_tpr6", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_tpr6 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_tpr6 value is 0x%x\n", val.val);
	dram_para.dram_tpr6 = val.val;

	type = script_get_item("dram_para", "dram_tpr7", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_tpr7 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_tpr7 value is 0x%x\n", val.val);
	dram_para.dram_tpr7 = val.val;

	type = script_get_item("dram_para", "dram_tpr8", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_tpr8 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_tpr8 value is 0x%x\n", val.val);
	dram_para.dram_tpr8 = val.val;

	type = script_get_item("dram_para", "dram_tpr9", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_tpr9 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_tpr9 value is 0x%x\n", val.val);
	dram_para.dram_tpr9 = val.val;

	type = script_get_item("dram_para", "dram_tpr10", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_tpr10 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_tpr10 value is 0x%x\n", val.val);
	dram_para.dram_tpr10 = val.val;

	type = script_get_item("dram_para", "dram_tpr11", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_tpr11 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_tpr11 value is 0x%x\n", val.val);
	dram_para.dram_tpr11 = val.val;

#if defined(CONFIG_ARCH_SUN8IW6P1)
	sunxi_ddrfreq_min = sunxi_ddrfreq_max / 4;
	if (sunxi_ddrfreq_min < SUNXI_DDRFREQ_MINFREQ_MIN)
		sunxi_ddrfreq_min = sunxi_ddrfreq_max / 3;
#elif defined(CONFIG_ARCH_SUN8IW7P1)
	sunxi_ddrfreq_min = 408000;
#else
	type = script_get_item("dram_para", "dram_tpr12", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_tpr12 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_tpr12 value is %u\n", val.val);
	dram_para.dram_tpr12 = val.val;
	sunxi_ddrfreq_min = dram_para.dram_tpr12 * 1000;
#endif
	pr_debug("sunxi_ddrfreq_min=%u\n", sunxi_ddrfreq_min);

	type = script_get_item("dram_para", "dram_tpr13", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		DDRFREQ_ERR("fetch dram_tpr13 from sysconfig failed\n");
		return -ENODEV;
	}
	pr_debug("dram_tpr13 value is 0x%x\n", val.val);
	dram_para.dram_tpr13 = val.val;
	ddrfreq_enable = (dram_para.dram_tpr13 >> 11) & 0x1;
	if (!ddrfreq_enable)
		printk("[ddrfreq] warning: disabled!\n");

#ifdef CONFIG_ARCH_SUN8IW6P1
	ddrfreq_odt_disable = (dram_para.dram_tpr13 >> 12) & 0x1;
#endif

#if defined(CONFIG_ARCH_SUN9IW1P1)
	dram_freq_adjust = (dram_para.dram_tpr13 >> 18) & 0x1F;
#elif defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW8P1)
	mdfs_in_cfs = (dram_para.dram_tpr13 >> 10) & 0x1;
#endif

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ
#if defined(CONFIG_ARCH_SUN9IW1P1)
	ahb2 = clk_get(NULL, AHB2_CLK);
	if (!ahb2 || IS_ERR(ahb2)) {
		DDRFREQ_ERR("try to get AHB2 failed!\n");
		err = -ENOENT;
		goto err_ahb2;
	}

	ahb1 = clk_get(NULL, AHB1_CLK);
	if (!ahb1 || IS_ERR(ahb1)) {
		DDRFREQ_ERR("try to get AHB1 failed!\n");
		err = -ENOENT;
		goto err_ahb1;
	}

	ahb0 = clk_get(NULL, AHB0_CLK);
	if (!ahb0 || IS_ERR(ahb0)) {
		DDRFREQ_ERR("try to get AHB0 failed!\n");
		err = -ENOENT;
		goto err_ahb0;
	}

	cci400 = clk_get(NULL, CCI400_CLK);
	if (!cci400 || IS_ERR(cci400)) {
		DDRFREQ_ERR("try to get CCI400 failed!\n");
		err = -ENOENT;
		goto err_cci400;
	}

	gtbus = clk_get(NULL, GT_CLK);
	if (!gtbus || IS_ERR(gtbus)) {
		DDRFREQ_ERR("try to get GTBUS failed!\n");
		err = -ENOENT;
		goto err_gtbus;
	}

#elif defined(CONFIG_ARCH_SUN8IW5P1)
	ahb1 = clk_get(NULL, AHB1_CLK);
	if (!ahb1 || IS_ERR(ahb1)) {
		DDRFREQ_ERR("try to get AHB1 failed!\n");
		err = -ENOENT;
		goto err_ahb1;
	}

#elif defined(CONFIG_ARCH_SUN8IW6P1)
	ahb1 = clk_get(NULL, AHB1_CLK);
	if (!ahb1 || IS_ERR(ahb1)) {
		DDRFREQ_ERR("try to get AHB1 failed!\n");
		err = -ENOENT;
		goto err_ahb1;
	}

	cci400 = clk_get(NULL, CCI400_CLK);
	if (!cci400 || IS_ERR(cci400)) {
		DDRFREQ_ERR("try to get CCI400 failed!\n");
		err = -ENOENT;
		goto err_cci400;
	}
#endif

#endif /* CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ */

#ifdef CONFIG_ARCH_SUN9IW1P1
	clk_pll4 = clk_get(NULL, PLL4_CLK);
	if (!clk_pll4 || IS_ERR(clk_pll4)) {
		DDRFREQ_ERR("try to get PLL4 failed!\n");
		err = -ENOENT;
		goto err_pll4;
	}

	clk_pll6 = clk_get(NULL, PLL6_CLK);
	if (!clk_pll6 || IS_ERR(clk_pll6)) {
		DDRFREQ_ERR("try to get PLL6 failed!\n");
		err = -ENOENT;
		goto err_pll6;
	}

#elif defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW8P1)
	clk_pll_ddr0 = clk_get(NULL, PLL_DDR0_CLK);
	if (!clk_pll_ddr0 || IS_ERR(clk_pll_ddr0)) {
		DDRFREQ_ERR("try to get clk_pll_ddr0 failed!\n");
		err = -ENOENT;
		goto err_pll_ddr0;
	}

	clk_pll_ddr1 = clk_get(NULL, PLL_DDR1_CLK);
	if (!clk_pll_ddr1 || IS_ERR(clk_pll_ddr1)) {
		DDRFREQ_ERR("try to get clk_pll_ddr1 failed!\n");
		err = -ENOENT;
		goto err_pll_ddr1;
	}

#elif defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW7P1)
	clk_pll_ddr0 = clk_get(NULL, PLL_DDR_CLK);
	if (!clk_pll_ddr0 || IS_ERR(clk_pll_ddr0)) {
		DDRFREQ_ERR("try to get clk_pll_ddr0 failed!\n");
		err = -ENOENT;
		goto err_pll_ddr0;
	}
#endif

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ
#if defined(CONFIG_ARCH_SUN9IW1P1)
	busfreq_tbl[0].bus_clk = gtbus;
	busfreq_tbl[1].bus_clk = cci400;
	busfreq_tbl[2].bus_clk = ahb0;
	busfreq_tbl[3].bus_clk = ahb1;
	busfreq_tbl[4].bus_clk = ahb2;
#elif defined(CONFIG_ARCH_SUN8IW5P1)
	busfreq_tbl[0].bus_clk = ahb1;
#elif defined(CONFIG_ARCH_SUN8IW6P1)
	busfreq_tbl[0].bus_clk = cci400;
	busfreq_tbl[1].bus_clk = ahb1;
#endif
#endif

#ifdef CONFIG_ARCH_SUN8IW6P1
	if (mdfs_in_cfs == 1) {
		sunxi_ddrfreq_min = 168000;
	}
#endif

	ddrfreq_profile.initial_freq = __ddrfreq_get();
#ifdef CONFIG_ARCH_SUN8IW7P1
	this_df = devfreq_add_device(&pdev->dev, &ddrfreq_profile, &devfreq_userspace, NULL);
#else
	this_df = devfreq_add_device(&pdev->dev, &ddrfreq_profile, &devfreq_dsm, NULL);
#endif
	if (IS_ERR(this_df)) {
		DDRFREQ_ERR("add devfreq device failed!\n");
		err = PTR_ERR(this_df);
		goto err_devfreq;
	}

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_VDDSYS
	err = __init_vftable_syscfg();
	if(err) {
		DDRFREQ_ERR("init V-F Table failed\n");
		goto err_vftable;
	}

	__vftable_show();

#if defined(CONFIG_ARCH_SUN9IW1P1)
	vdd_sys = regulator_get(NULL, "axp22_dcdc4");
#elif defined(CONFIG_ARCH_SUN8IW5P1)
	vdd_sys = regulator_get(NULL, "axp22_dcdc2");
#endif

	if (IS_ERR(vdd_sys)) {
		DDRFREQ_ERR("some error happen, fail to get regulator!");
		goto err_vftable;
	} else {
		last_vdd = regulator_get_voltage(vdd_sys) / 1000;
		pr_debug("last_vdd=%d\n", last_vdd);
	}

	if (vdd_sys) {
		regulator_put(vdd_sys);
		vdd_sys = NULL;
	}
#endif

	this_df->min_freq = this_df->scaling_min_freq = sunxi_ddrfreq_min;
	this_df->max_freq = this_df->scaling_max_freq = sunxi_ddrfreq_max;

	platform_set_drvdata(pdev, this_df);

#if defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW8P1)
	if (mdfs_in_cfs == 0) {
#endif

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_IN_VSYNC
		memset(&vbtime_ops, 0, sizeof(struct ddrfreq_vb_time_ops));
#endif

#if defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW8P1)
	}
#endif

#ifdef CONFIG_ARCH_SUN8IW5P1
#if defined(CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ) && defined(CONFIG_CPU_FREQ)
#if defined(CONFIG_DEVFREQ_GOV_DSM) && defined(CONFIG_EARLYSUSPEND)
	cpufreq_register_notifier(&ddrfreq_cpu_freq_transition_notifier,
						CPUFREQ_TRANSITION_NOTIFIER);
#endif
#endif

#endif

#if defined(CONFIG_ARCH_SUN8IW7P1) && defined(CONFIG_CPU_BUDGET_THERMAL)
	register_budget_cooling_notifier(&ddrfreq_budget_cooling_notifier);
#endif

#if defined(CONFIG_ARCH_SUN9IW1P1) || defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW7P1)
	pr_debug("__sram_start: 0x%0x, __sram_end: 0x%08x, "
			"__sram_text_start: 0x%08x, __sram_data_end: 0x%08x\n",
			(unsigned int)&__sram_start, (unsigned int)&__sram_end,
			(unsigned int)&__sram_text_start, (unsigned int)&__sram_data_end);
#endif

	return 0;

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_VDDSYS
err_vftable:
	devfreq_remove_device(this_df);
#endif

err_devfreq:
#ifdef CONFIG_ARCH_SUN9IW1P1
	clk_put(clk_pll6);
	clk_pll6 = NULL;
err_pll6:
	clk_put(clk_pll4);
	clk_pll4 = NULL;
err_pll4:
#elif defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW8P1)
	clk_put(clk_pll_ddr1);
	clk_pll_ddr1 = NULL;
err_pll_ddr1:
	clk_put(clk_pll_ddr0);
	clk_pll_ddr0 = NULL;
err_pll_ddr0:
#elif defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW7P1)
	clk_put(clk_pll_ddr0);
	clk_pll_ddr0 = NULL;
err_pll_ddr0:
#endif

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ
#if defined(CONFIG_ARCH_SUN9IW1P1)
	clk_put(gtbus);
	gtbus = NULL;
err_gtbus:
	clk_put(cci400);
	cci400 = NULL;
err_cci400:
	clk_put(ahb0);
	ahb0 = NULL;
err_ahb0:
	clk_put(ahb1);
	ahb1 = NULL;
err_ahb1:
	clk_put(ahb2);
	ahb2 = NULL;
err_ahb2:
#elif defined(CONFIG_ARCH_SUN8IW5P1)
	clk_put(ahb1);
	ahb1 = NULL;
err_ahb1:
#elif defined(CONFIG_ARCH_SUN8IW6P1)
	clk_put(cci400);
	cci400 = NULL;
err_cci400:
	clk_put(ahb1);
	ahb1 = NULL;
err_ahb1:
#endif
#endif /* CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ */

	return err;
}

static __devexit int sunxi_ddrfreq_remove(struct platform_device *pdev)
{
	struct devfreq *df = platform_get_drvdata(pdev);

#ifdef CONFIG_ARCH_SUN8IW5P1
#if defined(CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ) && defined(CONFIG_CPU_FREQ)
#if defined(CONFIG_DEVFREQ_GOV_DSM) && defined(CONFIG_EARLYSUSPEND)
	cpufreq_unregister_notifier(&ddrfreq_cpu_freq_transition_notifier,
						CPUFREQ_TRANSITION_NOTIFIER);
#endif
#endif
#endif

	devfreq_remove_device(df);

#ifdef CONFIG_ARCH_SUN9IW1P1
	if (!clk_pll6 || IS_ERR(clk_pll6)) {
		DDRFREQ_ERR("clk_pll6 handle is invalid, just return!\n");
		return -EINVAL;
	} else {
		clk_put(clk_pll6);
		clk_pll6 = NULL;
	}

	if (!clk_pll4 || IS_ERR(clk_pll4)) {
		DDRFREQ_ERR("clk_pll4 handle is invalid, just return!\n");
		return -EINVAL;
	} else {
		clk_put(clk_pll4);
		clk_pll4 = NULL;
	}

#elif defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW8P1)
	if (!clk_pll_ddr0 || IS_ERR(clk_pll_ddr0)) {
		DDRFREQ_ERR("clk_pll_ddr0 handle is invalid, just return!\n");
		return -EINVAL;
	} else {
		clk_put(clk_pll_ddr0);
		clk_pll_ddr0 = NULL;
	}

	if (!clk_pll_ddr1 || IS_ERR(clk_pll_ddr1)) {
		DDRFREQ_ERR("clk_pll_ddr1 handle is invalid, just return!\n");
		return -EINVAL;
	} else {
		clk_put(clk_pll_ddr1);
		clk_pll_ddr1 = NULL;
	}

#elif defined(CONFIG_ARCH_SUN8IW6P1) || defined(CONFIG_ARCH_SUN8IW7P1)
	if (!clk_pll_ddr0 || IS_ERR(clk_pll_ddr0)) {
		DDRFREQ_ERR("clk_pll_ddr0 handle is invalid, just return!\n");
		return -EINVAL;
	} else {
		clk_put(clk_pll_ddr0);
		clk_pll_ddr0 = NULL;
	}
#endif

#ifdef CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ
#if defined(CONFIG_ARCH_SUN9IW1P1)
	if (!gtbus || IS_ERR(gtbus)) {
		DDRFREQ_ERR("gtbus handle is invalid, just return!\n");
		return -EINVAL;
	} else {
		clk_put(gtbus);
		gtbus = NULL;
	}

	if (!cci400 || IS_ERR(cci400)) {
		DDRFREQ_ERR("cci400 handle is invalid, just return!\n");
		return -EINVAL;
	} else {
		clk_put(cci400);
		cci400 = NULL;
	}

	if (!ahb0 || IS_ERR(ahb0)) {
		DDRFREQ_ERR("ahb0 handle is invalid, just return!\n");
		return -EINVAL;
	} else {
		clk_put(ahb0);
		ahb0 = NULL;
	}

	if (!ahb1 || IS_ERR(ahb1)) {
		DDRFREQ_ERR("ahb1 handle is invalid, just return!\n");
		return -EINVAL;
	} else {
		clk_put(ahb1);
		ahb1 = NULL;
	}

	if (!ahb2 || IS_ERR(ahb2)) {
		DDRFREQ_ERR("ahb2 handle is invalid, just return!\n");
		return -EINVAL;
	} else {
		clk_put(ahb2);
		ahb2 = NULL;
	}

#elif defined(CONFIG_ARCH_SUN8IW5P1)
	if (!ahb1 || IS_ERR(ahb1)) {
		DDRFREQ_ERR("ahb1 handle is invalid, just return!\n");
		return -EINVAL;
	} else {
		clk_put(ahb1);
		ahb1 = NULL;
	}

#elif defined(CONFIG_ARCH_SUN8IW6P1)
	if (!cci400 || IS_ERR(cci400)) {
		DDRFREQ_ERR("cci400 handle is invalid, just return!\n");
		return -EINVAL;
	} else {
		clk_put(cci400);
		cci400 = NULL;
	}

	if (!ahb1 || IS_ERR(ahb1)) {
		DDRFREQ_ERR("ahb1 handle is invalid, just return!\n");
		return -EINVAL;
	} else {
		clk_put(ahb1);
		ahb1 = NULL;
	}
#endif

#endif /* CONFIG_DEVFREQ_DRAM_FREQ_BUSFREQ */

	return 0;
}

#ifdef CONFIG_PM
#ifndef CONFIG_ARCH_SUN8IW6P1
static int sunxi_ddrfreq_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct devfreq *df = platform_get_drvdata(pdev);
	int cur_freq;

	DDRFREQ_DBG(DEBUG_SUSPEND, "%s: enter!\n", __func__);

	cur_freq = __ddrfreq_get();
	if (cur_freq != df->max_freq) {
		if (!ddrfreq_target(&pdev->dev, &df->max_freq, 0)) {
			df->previous_freq = df->max_freq;
		}
	}

	return 0;
}

static int sunxi_ddrfreq_resume(struct platform_device *pdev)
{
	struct devfreq *df = platform_get_drvdata(pdev);
	int cur_freq;

	DDRFREQ_DBG(DEBUG_SUSPEND, "%s: enter!\n", __func__);

	cur_freq = __ddrfreq_get();
	if (df->previous_freq != cur_freq)
		df->previous_freq = cur_freq;

	return 0;
}
#endif
#endif

static struct platform_driver sunxi_ddrfreq_driver = {
	.probe  = sunxi_ddrfreq_probe,
	.remove = sunxi_ddrfreq_remove,
#ifdef CONFIG_PM
#ifndef CONFIG_ARCH_SUN8IW6P1
	.suspend = sunxi_ddrfreq_suspend,
	.resume  = sunxi_ddrfreq_resume,
#endif
#endif
	.driver = {
		.name  = "sunxi-ddrfreq",
		.owner = THIS_MODULE,
	},
};

struct platform_device sunxi_ddrfreq_device = {
	.name   = "sunxi-ddrfreq",
	.id     = -1,
};

static int __init sunxi_ddrfreq_init(void)
{
	int ret = 0;

	ret = platform_device_register(&sunxi_ddrfreq_device);
	if (ret) {
		DDRFREQ_ERR("dramfreq device init failed!\n");
		goto out;
	}

	ret = platform_driver_register(&sunxi_ddrfreq_driver);
	if (ret) {
		DDRFREQ_ERR("dramfreq driver init failed!\n");
		goto out;
	}

out:
	return ret;
}
fs_initcall(sunxi_ddrfreq_init);

#ifdef CONFIG_DEBUG_FS
static struct dentry *ddrfreq_root;

static int set_time_show(struct seq_file *s, void *data)
{
	seq_printf(s, "%llu\n", setfreq_time_usecs);
	return 0;
}

static int set_time_open(struct inode *inode, struct file *file)
{
	return single_open(file, set_time_show, inode->i_private);
}

static const struct file_operations set_time_fops = {
	.open = set_time_open,
	.read = seq_read,
};

static int get_time_show(struct seq_file *s, void *data)
{
	seq_printf(s, "%llu\n", getfreq_time_usecs);
	return 0;
}

static int get_time_open(struct inode *inode, struct file *file)
{
	return single_open(file, get_time_show, inode->i_private);
}

static const struct file_operations get_time_fops = {
	.open = get_time_open,
	.read = seq_read,
};

static int __init debug_init(void)
{
	int err = 0;

	ddrfreq_root = debugfs_create_dir("ddrfreq", 0);
	if (!ddrfreq_root)
		return -ENOMEM;

	if (!debugfs_create_file("get_time", 0444, ddrfreq_root,
								NULL, &get_time_fops))
	{
		err = -ENOMEM;
		goto out;
	}

	if (!debugfs_create_file("set_time", 0444, ddrfreq_root,
								NULL, &set_time_fops))
	{
		err = -ENOMEM;
		goto out;
	}

	return 0;

out:
	debugfs_remove_recursive(ddrfreq_root);
	return err;
}

static void __exit debug_exit(void)
{
	debugfs_remove_recursive(ddrfreq_root);
}

late_initcall(debug_init);
module_exit(debug_exit);
#endif /* CONFIG_DEBUG_FS */

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SUNXI ddrfreq driver with devfreq framework");
MODULE_AUTHOR("pannan");

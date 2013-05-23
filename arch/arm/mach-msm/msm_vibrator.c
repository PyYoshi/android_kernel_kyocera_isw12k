/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 */
/* include/asm/mach-msm/htc_pwrsink.h
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2011 Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/sched.h>
#include "pmic.h"
#include "timed_output.h"

#include <mach/msm_rpcrouter.h>

#include <linux/sensors/alps_io.h>

#define PM_LIBPROG      0x30000061

#ifdef CONFIG_FEATURE_KCC_00
#define PM_LIBVERS		0x00030005
#else 
#if (CONFIG_MSM_AMSS_VERSION == 6220) || (CONFIG_MSM_AMSS_VERSION == 6225)
#define PM_LIBVERS      0xfb837d0b
#else
#define PM_LIBVERS      0x10001
#endif
#endif 

#ifdef CONFIG_FEATURE_KCC_00
#define ONCRPC_PM_VIB_MOT_SET_VOLT_PROC	22
#else 
#define HTC_PROCEDURE_SET_VIB_ON_OFF	21
#endif 
#define PMIC_VIBRATOR_LEVEL	(3000)
#ifdef CONFIG_FEATURE_KCC_00
#define VIB_WORK_NUM		(5)

int (*g_AccSetPause)(int type, int pause_timer) = NULL;

struct vib_on_work_struct
{
	struct work_struct work_vibrator_on;
	int vibrator_timer;
};
static struct vib_on_work_struct vib_on_work_vibrator_on[VIB_WORK_NUM];
static struct work_struct work_vibrator_off[VIB_WORK_NUM];
static int work_vibrator_on_pos = 0;
static int work_vibrator_off_pos = 0;
static int rpc_vib_cnt = 0;
static struct mutex s_cmdmutex;
static struct mutex s_pedmutex;
#else
static struct work_struct work_vibrator_on;
static struct work_struct work_vibrator_off;
#endif
static struct hrtimer vibe_timer;

static int add_time_flag = 0;
#ifdef CONFIG_PM8XXX_RPC_VIBRATOR
static void set_pmic_vibrator(int on)
{
	int rc;

	rc = pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__MANUAL);
	if (rc) {
		pr_err("%s: Vibrator set mode failed", __func__);
		return;
	}

	if (on)
		rc = pmic_vib_mot_set_volt(PMIC_VIBRATOR_LEVEL);
	else
		rc = pmic_vib_mot_set_volt(0);

	if (rc)
		pr_err("%s: Vibrator set voltage level failed", __func__);
}
#else
#ifdef CONFIG_FEATURE_KCC_00
static void set_pmic_vibrator(int on, struct vib_on_work_struct* work)
#else 
static void set_pmic_vibrator(int on)
#endif 
{
	static struct msm_rpc_endpoint *vib_endpoint;
	struct set_vib_on_off_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;
    int ret;

	if (!vib_endpoint) {
		vib_endpoint = msm_rpc_connect(PM_LIBPROG, PM_LIBVERS, 0);
		if (IS_ERR(vib_endpoint)) {
			printk(KERN_ERR "init vib rpc failed!\n");
			vib_endpoint = 0;
			return;
		}
	}

    if (on && (g_AccSetPause != NULL)){
        mutex_lock(&s_pedmutex);
        ret = g_AccSetPause(SENSOR_EXT_FUNC_PEDOMETER | SENSOR_EXT_FUNC_VEHICLEMETER,work->vibrator_timer);
        mutex_unlock(&s_pedmutex);
    }

	if (on)
		req.data = cpu_to_be32(PMIC_VIBRATOR_LEVEL);
	else
		req.data = cpu_to_be32(0);
#ifdef CONFIG_FEATURE_KCC_00
    mutex_lock(&s_cmdmutex);
	msm_rpc_call(vib_endpoint, ONCRPC_PM_VIB_MOT_SET_VOLT_PROC, &req,
		sizeof(req), 1 * HZ);
/*		sizeof(req), 5 * HZ); */
    mutex_unlock(&s_cmdmutex);
	rpc_vib_cnt = on?(rpc_vib_cnt+1):0;
#else
	msm_rpc_call(vib_endpoint, HTC_PROCEDURE_SET_VIB_ON_OFF, &req,
		sizeof(req), 5 * HZ);
#endif 
#ifdef CONFIG_FEATURE_KCC_00
	if (on && work != NULL) {
		hrtimer_start(&vibe_timer,
				ktime_set(work->vibrator_timer / 1000, 
				(work->vibrator_timer % 1000) * 1000000),
				HRTIMER_MODE_REL);
	}
#endif
}
#endif 

static void pmic_vibrator_on(struct work_struct *work)
{
#ifdef CONFIG_FEATURE_KCC_00
	set_pmic_vibrator(1, (struct vib_on_work_struct*)work);
#else 
	set_pmic_vibrator(1);
#endif 
}

static void pmic_vibrator_off(struct work_struct *work)
{
#ifdef CONFIG_FEATURE_KCC_00
	set_pmic_vibrator(0, NULL);
#else 
	set_pmic_vibrator(0);
#endif 
}

#ifdef CONFIG_FEATURE_KCC_00
static void timed_vibrator_on(struct timed_output_dev *sdev, int timeout_val)
{
	vib_on_work_vibrator_on[work_vibrator_on_pos].vibrator_timer = timeout_val;

	if (schedule_work(&(vib_on_work_vibrator_on[work_vibrator_on_pos].work_vibrator_on)) != 0) {
		work_vibrator_on_pos++;
		if (work_vibrator_on_pos >= VIB_WORK_NUM) {
			work_vibrator_on_pos = 0;
		}
	}
}
#else
static void timed_vibrator_on(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_on);
}
#endif

static void timed_vibrator_off(struct timed_output_dev *sdev)
{
#ifdef CONFIG_FEATURE_KCC_00
	if (schedule_work(&work_vibrator_off[work_vibrator_off_pos]) != 0) {
		work_vibrator_off_pos++;
		if (work_vibrator_off_pos >= VIB_WORK_NUM) {
			work_vibrator_off_pos = 0;
		}
	}

#else 
	schedule_work(&work_vibrator_off);
#endif 
}

void registerCallbackFromAcc(int (*func)(int type, int pause_timer))
{
    g_AccSetPause = NULL;
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
#ifdef CONFIG_FEATURE_KCC_00
    if(((value <= 0) && (add_time_flag == 1)) ||
       ((value <= 0) && (rpc_vib_cnt == 0)))
#else 
    if((value <= 0) && (add_time_flag == 1))
#endif
    {
        return;
    }

	hrtimer_cancel(&vibe_timer);

#ifdef CONFIG_FEATURE_KCC_00
	if (value <= 0)
#else 
	if (value == 0)
#endif
		timed_vibrator_off(dev);
	else {
		value = (value > 15000 ? 15000 : value);
#ifdef CONFIG_FEATURE_KCC_00
        if(value <= 10)
        {
           value += 15;
           add_time_flag = 1;
        }
        else if(value <= 30)
        {
           value += 10;
           add_time_flag = 1;
        }
        else if(value <= 40)
        {
           value += 5;
           add_time_flag = 1;
        }

		timed_vibrator_on(dev, value);
#else 
		timed_vibrator_on(dev);

		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
#endif 
	}
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		struct timeval t = ktime_to_timeval(r);
		return t.tv_sec * 1000 + t.tv_usec / 1000;
	}
	return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
    add_time_flag = 0;

	timed_vibrator_off(NULL);
	return HRTIMER_NORESTART;
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

#ifdef CONFIG_FEATURE_KCC_00
static int __init msm_init_pmic_vibrator(void)
{
	int count = 0;

    mutex_init(&s_cmdmutex);
    mutex_init(&s_pedmutex);

	for (count = 0; count < VIB_WORK_NUM; count++) {
		INIT_WORK(&(vib_on_work_vibrator_on[count].work_vibrator_on), pmic_vibrator_on);
		INIT_WORK(&work_vibrator_off[count], pmic_vibrator_off);
		vib_on_work_vibrator_on[count].vibrator_timer = 0;
	}
	work_vibrator_on_pos = 0;
	work_vibrator_off_pos = 0;
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	return timed_output_dev_register(&pmic_vibrator);
}
static void __exit msm_exit_pmic_vibrator(void)
{
	timed_output_dev_unregister(&pmic_vibrator);
}
module_init(msm_init_pmic_vibrator);
module_exit(msm_exit_pmic_vibrator);

#else 
void __init msm_init_pmic_vibrator(void)
{
	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_WORK(&work_vibrator_off, pmic_vibrator_off);

	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&pmic_vibrator);
}
#endif 

EXPORT_SYMBOL(registerCallbackFromAcc);

MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");


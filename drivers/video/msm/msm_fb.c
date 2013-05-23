/* drivers/video/msm/msm_fb.c
 *
 * Core MSM framebuffer driver.
 *
 *
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 *
 * Copyright (C) 2007 Google Incorporated
 * Copyright (c) 2008-2012, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/msm_mdp.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <mach/board.h>
#include <linux/uaccess.h>

#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/debugfs.h>
#include <linux/console.h>
#include <linux/android_pmem.h>
#include <linux/leds.h>
#include <linux/pm_runtime.h>

#define MSM_FB_C
#include "msm_fb.h"
#include "mddihosti.h"
#include "tvenc.h"
#include "mdp.h"
#include "mdp4.h"

#include "mddihost.h"
#include <linux/kobject.h> 

#ifdef CONFIG_FB_MSM_LOGO
#define INIT_IMAGE_FILE "/initlogo.rle"
extern int load_565rle_image(char *filename);
#endif

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_NUM	3
#endif


#define MSM_FB_VAR_HEIGHT  86
#define MSM_FB_VAR_WIDTH   52
#define MSM_FB_REFRESH_RATE  58

static unsigned char *fbram;
static unsigned char *fbram_phys;
static int fbram_size;

static struct platform_device *pdev_list[MSM_FB_MAX_DEV_LIST];
static int pdev_list_cnt;

int vsync_mode = 1;

#define MAX_BLIT_REQ 256

#define MAX_FBI_LIST 32
static struct fb_info *fbi_list[MAX_FBI_LIST];
static int fbi_list_index;

static struct msm_fb_data_type *mfd_list[MAX_FBI_LIST];
static int mfd_list_index;

static u32 msm_fb_pseudo_palette[16] = {
	0x00000000, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
};

u32 msm_fb_debug_enabled;
/* Setting msm_fb_msg_level to 8 prints out ALL messages */
u32 msm_fb_msg_level = 7;

/* Setting mddi_msg_level to 8 prints out ALL messages */
u32 mddi_msg_level = 5;

extern int32 mdp_block_power_cnt[MDP_MAX_BLOCK];
extern unsigned long mdp_timer_duration;

static boolean msm_fb_first_opened = FALSE;

static int msm_fb_register(struct msm_fb_data_type *mfd);
static int msm_fb_open(struct fb_info *info, int user);
static int msm_fb_release(struct fb_info *info, int user);
static int msm_fb_pan_display(struct fb_var_screeninfo *var,
			      struct fb_info *info);
static int msm_fb_stop_sw_refresher(struct msm_fb_data_type *mfd);
int msm_fb_resume_sw_refresher(struct msm_fb_data_type *mfd);
static int msm_fb_check_var(struct fb_var_screeninfo *var,
			    struct fb_info *info);
static int msm_fb_set_par(struct fb_info *info);
static int msm_fb_blank_sub(int blank_mode, struct fb_info *info,
			    boolean op_enable);
static int msm_fb_suspend_sub(struct msm_fb_data_type *mfd);
static int msm_fb_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg);
static int msm_fb_mmap(struct fb_info *info, struct vm_area_struct * vma);

#ifdef MSM_FB_ENABLE_DBGFS

#define MSM_FB_MAX_DBGFS 1024
#define MAX_BACKLIGHT_BRIGHTNESS 255

int msm_fb_debugfs_file_index;
struct dentry *msm_fb_debugfs_root;
struct dentry *msm_fb_debugfs_file[MSM_FB_MAX_DBGFS];

uint8 g_mddi_pmdh_clock_off_prohibit = 0x00;
struct mddi_local_disp_state_type mddi_local_state;
extern struct semaphore disp_local_mutex;
uint8_t fb_dbg_msg_level = 0x00;

DEFINE_MUTEX(msm_fb_state_sem);

DEFINE_MUTEX(msm_fb_notify_update_sem);
void msmfb_no_update_notify_timer_cb(unsigned long data)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)data;
	if (!mfd)
		pr_err("%s mfd NULL\n", __func__);
	complete(&mfd->msmfb_no_update_notify);
}

struct dentry *msm_fb_get_debugfs_root(void)
{
	if (msm_fb_debugfs_root == NULL)
		msm_fb_debugfs_root = debugfs_create_dir("msm_fb", NULL);

	return msm_fb_debugfs_root;
}

void msm_fb_debugfs_file_create(struct dentry *root, const char *name,
				u32 *var)
{
	if (msm_fb_debugfs_file_index >= MSM_FB_MAX_DBGFS)
		return;

	msm_fb_debugfs_file[msm_fb_debugfs_file_index++] =
	    debugfs_create_u32(name, S_IRUGO | S_IWUSR, root, var);
}
#endif

int msm_fb_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	if (!mfd->cursor_update)
		return -ENODEV;

	return mfd->cursor_update(info, cursor);
}

static int msm_fb_resource_initialized;

#ifndef CONFIG_FB_BACKLIGHT
static int lcd_backlight_registered;

static void msm_fb_set_bl_brightness(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	struct msm_fb_data_type *mfd = dev_get_drvdata(led_cdev->dev->parent);
	int bl_lvl;

    DISP_LOCAL_LOG_EMERG("DISP msm_fb_set_bl_brightness S\n");
#if 0
	if (value > MAX_BACKLIGHT_BRIGHTNESS)
		value = MAX_BACKLIGHT_BRIGHTNESS;

	/* This maps android backlight level 0 to 255 into
	   driver backlight level 0 to bl_max with rounding */
	bl_lvl = (2 * value * mfd->panel_info.bl_max + MAX_BACKLIGHT_BRIGHTNESS)
		/(2 * MAX_BACKLIGHT_BRIGHTNESS);

	if (!bl_lvl && value)
		bl_lvl = 1;
#endif /* #if 0 */
    bl_lvl = value;

    if ( bl_lvl != mddi_local_state.mddi_brightness_level )
    {
        msm_fb_set_backlight(mfd, bl_lvl);
    }
    /* Maintain the brightness level of the state */
    mddi_local_state.mddi_brightness_level = bl_lvl;
    DISP_LOCAL_LOG_EMERG("DISP msm_fb_set_bl_brightness E\n");

}

static struct led_classdev backlight_led = {
	.name		= "lcd-backlight",
	.brightness	= MAX_BACKLIGHT_BRIGHTNESS,
	.brightness_set	= msm_fb_set_bl_brightness,
};
#endif

#define PANEL_NAME_MAX_LEN 30
static struct msm_fb_platform_data *msm_fb_pdata;
static char prim_panel_name[PANEL_NAME_MAX_LEN];
static char ext_panel_name[PANEL_NAME_MAX_LEN];
module_param_string(prim_display, prim_panel_name, sizeof(prim_panel_name) , 0);
module_param_string(ext_display, ext_panel_name, sizeof(ext_panel_name) , 0);

int msm_fb_detect_client(const char *name)
{
	int ret = 0;
	u32 len;
#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
/*	u32 id; */
#endif
	len = strnlen(name, PANEL_NAME_MAX_LEN);
	if (strnlen(prim_panel_name, PANEL_NAME_MAX_LEN)) {
		MSM_FB_DEBUG("\n name = %s, prim_display = %s",
			name, prim_panel_name);
		if (!strncmp((char *)prim_panel_name, name, len))
			return 0;
		else
			ret = -EPERM;
	}
	if (strnlen(ext_panel_name, PANEL_NAME_MAX_LEN)) {
		MSM_FB_DEBUG("\n name = %s, ext_display = %s",
			name, ext_panel_name);
		if (!strncmp((char *)ext_panel_name, name, len))
			return 0;
		else
			ret = -EPERM;
	}

	if (ret)
		return ret;

	ret = -EPERM;
	if (msm_fb_pdata && msm_fb_pdata->detect_client) {
		ret = msm_fb_pdata->detect_client(name);

		/* if it's non mddi panel, we need to pre-scan
		   mddi client to see if we can disable mddi host */

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
/* 		if (!ret && msm_fb_pdata->mddi_prescan) */
/* 			id = mddi_get_client_id(); */
#endif
	}

	return ret;
}

static ssize_t msm_fb_msm_fb_type(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct msm_fb_panel_data *pdata =
		(struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;

	switch (pdata->panel_info.type) {
	case NO_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "no panel\n");
		break;
	case MDDI_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "mddi panel\n");
		break;
	case EBI2_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "ebi2 panel\n");
		break;
	case LCDC_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "lcdc panel\n");
		break;
	case EXT_MDDI_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "ext mddi panel\n");
		break;
	case TV_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "tv panel\n");
		break;
	case HDMI_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "hdmi panel\n");
		break;
	case DTV_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "dtv panel\n");
		break;
	case MIPI_VIDEO_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "mipi dsi video panel\n");
		break;
	case MIPI_CMD_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "mipi dsi cmd panel\n");
		break;
	default:
		ret = snprintf(buf, PAGE_SIZE, "unknown panel\n");
		break;
	}

	return ret;
}

static u32 msm_fb_mode01;
static ssize_t msm_fb_store_mode01
(
    struct device *device,
    struct device_attribute *attr,
    const char *buf,
    size_t count
)
{
	char *last = NULL;

    msm_fb_mode01 = simple_strtoul(buf, &last, 0);

    return count;
}

static ssize_t msm_fb_show_mode01
(
    struct device *dev,
    struct device_attribute *attr,
    char *buf
)
{
    return  snprintf(buf, PAGE_SIZE,"%d\n", msm_fb_mode01);
}

struct fb_nv_data disp_cfg;
static ssize_t msm_fb_disp_cfg_store
(
    struct device *device,
    struct device_attribute *attr,
    const char *buf,
    size_t count
)
{
    struct fb_info *info;
    struct msm_fb_data_type *mfd;
    struct msm_fb_panel_data *pdata = NULL;

    info = dev_get_drvdata(device);
    mfd = (struct msm_fb_data_type *)info->par;
    pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;

    memset(&disp_cfg,0x0,sizeof(struct fb_nv_data));
    memcpy( &disp_cfg, buf, sizeof(struct fb_nv_data));

    pdata->set_nv(&disp_cfg);

    return count;
}

static uint8 msm_fb_rate01 = 0x00;
static ssize_t msm_fb_store_rate01
(
    struct device *device,
    struct device_attribute *attr,
    const char *buf,
    size_t count
)
{
    char *last = NULL;
    uint8 msm_fb_rate01_new;
    struct fb_info *fbi = dev_get_drvdata(device);

    msm_fb_rate01_new = (uint8)simple_strtoul(buf, &last, 0);
    if (last - buf >= count)
    {
        return -EINVAL;
    }

    if(msm_fb_rate01 != msm_fb_rate01_new)
    {
        msm_fb_rate01 = msm_fb_rate01_new;
        kobject_uevent(&(fbi->dev->kobj), KOBJ_CHANGE);
    }

    return count;
}

static ssize_t msm_fb_show_rate01
(
    struct device *dev,
    struct device_attribute *attr,
    char *buf
)
{
    return snprintf(buf, PAGE_SIZE,"%d\n", msm_fb_rate01);
}

static u32 msm_fb_mode02 = 0x00;
static ssize_t msm_fb_store_mode02
(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count
)
{
    char *last = NULL;
    struct fb_info *fbi = dev_get_drvdata(dev);
    struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
    struct msm_fb_panel_data *pdata =
        (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;

    msm_fb_mode02 = simple_strtoul(buf, &last, 0);

    if ((pdata) && (pdata->set_al_mode)) {
        /* PMDH Clock ON */
        pmdh_clk_func(1);
        pdata->set_al_mode(mfd, msm_fb_mode02);
        /* PMDH Clock OFF */
        pmdh_clk_func(0);
    }

    return count;
}

static ssize_t msm_fb_show_mode02
(
    struct device *dev,
    struct device_attribute *attr,
    char *buf
)
{
    return  snprintf(buf, PAGE_SIZE,"%d\n", msm_fb_mode02);
}
static DEVICE_ATTR(msm_fb_type, S_IRUGO, msm_fb_msm_fb_type, NULL);
static DEVICE_ATTR(msm_fb_mode01, S_IRUGO|S_IWUSR, msm_fb_show_mode01, msm_fb_store_mode01);

static DEVICE_ATTR(msm_fb_disp_cfg, S_IRUGO|S_IWUSR, NULL, msm_fb_disp_cfg_store);

static DEVICE_ATTR(msm_fb_rate01, S_IRUGO|S_IWUSR, msm_fb_show_rate01, msm_fb_store_rate01);
static DEVICE_ATTR(msm_fb_mode02, S_IRUGO|S_IWUSR, msm_fb_show_mode02, msm_fb_store_mode02);

static struct attribute *msm_fb_attrs[] = {
	&dev_attr_msm_fb_type.attr,
	&dev_attr_msm_fb_mode01.attr,
    &dev_attr_msm_fb_disp_cfg.attr,
    &dev_attr_msm_fb_rate01.attr,
    &dev_attr_msm_fb_mode02.attr,
	NULL,
};
static struct attribute_group msm_fb_attr_group = {
	.attrs = msm_fb_attrs,
};

static int msm_fb_create_sysfs(struct platform_device *pdev)
{
	int rc;
	struct msm_fb_data_type *mfd = platform_get_drvdata(pdev);

	rc = sysfs_create_group(&mfd->fbi->dev->kobj, &msm_fb_attr_group);
	if (rc)
		MSM_FB_ERR("%s: sysfs group creation failed, rc=%d\n", __func__,
			rc);
	return rc;
}
static void msm_fb_remove_sysfs(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd = platform_get_drvdata(pdev);
	sysfs_remove_group(&mfd->fbi->dev->kobj, &msm_fb_attr_group);
}

static void msm_fb_notify_done_open(struct fb_info *info)
{
    kobject_uevent(&(info->dev->kobj), KOBJ_CHANGE);

    return;
}

static int msm_fb_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	int rc;
	int err = 0;

	MSM_FB_DEBUG("msm_fb_probe\n");

	if ((pdev->id == 0) && (pdev->num_resources > 0)) {
		msm_fb_pdata = pdev->dev.platform_data;
		fbram_size =
			pdev->resource[0].end - pdev->resource[0].start + 1;
		fbram_phys = (char *)pdev->resource[0].start;
		fbram = __va(fbram_phys);

		if (!fbram) {
			printk(KERN_ERR "fbram ioremap failed!\n");
			return -ENOMEM;
		}
		MSM_FB_DEBUG("msm_fb_probe:  phy_Addr = 0x%x virt = 0x%x\n",
			     (int)fbram_phys, (int)fbram);

		msm_fb_resource_initialized = 1;
		return 0;
	}

	if (!msm_fb_resource_initialized)
		return -EPERM;

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (pdev_list_cnt >= MSM_FB_MAX_DEV_LIST)
		return -ENOMEM;

	mfd->panel_info.frame_count = 0;
	mfd->bl_level = 0;
#ifdef CONFIG_FB_MSM_OVERLAY
	mfd->overlay_play_enable = 1;
#endif
	rc = msm_fb_register(mfd);
	if (rc)
		return rc;
	err = pm_runtime_set_active(mfd->fbi->dev);
	if (err < 0)
		printk(KERN_ERR "pm_runtime: fail to set active.\n");
	pm_runtime_enable(mfd->fbi->dev);
#ifdef CONFIG_FB_BACKLIGHT
	msm_fb_config_backlight(mfd);
#else
	/* android supports only one lcd-backlight/lcd for now */
	if (!lcd_backlight_registered) {
		if (led_classdev_register(&pdev->dev, &backlight_led))
			printk(KERN_ERR "led_classdev_register failed\n");
		else
			lcd_backlight_registered = 1;
	}
#endif

	pdev_list[pdev_list_cnt++] = pdev;
	msm_fb_create_sysfs(pdev);
	return 0;
}

static int msm_fb_remove(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	MSM_FB_DEBUG("msm_fb_remove\n");

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	msm_fb_remove_sysfs(pdev);

	pm_runtime_disable(mfd->fbi->dev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (msm_fb_suspend_sub(mfd))
		printk(KERN_ERR "msm_fb_remove: can't stop the device %d\n", mfd->index);

	if (mfd->channel_irq != 0)
		free_irq(mfd->channel_irq, (void *)mfd);

	if (mfd->vsync_width_boundary)
		vfree(mfd->vsync_width_boundary);

	if (mfd->vsync_resync_timer.function)
		del_timer(&mfd->vsync_resync_timer);

	if (mfd->refresh_timer.function)
		del_timer(&mfd->refresh_timer);

	if (mfd->dma_hrtimer.function)
		hrtimer_cancel(&mfd->dma_hrtimer);

	if (mfd->msmfb_no_update_notify_timer.function)
		del_timer(&mfd->msmfb_no_update_notify_timer);
	complete(&mfd->msmfb_no_update_notify);
	complete(&mfd->msmfb_update_notify);

	/* remove /dev/fb* */
	unregister_framebuffer(mfd->fbi);

#ifdef CONFIG_FB_BACKLIGHT
	/* remove /sys/class/backlight */
	backlight_device_unregister(mfd->fbi->bl_dev);
#else
	if (lcd_backlight_registered) {
		lcd_backlight_registered = 0;
		led_classdev_unregister(&backlight_led);
	}
#endif

#ifdef MSM_FB_ENABLE_DBGFS
	if (mfd->sub_dir)
		debugfs_remove(mfd->sub_dir);
#endif

	return 0;
}

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int msm_fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct msm_fb_data_type *mfd;
	int ret = 0;

	MSM_FB_DEBUG("msm_fb_suspend\n");

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	if ((!mfd) || (mfd->key != MFD_KEY))
		return 0;

	console_lock();
	fb_set_suspend(mfd->fbi, FBINFO_STATE_SUSPENDED);

	ret = msm_fb_suspend_sub(mfd);
	if (ret != 0) {
		printk(KERN_ERR "msm_fb: failed to suspend! %d\n", ret);
		fb_set_suspend(mfd->fbi, FBINFO_STATE_RUNNING);
	} else {
		pdev->dev.power.power_state = state;
	}

	console_unlock();
	return ret;
}
#else
#define msm_fb_suspend NULL
#endif

static int msm_fb_suspend_sub(struct msm_fb_data_type *mfd)
{
	int ret = 0;

	if ((!mfd) || (mfd->key != MFD_KEY))
		return 0;

	if (mfd->msmfb_no_update_notify_timer.function)
		del_timer(&mfd->msmfb_no_update_notify_timer);
	complete(&mfd->msmfb_no_update_notify);

	/*
	 * suspend this channel
	 */
	mfd->suspend.sw_refreshing_enable = mfd->sw_refreshing_enable;
	mfd->suspend.op_enable = mfd->op_enable;
	mfd->suspend.panel_power_on = mfd->panel_power_on;

	if (mfd->op_enable) {
		ret =
		     msm_fb_blank_sub(FB_BLANK_POWERDOWN, mfd->fbi,
				      mfd->suspend.op_enable);
		if (ret) {
			MSM_FB_INFO
			    ("msm_fb_suspend: can't turn off display!\n");
			return ret;
		}
		mfd->op_enable = FALSE;
	}
	/*
	 * try to power down
	 */
	mdp_pipe_ctrl(MDP_MASTER_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	/*
	 * detach display channel irq if there's any
	 * or wait until vsync-resync completes
	 */
	if ((mfd->dest == DISPLAY_LCD)) {
		if (mfd->panel_info.lcd.vsync_enable) {
			if (mfd->panel_info.lcd.hw_vsync_mode) {
				if (mfd->channel_irq != 0)
					disable_irq(mfd->channel_irq);
			} else {
				volatile boolean vh_pending;
				do {
					vh_pending = mfd->vsync_handler_pending;
				} while (vh_pending);
			}
		}
	}

	return 0;
}

#ifdef CONFIG_PM
static int msm_fb_resume_sub(struct msm_fb_data_type *mfd)
{
	int ret = 0;

	if ((!mfd) || (mfd->key != MFD_KEY))
		return 0;

	/* attach display channel irq if there's any */
	if (mfd->channel_irq != 0)
		enable_irq(mfd->channel_irq);

	/* resume state var recover */
	mfd->sw_refreshing_enable = mfd->suspend.sw_refreshing_enable;
	mfd->op_enable = mfd->suspend.op_enable;

	if (mfd->suspend.panel_power_on) {
		ret =
		     msm_fb_blank_sub(FB_BLANK_UNBLANK, mfd->fbi,
				      mfd->op_enable);
		if (ret)
			MSM_FB_INFO("msm_fb_resume: can't turn on display!\n");
	}

	return ret;
}
#endif

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int msm_fb_resume(struct platform_device *pdev)
{
	/* This resume function is called when interrupt is enabled.
	 */
	int ret = 0;
	struct msm_fb_data_type *mfd;

	MSM_FB_DEBUG("msm_fb_resume\n");

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	if ((!mfd) || (mfd->key != MFD_KEY))
		return 0;

	console_lock();
	ret = msm_fb_resume_sub(mfd);
	pdev->dev.power.power_state = PMSG_ON;
	fb_set_suspend(mfd->fbi, FBINFO_STATE_RUNNING);
	console_unlock();

	return ret;
}
#else
#define msm_fb_resume NULL
#endif

static int msm_fb_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int msm_fb_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static int msm_fb_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: idling...\n");
	return 0;
}

static int msm_fb_ext_suspend(struct device *dev)
{
	struct msm_fb_data_type *mfd = dev_get_drvdata(dev);
	int ret = 0;

	if ((!mfd) || (mfd->key != MFD_KEY))
		return 0;

	if (mfd->panel_info.type == HDMI_PANEL ||
		mfd->panel_info.type == DTV_PANEL)
		ret = msm_fb_suspend_sub(mfd);

	return ret;
}

static int msm_fb_ext_resume(struct device *dev)
{
	struct msm_fb_data_type *mfd = dev_get_drvdata(dev);
	int ret = 0;

	if ((!mfd) || (mfd->key != MFD_KEY))
		return 0;

	if (mfd->panel_info.type == HDMI_PANEL ||
		mfd->panel_info.type == DTV_PANEL)
		ret = msm_fb_resume_sub(mfd);

	return ret;
}

static struct dev_pm_ops msm_fb_dev_pm_ops = {
	.runtime_suspend = msm_fb_runtime_suspend,
	.runtime_resume = msm_fb_runtime_resume,
	.runtime_idle = msm_fb_runtime_idle,
	.suspend = msm_fb_ext_suspend,
	.resume = msm_fb_ext_resume,
};

static struct platform_driver msm_fb_driver = {
	.probe = msm_fb_probe,
	.remove = msm_fb_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = msm_fb_suspend,
	.resume = msm_fb_resume,
#endif
	.shutdown = NULL,
	.driver = {
		   /* Driver name must match the device name added in platform.c. */
		   .name = "msm_fb",
		   .pm = &msm_fb_dev_pm_ops,
		   },
};

/* #if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB_MSM_MDP303) */
#if defined(CONFIG_HAS_EARLYSUSPEND)

static void memset32_io(u32 __iomem *_ptr, u32 val, size_t count)
{
	count >>= 2;
	while (count--)
		writel(val, _ptr++);
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void msmfb_early_suspend(struct early_suspend *h)
{
	struct msm_fb_data_type *mfd = container_of(h, struct msm_fb_data_type,
						    early_suspend);
/*#if defined(CONFIG_FB_MSM_MDP303) */

	/*
	* For MDP with overlay, set framebuffer with black pixels
	* to show black screen on HDMI.
	*/
	struct fb_info *fbi = mfd->fbi;
    DISP_LOCAL_LOG_EMERG("DISP msmfb_early_suspend S\n");

    mutex_lock(&msm_fb_state_sem);
/*	switch (mfd->fbi->var.bits_per_pixel) { */
/*	case 32: */
/*		memset32_io((void *)fbi->screen_base, 0xFF000000, */
/*							fbi->fix.smem_len); */
/*		break; */
/*	default: */
/*		memset32_io((void *)fbi->screen_base, 0x00, fbi->fix.smem_len); */
/*		break; */
/*	} */
/*#endif */
	if( msm_pmdh_base != NULL ){
		msm_fb_suspend_sub(mfd);
		switch (mfd->fbi->var.bits_per_pixel) {
		case 32:
			memset32_io((void *)fbi->screen_base, 0xFF000000,
								fbi->fix.smem_len);
			break;
		default:
			memset32_io((void *)fbi->screen_base, 0x00, fbi->fix.smem_len);
			break;
		}
	}

    mutex_unlock(&msm_fb_state_sem);
    DISP_LOCAL_LOG_EMERG("DISP msmfb_early_suspend E\n");
}

static void msmfb_early_resume(struct early_suspend *h)
{
	struct msm_fb_data_type *mfd = container_of(h, struct msm_fb_data_type,
						    early_suspend);

    DISP_LOCAL_LOG_EMERG("DISP msmfb_early_resume S\n");

    mutex_lock(&msm_fb_state_sem);

	if( msm_pmdh_base != NULL ){
		msm_fb_resume_sub(mfd);
	}

    mutex_unlock(&msm_fb_state_sem);

    DISP_LOCAL_LOG_EMERG("DISP msmfb_early_resume E\n");

}
#endif

static int unset_bl_level, bl_updated;
static int bl_level_old;

void msm_fb_set_backlight(struct msm_fb_data_type *mfd, __u32 bkl_lvl)
{
	struct msm_fb_panel_data *pdata;

	if (!mfd->panel_power_on || !bl_updated) {
		unset_bl_level = bkl_lvl;
		return;
	} else {
		unset_bl_level = 0;
	}

	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;

/*	if ((pdata) && (pdata->set_backlight)) { */
	if ((pdata) && (pdata->set_backlight) && (mfd->panel_power_on)) {

/*		down(&mfd->sem); */
		if (bl_level_old == bkl_lvl) {
/*			up(&mfd->sem); */
			return;
		}
		mfd->bl_level = bkl_lvl;
        /* PMDH Clock ON */
        pmdh_clk_func(1);

		pdata->set_backlight(mfd);
		bl_level_old = mfd->bl_level;
        /* PMDH Clock OFF */
        pmdh_clk_func(0);
/*		up(&mfd->sem); */

	}
}

static int msm_fb_blank_sub(int blank_mode, struct fb_info *info,
			    boolean op_enable)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct msm_fb_panel_data *pdata = NULL;
	int ret = 0;

    MSM_FB_DEBUG("msm_fb_blank_sub():start op_enable = %d-----\n",op_enable);

	if (!op_enable)
		return -EPERM;

	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;
	if ((!pdata) || (!pdata->on) || (!pdata->off)) {
		printk(KERN_ERR "msm_fb_blank_sub: no panel operation detected!\n");
		return -ENODEV;
	}

	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		if (!mfd->panel_power_on) {
			msleep(16);

        MSM_FB_DEBUG("msm_fb_blank_sub():pdata->on-----\n");

			ret = pdata->on(mfd->pdev);
			if (ret == 0) {
				mfd->panel_power_on = TRUE;

/* ToDo: possible conflict with android which doesn't expect sw refresher */
/*
	  if (!mfd->hw_refresh)
	  {
	    if ((ret = msm_fb_resume_sw_refresher(mfd)) != 0)
	    {
	      MSM_FB_INFO("msm_fb_blank_sub: msm_fb_resume_sw_refresher failed = %d!\n",ret);
	    }
	  }
*/
			}
		}
		break;

	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
	case FB_BLANK_POWERDOWN:
	default:
		if (mfd->panel_power_on) {
			int curr_pwr_state;

			mfd->op_enable = FALSE;
			curr_pwr_state = mfd->panel_power_on;
			mfd->panel_power_on = FALSE;
			bl_updated = 0;

			msleep(16);
			ret = pdata->off(mfd->pdev);
			if (ret)
				mfd->panel_power_on = curr_pwr_state;

			mfd->op_enable = TRUE;
		}
		break;
	}

	return ret;
}

static void msm_fb_fillrect(struct fb_info *info,
			    const struct fb_fillrect *rect)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	cfb_fillrect(info, rect);
	if (!mfd->hw_refresh && (info->var.yoffset == 0) &&
		!mfd->sw_currently_refreshing) {
		struct fb_var_screeninfo var;

		var = info->var;
		var.reserved[0] = 0x54445055;
		var.reserved[1] = (rect->dy << 16) | (rect->dx);
		var.reserved[2] = ((rect->dy + rect->height) << 16) |
		    (rect->dx + rect->width);

		msm_fb_pan_display(&var, info);
	}
}

static void msm_fb_copyarea(struct fb_info *info,
			    const struct fb_copyarea *area)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	cfb_copyarea(info, area);
	if (!mfd->hw_refresh && (info->var.yoffset == 0) &&
		!mfd->sw_currently_refreshing) {
		struct fb_var_screeninfo var;

		var = info->var;
		var.reserved[0] = 0x54445055;
		var.reserved[1] = (area->dy << 16) | (area->dx);
		var.reserved[2] = ((area->dy + area->height) << 16) |
		    (area->dx + area->width);

		msm_fb_pan_display(&var, info);
	}
}

static void msm_fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	cfb_imageblit(info, image);
	if (!mfd->hw_refresh && (info->var.yoffset == 0) &&
		!mfd->sw_currently_refreshing) {
		struct fb_var_screeninfo var;

		var = info->var;
		var.reserved[0] = 0x54445055;
		var.reserved[1] = (image->dy << 16) | (image->dx);
		var.reserved[2] = ((image->dy + image->height) << 16) |
		    (image->dx + image->width);

		msm_fb_pan_display(&var, info);
	}
}

static int msm_fb_blank(int blank_mode, struct fb_info *info)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	return msm_fb_blank_sub(blank_mode, info, mfd->op_enable);
}

static int msm_fb_set_lut(struct fb_cmap *cmap, struct fb_info *info)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	if (!mfd->lut_update)
		return -ENODEV;

	mfd->lut_update(info, cmap);
	return 0;
}

/*
 * Custom Framebuffer mmap() function for MSM driver.
 * Differs from standard mmap() function by allowing for customized
 * page-protection.
 */
static int msm_fb_mmap(struct fb_info *info, struct vm_area_struct * vma)
{
	/* Get frame buffer memory range. */
	unsigned long start = info->fix.smem_start;
	u32 len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.smem_len);
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	if (off >= len) {
		/* memory mapped io */
		off -= len;
		if (info->var.accel_flags) {
			mutex_unlock(&info->lock);
			return -EINVAL;
		}
		start = info->fix.mmio_start;
		len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.mmio_len);
	}

	/* Set VM flags. */
	start &= PAGE_MASK;
	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;
	vma->vm_pgoff = off >> PAGE_SHIFT;
	/* This is an IO map - tell maydump to skip this VMA */
	vma->vm_flags |= VM_IO | VM_RESERVED;

	/* Set VM page protection */
	if (mfd->mdp_fb_page_protection == MDP_FB_PAGE_PROTECTION_WRITECOMBINE)
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	else if (mfd->mdp_fb_page_protection ==
			MDP_FB_PAGE_PROTECTION_WRITETHROUGHCACHE)
		vma->vm_page_prot = pgprot_writethroughcache(vma->vm_page_prot);
	else if (mfd->mdp_fb_page_protection ==
			MDP_FB_PAGE_PROTECTION_WRITEBACKCACHE)
		vma->vm_page_prot = pgprot_writebackcache(vma->vm_page_prot);
	else if (mfd->mdp_fb_page_protection ==
			MDP_FB_PAGE_PROTECTION_WRITEBACKWACACHE)
		vma->vm_page_prot = pgprot_writebackwacache(vma->vm_page_prot);
	else
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* Remap the frame buffer I/O range */
	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static struct fb_ops msm_fb_ops = {
	.owner = THIS_MODULE,
	.fb_open = msm_fb_open,
	.fb_release = msm_fb_release,
	.fb_read = NULL,
	.fb_write = NULL,
	.fb_cursor = NULL,
	.fb_check_var = msm_fb_check_var,	/* vinfo check */
	.fb_set_par = msm_fb_set_par,	/* set the video mode according to info->var */
	.fb_setcolreg = NULL,	/* set color register */
	.fb_blank = msm_fb_blank,	/* blank display */
	.fb_pan_display = msm_fb_pan_display,	/* pan display */
	.fb_fillrect = msm_fb_fillrect,	/* Draws a rectangle */
	.fb_copyarea = msm_fb_copyarea,	/* Copy data from area to another */
	.fb_imageblit = msm_fb_imageblit,	/* Draws a image to the display */
	.fb_rotate = NULL,
	.fb_sync = NULL,	/* wait for blit idle, optional */
	.fb_ioctl = msm_fb_ioctl,	/* perform fb specific ioctl (optional) */
	.fb_mmap = msm_fb_mmap,
};

static __u32 msm_fb_line_length(__u32 fb_index, __u32 xres, int bpp)
{
	/* The adreno GPU hardware requires that the pitch be aligned to
	   32 pixels for color buffers, so for the cases where the GPU
	   is writing directly to fb0, the framebuffer pitch
	   also needs to be 32 pixel aligned */

	if (fb_index == 0)
		return ALIGN(xres, 32) * bpp;
	else
		return xres * bpp;
}

static int msm_fb_register(struct msm_fb_data_type *mfd)
{
	int ret = -ENODEV;
	int bpp;
	struct msm_panel_info *panel_info = &mfd->panel_info;
	struct fb_info *fbi = mfd->fbi;
	struct fb_fix_screeninfo *fix;
	struct fb_var_screeninfo *var;
	int *id;
	int fbram_offset;

	/*
	 * fb info initialization
	 */
	fix = &fbi->fix;
	var = &fbi->var;

	fix->type_aux = 0;	/* if type == FB_TYPE_INTERLEAVED_PLANES */
	fix->visual = FB_VISUAL_TRUECOLOR;	/* True Color */
	fix->ywrapstep = 0;	/* No support */
	fix->mmio_start = 0;	/* No MMIO Address */
	fix->mmio_len = 0;	/* No MMIO Address */
	fix->accel = FB_ACCEL_NONE;/* FB_ACCEL_MSM needes to be added in fb.h */

	var->xoffset = 0,	/* Offset from virtual to visible */
	var->yoffset = 0,	/* resolution */
	var->grayscale = 0,	/* No graylevels */
	var->nonstd = 0,	/* standard pixel format */
	var->activate = FB_ACTIVATE_VBL,	/* activate it at vsync */

/*	var->height = -1,	 */ /* height of picture in mm */
/*	var->width = -1,	 */ /* width of picture in mm */
    var->height = MSM_FB_VAR_HEIGHT, /* height of picture in mm */
    var->width  = MSM_FB_VAR_WIDTH,  /* width of picture in mm */

	var->accel_flags = 0,	/* acceleration flags */
	var->sync = 0,	/* see FB_SYNC_* */
	var->rotate = 0,	/* angle we rotate counter clockwise */
	mfd->op_enable = FALSE;

	switch (mfd->fb_imgType) {
	case MDP_RGB_565:
		fix->type = FB_TYPE_PACKED_PIXELS;
		fix->xpanstep = 1;
		fix->ypanstep = 1;
		var->vmode = FB_VMODE_NONINTERLACED;
		var->blue.offset = 0;
		var->green.offset = 5;
		var->red.offset = 11;
		var->blue.length = 5;
		var->green.length = 6;
		var->red.length = 5;
		var->blue.msb_right = 0;
		var->green.msb_right = 0;
		var->red.msb_right = 0;
		var->transp.offset = 0;
		var->transp.length = 0;
		bpp = 2;
		break;

	case MDP_RGB_888:
		fix->type = FB_TYPE_PACKED_PIXELS;
		fix->xpanstep = 1;
		fix->ypanstep = 1;
		var->vmode = FB_VMODE_NONINTERLACED;
		var->blue.offset = 0;
		var->green.offset = 8;
		var->red.offset = 16;
		var->blue.length = 8;
		var->green.length = 8;
		var->red.length = 8;
		var->blue.msb_right = 0;
		var->green.msb_right = 0;
		var->red.msb_right = 0;
		var->transp.offset = 0;
		var->transp.length = 0;
		bpp = 3;
		break;

	case MDP_ARGB_8888:
		fix->type = FB_TYPE_PACKED_PIXELS;
		fix->xpanstep = 1;
		fix->ypanstep = 1;
		var->vmode = FB_VMODE_NONINTERLACED;
		var->blue.offset = 0;
		var->green.offset = 8;
		var->red.offset = 16;
		var->blue.length = 8;
		var->green.length = 8;
		var->red.length = 8;
		var->blue.msb_right = 0;
		var->green.msb_right = 0;
		var->red.msb_right = 0;
		var->transp.offset = 24;
		var->transp.length = 8;
		bpp = 4;
		break;

	case MDP_RGBA_8888:
		fix->type = FB_TYPE_PACKED_PIXELS;
		fix->xpanstep = 1;
		fix->ypanstep = 1;
		var->vmode = FB_VMODE_NONINTERLACED;
		var->blue.offset = 8;
		var->green.offset = 16;
		var->red.offset = 24;
		var->blue.length = 8;
		var->green.length = 8;
		var->red.length = 8;
		var->blue.msb_right = 0;
		var->green.msb_right = 0;
		var->red.msb_right = 0;
		var->transp.offset = 0;
		var->transp.length = 8;
		bpp = 4;
		break;

	case MDP_YCRYCB_H2V1:
		/* ToDo: need to check TV-Out YUV422i framebuffer format */
		/*       we might need to create new type define */
		fix->type = FB_TYPE_INTERLEAVED_PLANES;
		fix->xpanstep = 2;
		fix->ypanstep = 1;
		var->vmode = FB_VMODE_NONINTERLACED;

		/* how about R/G/B offset? */
		var->blue.offset = 0;
		var->green.offset = 5;
		var->red.offset = 11;
		var->blue.length = 5;
		var->green.length = 6;
		var->red.length = 5;
		var->blue.msb_right = 0;
		var->green.msb_right = 0;
		var->red.msb_right = 0;
		var->transp.offset = 0;
		var->transp.length = 0;
		bpp = 2;
		break;

	default:
		MSM_FB_ERR("msm_fb_init: fb %d unkown image type!\n",
			   mfd->index);
		return ret;
	}

	fix->type = panel_info->is_3d_panel;

	fix->line_length = msm_fb_line_length(mfd->index, panel_info->xres,
					      bpp);
	/* calculate smem_len based on max size of two supplied modes */
	fix->smem_len = roundup(MAX(msm_fb_line_length(mfd->index,
					       panel_info->xres,
					       bpp) *
			    panel_info->yres * mfd->fb_page,
			    msm_fb_line_length(mfd->index,
					       panel_info->mode2_xres,
					       bpp) *
			    panel_info->mode2_yres * mfd->fb_page), PAGE_SIZE);



	mfd->var_xres = panel_info->xres;
	mfd->var_yres = panel_info->yres;

	var->pixclock = mfd->panel_info.clk_rate;
	mfd->var_pixclock = var->pixclock;

	var->xres = panel_info->xres;
	var->yres = panel_info->yres;
	var->xres_virtual = panel_info->xres;
	var->yres_virtual = panel_info->yres * mfd->fb_page;
	var->bits_per_pixel = bpp * 8;	/* FrameBuffer color depth */
	if (mfd->dest == DISPLAY_LCD) {
		var->reserved[4] = MSM_FB_REFRESH_RATE;
	} else {
		if (panel_info->type == MIPI_VIDEO_PANEL) {
			var->reserved[4] = panel_info->mipi.frame_rate;
		} else {
			var->reserved[4] = panel_info->clk_rate /
				((panel_info->lcdc.h_back_porch +
				  panel_info->lcdc.h_front_porch +
				  panel_info->lcdc.h_pulse_width +
				  panel_info->xres) *
				 (panel_info->lcdc.v_back_porch +
				  panel_info->lcdc.v_front_porch +
				  panel_info->lcdc.v_pulse_width +
				  panel_info->yres));
		}
	}
	pr_debug("reserved[4] %u\n", var->reserved[4]);

		/*
		 * id field for fb app
		 */
	    id = (int *)&mfd->panel;

#if defined(CONFIG_FB_MSM_MDP22)
	snprintf(fix->id, sizeof(fix->id), "msmfb22_%x", (__u32) *id);
#elif defined(CONFIG_FB_MSM_MDP30)
	snprintf(fix->id, sizeof(fix->id), "msmfb30_%x", (__u32) *id);
#elif defined(CONFIG_FB_MSM_MDP31)
	snprintf(fix->id, sizeof(fix->id), "msmfb31_%x", (__u32) *id);
#elif defined(CONFIG_FB_MSM_MDP40)
	snprintf(fix->id, sizeof(fix->id), "msmfb40_%x", (__u32) *id);
#else
	error CONFIG_FB_MSM_MDP undefined !
#endif
	 fbi->fbops = &msm_fb_ops;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->pseudo_palette = msm_fb_pseudo_palette;

	mfd->ref_cnt = 0;
	mfd->sw_currently_refreshing = FALSE;
	mfd->sw_refreshing_enable = TRUE;
	mfd->panel_power_on = FALSE;

	mfd->pan_waiting = FALSE;
	init_completion(&mfd->pan_comp);
	init_completion(&mfd->refresher_comp);
	sema_init(&mfd->sem, 1);

	init_timer(&mfd->msmfb_no_update_notify_timer);
	mfd->msmfb_no_update_notify_timer.function =
			msmfb_no_update_notify_timer_cb;
	mfd->msmfb_no_update_notify_timer.data = (unsigned long)mfd;
	init_completion(&mfd->msmfb_update_notify);
	init_completion(&mfd->msmfb_no_update_notify);

	fbram_offset = PAGE_ALIGN((int)fbram)-(int)fbram;
	fbram += fbram_offset;
	fbram_phys += fbram_offset;
	fbram_size -= fbram_offset;

	if (fbram_size < fix->smem_len) {
		printk(KERN_ERR "error: no more framebuffer memory!\n");
		return -ENOMEM;
	}

	fbi->screen_base = fbram;
	fbi->fix.smem_start = (unsigned long)fbram_phys;

	memset(fbi->screen_base, 0x0, fix->smem_len);

	mfd->op_enable = TRUE;
	mfd->panel_power_on = FALSE;

	/* cursor memory allocation */
	if (mfd->cursor_update) {
		mfd->cursor_buf = dma_alloc_coherent(NULL,
					MDP_CURSOR_SIZE,
					(dma_addr_t *) &mfd->cursor_buf_phys,
					GFP_KERNEL);
		if (!mfd->cursor_buf)
			mfd->cursor_update = 0;
	}

	if (mfd->lut_update) {
		ret = fb_alloc_cmap(&fbi->cmap, 256, 0);
		if (ret)
			printk(KERN_ERR "%s: fb_alloc_cmap() failed!\n",
					__func__);
	}

	if (register_framebuffer(fbi) < 0) {
		if (mfd->lut_update)
			fb_dealloc_cmap(&fbi->cmap);

		if (mfd->cursor_buf)
			dma_free_coherent(NULL,
				MDP_CURSOR_SIZE,
				mfd->cursor_buf,
				(dma_addr_t) mfd->cursor_buf_phys);

		mfd->op_enable = FALSE;
		return -EPERM;
	}

	fbram += fix->smem_len;
	fbram_phys += fix->smem_len;
	fbram_size -= fix->smem_len;

	MSM_FB_INFO
	    ("FrameBuffer[%d] %dx%d size=%d bytes is registered successfully!\n",
	     mfd->index, fbi->var.xres, fbi->var.yres, fbi->fix.smem_len);

#ifdef CONFIG_FB_MSM_LOGO
	if (!load_565rle_image(INIT_IMAGE_FILE)) ;	/* Flip buffer */
#endif
	ret = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (mfd->panel_info.type != DTV_PANEL) {
		mfd->early_suspend.suspend = msmfb_early_suspend;
		mfd->early_suspend.resume = msmfb_early_resume;
		mfd->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 2;
		register_early_suspend(&mfd->early_suspend);
	}
#endif

#ifdef MSM_FB_ENABLE_DBGFS
	{
		struct dentry *root;
		struct dentry *sub_dir;
		char sub_name[2];

		root = msm_fb_get_debugfs_root();
		if (root != NULL) {
			sub_name[0] = (char)(mfd->index + 0x30);
			sub_name[1] = '\0';
			sub_dir = debugfs_create_dir(sub_name, root);
		} else {
			sub_dir = NULL;
		}

		mfd->sub_dir = sub_dir;

		if (sub_dir) {
			msm_fb_debugfs_file_create(sub_dir, "op_enable",
						   (u32 *) &mfd->op_enable);
			msm_fb_debugfs_file_create(sub_dir, "panel_power_on",
						   (u32 *) &mfd->
						   panel_power_on);
			msm_fb_debugfs_file_create(sub_dir, "ref_cnt",
						   (u32 *) &mfd->ref_cnt);
			msm_fb_debugfs_file_create(sub_dir, "fb_imgType",
						   (u32 *) &mfd->fb_imgType);
			msm_fb_debugfs_file_create(sub_dir,
						   "sw_currently_refreshing",
						   (u32 *) &mfd->
						   sw_currently_refreshing);
			msm_fb_debugfs_file_create(sub_dir,
						   "sw_refreshing_enable",
						   (u32 *) &mfd->
						   sw_refreshing_enable);

			msm_fb_debugfs_file_create(sub_dir, "xres",
						   (u32 *) &mfd->panel_info.
						   xres);
			msm_fb_debugfs_file_create(sub_dir, "yres",
						   (u32 *) &mfd->panel_info.
						   yres);
			msm_fb_debugfs_file_create(sub_dir, "bpp",
						   (u32 *) &mfd->panel_info.
						   bpp);
			msm_fb_debugfs_file_create(sub_dir, "type",
						   (u32 *) &mfd->panel_info.
						   type);
			msm_fb_debugfs_file_create(sub_dir, "wait_cycle",
						   (u32 *) &mfd->panel_info.
						   wait_cycle);
			msm_fb_debugfs_file_create(sub_dir, "pdest",
						   (u32 *) &mfd->panel_info.
						   pdest);
			msm_fb_debugfs_file_create(sub_dir, "backbuff",
						   (u32 *) &mfd->panel_info.
						   fb_num);
			msm_fb_debugfs_file_create(sub_dir, "clk_rate",
						   (u32 *) &mfd->panel_info.
						   clk_rate);
			msm_fb_debugfs_file_create(sub_dir, "frame_count",
						   (u32 *) &mfd->panel_info.
						   frame_count);


			switch (mfd->dest) {
			case DISPLAY_LCD:
				msm_fb_debugfs_file_create(sub_dir,
				"vsync_enable",
				(u32 *)&mfd->panel_info.lcd.vsync_enable);
				msm_fb_debugfs_file_create(sub_dir,
				"refx100",
				(u32 *) &mfd->panel_info.lcd. refx100);
				msm_fb_debugfs_file_create(sub_dir,
				"v_back_porch",
				(u32 *) &mfd->panel_info.lcd.v_back_porch);
				msm_fb_debugfs_file_create(sub_dir,
				"v_front_porch",
				(u32 *) &mfd->panel_info.lcd.v_front_porch);
				msm_fb_debugfs_file_create(sub_dir,
				"v_pulse_width",
				(u32 *) &mfd->panel_info.lcd.v_pulse_width);
				msm_fb_debugfs_file_create(sub_dir,
				"hw_vsync_mode",
				(u32 *) &mfd->panel_info.lcd.hw_vsync_mode);
				msm_fb_debugfs_file_create(sub_dir,
				"vsync_notifier_period", (u32 *)
				&mfd->panel_info.lcd.vsync_notifier_period);
				break;

			case DISPLAY_LCDC:
				msm_fb_debugfs_file_create(sub_dir,
				"h_back_porch",
				(u32 *) &mfd->panel_info.lcdc.h_back_porch);
				msm_fb_debugfs_file_create(sub_dir,
				"h_front_porch",
				(u32 *) &mfd->panel_info.lcdc.h_front_porch);
				msm_fb_debugfs_file_create(sub_dir,
				"h_pulse_width",
				(u32 *) &mfd->panel_info.lcdc.h_pulse_width);
				msm_fb_debugfs_file_create(sub_dir,
				"v_back_porch",
				(u32 *) &mfd->panel_info.lcdc.v_back_porch);
				msm_fb_debugfs_file_create(sub_dir,
				"v_front_porch",
				(u32 *) &mfd->panel_info.lcdc.v_front_porch);
				msm_fb_debugfs_file_create(sub_dir,
				"v_pulse_width",
				(u32 *) &mfd->panel_info.lcdc.v_pulse_width);
				msm_fb_debugfs_file_create(sub_dir,
				"border_clr",
				(u32 *) &mfd->panel_info.lcdc.border_clr);
				msm_fb_debugfs_file_create(sub_dir,
				"underflow_clr",
				(u32 *) &mfd->panel_info.lcdc.underflow_clr);
				msm_fb_debugfs_file_create(sub_dir,
				"hsync_skew",
				(u32 *) &mfd->panel_info.lcdc.hsync_skew);
				break;

			default:
				break;
			}
		}
	}
#endif /* MSM_FB_ENABLE_DBGFS */

	return ret;
}

static int msm_fb_open(struct fb_info *info, int user)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	int result;

	result = pm_runtime_get_sync(info->dev);

    MSM_FB_DEBUG("msm_fb_open():start mfd->ref_cnt = %d-----\n",mfd->ref_cnt);

	if (result < 0) {
		printk(KERN_ERR "pm_runtime: fail to wake up\n");
	}

#if 0
	if (!mfd->ref_cnt) {
#else /* #if 0 */
    if (FALSE == msm_fb_first_opened) {
        msm_fb_first_opened = TRUE;
#endif /* #if 0 */
		mdp_set_dma_pan_info(info, NULL, TRUE);

        MSM_FB_DEBUG("msm_fb_open():CALL msm_fb_blank_sub-----\n");

		if (msm_fb_blank_sub(FB_BLANK_UNBLANK, info, mfd->op_enable)) {
			printk(KERN_ERR "msm_fb_open: can't turn on display!\n");
			return -1;
		}
        msm_fb_notify_done_open(info);
	}

	mfd->ref_cnt++;

    MSM_FB_DEBUG("msm_fb_open():end mfd->ref_cnt = %d-----\n",mfd->ref_cnt);

	return 0;
}

static int msm_fb_release(struct fb_info *info, int user)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	int ret = 0;

	if (!mfd->ref_cnt) {
		MSM_FB_INFO("msm_fb_release: try to close unopened fb %d!\n",
			    mfd->index);
		return -EINVAL;
	}

	mfd->ref_cnt--;

#if 0

	if (!mfd->ref_cnt) {
		if ((ret =
		     msm_fb_blank_sub(FB_BLANK_POWERDOWN, info,
				      mfd->op_enable)) != 0) {
			printk(KERN_ERR "msm_fb_release: can't turn off display!\n");
			return ret;
		}
	}
#endif /* #if 0 */

	pm_runtime_put(info->dev);
	return ret;
}

DEFINE_SEMAPHORE(msm_fb_pan_sem);

static int msm_fb_pan_display(struct fb_var_screeninfo *var,
			      struct fb_info *info)
{
	struct mdp_dirty_region dirty;
	struct mdp_dirty_region *dirtyPtr = NULL;
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct msm_fb_panel_data *pdata;

	if ((!mfd->op_enable) || (!mfd->panel_power_on))
		return -EPERM;

	if (var->xoffset > (info->var.xres_virtual - info->var.xres))
		return -EINVAL;

	if (var->yoffset > (info->var.yres_virtual - info->var.yres))
		return -EINVAL;

	if (info->fix.xpanstep)
		info->var.xoffset =
		    (var->xoffset / info->fix.xpanstep) * info->fix.xpanstep;

	if (info->fix.ypanstep)
		info->var.yoffset =
		    (var->yoffset / info->fix.ypanstep) * info->fix.ypanstep;

	/* "UPDT" */
	if (var->reserved[0] == 0x54445055) {
		dirty.xoffset = var->reserved[1] & 0xffff;
		dirty.yoffset = (var->reserved[1] >> 16) & 0xffff;

		if ((var->reserved[2] & 0xffff) <= dirty.xoffset)
			return -EINVAL;
		if (((var->reserved[2] >> 16) & 0xffff) <= dirty.yoffset)
			return -EINVAL;

		dirty.width = (var->reserved[2] & 0xffff) - dirty.xoffset;
		dirty.height =
		    ((var->reserved[2] >> 16) & 0xffff) - dirty.yoffset;
		info->var.yoffset = var->yoffset;

		if (dirty.xoffset < 0)
			return -EINVAL;

		if (dirty.yoffset < 0)
			return -EINVAL;

		if ((dirty.xoffset + dirty.width) > info->var.xres)
			return -EINVAL;

		if ((dirty.yoffset + dirty.height) > info->var.yres)
			return -EINVAL;

		if ((dirty.width <= 0) || (dirty.height <= 0))
			return -EINVAL;

		dirtyPtr = &dirty;
	}
	complete(&mfd->msmfb_update_notify);
	mutex_lock(&msm_fb_notify_update_sem);
	if (mfd->msmfb_no_update_notify_timer.function)
		del_timer(&mfd->msmfb_no_update_notify_timer);

	mfd->msmfb_no_update_notify_timer.expires =
				jiffies + ((1000 * HZ) / 1000);
	add_timer(&mfd->msmfb_no_update_notify_timer);
	mutex_unlock(&msm_fb_notify_update_sem);

	down(&msm_fb_pan_sem);
	mdp_set_dma_pan_info(info, dirtyPtr,
			     (var->activate == FB_ACTIVATE_VBL));
	mdp_dma_pan_update(info);
	up(&msm_fb_pan_sem);

	if (unset_bl_level && !bl_updated) {
		pdata = (struct msm_fb_panel_data *)mfd->pdev->
			dev.platform_data;
		if ((pdata) && (pdata->set_backlight)) {
			down(&mfd->sem);
			mfd->bl_level = unset_bl_level;
			pdata->set_backlight(mfd);
			bl_level_old = unset_bl_level;
			up(&mfd->sem);
			bl_updated = 1;
		}
	}

	++mfd->panel_info.frame_count;
	return 0;
}

static int msm_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	if (var->rotate != FB_ROTATE_UR)
		return -EINVAL;
	if (var->grayscale != info->var.grayscale)
		return -EINVAL;

	switch (var->bits_per_pixel) {
	case 16:
		if ((var->green.offset != 5) ||
			!((var->blue.offset == 11)
				|| (var->blue.offset == 0)) ||
			!((var->red.offset == 11)
				|| (var->red.offset == 0)) ||
			(var->blue.length != 5) ||
			(var->green.length != 6) ||
			(var->red.length != 5) ||
			(var->blue.msb_right != 0) ||
			(var->green.msb_right != 0) ||
			(var->red.msb_right != 0) ||
			(var->transp.offset != 0) ||
			(var->transp.length != 0))
				return -EINVAL;
		break;

	case 24:
		if ((var->blue.offset != 0) ||
			(var->green.offset != 8) ||
			(var->red.offset != 16) ||
			(var->blue.length != 8) ||
			(var->green.length != 8) ||
			(var->red.length != 8) ||
			(var->blue.msb_right != 0) ||
			(var->green.msb_right != 0) ||
			(var->red.msb_right != 0) ||
			!(((var->transp.offset == 0) &&
				(var->transp.length == 0)) ||
			  ((var->transp.offset == 24) &&
				(var->transp.length == 8))))
				return -EINVAL;
		break;

	case 32:
		/* Figure out if the user meant RGBA or ARGB
		   and verify the position of the RGB components */

		if (var->transp.offset == 24) {
			if ((var->blue.offset != 0) ||
			    (var->green.offset != 8) ||
			    (var->red.offset != 16))
				return -EINVAL;
		} else if (var->transp.offset == 0) {
			if ((var->blue.offset != 8) ||
			    (var->green.offset != 16) ||
			    (var->red.offset != 24))
				return -EINVAL;
		} else
			return -EINVAL;

		/* Check the common values for both RGBA and ARGB */

		if ((var->blue.length != 8) ||
		    (var->green.length != 8) ||
		    (var->red.length != 8) ||
		    (var->transp.length != 8) ||
		    (var->blue.msb_right != 0) ||
		    (var->green.msb_right != 0) ||
		    (var->red.msb_right != 0))
			return -EINVAL;

		break;

	default:
		return -EINVAL;
	}

	if ((var->xres_virtual <= 0) || (var->yres_virtual <= 0))
		return -EINVAL;

	if (info->fix.smem_len <
		(var->xres_virtual*var->yres_virtual*(var->bits_per_pixel/8)))
		return -EINVAL;

	if ((var->xres == 0) || (var->yres == 0))
		return -EINVAL;

	if ((var->xres > MAX(mfd->panel_info.xres,
			     mfd->panel_info.mode2_xres)) ||
		(var->yres > MAX(mfd->panel_info.yres,
				 mfd->panel_info.mode2_yres)))
		return -EINVAL;

	if (var->xoffset > (var->xres_virtual - var->xres))
		return -EINVAL;

	if (var->yoffset > (var->yres_virtual - var->yres))
		return -EINVAL;

	return 0;
}

static int msm_fb_set_par(struct fb_info *info)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct fb_var_screeninfo *var = &info->var;
	int old_imgType;
	int blank = 0;

	old_imgType = mfd->fb_imgType;
	switch (var->bits_per_pixel) {
	case 16:
		if (var->red.offset == 0)
			mfd->fb_imgType = MDP_BGR_565;
		else
			mfd->fb_imgType = MDP_RGB_565;
		break;

	case 24:
		if ((var->transp.offset == 0) && (var->transp.length == 0))
			mfd->fb_imgType = MDP_RGB_888;
		else if ((var->transp.offset == 24) &&
				(var->transp.length == 8)) {
			mfd->fb_imgType = MDP_ARGB_8888;
			info->var.bits_per_pixel = 32;
		}
		break;

	case 32:
		if (var->transp.offset == 24)
			mfd->fb_imgType = MDP_ARGB_8888;
		else
			mfd->fb_imgType = MDP_RGBA_8888;
		break;

	default:
		return -EINVAL;
	}

	if ((mfd->var_pixclock != var->pixclock) ||
		(mfd->hw_refresh && ((mfd->fb_imgType != old_imgType) ||
				(mfd->var_pixclock != var->pixclock) ||
				(mfd->var_xres != var->xres) ||
				(mfd->var_yres != var->yres)))) {
		mfd->var_xres = var->xres;
		mfd->var_yres = var->yres;
		mfd->var_pixclock = var->pixclock;
		blank = 1;
	}
	mfd->fbi->fix.line_length = msm_fb_line_length(mfd->index, var->xres,
						       var->bits_per_pixel/8);

	if (blank) {
		msm_fb_blank_sub(FB_BLANK_POWERDOWN, info, mfd->op_enable);
		msm_fb_blank_sub(FB_BLANK_UNBLANK, info, mfd->op_enable);
	}

	return 0;
}

static int msm_fb_stop_sw_refresher(struct msm_fb_data_type *mfd)
{
	if (mfd->hw_refresh)
		return -EPERM;

	if (mfd->sw_currently_refreshing) {
		down(&mfd->sem);
		mfd->sw_currently_refreshing = FALSE;
		up(&mfd->sem);

		/* wait until the refresher finishes the last job */
		wait_for_completion_killable(&mfd->refresher_comp);
	}

	return 0;
}

int msm_fb_resume_sw_refresher(struct msm_fb_data_type *mfd)
{
	boolean do_refresh;

	if (mfd->hw_refresh)
		return -EPERM;

	down(&mfd->sem);
	if ((!mfd->sw_currently_refreshing) && (mfd->sw_refreshing_enable)) {
		do_refresh = TRUE;
		mfd->sw_currently_refreshing = TRUE;
	} else {
		do_refresh = FALSE;
	}
	up(&mfd->sem);

	if (do_refresh)
		mdp_refresh_screen((unsigned long)mfd);

	return 0;
}

#if defined CONFIG_FB_MSM_MDP31
static int mdp_blit_split_height(struct fb_info *info,
				struct mdp_blit_req *req)
{
	int ret;
	struct mdp_blit_req splitreq;
	int s_x_0, s_x_1, s_w_0, s_w_1, s_y_0, s_y_1, s_h_0, s_h_1;
	int d_x_0, d_x_1, d_w_0, d_w_1, d_y_0, d_y_1, d_h_0, d_h_1;

	splitreq = *req;
	/* break dest roi at height*/
	d_x_0 = d_x_1 = req->dst_rect.x;
	d_w_0 = d_w_1 = req->dst_rect.w;
	d_y_0 = req->dst_rect.y;
	if (req->dst_rect.h % 32 == 3)
		d_h_1 = (req->dst_rect.h - 3) / 2 - 1;
	else if (req->dst_rect.h % 32 == 2)
		d_h_1 = (req->dst_rect.h - 2) / 2 - 6;
	else
		d_h_1 = (req->dst_rect.h - 1) / 2 - 1;
	d_h_0 = req->dst_rect.h - d_h_1;
	d_y_1 = d_y_0 + d_h_0;
	if (req->dst_rect.h == 3) {
		d_h_1 = 2;
		d_h_0 = 2;
		d_y_1 = d_y_0 + 1;
	}

	/* blit first region */
	if (((splitreq.flags & 0x07) == 0x04) ||
		((splitreq.flags & 0x07) == 0x0)) {

		if (splitreq.flags & MDP_ROT_90) {
			s_y_0 = s_y_1 = req->src_rect.y;
			s_h_0 = s_h_1 = req->src_rect.h;
			s_x_0 = req->src_rect.x;
			s_w_1 = (req->src_rect.w * d_h_1) / req->dst_rect.h;
			s_w_0 = req->src_rect.w - s_w_1;
			s_x_1 = s_x_0 + s_w_0;
			if (d_h_1 >= 8 * s_w_1) {
				s_w_1++;
				s_x_1--;
			}
		} else {
			s_x_0 = s_x_1 = req->src_rect.x;
			s_w_0 = s_w_1 = req->src_rect.w;
			s_y_0 = req->src_rect.y;
			s_h_1 = (req->src_rect.h * d_h_1) / req->dst_rect.h;
			s_h_0 = req->src_rect.h - s_h_1;
			s_y_1 = s_y_0 + s_h_0;
			if (d_h_1 >= 8 * s_h_1) {
				s_h_1++;
				s_y_1--;
			}
		}

		splitreq.src_rect.h = s_h_0;
		splitreq.src_rect.y = s_y_0;
		splitreq.dst_rect.h = d_h_0;
		splitreq.dst_rect.y = d_y_0;
		splitreq.src_rect.x = s_x_0;
		splitreq.src_rect.w = s_w_0;
		splitreq.dst_rect.x = d_x_0;
		splitreq.dst_rect.w = d_w_0;
	} else {

		if (splitreq.flags & MDP_ROT_90) {
			s_y_0 = s_y_1 = req->src_rect.y;
			s_h_0 = s_h_1 = req->src_rect.h;
			s_x_0 = req->src_rect.x;
			s_w_1 = (req->src_rect.w * d_h_0) / req->dst_rect.h;
			s_w_0 = req->src_rect.w - s_w_1;
			s_x_1 = s_x_0 + s_w_0;
			if (d_h_0 >= 8 * s_w_1) {
				s_w_1++;
				s_x_1--;
			}
		} else {
			s_x_0 = s_x_1 = req->src_rect.x;
			s_w_0 = s_w_1 = req->src_rect.w;
			s_y_0 = req->src_rect.y;
			s_h_1 = (req->src_rect.h * d_h_0) / req->dst_rect.h;
			s_h_0 = req->src_rect.h - s_h_1;
			s_y_1 = s_y_0 + s_h_0;
			if (d_h_0 >= 8 * s_h_1) {
				s_h_1++;
				s_y_1--;
			}
		}
		splitreq.src_rect.h = s_h_0;
		splitreq.src_rect.y = s_y_0;
		splitreq.dst_rect.h = d_h_1;
		splitreq.dst_rect.y = d_y_1;
		splitreq.src_rect.x = s_x_0;
		splitreq.src_rect.w = s_w_0;
		splitreq.dst_rect.x = d_x_1;
		splitreq.dst_rect.w = d_w_1;
	}
	ret = mdp_ppp_blit(info, &splitreq);
	if (ret)
		return ret;

	/* blit second region */
	if (((splitreq.flags & 0x07) == 0x04) ||
		((splitreq.flags & 0x07) == 0x0)) {
		splitreq.src_rect.h = s_h_1;
		splitreq.src_rect.y = s_y_1;
		splitreq.dst_rect.h = d_h_1;
		splitreq.dst_rect.y = d_y_1;
		splitreq.src_rect.x = s_x_1;
		splitreq.src_rect.w = s_w_1;
		splitreq.dst_rect.x = d_x_1;
		splitreq.dst_rect.w = d_w_1;
	} else {
		splitreq.src_rect.h = s_h_1;
		splitreq.src_rect.y = s_y_1;
		splitreq.dst_rect.h = d_h_0;
		splitreq.dst_rect.y = d_y_0;
		splitreq.src_rect.x = s_x_1;
		splitreq.src_rect.w = s_w_1;
		splitreq.dst_rect.x = d_x_0;
		splitreq.dst_rect.w = d_w_0;
	}
	ret = mdp_ppp_blit(info, &splitreq);
	return ret;
}
#endif

int mdp_blit(struct fb_info *info, struct mdp_blit_req *req)
{
	int ret;
#if defined CONFIG_FB_MSM_MDP31 || defined CONFIG_FB_MSM_MDP30
	unsigned int remainder = 0, is_bpp_4 = 0;
	struct mdp_blit_req splitreq;
	int s_x_0, s_x_1, s_w_0, s_w_1, s_y_0, s_y_1, s_h_0, s_h_1;
	int d_x_0, d_x_1, d_w_0, d_w_1, d_y_0, d_y_1, d_h_0, d_h_1;

	if (req->flags & MDP_ROT_90) {
		if (((req->dst_rect.h == 1) && ((req->src_rect.w != 1) ||
			(req->dst_rect.w != req->src_rect.h))) ||
			((req->dst_rect.w == 1) && ((req->src_rect.h != 1) ||
			(req->dst_rect.h != req->src_rect.w)))) {
			printk(KERN_ERR "mpd_ppp: error scaling when size is 1!\n");
			return -EINVAL;
		}
	} else {
		if (((req->dst_rect.w == 1) && ((req->src_rect.w != 1) ||
			(req->dst_rect.h != req->src_rect.h))) ||
			((req->dst_rect.h == 1) && ((req->src_rect.h != 1) ||
			(req->dst_rect.w != req->src_rect.w)))) {
			printk(KERN_ERR "mpd_ppp: error scaling when size is 1!\n");
			return -EINVAL;
		}
	}
#endif
	if (unlikely(req->src_rect.h == 0 || req->src_rect.w == 0)) {
		printk(KERN_ERR "mpd_ppp: src img of zero size!\n");
		return -EINVAL;
	}
	if (unlikely(req->dst_rect.h == 0 || req->dst_rect.w == 0))
		return 0;

#if defined CONFIG_FB_MSM_MDP31
	/* MDP width split workaround */
	remainder = (req->dst_rect.w)%32;
	ret = mdp_get_bytes_per_pixel(req->dst.format,
					(struct msm_fb_data_type *)info->par);
	if (ret <= 0) {
		printk(KERN_ERR "mdp_ppp: incorrect bpp!\n");
		return -EINVAL;
	}
	is_bpp_4 = (ret == 4) ? 1 : 0;

	if ((is_bpp_4 && (remainder == 6 || remainder == 14 ||
	remainder == 22 || remainder == 30)) || remainder == 3 ||
	(remainder == 1 && req->dst_rect.w != 1) ||
	(remainder == 2 && req->dst_rect.w != 2)) {
		/* make new request as provide by user */
		splitreq = *req;

		/* break dest roi at width*/
		d_y_0 = d_y_1 = req->dst_rect.y;
		d_h_0 = d_h_1 = req->dst_rect.h;
		d_x_0 = req->dst_rect.x;

		if (remainder == 14)
			d_w_1 = (req->dst_rect.w - 14) / 2 + 4;
		else if (remainder == 22)
			d_w_1 = (req->dst_rect.w - 22) / 2 + 10;
		else if (remainder == 30)
			d_w_1 = (req->dst_rect.w - 30) / 2 + 10;
		else if (remainder == 6)
			d_w_1 = req->dst_rect.w / 2 - 1;
		else if (remainder == 3)
			d_w_1 = (req->dst_rect.w - 3) / 2 - 1;
		else if (remainder == 2)
			d_w_1 = (req->dst_rect.w - 2) / 2 - 6;
		else
			d_w_1 = (req->dst_rect.w - 1) / 2 - 1;
		d_w_0 = req->dst_rect.w - d_w_1;
		d_x_1 = d_x_0 + d_w_0;
		if (req->dst_rect.w == 3) {
			d_w_1 = 2;
			d_w_0 = 2;
			d_x_1 = d_x_0 + 1;
		}

		/* blit first region */
		if (((splitreq.flags & 0x07) == 0x07) ||
			((splitreq.flags & 0x07) == 0x0)) {

			if (splitreq.flags & MDP_ROT_90) {
				s_x_0 = s_x_1 = req->src_rect.x;
				s_w_0 = s_w_1 = req->src_rect.w;
				s_y_0 = req->src_rect.y;
				s_h_1 = (req->src_rect.h * d_w_1) /
					req->dst_rect.w;
				s_h_0 = req->src_rect.h - s_h_1;
				s_y_1 = s_y_0 + s_h_0;
				if (d_w_1 >= 8 * s_h_1) {
					s_h_1++;
					s_y_1--;
				}
			} else {
				s_y_0 = s_y_1 = req->src_rect.y;
				s_h_0 = s_h_1 = req->src_rect.h;
				s_x_0 = req->src_rect.x;
				s_w_1 = (req->src_rect.w * d_w_1) /
					req->dst_rect.w;
				s_w_0 = req->src_rect.w - s_w_1;
				s_x_1 = s_x_0 + s_w_0;
				if (d_w_1 >= 8 * s_w_1) {
					s_w_1++;
					s_x_1--;
				}
			}

			splitreq.src_rect.h = s_h_0;
			splitreq.src_rect.y = s_y_0;
			splitreq.dst_rect.h = d_h_0;
			splitreq.dst_rect.y = d_y_0;
			splitreq.src_rect.x = s_x_0;
			splitreq.src_rect.w = s_w_0;
			splitreq.dst_rect.x = d_x_0;
			splitreq.dst_rect.w = d_w_0;
		} else {
			if (splitreq.flags & MDP_ROT_90) {
				s_x_0 = s_x_1 = req->src_rect.x;
				s_w_0 = s_w_1 = req->src_rect.w;
				s_y_0 = req->src_rect.y;
				s_h_1 = (req->src_rect.h * d_w_0) /
					req->dst_rect.w;
				s_h_0 = req->src_rect.h - s_h_1;
				s_y_1 = s_y_0 + s_h_0;
				if (d_w_0 >= 8 * s_h_1) {
					s_h_1++;
					s_y_1--;
				}
			} else {
				s_y_0 = s_y_1 = req->src_rect.y;
				s_h_0 = s_h_1 = req->src_rect.h;
				s_x_0 = req->src_rect.x;
				s_w_1 = (req->src_rect.w * d_w_0) /
					req->dst_rect.w;
				s_w_0 = req->src_rect.w - s_w_1;
				s_x_1 = s_x_0 + s_w_0;
				if (d_w_0 >= 8 * s_w_1) {
					s_w_1++;
					s_x_1--;
				}
			}
			splitreq.src_rect.h = s_h_0;
			splitreq.src_rect.y = s_y_0;
			splitreq.dst_rect.h = d_h_1;
			splitreq.dst_rect.y = d_y_1;
			splitreq.src_rect.x = s_x_0;
			splitreq.src_rect.w = s_w_0;
			splitreq.dst_rect.x = d_x_1;
			splitreq.dst_rect.w = d_w_1;
		}

		if ((splitreq.dst_rect.h % 32 == 3) ||
			((req->dst_rect.h % 32) == 1 && req->dst_rect.h != 1) ||
			((req->dst_rect.h % 32) == 2 && req->dst_rect.h != 2))
			ret = mdp_blit_split_height(info, &splitreq);
		else
			ret = mdp_ppp_blit(info, &splitreq);
		if (ret)
			return ret;
		/* blit second region */
		if (((splitreq.flags & 0x07) == 0x07) ||
			((splitreq.flags & 0x07) == 0x0)) {
			splitreq.src_rect.h = s_h_1;
			splitreq.src_rect.y = s_y_1;
			splitreq.dst_rect.h = d_h_1;
			splitreq.dst_rect.y = d_y_1;
			splitreq.src_rect.x = s_x_1;
			splitreq.src_rect.w = s_w_1;
			splitreq.dst_rect.x = d_x_1;
			splitreq.dst_rect.w = d_w_1;
		} else {
			splitreq.src_rect.h = s_h_1;
			splitreq.src_rect.y = s_y_1;
			splitreq.dst_rect.h = d_h_0;
			splitreq.dst_rect.y = d_y_0;
			splitreq.src_rect.x = s_x_1;
			splitreq.src_rect.w = s_w_1;
			splitreq.dst_rect.x = d_x_0;
			splitreq.dst_rect.w = d_w_0;
		}
		if (((splitreq.dst_rect.h % 32) == 3) ||
			((req->dst_rect.h % 32) == 1 && req->dst_rect.h != 1) ||
			((req->dst_rect.h % 32) == 2 && req->dst_rect.h != 2))
			ret = mdp_blit_split_height(info, &splitreq);
		else
			ret = mdp_ppp_blit(info, &splitreq);
		if (ret)
			return ret;
	} else if ((req->dst_rect.h % 32) == 3 ||
		((req->dst_rect.h % 32) == 1 && req->dst_rect.h != 1) ||
		((req->dst_rect.h % 32) == 2 && req->dst_rect.h != 2))
		ret = mdp_blit_split_height(info, req);
	else
		ret = mdp_ppp_blit(info, req);
	return ret;
#elif defined CONFIG_FB_MSM_MDP30
	/* MDP width split workaround */
	remainder = (req->dst_rect.w)%16;
	ret = mdp_get_bytes_per_pixel(req->dst.format,
					(struct msm_fb_data_type *)info->par);
	if (ret <= 0) {
		printk(KERN_ERR "mdp_ppp: incorrect bpp!\n");
		return -EINVAL;
	}
	is_bpp_4 = (ret == 4) ? 1 : 0;

	if ((is_bpp_4 && (remainder == 6 || remainder == 14))) {

		/* make new request as provide by user */
		splitreq = *req;

		/* break dest roi at width*/
		d_y_0 = d_y_1 = req->dst_rect.y;
		d_h_0 = d_h_1 = req->dst_rect.h;
		d_x_0 = req->dst_rect.x;

		if (remainder == 14 || remainder == 6)
			d_w_1 = req->dst_rect.w / 2;
		else
			d_w_1 = (req->dst_rect.w - 1) / 2 - 1;

		d_w_0 = req->dst_rect.w - d_w_1;
		d_x_1 = d_x_0 + d_w_0;

		/* blit first region */
		if (((splitreq.flags & 0x07) == 0x07) ||
			((splitreq.flags & 0x07) == 0x0)) {

			if (splitreq.flags & MDP_ROT_90) {
				s_x_0 = s_x_1 = req->src_rect.x;
				s_w_0 = s_w_1 = req->src_rect.w;
				s_y_0 = req->src_rect.y;
				s_h_1 = (req->src_rect.h * d_w_1) /
					req->dst_rect.w;
				s_h_0 = req->src_rect.h - s_h_1;
				s_y_1 = s_y_0 + s_h_0;
				if (d_w_1 >= 8 * s_h_1) {
					s_h_1++;
					s_y_1--;
				}
			} else {
				s_y_0 = s_y_1 = req->src_rect.y;
				s_h_0 = s_h_1 = req->src_rect.h;
				s_x_0 = req->src_rect.x;
				s_w_1 = (req->src_rect.w * d_w_1) /
					req->dst_rect.w;
				s_w_0 = req->src_rect.w - s_w_1;
				s_x_1 = s_x_0 + s_w_0;
				if (d_w_1 >= 8 * s_w_1) {
					s_w_1++;
					s_x_1--;
				}
			}

			splitreq.src_rect.h = s_h_0;
			splitreq.src_rect.y = s_y_0;
			splitreq.dst_rect.h = d_h_0;
			splitreq.dst_rect.y = d_y_0;
			splitreq.src_rect.x = s_x_0;
			splitreq.src_rect.w = s_w_0;
			splitreq.dst_rect.x = d_x_0;
			splitreq.dst_rect.w = d_w_0;
		} else {
			if (splitreq.flags & MDP_ROT_90) {
				s_x_0 = s_x_1 = req->src_rect.x;
				s_w_0 = s_w_1 = req->src_rect.w;
				s_y_0 = req->src_rect.y;
				s_h_1 = (req->src_rect.h * d_w_0) /
					req->dst_rect.w;
				s_h_0 = req->src_rect.h - s_h_1;
				s_y_1 = s_y_0 + s_h_0;
				if (d_w_0 >= 8 * s_h_1) {
					s_h_1++;
					s_y_1--;
				}
			} else {
				s_y_0 = s_y_1 = req->src_rect.y;
				s_h_0 = s_h_1 = req->src_rect.h;
				s_x_0 = req->src_rect.x;
				s_w_1 = (req->src_rect.w * d_w_0) /
					req->dst_rect.w;
				s_w_0 = req->src_rect.w - s_w_1;
				s_x_1 = s_x_0 + s_w_0;
				if (d_w_0 >= 8 * s_w_1) {
					s_w_1++;
					s_x_1--;
				}
			}
			splitreq.src_rect.h = s_h_0;
			splitreq.src_rect.y = s_y_0;
			splitreq.dst_rect.h = d_h_1;
			splitreq.dst_rect.y = d_y_1;
			splitreq.src_rect.x = s_x_0;
			splitreq.src_rect.w = s_w_0;
			splitreq.dst_rect.x = d_x_1;
			splitreq.dst_rect.w = d_w_1;
		}

		/* No need to split in height */
		ret = mdp_ppp_blit(info, &splitreq);

		if (ret)
			return ret;

		/* blit second region */
		if (((splitreq.flags & 0x07) == 0x07) ||
			((splitreq.flags & 0x07) == 0x0)) {
			splitreq.src_rect.h = s_h_1;
			splitreq.src_rect.y = s_y_1;
			splitreq.dst_rect.h = d_h_1;
			splitreq.dst_rect.y = d_y_1;
			splitreq.src_rect.x = s_x_1;
			splitreq.src_rect.w = s_w_1;
			splitreq.dst_rect.x = d_x_1;
			splitreq.dst_rect.w = d_w_1;
		} else {
			splitreq.src_rect.h = s_h_1;
			splitreq.src_rect.y = s_y_1;
			splitreq.dst_rect.h = d_h_0;
			splitreq.dst_rect.y = d_y_0;
			splitreq.src_rect.x = s_x_1;
			splitreq.src_rect.w = s_w_1;
			splitreq.dst_rect.x = d_x_0;
			splitreq.dst_rect.w = d_w_0;
		}

		/* No need to split in height ... just width */
		ret = mdp_ppp_blit(info, &splitreq);

		if (ret)
			return ret;

	} else
		ret = mdp_ppp_blit(info, req);
	return ret;
#else
	ret = mdp_ppp_blit(info, req);
	return ret;
#endif
}

typedef void (*msm_dma_barrier_function_pointer) (void *, size_t);

static inline void msm_fb_dma_barrier_for_rect(struct fb_info *info,
			struct mdp_img *img, struct mdp_rect *rect,
			msm_dma_barrier_function_pointer dma_barrier_fp
			)
{
	/*
	 * Compute the start and end addresses of the rectangles.
	 * NOTE: As currently implemented, the data between
	 *       the end of one row and the start of the next is
	 *       included in the address range rather than
	 *       doing multiple calls for each row.
	 */
	unsigned long start;
	size_t size;
	char * const pmem_start = info->screen_base;
	int bytes_per_pixel = mdp_get_bytes_per_pixel(img->format,
					(struct msm_fb_data_type *)info->par);
	if (bytes_per_pixel <= 0) {
		printk(KERN_ERR "%s incorrect bpp!\n", __func__);
		return;
	}
	start = (unsigned long)pmem_start + img->offset +
		(img->width * rect->y + rect->x) * bytes_per_pixel;
	size  = (rect->h * img->width + rect->w) * bytes_per_pixel;
	(*dma_barrier_fp) ((void *) start, size);

}

static inline void msm_dma_nc_pre(void)
{
	dmb();
}
static inline void msm_dma_wt_pre(void)
{
	dmb();
}
static inline void msm_dma_todevice_wb_pre(void *start, size_t size)
{
	dma_cache_pre_ops(start, size, DMA_TO_DEVICE);
}

static inline void msm_dma_fromdevice_wb_pre(void *start, size_t size)
{
	dma_cache_pre_ops(start, size, DMA_FROM_DEVICE);
}

static inline void msm_dma_nc_post(void)
{
	dmb();
}

static inline void msm_dma_fromdevice_wt_post(void *start, size_t size)
{
	dma_cache_post_ops(start, size, DMA_FROM_DEVICE);
}

static inline void msm_dma_todevice_wb_post(void *start, size_t size)
{
	dma_cache_post_ops(start, size, DMA_TO_DEVICE);
}

static inline void msm_dma_fromdevice_wb_post(void *start, size_t size)
{
	dma_cache_post_ops(start, size, DMA_FROM_DEVICE);
}

/*
 * Do the write barriers required to guarantee data is committed to RAM
 * (from CPU cache or internal buffers) before a DMA operation starts.
 * NOTE: As currently implemented, the data between
 *       the end of one row and the start of the next is
 *       included in the address range rather than
 *       doing multiple calls for each row.
*/
static void msm_fb_ensure_memory_coherency_before_dma(struct fb_info *info,
		struct mdp_blit_req *req_list,
		int req_list_count)
{
#ifdef CONFIG_ARCH_QSD8X50
	int i;

	/*
	 * Normally, do the requested barriers for each address
	 * range that corresponds to a rectangle.
	 *
	 * But if at least one write barrier is requested for data
	 * going to or from the device but no address range is
	 * needed for that barrier, then do the barrier, but do it
	 * only once, no matter how many requests there are.
	 */
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	switch (mfd->mdp_fb_page_protection)	{
	default:
	case MDP_FB_PAGE_PROTECTION_NONCACHED:
	case MDP_FB_PAGE_PROTECTION_WRITECOMBINE:
		/*
		 * The following barrier is only done at most once,
		 * since further calls would be redundant.
		 */
		for (i = 0; i < req_list_count; i++) {
			if (!(req_list[i].flags
				& MDP_NO_DMA_BARRIER_START)) {
				msm_dma_nc_pre();
				break;
			}
		}
		break;

	case MDP_FB_PAGE_PROTECTION_WRITETHROUGHCACHE:
		/*
		 * The following barrier is only done at most once,
		 * since further calls would be redundant.
		 */
		for (i = 0; i < req_list_count; i++) {
			if (!(req_list[i].flags
				& MDP_NO_DMA_BARRIER_START)) {
				msm_dma_wt_pre();
				break;
			}
		}
		break;

	case MDP_FB_PAGE_PROTECTION_WRITEBACKCACHE:
	case MDP_FB_PAGE_PROTECTION_WRITEBACKWACACHE:
		for (i = 0; i < req_list_count; i++) {
			if (!(req_list[i].flags &
					MDP_NO_DMA_BARRIER_START)) {

				msm_fb_dma_barrier_for_rect(info,
						&(req_list[i].src),
						&(req_list[i].src_rect),
						msm_dma_todevice_wb_pre
						);

				msm_fb_dma_barrier_for_rect(info,
						&(req_list[i].dst),
						&(req_list[i].dst_rect),
						msm_dma_todevice_wb_pre
						);
			}
		}
		break;
	}
#else
	dmb();
#endif
}


/*
 * Do the write barriers required to guarantee data will be re-read from RAM by
 * the CPU after a DMA operation ends.
 * NOTE: As currently implemented, the data between
 *       the end of one row and the start of the next is
 *       included in the address range rather than
 *       doing multiple calls for each row.
*/
static void msm_fb_ensure_memory_coherency_after_dma(struct fb_info *info,
		struct mdp_blit_req *req_list,
		int req_list_count)
{
#ifdef CONFIG_ARCH_QSD8X50
	int i;

	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	switch (mfd->mdp_fb_page_protection)	{
	default:
	case MDP_FB_PAGE_PROTECTION_NONCACHED:
	case MDP_FB_PAGE_PROTECTION_WRITECOMBINE:
		/*
		 * The following barrier is only done at most once,
		 * since further calls would be redundant.
		 */
		for (i = 0; i < req_list_count; i++) {
			if (!(req_list[i].flags
				& MDP_NO_DMA_BARRIER_END)) {
				msm_dma_nc_post();
				break;
			}
		}
		break;

	case MDP_FB_PAGE_PROTECTION_WRITETHROUGHCACHE:
		for (i = 0; i < req_list_count; i++) {
			if (!(req_list[i].flags &
					MDP_NO_DMA_BARRIER_END)) {

				msm_fb_dma_barrier_for_rect(info,
						&(req_list[i].dst),
						&(req_list[i].dst_rect),
						msm_dma_fromdevice_wt_post
						);
			}
		}
		break;
	case MDP_FB_PAGE_PROTECTION_WRITEBACKCACHE:
	case MDP_FB_PAGE_PROTECTION_WRITEBACKWACACHE:
		for (i = 0; i < req_list_count; i++) {
			if (!(req_list[i].flags &
					MDP_NO_DMA_BARRIER_END)) {

				msm_fb_dma_barrier_for_rect(info,
						&(req_list[i].dst),
						&(req_list[i].dst_rect),
						msm_dma_fromdevice_wb_post
						);
			}
		}
		break;
	}
#else
	dmb();
#endif
}

/*
 * NOTE: The userspace issues blit operations in a sequence, the sequence
 * start with a operation marked START and ends in an operation marked
 * END. It is guranteed by the userspace that all the blit operations
 * between START and END are only within the regions of areas designated
 * by the START and END operations and that the userspace doesnt modify
 * those areas. Hence it would be enough to perform barrier/cache operations
 * only on the START and END operations.
 */
static int msmfb_blit(struct fb_info *info, void __user *p)
{
	/*
	 * CAUTION: The names of the struct types intentionally *DON'T* match
	 * the names of the variables declared -- they appear to be swapped.
	 * Read the code carefully and you should see that the variable names
	 * make sense.
	 */
	const int MAX_LIST_WINDOW = 16;
	struct mdp_blit_req req_list[MAX_LIST_WINDOW];
	struct mdp_blit_req_list req_list_header;

	int count, i, req_list_count;

	/* Get the count size for the total BLIT request. */
	if (copy_from_user(&req_list_header, p, sizeof(req_list_header)))
		return -EFAULT;
	p += sizeof(req_list_header);
	count = req_list_header.count;
	if (count < 0 || count >= MAX_BLIT_REQ)
		return -EINVAL;
	while (count > 0) {
		/*
		 * Access the requests through a narrow window to decrease copy
		 * overhead and make larger requests accessible to the
		 * coherency management code.
		 * NOTE: The window size is intended to be larger than the
		 *       typical request size, but not require more than 2
		 *       kbytes of stack storage.
		 */
		req_list_count = count;
		if (req_list_count > MAX_LIST_WINDOW)
			req_list_count = MAX_LIST_WINDOW;
		if (copy_from_user(&req_list, p,
				sizeof(struct mdp_blit_req)*req_list_count))
			return -EFAULT;

		/*
		 * Ensure that any data CPU may have previously written to
		 * internal state (but not yet committed to memory) is
		 * guaranteed to be committed to memory now.
		 */
		msm_fb_ensure_memory_coherency_before_dma(info,
				req_list, req_list_count);

		/*
		 * Do the blit DMA, if required -- returning early only if
		 * there is a failure.
		 */
		for (i = 0; i < req_list_count; i++) {
			if (!(req_list[i].flags & MDP_NO_BLIT)) {
				/* Do the actual blit. */
				int ret = mdp_blit(info, &(req_list[i]));

				/*
				 * Note that early returns don't guarantee
				 * memory coherency.
				 */
				if (ret)
					return ret;
			}
		}

		/*
		 * Ensure that CPU cache and other internal CPU state is
		 * updated to reflect any change in memory modified by MDP blit
		 * DMA.
		 */
		msm_fb_ensure_memory_coherency_after_dma(info,
				req_list,
				req_list_count);

		/* Go to next window of requests. */
		count -= req_list_count;
		p += sizeof(struct mdp_blit_req)*req_list_count;
	}
	return 0;
}

#ifdef CONFIG_FB_MSM_OVERLAY
static int msmfb_overlay_get(struct fb_info *info, void __user *p)
{
	struct mdp_overlay req;
	int ret;

	if (copy_from_user(&req, p, sizeof(req)))
		return -EFAULT;

	ret = mdp4_overlay_get(info, &req);
	if (ret) {
		printk(KERN_ERR "%s: ioctl failed \n",
			__func__);
		return ret;
	}
	if (copy_to_user(p, &req, sizeof(req))) {
		printk(KERN_ERR "%s: copy2user failed \n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int msmfb_overlay_set(struct fb_info *info, void __user *p)
{
	struct mdp_overlay req;
	int ret;

	if (copy_from_user(&req, p, sizeof(req)))
		return -EFAULT;

	ret = mdp4_overlay_set(info, &req);
	if (ret) {
		printk(KERN_ERR "%s: ioctl failed, rc=%d\n",
			__func__, ret);
		return ret;
	}

	if (copy_to_user(p, &req, sizeof(req))) {
		printk(KERN_ERR "%s: copy2user failed \n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int msmfb_overlay_unset(struct fb_info *info, unsigned long *argp)
{
	int	ret, ndx;

	ret = copy_from_user(&ndx, argp, sizeof(ndx));
	if (ret) {
		printk(KERN_ERR "%s:msmfb_overlay_unset ioctl failed \n",
			__func__);
		return ret;
	}

	return mdp4_overlay_unset(info, ndx);
}

static int msmfb_overlay_play_wait(struct fb_info *info, unsigned long *argp)
{
	int ret;
	struct msmfb_overlay_data req;
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	if (mfd->overlay_play_enable == 0)      /* nothing to do */
		return 0;

	ret = copy_from_user(&req, argp, sizeof(req));
	if (ret) {
		pr_err("%s:msmfb_overlay_wait ioctl failed", __func__);
		return ret;
	}

	ret = mdp4_overlay_play_wait(info, &req);

	return ret;
}

static int msmfb_overlay_play(struct fb_info *info, unsigned long *argp)
{
	int	ret;
	struct msmfb_overlay_data req;
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct msm_fb_panel_data *pdata;

	if (mfd->overlay_play_enable == 0)	/* nothing to do */
		return 0;

	ret = copy_from_user(&req, argp, sizeof(req));
	if (ret) {
		printk(KERN_ERR "%s:msmfb_overlay_play ioctl failed \n",
			__func__);
		return ret;
	}

	complete(&mfd->msmfb_update_notify);
	mutex_lock(&msm_fb_notify_update_sem);
	if (mfd->msmfb_no_update_notify_timer.function)
		del_timer(&mfd->msmfb_no_update_notify_timer);

	mfd->msmfb_no_update_notify_timer.expires =
				jiffies + ((1000 * HZ) / 1000);
	add_timer(&mfd->msmfb_no_update_notify_timer);
	mutex_unlock(&msm_fb_notify_update_sem);

	ret = mdp4_overlay_play(info, &req);

	if (unset_bl_level && !bl_updated) {
		pdata = (struct msm_fb_panel_data *)mfd->pdev->
			dev.platform_data;
		if ((pdata) && (pdata->set_backlight)) {
			down(&mfd->sem);
			mfd->bl_level = unset_bl_level;
			pdata->set_backlight(mfd);
			bl_level_old = unset_bl_level;
			up(&mfd->sem);
			bl_updated = 1;
		}
	}

	return ret;
}

static int msmfb_overlay_play_enable(struct fb_info *info, unsigned long *argp)
{
	int	ret, enable;
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	ret = copy_from_user(&enable, argp, sizeof(enable));
	if (ret) {
		printk(KERN_ERR "%s:msmfb_overlay_play_enable ioctl failed \n",
			__func__);
		return ret;
	}

	mfd->overlay_play_enable = enable;

	return 0;
}

static int msmfb_overlay_blt(struct fb_info *info, unsigned long *argp)
{
	int     ret;
	struct msmfb_overlay_blt req;

	ret = copy_from_user(&req, argp, sizeof(req));
	if (ret) {
		pr_err("%s: failed\n", __func__);
		return ret;
	}

	ret = mdp4_overlay_blt(info, &req);

	return ret;
}

static int msmfb_overlay_blt_off(struct fb_info *info, unsigned long *argp)
{
	int	ret;
	struct msmfb_overlay_blt req;

	ret = copy_from_user(&req, argp, sizeof(req));
	if (ret) {
		pr_err("%s: failed\n", __func__);
		return ret;
	}

	ret = mdp4_overlay_blt_offset(info, &req);

	ret = copy_to_user(argp, &req, sizeof(req));
	if (ret)
		printk(KERN_ERR "%s:msmfb_overlay_blt_off ioctl failed\n",
		__func__);

	return ret;
}

#ifdef CONFIG_FB_MSM_WRITEBACK_MSM_PANEL
static int msmfb_overlay_ioctl_writeback_init(struct fb_info *info)
{
	return mdp4_writeback_init(info);
}
static int msmfb_overlay_ioctl_writeback_start(
		struct fb_info *info)
{
	int ret = 0;
	ret = mdp4_writeback_start(info);
	if (ret)
		goto error;
error:
	if (ret)
		pr_err("%s:msmfb_writeback_start "
				" ioctl failed\n", __func__);
	return ret;
}

static int msmfb_overlay_ioctl_writeback_stop(
		struct fb_info *info)
{
	int ret = 0;
	ret = mdp4_writeback_stop(info);
	if (ret)
		goto error;

error:
	if (ret)
		pr_err("%s:msmfb_writeback_stop ioctl failed\n",
				__func__);
	return ret;
}

static int msmfb_overlay_ioctl_writeback_queue_buffer(
		struct fb_info *info, unsigned long *argp)
{
	int ret = 0;
	struct msmfb_data data;

	ret = copy_from_user(&data, argp, sizeof(data));
	if (ret)
		goto error;

	ret = mdp4_writeback_queue_buffer(info, &data);
	if (ret)
		goto error;

error:
	if (ret)
		pr_err("%s:msmfb_writeback_queue_buffer ioctl failed\n",
				__func__);
	return ret;
}

static int msmfb_overlay_ioctl_writeback_dequeue_buffer(
		struct fb_info *info, unsigned long *argp)
{
	int ret = 0;
	struct msmfb_data data;

	ret = copy_from_user(&data, argp, sizeof(data));
	if (ret)
		goto error;

	ret = mdp4_writeback_dequeue_buffer(info, &data);
	if (ret)
		goto error;

	ret = copy_to_user(argp, &data, sizeof(data));
	if (ret)
		goto error;

error:
	if (ret)
		pr_err("%s:msmfb_writeback_dequeue_buffer ioctl failed\n",
				__func__);
	return ret;
}
static int msmfb_overlay_ioctl_writeback_terminate(struct fb_info *info)
{
	return mdp4_writeback_terminate(info);
}

#else
static int msmfb_overlay_ioctl_writeback_init(struct fb_info *info)
{
	return -ENOTSUPP;
}
static int msmfb_overlay_ioctl_writeback_start(
		struct fb_info *info)
{
	return -ENOTSUPP;
}

static int msmfb_overlay_ioctl_writeback_stop(
		struct fb_info *info)
{
	return -ENOTSUPP;
}

static int msmfb_overlay_ioctl_writeback_queue_buffer(
		struct fb_info *info, unsigned long *argp)
{
	return -ENOTSUPP;
}

static int msmfb_overlay_ioctl_writeback_dequeue_buffer(
		struct fb_info *info, unsigned long *argp)
{
	return -ENOTSUPP;
}
static int msmfb_overlay_ioctl_writeback_terminate(struct fb_info *info)
{
	return -ENOTSUPP;
}
#endif

static int msmfb_overlay_3d_sbys(struct fb_info *info, unsigned long *argp)
{
	int	ret;
	struct msmfb_overlay_3d req;

	ret = copy_from_user(&req, argp, sizeof(req));
	if (ret) {
		pr_err("%s:msmfb_overlay_3d_ctrl ioctl failed\n",
			__func__);
		return ret;
	}

	ret = mdp4_overlay_3d_sbys(info, &req);

	return ret;
}

static int msmfb_mixer_info(struct fb_info *info, unsigned long *argp)
{
	int     ret, cnt;
	struct msmfb_mixer_info_req req;

	ret = copy_from_user(&req, argp, sizeof(req));
	if (ret) {
		pr_err("%s: failed\n", __func__);
		return ret;
	}

	cnt = mdp4_mixer_info(req.mixer_num, req.info);
	req.cnt = cnt;
	ret = copy_to_user(argp, &req, sizeof(req));
	if (ret)
		pr_err("%s:msmfb_overlay_blt_off ioctl failed\n",
		__func__);

	return cnt;
}

#endif

DEFINE_SEMAPHORE(msm_fb_ioctl_ppp_sem);
DEFINE_MUTEX(msm_fb_ioctl_lut_sem);
DEFINE_MUTEX(msm_fb_ioctl_hist_sem);

/* Set color conversion matrix from user space */

#ifndef CONFIG_FB_MSM_MDP40
static void msmfb_set_color_conv(struct mdp_ccs *p)
{
	int i;

	if (p->direction == MDP_CCS_RGB2YUV) {
		/* MDP cmd block enable */
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

		/* RGB->YUV primary forward matrix */
		for (i = 0; i < MDP_CCS_SIZE; i++)
			writel(p->ccs[i], MDP_CSC_PFMVn(i));

		#ifdef CONFIG_FB_MSM_MDP31
		for (i = 0; i < MDP_BV_SIZE; i++)
			writel(p->bv[i], MDP_CSC_POST_BV2n(i));
		#endif

		/* MDP cmd block disable */
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
	} else {
		/* MDP cmd block enable */
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

		/* YUV->RGB primary reverse matrix */
		for (i = 0; i < MDP_CCS_SIZE; i++)
			writel(p->ccs[i], MDP_CSC_PRMVn(i));
		for (i = 0; i < MDP_BV_SIZE; i++)
			writel(p->bv[i], MDP_CSC_PRE_BV1n(i));

		/* MDP cmd block disable */
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
	}
}
#else
static void msmfb_set_color_conv(struct mdp_csc *p)
{
	mdp4_vg_csc_update(p);
}
#endif

static int msmfb_notify_update(struct fb_info *info, unsigned long *argp)
{
	int ret, notify;
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	ret = copy_from_user(&notify, argp, sizeof(int));
	if (ret) {
		pr_err("%s:ioctl failed\n", __func__);
		return ret;
	}

	if (notify > NOTIFY_UPDATE_STOP)
		return -EINVAL;

	if (notify == NOTIFY_UPDATE_START) {
		INIT_COMPLETION(mfd->msmfb_update_notify);
		wait_for_completion_interruptible(&mfd->msmfb_update_notify);
	} else {
		INIT_COMPLETION(mfd->msmfb_no_update_notify);
		wait_for_completion_interruptible(&mfd->msmfb_no_update_notify);
	}
	return 0;
}

static int msmfb_handle_pp_ioctl(struct msmfb_mdp_pp *pp_ptr)
{
	int ret = -1;

	if (!pp_ptr)
		return ret;

	switch (pp_ptr->op) {
#ifdef CONFIG_FB_MSM_MDP40
	case mdp_op_csc_cfg:
		ret = mdp4_csc_config(&(pp_ptr->data.csc_cfg_data));
		break;

	case mdp_op_pcc_cfg:
		ret = mdp4_pcc_cfg(&(pp_ptr->data.pcc_cfg_data));
		break;

	case mdp_op_lut_cfg:
		switch (pp_ptr->data.lut_cfg_data.lut_type) {
		case mdp_lut_igc:
			ret = mdp4_igc_lut_config(
					(struct mdp_igc_lut_data *)
					&pp_ptr->data.lut_cfg_data.data);
			break;

		case mdp_lut_pgc:
			ret = mdp4_pgc_cfg(
				&pp_ptr->data.lut_cfg_data.data.pgc_lut_data);
			break;

		case mdp_lut_hist:
			ret = mdp_hist_lut_config(
					(struct mdp_hist_lut_data *)
					&pp_ptr->data.lut_cfg_data.data);
			break;

		default:
			break;
		}
		break;
#endif
	default:
		pr_warn("Unsupported request to MDP_PP IOCTL.\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int msm_fb_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	void __user *argp = (void __user *)arg;
	struct fb_cursor cursor;
	struct fb_cmap cmap;
	struct mdp_histogram hist;
#ifndef CONFIG_FB_MSM_MDP40
	struct mdp_ccs ccs_matrix;
#else
	struct mdp_csc csc_matrix;
#endif
	struct mdp_page_protection fb_page_protection;
	struct msmfb_mdp_pp mdp_pp;
    struct disp_diag_reg_type reg_data;
    struct msm_fb_panel_data* pdata = NULL;
    uint32_t mddi_crc_count_rsp;
    struct mddi_local_disp_state_type status;
    unsigned int refresh_kind;
	int ret = 0;

	switch (cmd) {
#ifdef CONFIG_FB_MSM_OVERLAY
	case MSMFB_OVERLAY_GET:
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_OVERLAY_GET S\n");
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_get(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_OVERLAY_GET E\n");
		break;
	case MSMFB_OVERLAY_SET:
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_OVERLAY_SET S\n");
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_set(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_OVERLAY_SET E\n");
		break;
	case MSMFB_OVERLAY_UNSET:
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_OVERLAY_UNSET S\n");
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_unset(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_OVERLAY_UNSET E\n");
		break;
	case MSMFB_OVERLAY_PLAY:
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_OVERLAY_PLAY S\n");
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_play(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_OVERLAY_PLAY E\n");
		break;
	case MSMFB_OVERLAY_PLAY_ENABLE:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_play_enable(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
		break;
	case MSMFB_OVERLAY_PLAY_WAIT:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_play_wait(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
		break;
	case MSMFB_OVERLAY_BLT:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_blt(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
		break;
	case MSMFB_OVERLAY_BLT_OFFSET:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_blt_off(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
		break;
	case MSMFB_OVERLAY_3D:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_3d_sbys(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
		break;
	case MSMFB_MIXER_INFO:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_mixer_info(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
		break;
	case MSMFB_WRITEBACK_INIT:
		ret = msmfb_overlay_ioctl_writeback_init(info);
		break;
	case MSMFB_WRITEBACK_START:
		ret = msmfb_overlay_ioctl_writeback_start(
				info);
		break;
	case MSMFB_WRITEBACK_STOP:
		ret = msmfb_overlay_ioctl_writeback_stop(
				info);
		break;
	case MSMFB_WRITEBACK_QUEUE_BUFFER:
		ret = msmfb_overlay_ioctl_writeback_queue_buffer(
				info, argp);
		break;
	case MSMFB_WRITEBACK_DEQUEUE_BUFFER:
		ret = msmfb_overlay_ioctl_writeback_dequeue_buffer(
				info, argp);
		break;
	case MSMFB_WRITEBACK_TERMINATE:
		ret = msmfb_overlay_ioctl_writeback_terminate(info);
		break;
#endif
	case MSMFB_BLIT:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_blit(info, argp);
		up(&msm_fb_ioctl_ppp_sem);

		break;

	/* Ioctl for setting ccs matrix from user space */
	case MSMFB_SET_CCS_MATRIX:
#ifndef CONFIG_FB_MSM_MDP40
		ret = copy_from_user(&ccs_matrix, argp, sizeof(ccs_matrix));
		if (ret) {
			printk(KERN_ERR
				"%s:MSMFB_SET_CCS_MATRIX ioctl failed \n",
				__func__);
			return ret;
		}

		down(&msm_fb_ioctl_ppp_sem);
		if (ccs_matrix.direction == MDP_CCS_RGB2YUV)
			mdp_ccs_rgb2yuv = ccs_matrix;
		else
			mdp_ccs_yuv2rgb = ccs_matrix;

		msmfb_set_color_conv(&ccs_matrix) ;
		up(&msm_fb_ioctl_ppp_sem);
#else
		ret = copy_from_user(&csc_matrix, argp, sizeof(csc_matrix));
		if (ret) {
			pr_err("%s:MSMFB_SET_CSC_MATRIX ioctl failed\n",
				__func__);
			return ret;
		}
		down(&msm_fb_ioctl_ppp_sem);
		msmfb_set_color_conv(&csc_matrix);
		up(&msm_fb_ioctl_ppp_sem);

#endif

		break;

	/* Ioctl for getting ccs matrix to user space */
	case MSMFB_GET_CCS_MATRIX:
#ifndef CONFIG_FB_MSM_MDP40
		ret = copy_from_user(&ccs_matrix, argp, sizeof(ccs_matrix)) ;
		if (ret) {
			printk(KERN_ERR
				"%s:MSMFB_GET_CCS_MATRIX ioctl failed \n",
				 __func__);
			return ret;
		}

		down(&msm_fb_ioctl_ppp_sem);
		if (ccs_matrix.direction == MDP_CCS_RGB2YUV)
			ccs_matrix = mdp_ccs_rgb2yuv;
		 else
			ccs_matrix =  mdp_ccs_yuv2rgb;

		ret = copy_to_user(argp, &ccs_matrix, sizeof(ccs_matrix));

		if (ret)	{
			printk(KERN_ERR
				"%s:MSMFB_GET_CCS_MATRIX ioctl failed \n",
				 __func__);
			return ret ;
		}
		up(&msm_fb_ioctl_ppp_sem);
#else
		ret = -EINVAL;
#endif

		break;

	case MSMFB_GRP_DISP:
#ifdef CONFIG_FB_MSM_MDP22
		{
			unsigned long grp_id;

			ret = copy_from_user(&grp_id, argp, sizeof(grp_id));
			if (ret)
				return ret;

			mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
			writel(grp_id, MDP_FULL_BYPASS_WORD43);
			mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF,
				      FALSE);
			break;
		}
#else
		return -EFAULT;
#endif
	case MSMFB_SUSPEND_SW_REFRESHER:
		if (!mfd->panel_power_on)
			return -EPERM;

		mfd->sw_refreshing_enable = FALSE;
		ret = msm_fb_stop_sw_refresher(mfd);
		break;

	case MSMFB_RESUME_SW_REFRESHER:
		if (!mfd->panel_power_on)
			return -EPERM;

		mfd->sw_refreshing_enable = TRUE;
		ret = msm_fb_resume_sw_refresher(mfd);
		break;

	case MSMFB_CURSOR:
		ret = copy_from_user(&cursor, argp, sizeof(cursor));
		if (ret)
			return ret;

		ret = msm_fb_cursor(info, &cursor);
		break;

	case MSMFB_SET_LUT:
		ret = copy_from_user(&cmap, argp, sizeof(cmap));
		if (ret)
			return ret;

		mutex_lock(&msm_fb_ioctl_lut_sem);
		ret = msm_fb_set_lut(&cmap, info);
		mutex_unlock(&msm_fb_ioctl_lut_sem);
		break;

	case MSMFB_HISTOGRAM:
		if (!mfd->panel_power_on)
			return -EPERM;

		if (!mfd->do_histogram)
			return -ENODEV;

		ret = copy_from_user(&hist, argp, sizeof(hist));
		if (ret)
			return ret;

		mutex_lock(&msm_fb_ioctl_hist_sem);
		ret = mfd->do_histogram(info, &hist);
		mutex_unlock(&msm_fb_ioctl_hist_sem);
		break;

	case MSMFB_HISTOGRAM_START:
		if (!mfd->panel_power_on)
			return -EPERM;

		if (!mfd->do_histogram)
			return -ENODEV;
		ret = mdp_start_histogram(info);
		break;

	case MSMFB_HISTOGRAM_STOP:
		if (!mfd->do_histogram)
			return -ENODEV;
		ret = mdp_stop_histogram(info);
		break;


	case MSMFB_GET_PAGE_PROTECTION:
		fb_page_protection.page_protection
			= mfd->mdp_fb_page_protection;
		ret = copy_to_user(argp, &fb_page_protection,
				sizeof(fb_page_protection));
		if (ret)
				return ret;
		break;

	case MSMFB_NOTIFY_UPDATE:
		ret = msmfb_notify_update(info, argp);
		break;

	case MSMFB_SET_PAGE_PROTECTION:
#if defined CONFIG_ARCH_QSD8X50 || defined CONFIG_ARCH_MSM8X60
		ret = copy_from_user(&fb_page_protection, argp,
				sizeof(fb_page_protection));
		if (ret)
				return ret;

		/* Validate the proposed page protection settings. */
		switch (fb_page_protection.page_protection)	{
		case MDP_FB_PAGE_PROTECTION_NONCACHED:
		case MDP_FB_PAGE_PROTECTION_WRITECOMBINE:
		case MDP_FB_PAGE_PROTECTION_WRITETHROUGHCACHE:
		/* Write-back cache (read allocate)  */
		case MDP_FB_PAGE_PROTECTION_WRITEBACKCACHE:
		/* Write-back cache (write allocate) */
		case MDP_FB_PAGE_PROTECTION_WRITEBACKWACACHE:
			mfd->mdp_fb_page_protection =
				fb_page_protection.page_protection;
			break;
		default:
			ret = -EINVAL;
			break;
		}
#else
		/*
		 * Don't allow caching until 7k DMA cache operations are
		 * available.
		 */
		ret = -EINVAL;
#endif
		break;

	case MSMFB_MDP_PP:
		ret = copy_from_user(&mdp_pp, argp, sizeof(mdp_pp));
		if (ret)
			return ret;

		ret = msmfb_handle_pp_ioctl(&mdp_pp);
		break;


    case MSMFB_MDDI_REG_WRITE:
        /* Get arg */
        ret = copy_from_user(&reg_data, argp, sizeof(reg_data));
        if (ret)
        {
            /* Error */
            return ret;
        }
        /* PMDH Clock OFF prohibition for Diag */
        g_mddi_pmdh_clock_off_prohibit = 0x01;
        /* PMDH Clock ON */
        pmdh_clk_func(1);
        /* Register Write */
        ret = mddi_queue_register_write(reg_data.addr, reg_data.data, TRUE, 0);
        /* PMDH Clock OFF Permission for Diag */
        g_mddi_pmdh_clock_off_prohibit = 0x00;
        /* PMDH Clock OFF */
        pmdh_clk_func(0);
        break;

    case MSMFB_MDDI_REG_READ:
        /* Get arg */
        ret = copy_from_user(&reg_data, argp, sizeof(reg_data));
        if (ret)
        {
            /* Error */
            return ret;
        }
        /* PMDH Clock OFF prohibition for Diag */
        g_mddi_pmdh_clock_off_prohibit = 0x01;
        /* PMDH Clock ON */
        pmdh_clk_func(1);
        /* 10ms wait */
        msleep(10);
        /* Register Read */
        ret = mddi_queue_register_read(reg_data.addr, &reg_data.data, TRUE, 0);
        /* PMDH Clock OFF Permission for Diag */
        g_mddi_pmdh_clock_off_prohibit = 0x00;
        /* PMDH Clock OFF */
        pmdh_clk_func(0);
        /* 10ms wait */
        msleep(10);
        if (ret)
        {
            /* Error */
            return ret;
        }
        /* Set arg */
        ret = copy_to_user(argp, &reg_data, sizeof(reg_data));
        if (ret)
        {
            /* Error */
            return ret;
        }
        break;

    case MSMFB_MDDI_CRC_GET:

        /* get mddi_crc_count */
        mddi_crc_count_rsp = mddi_local_state.crc_error_count;

        /* Set arg */
        ret = copy_to_user(argp, &mddi_crc_count_rsp, sizeof(mddi_crc_count_rsp));

        if (ret)
        {
            /* Error */
            return ret;
        }
        break;


    case MSMFB_MDDI_CRC_RESET:

        /* reset mddi_crc_cout */
        mddi_local_state.crc_error_count = 0x00000000;

        break;

    case MSMFB_GET_STATUS:
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_GET_STATUS S\n");
        status = mddi_local_state;
        ret = copy_to_user(argp, &status, sizeof(status));
        if (ret)
        {
            DISP_LOCAL_LOG_EMERG("DISP MSMFB_GET_STATUS Error\n");
            /* Error */
            return ret;
        }
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_GET_STATUS E\n");
        break;

    case MSMFB_CRC_CHECK:
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_CRC_CHECK S\n");
        down(&disp_local_mutex);
        if(MDDI_LOCAL_DISPLAY_ON == mddi_local_state.disp_state)
        {
            pmdh_clk_func(1);

            mddi_set_hibernation_to_active();

            mddi_local_crc_error_check();

            mddi_set_auto_hibernation();

            pmdh_clk_func(0);
        }
        up(&disp_local_mutex);
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_CRC_CHECK E\n");
        break;

    case MSMFB_REFRESH_SEQ:
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_REFRESH_SEQ S\n");
        pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;
        if (NULL == pdata)
        {
            DISP_LOCAL_LOG_EMERG("DISP MSMFB_REFRESH_SEQ Error1\n");
            return -ENODEV;
        }
        ret = copy_from_user(&refresh_kind, argp, sizeof(refresh_kind));
        if (ret)
        {
            DISP_LOCAL_LOG_EMERG("DISP MSMFB_REFRESH_SEQ Error2\n");
            return ret;
        }
        down(&disp_local_mutex);
        if(MDDI_LOCAL_DISPLAY_ON == mddi_local_state.disp_state)
        {
            pmdh_clk_func(1);

            mddi_set_hibernation_to_active();

            pdata->refresh( refresh_kind );

            mddi_set_auto_hibernation();

            pmdh_clk_func(0);
        }
        up(&disp_local_mutex);
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_REFRESH_SEQ E\n");
        break;

    case MSMFB_RE_UPDATE:
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_RE_UPDATE S\n");
        pmdh_clk_func(1);
        mdp4_mddi_overlay_restore();
        pmdh_clk_func(0);
        DISP_LOCAL_LOG_EMERG("DISP MSMFB_RE_UPDATE E\n");
        break;

    case MSMFB_DISP_MSG_OUT:
        ret = copy_from_user(&fb_dbg_msg_level, argp,
                              sizeof(fb_dbg_msg_level));
        break;  
	default:
		MSM_FB_INFO("MDP: unknown ioctl (cmd=%x) received!\n", cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int msm_fb_register_driver(void)
{
	return platform_driver_register(&msm_fb_driver);
}

#ifdef CONFIG_FB_MSM_WRITEBACK_MSM_PANEL
struct fb_info *msm_fb_get_writeback_fb(void)
{
	int c = 0;
	for (c = 0; c < fbi_list_index; ++c) {
		struct msm_fb_data_type *mfd;
		mfd = (struct msm_fb_data_type *)fbi_list[c]->par;
		if (mfd->panel.type == WRITEBACK_PANEL)
			return fbi_list[c];
	}

	return NULL;
}
EXPORT_SYMBOL(msm_fb_get_writeback_fb);

int msm_fb_writeback_start(struct fb_info *info)
{
	return mdp4_writeback_start(info);
}
EXPORT_SYMBOL(msm_fb_writeback_start);

int msm_fb_writeback_queue_buffer(struct fb_info *info,
		struct msmfb_data *data)
{
	return mdp4_writeback_queue_buffer(info, data);
}
EXPORT_SYMBOL(msm_fb_writeback_queue_buffer);

int msm_fb_writeback_dequeue_buffer(struct fb_info *info,
		struct msmfb_data *data)
{
	return mdp4_writeback_dequeue_buffer(info, data);
}
EXPORT_SYMBOL(msm_fb_writeback_dequeue_buffer);

int msm_fb_writeback_stop(struct fb_info *info)
{
	return mdp4_writeback_stop(info);
}
EXPORT_SYMBOL(msm_fb_writeback_stop);
int msm_fb_writeback_init(struct fb_info *info)
{
	return mdp4_writeback_init(info);
}
EXPORT_SYMBOL(msm_fb_writeback_init);
int msm_fb_writeback_terminate(struct fb_info *info)
{
	return mdp4_writeback_terminate(info);
}
EXPORT_SYMBOL(msm_fb_writeback_terminate);
#endif

struct platform_device *msm_fb_add_device(struct platform_device *pdev)
{
	struct msm_fb_panel_data *pdata;
	struct platform_device *this_dev = NULL;
	struct fb_info *fbi;
	struct msm_fb_data_type *mfd = NULL;
	u32 type, id, fb_num;

	if (!pdev)
		return NULL;
	id = pdev->id;

	pdata = pdev->dev.platform_data;
	if (!pdata)
		return NULL;
	type = pdata->panel_info.type;

#if defined MSM_FB_NUM
	/*
	 * over written fb_num which defined
	 * at panel_info
	 *
	 */
	if (type == HDMI_PANEL || type == DTV_PANEL ||
		type == TV_PANEL || type == WRITEBACK_PANEL) {
#ifdef CONFIG_FB_MSM_HDMI_AS_PRIMARY
		pdata->panel_info.fb_num = 2;
#else
		pdata->panel_info.fb_num = 1;
#endif
	}
	else
		pdata->panel_info.fb_num = MSM_FB_NUM;

	MSM_FB_INFO("setting pdata->panel_info.fb_num to %d. type: %d\n",
			pdata->panel_info.fb_num, type);
#endif
	fb_num = pdata->panel_info.fb_num;

	if (fb_num <= 0)
		return NULL;

	if (fbi_list_index >= MAX_FBI_LIST) {
		printk(KERN_ERR "msm_fb: no more framebuffer info list!\n");
		return NULL;
	}
	/*
	 * alloc panel device data
	 */
	this_dev = msm_fb_device_alloc(pdata, type, id);

	if (!this_dev) {
		printk(KERN_ERR
		"%s: msm_fb_device_alloc failed!\n", __func__);
		return NULL;
	}

	/*
	 * alloc framebuffer info + par data
	 */
	fbi = framebuffer_alloc(sizeof(struct msm_fb_data_type), NULL);
	if (fbi == NULL) {
		platform_device_put(this_dev);
		printk(KERN_ERR "msm_fb: can't alloca framebuffer info data!\n");
		return NULL;
	}

	mfd = (struct msm_fb_data_type *)fbi->par;
	mfd->key = MFD_KEY;
	mfd->fbi = fbi;
	mfd->panel.type = type;
	mfd->panel.id = id;
	mfd->fb_page = fb_num;
	mfd->index = fbi_list_index;
	mfd->mdp_fb_page_protection = MDP_FB_PAGE_PROTECTION_WRITECOMBINE;
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	mfd->iclient = msm_ion_client_create(-1, pdev->name);
#else
	mfd->iclient = NULL;
#endif
	/* link to the latest pdev */
	mfd->pdev = this_dev;

	mfd_list[mfd_list_index++] = mfd;
	fbi_list[fbi_list_index++] = fbi;

	/*
	 * set driver data
	 */
	platform_set_drvdata(this_dev, mfd);

	if (platform_device_add(this_dev)) {
		printk(KERN_ERR "msm_fb: platform_device_add failed!\n");
		platform_device_put(this_dev);
		framebuffer_release(fbi);
		fbi_list_index--;
		return NULL;
	}
	return this_dev;
}
EXPORT_SYMBOL(msm_fb_add_device);

int get_fb_phys_info(unsigned long *start, unsigned long *len, int fb_num)
{
	struct fb_info *info;

	if (fb_num > MAX_FBI_LIST)
		return -1;

	info = fbi_list[fb_num];
	if (!info)
		return -1;

	*start = info->fix.smem_start;
	*len = info->fix.smem_len;
	return 0;
}
EXPORT_SYMBOL(get_fb_phys_info);

int __init msm_fb_init(void)
{
	int rc = -ENODEV;

	if (msm_fb_register_driver())
		return rc;

#ifdef MSM_FB_ENABLE_DBGFS
	{
		struct dentry *root;

		if ((root = msm_fb_get_debugfs_root()) != NULL) {
			msm_fb_debugfs_file_create(root,
						   "msm_fb_msg_printing_level",
						   (u32 *) &msm_fb_msg_level);
			msm_fb_debugfs_file_create(root,
						   "mddi_msg_printing_level",
						   (u32 *) &mddi_msg_level);
			msm_fb_debugfs_file_create(root, "msm_fb_debug_enabled",
						   (u32 *) &msm_fb_debug_enabled);
		}
	}
#endif

	return 0;
}

module_init(msm_fb_init);

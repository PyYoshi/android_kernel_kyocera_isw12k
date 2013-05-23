/* 
 * ~/arch/arm/mach-msm/board-msm7x30-wifi.c
 *
 * Copyright (C) 2011 Renesas Mobile Corporation.
 * Copyright (C) 2011 Renesas Design Vietnam Co., Ltd
 *
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 *	Based on board-ape5r-wifi.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/wlan_plat.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <mach/pmic.h>

#define GPIO_WLAN_REG_ON	90
#if 0
#define GPIO_REG_ON		91
#define GPIO_RST_N		92
#endif
#define GPIO_WLAN_IRQ		147

#define KC_MAC_CFG		"/data/misc/wifi/kc_mac_cfg.ini"
#define MAX_CFGBUF_SIZE		4096
#define NETWORKADDRESS		"NetworkAddress="
#define GAPMACADDR		"gAPMacAddr="

static int msm7x30_wifi_power(int on);
static void msm7x30_cfg_bcmdhd(void);

static int msm7x30_wifi_cd = 0;	/* WIFI virtual 'card detect' status */

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

int msm7x30_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;

	return 0;
}

unsigned int msm7x30_wifi_status(struct device *dev)
{
	return msm7x30_wifi_cd;
}

static int msm7x30_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	msm7x30_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: nobody to notify\n", __func__);

	return 0;
}

static int msm7x30_wifi_power(int on)
{
	if (current && current->comm)
		printk(KERN_INFO "%s: %d(%s)\n", __func__, on, current->comm);
	else
		printk(KERN_INFO "%s: %d\n", __func__, on);

	if (on) {
		int ret;

		if ((ret = pmic_hwp_get_value(2)))
			if (ret < 0)
				printk(KERN_WARNING "%s: pmic_hwp_get_value "
					"error(%d)\n", __func__, ret);
			else
				printk(KERN_INFO "%s: PA Power High\n",
					__func__);
		else {
			printk(KERN_ERR "%s: PA Power Low \n", __func__);
			return -EINVAL;
		}
	}
	/* Turn on/off WLAN chipset */
	mdelay(200);
	gpio_set_value(GPIO_WLAN_REG_ON, on);
	printk(KERN_INFO "Expected value is %d, result value is %d\n",
			on, gpio_get_value(GPIO_WLAN_REG_ON));

	return 0;
}

static void *_os_malloc(unsigned int size)
{
	void *addr;
	gfp_t flags;

	flags =(in_atomic()) ? GFP_ATOMIC : GFP_KERNEL;
	addr = kmalloc(size, flags);

	return addr;
}

static struct file *_os_open_file(char *filename)
{
	struct file *fp;

	fp = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		printk(KERN_ERR "filp_open returned error %ld\n", PTR_ERR(fp));
		fp = NULL;
	}

	return fp;
}

static int _os_get_file_block(char *buf, int len, struct file *fp)
{
	int rdlen;

	if (!fp)
		return 0;

	rdlen = kernel_read(fp, fp->f_pos, buf, len);
	if (rdlen > 0)
		fp->f_pos += rdlen;

	return rdlen;
}

static void _os_close_file(struct file *fp)
{
	if (fp)
		filp_close(fp, NULL);
}

static int _cfg_vars(char *cfgbuf, int len)
{
	char *dp = cfgbuf;
	int nl = 0;
	int column = 0;
	int i;
	int buf_len;

	for (i = 0; i < len; i++) {
		if (cfgbuf[i] == '\r')
			continue;
		if (nl && cfgbuf[i] != '\n')
			continue;
		nl = 0;
		if (cfgbuf[i] == '#') {
			nl = 1;
			continue;
		}
		if (cfgbuf[i] == '\n') {
			if (!column)
				continue;
			*dp++ = cfgbuf[i];
			column = 0;
			continue;
		} else {
			*dp++ = cfgbuf[i];
			column++;
		}
	}
	buf_len = (int)(dp - cfgbuf);

	while (dp < cfgbuf + i)
		*dp++ = 0;

	return buf_len;
}

static int msm7x30_wifi_get_mac_addr(unsigned char *buf)
{
	int i, find;
	char macaddr[6];
	char temp[4];
	void *memptr = NULL;
	struct file *fp = NULL;
	int len;
	char *sp;
	unsigned int val;

	memset(macaddr, 0, sizeof(macaddr));
	fp = _os_open_file(KC_MAC_CFG);
	if (!fp) {
		printk(KERN_ERR "MAC Address File Open Error(%s)\n",
			KC_MAC_CFG);
		return -EFAULT;
	}

	memptr = _os_malloc(MAX_CFGBUF_SIZE);
	if (!memptr) {
		printk(KERN_ERR "Failed to allocate memory %d byte\n",
			MAX_CFGBUF_SIZE);
		return -EFAULT;
	}

	find = 0;
	memset(memptr, 0, MAX_CFGBUF_SIZE);
	len = _os_get_file_block((char *)memptr, MAX_CFGBUF_SIZE, fp);
	if (len > 0 && len < MAX_CFGBUF_SIZE) {
		len = _cfg_vars((char *)memptr, len);

		if ((sp = strstr(memptr, NETWORKADDRESS)))
			sp += strlen(NETWORKADDRESS);
		else if ((sp = strstr(memptr, GAPMACADDR)))
			sp += strlen(GAPMACADDR);

		if (sp) {
			for (i = 0; i < 12; i++) {
				if (!isxdigit(*(sp + i)))
					break;
			}

			if (i >= 12) {
				for (i = 0; i < 6; i++) {
					val = 0;
					memset(temp, 0, sizeof(temp));
					strncpy(temp, sp + (i << 1), 2);
					sscanf(temp, "%02x", &val);
					macaddr[i] = val;
				}
				find = 1;
			} else
				printk(KERN_ERR "Error Mac Address field\n");
		} else
			printk(KERN_ERR "No such MAC Address field\n");
	} else
		printk(KERN_ERR "Error reading cfg file:%d\n", len);

	if (memptr)
		kfree(memptr);

	if (fp)
		_os_close_file(fp);

	if (find) {
		printk(KERN_INFO "NV MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
			macaddr[0], macaddr[1], macaddr[2],
			macaddr[3], macaddr[4], macaddr[5]);
		if ((macaddr[0] | macaddr[1] | macaddr[2] |
		     macaddr[3] | macaddr[4] | macaddr[5])) {
			memcpy(buf, macaddr, 6);
			return 0;
		} else {
			printk(KERN_ERR "Bad Mac Address\n");
			return -EFAULT;
		}
	} else
		return -EFAULT;
}

/* Customized Locale table : OPTIONAL feature */
#define WLC_CNTRY_BUF_SZ	4
typedef struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int  custom_locale_rev;
} cntry_locales_custom_t;

static cntry_locales_custom_t msm7x30_wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"JP", "XY", 6},	/* universal */
	{"US", "US", 69},	/* input ISO "US" to : US regrev 69 */
	{"CA", "US", 69},	/* input ISO "CA" to : US regrev 69 */
	{"EU", "EU", 5},	/* European union countries */
	{"AT", "EU", 5},
	{"BE", "EU", 5},
	{"BG", "EU", 5},
	{"CY", "EU", 5},
	{"CZ", "EU", 5},
	{"DK", "EU", 5},
	{"EE", "EU", 5},
	{"FI", "EU", 5},
	{"FR", "EU", 5},
	{"DE", "EU", 5},
	{"GR", "EU", 5},
	{"HU", "EU", 5},
	{"IE", "EU", 5},
	{"IT", "EU", 5},
	{"LV", "EU", 5},
	{"LI", "EU", 5},
	{"LT", "EU", 5},
	{"LU", "EU", 5},
	{"MT", "EU", 5},
	{"NL", "EU", 5},
	{"PL", "EU", 5},
	{"PT", "EU", 5},
	{"RO", "EU", 5},
	{"SK", "EU", 5},
	{"SI", "EU", 5},
	{"ES", "EU", 5},
	{"SE", "EU", 5},
	{"GB", "EU", 5},	/* input ISO "GB" to : EU regrev 05 */
	{"IL", "IL", 0},
	{"CH", "CH", 0},
	{"TR", "TR", 0},
	{"NO", "NO", 0},
	{"KR", "XY", 3},
	{"AU", "XY", 3},
	{"CN", "XY", 3},	/* input ISO "CN" to : XY regrev 03 */
	{"TW", "XY", 3},
	{"AR", "XY", 3},
	{"MX", "XY", 3}
};

static void *msm7x30_wifi_get_country_code(char *ccode)
{
#if 0
	int size = ARRAY_SIZE(msm7x30_wifi_translate_custom_table);
	int i;
#endif

	if (!ccode)
		return NULL;

	printk(KERN_INFO "%s: start (ccode:%s)\n", __func__, ccode);

#if 0
	for (i = 0; i < size; i++)
		if (!strcmp(ccode,
			msm7x30_wifi_translate_custom_table[i].iso_abbrev))
			return &msm7x30_wifi_translate_custom_table[i];
#endif
	return &msm7x30_wifi_translate_custom_table[0];
}

static struct resource msm7x30_wifi_resources[] = {
	[0] = {
		.name	= "bcmdhd_wlan_irq",
		.start	= MSM_GPIO_TO_INT(GPIO_WLAN_IRQ),
		.end	= MSM_GPIO_TO_INT(GPIO_WLAN_IRQ),
		.flags	= IORESOURCE_IRQ |
			  IORESOURCE_IRQ_HIGHLEVEL |
			  IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct wifi_platform_data msm7x30_wifi_control = {
	.set_power	= msm7x30_wifi_power,
	.set_carddetect	= msm7x30_wifi_set_carddetect,
	.get_mac_addr	= msm7x30_wifi_get_mac_addr,
	.get_country_code	= msm7x30_wifi_get_country_code,
};

static struct platform_device msm7x30_wifi_device = {
        .name           = "bcmdhd_wlan",
        .id             = 1,
	.num_resources	= ARRAY_SIZE(msm7x30_wifi_resources),
	.resource	= msm7x30_wifi_resources,
        .dev            = {
                .platform_data = &msm7x30_wifi_control,
        },
};

static struct msm_gpio bcmdhd_gpios[] = {
	{ GPIO_CFG(GPIO_WLAN_REG_ON, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
	  GPIO_CFG_2MA), "WLAN_REG_ON" },
#if 0
	{ GPIO_CFG(GPIO_REG_ON, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
	  GPIO_CFG_2MA), "BT_REG_ON" },
	{ GPIO_CFG(GPIO_RST_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
	  GPIO_CFG_2MA), "BT_RST_N" },
#endif
	{ GPIO_CFG(GPIO_WLAN_IRQ, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
	  GPIO_CFG_2MA), "WLAN_IRQ" },
};

static void msm7x30_cfg_bcmdhd(void)
{
	if (msm_gpios_request_enable(bcmdhd_gpios, ARRAY_SIZE(bcmdhd_gpios)))
		printk(KERN_ERR "%s: unable to enable gpios\n", __func__);
}

static struct proc_dir_entry *msm7x30_wifi_proc_dir;

static int msm7x30_wifi_proc_read(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	int len = 0;
	int ret;

	if ((ret = pmic_hwp_get_value(2)))
		if (ret < 0) {
			printk(KERN_ERR "%s: pmic_hwp_get_value error(%d)\n",
				__func__, ret);
			len = sprintf(page, "%s\n", "??");
		} else
			len = sprintf(page, "%s\n", "OK");
	else
		len = sprintf(page, "%s\n", "NG");
	*eof = 1;

	return len;
}

static int __init msm7x30_wifi_proc_init(void)
{
	struct proc_dir_entry *ent;

	msm7x30_wifi_proc_dir = proc_mkdir("bcm4330", NULL);
	if (!msm7x30_wifi_proc_dir) {
		printk(KERN_ERR "%s :Unable to create /proc/bcm4330 "
			"directory\n", __func__);
		return -ENOMEM;
	}

	ent = create_proc_entry("pa_status", 0, msm7x30_wifi_proc_dir);
	if (!ent) {
		printk(KERN_ERR "%s: Unable to create /proc/bcm4330/pa_status "
			"entry\n", __func__);
		remove_proc_entry("bcm4330", 0);
		return -ENOMEM;
	}

	ent->mode = S_IRUGO;
	ent->gid = 0;
	ent->uid = 0;
	ent->read_proc = (read_proc_t *)msm7x30_wifi_proc_read;

	return 0;
}

#ifdef CONFIG_BCMDHD_MODULE
static int __init msm7x30_wifi_init(void)
#else
int __init msm7x30_wifi_init(void)
#endif
{
	printk(KERN_INFO "%s: start\n", __func__);
	msm7x30_cfg_bcmdhd();
	msm7x30_wifi_proc_init();
	return platform_device_register(&msm7x30_wifi_device);
}
#ifdef CONFIG_BCMDHD_MODULE
late_initcall(msm7x30_wifi_init);
#endif

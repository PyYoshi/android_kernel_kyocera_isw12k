/* security/kyocera/kclsm_sysfs.c  (kclsm LSM module sysfs interface)
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include "kclsm_sysfs.h"

static int debuggerd;
static int ueventd;
static int vold;
int debuggerd_pid(void)
{
	return debuggerd;
}
int ueventd_pid(void)
{
	return ueventd;
}
int vold_pid(void)
{
	return vold;
}

static ssize_t debuggerd_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	if (current->pid != 1)
		return count;

	sscanf(buf, "%du", &debuggerd);
	return count;
}
static struct kobj_attribute kclsm_debuggerd_attr = __ATTR(debuggerd,
					S_IWUSR, NULL, debuggerd_store);

static ssize_t ueventd_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	if (current->pid != 1)
		return count;

	sscanf(buf, "%du", &ueventd);
	return count;
}
static struct kobj_attribute kclsm_ueventd_attr =  __ATTR(ueventd,
					S_IWUSR, NULL, ueventd_store);

static ssize_t vold_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	if (current->pid != 1)
		return count;

	sscanf(buf, "%du", &vold);
	return count;
}
static struct kobj_attribute kclsm_vold_attr =  __ATTR(vold,
					S_IWUSR, NULL, vold_store);

static struct attribute *kclsm_attrs[] = {
	&kclsm_debuggerd_attr.attr,
	&kclsm_ueventd_attr.attr,
	&kclsm_vold_attr.attr,
	NULL,
};

static struct attribute_group kclsm_attr_group = {
	.attrs = kclsm_attrs,
};

static struct kobject *kclsm_kobj;
static int __init kclsm_sysfs_init(void)
{
	int retval;

	kclsm_kobj = kobject_create_and_add("kclsm", kernel_kobj);
	if (!kclsm_kobj)
		return -ENOMEM;

	
	retval = sysfs_create_group(kclsm_kobj, &kclsm_attr_group);
	if (retval == 0) {
		pr_info("kclsm_sysfs initialized\n");
	} else {
		pr_info("kclsm_sysfs init failed\n");
		kobject_put(kclsm_kobj);
	}

	return retval;
}

static void __exit kclsm_sysfs_exit(void)
{
	kobject_put(kclsm_kobj);
}
module_init(kclsm_sysfs_init);
module_exit(kclsm_sysfs_exit);
MODULE_LICENSE("GPL");

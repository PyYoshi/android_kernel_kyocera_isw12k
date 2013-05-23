/* security/kyocera/kclsm.c  (kclsm LSM module)
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
#include <linux/security.h>
#include <linux/ptrace.h>
#include <linux/mount.h>
#include <linux/path.h>
#include "kclsm.h"
#include "kclsm_sysfs.h"

#ifdef CONFIG_SECURITY_KCLSM_PTRACE
static int kclsm_ptrace_access_check(struct task_struct *child,
				     unsigned int mode)
{
	if (current_uid() != 0)
		return KCLSM_RETURN(0);

	if (current->pid == debuggerd_pid())
		return KCLSM_RETURN(0);

	if (mode == PTRACE_MODE_READ)
		return KCLSM_RETURN(0);

	pr_warn("no permission in %s pid=%d pname=%s child=%s mode=%d\n",
		__FUNCTION__, current->pid, current->comm, child->comm, mode);
	return KCLSM_RETURN(-EPERM);
}

static int kclsm_ptrace_traceme(struct task_struct *parent)
{
	if (current_uid() != 0)
		return KCLSM_RETURN(0);

	pr_warn("no permission in %s pid=%d pname=%s parent=%s\n",
		__FUNCTION__, current->pid, current->comm, parent->comm);
	return KCLSM_RETURN(-EPERM);
}
#endif

#ifdef CONFIG_SECURITY_KCLSM_INSMOD
static int kclsm_kernel_setup_load_info(char *kmod_name)
{
	int i;

	for (i=0; kclsm_module_checklist[i]!=NULL; i++)
		if (strcmp(kclsm_module_checklist[i], kmod_name) == 0)
			return KCLSM_RETURN(0);

	pr_warn("no permission in %s pid=%d pname=%s module=%s\n",
		__FUNCTION__, current->pid, current->comm, kmod_name);
	return KCLSM_RETURN(-EPERM);
}
#endif

#ifdef CONFIG_SECURITY_KCLSM_MOUNT
static int kclsm_sb_mount(char *dev_name, struct path *path,
			    char *type, unsigned long flags, void *data)
{
	int i, ret = 0;
	char *ptr, *realpath = NULL;

	ptr = kmalloc(PATH_MAX, GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	realpath = d_path(path, ptr, PATH_MAX);

	if (strncmp(realpath, KCLSM_SYSTEM_MOUNT_POINT,
		    strlen(KCLSM_SYSTEM_MOUNT_POINT)) != 0)
		goto out;

	if (strcmp(realpath, KCLSM_SYSTEM_MOUNT_POINT) != 0) {
		if (current->pid == 1)
			goto out;
		else
			goto err;
	}

	if (flags & MS_REMOUNT)
		goto err;

	for (i=0; kclsm_mount_checklist[i] != NULL; i++)
		if (strcmp(dev_name, kclsm_mount_checklist[i]) == 0)
			goto out;

err:
	pr_warn("no permission in %s pid=%d pname=%s"
		"realpath=%s dev_name=%s\n",
		__FUNCTION__, current->pid, current->comm,
		realpath, dev_name);
	ret = -EPERM;
out:
	kfree(ptr);
	return KCLSM_RETURN(ret);
}
#endif

#ifdef CONFIG_SECURITY_KCLSM_UMOUNT
#define KCLSM_BOOTMODE_NORMAL 0
#define KCLSM_BOOTMODE_UPDATE 1
static int kclsm_bootmode = KCLSM_BOOTMODE_NORMAL;
static int __init kclsm_bootmode_setup(char *buf)
{
	if (strcmp(buf, "f-ksg") == 0)
		kclsm_bootmode = KCLSM_BOOTMODE_UPDATE;
	return 0;
}
early_param("kcdroidboot.mode", kclsm_bootmode_setup);

static int kclsm_sb_umount(struct vfsmount *mnt, int flags)
{
	int ret = 0;
	struct path umount_path;
	char *ptr, *realpath = NULL;

	ptr = kmalloc(PATH_MAX, GFP_KERNEL);
	if(!ptr)
		return -ENOMEM;

	umount_path.mnt = mnt;
	umount_path.dentry = mnt->mnt_root;

	realpath = d_path(&umount_path, ptr, PATH_MAX);

	if (strncmp(realpath, KCLSM_SYSTEM_MOUNT_POINT,
		    strlen(KCLSM_SYSTEM_MOUNT_POINT)) != 0)
		goto out;

	if (kclsm_bootmode == KCLSM_BOOTMODE_UPDATE)
		goto out;

	pr_warn("no permission in %s pid=%d pname=%s realpath=%s\n",
		__FUNCTION__, current->pid, current->comm, realpath);
	ret = -EPERM;
out:
	kfree(ptr);
	return KCLSM_RETURN(ret);
}
#endif

#ifdef CONFIG_SECURITY_KCLSM_PIVOTROOT
static int kclsm_sb_pivotroot(struct path *old_path, struct path *new_path)
{
	pr_warn("no permission in %s pid=%d pname=%s\n",
		__FUNCTION__, current->pid, current->comm);
	return KCLSM_RETURN(-EPERM);
}
#endif

#ifdef CONFIG_SECURITY_KCLSM_CHROOT
static int kclsm_path_chroot(struct path *path)
{
	pr_warn("no permission in %s pid=%d pname=%s\n",
		__FUNCTION__, current->pid, current->comm);
	return KCLSM_RETURN(-EPERM);
}
#endif

#ifdef CONFIG_SECURITY_KCLSM_DEVICEPERMISSION

static int kclsm_device_permission_check(const char *realpath)
{
	int i;
	if (strncmp(realpath, KCLSM_DEVICE_DIR,
		    strlen(KCLSM_DEVICE_DIR)) != 0)
		return 0;

	if (current->pid == 1 ||
	    current->pid == ueventd_pid() || current->pid == vold_pid() ||
	    current->group_leader->pid == vold_pid())
		return 0;

	for (i=0; kclsm_device_checklist[i] != NULL; i++)
		if (strcmp(realpath, kclsm_device_checklist[i]) == 0)
			return 0;

	return -EPERM;
}

static int kclsm_path_mknod(struct path *dir, struct dentry *dentry, int mode,
			    unsigned int dev)
{
	int ret = 0;
	struct path mknod_path;
	char *ptr, *realpath = NULL;

	mknod_path.mnt = dir->mnt;
	mknod_path.dentry = dentry;

	if (S_ISFIFO(mode) || S_ISSOCK(mode))
		return 0;

	ptr = kmalloc(PATH_MAX, GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	realpath = d_path(&mknod_path, ptr, PATH_MAX);
	ret = kclsm_device_permission_check(realpath);
	if (ret)
		pr_warn("no permission in %s pid=%d pname=%s realpath=%s\n",
			__FUNCTION__, current->pid, current->comm, realpath);

	kfree(ptr);
	return KCLSM_RETURN(ret);
}

#if 0
static int kclsm_path_chmod(struct dentry *dentry, struct vfsmount *mnt,
			    mode_t mode)
{
	int ret = 0;
	struct path chmod_path;
	char *ptr, *realpath = NULL;
	int i_mode = dentry->d_inode->i_mode;
	
	chmod_path.mnt = mnt;
	chmod_path.dentry = dentry;

	if (S_ISFIFO(i_mode) || S_ISSOCK(i_mode))
		return 0;

	ptr = kmalloc(PATH_MAX, GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	realpath = d_path(&chmod_path, ptr, PATH_MAX);
	ret = kclsm_device_permission_check(realpath);
	if (ret)
		pr_warn("no permission in %s pid=%d pname=%s realpath=%s\n",
			__FUNCTION__, current->pid, current->comm, realpath);

	kfree(ptr);
	return KCLSM_RETURN(ret);
}

static int kclsm_path_chown(struct path *path, uid_t uid, gid_t gid)
{
	int ret = 0;
	char *ptr, *realpath = NULL;
	int i_mode = path->dentry->d_inode->i_mode;

	if (S_ISFIFO(i_mode) || S_ISSOCK(i_mode))
		return 0;

	ptr = kmalloc(PATH_MAX, GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	realpath = d_path(path, ptr, PATH_MAX);
	ret = kclsm_device_permission_check(realpath);
	if (ret)
		pr_warn("no permission in %s pid=%d pname=%s realpath=%s\n",
			__FUNCTION__, current->pid, current->comm, realpath);

	kfree(ptr);
	return KCLSM_RETURN(ret);
}
#endif
#endif

static struct security_operations kclsm_security_ops = {
	.name = "kclsm",
#ifdef CONFIG_SECURITY_KCLSM_PTRACE
	.ptrace_access_check = kclsm_ptrace_access_check,
	.ptrace_traceme = kclsm_ptrace_traceme,
#endif
#ifdef CONFIG_SECURITY_KCLSM_INSMOD
	.kernel_setup_load_info  = kclsm_kernel_setup_load_info,
#endif
#ifdef CONFIG_SECURITY_KCLSM_MOUNT
	.sb_mount = kclsm_sb_mount,
#endif
#ifdef CONFIG_SECURITY_KCLSM_UMOUNT
	.sb_umount  = kclsm_sb_umount,
#endif
#ifdef CONFIG_SECURITY_KCLSM_PIVOTROOT
	.sb_pivotroot = kclsm_sb_pivotroot,
#endif
#ifdef CONFIG_SECURITY_KCLSM_CHROOT
	.path_chroot = kclsm_path_chroot,
#endif
#ifdef CONFIG_SECURITY_KCLSM_DEVICEPERMISSION
	.path_mknod = kclsm_path_mknod,
#if 0
	.path_chmod = kclsm_path_chmod,
	.path_chown = kclsm_path_chown,
#endif
#endif
};

static int __init kclsm_init(void)
{
	int ret;

	ret = register_security(&kclsm_security_ops);
	if (ret) {
		pr_err("Unable to register kclsm\n");
		return ret;
	}

	pr_info("kclsm initialized\n");

	return 0;
}
security_initcall(kclsm_init);

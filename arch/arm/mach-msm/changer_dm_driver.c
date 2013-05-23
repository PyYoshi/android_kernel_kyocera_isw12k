/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <mach/changer_dm_driver.h>
#include <mach/rpc_server_handset.h>


static int32_t changer_dm_driver_open(struct inode *inode, struct file *file)
{
    printk(KERN_DEBUG "%s START", __func__);
    return 0;
}


/* static int32_t changer_dm_driver_ioctl(struct inode* inode, struct file* filp, */ /* CHG ICS */
static long changer_dm_driver_ioctl(struct file* filp,                            /* CHG ICS */
                                 unsigned int cmd, unsigned long data)
{
    int32_t ret = 0;
    int32_t the_v[2];
    printk(KERN_DEBUG "%s START", __func__);
    memset(the_v, 0x00, sizeof(the_v));

    switch (cmd)
    {
        case CHANGER_DM_DRIVER_IOCTL_01:
        {
            kc_changer_ic_get_dminfo(CHANGER_DM_DRIVER_GET_SW_SET_MODE, the_v);
            if (copy_to_user((void *)data, the_v, sizeof(the_v)))
            {
                 ret = -EFAULT;
            }
        }
        break;
        case CHANGER_DM_DRIVER_IOCTL_02:
        {
            kc_changer_ic_get_dminfo(CHANGER_DM_DRIVER_GET_DETECTION_ID_STATE, the_v);
            if (copy_to_user((void *)data, the_v, sizeof(the_v)))
            {
                 ret = -EFAULT;
            }
        }
        break;
        case CHANGER_DM_DRIVER_IOCTL_03:
        {
            kc_changer_ic_get_dminfo(CHANGER_DM_DRIVER_GET_INTERRUPT_CASE, the_v);
            if (copy_to_user((void *)data, the_v, sizeof(the_v)))
            {
                 ret = -EFAULT;
            }
        }
        break;
        case CHANGER_DM_DRIVER_IOCTL_04:
        {
            kc_changer_ic_get_dminfo(CHANGER_DM_DRIVER_GET_CHG_DET_MODE, the_v);
            if (copy_to_user((void *)data, the_v, sizeof(the_v)))
            {
                 ret = -EFAULT;
            }
        }
        break;
        case CHANGER_DM_DRIVER_IOCTL_05:
        {
            if (copy_from_user(the_v, (void *) data, sizeof(sizeof(the_v))))
            {
                ret = -EFAULT;
                break;
            }
            /* kc_hs_jack_cmd(the_v); */ /* ICS tmp */
        }
        break;
        case CHANGER_DM_DRIVER_IOCTL_06:
        {
            if (copy_from_user(the_v, (void *) data, sizeof(sizeof(the_v))))
            {
                ret = -EFAULT;
                break;
            }
            kc_changer_ic_set_chgdet_wait(the_v);
        }
        break;
        default:
        {
            ret = -1;
        }
        break;
    }
    printk(KERN_DEBUG "%s END", __func__);
    return ret;
}


static const struct file_operations changer_dm_driverfops = {
    .owner      = THIS_MODULE,
    .open       = changer_dm_driver_open,
/*    .ioctl      = changer_dm_driver_ioctl, */ /* CHG ICS */
    .unlocked_ioctl = changer_dm_driver_ioctl,  /* CHG ICS */
};

static struct miscdevice changer_dm = {
    .fops       = &changer_dm_driverfops,
    .name       = "changer_dm",
    .minor      = MISC_DYNAMIC_MINOR,
};


static int32_t __init changer_dm_driver_init(void)
{
    printk(KERN_DEBUG "%s START", __func__);
    return misc_register(&changer_dm);
}


static void __exit changer_dm_driver_exit(void)
{
    printk(KERN_DEBUG "%s START", __func__);
    misc_deregister(&changer_dm);
}


module_init(changer_dm_driver_init);
module_exit(changer_dm_driver_exit);

MODULE_AUTHOR("KC");
MODULE_DESCRIPTION("CHANGER DM Driver");
MODULE_LICENSE("GPL v2");

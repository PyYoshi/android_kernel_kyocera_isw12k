/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                               <sdgpiodrv.c>
DESCRIPTION
  Driver for GPIO wrapper of SD card control.

EXTERNALIZED FUNCTIONS
  Operete gpio.

This software is contributed or developed by KYOCERA Corporation.
(C) 2011 KYOCERA Corporation
(C) 2012 KYOCERA Corporation
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include "sdgpiodrv.h"
#include <linux/sched.h>
#include <linux/clk.h>

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("kc_sdgdrv");

#define SDGDRV_DEBUG
#ifdef SDGDRV_DEBUG
    #define SDG_DRV_DEBUG(string, args...) \
        { \
            printk("%s: " string "\n", __func__, ##args); \
        }
#else
    #define SDG_DRV_DEBUG(string, args...)
#endif


#define SDGDRV_UNUSED    0
#define SDGDRV_USED      1


#define SDGDRV_UNABLE    0
#define SDGDRV_ABLE      1

#define MAXDEVBUF sizeof(SDGDRV_IF)
static unsigned char devbuf[MAXDEVBUF];
static int devbufpos;

typedef struct sdgdev_info {
    int use;                
    int readable;           
    wait_queue_head_t wq;   
    wait_queue_head_t rq;   
} SDGDEV_INFO;
static SDGDEV_INFO *sdgdrv_devs;

static dev_t            st_dev_id;
static unsigned int     st_major;
static unsigned int     st_minor;
static struct class*    st_class = NULL;
static struct cdev      st_cdev;
static int              st_devs_num = 1;
static struct semaphore st_sem;


/*===========================================================================
 FUNCTION sdgdrv_read

 DESCRIPTION
    The function which is called at the time of read system call.

 DEPENDENCIES
    none

 PARAMETER
    filp:   pointer of file structure.
    buf:    read buffer.
    count:  size of buf.
    pos:    position of buf.

 RETURN VALUE
    size of read buffer.

 SIDE EFFECTS
    none

===========================================================================*/
static ssize_t 
sdgdrv_read( struct file* filp, char* buf, size_t count, loff_t* pos )
{
    SDGDEV_INFO *dev = (SDGDEV_INFO *)filp->private_data;
    int i = 0;
    int cpbuflen = 0;

    if (down_interruptible(&st_sem))
    {
        SDG_DRV_DEBUG("down_interruptible() for read failed");
        return -ERESTARTSYS;
    }

    if (count >= devbufpos)
    {
        cpbuflen = devbufpos;
    }
    else
    {
        up(&st_sem);
        SDG_DRV_DEBUG("Parameter \"count\" is invalid value");
        return -EFAULT;
    }

    if (copy_to_user(buf, devbuf, cpbuflen))
    {
        up(&st_sem);
        SDG_DRV_DEBUG("copy_to_user() failed");
        return -EFAULT;
    }

    *pos += cpbuflen;

    for (i = cpbuflen; i < devbufpos; i++)
    {
        devbuf[i - cpbuflen] = devbuf[i];
    }

    devbufpos -= cpbuflen;

    dev->readable = SDGDRV_UNABLE;
    up(&st_sem);
    wake_up_interruptible(&dev->wq);

    return cpbuflen;
}

/*===========================================================================
 FUNCTION sdgdrv_write

 DESCRIPTION
    The function which is called at the time of write system call.

 DEPENDENCIES
    none

 PARAMETER
    filp:   pointer of file structure.
    buf:    write buffer.
    count:  size of buf.
    pos:    position of buf.

 RETURN VALUE
    size of written buffer.

 SIDE EFFECTS
    none

===========================================================================*/
static ssize_t
sdgdrv_write(struct file* filp, const char* buf, size_t count, loff_t* pos)
{
    SDGDEV_INFO *dev = (SDGDEV_INFO *)filp->private_data;
    SDGDRV_IF *gpioifp = NULL;
    int gpiores = 0;

    if (down_interruptible(&st_sem))
    {
        SDG_DRV_DEBUG("down_interruptible() for write failed");
        return -ERESTARTSYS;
    }

    if (MAXDEVBUF != count)
    {
        up(&st_sem);
        SDG_DRV_DEBUG("Not a data message");
        return -EBADMSG;
    }

    if (copy_from_user(devbuf, buf, count))
    {
        up(&st_sem);
        SDG_DRV_DEBUG("copy_from_user() failed");
        return -EFAULT;
    }

    gpioifp = (SDGDRV_IF *)devbuf;

    switch (gpioifp->gpiofuncid)
    {
    case SDG_GET_VALUE:
        gpiores = gpio_get_value_cansleep(gpioifp->gpio);
        break;
    case SDG_SET_VALUE:
            gpio_set_value_cansleep(gpioifp->gpio, gpioifp->value);
        gpiores = 0;
        break;
    case SDG_TO_IRQ:
        gpiores = gpio_to_irq(gpioifp->gpio);
        break;
    default:
        up(&st_sem);
        SDG_DRV_DEBUG("Not a data message(gpiofuncid = %d)", gpioifp->gpiofuncid);
        return -EBADMSG;
    }

    gpioifp->value = gpiores;
    devbufpos = count;

    dev->readable = SDGDRV_ABLE;
    up(&st_sem);
    wake_up_interruptible(&dev->rq);

    return devbufpos;
}

/*===========================================================================
 FUNCTION sdgdrv_poll

 DESCRIPTION
    The function which is called at the time of select system call and
    wake_up_interruptible().

 DEPENDENCIES
    none

 PARAMETER
    filp:   pointer of file structure.
    wait:   pointer of poll_table.

 RETURN VALUE
    (POLLIN | POLLRDNORM):  Readable
    (POLLOUT | POLLWRNORM): Writeable

 SIDE EFFECTS
    none

===========================================================================*/
static unsigned int sdgdrv_poll(struct file* filp, poll_table* wait)
{
    SDGDEV_INFO *dev = (SDGDEV_INFO *)filp->private_data;
    unsigned int retmask = 0;

    poll_wait(filp, &dev->rq, wait);
    poll_wait(filp, &dev->wq, wait);

    if (dev->readable)
    {
        retmask |= (POLLIN | POLLRDNORM);
    }
    if (!dev->readable)
    {
        retmask |= (POLLOUT | POLLWRNORM);
    }

    return retmask;
}

/*===========================================================================
 FUNCTION sdgdrv_open

 DESCRIPTION
    The function which is called at the time of open system call.

 DEPENDENCIES
    none

 PARAMETER
    inode:  pointer of inode structure.
    filp:   pointer of file structure.

 RETURN VALUE
    0:      Success
    not 0:  Failed

 SIDE EFFECTS
    none

===========================================================================*/
static int sdgdrv_open(struct inode* inode, struct file* filp)
{
    SDGDEV_INFO *dev = NULL;
    int i = 0;

    for (i = 0; i < st_devs_num; i++)
    {
        dev = sdgdrv_devs + i;
        if (!dev->use)
        {
            break;
        }
    }

    if (i == st_devs_num)
    {
        SDG_DRV_DEBUG("All devices is used");
        return -EBUSY;
    }

    if (down_interruptible(&st_sem))
    {
        SDG_DRV_DEBUG("down_interruptible() for read failed");
        return -ERESTARTSYS;
    }
    dev->use = SDGDRV_USED;
    dev->readable = SDGDRV_UNABLE;

    up(&st_sem);
    filp->private_data = (void *)dev;

    return 0;
}

/*===========================================================================
 FUNCTION sdgdrv_close

 DESCRIPTION
    The function which is called at the time of close system call.

 DEPENDENCIES
    none

 PARAMETER
    inode:  pointer of inode structure.
    filp:   pointer of file structure.

 RETURN VALUE
    0:      Success
    Not 0:  Failed

 SIDE EFFECTS
    none

===========================================================================*/
static int sdgdrv_close(struct inode* inode, struct file* filp)
{
    SDGDEV_INFO *dev = (SDGDEV_INFO *)filp->private_data;

    if (down_interruptible(&st_sem))
    {
        SDG_DRV_DEBUG("down_interruptible() for read failed");
        return -ERESTARTSYS;
    }
    dev->use = SDGDRV_UNUSED;

    up(&st_sem);

    return 0;
}

static struct file_operations sdgdrv_fops = 
{
    .owner   = THIS_MODULE,
    .read    = sdgdrv_read,
    .write   = sdgdrv_write,
    .poll    = sdgdrv_poll,
    .open    = sdgdrv_open,
    .release = sdgdrv_close,
};

/*===========================================================================
 FUNCTION sdgdrv_init

 DESCRIPTION
    Initialization. To register as a character device to the Kernel.

 DEPENDENCIES
    none

 PARAMETER
    none

 RETURN VALUE
    0:      Success
    Not 0:  Failed

 SIDE EFFECTS
    none

===========================================================================*/
static int __init sdgdrv_init(void)
{
    SDGDEV_INFO *dev = NULL;
    struct device *dev_device = NULL;
    int i = 0;
    int ret = -1;

    ret = alloc_chrdev_region(&st_dev_id, 0, st_devs_num, SDGDRV_DEVICE_NAME);
    if (0 > ret)
    {
        SDG_DRV_DEBUG("Major number not allocated.");
        return -EBUSY;
    }
    st_major = MAJOR(st_dev_id);
    st_minor = MINOR(st_dev_id);

    cdev_init(&st_cdev, &sdgdrv_fops);

    st_cdev.owner = THIS_MODULE;
    st_cdev.ops = &sdgdrv_fops;

    ret = cdev_add(&st_cdev, st_dev_id, st_devs_num);
    if (0 > ret)
    {
        SDG_DRV_DEBUG("cdev registration failed.");
        return -EBUSY;
    }

    st_class = class_create(THIS_MODULE, SDGDRV_DEVICE_NAME);
    if (IS_ERR(st_class))
    {
        SDG_DRV_DEBUG("Error creating class.");
        return -EBUSY;
    }

    dev_device = device_create(st_class, NULL, st_dev_id, NULL, SDGDRV_DEVICE_NAME);
    if (IS_ERR(dev_device))
    {
        SDG_DRV_DEBUG("Error creating device.");
        return -EBUSY;
    }

    sdgdrv_devs = (SDGDEV_INFO *)kmalloc((sizeof(SDGDEV_INFO) * st_devs_num), GFP_KERNEL);
    if (!sdgdrv_devs)
    {
        SDG_DRV_DEBUG("kmalloc() failed.");
        return -ENOMEM;
    }

    for (i = 0; i < st_devs_num; i++)
    {
        dev = sdgdrv_devs + i;
        dev->use = SDGDRV_UNUSED;
        dev->readable = SDGDRV_UNABLE;
        init_waitqueue_head(&dev->wq);
        init_waitqueue_head(&dev->rq);
    }

    sema_init(&st_sem, 1);

    return 0;
}

/*===========================================================================
 FUNCTION sdgdrv_exit

 DESCRIPTION
    Shoutdown processing. Resource releaase.

 DEPENDENCIES
    none

 PARAMETER
    none

 RETURN VALUE
    none

 SIDE EFFECTS
    none

===========================================================================*/
static void __exit sdgdrv_exit(void)
{
    if (!IS_ERR(st_class))
    {
        device_destroy(st_class, st_dev_id);
        class_destroy(st_class);
    }
    cdev_del(&st_cdev);
    unregister_chrdev_region(st_dev_id, st_devs_num);
    kfree(sdgdrv_devs);
}

module_init(sdgdrv_init);
module_exit(sdgdrv_exit);


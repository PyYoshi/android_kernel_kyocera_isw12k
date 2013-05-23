/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 *
 * drivers/input/touchscreen/ts_ctrl.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/

#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/input.h>

#include <linux/namei.h>
#include <linux/compiler.h>
#include <asm/thread_info.h>
#include <asm/string.h>
#include <asm/io.h>
#include <linux/vmalloc.h>

#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/mutex.h>

#define TOUCH_PRINTK_ERROR(fmt, arg...)    printk(KERN_WARNING fmt, ## arg)

#define LOG_BUF_SIZE 8

static DECLARE_WAIT_QUEUE_HEAD(log_wait);
static LIST_HEAD(touch_log_queue_head);
static DEFINE_MUTEX( touch_log_mutex );

extern int ts_diag_data_start(void);
extern int ts_diag_data_end(void);
extern int ts_diag_data_get(void *diag_val);
extern int ts_nv_data_set(void *val);
extern int ts_nv_board_get(void *mode);
extern int ts_diag_log_level(char mode);
extern int ts_diag_event_ctrl(char mode);

struct ts_ctrl {
    int node;
    int major;
    struct device *device;
    struct device *dev;
};

struct ts_ctrl ts_ctrl_info;

struct touch_log_data {
    s16    msg_code;
    unsigned char log_buf[LOG_BUF_SIZE];
};

struct touch_log_msg {
    struct list_head list;
    struct touch_log_data  log_data;
};
unsigned long touch_log_msg_size = sizeof(struct touch_log_msg);
unsigned long touch_log_data_size = sizeof(struct touch_log_data);


void touch_kcc_write_log( unsigned char* buf_p, int buf_size, s16 msg_code )
{
    s16 i;
    struct touch_log_msg *log_msg;
    unsigned char* buf_tmp_p;

    if( msg_code > 0 )
      printk( KERN_ERR "touch_kcc_write_log(msg code=%d)\n", msg_code);

    log_msg = (struct touch_log_msg *)vmalloc(touch_log_msg_size);
    if (!log_msg) {
      return;
    }

    if( buf_size > LOG_BUF_SIZE ){
      buf_size = LOG_BUF_SIZE;
    }

    buf_tmp_p = buf_p;
    for(i=0; i < buf_size ;i++){
      log_msg->log_data.log_buf[i] = *buf_tmp_p;
      buf_tmp_p++;
    }
    log_msg->log_data.msg_code = msg_code;

    mutex_lock(&touch_log_mutex);
    list_add_tail(&log_msg->list, &touch_log_queue_head);
    mutex_unlock(&touch_log_mutex);

    wake_up(&log_wait);

    return;
}

int touch_kcc_get_log( void __user *argp )
{
    int ret;
    struct touch_log_msg *log_msg = NULL;

    while(1)
    {
      ret = wait_event_interruptible( log_wait,
                            !list_empty(&touch_log_queue_head) );
      if(0 == ret)
      {
        break;
      }
    }
    mutex_lock(&touch_log_mutex);
    log_msg = list_first_entry( &touch_log_queue_head,
                              struct touch_log_msg,
                              list );
    if( log_msg == 0 ){
      printk( KERN_ERR "touch_kcc_get_log: fail get list_first_entry\n");
      mutex_unlock(&touch_log_mutex);
      return 0;
    }
    
    list_del(&log_msg->list);
    mutex_unlock(&touch_log_mutex);

    ret = copy_to_user(argp, &(log_msg->log_data), touch_log_data_size);

    vfree(log_msg);

    return 0;
}

static int ts_ctrl_open(struct inode *inode, struct file *file)
{
    return 0;
}

static long ts_ctrl_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    void __user *argp = (void __user *)arg;
    struct ts_diag_type ts_data;
    unsigned char mode = 0;
    unsigned char level = 0;
    struct ts_nv_data_table ts_nv_data;
    int  chg = 0;
	
    switch (cmd) {
        case TS_DIAG_START:
            ret = ts_diag_data_start();
            break;

        case TS_MULTI_GET:
        case TS_COODINATE_GET:
            ret = ts_diag_data_get(&ts_data);

            ret = copy_to_user(argp, &ts_data, sizeof(ts_data));
            if (ret) {
                TOUCH_PRINTK_ERROR("ts_ctrl_ioctl (TS_MULTI_GET) Copy to User Error\n");
                return -EFAULT;
            }
            break;

        case TS_DIAG_END:
            ret = ts_diag_data_end();
            break;

        case EVIOCNVDTS:
            ret = copy_from_user(&ts_nv_data, argp, sizeof(struct ts_nv_data_table));
            if (ret){
                TOUCH_PRINTK_ERROR("ts_ctrl_ioctl (EVIOCNVDTS) Copy from User Error\n");
                return -EFAULT;
            }

            ret = ts_nv_data_set(&ts_nv_data);
            return ret;

        case EVIOCBOCHK:
            ret = ts_nv_board_get(&mode);
            ret = copy_to_user(argp, &mode, sizeof(mode));
            if (ret) {
                TOUCH_PRINTK_ERROR("ts_ctrl_ioctl (EVIOCBOCHK) Copy to User Error\n");
                return -EFAULT;
            }
            return ret;

        case TS_DIAG_LOG_LEVEL:
            ret = copy_from_user(&level, argp, sizeof(unsigned char));
            if (ret){
                TOUCH_PRINTK_ERROR("ts_ctrl_ioctl (TS_DIAG_LOG_LEVEL) Copy from User Error\n");
                return -EFAULT;
            }
            ts_diag_log_level(level);
            return ret;

        case TS_DIAG_EVENT_CTRL:
            ret = copy_from_user(&mode, argp, sizeof(unsigned char));
            if (ret){
                TOUCH_PRINTK_ERROR("ts_ctrl_ioctl (TS_DIAG_EVENT_CTRL) Copy from User Error\n");
                return -EFAULT;
            }
            ts_diag_event_ctrl(mode);
            return ret;

        case TS_LOG_GET:
            {
              extern int touch_kcc_get_log( void __user *argp );
              ret = touch_kcc_get_log( argp );
            }
            return ret;
    
        case TS_SET_POWER_CHG:
            {
              extern int ts_touch_power_charge( int chg );
    
              ret = copy_from_user(&chg, argp, sizeof(int));
              if (ret){
                  TOUCH_PRINTK_ERROR("ts_ctrl_ioctl (TS_SET_POWER_CHG) Copy from User Error\n");
                  return -EFAULT;
              }
              ret = ts_touch_power_charge( chg );
            }
            return ret;

        default:
            ret = 0;
    }
    return ret;
}

static int ts_ctrl_release(struct inode *inode, struct file *file)
{
    return 0;
}

static const struct file_operations ts_ctrl_fops = {
    .owner            = THIS_MODULE,
    .open             = ts_ctrl_open,
    .release          = ts_ctrl_release,
    .unlocked_ioctl   = ts_ctrl_ioctl
};

struct class *ts_ctrl_class;
EXPORT_SYMBOL(ts_ctrl_class);

static int __init ts_ctrl_init(void)
{
    ts_ctrl_info.major = register_chrdev(0,"ts_ctrl",&ts_ctrl_fops);
    if(ts_ctrl_info.major < 0)
        TOUCH_PRINTK_ERROR("unable to get major for ts_ctrl devs\n");

    ts_ctrl_class = class_create(THIS_MODULE, "ts_ctrl");
    if (IS_ERR(ts_ctrl_class)) {
        TOUCH_PRINTK_ERROR(KERN_WARNING "Unable to create ts class; errno = %ld\n", PTR_ERR(ts_ctrl_class));
        ts_ctrl_class = NULL;
    }

    ts_ctrl_info.dev = device_create(ts_ctrl_class, ts_ctrl_info.device, MKDEV(ts_ctrl_info.major,0), NULL, "ts_ctrl%d", 0);

    if (IS_ERR(ts_ctrl_info.dev)) {
        TOUCH_PRINTK_ERROR(KERN_WARNING "Unable to create device for framebuffer %d; errno = %ld\n", 0, PTR_ERR(ts_ctrl_info.dev));
        ts_ctrl_info.dev = NULL;
    }
    return 0;
}

static void __exit ts_ctrl_exit(void)
{
    class_destroy(ts_ctrl_class);
    unregister_chrdev(ts_ctrl_info.major, "ts_ctrl");
}

module_init(ts_ctrl_init);
module_exit(ts_ctrl_exit);

MODULE_AUTHOR("KYOCERA");
MODULE_DESCRIPTION("TouchScreen Ctrl");
MODULE_LICENSE("GPL");

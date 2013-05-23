/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
drivers/kwimax_onoff/kwimax_onoff.c

This software is contributed or developed by KYOCERA Corporation.
(C) 2011 KYOCERA Corporation
(C) 2012 KYOCERA Corporation
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/* This program is free software; you can redistribute it and/or modify
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
 
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include <linux/gpio.h>
#include <linux/delay.h>

#ifdef FEATURE_KCC_WIMAX_CHG_PAFAULT_PORT
#include <mach/pmic.h>
#else
#include <linux/mfd/pm8058.h>
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("WiMAX ON/OFF Driver");

#ifdef KWIMAX_ONOFF_DEBUG
    #define KWIMAX_ONOFF_PRINTK( fmt, args... ) printk( KERN_DEBUG "%s: "fmt, __func__, ## args );
#else
    #define KWIMAX_ONOFF_PRINTK( fmt, args... )
#endif

#ifdef FEATURE_KCC_WIMAX_BOOT_APP
#define KWIMAX_ONOFF_ERR_PRINTK( fmt, args... ) printk( KERN_ERR "ERR %s: "fmt, __func__, ## args );
#endif // FEATURE_KCC_WIMAX_BOOT_APP


#ifdef FEATURE_KCC_WIMAX_CHG_PAFAULT_PORT
#define KWIMAX_ONOFF_MPP_VBATT_PA_DET        2
#else
#define KWIMAX_ONOFF_GPIO_VBATT_PA_WiMAX     16
#endif

#define KWIMAX_ONOFF_GPIO_WIMAX_RST_N        55
#define KWIMAX_ONOFF_GPIO_VWIMAX_ON          165

#define KWIMAX_ONOFF_GPIO_LOW                0
#define KWIMAX_ONOFF_GPIO_HIGH               1

#define KWIMAX_ONOFF_WAIT_WIMAX_ON_HIGH      5
#define KWIMAX_ONOFF_WAIT_WIMAX_RST_N_HIGH   20

#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)


#ifdef FEATURE_KCC_WIMAX_BOOT_APP

#define KWIMAX_ONOFF_CMD_MAX                 32
#define KWIMAX_ONOFF_POWER_ON               "power on wimax"
#define KWIMAX_ONOFF_POWER_OFF              "power off wimax"

#define KWIMAX_ONOFF_RET_PAFAULT             91

#endif // FEATURE_KCC_WIMAX_BOOT_APP


#ifdef FEATURE_KCC_WIMAX_BOOT_APP
static ssize_t kwimax_onoff_write(struct file *filep, const char __user *buff, size_t count, loff_t *offp);
static ssize_t kwimax_onoff_power_on( void );
static ssize_t kwimax_onoff_power_off( void );
#endif // FEATURE_KCC_WIMAX_BOOT_APP



/*===========================================================================
FUNCTION kwimax_onoff_init

DESCRIPTION
  initialize driver.

DEPENDENCIES
  None

RETURN VALUE
  0:      OK
  Not 0:  NG

SIDE EFFECTS

===========================================================================*/
#ifdef FEATURE_KCC_WIMAX_BOOT_APP

static const struct file_operations kwimax_onoff_fops_g =
{
    .owner       = THIS_MODULE,
    .write       = kwimax_onoff_write,
};

static struct miscdevice kwimax_onoff_dev_g = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "wimaxonoff",
    .fops = &kwimax_onoff_fops_g,
};

static int32_t kwimax_onoff_init( void )
{
    
    int32_t ret=0;
    
    KWIMAX_ONOFF_PRINTK( "start\n" );
    
#ifdef KWIMAX_ONOFF_DEBUG001
    ret = -1;
#else
    ret = misc_register( &kwimax_onoff_dev_g );
#endif
    if( ret != 0 )
    {
        ret = -1;
        KWIMAX_ONOFF_ERR_PRINTK( "misc_register ng ret=%d\n", ret );
    }
    
    KWIMAX_ONOFF_PRINTK( "end ret=%d\n", ret );
    
    return( ret );
    
}

#else  // FEATURE_KCC_WIMAX_BOOT_APP

static int32_t kwimax_onoff_init( void )
{
    int32_t ret=0;
    int32_t pa;

    KWIMAX_ONOFF_PRINTK( "%s:start\n", __func__ );

#ifdef FEATURE_KCC_WIMAX_CHG_PAFAULT_PORT
    pa = pmic_hwp_get_value( KWIMAX_ONOFF_MPP_VBATT_PA_DET );
    KWIMAX_ONOFF_PRINTK( "VBATT_PA_DET=%d\n", pa );
#else
    pa = gpio_get_value ( PM8058_GPIO_PM_TO_SYS(KWIMAX_ONOFF_GPIO_VBATT_PA_WiMAX) );
    KWIMAX_ONOFF_PRINTK( "%s:VBATT_PA_WiMAX=%d\n", __func__, pa );
#endif

    if( pa == KWIMAX_ONOFF_GPIO_HIGH )
    {
        gpio_set_value( KWIMAX_ONOFF_GPIO_VWIMAX_ON,   KWIMAX_ONOFF_GPIO_HIGH );
        msleep( KWIMAX_ONOFF_WAIT_WIMAX_ON_HIGH );
        gpio_set_value( KWIMAX_ONOFF_GPIO_WIMAX_RST_N, KWIMAX_ONOFF_GPIO_HIGH );
        msleep( KWIMAX_ONOFF_WAIT_WIMAX_RST_N_HIGH );
    }
    else
    {
        ret = -1;
    }

    KWIMAX_ONOFF_PRINTK( "%s:end ret=%d\n", __func__, ret );
    
    return( ret );
}

#endif // FEATURE_KCC_WIMAX_BOOT_APP


/*===========================================================================
FUNCTION kwimax_onoff_exit

DESCRIPTION
  shut down the driver.

DEPENDENCIES
  None

RETURN VALUE
  0:      OK

SIDE EFFECTS

===========================================================================*/
static void kwimax_onoff_exit( void )
{
    KWIMAX_ONOFF_PRINTK( "start\n" );

#ifdef FEATURE_KCC_WIMAX_BOOT_APP

    misc_deregister( &kwimax_onoff_dev_g );

#else  // FEATURE_KCC_WIMAX_BOOT_APP

    gpio_set_value( KWIMAX_ONOFF_GPIO_WIMAX_RST_N, KWIMAX_ONOFF_GPIO_LOW );
    gpio_set_value( KWIMAX_ONOFF_GPIO_VWIMAX_ON,   KWIMAX_ONOFF_GPIO_LOW );

#endif // FEATURE_KCC_WIMAX_BOOT_APP

    KWIMAX_ONOFF_PRINTK( "exit end\n" );
}

module_init(kwimax_onoff_init);
module_exit(kwimax_onoff_exit);


#ifdef FEATURE_KCC_WIMAX_BOOT_APP

/*===========================================================================
FUNCTION kwimax_onoff_write

DESCRIPTION
  write user data.

DEPENDENCIES
  file  [in]   file information
  buff  [in]   user data
  count [in]   user data size

RETURN VALUE
  0:      OK
  91:     PA failure
  -ENOTTY:undefined command
  -EFAULT:abnormal end

SIDE EFFECTS

===========================================================================*/
static ssize_t kwimax_onoff_write(struct file *filep, const char __user *buff, size_t count, loff_t *offp)
{
    ssize_t ret = 0;
    char cmd[KWIMAX_ONOFF_CMD_MAX];

    KWIMAX_ONOFF_PRINTK( "start count=%u\n", count );

    if( count >= KWIMAX_ONOFF_CMD_MAX )
    {
        KWIMAX_ONOFF_ERR_PRINTK( "count over flow count=%u\n",count );
        return( -ENOTTY );
    }

#ifdef KWIMAX_ONOFF_DEBUG002
    ret = -1;
#else
    memset( cmd, 0x00, KWIMAX_ONOFF_CMD_MAX );
    ret = copy_from_user( cmd, buff, count );
#endif // KWIMAX_ONOFF_DEBUG002
    if( ret != 0 )
    {
        KWIMAX_ONOFF_ERR_PRINTK( "count copy_from_user ng ret=%d\n",ret );
        return( -EFAULT );
    }
    KWIMAX_ONOFF_PRINTK( "cmd=%s\n", cmd );

    if( strncmp( cmd, KWIMAX_ONOFF_POWER_ON, count ) == 0 )
    {
        ret= kwimax_onoff_power_on();
    }
    else if( strncmp( cmd, KWIMAX_ONOFF_POWER_OFF, count ) == 0 )
    {
        ret= kwimax_onoff_power_off();
    }
    else
    {
        KWIMAX_ONOFF_ERR_PRINTK( "undefined command comman=%s\n", cmd );
        ret = -ENOTTY;
    }

    KWIMAX_ONOFF_PRINTK( "write end ret=%d\n", ret );

    return( ret );
}

/*===========================================================================
FUNCTION kwimax_onoff_power_on

DESCRIPTION
  run a check on PA.
  power on the WiMAX chip.
  

DEPENDENCIES
  None

RETURN VALUE
  0:      OK
  91:     PA failure

SIDE EFFECTS

===========================================================================*/
static ssize_t kwimax_onoff_power_on( void )
{
    ssize_t ret = 0;
    int32_t pa;

    KWIMAX_ONOFF_PRINTK( "start\n" );

#ifdef FEATURE_KCC_WIMAX_CHG_PAFAULT_PORT
    pa = pmic_hwp_get_value( KWIMAX_ONOFF_MPP_VBATT_PA_DET );
    KWIMAX_ONOFF_PRINTK( "VBATT_PA_DET=%d\n", pa );
#else
    pa = gpio_get_value ( PM8058_GPIO_PM_TO_SYS(KWIMAX_ONOFF_GPIO_VBATT_PA_WiMAX) );
    KWIMAX_ONOFF_PRINTK( "VBATT_PA_WiMAX=%d\n", pa );
#endif

    if( pa == KWIMAX_ONOFF_GPIO_HIGH )
    {
        gpio_set_value( KWIMAX_ONOFF_GPIO_VWIMAX_ON,   KWIMAX_ONOFF_GPIO_HIGH );
        msleep( KWIMAX_ONOFF_WAIT_WIMAX_ON_HIGH );
        gpio_set_value( KWIMAX_ONOFF_GPIO_WIMAX_RST_N, KWIMAX_ONOFF_GPIO_HIGH );
        msleep( KWIMAX_ONOFF_WAIT_WIMAX_RST_N_HIGH );
    }
    else
    {
        ret = KWIMAX_ONOFF_RET_PAFAULT;
        KWIMAX_ONOFF_ERR_PRINTK( "PA Failure!" );
    }

    KWIMAX_ONOFF_PRINTK( "end ret=%d\n", ret );

    return( ret );
}

/*===========================================================================
FUNCTION kwimax_onoff_power_off

DESCRIPTION
  power off the WiMAX chip.

DEPENDENCIES
  None

RETURN VALUE
  0:      OK

SIDE EFFECTS

===========================================================================*/
static ssize_t kwimax_onoff_power_off( void )
{
    KWIMAX_ONOFF_PRINTK( "start\n" );

    gpio_set_value( KWIMAX_ONOFF_GPIO_WIMAX_RST_N, KWIMAX_ONOFF_GPIO_LOW );
    gpio_set_value( KWIMAX_ONOFF_GPIO_VWIMAX_ON,   KWIMAX_ONOFF_GPIO_LOW );

    KWIMAX_ONOFF_PRINTK( "end\n" );
    
    return( 0 );
}
#endif // FEATURE_KCC_WIMAX_BOOT_APP

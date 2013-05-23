/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 */
/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h> 
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/android_alarm.h>
#include <linux/wakelock.h>

#include <../../arch/arm/mach-msm/include/mach/pmic.h>

#define   LED_DEBUG 0

#if LED_DEBUG
#define DEBUG_PRINT( arg... )   printk( KERN_INFO "[KCLIGHT][DRV]:" arg )
#else
#define DEBUG_PRINT( arg... )
#endif

#define DEBUG_PRINT_ERR( arg... )   printk( KERN_ERR "[KCLIGHT][DRV]:" arg )

#define LED_INDIVIDUAL_ON

enum light_index_enum{
    MOBILELIGHT_INDEX = 0,
    LEDLIGHT_INDEX,
    LIGHT_INDEX_MAX
};

#define LED_DRV_NAME                        "AS3368"

#define LED_SLAVE_ADDR                      (0x84 >> 1)

#define I2C_RETRIES_NUM                     5

#define LED_WRITE_BUF_LEN                   2

#define MOBILELIGHT_INFO                    "mobilelightinfo"
#define LED_INFO                            "ledinfo"

#define MOBILELIGHT_MAX_BRIGHT_VAL          0xFFFFFFFFu
#define LEDLIGHT_MAX_BRIGHT_VAL             0xFFFFFFFFu

#define REG_ADDR_00                         0x00
#define REG_ADDR_01                         0x01
#define REG_ADDR_02                         0x02
#define REG_ADDR_03                         0x03
#define REG_ADDR_04                         0x04
#define REG_ADDR_05                         0x05
#define REG_ADDR_06                         0x06
#define REG_ADDR_07                         0x07
#define REG_ADDR_15                         0x15
#define REG_ADDR_16                         0x16
#define REG_ADDR_18                         0x18
#define REG_ADDR_1B                         0x1B
#define REG_ADDR_1D                         0x1D
#define REG_ADDR_23                         0x23
#define REG_ADDR_26                         0x26
#define REG_ADDR_29                         0x29
#define REG_ADDR_40                         0x40
#define REG_ADDR_41                         0x41
#define REG_ADDR_42                         0x42
#define REG_ADDR_43                         0x43

#define REG_ADDR_17                         0x17
#define REG_ADDR_19                         0x19
#define REG_ADDR_1A                         0x1A
#define REG_ADDR_1C                         0x1C


#define REG_ADDR_00_INIT                    0x00
#define REG_ADDR_01_INIT                    0x00
#define REG_ADDR_01_MASK                    0xFF
#define REG_ADDR_02_INIT                    0x40
#define REG_ADDR_03_INIT                    0x00
#define REG_ADDR_04_INIT                    0x00
#define REG_ADDR_05_INIT                    0x00
#define REG_ADDR_06_INIT                    0x02
#define REG_ADDR_07_INIT                    0x00
#define REG_ADDR_15_INIT                    0x00
#define REG_ADDR_16_INIT                    0x00
#define REG_ADDR_18_INIT                    0x18
#define REG_ADDR_1B_INIT                    0x00
#define REG_ADDR_1B_ON                      0x02
#define REG_ADDR_1B_OFF                     0x00
#define REG_ADDR_1B_MASK                    0x07
#define REG_ADDR_1D_INIT                    0x00
#define REG_ADDR_23_INIT                    0x11
#define REG_ADDR_26_INIT                    0x00
#define REG_ADDR_29_INIT                    0x11
#define REG_ADDR_40_INIT                    0x00
#define REG_ADDR_41_INIT                    0x00
#define REG_ADDR_42_INIT                    0x00
#define REG_ADDR_43_INIT                    0x00

#define REG_ADDR_17_INIT                    0x00
#define REG_ADDR_19_INIT                    0x00
#define REG_ADDR_1A_INIT                    0x00
#define REG_ADDR_1C_INIT                    0x00


#define INDLED_OFF_VAL                      0x00
#define INDLED_RED_ON_VAL                   0x01
#define INDLED_GREEN_ON_VAL                 0x04
#define INDLED_BLUE_ON_VAL                  0x10
#define INDLED_PATARN_RED_ON_VAL            0x03
#define INDLED_PATARN_GREEN_ON_VAL          0x0C
#define INDLED_PATARN_BLUE_ON_VAL           0x30

#define LEDLIGHT_COL_BLACK                  0x000000

#define LEDLIGHT_CURVAL_MAX                 0x0F
#define LEDLIGHT_CURVAL_MIX                 0x05
#define LEDLIGHT_CURVAL_OFF                 0x00
#define LEDLIGHT_CURVAL_RED_MAX             0x0A
#define LEDLIGHT_CURVAL_GREEN_MAX           0x0A
#define LEDLIGHT_CURVAL_BLUE_MAX            0x0F

#define MOBILELIGHT_OFF_MODE                0x00
#define MOBILELIGHT_BATTTERM_LOW_MODE       0x02
#define MOBILELIGHT_BATTTERM_HIGH_MODE      0x01
#define MOBILELIGHT_VIEWING_MODE            0x03

#define MOBILELIGHT_BATTTERM_LOW_VAL        0x15
#define MOBILELIGHT_BATTTERM_HIGH_VAL       0x0A
#define MOBILELIGHT_VIEWING_VAL             0x00

#define FLG_ON                              0x01
#define FLG_OFF                             0x00

#define LED_ON                              0x01
#define LED_OFF                             0x00

#define LEDLIGHT_BLINK_NUM                  (4)
#define LEDLIGHT    'L'
#define LEDLIGHT_SET_BLINK                  _IOW(LEDLIGHT, 0, T_LEDLIGHT_IOCTL)
#define MIN_BLINK_ALARM_SET_DELAY  100

#define CARGE_PUMP_SETTING_ON_VAL           0x04
#define CARGE_PUMP_ON_VAL                   0x15
#define CARGE_PUMP_SETTING_OFF_VAL          0x00
#define CARGE_PUMP_OFF_VAL                  0x11

#define LED_LDO_GPIO                        96

#define LED_GPIO_HIGH_VAL                   1
#define LED_GPIO_LOW_VAL                    0

#define LED_PMLED_HIGH_VAL                  40
#define LED_PMLED_LOW_VAL                   20
#define LED_PMLED_INIT_VAL                  0

#define LED_COLOR_PATTREN_RED               0
#define LED_COLOR_PATTREN_GREEN             1
#define LED_COLOR_PATTREN_BLUE              2
#define LED_COLOR_PATTREN_RGB               3

#define AS3368_TON_VAL1             (0.04 * 1000)
#define AS3368_TON_VAL2             (0.07 * 1000)
#define AS3368_TON_VAL3             (0.14 * 1000)
#define AS3368_TON_VAL4             (0.27 * 1000)
#define AS3368_TON_VAL5             (0.53 * 1000)
#define AS3368_TON_VAL6             (1.10 * 1000)
#define AS3368_TON_VAL7             (2.10 * 1000)
#define AS3368_TON_VAL8             (4.20 * 1000)
#define AS3368_TOFF_VAL1            (0.08 * 1000)
#define AS3368_TOFF_VAL2            (0.15 * 1000)
#define AS3368_TOFF_VAL3            (0.28 * 1000)
#define AS3368_TOFF_VAL4            (0.54 * 1000)
#define AS3368_TOFF_VAL5            (1.10 * 1000)
#define AS3368_TOFF_VAL6            (2.10 * 1000)
#define AS3368_TOFF_VAL7            (4.20 * 1000)
#define AS3368_TOFF_VAL8            (8.40 * 1000)


enum color_pattern_enum{
    COLOR_PATTERN_1 = 0,
    COLOR_PATTERN_2,
    COLOR_PATTERN_3,
    COLOR_PATTERN_4,
    COLOR_PATTERN_5,
    COLOR_PATTERN_6,
    COLOR_PATTERN_7,
    COLOR_PATTERN_MAX
};

static uint32_t const gul_color_pattern_val[COLOR_PATTERN_MAX][LED_COLOR_PATTREN_RGB] = {
    { LEDLIGHT_CURVAL_OFF,     LEDLIGHT_CURVAL_OFF,       LEDLIGHT_CURVAL_BLUE_MAX },
    { LEDLIGHT_CURVAL_RED_MAX, LEDLIGHT_CURVAL_OFF,       LEDLIGHT_CURVAL_OFF      },
    { LEDLIGHT_CURVAL_OFF,     LEDLIGHT_CURVAL_GREEN_MAX, LEDLIGHT_CURVAL_OFF      },
    { LEDLIGHT_CURVAL_MIX,     LEDLIGHT_CURVAL_OFF,       LEDLIGHT_CURVAL_MIX      },
    { LEDLIGHT_CURVAL_OFF,     LEDLIGHT_CURVAL_MIX,       LEDLIGHT_CURVAL_MIX      },
    { LEDLIGHT_CURVAL_MIX,     LEDLIGHT_CURVAL_MIX,       LEDLIGHT_CURVAL_OFF      },
    { LEDLIGHT_CURVAL_MIX,     LEDLIGHT_CURVAL_MIX,       LEDLIGHT_CURVAL_MIX      }
};

static uint32_t const gul_pattern_onoff_val[COLOR_PATTERN_MAX][LED_COLOR_PATTREN_RGB] = {
    { INDLED_OFF_VAL    , INDLED_OFF_VAL      , INDLED_BLUE_ON_VAL  },
    { INDLED_RED_ON_VAL , INDLED_OFF_VAL      , INDLED_OFF_VAL      },
    { INDLED_OFF_VAL    , INDLED_GREEN_ON_VAL , INDLED_OFF_VAL      },
    { INDLED_RED_ON_VAL , INDLED_OFF_VAL      , INDLED_BLUE_ON_VAL  },
    { INDLED_OFF_VAL    , INDLED_GREEN_ON_VAL , INDLED_BLUE_ON_VAL  },
    { INDLED_RED_ON_VAL , INDLED_GREEN_ON_VAL , INDLED_OFF_VAL      },
    { INDLED_RED_ON_VAL , INDLED_GREEN_ON_VAL , INDLED_BLUE_ON_VAL  }
};

static uint32_t const gul_blink_pattern_onoff_val[COLOR_PATTERN_MAX][LED_COLOR_PATTREN_RGB] = {
    { INDLED_OFF_VAL           , INDLED_OFF_VAL             , INDLED_PATARN_BLUE_ON_VAL  },
    { INDLED_PATARN_RED_ON_VAL , INDLED_OFF_VAL             , INDLED_OFF_VAL             },
    { INDLED_OFF_VAL           , INDLED_PATARN_GREEN_ON_VAL , INDLED_OFF_VAL             },
    { INDLED_PATARN_RED_ON_VAL , INDLED_OFF_VAL             , INDLED_PATARN_BLUE_ON_VAL  },
    { INDLED_OFF_VAL           , INDLED_PATARN_GREEN_ON_VAL , INDLED_PATARN_BLUE_ON_VAL  },
    { INDLED_PATARN_RED_ON_VAL , INDLED_PATARN_GREEN_ON_VAL , INDLED_OFF_VAL             },
    { INDLED_PATARN_RED_ON_VAL , INDLED_PATARN_GREEN_ON_VAL , INDLED_PATARN_BLUE_ON_VAL  }
};

static uint32_t const gul_delay_time[8][2] = {
    { AS3368_TON_VAL1 , AS3368_TOFF_VAL1 },
    { AS3368_TON_VAL2 , AS3368_TOFF_VAL2 },
    { AS3368_TON_VAL3 , AS3368_TOFF_VAL3 },
    { AS3368_TON_VAL4 , AS3368_TOFF_VAL4 },
    { AS3368_TON_VAL5 , AS3368_TOFF_VAL5 },
    { AS3368_TON_VAL6 , AS3368_TOFF_VAL6 },
    { AS3368_TON_VAL7 , AS3368_TOFF_VAL7 },
    { AS3368_TON_VAL8 , AS3368_TOFF_VAL8 }
};


static uint8_t    guc_led_on_flg = FLG_OFF;
static uint8_t    guc_led_pattern_flg = FLG_OFF;
static uint8_t  guc_reg0_val = FLG_OFF;

enum blink_control_enum{
    NO_BLINK_REQUEST = 0,
    BLINK_HARD_REQUEST,
    BLINK_SOFT_REQUEST_ON,
    BLINK_SOFT_REQUEST_OFF
};

struct light_led_data_type {
    struct led_classdev     st_cdev;
    struct i2c_client       *pst_client ;
    uint32_t                ul_id;
    uint32_t                ul_value;
    uint32_t                ul_flags;
    struct work_struct      work;
    struct mutex            lock;
    spinlock_t              value_lock;
    uint8_t                 uc_indled_val;

    struct alarm            led_alarm;  
    struct wake_lock        work_wake_lock;
    uint32_t                blink_control; 
    uint32_t                blink_on_delay;
    uint32_t                blink_off_delay;
    uint32_t                blink_off_color;
    ktime_t                 last_elapsed_time;
    uint32_t                saved_color_value;
    uint32_t                lightkind;
    uint8_t                 uc_led_red_val;
    uint8_t                 uc_led_green_val;
    uint8_t                 uc_led_blue_val;

};

static struct light_led_data_type *gpst_light_led_data = NULL;

static uint32_t const gul_color_pattern_value[COLOR_PATTERN_MAX] = {
    0x000000FF,
    0x00FF0000,
    0x0000FF00,
    0x00FF00FF,
    0x0000FFFF,
    0x00FFFF00,
    0x00C0C0C0
};

typedef struct _t_ledlight_ioctl
{
    uint32_t data[LEDLIGHT_BLINK_NUM];
}T_LEDLIGHT_IOCTL;


static void led_3colorled_on_set(struct light_led_data_type *pst_light_led_data );
static void led_3colorled_off_set(struct light_led_data_type *pst_light_led_data);
static void led_light_on_set(struct light_led_data_type *pst_light_led_data);
static void led_pattern_on_set(struct light_led_data_type *pst_light_led_data );
static void led_pattern_off_set(struct light_led_data_type *pst_light_led_data);


static void led_alarm(struct alarm *alarm)
{
    struct light_led_data_type *pst_light_led_data =
        container_of(alarm, struct light_led_data_type, led_alarm);
    DEBUG_PRINT("LED DEBUG:led_alarm Call\n");
    wake_lock(&pst_light_led_data->work_wake_lock);
    schedule_work(&pst_light_led_data->work);
}

static void led_set_alarm(struct light_led_data_type *pst_light_led_data, int32_t msec)
{
    
    int32_t interval_sec;
    int32_t interval_nsec; 
    ktime_t interval ;
    ktime_t next;
    
    if(msec < MIN_BLINK_ALARM_SET_DELAY)
    {
        msec = MIN_BLINK_ALARM_SET_DELAY;
    }
    interval_sec = msec/MSEC_PER_SEC;
    interval_nsec= (msec%MSEC_PER_SEC) * NSEC_PER_MSEC; 
    interval = ktime_set(interval_sec, interval_nsec);
    
    pst_light_led_data->last_elapsed_time = alarm_get_elapsed_realtime();

    next = ktime_add(pst_light_led_data->last_elapsed_time, interval);

    alarm_start_range(&pst_light_led_data->led_alarm, next, next);
}

static int32_t ledlight_write (struct i2c_client *pst_client, uint8_t uc_reg, uint8_t uc_val)
{
    int32_t lret = 0;
    uint32_t ul_tryn = 0 ;
    struct light_led_data_type *pst_light_led_data = i2c_get_clientdata(pst_client);
    u8 ucwritebuf[LED_WRITE_BUF_LEN] ;
    uint8_t uc_len;
    struct i2c_msg msg ;

    DEBUG_PRINT("%s() start uc_reg=0x%02x uc_val=0x%02x\n",
            __func__,
            uc_reg,
            uc_val) ;

    DEBUG_PRINT("%s() start pst_client=0x%08x pst_light_led_data=0x%08x\n",
            __func__, 
            (unsigned int)pst_client ,
            (unsigned int)pst_light_led_data);


    DEBUG_PRINT("%s() start uc_reg=0x%02x uc_val=0x%02x, guc_reg0_val=0x%02x\n",
            __func__,
            uc_reg,
            uc_val,
        guc_reg0_val);
    if( REG_ADDR_00 == uc_reg )
    {
        if( guc_reg0_val == uc_val )
        {
            goto noupdate ;
        }
        else
        {
            guc_reg0_val = uc_val ;
        }
    }

    ucwritebuf[0] = uc_reg ;
    ucwritebuf[1] = uc_val ;
    uc_len = sizeof(ucwritebuf) ;

    msg.addr = pst_light_led_data->pst_client->addr;
    msg.flags = 0;
    msg.len = uc_len;
    msg.buf = &ucwritebuf[0];

    DEBUG_PRINT("%s() start uc_reg=0x%02x uc_val=0x%02x, guc_reg0_val=0x%02x\n",
            __func__,
            uc_reg,
            uc_val,
            guc_reg0_val);

    do{
        DEBUG_PRINT("%s() i2c_transfer() call\n", __func__);
        lret = i2c_transfer(pst_light_led_data->pst_client->adapter, &msg, 1) ;
        DEBUG_PRINT("%s() i2c_transfer() call end lret=%d\n", __func__ , lret );
    }while ((lret != 1) && (++ul_tryn < I2C_RETRIES_NUM));

    if(lret != 1){
        DEBUG_PRINT_ERR("%s(): uc_reg 0x%02x, uc_val 0x%02x lret %d\n",
                __func__, 
                ucwritebuf[0],
                ucwritebuf[1],
                lret);
        lret = -1 ;
    }else{
        lret = 0 ;
    }
    DEBUG_PRINT("%s() end\n", __func__);

    return lret;

noupdate:
    return lret;
}

static int32_t led_color_pattern_check(uint32_t ul_colorval)
{
    int32_t lret = COLOR_PATTERN_MAX ;
    int32_t lmatch_pattern = 0;

    DEBUG_PRINT("%s() start \n", __func__ );

    for ( lmatch_pattern=0; lmatch_pattern<COLOR_PATTERN_MAX; lmatch_pattern++ )
    {
        if ( ul_colorval == gul_color_pattern_value[lmatch_pattern] )
        {
            DEBUG_PRINT("%s() color pattern match %d \n", __func__, lmatch_pattern);
            lret = lmatch_pattern;
            break;
        }
    }

    DEBUG_PRINT("%s() end\n", __func__ );

    return lret ;
}

static void led_set(struct light_led_data_type *pst_light_led_data)
{
    uint32_t ul_colorval ;
    uint8_t uc_color_red, uc_color_green, uc_color_blue ;
    uint32_t ul_color_pattern_val;
    int32_t i;
    int32_t lret = 0 ;
    uint8_t uc_ptn_on = 0;
    uint8_t uc_reg18_val = 0;


    DEBUG_PRINT("%s() start \n", __func__ );

    pst_light_led_data->uc_indled_val = INDLED_OFF_VAL ;

    if( LEDLIGHT_COL_BLACK == (pst_light_led_data->ul_value&0x00FFFFFF)){
        led_pattern_off_set( pst_light_led_data );
        guc_led_on_flg = FLG_OFF;
        DEBUG_PRINT("%s() guc_led_on_flg(%d) \n", __func__, guc_led_on_flg ) ;
    }
    else {
        ul_colorval = ( pst_light_led_data->ul_value&0x00FFFFFF ) ;

        uc_color_red = (u8)((ul_colorval>>(16)) & 0xff) ;
        uc_color_green = (u8)((ul_colorval>>(8)) & 0xff) ;
        uc_color_blue = (u8)((ul_colorval) & 0xff) ;

        if( uc_color_red <= LEDLIGHT_CURVAL_MAX )
        {
            pst_light_led_data->uc_led_red_val = uc_color_red ;
        }
        else
        {
            pst_light_led_data->uc_led_red_val = LEDLIGHT_CURVAL_MAX ;
        }
        if( uc_color_green <= LEDLIGHT_CURVAL_MAX )
        {
            pst_light_led_data->uc_led_green_val = uc_color_green ;
        }
        else
        {
            pst_light_led_data->uc_led_green_val = LEDLIGHT_CURVAL_MAX ;
        }
        if( uc_color_blue <= LEDLIGHT_CURVAL_MAX )
        {
            pst_light_led_data->uc_led_blue_val = uc_color_blue ;
        }
        else
        {
            pst_light_led_data->uc_led_blue_val = LEDLIGHT_CURVAL_MAX ;
        }
        DEBUG_PRINT("%s() uc_color_red=0x%x uc_color_red_val=0x%02x\n",
                __func__,
                uc_color_red,
                pst_light_led_data->uc_led_red_val);

        DEBUG_PRINT("%s() uc_color_green=0x%02x uc_color_green_val=0x%02x\n",
                __func__,
                uc_color_green,
                pst_light_led_data->uc_led_green_val);

        DEBUG_PRINT("%s() uc_color_blue=0x%02x uc_color_blue_val=0x%02x\n",
                __func__,
                uc_color_blue,
                pst_light_led_data->uc_led_blue_val);

        ul_color_pattern_val = led_color_pattern_check(ul_colorval);
        if ( gpst_light_led_data[LEDLIGHT_INDEX].blink_control == BLINK_HARD_REQUEST)
        {
            uc_reg18_val = 0;
            uc_reg18_val = ( gpst_light_led_data[LEDLIGHT_INDEX].blink_off_delay << 3) | gpst_light_led_data[LEDLIGHT_INDEX].blink_on_delay;
            DEBUG_PRINT("%s() blink_off_delay=0x%02x \n",
                    __func__,
                    gpst_light_led_data[LEDLIGHT_INDEX].blink_off_delay);

            DEBUG_PRINT("%s() blink_on_delay=0x%02x \n",
                    __func__,
                    gpst_light_led_data[LEDLIGHT_INDEX].blink_on_delay);

            DEBUG_PRINT("%s() ul_onoff_time_val=0x%08x \n",
                    __func__,
                    uc_reg18_val);
            lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_18 , uc_reg18_val );
            if (lret) {
                DEBUG_PRINT_ERR("%s() REG_ADDR_18 (0x%02x) \n",
                        __func__,
                        uc_reg18_val);
                return;
            }
                
            if ( ul_color_pattern_val != COLOR_PATTERN_MAX )
            {
                for( i =0 ;i < LED_COLOR_PATTREN_RGB ;i++ )
                {
                    pst_light_led_data->uc_indled_val |= gul_blink_pattern_onoff_val[ul_color_pattern_val][i];
                    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_02 + i, gul_color_pattern_val[ul_color_pattern_val][i]);
                    if (lret) {
                        DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value uc_color_val(i) (0x%02x) \n",
                                __func__,
                                gul_color_pattern_val[ul_color_pattern_val][i]);
                        return;
                    }
                }
            }else
            {
                led_pattern_on_set( pst_light_led_data );
            }
            uc_ptn_on = REG_ADDR_1B_ON & REG_ADDR_1B_MASK;
            
            lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_1B , uc_ptn_on);
            if (lret) {
                DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value uc_ptn_on (0x%02x) \n",
                        __func__,
                        uc_ptn_on);
                return;
            }
            
            led_light_on_set( pst_light_led_data );
            guc_led_pattern_flg = FLG_ON;
        }
        else
        {
            
            if ( ul_color_pattern_val != COLOR_PATTERN_MAX )
            {
                for( i =0 ;i < LED_COLOR_PATTREN_RGB ;i++ )
                {
                    pst_light_led_data->uc_indled_val |= gul_pattern_onoff_val[ul_color_pattern_val][i];
                    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_02 + i, gul_color_pattern_val[ul_color_pattern_val][i]);
                        if (lret) {
                        DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value uc_color_val(i) (0x%02x) \n",
                                __func__,
                                gul_color_pattern_val[ul_color_pattern_val][i]);
                        return;
                    }
                }
            }else
            {
                led_3colorled_on_set( pst_light_led_data );
            }
            led_light_on_set( pst_light_led_data );
            guc_led_on_flg = FLG_ON;
        }
    }
    DEBUG_PRINT("%s() end\n", __func__ );
}


static void led_3colorled_off_set(struct light_led_data_type *pst_light_led_data)
{
    int32_t lret = 0;
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_02, INDLED_OFF_VAL);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value uc_color_red_val (0x%02x) \n",
                __func__,
                INDLED_OFF_VAL);
        return;
    }
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_03, INDLED_OFF_VAL);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value uc_color_green_val (0x%02x) \n",
                __func__,
                INDLED_OFF_VAL);
        return;
    }
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_04, INDLED_OFF_VAL);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value uc_color_bule_val (0x%02x) \n",
                __func__,
                INDLED_OFF_VAL);
        return;
    }

}

static void led_3colorled_on_set(struct light_led_data_type *pst_light_led_data )
{
    int32_t lret = 0;

    if( pst_light_led_data->uc_led_red_val != INDLED_OFF_VAL )
    {
        pst_light_led_data->uc_indled_val |= INDLED_RED_ON_VAL;
    }
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_02, pst_light_led_data->uc_led_red_val);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value uc_ledr_red_val (0x%02x) \n",
                __func__,
                pst_light_led_data->uc_led_red_val);
        return;
    }
    
    if( pst_light_led_data->uc_led_green_val != INDLED_OFF_VAL )
    {
        pst_light_led_data->uc_indled_val |= INDLED_GREEN_ON_VAL;
    }
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_03, pst_light_led_data->uc_led_green_val);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value uc_led_green_val (0x%02x) \n",
                __func__,
                pst_light_led_data->uc_led_green_val);
        return;
    }
    
    if( pst_light_led_data->uc_led_blue_val != INDLED_OFF_VAL )
    {
        pst_light_led_data->uc_indled_val |= INDLED_BLUE_ON_VAL;
    }
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_04, pst_light_led_data->uc_led_blue_val);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value uc_led_bule_val (0x%02x) \n",
                __func__,
                pst_light_led_data->uc_led_blue_val);
        return;
    }
}


static void led_pattern_off_set(struct light_led_data_type *pst_light_led_data)
{
    int32_t lret = 0;

    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_01 , INDLED_OFF_VAL);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value led_lighting_val (0x%02x) \n",
                __func__,
                INDLED_OFF_VAL);
        return;
    }

    
    if ( guc_led_pattern_flg )
    {
        lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_18 , REG_ADDR_18_INIT);
        if (lret) {
            DEBUG_PRINT_ERR("%s() REG_ADDR_18 (0x%02x) \n",
                    __func__,
                    REG_ADDR_18_INIT);
            return;
        }

        guc_led_pattern_flg = FLG_OFF;
    }

    led_3colorled_off_set( pst_light_led_data );


}

static void led_pattern_on_set(struct light_led_data_type *pst_light_led_data )
{
    int32_t lret = 0;

    if( pst_light_led_data->uc_led_red_val != INDLED_OFF_VAL )
    {
        pst_light_led_data->uc_indled_val |= INDLED_PATARN_RED_ON_VAL;
    }
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_02, pst_light_led_data->uc_led_red_val);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value uc_ledr_red_val (0x%02x) \n",
                __func__,
                pst_light_led_data->uc_led_red_val);
        return;
    }
    
    if( pst_light_led_data->uc_led_green_val != INDLED_OFF_VAL )
    {
        pst_light_led_data->uc_indled_val |= INDLED_PATARN_GREEN_ON_VAL;
    }
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_03, pst_light_led_data->uc_led_green_val);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value uc_led_green_val (0x%02x) \n",
                __func__,
                pst_light_led_data->uc_led_green_val);
        return;
    }
    
    if( pst_light_led_data->uc_led_blue_val != INDLED_OFF_VAL )
    {
        pst_light_led_data->uc_indled_val |= INDLED_PATARN_BLUE_ON_VAL;
    }
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_04, pst_light_led_data->uc_led_blue_val);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value uc_led_bule_val (0x%02x) \n",
                __func__,
                pst_light_led_data->uc_led_blue_val);
        return;
    }
    
}
static void led_light_on_set(struct light_led_data_type *pst_light_led_data)
{
    int32_t lret = 0;

    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_01 , pst_light_led_data->uc_indled_val & REG_ADDR_01_MASK);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value led_lighting_val (0x%02x) \n",
                __func__,
                pst_light_led_data->uc_indled_val);
        return;
    }

}


static void mobilelight_set(struct light_led_data_type *pst_light_led_data)
{
    int32_t lret = 0 ;
    uint8_t uc_value ;
    int ret = 0 ;

    DEBUG_PRINT("%s() start \n", __func__);

    if( MOBILELIGHT_OFF_MODE == (pst_light_led_data->ul_value&0x00FFFFFF) )
    {
        ret = pmic_low_current_led_set_current( LOW_CURRENT_LED_DRV1 , 0 );
        DEBUG_PRINT("%s() lpmic_low_current_led_set_current ret = %d \n",__func__, ret);
        DEBUG_PRINT("%s() LOW_CURRENT_LED_DRV1 = %d,LED_PMLED_LOW_VAL = %d \n",__func__, LOW_CURRENT_LED_DRV1, 0);
        gpio_set_value( LED_LDO_GPIO, LED_GPIO_LOW_VAL);
        DEBUG_PRINT("%s() GPIO96 set LED_GPIO_HIGH_VAL = %d\n",__func__, LED_GPIO_LOW_VAL);
        
        lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_23 , CARGE_PUMP_OFF_VAL);
        if (lret) {
            DEBUG_PRINT_ERR("%s() CARGE_PUMP_OFF_VAL (0x%02x) \n",
                    __func__,
                    CARGE_PUMP_OFF_VAL );
            return;
        }
        lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_00 , CARGE_PUMP_SETTING_OFF_VAL);
        if (lret) {
            DEBUG_PRINT_ERR("%s() CARGE_PUMP_SETTING_OFF_VAL (0x%02x) \n",
                    __func__,
                    CARGE_PUMP_SETTING_OFF_VAL );
            return;
        }
    }else{
        lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_00 , CARGE_PUMP_SETTING_ON_VAL);
        if (lret) {
            DEBUG_PRINT_ERR("%s() CARGE_PUMP_SETTING_ON_VAL (0x%02x) \n",
                    __func__,
                    CARGE_PUMP_SETTING_ON_VAL );
            return;
        }
        lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_23 , CARGE_PUMP_ON_VAL);
        if (lret) {
            DEBUG_PRINT_ERR("%s() CARGE_PUMP_ON_VAL (0x%02x) \n",
                    __func__,
                    CARGE_PUMP_ON_VAL );
            return;
        }
        
        mdelay(50) ;
        DEBUG_PRINT("%s() wait 50msec \n",__func__);
        gpio_set_value( LED_LDO_GPIO, LED_GPIO_HIGH_VAL);
        DEBUG_PRINT("%s() GPIO96 set LED_GPIO_HIGH_VAL = %d\n",__func__,LED_GPIO_HIGH_VAL);
        mdelay(1) ;
        DEBUG_PRINT("%s() wait 1msec \n",__func__);
        DEBUG_PRINT_ERR("%s() pst_light_led_data->ul_value (0x%08x) \n",
                __func__,
                pst_light_led_data->ul_value&0x00FFFFFF );

        switch( (pst_light_led_data->ul_value&0x00FFFFFF) ) 
        {
            case MOBILELIGHT_BATTTERM_HIGH_MODE:
                uc_value = MOBILELIGHT_BATTTERM_HIGH_VAL ;
                ret = pmic_low_current_led_set_current( LOW_CURRENT_LED_DRV1 , LED_PMLED_HIGH_VAL );
                DEBUG_PRINT("%s() lpmic_low_current_led_set_current ret = %d \n",__func__, ret);
                DEBUG_PRINT("%s() LOW_CURRENT_LED_DRV1 = %d,LED_PMLED_HIGH_VAL = %d \n",__func__, LOW_CURRENT_LED_DRV1, LED_PMLED_HIGH_VAL);
                break ;
            case MOBILELIGHT_BATTTERM_LOW_MODE:
                uc_value = MOBILELIGHT_BATTTERM_LOW_VAL ;
                ret = pmic_low_current_led_set_current( LOW_CURRENT_LED_DRV1 , LED_PMLED_LOW_VAL );
                DEBUG_PRINT("%s() lpmic_low_current_led_set_current ret = %d \n",__func__, ret);
                DEBUG_PRINT("%s() LOW_CURRENT_LED_DRV1 = %d,LED_PMLED_LOW_VAL = %d \n",__func__, LOW_CURRENT_LED_DRV1, LED_PMLED_LOW_VAL);
                break ;
            default :
                uc_value = MOBILELIGHT_BATTTERM_LOW_VAL ;
                ret = pmic_low_current_led_set_current( LOW_CURRENT_LED_DRV1 , LED_PMLED_LOW_VAL );
                DEBUG_PRINT("%s() lpmic_low_current_led_set_current ret = %d \n",__func__, ret);
                DEBUG_PRINT("%s() LOW_CURRENT_LED_DRV1 = %d,LED_PMLED_LOW_VAL = %d \n",__func__, LOW_CURRENT_LED_DRV1, LED_PMLED_LOW_VAL);
                break ;
        }
    }
    DEBUG_PRINT("%s() end\n", __func__);
}

static void light_led_set(struct led_classdev *pst_cdev, enum led_brightness value)
{
    struct light_led_data_type *pst_light_led_data;

    pst_light_led_data = container_of(pst_cdev, struct light_led_data_type, st_cdev); 

    DEBUG_PRINT("%s() start name=%s value=0x%08x\n",
            __func__,
            pst_cdev->name,
            value);

    mutex_lock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
    pst_light_led_data->ul_value = value ;
    pst_light_led_data->saved_color_value = pst_light_led_data->ul_value;
    mutex_unlock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
    schedule_work(&pst_light_led_data->work);

}

static void light_led_work(struct work_struct *work)
{
    struct light_led_data_type *pst_light_led_data = container_of(work,
                     struct light_led_data_type, work);
    uint32_t       delay_time = 0;
    unsigned long  flags;

    DEBUG_PRINT("%s() name=%s\n", __func__, pst_light_led_data->st_cdev.name );
    DEBUG_PRINT("%s() ul_value=0x%08x\n", __func__, pst_light_led_data->ul_value );

    if(!strcmp(pst_light_led_data->st_cdev.name, MOBILELIGHT_INFO))
    {
        mobilelight_set(pst_light_led_data) ;
    }
    else if(!strcmp(pst_light_led_data->st_cdev.name, LED_INFO)) 
    {
        mutex_lock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
        DEBUG_PRINT("LED DEBUG:light_led_work Call Bink=%2x \n",pst_light_led_data->blink_control);
        if(pst_light_led_data->blink_control == BLINK_SOFT_REQUEST_ON)
        {
            pst_light_led_data->ul_value = pst_light_led_data->saved_color_value;
        }else if(pst_light_led_data->blink_control == BLINK_SOFT_REQUEST_OFF)
        {
            pst_light_led_data->ul_value = pst_light_led_data->blink_off_color;
        }
        led_set(pst_light_led_data);
        
        if(pst_light_led_data->blink_control == NO_BLINK_REQUEST ||
           pst_light_led_data->blink_control == BLINK_HARD_REQUEST)
        {
            alarm_cancel(&pst_light_led_data->led_alarm);
            wake_unlock(&pst_light_led_data->work_wake_lock);
        }
        else
        {
            if(pst_light_led_data->blink_control == BLINK_SOFT_REQUEST_ON)
            {
                pst_light_led_data->blink_control = BLINK_SOFT_REQUEST_OFF;
                delay_time = gul_delay_time[pst_light_led_data->blink_on_delay][0];
                DEBUG_PRINT("%s:delay_time = %d\n",__func__,delay_time);
            }
            else if(pst_light_led_data->blink_control == BLINK_SOFT_REQUEST_OFF)
            {
                pst_light_led_data->blink_control = BLINK_SOFT_REQUEST_ON;
                delay_time = gul_delay_time[pst_light_led_data->blink_off_delay][1];
                DEBUG_PRINT("%s:delay_time = %d\n",__func__,delay_time);
            }
            
            pst_light_led_data->last_elapsed_time = alarm_get_elapsed_realtime();
            local_irq_save(flags);
            led_set_alarm(pst_light_led_data,delay_time);
            wake_unlock(&pst_light_led_data->work_wake_lock);
            local_irq_restore(flags);
        }
        mutex_unlock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
    }

}

static enum led_brightness light_led_get(struct led_classdev *pst_cdev)
{
    int32_t lret = 0 ;
    struct light_led_data_type *pst_light_led_data;
    DEBUG_PRINT("%s() start\n", __func__ );

    pst_light_led_data = container_of(pst_cdev, struct light_led_data_type, st_cdev); 
    lret = pst_light_led_data->ul_value ;

    DEBUG_PRINT("%s:%d\n",__func__,lret);

    DEBUG_PRINT("%s() end\n", __func__);

    return lret ;
}

static int32_t light_led_init_reg( struct light_led_data_type *pst_light_led_data )
{
    int32_t lret = 0;
    int32_t ret = 0;

    DEBUG_PRINT("%s() start\n", __func__ );

    ret = pmic_low_current_led_set_ext_signal( LOW_CURRENT_LED_DRV1 , EXT_SIGNAL_CURRENT_SINK_MANUAL_MODE );
    ret = pmic_low_current_led_set_current( LOW_CURRENT_LED_DRV1 , LED_PMLED_INIT_VAL );

    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_06, REG_ADDR_06_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg06_val (0x%02x) \n",
                __func__,
                REG_ADDR_06_INIT );
        return lret;
    }

    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_07, REG_ADDR_07_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg07_val (0x%02x) \n",
                __func__,
                REG_ADDR_07_INIT );
        return lret;
    }

    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_15, REG_ADDR_15_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg15_val (0x%02x) \n",
                __func__,
                REG_ADDR_15_INIT );
        return lret;
    }

    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_16, REG_ADDR_16_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg16_val (0x%02x) \n",
                __func__,
                REG_ADDR_16_INIT );
        return lret;
    }
    
    
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_1D, REG_ADDR_1D_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg1D_val (0x%02x) \n",
                __func__,
                REG_ADDR_1D_INIT );
        return lret;
    }

    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_26, REG_ADDR_26_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg26_val (0x%02x) \n",
                __func__,
                REG_ADDR_26_INIT );
        return lret;
    }
    
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_29, REG_ADDR_29_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg29_val (0x%02x) \n",
                __func__,
                REG_ADDR_29_INIT );
        return lret;
    }
    
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_40, REG_ADDR_40_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg40_val (0x%02x) \n",
                __func__,
                REG_ADDR_40_INIT );
        return lret;
    }
    
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_41, REG_ADDR_41_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg41_val (0x%02x) \n",
                __func__,
                REG_ADDR_41_INIT );
        return lret;
    }
    
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_42, REG_ADDR_42_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg42_val (0x%02x) \n",
                __func__,
                REG_ADDR_42_INIT );
        return lret;
    }
    
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_43, REG_ADDR_43_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg43_val (0x%02x) \n",
                __func__,
                REG_ADDR_43_INIT );
        return lret;
    }
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_17, REG_ADDR_17_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg17_val (0x%02x) \n",
                __func__,
                REG_ADDR_17_INIT );
        return lret;
    }
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_19, REG_ADDR_19_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg19_val (0x%02x) \n",
                __func__,
                REG_ADDR_19_INIT );
        return lret;
    }
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_1A, REG_ADDR_1A_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg1A_val (0x%02x) \n",
                __func__,
                REG_ADDR_1A_INIT );
        return lret;
    }
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_1C, REG_ADDR_1C_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg1C_val (0x%02x) \n",
                __func__,
                REG_ADDR_1C_INIT );
        return lret;
    }

    DEBUG_PRINT("%s() end\n", __func__);

    return lret ;
}

static long leds_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = -1;
    T_LEDLIGHT_IOCTL st_ioctl;

    switch (cmd) {
    case LEDLIGHT_SET_BLINK:
        ret = copy_from_user(&st_ioctl, argp, sizeof(T_LEDLIGHT_IOCTL));
        if (ret) {
            printk("error : leds_ioctl(cmd = LEDLIGHT_SET_BLINK)\n" );
            return -EFAULT;
        }
        DEBUG_PRINT("%s:%d,%d,%d,%d\n",__func__,st_ioctl.data[0],st_ioctl.data[1],st_ioctl.data[2],st_ioctl.data[3]);
        mutex_lock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
        gpst_light_led_data[LEDLIGHT_INDEX].blink_control      = st_ioctl.data[0];
        gpst_light_led_data[LEDLIGHT_INDEX].blink_on_delay    = st_ioctl.data[1];
        gpst_light_led_data[LEDLIGHT_INDEX].blink_off_delay  = st_ioctl.data[2];
        gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color  = st_ioctl.data[3];
        mutex_unlock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
    break;
  default:
    return -ENOTTY;
  }
  return 0;
}

static int32_t leds_open( struct inode* inode, struct file* filp )
{
    return 0;
}

static int32_t leds_release( struct inode* inode, struct file* filp )
{
    return 0;
}

static struct file_operations leds_fops = {
    .owner      = THIS_MODULE,
    .open       = leds_open,
    .release    = leds_release,
    .unlocked_ioctl   = leds_ioctl,
};
static struct miscdevice leds_device = {
    .minor      = MISC_DYNAMIC_MINOR,
    .name       = "leds-ledlight",
    .fops       = &leds_fops,
};

static void led_class_setinfo(struct light_led_data_type *pst_light_led_data, uint32_t ul_ledindex)
{
    
    switch( ul_ledindex )
    {
        case MOBILELIGHT_INDEX :
            pst_light_led_data[ul_ledindex].st_cdev.name               = MOBILELIGHT_INFO;
            pst_light_led_data[ul_ledindex].st_cdev.max_brightness     = MOBILELIGHT_MAX_BRIGHT_VAL;
        break ;

        case LEDLIGHT_INDEX :
            pst_light_led_data[ul_ledindex].st_cdev.name               = LED_INFO;
            pst_light_led_data[ul_ledindex].st_cdev.max_brightness     = (unsigned int)LEDLIGHT_MAX_BRIGHT_VAL;
        break ;
        
    }

    pst_light_led_data[ul_ledindex].st_cdev.brightness_set     = light_led_set;
    pst_light_led_data[ul_ledindex].st_cdev.brightness_get     = light_led_get;
    pst_light_led_data[ul_ledindex].st_cdev.brightness         = LED_OFF;
    pst_light_led_data[ul_ledindex].st_cdev.flags              = 0;
}

static int32_t __devinit light_led_probe(struct i2c_client *pst_client,
            const struct i2c_device_id *id)
{
    int32_t lret=0;
    struct light_led_data_type *pst_light_led_data;
    int32_t i ;

    DEBUG_PRINT("%s() start pst_client=0x%08x idname=%s\n", __func__, (unsigned int)pst_client, id->name);

    pst_light_led_data = kzalloc(sizeof(struct light_led_data_type)*LIGHT_INDEX_MAX, GFP_KERNEL);
    if (!pst_light_led_data) {
        DEBUG_PRINT_ERR("%s() failed to allocate driver data\n", __func__);
        return -1;
    }
    for( i=0; i<LIGHT_INDEX_MAX; i++ )
    {
        INIT_WORK(&pst_light_led_data[i].work, light_led_work);
    }
    i2c_set_clientdata(pst_client, pst_light_led_data);

    gpst_light_led_data = pst_light_led_data;
    pst_light_led_data[MOBILELIGHT_INDEX].pst_client  = pst_client;
    pst_light_led_data[LEDLIGHT_INDEX].pst_client     = pst_client;

    for( i=0; i<LIGHT_INDEX_MAX; i++ )
    {
        mutex_init(&pst_light_led_data[i].lock);
        spin_lock_init(&pst_light_led_data[i].value_lock);

        DEBUG_PRINT("%s() light_led_init_reg() call \n", __func__ );
        if ( i == 0){
            lret = light_led_init_reg( &(pst_light_led_data[i]) ) ;
            if(lret){
                DEBUG_PRINT_ERR("%s() light_led_init_reg() lret=%d\n", __func__, lret );
                goto fail_id_check;
            }
        }

        led_class_setinfo(pst_light_led_data, i) ;

        lret = led_classdev_register(&pst_light_led_data[i].pst_client->dev, &pst_light_led_data[i].st_cdev);
        if (lret) {
            DEBUG_PRINT_ERR("%s() unable to register led %s\n",
                    __func__,
                    pst_light_led_data[i].st_cdev.name);
            goto fail_id_check;
        }
        pst_light_led_data[i].blink_control     = NO_BLINK_REQUEST; 
        pst_light_led_data[i].blink_on_delay    = 0;
        pst_light_led_data[i].blink_off_delay   = 0;
        pst_light_led_data[i].blink_off_color   = 0;
        pst_light_led_data[i].saved_color_value = pst_light_led_data[LEDLIGHT_INDEX].ul_value;
    }

    wake_lock_init(&pst_light_led_data[LEDLIGHT_INDEX].work_wake_lock, WAKE_LOCK_SUSPEND,"led-lights");
    alarm_init(&pst_light_led_data[LEDLIGHT_INDEX].led_alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,led_alarm);
    
    DEBUG_PRINT("%s() end\n", __func__ );
    return lret;

fail_id_check:
    while( i-- )
    {
        led_classdev_unregister(&pst_light_led_data[i].st_cdev);
    }

    kfree(pst_light_led_data);
    return lret;

}

static int32_t __exit light_led_remove(struct i2c_client *pst_client)
{
    int32_t lret = 0 ;
    struct light_led_data_type *pst_light_led_data = i2c_get_clientdata(pst_client);
    int32_t i ;

    DEBUG_PRINT("%s() start\n", __func__ );

    for( i=0; i<LIGHT_INDEX_MAX; i++ )
    {
        led_classdev_unregister(&pst_light_led_data[i].st_cdev);
    }

    kfree(pst_light_led_data);

    DEBUG_PRINT("%s() end\n", __func__ );
    return lret;
}

static int32_t light_led_suspend(struct i2c_client *pst_client, pm_message_t mesg)
{
    int32_t lret= 0 ;
    DEBUG_PRINT("%s() start \n", __func__ ) ;
    return lret;
}

static int32_t light_led_resume(struct i2c_client *pst_client)
{
    int32_t lret= 0 ;
    DEBUG_PRINT("%s() start \n", __func__ ) ;
    return lret;
}

static void light_led_shutdown(struct i2c_client *pst_client)
{
    int32_t lret=0;
    int32_t ret = 0; 
    struct light_led_data_type *pst_light_led_data = i2c_get_clientdata(pst_client);

    ret = pmic_low_current_led_set_ext_signal( LOW_CURRENT_LED_DRV1 , EXT_SIGNAL_CURRENT_SINK_MANUAL_MODE );
    ret = pmic_low_current_led_set_current( LOW_CURRENT_LED_DRV1 , LED_PMLED_INIT_VAL );
    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_00, REG_ADDR_00_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg00_val (0x%02x) \n",
                __func__,
                REG_ADDR_00_INIT );
        return ;
    }

    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_01, REG_ADDR_01_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg01_val (0x%02x) \n",
                __func__,
                REG_ADDR_01_INIT );
        return ;
    }

    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_1B, REG_ADDR_1B_ON);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg1B_val (0x%02x) \n",
                __func__,
                REG_ADDR_1B_ON );
        return ;
    }

    lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_23, REG_ADDR_23_INIT);
    if (lret) {
        DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value init_reg23_val (0x%02x) \n",
                __func__,
                REG_ADDR_23_INIT );
        return ;
    }

    return ;
}

static const struct i2c_device_id ledlight_id[] = {
    { LED_DRV_NAME, 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, ledlight_id);

static struct i2c_driver light_led_driver = {
    .driver = {
        .name   = LED_DRV_NAME,
    },
    .probe        = light_led_probe,
    .remove        = __exit_p(light_led_remove),
    .suspend    = light_led_suspend,
    .resume        = light_led_resume,
    .shutdown    = light_led_shutdown,
    .id_table    = ledlight_id,

};

static int32_t __init light_led_init(void)
{
    int32_t lret=0;
    DEBUG_PRINT("%s() start\n", __func__ );

    DEBUG_PRINT("%s() i2c_add_driver() call\n", __func__ );
    lret=i2c_add_driver(&light_led_driver);
    DEBUG_PRINT("%s() i2c_add_driver() call end lret=%d\n", __func__ , lret );

    misc_register(&leds_device);

    DEBUG_PRINT("%s() end\n", __func__ );
    return lret ;

}
module_init(light_led_init);

static void __exit light_led_exit(void)
{
    DEBUG_PRINT("%s() start\n", __func__ );
    misc_deregister(&leds_device);
    return i2c_del_driver(&light_led_driver);
}
module_exit(light_led_exit);

MODULE_AUTHOR("KYOCERA");
MODULE_DESCRIPTION("ledlight driver");
MODULE_LICENSE("GPL");


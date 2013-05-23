/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 */
/*
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
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/key_dm_driver.h>

#include <linux/workqueue.h>
#include <mach/rpc_server_handset.h>
#include <mach/changer_dm_driver.h>
#include <mach/kc_changer_ic.h>
#include <mach/rpc_hsusb.h>
#include <linux/wakelock.h>
#define CHGIC_DEBUG             0
#if CHGIC_DEBUG
#define CHGIC_DBG_PRINT(fmt, ...) printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#else 
#define CHGIC_DBG_PRINT(fmt, ...) 
#endif 

#define CHGIC_REG_00H                           0x00
#define CHGIC_REG_01H                           0x01
#define CHGIC_REG_02H                           0x02
#define CHGIC_REG_03H                           0x03
#define CHGIC_REG_04H                           0x04
#define CHGIC_REG_05H                           0x05
#define CHGIC_REG_06H                           0x06
#define CHGIC_REG_07H                           0x07
#define CHGIC_REG_10H                           0x10
#define CHGIC_REG_11H                           0x11

#define CHGIC_REG_00H_M                         0x07
#define CHGIC_REG_01H_M                         0xFE
#define CHGIC_REG_02H_M                         0xFF
#define CHGIC_REG_03H_M                         0x1F
#define CHGIC_REG_04H_M                         0x0F
#define CHGIC_REG_05H_M                         0x0F
#define CHGIC_REG_06H_M                         0x0F
#define CHGIC_REG_07H_M                         0xFF
#define CHGIC_REG_10H_M                         0x07
#define CHGIC_REG_11H_M                         0x07

#define CHGIC_DATA_CHG_DET_OFF                  0x01
#define CHGIC_DATA_CHG_DET_ON                   0x03

#define CHGIC_DATA_05H_ON                       CHGIC_REG_04H_M
#define CHGIC_DATA_05H_OFF                      0x00
#define CHGIC_DATA_05H_SW                       0x0C
#define CHGIC_DATA_06H_NO                       CHGIC_REG_04H_M
#define CHGIC_DATA_06H_STANDBY                  0x0C
#define CHGIC_DATA_06H_ALL                      0x00
#define CHGIC_DATA_ALL_NORMAL                   0x00
#define CHGIC_DATA_ADC_ON                       0x04

#define CHGIC_DATA_01H_DSHORT_DET               0x02   
#define CHGIC_DATA_01H_VBUS_DET                 0x04    
#define CHGIC_DATA_01H_DETECT                   (CHGIC_DATA_01H_DSHORT_DET | CHGIC_DATA_01H_VBUS_DET)
#define CHGIC_DATA_04H_DET_ID                   0x01
#define CHGIC_DATA_04H_DET_CHG                  0x02
#define CHGIC_DATA_04H_DET                      (CHGIC_DATA_04H_DET_ID | CHGIC_DATA_04H_DET_CHG)
#define CHGIC_DATA_04H_SW_OFF                   0x04
#define CHGIC_DATA_04H_SW_ON                    0x08
#define CHGIC_DATA_04H_SW                       (CHGIC_DATA_04H_SW_ON | CHGIC_DATA_04H_SW_OFF)
#define CHGIC_DATA_RID_OPEN                     0x1F
#define CHGIC_DATA_AUDIO_MIC_MONO               0x1E
#define CHGIC_DATA_AUDIO_MIC_MONO_CHG           0x1C
#define CHGIC_DATA_AUDIO_MIC_STEREO             0x18
#define CHGIC_DATA_AUDIO_STEREO                 0x14
#define CHGIC_DATA_AUDIO_SW_ON_INS              0x12
#define CHGIC_DATA_AUDIO_CHG_SW_ON_INS          0x11
#define CHGIC_DATA_CHG_DET_MODE1                0x01
#define CHGIC_DATA_CHG_DET_MODE2                0x02
#define CHGIC_DATA_CHG_DET_MODE3                0x03
#define CHGIC_DATA_CHG_DET_MODE4                0x07

#define CHGIC_SET_DM_INFO(p1, p2, p3, p4)       {dm_info_sw_set_mode = p1; dm_info_detection_id_state = p2; dm_info_interrupt_case = p3; dm_info_chg_det_mode = p4;}
#define MAX_CALLBACK_CLIENTS                    4

#define CHGIC_REG_WRITE_DATA(client, reg, val)  chgic_write_data(client, reg, reg##_M, val)
#define CHGIC_REG_READ_DATA(client, reg)        chgic_read_data(client, reg, reg##_M)

#define CHGIC_MODE_VBUS_MASK                    0x04
#define MAX_CHG_VAL                             1800
#define WAIT_START_RETRY                        100

#define CHGIC_I2C_WRITE_SIZE                    1
#define CHGIC_I2C_READ_SIZE                     2
#define CHGIC_I2C_RETRIES_NUM                   5
#define CHGIC_I2C_RETRIES_WAIT                  1

#define CHGIC_WAKE_LOCK_TIME                    (5 * HZ)

#define PM8058_GPIO_PM_TO_SYS(pm_gpio)          (pm_gpio + NR_GPIO_IRQS)
#define CHGIC_USBPHY_LC824204_GPIO              PM8058_GPIO_PM_TO_SYS(4)
#define CHGIC_GPIO_ON                           1
#define CHGIC_GPIO_OFF                          0

#define CHGIC_OVP_DET_MASK                      0x04

struct lc824204_changer {
    struct i2c_client       *client;
    struct work_struct      work;
    struct work_struct      chg_work;
};

struct kc_changer_cb_info {
    struct kc_changer_ic_event_callback *cb_tbl[MAX_CALLBACK_CLIENTS];
};

enum charger_type {
    USB_CHG_TYPE__STEREO_EARPHONE = (USB_CHG_TYPE__INVALID + 1),
    USB_CHG_TYPE__MONO_EARPHONE,
    USB_CHG_TYPE__NONE,
};
static struct lc824204_changer *changer_info;
static u8 changer_set_mode = KC_CHANGER_UNINITIALIZE;

static u8 changer_set_mode_prev = KC_CHANGER_UNINITIALIZE;

static bool earphone_flg = false;

static u8 dm_info_sw_set_mode = 0;
static u8 dm_info_detection_id_state = 0;
static u8 dm_info_interrupt_case = 0;
static u8 dm_info_chg_det_mode = 0;

static u8 bk_interrupt_mask = 0;
static u8 dm_set_chg_det_wait = 10; 

static int32_t changer_ic_irq = 0;

static struct kc_changer_cb_info *changer_cb_info = NULL;
static bool next_detect_force_flag = false;

static struct wake_lock chgic_wake_lock;

static kc_changer_ic_ovp_det_cb chgic_ovp_det_cb = NULL;
static kc_changer_ic_ovp_detection_enum chgic_ovp_state = KC_CHANGER_OVP_NOT_DETECTION;

static bool delay_fix_flag = false;
static int32_t chgic_write_data(struct i2c_client *client, u8 reg, u8 reg_mask, u8 val);
static int32_t chgic_read_data(struct i2c_client *client, u8 reg, u8 reg_mask);
static u8 chgic_read_register_status(u8 *sw_state, u8 *interrupt_case, u8 *id_state, u8 *chg_det_mode, bool is_chg_det);
static void chgic_fix_accessory(u8 val);
static irqreturn_t chgic_interrupt(int32_t irq, void *dev_id);
static void chgic_workqueue(struct work_struct *work);
static void chgic_interrupt_exec(bool force_flag);
static void chgic_interrupt_plug_out(void);
static void chgic_interrupt_plug_in(void);
static void chgic_interrupt_mic_sw(u8 interrupt_case);
static int32_t chgic_suspend(struct i2c_client *client, pm_message_t mesg);
static int32_t chgic_resume(struct i2c_client *client);
static void chgic_chg_workqueue(struct work_struct *work);
static void chgic_update_ovp_state(u8 ovp_state);

static bool kc_changer_delay_fix_accessory(u8 sw_state);
static int32_t chgic_suspend(struct i2c_client *client, pm_message_t mesg)
{
    CHGIC_DBG_PRINT("%s: \n", __func__);
    if (device_may_wakeup(&client->dev))
    {
        enable_irq_wake(changer_ic_irq);
        disable_irq(changer_ic_irq);
    }
    return 0;
}

static int32_t chgic_resume(struct i2c_client *client)
{
    CHGIC_DBG_PRINT("%s: \n", __func__);
    if (device_may_wakeup(&client->dev))
    {
        disable_irq_wake(changer_ic_irq);
        enable_irq(changer_ic_irq);
    }
    chgic_interrupt_exec(true);
    return 0;
}
static int32_t chgic_write_data(struct i2c_client *client, u8 reg, u8 reg_mask, u8 val)
{
    int32_t err = 0;
    int32_t retry_count = 0;
    struct i2c_msg msg[1];
    unsigned char data[2];

    if (!client->adapter)
    {
        printk(KERN_WARNING "LC824204 adapter info failed.\n");
        return -ENODEV;
    }

    data[0] = reg;
    data[1] = (val & reg_mask);
    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 2;
    msg[0].buf = data;

    do {
        err = i2c_transfer(client->adapter, msg, CHGIC_I2C_WRITE_SIZE);
        if (err == CHGIC_I2C_WRITE_SIZE)
        {
            return 0;
        }
        msleep(CHGIC_I2C_RETRIES_WAIT);
    } while (++retry_count < CHGIC_I2C_RETRIES_NUM);

    printk(KERN_WARNING "LC824204 %02xH write failed.\n", reg);
    return err;
}

static int32_t chgic_read_data(struct i2c_client *client, u8 reg, u8 reg_mask)
{
    int32_t err = 0;
    int32_t retry_count = 0;
    struct i2c_msg msg[2];
    u8 reg_buf, data_buf = 0;

    if (!client->adapter)
    {
        printk(KERN_WARNING "LC824204 adapter info failed.\n");
        return 0;
    }

    reg_buf = reg;
    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &reg_buf;
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = 1;
    msg[1].buf = &data_buf;

    do {
        err = i2c_transfer(client->adapter, msg, CHGIC_I2C_READ_SIZE);
        if (err == CHGIC_I2C_READ_SIZE)
        {
            return (data_buf & reg_mask);
        }
        msleep(CHGIC_I2C_RETRIES_WAIT);
    } while (++retry_count < CHGIC_I2C_RETRIES_NUM);

    printk(KERN_WARNING "LC824204 %02xH read failed.\n", reg);
    return 0;
}

u8 kc_changer_ic_set_chgdet_wait(int32_t *val)
{
    CHGIC_DBG_PRINT("%s: val[0]:%x\n", __func__, val[0]);
    dm_set_chg_det_wait = val[0];
    return 0;
}

u8 kc_changer_ic_get_dminfo(unsigned char cmd, int32_t *val)
{
    switch (cmd)
    {
        case CHANGER_DM_DRIVER_GET_SW_SET_MODE:
        {
            *val = dm_info_sw_set_mode;
        }
        break;
        case CHANGER_DM_DRIVER_GET_DETECTION_ID_STATE:
        {
            *val = dm_info_detection_id_state;
        }
        break;
        case CHANGER_DM_DRIVER_GET_INTERRUPT_CASE:
        {
            *val = dm_info_interrupt_case;
        }
        break;
        case CHANGER_DM_DRIVER_GET_CHG_DET_MODE:
        {
            *val = dm_info_chg_det_mode;
        }
        break;
        default:
        {
        }
        break;
    }
    return 0;
}

static u8 chgic_read_register_status(u8 *sw_state, u8 *interrupt_case, u8 *id_state, u8 *chg_det_mode, bool is_chg_det)
{
    u8 analyze_ret = KC_CHANGER_NO_ACCESSORY;
    u8 ovp_read_val = 0;

    if (is_chg_det)
    {
        msleep(dm_set_chg_det_wait * 10);   /* (10ms * DM setting) sleep */
        CHGIC_REG_WRITE_DATA(changer_info->client, CHGIC_REG_00H, CHGIC_DATA_CHG_DET_OFF);
        CHGIC_REG_WRITE_DATA(changer_info->client, CHGIC_REG_00H, CHGIC_DATA_CHG_DET_ON);
    }

    ovp_read_val = CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_11H);
    chgic_update_ovp_state(ovp_read_val);

    *sw_state = CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_01H);
    *interrupt_case = CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_04H);

    CHGIC_REG_WRITE_DATA(changer_info->client, CHGIC_REG_05H, CHGIC_DATA_05H_ON);
    if ((*interrupt_case & CHGIC_DATA_04H_SW) != 0)
    {
        CHGIC_REG_WRITE_DATA(changer_info->client, CHGIC_REG_05H, CHGIC_DATA_05H_OFF);
    }
    else
    {
        CHGIC_REG_WRITE_DATA(changer_info->client, CHGIC_REG_05H, CHGIC_DATA_05H_OFF);
    }

    if (0 <= CHGIC_REG_WRITE_DATA(changer_info->client, CHGIC_REG_07H, CHGIC_DATA_ADC_ON))
    {
        *id_state = CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_03H);
    }
    analyze_ret = (*id_state << 3) | (*sw_state & CHGIC_DATA_01H_DETECT);

    if (is_chg_det)
    {
        *chg_det_mode = CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_10H);
        if ((analyze_ret == KC_CHANGER_USB_MODE) &&
            ((*chg_det_mode == CHGIC_DATA_CHG_DET_MODE1) ||
             (*chg_det_mode == CHGIC_DATA_CHG_DET_MODE2) ||
             (*chg_det_mode == CHGIC_DATA_CHG_DET_MODE3) ||
             (*chg_det_mode == CHGIC_DATA_CHG_DET_MODE4)))
        {
            analyze_ret = KC_CHANGER_AC_ADAPTER;
        }
    }
    return analyze_ret;
}

static void chgic_fix_accessory(u8 val)
{
    int32_t i;

    if (kc_changer_ic_is_fix_accessory(val) != true)
    {
        return;
    }
    CHGIC_DBG_PRINT("### %s: val:%02x\n", __func__, val);
    if (0 <= CHGIC_REG_WRITE_DATA(changer_info->client, CHGIC_REG_02H, val))
    {
        if(changer_set_mode == val)
        {
            CHGIC_DBG_PRINT("### %s: accessory no change\n", __func__);
            return;
        }

        printk("### fix_accessory = 0x%x\n",val);
        key_dm_driver_set_port(val);
        changer_set_mode_prev = changer_set_mode;
        changer_set_mode = val;
        schedule_work(&changer_info->chg_work);
        if (changer_cb_info != NULL)
        {
            for (i = 0; i < MAX_CALLBACK_CLIENTS; ++i)
            {
                if (changer_cb_info->cb_tbl[i] && changer_cb_info->cb_tbl[i]->fn)
                {
                    changer_cb_info->cb_tbl[i]->fn(val);
                }
            }
        }
    }
}

static irqreturn_t chgic_interrupt(int32_t irq, void *dev_id)
{
    if (changer_ic_irq != irq)
    {
        return IRQ_NONE;
    }
    schedule_work(&changer_info->work);

    return IRQ_HANDLED;
}

static void chgic_workqueue(struct work_struct *work)
{
    CHGIC_DBG_PRINT("%s: \n", __func__);
    chgic_interrupt_exec(false);
}

static void chgic_interrupt_exec(bool force_flag)
{
    u8 sw_state = 0;
    u8 interrupt_case = 0;
    u8 id_state = 0;
    u8 chg_det_mode = 0;
    u8 set_mode = 0;

    if (next_detect_force_flag)
    {
        next_detect_force_flag = false;
        force_flag = true;
        delay_fix_flag = true;
    }
    CHGIC_DBG_PRINT(" %02xH: 0x%02x\n", CHGIC_REG_00H, CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_00H));
    CHGIC_DBG_PRINT(" %02xH: 0x%02x\n", CHGIC_REG_01H, CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_01H));
    CHGIC_DBG_PRINT(" %02xH: 0x%02x\n", CHGIC_REG_02H, CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_02H));
    CHGIC_DBG_PRINT(" %02xH: 0x%02x\n", CHGIC_REG_03H, CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_03H));
    CHGIC_DBG_PRINT(" %02xH: 0x%02x\n", CHGIC_REG_04H, CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_04H));
    CHGIC_DBG_PRINT(" %02xH: 0x%02x\n", CHGIC_REG_05H, CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_05H));
    CHGIC_DBG_PRINT(" %02xH: 0x%02x\n", CHGIC_REG_06H, CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_06H));
    CHGIC_DBG_PRINT(" %02xH: 0x%02x\n", CHGIC_REG_07H, CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_07H));
    CHGIC_DBG_PRINT(" %02xH: 0x%02x\n", CHGIC_REG_10H, CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_10H));
    CHGIC_DBG_PRINT(" %02xH: 0x%02x\n", CHGIC_REG_11H, CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_11H));

    bk_interrupt_mask = CHGIC_REG_READ_DATA(changer_info->client, CHGIC_REG_06H);
    CHGIC_REG_WRITE_DATA(changer_info->client, CHGIC_REG_06H, CHGIC_DATA_06H_NO);
    set_mode = chgic_read_register_status(&sw_state, &interrupt_case, &id_state, &chg_det_mode, false);
    CHGIC_DBG_PRINT("### set_mode:%x, sw_state:%x, interrupt_case:%x, id_state:%x, chg_det_mode:%x\n", set_mode, sw_state, interrupt_case, id_state, chg_det_mode);

    if (set_mode == KC_CHANGER_NO_ACCESSORY)
    {
        if(KC_CHANGER_NO_ACCESSORY != changer_set_mode)
        {
            wake_lock_timeout( &chgic_wake_lock, CHGIC_WAKE_LOCK_TIME );
        }
        chgic_interrupt_plug_out();
    }
    else
    {
        if (force_flag || (interrupt_case & CHGIC_DATA_04H_DET) != 0)
        {
            chgic_interrupt_plug_in();
        }
        if ((interrupt_case & CHGIC_DATA_04H_SW) != 0)
        {
            u8 tmp_interrupt_case = 0;

            msleep(20);
            set_mode = chgic_read_register_status(&sw_state, &tmp_interrupt_case, &id_state, &chg_det_mode, false);
            CHGIC_DBG_PRINT("### SW interrupt set_mode:%x, sw_state:%x, interrupt_case:%x, id_state:%x, chg_det_mode:%x\n", set_mode, sw_state, tmp_interrupt_case, id_state, chg_det_mode);

            if ((id_state == CHGIC_DATA_AUDIO_SW_ON_INS) || (id_state == CHGIC_DATA_AUDIO_CHG_SW_ON_INS))
            {
              chgic_interrupt_mic_sw(interrupt_case);
            }
            else
            {
              if( interrupt_case & CHGIC_DATA_04H_SW_OFF )
              {
                chgic_interrupt_mic_sw(interrupt_case);
              }
            }
        }
    }
    delay_fix_flag = false;
    CHGIC_REG_WRITE_DATA(changer_info->client, CHGIC_REG_06H, bk_interrupt_mask);
}

static void chgic_interrupt_plug_out(void)
{
    int32_t usbphy_val = gpio_get_value_cansleep(CHGIC_USBPHY_LC824204_GPIO);
    CHGIC_DBG_PRINT("### %s:before:usbphy=%d\n",__func__,usbphy_val);
    if(usbphy_val != CHGIC_GPIO_OFF)
    {
        gpio_set_value_cansleep(CHGIC_USBPHY_LC824204_GPIO, CHGIC_GPIO_OFF);
    }
    chgic_fix_accessory(KC_CHANGER_NO_ACCESSORY);

    CHGIC_REG_WRITE_DATA(changer_info->client, CHGIC_REG_07H, CHGIC_DATA_ALL_NORMAL);
    CHGIC_REG_WRITE_DATA(changer_info->client, CHGIC_REG_05H, CHGIC_DATA_05H_SW);

    bk_interrupt_mask = CHGIC_DATA_06H_STANDBY;
    if (earphone_flg == true)
    {
        key_dm_driver_set_port(0x12);
        kc_hs_jack_change_state(false);
        earphone_flg = false;
    }

}

static void chgic_interrupt_plug_in(void)
{
    u8 sw_state = 0;
    u8 interrupt_case = 0;
    u8 id_state = 0;
    u8 chg_det_mode = 0;
    u8 set_mode = 0;
    int32_t usbphy_val = 0;

    set_mode = chgic_read_register_status(&sw_state, &interrupt_case, &id_state, &chg_det_mode, true);
    CHGIC_DBG_PRINT("### set_mode:%x, sw_state:%x, interrupt_case:%x, id_state:%x, chg_det_mode:%x\n", set_mode, sw_state, interrupt_case, id_state, chg_det_mode);

    if(!(changer_set_mode == KC_CHANGER_UNINITIALIZE) &&
        (changer_set_mode != KC_CHANGER_NO_ACCESSORY) &&
       !((set_mode == KC_CHANGER_NO_ACCESSORY) ||
        (kc_changer_delay_fix_accessory(sw_state) == true)))
    {
       CHGIC_DBG_PRINT("chgic_interrupt_plug_in:Return changer_set_mode:%x delay_fix_flag:%x\n",
                         changer_set_mode,delay_fix_flag);
       return;
    }

    usbphy_val = gpio_get_value_cansleep(CHGIC_USBPHY_LC824204_GPIO);
    CHGIC_DBG_PRINT("### %s:before:usbphy=%d\n",__func__,usbphy_val);
    if(sw_state & CHGIC_DATA_01H_VBUS_DET &&
        usbphy_val == CHGIC_GPIO_OFF)
    {
        gpio_set_value_cansleep(CHGIC_USBPHY_LC824204_GPIO, CHGIC_GPIO_ON);
    }

    switch (id_state)
    {
        case CHGIC_DATA_RID_OPEN:
        {
            if (set_mode == KC_CHANGER_AC_ADAPTER)
            {
                CHGIC_SET_DM_INFO(sw_state, id_state, interrupt_case, chg_det_mode);
            }
            if (set_mode == KC_CHANGER_NO_ACCESSORY)
            {
               chgic_interrupt_plug_out();
            }
            else
            {
                chgic_fix_accessory(set_mode);
            }
        }
        break;
        case CHGIC_DATA_AUDIO_MIC_MONO:
        case CHGIC_DATA_AUDIO_MIC_MONO_CHG:
        case CHGIC_DATA_AUDIO_MIC_STEREO:
        case CHGIC_DATA_AUDIO_STEREO:
        {
            CHGIC_SET_DM_INFO(sw_state, id_state, interrupt_case, chg_det_mode);
            chgic_fix_accessory(set_mode);

            if ((id_state == CHGIC_DATA_AUDIO_MIC_MONO) || 
                 (id_state == CHGIC_DATA_AUDIO_MIC_MONO_CHG) || 
                 (id_state == CHGIC_DATA_AUDIO_MIC_STEREO))
            {
                CHGIC_REG_WRITE_DATA(changer_info->client, CHGIC_REG_05H, CHGIC_DATA_05H_OFF);
                bk_interrupt_mask = CHGIC_DATA_06H_ALL;
            }
            else
            {
                CHGIC_REG_WRITE_DATA(changer_info->client, CHGIC_REG_07H, CHGIC_DATA_ALL_NORMAL);
            }
            kc_hs_jack_change_state(true);
            earphone_flg = true;
        }
        break;
        case CHGIC_DATA_AUDIO_SW_ON_INS:
        case CHGIC_DATA_AUDIO_CHG_SW_ON_INS:
        {
            CHGIC_SET_DM_INFO(sw_state, id_state, interrupt_case, chg_det_mode);
            if ((sw_state & CHGIC_DATA_01H_VBUS_DET) == 0)
            {
                chgic_fix_accessory(KC_CHANGER_AUDIO_MIC_STEREO);
            }
            else
            {
                chgic_fix_accessory(KC_CHANGER_AUDIO_CHG_STEREO);
            }
            next_detect_force_flag = true;

            CHGIC_REG_WRITE_DATA(changer_info->client, CHGIC_REG_05H, CHGIC_DATA_05H_OFF);
            bk_interrupt_mask = CHGIC_DATA_06H_ALL;

            kc_hs_jack_change_state(true);
            earphone_flg = true;
        }
        break;
        default:
        {
            CHGIC_SET_DM_INFO(sw_state, id_state, interrupt_case, chg_det_mode);
            chgic_fix_accessory(set_mode);
            CHGIC_REG_WRITE_DATA(changer_info->client, CHGIC_REG_07H, CHGIC_DATA_ALL_NORMAL);
        }
        break;
    }
}

static void chgic_interrupt_mic_sw(u8 interrupt_case)
{
    if (earphone_flg)
    {
        if ((interrupt_case & CHGIC_DATA_04H_SW) != CHGIC_DATA_04H_SW)
        {
            if ((interrupt_case & CHGIC_DATA_04H_SW_ON) != 0)
            {
                key_dm_driver_set_port(0x11);
                kc_hs_switch_change_state(true);
            }
            else if ((interrupt_case & CHGIC_DATA_04H_SW_OFF) != 0)
            {
                key_dm_driver_set_port(0x10);
                kc_hs_switch_change_state(false);
            }
        }
    }
    else
    {
        bk_interrupt_mask = CHGIC_DATA_06H_STANDBY;
    }
}

static void chgic_update_ovp_state(u8 ovp_state)
{
    kc_changer_ic_ovp_detection_enum current_ovp_state;
    if((ovp_state & CHGIC_OVP_DET_MASK) == CHGIC_OVP_DET_MASK){
        current_ovp_state = KC_CHANGER_OVP_DETECTION;
    }else{
        current_ovp_state = KC_CHANGER_OVP_NOT_DETECTION;
    }
    
    if(current_ovp_state != chgic_ovp_state){
        chgic_ovp_state = current_ovp_state;
        if( chgic_ovp_det_cb != NULL){
            wake_lock_timeout( &chgic_wake_lock, CHGIC_WAKE_LOCK_TIME );
            chgic_ovp_det_cb(chgic_ovp_state);
        }
    }
}

int32_t kc_changer_ic_ovp_det_reg_cbfunc(kc_changer_ic_ovp_det_cb cb)
{
    int32_t ret = 0;

    if (cb != NULL){
        if (chgic_ovp_det_cb != NULL){
            CHGIC_DBG_PRINT("%s: cb func already register (overwrite) \n", __func__);
        }
        chgic_ovp_det_cb =cb;
    }else{
        CHGIC_DBG_PRINT("%s: faild to register cb \n", __func__);
        ret = -1;
    }

    return ret;
}

int32_t kc_changer_ic_ovp_det_unreg_cbfunc(kc_changer_ic_ovp_det_cb cb)
{
    int32_t ret = 0;

    if (chgic_ovp_det_cb != NULL)
    {
        chgic_ovp_det_cb = NULL;
    }else{
        CHGIC_DBG_PRINT("%s: faild to unregister cb  \n", __func__);
        ret = -1;
    }

    return ret;
}

kc_changer_ic_ovp_detection_enum kc_changer_ic_get_ovp_state(void)
{
    return chgic_ovp_state;
}

int32_t kc_changer_ic_reg_cbfunc(struct kc_changer_ic_event_callback* cb)
{
    int32_t i;

    if (changer_cb_info == NULL)
    {
        changer_cb_info = kzalloc(sizeof(struct kc_changer_cb_info), GFP_KERNEL);
        if (changer_cb_info == NULL)
        {
            return -1;
        }
    }
    for (i = 0; i < MAX_CALLBACK_CLIENTS; ++i)
    {
        if (changer_cb_info->cb_tbl[i] == NULL)
        {
            changer_cb_info->cb_tbl[i] = cb;
            return 0;
        }
    }
    return -1;
}

int32_t kc_changer_ic_unreg_cbfunc(struct kc_changer_ic_event_callback* cb)
{
    int32_t i;

    if (changer_cb_info != NULL)
    {
        for (i = 0; i < MAX_CALLBACK_CLIENTS; ++i)
        {
            if (changer_cb_info->cb_tbl[i] == cb)
            {
                changer_cb_info->cb_tbl[i] = NULL;
                return 0;
            }
        }
    }
    return -1;
}

kc_changer_ic_accessory_enum kc_changer_ic_get_accessory(void)
{
    return changer_set_mode;
}

static void chgic_chg_workqueue(struct work_struct *work)
{
    uint32_t type = USB_CHG_TYPE__NONE;
    unsigned chg = 0;
    bool retryflg = false;

    CHGIC_DBG_PRINT("%s: changer_set_mode:%x\n", __func__, changer_set_mode);
    switch (changer_set_mode)
    {
        case KC_CHANGER_NO_ACCESSORY:
            type = USB_CHG_TYPE__INVALID;
            break;
        case KC_CHANGER_USB_MODE:
            type = USB_CHG_TYPE__NONE;
            break;
        case KC_CHANGER_AC_ADAPTER:
            type = USB_CHG_TYPE__WALLCHARGER;
            chg = MAX_CHG_VAL;
            break;
        case  KC_CHANGER_AUDIO_CHG_MIC_MONO:
            type = USB_CHG_TYPE__MONO_EARPHONE;
            chg = MAX_CHG_VAL;
            break;
        case KC_CHANGER_AUDIO_CHG_STEREO:
            type = USB_CHG_TYPE__STEREO_EARPHONE;
            chg = MAX_CHG_VAL;
            break;
        default:
            if (changer_set_mode & CHGIC_MODE_VBUS_MASK)
            {
                type = USB_CHG_TYPE__WALLCHARGER;
                chg = MAX_CHG_VAL;
            }
            else
            {
                type = USB_CHG_TYPE__INVALID;
            }
            break;
    }
    if (type != USB_CHG_TYPE__NONE && type != USB_CHG_TYPE__INVALID)
    {
        if ((msm_chg_usb_charger_connected(type) < 0)
         || (msm_chg_usb_i_is_available(chg) < 0))
        {
            retryflg = true;
        }
    }
    else if (type == USB_CHG_TYPE__INVALID)
    {
        if ((changer_set_mode_prev != KC_CHANGER_USB_MODE) &&
            (changer_set_mode_prev != KC_CHANGER_NO_ACCESSORY) )
        {
            if ((msm_chg_usb_i_is_not_available() < 0)
             || (msm_chg_usb_charger_disconnected() < 0))
            {
                retryflg = true;
            }
        }
    }
    if (retryflg)
    {
        msleep(WAIT_START_RETRY);
        schedule_work(&changer_info->chg_work);
    }
}
static bool kc_changer_delay_fix_accessory(u8 sw_state)
{
    bool ret = false;
    u8   current_id_state = (changer_set_mode & 0xF8 ) >> 3;

    CHGIC_DBG_PRINT("%s: start changer_set_mode:%x\n", __func__, changer_set_mode);

    switch (current_id_state) {
    case CHGIC_DATA_AUDIO_MIC_MONO :    
    case CHGIC_DATA_AUDIO_MIC_MONO_CHG :
    case CHGIC_DATA_AUDIO_MIC_STEREO :  
    case CHGIC_DATA_AUDIO_STEREO :
        if((delay_fix_flag == true) ||
        ((changer_set_mode & CHGIC_DATA_01H_VBUS_DET) != (sw_state & CHGIC_DATA_01H_VBUS_DET)))
        {
            ret = true;
        }
        break;
    default : 
        break;
    }
    CHGIC_DBG_PRINT("%s: end ret = %d ",__func__,(int)ret);

    return ret;
}
static int32_t __devinit chgic_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int32_t ret = 0;

    changer_info = kzalloc(sizeof(struct lc824204_changer), GFP_KERNEL);

    if (!changer_info)
    {
        printk(KERN_ERR "LC824204 failed to allocate driver data\n");
        return -ENOMEM;
    }
    i2c_set_clientdata(client, changer_info);
    changer_info->client = client;

    ret = gpio_request(client->irq, "changer_ic");
    if (ret < 0)
    {
        printk(KERN_ERR "LC824204 failed to request GPIO %d, ret %d\n", client->irq, ret);
        goto failed_free_i2c;
    }
    ret = gpio_direction_input(client->irq);
    if (ret < 0) 
    {
        printk(KERN_ERR "LC824204 failed to configure direction for GPIO %d, error %d\n", client->irq, ret);
        goto failed_free_i2c;
    }
    changer_ic_irq = gpio_to_irq(client->irq);
    if (changer_ic_irq < 0)
    {
        printk(KERN_ERR "LC824204 Unable to get irq number for GPIO %d, error %d\n", client->irq, changer_ic_irq);
        goto failed_free_i2c;
    }

    wake_lock_init( &chgic_wake_lock, WAKE_LOCK_SUSPEND, "changer_ic" );

    INIT_WORK(&changer_info->work, chgic_workqueue);
    INIT_WORK(&changer_info->chg_work, chgic_chg_workqueue);

    ret = request_any_context_irq(changer_ic_irq, chgic_interrupt, IRQF_TRIGGER_FALLING, "changer_ic", changer_info);
    if (ret < 0)
    {
        printk(KERN_ERR "LC824204 Unable to claim irq %d; error %d\n", changer_ic_irq, ret);
        goto failed_free_all;
    }
    disable_irq(changer_ic_irq);

    msleep(2);
    CHGIC_REG_WRITE_DATA(client, CHGIC_REG_05H, CHGIC_DATA_05H_ON);
    CHGIC_REG_WRITE_DATA(client, CHGIC_REG_05H, CHGIC_DATA_05H_SW);
    CHGIC_REG_WRITE_DATA(client, CHGIC_REG_06H, CHGIC_DATA_06H_STANDBY);
    CHGIC_REG_WRITE_DATA(client, CHGIC_REG_00H, CHGIC_DATA_CHG_DET_OFF);
    CHGIC_REG_WRITE_DATA(client, CHGIC_REG_00H, CHGIC_DATA_CHG_DET_ON);
    chgic_interrupt_exec(true);
    enable_irq(changer_ic_irq);
    device_init_wakeup(&client->dev, 1);

    return 0;

failed_free_all:
    free_irq(changer_ic_irq, changer_info);
    changer_ic_irq = 0;
failed_free_i2c:
    i2c_set_clientdata(client, NULL);
    kfree(changer_info);
    changer_info = NULL;

    return ret;
}

static int32_t __exit chgic_remove(struct i2c_client *client)
{
    CHGIC_REG_WRITE_DATA(client, CHGIC_REG_06H, CHGIC_DATA_06H_NO);
    CHGIC_REG_READ_DATA(client, CHGIC_REG_06H);
    i2c_set_clientdata(client, NULL);
    cancel_work_sync(&changer_info->work);
    free_irq(changer_ic_irq, changer_info);
    changer_ic_irq = 0;
    kfree(changer_info);
    changer_info = NULL;
    wake_lock_destroy( &chgic_wake_lock );
    kfree(changer_cb_info);
    changer_cb_info = NULL;
    changer_set_mode = KC_CHANGER_UNINITIALIZE;
    return 0;
}

static const struct i2c_device_id lc824204_id[] = {
    { "LC824204", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, lc824204_id);

static struct i2c_driver kc_changer_ic_driver = {
    .driver = {
        .name   = "LC824204",
    },
    .probe      = chgic_probe,
    .remove     = __exit_p(chgic_remove),
    .suspend    = chgic_suspend,
    .resume     = chgic_resume,
    .id_table   = lc824204_id,
};


static int32_t __init chgic_init(void)
{
    return i2c_add_driver(&kc_changer_ic_driver);
}
module_init(chgic_init);

static void __exit chgic_exit(void)
{
    i2c_del_driver(&kc_changer_ic_driver);
}
module_exit(chgic_exit);

MODULE_AUTHOR("KYOCERA");
MODULE_DESCRIPTION("LC824204 CHANGER Driver");
MODULE_LICENSE("GPL v2");

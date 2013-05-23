/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 *
 * drivers/input/touchscreen/ts_mxt224.c
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

#ifndef XXXDEBUG_LOGIC_TEST
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mfd/pmic8058.h>


#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#endif

#define PM8058_GPIO_PM_TO_SYS(pm_gpio)          (pm_gpio + 182)

#define TOUCH_PRINTK_DEFAULT(fmt, arg...)  printk(KERN_WARNING fmt, ## arg)
#define TOUCH_PRINTK_ERROR(fmt, arg...)    printk(KERN_WARNING fmt, ## arg)
#define TOUCH_PRINTK_DEBUG(fmt, arg...)    if(ts_log_level!=0) printk(KERN_WARNING fmt, ## arg)



#define BUFFER_MAX          10
#define TS_TOUCH_INT1       19

#define MAX_NUM_TOUCHES     10
#define TS_END_OF_MESSAGES  0xFF

#define TS_T6_RESET_ORDER     0x01
#define TS_T6_CALIBRATE_ORDER 0x01
#define TS_T6_BACKUPNV_ORDER  0x55

#define TS_T6_STATUS_RESET    0x80
#define TS_T6_STATUS_OFL      0x40
#define TS_T6_STATUS_CAL      0x10
#define TS_T6_STATUS_CFGERR   0x08

#define TS_T9_STATUS_DETECT   0x80
#define TS_T9_STATUS_PRESS    0x40
#define TS_T9_STATUS_RELEASE  0x20
#define TS_T9_STATUS_SUPPRESS 0x02

#define TS_T42_CTRL_ENABLE    0x01

#define TS_T48_CALCFG_CHRGON  0x20

#define TS_OBJECT_CTRL_FIELD     0
#define TS_OBJECT_RPTEN_BIT   0x02

#define TS_CHARGER_TURN_OFF     0
#define TS_CHARGER_TURN_ON      1
#define TS_CHARGER_AUTO_ADJUST  2


#define TS_I2C_SUCCESS     0
#define TS_I2C_FAILURE     1

#define TS_I2C_MODE_WRITE  0
#define TS_I2C_MODE_READ   1
#define TS_I2C_RETRYCOUNT  5

#define TS_I2C_SLAVE_ADDR  (0x94 >> 1)

#define TS_EVENT_ENABLE    0

#define REPORTID_GEN_COMMANDPROCESSOR_T6         0x01
#define REPORTID_TOUCH_KEYARRAY_T15              0x0C
#define REPORTID_SPI_GPIOPWM_T19                 0x0D
#define REPORTID_TOUCH_PROXIMITY_T23             0x0E
#define REPORTID_SPT_SELFTEST_T25                0x0F
#define REPORTID_TOUCHSUPPRESSION_T42            0x10
#define REPORTID_STP_CTECONFIG_T46               0x11
#define REPORTID_NOISESUPPRESSION_T48            0x12


#define TS_DEVICE_CONDITION_ACTIVE     0
#define TS_DEVICE_CONDITION_SLEEP      1

#define TOUCH_OUTPUT_LOG_ON  1
#define TOUCH_OUTPUT_LOG_OFF 0

extern void touch_kcc_write_log( unsigned char* buf_p, int buf_size, s16 msg_code );

static int  ts_touch_probe0(struct i2c_client *client, const struct i2c_device_id *id);
static u8   ts_touch_i2c_access(int mode, struct i2c_client *client, char *buf, int size);
static void ts_update_command_processor(void);

const unsigned char ini_t7[]={  0x40,0xff,0x32 };

const unsigned char ini_t8[]={  0x16,0x00,0x05,0x01,0x00,0x00,0x00,0x0C,
                                0x0C,0x02 };

const unsigned char ini_t9[]={  0x8B,0x00,0x00,0x13,0x0B,0x00,0x20,0x3E,
                                0x02,0x01,0x0a,0x03,0x01,0x2F,0x0a,0x14,
                                0x32,0x14,0x1F,0x03,0xDF,0x01,0x01,0x01,
                                0x08,0x08,0xE0,0x2D,0xDF,0x4B,0x14,0x0f,
                                0x00,0x00,0x02 };

const unsigned char ini_t15[]={ 0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,
                                0x00,0x00,0x00 };

const unsigned char ini_t19[]={ 0x01,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,
                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };

const unsigned char ini_t40[]={ 0x00,0x00,0x00,0x00,0x00 };

const unsigned char ini_t42[]={ 0x00,0x00,0x40,0x40,0xFF,0x00,0x00,0x00 };

const unsigned char ini_t46[]={ 0x00,0x03,0x10,0x20,0x00,0x00,0x01,0x00,
                                0x00 };

const unsigned char ini_t47[]={ 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                0x00,0x00 };

const unsigned char ini_t48[]={ 0x01,0x85,0x62,0x00,0x00,0x00,0x00,0x00,
                                0x0A,0x0F,0x00,0x00,0x00,0x06,0x06,0x00,
                                0x00,0x64,0x04,0x40,0x0A,0x00,0x14,0x05,
                                0x00,0x26,0x00,0x14,0x00,0x00,0x00,0x00,
                                0x00,0x00,0x00,0x28,0x01,0x01,0x01,0x2F,
                                0x0A,0x14,0x28,0x01,0x01,0x01,0x01,0xE4,
                                0x2D,0xE1,0x4B,0x19,0x0F,0x02 };

static unsigned char touch_debug_message   = 0;
static unsigned char touch_debug_charge    = 1;

static struct workqueue_struct *tstouch_work0;

struct ts_ID_Infromation {
  u8  FamilyID;
  u8  ValiantID;
  u8  Version;
  u8  Build;
  u8  xSize;
  u8  ySize;
  u8  elements;
};

struct ts_ObjectTable {
  u8  Type;
  u16 StartPosiotion;
  u8  Size;
  u8  Instance;
  u8  ReportID;
};


struct ts_mxt224_Info {
  struct ts_ID_Infromation info;
  struct ts_ObjectTable    table[255];
  u8      panel_T7register[3];
  u8      panel_T8register[10];
  u8      panel_T9register[35];
  u8      panel_T15register[11];
  u8      panel_T19register[16];
  u8      panel_T20register[12];
  u8      panel_T22register[17];
  u8      panel_T28register[6];
  u8      panel_T40register[5];
  u8      panel_T42register[8];
  u8      panel_T46register[9];
  u8      panel_T47register[10];
  u8      panel_T48register[54];
  int     message_processor_address;

  u16     add_t6;
  u16     add_t7;
  u16     add_t8;
  u16     add_t9;
  u16     add_t15;
  u16     add_t17;
  u16     add_t18;
  u16     add_t19;
  u16     add_t20;
  u16     add_t22;
  u16     add_t23;
  u16     add_t24;
  u16     add_t25;
  u16     add_t27;
  u16     add_t28;
  u16     add_t37;
  u16     add_t38;
  u16     add_t40;
  u16     add_t42;
  u16     add_t46;
  u16     add_t47;
  u16     add_t48;

  int     touch_start;
  int     touch_end;

  int     irq;

  struct i2c_client    *client;
};


struct ts_event {
  int  x;
  int  y;
  int  state;
  int  width;
};


struct ts_touch {
  struct input_dev     *input;
  char                 phys[32];

  struct work_struct   work0;

  int                  lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
  struct early_suspend early_suspend;
#endif

  struct ts_mxt224_Info panel_info;
  struct ts_event       tc[MAX_NUM_TOUCHES];
  u8                    touch_count;
  int                   early_suspend_sleep;
  
  u8                    device_condition;
  u8                    t6_status_value;
  u16                   detect_reset;
  u8                    last_reportid;
  u8                    model_type;
  u16                   queue_count;
  s16                   charge_condition;
  u8                    object_field_chk;
};


static struct ts_touch *ts_data;
static u16 ts_touch_read_values(struct ts_mxt224_Info *tsc);


#define TP_I2CLOCK() while(ts_data->lock)mdelay(1);ts_data->lock=1;
#define TP_I2CUNLOCK() ts_data->lock=0;

static int ts_diag_start_flag = 0;

struct ts_event_diag {
  int  x;
  int  y;
  int  width;
};

static struct ts_diag_type *diag_data;

static u16 ts_diag_lock = 0; 
#define DIAGDATA_LOCK() while(ts_diag_lock)mdelay(1);ts_diag_lock=1;
#define DIAGDATA_UNLOCK() ts_diag_lock=0;

static int ts_m_diag_count = 0;

char ts_log_level = 0;
char ts_event_control = TS_EVENT_ENABLE;

static int ts_touch_init_device(struct ts_mxt224_Info *data);

static int ts_touch_T42_enable_ctrl( int chg )
{
  u8  buf[10];
  u8  rd_data = 0;
  u8  wr_data = 0;
  u8  i2c_ret;
  int ret = 0;
  int log_id = 0;

  if(chg==TS_CHARGER_AUTO_ADJUST){
     chg = ts_data->charge_condition;
  }

  buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t42].StartPosiotion&0x00ff);
  buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t42].StartPosiotion&0xff00)>>8;

  i2c_ret = i2c_master_send(ts_data->panel_info.client, buf, 2);

  i2c_ret = i2c_master_recv(ts_data->panel_info.client, buf, 1);
  TOUCH_PRINTK_DEBUG("[%s] T42 CTRL Field Read Val: 0x%02x\n",__func__, buf[0] );

  rd_data = buf[0];

  if( ((( rd_data & TS_T42_CTRL_ENABLE) == TS_T42_CTRL_ENABLE )&& ( chg == 0 )) ||
      ((( rd_data & TS_T42_CTRL_ENABLE) == 0 )&& ( chg == 1 ))  ){

    TOUCH_PRINTK_DEBUG("[%s] condition and request value is equal: CTRL(0x%02x) req value(%d)\n",__func__, rd_data, chg );
  }
  else{
    if( TS_CHARGER_TURN_ON == chg )
    {
      wr_data = (rd_data & (~TS_T42_CTRL_ENABLE));
      log_id = 22;
    }
    else
    {
      wr_data = (rd_data | TS_T42_CTRL_ENABLE);
      log_id = 23;
    }

    buf[0] = (ts_data->panel_info.table[ts_data->panel_info.add_t42].StartPosiotion&0x00ff);
    buf[1] = (ts_data->panel_info.table[ts_data->panel_info.add_t42].StartPosiotion&0xff00)>>8;
    buf[2] = wr_data;

    i2c_ret = i2c_master_send(ts_data->panel_info.client, buf, 3);
    TOUCH_PRINTK_DEBUG("[%s] T42 Write: 0x%02x 0x%02x 0x%02x\n",__func__, buf[0], buf[1], buf[2] );

    ret = (int)i2c_ret;

    if( touch_debug_message > 0 ){
      buf[0] = buf[2];
      touch_kcc_write_log( &buf[0],8, log_id );
    }
  }

  return ret;
}

static int ts_touch_T48_chrgon_ctrl( int chg )
{
  u8  buf[10];
  u8  rd_data = 0;
  u8  wr_data = 0;
  u8  i2c_ret;
  int ret = 0;
  int log_id = 0;

  if(chg==TS_CHARGER_AUTO_ADJUST){
     chg = ts_data->charge_condition;
  }
  
  buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t48].StartPosiotion&0x00ff);
  buf[0]=(buf[0]+2);
  buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t48].StartPosiotion&0xff00)>>8;

  i2c_ret = i2c_master_send(ts_data->panel_info.client, buf, 2);

  i2c_ret = i2c_master_recv(ts_data->panel_info.client, buf, 1);
  TOUCH_PRINTK_DEBUG("[%s] T48 CALCFG Read Val: 0x%02x\n",__func__, buf[0] );

  rd_data = buf[0];

  if( ((( rd_data & TS_T48_CALCFG_CHRGON) == TS_T48_CALCFG_CHRGON )&& ( chg == 1 )) ||
      ((( rd_data & TS_T48_CALCFG_CHRGON) == 0 )&& ( chg == 0 ))  ){

    TOUCH_PRINTK_DEBUG("[%s] condition and request value is equal: CALCFG(0x%02x) req value(%d)\n",__func__, rd_data, chg );
  }
  else{
    if( 0x01 == chg )
    {
      wr_data = (rd_data | TS_T48_CALCFG_CHRGON);
      log_id = 8;
    }
    else
    {
      wr_data = (rd_data & (~TS_T48_CALCFG_CHRGON));
      log_id = 9;
    }

    buf[0] = (ts_data->panel_info.table[ts_data->panel_info.add_t48].StartPosiotion&0x00ff);
    buf[0]=(buf[0]+2);
    buf[1] = (ts_data->panel_info.table[ts_data->panel_info.add_t48].StartPosiotion&0xff00)>>8;
    buf[2] = wr_data;

    i2c_ret = i2c_master_send(ts_data->panel_info.client, buf, 3);
    TOUCH_PRINTK_DEBUG("[%s] T48 Write: 0x%02x 0x%02x 0x%02x\n",__func__, buf[0], buf[1], buf[2] );

    ret = (int)i2c_ret;

    if( touch_debug_message > 0 ){
      buf[0] = buf[2];
      touch_kcc_write_log( &buf[0],8, log_id );
    }
  }

  return ret;
}

int ts_touch_power_charge( int chg )
{
  u8  buf[10];
  int ret = 0;

  if( touch_debug_message > 0 ){
    buf[0] = chg;
    touch_kcc_write_log( &buf[0], 8 , 21 );
  }

  if( chg > 0 ){
    chg = TS_CHARGER_TURN_ON;
  }

  ts_data->charge_condition = chg;

  if( ts_data->object_field_chk > 0 ){
    
    TP_I2CLOCK();
    ts_touch_T48_chrgon_ctrl(chg);
    ts_touch_T42_enable_ctrl(chg);
    TP_I2CUNLOCK();
    
  }
  else{
    TOUCH_PRINTK_DEBUG("[%s] Skip CHRON Bit Set\n",__func__ );
  }

  TOUCH_PRINTK_DEBUG("[%s] Charge condition (%d)\n",__func__, chg );

  return ret;

}

static int ts_confirm_check_values(u16 position, u8 size, u8 *base)
{
    u8  write_buf[60];
    u8  read_buf[60];
    u8  result = TS_I2C_FAILURE;
    int ret;
    int count;

    write_buf[0]=(position&0x00ff);
    write_buf[1]=(position&0xff00)>>8;
    result = ts_touch_i2c_access(TS_I2C_MODE_WRITE,ts_data->panel_info.client, write_buf, 2);

    result = ts_touch_i2c_access(TS_I2C_MODE_READ,ts_data->panel_info.client, read_buf, size);
    ret = memcmp( base, &read_buf, size );

    if( ret != 0 ){
      TOUCH_PRINTK_DEBUG("[%s] Compare Result(%d)\n", __func__, ret);
      TOUCH_PRINTK_DEBUG("        Device/ NV value\n" );
      for(count=0; count < size ; count++){
          TOUCH_PRINTK_DEBUG("     [%d] 0x%02x / 0x%02x\n", count, read_buf[count], *(base+count) );
      }
    }

    return ret;
}

static void ts_touch_confirm_object(void)
{

  int cmp_result;
  u8  buf[8];
  u8  result;
  u8  save_buf[2];

  do{
    
    if( ts_data->early_suspend_sleep == TS_DEVICE_CONDITION_SLEEP ){
      save_buf[0]=ts_data->panel_info.panel_T7register[0];
      save_buf[1]=ts_data->panel_info.panel_T7register[1];
      ts_data->panel_info.panel_T7register[0]=0;
      ts_data->panel_info.panel_T7register[1]=0;
    }

    cmp_result = ts_confirm_check_values(
      ts_data->panel_info.table[ts_data->panel_info.add_t7].StartPosiotion,
      ts_data->panel_info.table[ts_data->panel_info.add_t7].Size,
      &(ts_data->panel_info.panel_T7register[0]));

    if( ts_data->early_suspend_sleep == TS_DEVICE_CONDITION_SLEEP ){
      ts_data->panel_info.panel_T7register[0]=save_buf[0];
      ts_data->panel_info.panel_T7register[1]=save_buf[1];
    }

    if( cmp_result != 0 ){
      break;
    }
  
    cmp_result = ts_confirm_check_values(
      ts_data->panel_info.table[ts_data->panel_info.add_t8].StartPosiotion,
      ts_data->panel_info.table[ts_data->panel_info.add_t8].Size,
      &(ts_data->panel_info.panel_T8register[0]));
    if( cmp_result != 0 ){
      break;
    }
  
    cmp_result = ts_confirm_check_values(
      ts_data->panel_info.table[ts_data->panel_info.add_t9].StartPosiotion,
      ts_data->panel_info.table[ts_data->panel_info.add_t9].Size,
      &(ts_data->panel_info.panel_T9register[0]));
    if( cmp_result != 0 ){
      break;
    }

    cmp_result = ts_confirm_check_values(
      ts_data->panel_info.table[ts_data->panel_info.add_t15].StartPosiotion,
      ts_data->panel_info.table[ts_data->panel_info.add_t15].Size,
      &(ts_data->panel_info.panel_T15register[0]));
    if( cmp_result != 0 ){
      break;
    }
  
    cmp_result = ts_confirm_check_values(
      ts_data->panel_info.table[ts_data->panel_info.add_t19].StartPosiotion,
      ts_data->panel_info.table[ts_data->panel_info.add_t19].Size,
      &(ts_data->panel_info.panel_T19register[0]));
    if( cmp_result != 0 ){
      break;
    }

    cmp_result = ts_confirm_check_values(
      ts_data->panel_info.table[ts_data->panel_info.add_t40].StartPosiotion,
      ts_data->panel_info.table[ts_data->panel_info.add_t40].Size,
      &(ts_data->panel_info.panel_T40register[0]));
    if( cmp_result != 0 ){
      break;
    }
  
    if( ts_data->charge_condition == TS_CHARGER_TURN_OFF ){
      ts_data->panel_info.panel_T42register[0] = (ts_data->panel_info.panel_T42register[0] | TS_T42_CTRL_ENABLE );
    }

    cmp_result = ts_confirm_check_values(
      ts_data->panel_info.table[ts_data->panel_info.add_t42].StartPosiotion,
      ts_data->panel_info.table[ts_data->panel_info.add_t42].Size,
      &(ts_data->panel_info.panel_T42register[0]));

    if( ts_data->charge_condition == TS_CHARGER_TURN_OFF ){
      ts_data->panel_info.panel_T42register[0] = (ts_data->panel_info.panel_T42register[0] & ~(TS_T42_CTRL_ENABLE));
    }

    if( cmp_result != 0 ){
      break;
    }

    cmp_result = ts_confirm_check_values(
      ts_data->panel_info.table[ts_data->panel_info.add_t46].StartPosiotion,
      ts_data->panel_info.table[ts_data->panel_info.add_t46].Size,
      &(ts_data->panel_info.panel_T46register[0]));
    if( cmp_result != 0 ){
      break;
    }

    cmp_result = ts_confirm_check_values(
      ts_data->panel_info.table[ts_data->panel_info.add_t47].StartPosiotion,
      ts_data->panel_info.table[ts_data->panel_info.add_t47].Size,
      &(ts_data->panel_info.panel_T47register[0]));
    if( cmp_result != 0 ){
      break;
    }


    if( ts_data->charge_condition == TS_CHARGER_TURN_OFF ){
      ts_data->panel_info.panel_T48register[2] = (ts_data->panel_info.panel_T48register[2] & ~(TS_T48_CALCFG_CHRGON));
    }

    cmp_result = ts_confirm_check_values(
      ts_data->panel_info.table[ts_data->panel_info.add_t48].StartPosiotion,
      ts_data->panel_info.table[ts_data->panel_info.add_t48].Size,
      &(ts_data->panel_info.panel_T48register[0]));

    if( ts_data->charge_condition == TS_CHARGER_TURN_OFF ){
      ts_data->panel_info.panel_T48register[2] = (ts_data->panel_info.panel_T48register[2] | TS_T48_CALCFG_CHRGON );
    }

    if( cmp_result != 0 ){
      break;
    }

  }while(0);

  if( cmp_result != 0 ){
    ts_update_command_processor();

    buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t6].StartPosiotion&0x00ff);
    buf[0]=(buf[0]+1);
    buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t6].StartPosiotion&0xff00)>>8;
    buf[2]=TS_T6_BACKUPNV_ORDER;
    result = ts_touch_i2c_access(TS_I2C_MODE_WRITE,ts_data->panel_info.client, buf, 3);
    TOUCH_PRINTK_DEBUG("     BACKUPNV Result(%d): 0x%02x 0x%02x 0x%02x\n", result, buf[0],buf[1],buf[2] );

  }

  if( ts_data->charge_condition == TS_CHARGER_TURN_OFF ){
    ts_touch_T48_chrgon_ctrl(TS_CHARGER_TURN_OFF);
    ts_touch_T42_enable_ctrl(TS_CHARGER_TURN_OFF);
  }

  TOUCH_PRINTK_DEBUG("[E]%s\n",__func__);

}

static void ts_touch_software_reset(void)
{
  u8  buf[8];
  u8  retry_cnt;
  u8  loop_cnt;
  int i2c_access_length;

  for(retry_cnt=0; retry_cnt < 2; retry_cnt++){

    buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t6].StartPosiotion&0x00ff);
    buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t6].StartPosiotion&0xff00)>>8;
    buf[2]=TS_T6_RESET_ORDER;
    i2c_master_send(ts_data->panel_info.client, buf, 3);

    ts_data->t6_status_value = 0;

    msleep(75); 

    for(loop_cnt=0; loop_cnt < (160-75)/5; loop_cnt++){

      buf[0] =   ts_data->panel_info.message_processor_address & 0xff;
      buf[1] = ((ts_data->panel_info.message_processor_address) >> 8);
      i2c_access_length = i2c_master_send(ts_data->panel_info.client, buf, 2);
      if(i2c_access_length == 2){

        i2c_access_length = i2c_master_recv(ts_data->panel_info.client, buf, 8);
        if(i2c_access_length == 8){
        
          TOUCH_PRINTK_DEBUG("[%s] Report ID:%d 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
               __func__, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6]);

          ts_data->last_reportid = buf[0];
          if( ts_data->last_reportid == ts_data->panel_info.table[ts_data->panel_info.add_t6].ReportID ){
            ts_data->t6_status_value = buf[1];

            if(ts_data->t6_status_value & TS_T6_STATUS_RESET) {
              break;
            }
          }
        }
      }
      msleep(5);
    }

    if(ts_data->t6_status_value & TS_T6_STATUS_RESET) {
      ts_data->detect_reset = true;
      TOUCH_PRINTK_DEBUG("[%s] reset complete\n",__func__);
      if( touch_debug_message > 0 ){
          touch_kcc_write_log( &buf[0],8,3);
      }
      break;
    }
    else{
      TOUCH_PRINTK_ERROR("[%s] reset fail\n",__func__);
      if( touch_debug_message > 0 ){
          touch_kcc_write_log( &buf[0],8,6);
      }
    }
  }

  TOUCH_PRINTK_DEBUG("[E][%s] \n",__func__);

}

static void ts_select_command_processor_value(void)
{
  memcpy(ts_data->panel_info.panel_T7register, ini_t7, sizeof(ini_t7));
  memcpy(ts_data->panel_info.panel_T8register, ini_t8, sizeof(ini_t8));
  memcpy(ts_data->panel_info.panel_T9register, ini_t9, sizeof(ini_t9));
  memcpy(ts_data->panel_info.panel_T15register,ini_t15,sizeof(ini_t15));
  memcpy(ts_data->panel_info.panel_T19register,ini_t19,sizeof(ini_t19));
  memcpy(ts_data->panel_info.panel_T40register,ini_t40,sizeof(ini_t40));
  memcpy(ts_data->panel_info.panel_T42register,ini_t42,sizeof(ini_t42));
  memcpy(ts_data->panel_info.panel_T46register,ini_t46,sizeof(ini_t46));
  memcpy(ts_data->panel_info.panel_T47register,ini_t47,sizeof(ini_t47));
  memcpy(ts_data->panel_info.panel_T48register,ini_t48,sizeof(ini_t48));

  TOUCH_PRINTK_DEBUG("[%s]\n",__func__);

}

static void ts_update_command_processor(void)
{
  u8  buf[128];

  u8  i2c_access_result = TS_I2C_FAILURE;

    buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t7].StartPosiotion&0x00ff);
    buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t7].StartPosiotion&0xff00)>>8;
    memcpy(&buf[2],ts_data->panel_info.panel_T7register,ts_data->panel_info.table[ts_data->panel_info.add_t7].Size);
    i2c_access_result = i2c_master_send( ts_data->panel_info.client, buf, ts_data->panel_info.table[ts_data->panel_info.add_t7].Size+2);
      TOUCH_PRINTK_DEBUG("ts_touch:panel_T7register write(write vol=%d/write adrs=%d)\n", i2c_access_result, 
            ts_data->panel_info.table[ts_data->panel_info.add_t7].StartPosiotion);

    buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t8].StartPosiotion&0x00ff);
    buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t8].StartPosiotion&0xff00)>>8;
    memcpy(&buf[2],ts_data->panel_info.panel_T8register,ts_data->panel_info.table[ts_data->panel_info.add_t8].Size);
    i2c_access_result = i2c_master_send( ts_data->panel_info.client, buf, ts_data->panel_info.table[ts_data->panel_info.add_t8].Size+2);
      TOUCH_PRINTK_DEBUG("ts_touch:panel_T8register write(write vol=%d/write adrs=%d)\n", i2c_access_result, 
            ts_data->panel_info.table[ts_data->panel_info.add_t8].StartPosiotion);

    buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t9].StartPosiotion&0x00ff);
    buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t9].StartPosiotion&0xff00)>>8;
    memcpy(&buf[2],ts_data->panel_info.panel_T9register,ts_data->panel_info.table[ts_data->panel_info.add_t9].Size);
    i2c_access_result = i2c_master_send( ts_data->panel_info.client, buf, ts_data->panel_info.table[ts_data->panel_info.add_t9].Size+2 );
      TOUCH_PRINTK_DEBUG("ts_touch:panel_T9register write(write vol=%d/write adrs=%d)\n", i2c_access_result, 
            ts_data->panel_info.table[ts_data->panel_info.add_t9].StartPosiotion);

    buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t15].StartPosiotion&0x00ff);
    buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t15].StartPosiotion&0xff00)>>8;
    memcpy(&buf[2],ts_data->panel_info.panel_T15register,ts_data->panel_info.table[ts_data->panel_info.add_t15].Size);
    i2c_access_result = i2c_master_send( ts_data->panel_info.client, buf, ts_data->panel_info.table[ts_data->panel_info.add_t15].Size+2);
      TOUCH_PRINTK_DEBUG("ts_touch:panel_T15register write(write vol=%d/write adrs=%d)\n", i2c_access_result, 
            ts_data->panel_info.table[ts_data->panel_info.add_t15].StartPosiotion);

    buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t19].StartPosiotion&0x00ff);
    buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t19].StartPosiotion&0xff00)>>8;
    memcpy(&buf[2],ts_data->panel_info.panel_T19register,ts_data->panel_info.table[ts_data->panel_info.add_t19].Size);
    i2c_master_send( ts_data->panel_info.client, buf, ts_data->panel_info.table[ts_data->panel_info.add_t19].Size+2);
    TOUCH_PRINTK_DEBUG("ts_touch:panel_T19register write(write vol=%d/write adrs=%d)\n", i2c_access_result, 
            ts_data->panel_info.table[ts_data->panel_info.add_t19].StartPosiotion);

    buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t40].StartPosiotion&0x00ff);
    buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t40].StartPosiotion&0xff00)>>8;
    memcpy(&buf[2],ts_data->panel_info.panel_T40register,ts_data->panel_info.table[ts_data->panel_info.add_t40].Size);
    i2c_access_result = i2c_master_send( ts_data->panel_info.client, buf, ts_data->panel_info.table[ts_data->panel_info.add_t40].Size+2);
    TOUCH_PRINTK_DEBUG("ts_touch:panel_T40register write(write vol=%d/write adrs=%d)\n", i2c_access_result, 
        ts_data->panel_info.table[ts_data->panel_info.add_t40].StartPosiotion);

    buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t42].StartPosiotion&0x00ff);
    buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t42].StartPosiotion&0xff00)>>8;
    memcpy(&buf[2],ts_data->panel_info.panel_T42register,ts_data->panel_info.table[ts_data->panel_info.add_t42].Size);
    i2c_access_result = i2c_master_send( ts_data->panel_info.client, buf, ts_data->panel_info.table[ts_data->panel_info.add_t42].Size+2);
    TOUCH_PRINTK_DEBUG("ts_touch:panel_T42register write(write vol=%d/write adrs=%d)\n", i2c_access_result, 
        ts_data->panel_info.table[ts_data->panel_info.add_t42].StartPosiotion);

    buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t46].StartPosiotion&0x00ff);
    buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t46].StartPosiotion&0xff00)>>8;
    memcpy(&buf[2],ts_data->panel_info.panel_T46register,ts_data->panel_info.table[ts_data->panel_info.add_t46].Size);
    i2c_access_result = i2c_master_send( ts_data->panel_info.client, buf, ts_data->panel_info.table[ts_data->panel_info.add_t46].Size+2);
    TOUCH_PRINTK_DEBUG("ts_touch:panel_T46register write(write vol=%d/write adrs=%d)\n", i2c_access_result, 
        ts_data->panel_info.table[ts_data->panel_info.add_t46].StartPosiotion);

    buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t47].StartPosiotion&0x00ff);
    buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t47].StartPosiotion&0xff00)>>8;
    memcpy(&buf[2],ts_data->panel_info.panel_T47register,ts_data->panel_info.table[ts_data->panel_info.add_t47].Size);
    i2c_access_result = i2c_master_send( ts_data->panel_info.client, buf, ts_data->panel_info.table[ts_data->panel_info.add_t47].Size+2);
    TOUCH_PRINTK_DEBUG("ts_touch:panel_T47register write(write vol=%d/write adrs=%d)\n", i2c_access_result, 
        ts_data->panel_info.table[ts_data->panel_info.add_t47].StartPosiotion);

    buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t48].StartPosiotion&0x00ff);
    buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t48].StartPosiotion&0xff00)>>8;
    memcpy(&buf[2],ts_data->panel_info.panel_T48register,ts_data->panel_info.table[ts_data->panel_info.add_t48].Size);
    i2c_access_result = i2c_master_send( ts_data->panel_info.client, buf, ts_data->panel_info.table[ts_data->panel_info.add_t48].Size+2);
    TOUCH_PRINTK_DEBUG("ts_touch:panel_T48register write(write vol=%d/write adrs=%d)\n", i2c_access_result, 
        ts_data->panel_info.table[ts_data->panel_info.add_t48].StartPosiotion);

  TOUCH_PRINTK_DEBUG("[%s]\n",__func__);

}

static u8 ts_touch_i2c_access(int mode, struct i2c_client *client, char *buf, int size)
{
  u8  i2c_access_count;
  u8  i2c_access_result = TS_I2C_FAILURE;
  int i2c_access_length;

  for( i2c_access_count = 0; i2c_access_count<TS_I2C_RETRYCOUNT ; i2c_access_count++ ){
    if( mode == TS_I2C_MODE_WRITE )
      i2c_access_length = i2c_master_send(client, buf, size);
    else
      i2c_access_length = i2c_master_recv(client, buf, size);

    if( i2c_access_length == size ){
      i2c_access_result = TS_I2C_SUCCESS;
      break;
    }
  }

  if( i2c_access_result == TS_I2C_FAILURE ){
    if( ts_data->device_condition ){
      ts_touch_software_reset();
      if(ts_data->t6_status_value & TS_T6_STATUS_RESET) {
        TOUCH_PRINTK_ERROR("[E][%s] Done Reset. mode(%d)/result(%d)/access_length(%d)\n",__func__, mode, i2c_access_result,i2c_access_length);
        ts_touch_T48_chrgon_ctrl(TS_CHARGER_AUTO_ADJUST);
        ts_touch_T42_enable_ctrl(TS_CHARGER_AUTO_ADJUST);
      }
    }
    else{
      TOUCH_PRINTK_ERROR("[E][%s] Device deactive. mode(%d)/result(%d)/access_length(%d)\n",__func__, mode, i2c_access_result,i2c_access_length);
    }
  }

  return i2c_access_result;

}

static void ts_touch_send_event(void)
{
  struct input_dev *input = ts_data->input;
  u8  n=0;
  int push = 0;
  int btn = 0;

  if (ts_diag_start_flag != 0) {
    memset(diag_data,0,sizeof(struct ts_diag_type));
    ts_m_diag_count = 0;
  }

  for(n=0; n<MAX_NUM_TOUCHES ; n++)
  {
    if( ts_data->tc[n].state ){
      if(ts_data->tc[n].state & TS_T9_STATUS_PRESS){
        TOUCH_PRINTK_DEBUG("[%s] <Press> [x=%d/y=%d]\n", __func__ , ts_data->tc[n].x, ts_data->tc[n].y);
        push = 255;
        btn = 1;
      }
      else if(ts_data->tc[n].state & TS_T9_STATUS_DETECT){
        TOUCH_PRINTK_DEBUG("[%s] <Detect> [x=%d/y=%d]\n", __func__ , ts_data->tc[n].x, ts_data->tc[n].y);
        push = 255;
        btn = 1;
      }
      else if((ts_data->tc[n].state & TS_T9_STATUS_RELEASE) || (ts_data->tc[n].state & TS_T9_STATUS_SUPPRESS)){
        TOUCH_PRINTK_DEBUG("[%s] <Release> [x=%d/y=%d]\n", __func__ , ts_data->tc[n].x, ts_data->tc[n].y);
        push = 0;
        btn = 0;
      }
      else{
        TOUCH_PRINTK_DEBUG("[%s] <unknown> [x=%d/y=%d]\n", __func__ , ts_data->tc[n].x, ts_data->tc[n].y);
        ts_data->touch_count--;
        push = 0;
        btn = 0;
      }

      if(ts_event_control == TS_EVENT_ENABLE ){
          input_report_key(input, BTN_TOUCH, btn);
          input_report_abs(input, ABS_MT_TOUCH_MAJOR,push);
          input_report_abs(input, ABS_MT_POSITION_X, ts_data->tc[n].x);
          input_report_abs(input, ABS_MT_POSITION_Y, ts_data->tc[n].y);
          input_report_abs(input, ABS_MT_WIDTH_MAJOR,ts_data->tc[n].width<<2);
          input_report_abs(input, ABS_MT_TRACKING_ID, n);
          input_report_abs(input, ABS_MT_PRESSURE,  push);
          TOUCH_PRINTK_DEBUG("ts_touch_send_event: <TrackingID> [%d]\n", n);
          input_mt_sync(input);
      }
      else{
          TOUCH_PRINTK_DEBUG("Disable input_report_abs:<TrackingID:%d>[x=%d/y=%d] \n", n, ts_data->tc[n].x, ts_data->tc[n].y);
      }

      if(( ts_data->tc[n].state&TS_T9_STATUS_RELEASE ) || (ts_data->tc[n].state & TS_T9_STATUS_SUPPRESS)){
        ts_data->tc[n].state=0;
        ts_data->tc[n].x=0;
        ts_data->tc[n].y=0;
        ts_data->tc[n].width=0;
        ts_data->touch_count--;
      }

      if (ts_diag_start_flag != 0) {
        diag_data->ts[n].x = ts_data->tc[n].x;
        diag_data->ts[n].y = ts_data->tc[n].y;
        diag_data->ts[n].width = ts_data->tc[n].width<<2;
        ts_m_diag_count = ts_data->touch_count;

        TOUCH_PRINTK_DEBUG("####### DIAG BASE[x=%d/y=%d/width:[%d]] [x=%d/y=%d/width:[%d]]#########\n",
               ts_data->tc[n].x,
               ts_data->tc[n].y,
               ts_data->tc[n].width<<2,
               diag_data->ts[n].x, 
               diag_data->ts[n].y, 
               diag_data->ts[n].width);
      }
    }
  }

  if(ts_event_control == TS_EVENT_ENABLE ){
    TOUCH_PRINTK_DEBUG("ts_touch_send_event: <input_sync>\n");
    input_sync(input);
  }
  else{
    TOUCH_PRINTK_DEBUG("Disable input_sync \n");
  }

  if (ts_diag_start_flag != 0) {
      diag_data->diag_count = ts_m_diag_count;
  }

  TOUCH_PRINTK_DEBUG("ts_touch_send_event end: [count=%d]\n",ts_data->touch_count);


}

static u16 ts_touch_read_values(struct ts_mxt224_Info *tsc)
{
  u8  buf[64];
  int x,y;
  u16 loop_cnt;
  int ret_code = 1;

  u8 invalid_report_id=0;

  u8   count;

  if(0 == ts_data->device_condition){
      return ret_code;
  }

  ts_data->detect_reset = false;

  for(loop_cnt=0; loop_cnt < 23; loop_cnt++)
  {
    buf[0] =   tsc->message_processor_address & 0xff;
    buf[1] = ((tsc->message_processor_address) >> 8);

    if( ts_touch_i2c_access(TS_I2C_MODE_WRITE,tsc->client, buf, 2) == TS_I2C_FAILURE ){
      break;
    }
    if( ts_touch_i2c_access(TS_I2C_MODE_READ, tsc->client, buf, 8) == TS_I2C_FAILURE ){
      break;
    }

    if( touch_debug_message > 0 ){
        touch_kcc_write_log( &buf[0],8,0);
    }

    ts_data->last_reportid = buf[0];
    
    if( ts_data->last_reportid == TS_END_OF_MESSAGES ){
      TOUCH_PRINTK_DEBUG("[%s] process end(0x%x)\n", __func__, ts_data->last_reportid );
      break;
    }

    TOUCH_PRINTK_DEBUG("[%s] reportID(0x%x)\n", __func__, ts_data->last_reportid );
    if(buf[0] >= tsc->touch_start && buf[0] <= tsc->touch_end){

      if(( ts_data->panel_info.panel_T9register[TS_OBJECT_CTRL_FIELD] & TS_OBJECT_RPTEN_BIT ) != 0){
        x=((buf[2]<<2)| (buf[4]>>6));
        y=((buf[3]<<2)|((buf[4]&0x0c)>>2));

        TOUCH_PRINTK_DEBUG("[%s]Message ID=%d (Report ID:%d) <X=%d/Y=%d>[STATUS=0x%02x/XHi=0x%02x/YHi=0x%02x/XYLow=0x%02x/width=0x%02x] 0x%02x\n",
               __func__, (buf[0]-tsc->touch_start), buf[0], x , y, buf[1],buf[2],buf[3],buf[4],buf[5],buf[6] );

        ts_data->tc[(buf[0]-tsc->touch_start)].state=buf[1];
        ts_data->tc[(buf[0]-tsc->touch_start)].x=x;
        ts_data->tc[(buf[0]-tsc->touch_start)].y=y;
        ts_data->tc[(buf[0]-tsc->touch_start)].width=buf[5];

        if( ts_data->tc[(buf[0]-tsc->touch_start)].state & TS_T9_STATUS_PRESS ){
          ts_data->touch_count++;
        }

        ret_code = 0;
      }
      else{
        TOUCH_PRINTK_DEBUG("[%s] Detect invalid Message \n",__func__);
        if( touch_debug_message > 0 ){
          touch_kcc_write_log( &buf[0],8,10);
        }
        ts_touch_software_reset();
      }
    }
    else{

      TOUCH_PRINTK_DEBUG("[%s] Report ID:%d 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
               __func__, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6]);
               
      invalid_report_id=0;
      switch(ts_data->last_reportid){

        case REPORTID_GEN_COMMANDPROCESSOR_T6:
          if(buf[1] & TS_T6_STATUS_RESET) {
              TOUCH_PRINTK_DEBUG("[%s] device has reset\n",__func__);
              ts_data->detect_reset = true;
          } 
          if(buf[1] & TS_T6_STATUS_CAL) {
            TOUCH_PRINTK_DEBUG("[%s] Device is calibrating\n",__func__);
          } 
          if(buf[1] & TS_T6_STATUS_OFL) {
            TOUCH_PRINTK_DEBUG("[%s] overflowed\n",__func__);
            invalid_report_id = 2;
          } 
          if(buf[1] & TS_T6_STATUS_CFGERR) {
            TOUCH_PRINTK_DEBUG("[%s] Configuration Error\n",__func__);
            invalid_report_id = 11;
          } 
          break;

        case REPORTID_TOUCH_KEYARRAY_T15:
          if(( ts_data->panel_info.panel_T15register[TS_OBJECT_CTRL_FIELD] & TS_OBJECT_RPTEN_BIT ) == 0){
            invalid_report_id = 12;
          }
          else{
            TOUCH_PRINTK_DEBUG("[%s] REPORTID_TOUCH_KEYARRAY_T15 Message OK !!\n",__func__);
          }
          break;

        case REPORTID_SPI_GPIOPWM_T19:
          if(( ts_data->panel_info.panel_T19register[TS_OBJECT_CTRL_FIELD] & TS_OBJECT_RPTEN_BIT ) == 0){
            invalid_report_id = 13;
          }
          else{
            TOUCH_PRINTK_DEBUG("[%s] REPORTID_SPI_GPIOPWM_T19 Message OK !!\n",__func__);
          }
          break;

        case REPORTID_TOUCH_PROXIMITY_T23:
          invalid_report_id = 15;
          break;

        case REPORTID_SPT_SELFTEST_T25:
          invalid_report_id = 17;
          break;

        case REPORTID_TOUCHSUPPRESSION_T42:
          if(( ts_data->panel_info.panel_T42register[TS_OBJECT_CTRL_FIELD] & TS_OBJECT_RPTEN_BIT ) == 0){
            invalid_report_id = 19;
          }
          else{
            TOUCH_PRINTK_DEBUG("[%s] REPORTID_TOUCHSUPPRESSION_T42 Message OK !!\n",__func__);
          }
          break;

        case REPORTID_NOISESUPPRESSION_T48:
          if(( ts_data->panel_info.panel_T48register[TS_OBJECT_CTRL_FIELD] & TS_OBJECT_RPTEN_BIT ) == 0){
            invalid_report_id = 20;
          }
          else{
            TOUCH_PRINTK_DEBUG("[%s] REPORTID_NOISESUPPRESSION_T48 Message OK !!\n",__func__);
          }
          break;

        default:
          ;
      }

      if( invalid_report_id > 0 ){
          TOUCH_PRINTK_DEBUG("[%s] Reported invalid Message (0x%02x)\n",__func__, ts_data->last_reportid );
          if( touch_debug_message > 0 ){
            touch_kcc_write_log( &buf[0],8,(int)invalid_report_id);
          }
          ts_touch_software_reset();
      }
    }
  }

  if(( ts_data->detect_reset == true )||
     ( ts_data->early_suspend_sleep == TS_DEVICE_CONDITION_SLEEP )){

    TOUCH_PRINTK_DEBUG("[%s] Force Release process(Touch Press count:%d)\n",__func__, ts_data->touch_count);

    if( ts_data->touch_count > 0 ){
      for(count=0; count<MAX_NUM_TOUCHES ; count++){
        if( ts_data->tc[count].state ){
          ts_data->tc[count].state = TS_T9_STATUS_RELEASE;
          TOUCH_PRINTK_DEBUG("Force Release <TrackingID:%d>\n", count);
          ret_code = 0;
        }
      }
    }
    else{
      for(count=0; count<MAX_NUM_TOUCHES ; count++){
        if( ts_data->tc[count].state ){
          TOUCH_PRINTK_DEBUG("Force clear <TrackingID:%d State=0x%02x>\n", count, ts_data->tc[count].state );
          ts_data->tc[count].state = 0;
        }
      }
    }

    if( ts_data->early_suspend_sleep == TS_DEVICE_CONDITION_SLEEP ){
      buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t7].StartPosiotion&0x00ff);
      buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t7].StartPosiotion&0xff00)>>8;
      buf[2]=0;
      buf[3]=0;
      ts_touch_i2c_access(TS_I2C_MODE_WRITE, ts_data->panel_info.client, buf, 4);
      TOUCH_PRINTK_DEBUG("[%s]Device was wakeup,set sleep mode \n", __func__);
    }

    ts_touch_confirm_object();
  }

    return ret_code;
}

static irqreturn_t ts_touch_irq(int irq, void *handle)
{
  ts_data->queue_count++;
  if( ts_data->queue_count == 1 ){
    queue_work(tstouch_work0, &ts_data->work0);
  }
  TOUCH_PRINTK_DEBUG("[%s] queue_count=%d\n", __func__,ts_data->queue_count);

  return IRQ_HANDLED;
}

static void tstouch_work_0(struct work_struct *work)
{
  TOUCH_PRINTK_DEBUG("[S]tstouch_work_0\n");

  ts_data->queue_count=0;

  TP_I2CLOCK();
  if(!ts_touch_read_values(&ts_data->panel_info)){
    ts_touch_send_event();
  }
  TP_I2CUNLOCK();

  TOUCH_PRINTK_DEBUG("[E]tstouch_work_0\n");

}

static int ts_touch_remove(struct i2c_client *client)
{
  free_irq(ts_data->panel_info.irq, ts_data);
#ifdef CONFIG_HAS_EARLYSUSPEND
  unregister_early_suspend(&ts_data->early_suspend);
#endif
  input_unregister_device(ts_data->input);
  kfree(ts_data);

  if (diag_data != NULL) {
    kfree(diag_data);
    diag_data = NULL;
  }
  return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ts_touch_early_suspend(struct early_suspend *h)
{
  u8 buf[4];

  u16 count;
  u16 active_state=0;

  TOUCH_PRINTK_DEBUG("[S]%s Sleep=%d \n", __func__, ts_data->early_suspend_sleep);

  if(0 == ts_data->device_condition){
      ts_data->early_suspend_sleep=TS_DEVICE_CONDITION_SLEEP; 
      TOUCH_PRINTK_DEBUG("[E]%s Sleep=%d \n", __func__, ts_data->early_suspend_sleep);
      return;
  }

  TP_I2CLOCK();
  buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t7].StartPosiotion&0x00ff);
  buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t7].StartPosiotion&0xff00)>>8;
  buf[2]=0;
  buf[3]=0;
  ts_touch_i2c_access(TS_I2C_MODE_WRITE, ts_data->panel_info.client, buf, 4);
  TP_I2CUNLOCK();

  for(count=0; count<MAX_NUM_TOUCHES ; count++){
    if( ts_data->tc[count].state ){
      ts_data->tc[count].state = TS_T9_STATUS_RELEASE;
      TOUCH_PRINTK_DEBUG("Force clear <TrackingID:%d State=0x%02x>\n", count, ts_data->tc[count].state );
      active_state++;
    }
  }
  if( active_state > 0 ){
    ts_touch_send_event();
  }

  ts_data->early_suspend_sleep=TS_DEVICE_CONDITION_SLEEP; 

  TOUCH_PRINTK_DEBUG("[E]%s Sleep=%d \n", __func__, ts_data->early_suspend_sleep);

}

static void ts_touch_early_resume(struct early_suspend *h)
{
  u8  buf[10];
  u8  result;
  int port_read_value;


  TOUCH_PRINTK_DEBUG("[S]%s\n" , __func__ );

  ts_data->early_suspend_sleep=TS_DEVICE_CONDITION_ACTIVE;

  if(0 == ts_data->device_condition){
    TOUCH_PRINTK_DEBUG("[%s] recoverly start \n", __func__ );
    if( ts_touch_init_device(&ts_data->panel_info) == 0){
        TP_I2CLOCK();
        ts_touch_software_reset();
        ts_touch_confirm_object();

        TP_I2CUNLOCK();
        TOUCH_PRINTK_DEBUG("[%s] recoverly sucusess \n", __func__ );
        ts_data->device_condition = 1;
    }
  }

  TP_I2CLOCK();

  ts_touch_software_reset();

  buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t7].StartPosiotion&0x00ff);
  buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t7].StartPosiotion&0xff00)>>8;
  buf[2]=ts_data->panel_info.panel_T7register[0];
  buf[3]=ts_data->panel_info.panel_T7register[1];
  result = ts_touch_i2c_access(TS_I2C_MODE_WRITE,ts_data->panel_info.client, buf, 4);

  buf[0]=(ts_data->panel_info.table[ts_data->panel_info.add_t6].StartPosiotion&0x00ff);
  buf[0]=(buf[0]+2);
  buf[1]=(ts_data->panel_info.table[ts_data->panel_info.add_t6].StartPosiotion&0xff00)>>8;
  buf[2]=TS_T6_CALIBRATE_ORDER;
  result = ts_touch_i2c_access(TS_I2C_MODE_WRITE,ts_data->panel_info.client, buf, 3);
  TOUCH_PRINTK_DEBUG("     Calibration Result(%d): 0x%02x 0x%02x 0x%02x\n", result, buf[0],buf[1],buf[2] );
  if( touch_debug_message > 0 ){
    touch_kcc_write_log( &buf[0],8,4);
  }

  ts_touch_T48_chrgon_ctrl(TS_CHARGER_TURN_ON);
  ts_touch_T42_enable_ctrl(TS_CHARGER_TURN_ON);

  if( ts_data->charge_condition == TS_CHARGER_TURN_OFF ){
    ts_touch_T48_chrgon_ctrl(TS_CHARGER_TURN_OFF);
    ts_touch_T42_enable_ctrl(TS_CHARGER_TURN_OFF);
  }

  ts_data->queue_count=0;
  port_read_value = gpio_get_value_cansleep(PM8058_GPIO_PM_TO_SYS(TS_TOUCH_INT1-1));
  if(port_read_value == 0)
  {
    TOUCH_PRINTK_DEBUG("    %s:gpio=%d\n", __func__ , port_read_value);
    ts_touch_read_values(&ts_data->panel_info);
  }

  TP_I2CUNLOCK();

  TOUCH_PRINTK_DEBUG("[E]%s\n" , __func__ );

}
#endif

static int ts_touch_suspend(struct i2c_client *client, pm_message_t state)
{
  TOUCH_PRINTK_DEBUG("[%s]\n" , __func__ );

  return 0;
}

static int ts_touch_resume(struct i2c_client *client)
{
  TOUCH_PRINTK_DEBUG("[%s]\n" , __func__ );

  return 0;
}

static struct i2c_device_id ts_touch_idtable[] = {
  { "ts_mxt224", 0 },
  { }
};

MODULE_DEVICE_TABLE(i2c, ts_touch_idtable);

static struct i2c_driver ts_touch_driver = {
  .driver = {
    .owner = THIS_MODULE,
    .name  = "ts_mxt224",
  },
  .id_table = ts_touch_idtable,
  .probe    = ts_touch_probe0,
  .remove   = __devexit_p(ts_touch_remove),
  .suspend  = ts_touch_suspend,
  .resume   = ts_touch_resume,
};

static int ts_touch_init_device(struct ts_mxt224_Info *data)
{

  u8  readbuf[128];
  u8  buf[128];
  int cnt=0;
  int max_report_id = 0;
  int ret_code=0;

  memset(buf,0,128);
  memset(readbuf,0,128);

  TP_I2CLOCK();
  buf[0]=0;
  buf[1]=0;

  if( ts_touch_i2c_access(TS_I2C_MODE_WRITE,data->client, buf, 2) == TS_I2C_FAILURE ){
    ret_code=1;
    goto init_error;
  }
  if( ts_touch_i2c_access(TS_I2C_MODE_READ, data->client, readbuf, 7) == TS_I2C_FAILURE ){
    ret_code=1;
    goto init_error;
  }

  data->info.FamilyID  = readbuf[0];
  data->info.ValiantID = readbuf[1];
  data->info.Version   = readbuf[2];
  data->info.Build     = readbuf[3];
  data->info.xSize     = readbuf[4];
  data->info.ySize     = readbuf[5];
  data->info.elements  = readbuf[6];

  TOUCH_PRINTK_DEBUG("[%s]FamilyID=0x%02x/ValiantID=0x%02x/Version =0x%02x/Build=0x%02x\n", 
      __func__, data->info.FamilyID, data->info.ValiantID, data->info.Version, data->info.Build);
  TOUCH_PRINTK_DEBUG("[%s]xSize=%d/ySize=%d/elements=%d\n",
      __func__, data->info.xSize, data->info.ySize, data->info.elements);

  for (cnt = 1; cnt <= data->info.elements; cnt++){
    buf[0]=cnt*6+1;
    buf[1]=0;
    readbuf[0]=0;

    if( ts_touch_i2c_access(TS_I2C_MODE_WRITE,data->client, buf, 2) == TS_I2C_FAILURE ){
      TOUCH_PRINTK_ERROR("[%s] i2c write failed\n", __func__ );
      ret_code=1;
      goto init_error;
    }
    if( ts_touch_i2c_access(TS_I2C_MODE_READ, data->client, readbuf, 6) == TS_I2C_FAILURE ){
      TOUCH_PRINTK_ERROR("[%s] i2c write failed\n", __func__ );
      ret_code=1;
      goto init_error;
    }

    data->table[cnt].Type=readbuf[0];
    data->table[cnt].StartPosiotion=readbuf[1]|(readbuf[2]<<8);
    data->table[cnt].Size=readbuf[3]+1;
    data->table[cnt].Instance=readbuf[4];
    data->table[cnt].ReportID=readbuf[5];

    TOUCH_PRINTK_DEBUG("[%s]StartPosiotion=0x%x/Size=%d/Instance=%d/ReportID=%d/Type=[%d]\n",
        __func__, data->table[cnt].StartPosiotion, data->table[cnt].Size, data->table[cnt].Instance, data->table[cnt].ReportID,data->table[cnt].Type);

    switch(data->table[cnt].Type){
    case 5:
      TOUCH_PRINTK_DEBUG("[%s]MESSAGEPROCESSOR_T5",__func__);
      data->message_processor_address=data->table[cnt].StartPosiotion;
      break;

    case 6:
      data->add_t6=cnt;
      TOUCH_PRINTK_DEBUG("[%s]GEN_COMMANDPROCESSOR_T6[%d]\n",__func__,cnt);
      break;

    case 7:
      data->add_t7=cnt;
      TOUCH_PRINTK_DEBUG("[%s]GEN_POWERCONFIG_T7[%d]\n",__func__,cnt);
      break;

    case 8:
      data->add_t8=cnt;
      TOUCH_PRINTK_DEBUG("[%s]GEN_ACQUIRECONFIGT_8[%d]\n",__func__,cnt);
      break;

    case 9:
      data->add_t9=cnt;
      data->touch_start = max_report_id+1;
      data->touch_end = max_report_id + data->table[cnt].ReportID;
      TOUCH_PRINTK_DEBUG("[%s]TOUCH_MULTITOUCHSCREEN_T9[%d]\n",__func__,cnt);
      break;

    case 15:
      data->add_t15=cnt;
      TOUCH_PRINTK_DEBUG("[%s]TOUCH_KEYARRAY_T15[%d]\n",__func__,cnt);
      break;

    case 18:
      data->add_t18=cnt;
      TOUCH_PRINTK_DEBUG("[%s]SPT_COMMSCONFIG_T18[%d]\n",__func__,cnt);
      break;

    case 19:
      data->add_t19=cnt;
      TOUCH_PRINTK_DEBUG("[%s]SPI_GPIOPWM_T19[%d]\n",__func__,cnt);
      break;

    case 20:
      data->add_t20=cnt;
      TOUCH_PRINTK_DEBUG("[%s]PROCI_GRIPFACESUPPRESSION_T20[%d]\n",__func__,cnt);
      break;

    case 22:
      data->add_t22=cnt;
      TOUCH_PRINTK_DEBUG("[%s]PROCI_NOISESUPPRESSION_T22[%d]\n",__func__,cnt);
      break;

    case 23:
      data->add_t23=cnt;
      TOUCH_PRINTK_DEBUG("[%s]TOUCH_PROXIMITY_T23[%d]\n",__func__,cnt);
      break;

    case 24:
      data->add_t24=cnt;
      TOUCH_PRINTK_DEBUG("[%s]PROCI_ONETOUCHGESTUREPROCESSOR_T24[%d]\n",__func__,cnt);
      break;

    case 25:
      data->add_t25=cnt;
      TOUCH_PRINTK_DEBUG("[%s]SPT_SELFTEST_T25[%d]\n",__func__,cnt);
      break;

    case 27:
      data->add_t27=cnt;
      TOUCH_PRINTK_DEBUG("[%s]PROCI_TWOTOUCHGESTUREPROCESSOR_T27[%d]\n",__func__,cnt);
      break;

    case 28:
      data->add_t28=cnt;
      TOUCH_PRINTK_DEBUG("[%s]SPT_CTECONFIG_T28[%d]\n",__func__,cnt);
      break;

    case 37:
      data->add_t37=cnt;
      TOUCH_PRINTK_DEBUG("[%s]DEBUG T37[%d]\n",__func__,cnt);
      break;

    case 38:
      data->add_t38=cnt;
      TOUCH_PRINTK_DEBUG("[%s]SPT_USERDATA_T38[%d]\n",__func__,cnt);
      break;

    case 40:
      data->add_t40=cnt;
      TOUCH_PRINTK_DEBUG("[%s]PROCI_GRIPSUPPRESSION_T40[%d]\n",__func__,cnt);
      break;

    case 42:
      data->add_t42=cnt;
      TOUCH_PRINTK_DEBUG("[%s]PROCI_TOUCHSUPPRESSION_T42[%d]\n",__func__,cnt);
      break;

    case 46:
      data->add_t46=cnt;
      TOUCH_PRINTK_DEBUG("[%s]STP_CTECONFIG_T46[%d]\n",__func__,cnt);
      break;

    case 47:
      data->add_t47=cnt;
      TOUCH_PRINTK_DEBUG("[%s]PROCI_STYLUS_T47[%d]\n",__func__,cnt);
      break;

    case 48:
      data->add_t48=cnt;
      TOUCH_PRINTK_DEBUG("[%s]PROCG_NOISESUPPRESSION_T48[%d]\n",__func__,cnt);
      break;
    }      

    max_report_id += data->table[cnt].ReportID;

  }
init_error:
  TOUCH_PRINTK_DEBUG("[%s](ret=%d)\n", __func__, ret_code);
  TP_I2CUNLOCK();

  return ret_code;

}

static int ts_touch_probe0(struct i2c_client *client,const struct i2c_device_id *id)
{
  struct input_dev *input_dev;

  int   err=0;
  int   port_read_value;
  u16   loop_cnt;
  int   error=0;

  TOUCH_PRINTK_DEBUG("[S]ts_touch_probe0\n");

  ts_data = kzalloc(sizeof(struct ts_touch), GFP_KERNEL);

  if (!ts_data) {
    TOUCH_PRINTK_ERROR("[%s] init error (ts_data=%p)\n",__func__, ts_data);
    err = -ENOMEM;

    if (diag_data != NULL) {
      kfree(diag_data);
      diag_data = NULL;
    }
    return err;
    
  }

  memset(ts_data,0,sizeof(struct ts_touch));

  diag_data = NULL;
  client->addr = TS_I2C_SLAVE_ADDR;

  TOUCH_PRINTK_DEBUG("[%s]Slave ADRS(0x%x)\n",__func__,client->addr);

  error = gpio_request(client->irq, "touch0");
  if (error < 0) {
    TOUCH_PRINTK_ERROR("failed to request GPIO %d, error %d\n", client->irq, error);
  }

  error = gpio_direction_input(client->irq);
  if (error < 0) {
    TOUCH_PRINTK_ERROR("failed to configure direction for GPIO %d, error %d\n", client->irq, error);
  }

  msleep(50);

  input_dev = input_allocate_device();
  if (!input_dev) {
    TOUCH_PRINTK_ERROR("[%s] init error (input_dev=%p)\n",__func__, input_dev);

    err = input_register_device(input_dev);
    if (err < 0){
      input_free_device(input_dev);
    }
    kfree(ts_data);

    if (diag_data != NULL) {
      kfree(diag_data);
      diag_data = NULL;
    }

    return err;

  }

  ts_data->lock = 0;
  ts_data->panel_info.client = client;
  client->driver = &ts_touch_driver;
  i2c_set_clientdata(client, ts_data);

  ts_data->input = input_dev;

  INIT_WORK(&ts_data->work0, tstouch_work_0);

  for(loop_cnt=0; loop_cnt < 16; loop_cnt++){

    port_read_value = gpio_get_value_cansleep(PM8058_GPIO_PM_TO_SYS(TS_TOUCH_INT1-1));
    
    if(port_read_value == 0)
      break;
    msleep(10); 
  };


  if( ts_touch_init_device(&ts_data->panel_info) != 0)
  {
    TOUCH_PRINTK_ERROR("failed ts_touch_init_device()\n");
    ts_data->device_condition = 0;
  }
  else
  {
    ts_data->device_condition = 1;
  }

  ts_touch_software_reset();

  ts_data->panel_info.irq = gpio_to_irq(client->irq);
  snprintf(ts_data->phys, sizeof(ts_data->phys),
    "%s/input/input0", dev_name(&client->dev));

  dev_info(&client->dev, "%s\n", ts_data->phys);

  input_set_drvdata(ts_data->input, ts_data);
  input_dev->name       = "ts_mxt224";
  input_dev->phys       = ts_data->phys;
  input_dev->id.bustype = BUS_I2C;
  input_dev->id.vendor  = 0x0001;
  input_dev->id.product = 0x0002;
  input_dev->id.version = 0x0100;
  input_dev->dev.parent = &client->dev;

  input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
  input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

  input_dev->absbit[BIT_WORD(ABS_MT_POSITION_X)]  = BIT_MASK(ABS_MT_POSITION_X);
  input_dev->absbit[BIT_WORD(ABS_MT_POSITION_Y)]  = BIT_MASK(ABS_MT_POSITION_Y);
  input_dev->absbit[BIT_WORD(ABS_MT_TOUCH_MAJOR)] = BIT_MASK(ABS_MT_TOUCH_MAJOR);
  input_dev->absbit[BIT_WORD(ABS_MT_WIDTH_MAJOR)] = BIT_MASK(ABS_MT_WIDTH_MAJOR);
  input_dev->absbit[BIT_WORD(ABS_MT_TRACKING_ID)] = BIT_MASK(ABS_MT_TRACKING_ID);
  input_dev->absbit[BIT_WORD(ABS_MT_PRESSURE)] = BIT_MASK(ABS_MT_PRESSURE);

  input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 479, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, 799, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,0, 255, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,0, 255, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,0, MAX_NUM_TOUCHES, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0,  255, 0, 0);

  err = input_register_device(input_dev);


#ifdef CONFIG_HAS_EARLYSUSPEND
  ts_data->early_suspend.suspend = ts_touch_early_suspend;
  ts_data->early_suspend.resume  = ts_touch_early_resume;
  ts_data->early_suspend.level   = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
  register_early_suspend(&ts_data->early_suspend);
#endif

  ts_data->last_reportid = 0;
  ts_data->queue_count = 0;
  ts_data->early_suspend_sleep=TS_DEVICE_CONDITION_ACTIVE;

  ts_select_command_processor_value();

  ts_data->object_field_chk = 0;

  ts_touch_read_values(&ts_data->panel_info);

  err = request_any_context_irq(gpio_to_irq(client->irq), ts_touch_irq, IRQF_TRIGGER_FALLING,"touch0", ts_data);

  port_read_value = gpio_get_value_cansleep(PM8058_GPIO_PM_TO_SYS(TS_TOUCH_INT1-1));
  if(port_read_value == 0)
  {
    TOUCH_PRINTK_DEBUG("ts_touch_probe0 gpio=%d\n", port_read_value);
    ts_touch_read_values(&ts_data->panel_info);
  }

  TOUCH_PRINTK_DEBUG("[E]ts_touch_probe0 Normal end\n");


  return 0;

}

static int __init ts_touch_init(void)
{

  tstouch_work0 = create_singlethread_workqueue("tstouch_work0");

  ts_data = (void*)0;

  return i2c_add_driver(&ts_touch_driver);
}

static void __exit ts_touch_exit(void)
{

  i2c_del_driver(&ts_touch_driver);

  if (tstouch_work0)
    destroy_workqueue(tstouch_work0);

}

int ts_diag_data_start(void)
{

  TOUCH_PRINTK_DEBUG("ts_diag_data_start");
  if (diag_data == NULL) {
      diag_data = kzalloc(sizeof(struct ts_diag_type), GFP_KERNEL);
      TOUCH_PRINTK_DEBUG("allocate memory");
  }
  memset(diag_data,0,sizeof(struct ts_diag_type));

  ts_diag_start_flag = 1;
  return 0;
} 

int ts_diag_data_end(void)
{
  TOUCH_PRINTK_DEBUG("ts_diag_data_end");
  ts_diag_start_flag = 0;

  if (diag_data != NULL) {
      DIAGDATA_LOCK();
      kfree(diag_data);
      diag_data = NULL;
      DIAGDATA_UNLOCK();
      TOUCH_PRINTK_ERROR("free memory");
  }

  return 0;
}

int ts_diag_data_get(void *diag_val)
{
  TOUCH_PRINTK_DEBUG("ts_diag_data_get");
  if (diag_data != NULL) {
      memcpy(diag_val, diag_data, sizeof(struct ts_diag_type));
  }
  else
  {
    TOUCH_PRINTK_ERROR("Error ts diag inactive");
  }

  return 0;
}

void ts_panel_T7_register_set(struct ts_nv_data_table *stNv)
{
  memcpy(ts_data->panel_info.panel_T7register,
         stNv->sz_touch_panel_atmel_t7,
         sizeof(stNv->sz_touch_panel_atmel_t7));
  TOUCH_PRINTK_DEBUG("Get NV Data (T7)\n");
}

void ts_panel_T8_register_set(struct ts_nv_data_table *stNv)
{
  memcpy(ts_data->panel_info.panel_T8register,
         stNv->sz_touch_panel_atmel_t8,
         sizeof(stNv->sz_touch_panel_atmel_t8));
  TOUCH_PRINTK_DEBUG("Get NV Data (T8)\n");
}

void ts_panel_T9_register_set(struct ts_nv_data_table *stNv)
{
  memcpy(ts_data->panel_info.panel_T9register,
         stNv->sz_touch_panel_atmel_t9,
         sizeof(stNv->sz_touch_panel_atmel_t9));
  TOUCH_PRINTK_DEBUG("Get NV Data (T9)\n");
}

void ts_panel_T15_register_set(struct ts_nv_data_table *stNv)
{

  touch_debug_message = stNv->sz_touch_panel_atmel_t15[3];

  TOUCH_PRINTK_DEBUG("[%s] Debug Option [CHRG:%d][MSG:%d]\n",__func__, 
      touch_debug_charge, touch_debug_message );

  memcpy(ts_data->panel_info.panel_T15register,
             stNv->sz_touch_panel_atmel_t15,
             sizeof(stNv->sz_touch_panel_atmel_t15));
  TOUCH_PRINTK_DEBUG("Get NV Data (T15)\n");
}

void ts_panel_T19_register_set(struct ts_nv_data_table *stNv)
{
  memcpy(ts_data->panel_info.panel_T19register,
         stNv->sz_touch_panel_atmel_t19,
         sizeof(stNv->sz_touch_panel_atmel_t19));
  TOUCH_PRINTK_DEBUG("Get NV Data (T19)\n");
}

void ts_panel_T40_register_set(struct ts_nv_data_table *stNv)
{
  memcpy(ts_data->panel_info.panel_T40register,
         stNv->sz_touch_panel_atmel_t40,
         sizeof(stNv->sz_touch_panel_atmel_t40));
  TOUCH_PRINTK_DEBUG("Get NV Data (T40)\n");
}

void ts_panel_T42_register_set(struct ts_nv_data_table *stNv)
{
  memcpy(ts_data->panel_info.panel_T42register,
         stNv->sz_touch_panel_atmel_t42,
         sizeof(stNv->sz_touch_panel_atmel_t42));
  TOUCH_PRINTK_DEBUG("Get NV Data (T42)\n");
}

void ts_panel_T46_register_set(struct ts_nv_data_table *stNv)
{
  memcpy(ts_data->panel_info.panel_T46register,
         stNv->sz_touch_panel_atmel_t46,
         sizeof(stNv->sz_touch_panel_atmel_t46));
  TOUCH_PRINTK_DEBUG("Get NV Data (T46)\n");
}

void ts_panel_T47_register_set(struct ts_nv_data_table *stNv)
{
  memcpy(ts_data->panel_info.panel_T47register,
         stNv->sz_touch_panel_atmel_t47,
         sizeof(stNv->sz_touch_panel_atmel_t47));
  TOUCH_PRINTK_DEBUG("Get NV Data (T47)\n");
}

void ts_panel_T48_register_set(struct ts_nv_data_table *stNv)
{
  memcpy(ts_data->panel_info.panel_T48register,
         stNv->sz_touch_panel_atmel_t48,
         sizeof(stNv->sz_touch_panel_atmel_t48));
  TOUCH_PRINTK_DEBUG("Get NV Data (T48)\n");
}

int ts_nv_data_set(void *val)
{
  struct ts_nv_data_table stNV;

  if(0 == ts_data->device_condition){
    TOUCH_PRINTK_DEBUG("[%s] recoverly start \n", __func__ );
    if( ts_touch_init_device(&ts_data->panel_info) == 0){
      TP_I2CLOCK();
      ts_touch_software_reset();
      TP_I2CUNLOCK();
      TOUCH_PRINTK_DEBUG("[%s] recoverly sucusess \n", __func__ );
      ts_data->device_condition = 1;
    }
  }

  memcpy(&stNV, val, sizeof(struct ts_nv_data_table));

  if( stNV.result == 0 ){
    TOUCH_PRINTK_DEBUG("[%s]ts_nv_data_set write start.[result:0x%02x]\n", __func__ ,stNV.result);

    ts_panel_T7_register_set(&stNV);
    ts_panel_T8_register_set(&stNV);
    ts_panel_T9_register_set(&stNV);
    ts_panel_T15_register_set(&stNV);
    ts_panel_T19_register_set(&stNV);
    ts_panel_T40_register_set(&stNV);
    ts_panel_T42_register_set(&stNV);
    ts_panel_T46_register_set(&stNV);
    ts_panel_T47_register_set(&stNV);
    ts_panel_T48_register_set(&stNV);

    if( ts_data->device_condition ){
        TP_I2CLOCK();

        ts_touch_confirm_object();

        TP_I2CUNLOCK();
    }
    TOUCH_PRINTK_DEBUG("ts_nv_data_set end");
  }
  else{
    TOUCH_PRINTK_ERROR("[%s]fail ts_nv_data_set read result[0x%02x]\n", __func__, stNV.result);
  }

  ts_data->object_field_chk = 1;

  return 0;
}

int ts_nv_board_get(void *mode)
{
  ts_data->model_type = 0x01;
  TOUCH_PRINTK_DEBUG("[%s]board_type:[%d]\n",__func__,ts_data->model_type);
  memcpy(mode, &ts_data->model_type, sizeof(unsigned char));
  return 0;
}

int ts_diag_log_level(unsigned char mode)
{
  ts_log_level = mode;
  TOUCH_PRINTK_DEFAULT("[%s]ts_diag_log_level ts_log_level :[%d]\n",__func__,ts_log_level);

  return 0;
}

int ts_diag_event_ctrl(unsigned char mode)
{
  ts_event_control = mode;
  TOUCH_PRINTK_DEFAULT("[%s] ts_event_control:[%d]\n",__func__,ts_event_control);

  return 0;
}

#ifndef XXXDEBUG_LOGIC_TEST
module_init(ts_touch_init);
module_exit(ts_touch_exit);

MODULE_AUTHOR("KYOCERA");
MODULE_DESCRIPTION("TouchScreen Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ts_mxt224");
#endif

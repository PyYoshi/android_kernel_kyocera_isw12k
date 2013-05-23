/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 */
/*
 *  alps-input.c - Linux kernel modules for interfase of acceleration and magnetic field sensor
 *
 *  Copyright (C) 2010 ALPS
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input-polldev.h>

#include <asm/uaccess.h> 
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/sensors/alps_io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/poll.h>
#include <linux/sensors/ml610q792.h>
#include <linux/earlysuspend.h>

#define EVENT_TYPE_ACCEL_X          ABS_X
#define EVENT_TYPE_ACCEL_Y          ABS_Y
#define EVENT_TYPE_ACCEL_Z          ABS_Z

#define EVENT_TYPE_MAGV_X           ABS_HAT0X
#define EVENT_TYPE_MAGV_Y           ABS_HAT0Y
#define EVENT_TYPE_MAGV_Z           ABS_BRAKE

#define ALPS_POLL_INTERVAL  1000
#define ALPS_INPUT_FUZZ  0
#define ALPS_INPUT_FLAT  0

#define POLL_STOP_TIME 100

#define DEFAULT_DELAY  200

#define EVENT_ACC_DATA_MAX 8192
#define EVENT_MAG_DATA_MAX 4096

#define ACC_DATA_NUM        4
#define ACC_OFFSET_NUM      6
#define MAG_DATA_NUM        3
#define PITCH_ROLL_DATA_NUM 2

#define CALIB_START_MODE    0
#define CALIB_START_RESULT  1

#define CALIB_WAIT          0
#define CALIB_FINISH        1
#define CALIB_FINISH2       2

#define INDEX_X            0
#define INDEX_Y            1
#define INDEX_Z            2
#define INDEX_SUM          3
#define INDEX_PITCH        4
#define INDEX_ROLL         5

#define ACT_UPDATE_FALSE   0
#define ACT_UPDATE_TRUE    1
#define ACC_POWER_ON       1
#define ACC_POWER_OFF      0
#define MAG_POWER_ON       1
#define MAG_POWER_OFF      0

#define MODE_0             0
#define MODE_1             1
#define MODE_2             2
#define MODE_3             3
#define MODE_4             4

#define PEDOMETER_ENA_PARM 0
#define PEDOMETER_ENA_RET  1

#define PEDOMETER_SET_P1   0
#define PEDOMETER_SET_P2   1
#define PEDOMETER_SET_RET  2

#define PEDOMETER_GET_P1   0
#define PEDOMETER_GET_P2   1
#define PEDOMETER_GET_RET  2

#define BYTE_MASK          0xff
#define REGISTER_LENGTH    1
#define I2C_WRITE_SIZE_MIN 2
#define INDEX_REG          0
#define INDEX_VAL          1

#define EVENT_KIND_ACC     1
#define EVENT_KIND_MAG     2

static DEFINE_MUTEX(alps_lock);

static struct platform_device *pdev;
static struct input_polled_dev *alps_idev;

static int32_t flgM = 0, flgA = 0;
static int32_t delay = DEFAULT_DELAY;
static int32_t poll_stop_cnt = 0;
static bool poll_stop_sleep = false;

static bool g_bIsAlreadyExistAccImitationXData       = false;
static bool g_bIsAlreadyExistAccImitationYData       = false;
static bool g_bIsAlreadyExistAccImitationZData       = false;
static int32_t  g_naAccImitationData[ACC_DATA_NUM];

static bool g_bIsAlreadyExistAccImitationPitchData   = false;
static int32_t  g_nAccImitationPitchData;

static bool g_bIsAlreadyExistAccImitationRollData    = false;
static int32_t  g_nAccImitationRollData;

static bool g_bIsAlreadyExistMagImitationXData       = false;
static bool g_bIsAlreadyExistMagImitationYData       = false;
static bool g_bIsAlreadyExistMagImitationZData       = false;
static int32_t  g_naMagImitationData[MAG_DATA_NUM];

static bool g_bIsAlreadyExistMagImitationOffsetXData = false;
static bool g_bIsAlreadyExistMagImitationOffsetYData = false;
static bool g_bIsAlreadyExistMagImitationOffsetZData = false;
static int32_t  g_naMagImitationOffsetData[MAG_DATA_NUM];
static int32_t  g_naMagSaveOffsetData[MAG_DATA_NUM];
static int32_t  g_nAccCalibrationMode = -1;

static bool g_bIsMagDmExecuting = false;

static int32_t g_nDMAccPower = 0;


extern int32_t hscd_i2c_readm(u8 *rxData, int32_t length);
extern int32_t hscd_i2c_writem(u8 *txData, int32_t length);
extern void hscd_activate(int32_t flgatm, int32_t flg, int32_t dtime);
extern int32_t hscd_get_magnetic_field_data(int32_t *xyz);
extern int32_t hscd_get_magnetic_field_offset_data(int32_t *xyz);
extern int32_t hscd_set_magnetic_field_offset_Xdata(int32_t arg_nOffsetData);
extern int32_t hscd_set_magnetic_field_offset_Ydata(int32_t arg_nOffsetData);
extern int32_t hscd_set_magnetic_field_offset_Zdata(int32_t arg_nOffsetData);
extern int32_t hscd_set_temp_correct(void);
extern int32_t hscd_resume (struct i2c_client *client);
extern int32_t hscd_suspend(struct i2c_client *client,pm_message_t mesg);
static void alps_early_suspend(struct early_suspend *h);
static void alps_late_resume(struct early_suspend *h);

extern void registerCallbackFromAcc(int (*func)(int type, int pause_timer));

static int32_t accsns_err_check(void)
{
  int ret_val = 0;

  if (accsns_spi_error_check() != false) {
    accsns_initialize();
    ret_val = -ECOMM;
  } else {
    ret_val = -EFAULT;
  }
  
  return ret_val;
}

static int32_t accsns_dm_write
(
 IoCtlAccDataDmSetCmd* arg_stpDmWriteCommand
 )
{
  int32_t nRet = 0;
  
  switch (arg_stpDmWriteCommand->m_nCommand) {
  case ACC_DM_W_X_CMND:
    g_bIsAlreadyExistAccImitationXData = true;
    g_naAccImitationData[INDEX_X] = arg_stpDmWriteCommand->m_nData;
    break;
  case ACC_DM_W_Y_CMND:
    g_bIsAlreadyExistAccImitationYData = true;
    g_naAccImitationData[INDEX_Y] = arg_stpDmWriteCommand->m_nData;
    break;
  case ACC_DM_W_Z_CMND:
    g_bIsAlreadyExistAccImitationZData = true;
    g_naAccImitationData[INDEX_Z] = arg_stpDmWriteCommand->m_nData;
    break;
  case ACC_DM_W_PITCH_CMND:
    g_bIsAlreadyExistAccImitationPitchData = true;
    g_nAccImitationPitchData = arg_stpDmWriteCommand->m_nData;
    break;
  case ACC_DM_W_ROLL_CMND:
    g_bIsAlreadyExistAccImitationRollData = true;
    g_nAccImitationRollData = arg_stpDmWriteCommand->m_nData;
    break;
  case ACC_DM_W_REL_X_CMND:
    g_bIsAlreadyExistAccImitationXData = false;
    break;
  case ACC_DM_W_REL_Y_CMND:
    g_bIsAlreadyExistAccImitationYData = false;
    break;
  case ACC_DM_W_REL_Z_CMND:
    g_bIsAlreadyExistAccImitationZData = false;
    break;
  case ACC_DM_W_REL_PITCH_CMND:
    g_bIsAlreadyExistAccImitationPitchData = false;
    break;
  case ACC_DM_W_REL_ROLL_CMND:
    g_bIsAlreadyExistAccImitationRollData = false;
    break;
  default:
    printk("DM Command Error!! m_nCommand = 0x%02x\n",
           arg_stpDmWriteCommand->m_nCommand);
    nRet = -1;
    break;
  }
#ifdef ALPS_DEBUG
  printk("Imitation x:(%d)%03x y:(%d)%03x z:(%d)%03x p:(%d)%03x r:(%d)%03x\n",
         g_bIsAlreadyExistAccImitationXData, g_naAccImitationData[INDEX_X],
         g_bIsAlreadyExistAccImitationYData, g_naAccImitationData[INDEX_Y],
         g_bIsAlreadyExistAccImitationZData, g_naAccImitationData[INDEX_Z],
         g_bIsAlreadyExistAccImitationPitchData, g_nAccImitationPitchData,
         g_bIsAlreadyExistAccImitationRollData, g_nAccImitationRollData);
#endif
  return nRet;
}

static int32_t accsns_dm_read
(
 IoCtlAccDataDmGetCmd* arg_stpDmReadCommand
 )
{
  int32_t nRet = -1;
  
  switch (arg_stpDmReadCommand->m_nCommand) {
  case ACC_DM_R_RAW_CMND:
    if(flgA++ == 0) {
      accsns_activate(ACT_UPDATE_FALSE, ACC_POWER_ON);
      mutex_unlock(&alps_lock);
      msleep(delay + delay);
      mutex_lock(&alps_lock);
    }

    nRet = accsns_get_acceleration_data(arg_stpDmReadCommand->m_naData);
#ifdef ALPS_DEBUG
    if(nRet) {
      printk("accsns_get_acceleratio_data() return error\n");
    }
#endif

    if(--flgA == 0) {
      accsns_activate(ACT_UPDATE_FALSE, ACC_POWER_OFF);
    }
    break;
    
  case ACC_DM_R_RAW_PEDOM_CMND:
    nRet = accsns_get_pedometer_data(arg_stpDmReadCommand->m_naData);
    break;
    
  case ACC_DM_R_RAW_VEHICLE_CMND:
    nRet = accsns_get_vehicle_data(arg_stpDmReadCommand->m_naData);
    break;

  case ACC_DM_R_RAW_MOVE_CMND:
    nRet = accsns_get_move_data(arg_stpDmReadCommand->m_naData);
    break;
    
  case ACC_DM_R_CALIB_MODE_CMND:
  {
    int32_t iRet = 0;
    iRet = accsns_calibration_mode();
    arg_stpDmReadCommand->m_naData[0] = iRet;
    nRet = 0;
    break;
  }
  case ACC_DM_R_CALIB_STRT_CMND:
  {
    int32_t iRet = 0;

    iRet = accsns_calibration_is_wait();

    switch(arg_stpDmReadCommand->m_naData[CALIB_START_MODE]) {
      case MODE_0:
      case MODE_1:
      case MODE_2:
      case MODE_3:
      case MODE_4:
        break;
        
      default:
        iRet = 0;
        break;
    }

    if (iRet == 0) {
      nRet = 1;
      arg_stpDmReadCommand->m_naData[CALIB_START_RESULT] = 0xff;
    } else {
      if(g_nDMAccPower++ == 0) {
        if(flgA++ == 0) {
          accsns_activate(ACT_UPDATE_FALSE, ACC_POWER_ON);
        }
      }
      iRet = accsns_calibration_start(arg_stpDmReadCommand->m_naData[CALIB_START_MODE]);
      arg_stpDmReadCommand->m_naData[CALIB_START_RESULT] = iRet;
      g_nAccCalibrationMode = arg_stpDmReadCommand->m_naData[CALIB_START_MODE];
      if (iRet != 0) {
        if (--g_nDMAccPower == 0) {
          if (--flgA == 0) {
            accsns_activate(ACT_UPDATE_FALSE,ACC_POWER_ON);
          } else {
            if (flgA < 0) {
              printk("ACC Power state error %d\n", flgA);
              flgA = 0;
            }
          }
        } else {
          if (g_nDMAccPower < 0) {
            printk("g_nDMAccPower state error %d\n", g_nDMAccPower);
            g_nDMAccPower = 0;
          }
        }
      }
      nRet = 0;
    }
    printk("ACC POWER1:%d\n", flgA);
    break;
  }
  case ACC_DM_R_CALIB_STTS_CMND:
  {
    int32_t naData[ACC_OFFSET_NUM];
    int32_t iRet = 0;

    iRet = accsns_calibration_is_comp(naData);
    if (iRet == 0) {
      arg_stpDmReadCommand->m_naData[0] = CALIB_WAIT;
    } else {
      int32_t nIdxRes  = 0;
      int32_t nIdxRead = 0;
      if(g_nAccCalibrationMode != MODE_4){
        arg_stpDmReadCommand->m_naData[nIdxRes++] = CALIB_FINISH;
      }else{
        arg_stpDmReadCommand->m_naData[nIdxRes++] = CALIB_FINISH2;
      }
      
      arg_stpDmReadCommand->m_naData[nIdxRes++] = naData[nIdxRead++];
      arg_stpDmReadCommand->m_naData[nIdxRes++] = naData[nIdxRead++];
      arg_stpDmReadCommand->m_naData[nIdxRes++] = naData[nIdxRead++];
      arg_stpDmReadCommand->m_naData[nIdxRes++] = naData[nIdxRead++];
      arg_stpDmReadCommand->m_naData[nIdxRes++] = naData[nIdxRead++];
      arg_stpDmReadCommand->m_naData[nIdxRes] = naData[nIdxRead];

      switch(g_nAccCalibrationMode) {
      case MODE_0:
      case MODE_1:
      case MODE_2:
      case MODE_3:
      case MODE_4:
        if (--g_nDMAccPower == 0) {
          if (--flgA == 0) {
            accsns_activate(ACT_UPDATE_FALSE,ACC_POWER_OFF);
          } else {
            if (flgA < 0) {
              printk("ACC Power state error %d\n", flgA);
              flgA = 0;
            }
          }
        } else {
          if (g_nDMAccPower < 0) {
            printk("g_nDMAccPower state error %d\n", g_nDMAccPower);
            g_nDMAccPower = 0;
          }
        }
        break;
      default:
        printk("AccCalib: calibration mode err %d\n", g_nAccCalibrationMode);
        break;        
      }
    }
    nRet = 0;
    printk("ACC POWER2:%d\n", flgA);
    break;
  }

  case ACC_DM_R_INIT_SEQ_CMND:
  case ACC_DM_R_PEDOMETER_INFO_CLEAR_CMND:
  case ACC_DM_R_VEHICLE_INFO_CLEAR_CMND:
  case ACC_DM_R_CORRECT_VALUE_SET_CMND:
    arg_stpDmReadCommand->m_naData[0] = accsns_dm_seqctrl(arg_stpDmReadCommand->m_nCommand);
    nRet = 0;
    break;
  
  case ACC_DM_R_POLL_ON_CMND:
  case ACC_DM_R_PEDOMETER_ON_CMND:
  case ACC_DM_R_PEDOMETER_AND_POLL_ON_CMND:
  case ACC_DM_R_VEHICLE_ON_CMND:
  case ACC_DM_R_VEHICLE_AND_POLL_ON_CMND:
  case ACC_DM_R_MOVE_ON_CMND:
  case ACC_DM_R_PEDOM_TIMER_ON_CMND:
    arg_stpDmReadCommand->m_naData[0] = accsns_dm_seqctrl(arg_stpDmReadCommand->m_nCommand);
    if(arg_stpDmReadCommand->m_naData[0] == 0){
        flgA++;
    }
    nRet = 0;
    break;

  case ACC_DM_R_POLL_OFF_CMND:
  case ACC_DM_R_PEDOMETER_OFF_CMND:
  case ACC_DM_R_VEHICLE_OFF_CMND:
  case ACC_DM_R_MOVE_OFF_CMND:
  case ACC_DM_R_PEDOM_TIMER_OFF_CMND:
    if (--flgA < 0) {
       flgA = 0;
    }
    arg_stpDmReadCommand->m_naData[0] = accsns_dm_seqctrl(arg_stpDmReadCommand->m_nCommand);
    nRet = 0;
    break;

  default:
#ifdef ALPS_DEBUG
    printk("command errror!! iCommand = 0x%x\n",arg_stpDmReadCommand->m_nCommand);
#endif
    break;
  }

#ifdef ALPS_DEBUG
  printk("Imitation x:(%d)%03x y:(%d)%03x z:(%d)%03x p:(%d)%03x r:(%d)%03x\n",
         g_bIsAlreadyExistAccImitationXData, g_naAccImitationData[INDEX_X],
         g_bIsAlreadyExistAccImitationYData, g_naAccImitationData[INDEX_Y],
         g_bIsAlreadyExistAccImitationZData, g_naAccImitationData[INDEX_Z],
         g_bIsAlreadyExistAccImitationPitchData, g_nAccImitationPitchData,
         g_bIsAlreadyExistAccImitationRollData, g_nAccImitationRollData);
#endif
  return nRet;
}

static int32_t hscd_dm_write
(
 IoCtlMagDataDmSetCmd* arg_stpDmWriteCommand
 )
{
  int32_t nRet = 0;
  int32_t iaOffsetData[MAG_DATA_NUM];
  
  switch (arg_stpDmWriteCommand->m_nCommand) {
  case MAG_DM_W_X_CMND:
    g_bIsAlreadyExistMagImitationXData = true;
    g_naMagImitationData[INDEX_X] = arg_stpDmWriteCommand->m_nData;
    break;
  case MAG_DM_W_Y_CMND:
    g_bIsAlreadyExistMagImitationYData = true;
    g_naMagImitationData[INDEX_Y] = arg_stpDmWriteCommand->m_nData;
    break;
  case MAG_DM_W_Z_CMND:
    g_bIsAlreadyExistMagImitationZData = true;
    g_naMagImitationData[INDEX_Z] = arg_stpDmWriteCommand->m_nData;
    break;
  case MAG_DM_W_X_OFST_CMND:
    if(g_bIsAlreadyExistMagImitationOffsetXData == false) {
      g_bIsAlreadyExistMagImitationOffsetXData = true;
      hscd_get_magnetic_field_offset_data(&iaOffsetData[0]);
      g_naMagSaveOffsetData[INDEX_X] = iaOffsetData[INDEX_X];
    }
    g_naMagImitationOffsetData[INDEX_X] = arg_stpDmWriteCommand->m_nData;
    hscd_set_magnetic_field_offset_Xdata(g_naMagImitationOffsetData[INDEX_X]);
    break;
  case MAG_DM_W_Y_OFST_CMND:
    if(g_bIsAlreadyExistMagImitationOffsetYData == false) {
      g_bIsAlreadyExistMagImitationOffsetYData = true;
      hscd_get_magnetic_field_offset_data(&iaOffsetData[0]);
      g_naMagSaveOffsetData[INDEX_Y] = iaOffsetData[INDEX_Y];
    }
    g_naMagImitationOffsetData[INDEX_Y] = arg_stpDmWriteCommand->m_nData;
    hscd_set_magnetic_field_offset_Ydata(g_naMagImitationOffsetData[INDEX_Y]);
    break;
  case MAG_DM_W_Z_OFST_CMND:
    if(g_bIsAlreadyExistMagImitationOffsetZData == false) {
      g_bIsAlreadyExistMagImitationOffsetZData = true;
      hscd_get_magnetic_field_offset_data(&iaOffsetData[0]);
      g_naMagSaveOffsetData[INDEX_Z] = iaOffsetData[INDEX_Z];
    }
    g_naMagImitationOffsetData[INDEX_Z] = arg_stpDmWriteCommand->m_nData;
    hscd_set_magnetic_field_offset_Zdata(g_naMagImitationOffsetData[INDEX_Z]);
    break;
  case MAG_DM_W_REL_X_CMND:
    g_bIsAlreadyExistMagImitationXData = false;
    break;
  case MAG_DM_W_REL_Y_CMND:
    g_bIsAlreadyExistMagImitationYData = false;
    break;
  case MAG_DM_W_REL_Z_CMND:
    g_bIsAlreadyExistMagImitationZData = false;
    break;
  case MAG_DM_W_REL_OFST_X_CMND:
    if(g_bIsAlreadyExistMagImitationOffsetXData == true) {
      hscd_set_magnetic_field_offset_Xdata(g_naMagSaveOffsetData[INDEX_X]);
    }
    g_bIsAlreadyExistMagImitationOffsetXData = false;
    break;
  case MAG_DM_W_REL_OFST_Y_CMND:
    if(g_bIsAlreadyExistMagImitationOffsetYData == true) {
      hscd_set_magnetic_field_offset_Ydata(g_naMagSaveOffsetData[INDEX_Y]);
    }
    g_bIsAlreadyExistMagImitationOffsetYData = false;
    break;
  case MAG_DM_W_REL_OFST_Z_CMND:
    if(g_bIsAlreadyExistMagImitationOffsetZData == true) {
      hscd_set_magnetic_field_offset_Zdata(g_naMagSaveOffsetData[INDEX_Z]);
    }
    g_bIsAlreadyExistMagImitationOffsetZData = false;
    break;
  case MAG_DM_W_EXECUTING_STATUS:
    g_bIsMagDmExecuting = (bool)arg_stpDmWriteCommand->m_nData;
    break;

  case MAG_DM_DISABLE_TEMP_CORRECT_CMND:
    nRet = hscd_set_temp_correct();
    break;

  default:
    nRet = -1;
    break;
  }
#ifdef ALPS_DEBUG
  printk("Imitation x:(%d)%03x y:(%d)%03x z:(%d)%03x ox:(%d)%03x oy:(%d)%03x oz:(%d)%03x\n",
         g_bIsAlreadyExistMagImitationXData,
         g_naMagImitationData[INDEX_X],
         g_bIsAlreadyExistMagImitationYData,
         g_naMagImitationData[INDEX_Y],
         g_bIsAlreadyExistMagImitationZData,
         g_naMagImitationData[INDEX_Z],
         g_bIsAlreadyExistMagImitationOffsetXData,
         g_naMagImitationOffsetData[INDEX_X],
         g_bIsAlreadyExistMagImitationOffsetYData,
         g_naMagImitationOffsetData[INDEX_Y],
         g_bIsAlreadyExistMagImitationOffsetZData,
         g_naMagImitationOffsetData[INDEX_Z]);
#endif
  return nRet;
}

static int32_t hscd_dm_read
(
 IoCtlMagDataDmGetCmd* arg_stpDmReadCommand
 )
{
  int32_t nRet = -1;
  
  switch (arg_stpDmReadCommand->m_nCommand) {
  case MAG_DM_R_RAW_CMND:
    if(flgM++ == 0) {
      hscd_activate(ACT_UPDATE_FALSE, MAG_POWER_ON, delay);
      mutex_unlock(&alps_lock);
      msleep(delay + delay);
      mutex_lock(&alps_lock);
    }
    nRet = hscd_get_magnetic_field_data(arg_stpDmReadCommand->m_naData);
    if(--flgM == 0) {
      hscd_activate(ACT_UPDATE_FALSE, MAG_POWER_OFF, delay);
    }
    break;
    
  case MAG_DM_R_RAW_OFST_CMND:
    nRet = hscd_get_magnetic_field_offset_data(arg_stpDmReadCommand->m_naData);
    break;

  case MAG_DM_R_EXECUTING_STATUS:
    arg_stpDmReadCommand->m_naData[0] = (int32_t)g_bIsMagDmExecuting;
    nRet = 0;
    break;
  case MAG_DM_R_RAW_AZIMUTH_CMND:
  case MAG_DM_R_RAW_INCRINATION_CMND:
  default:
    break;
  }
#ifdef ALPS_DEBUG
  printk("Imitation x:(%d)%03x y:(%d)%03x z:(%d)%03x ox:(%d)%03x oy:(%d)%03x oz:(%d)%03x\n",
         g_bIsAlreadyExistMagImitationXData,
         g_naMagImitationData[INDEX_X],
         g_bIsAlreadyExistMagImitationYData,
         g_naMagImitationData[INDEX_Y],
         g_bIsAlreadyExistMagImitationZData,
         g_naMagImitationData[INDEX_Z],
         g_bIsAlreadyExistMagImitationOffsetXData,
         g_naMagImitationOffsetData[INDEX_X],
         g_bIsAlreadyExistMagImitationOffsetYData,
         g_naMagImitationOffsetData[INDEX_Y],
         g_bIsAlreadyExistMagImitationOffsetZData,
         g_naMagImitationOffsetData[INDEX_Z]);
#endif
  return nRet;
}

static long alps_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  void __user *argp = (void __user *)arg;
  int32_t ret = -1, tmpval = 0;
  
  switch (cmd) {
  case ALPSIO_SET_MAGACTIVATE:
    ret = copy_from_user(&tmpval, argp, sizeof(tmpval));
    if (ret) {
      printk("error : alps_ioctl(cmd = ALPSIO_SET_MAGACTIVATE)\n" );
      return -EFAULT;
    }
#ifdef ALPS_DEBUG
    printk("alps_ioctl(cmd = ALPSIO_SET_MAGACTIVATE), flgM = %d\n", tmpval);
#endif
    mutex_lock(&alps_lock);
    flgM += (tmpval != 0)? 1: -1;
    tmpval = (flgM == 0)? MAG_POWER_OFF : MAG_POWER_ON;
    hscd_activate(ACT_UPDATE_TRUE, tmpval, delay);
    mutex_unlock(&alps_lock);
    break;
    
  case ALPSIO_SET_ACCACTIVATE:
    ret = copy_from_user(&tmpval, argp, sizeof(tmpval));
    if (ret) {
      printk("error : alps_ioctl(cmd = ALPSIO_SET_ACCACTIVATE)\n");
      return -EFAULT;
    }
#ifdef ALPS_DEBUG
    printk("alps_ioctl(cmd = ALPSIO_SET_ACCACTIVATE), flgA = %d\n", tmpval);
#endif
    mutex_lock(&alps_lock);
    flgA += (tmpval != 0)? 1: -1;
    tmpval = (flgA == 0)? ACC_POWER_OFF : ACC_POWER_ON;
    ret = accsns_activate(ACT_UPDATE_TRUE, tmpval);
    mutex_unlock(&alps_lock);

    if (ret) {
      printk("error : alps_ioctl(cmd = ALPSIO_SET_ACCACTIVATE)\n" );
      return accsns_err_check();
    }
    break;
    
  case ALPSIO_SET_DELAY:
    ret = copy_from_user(&tmpval, argp, sizeof(tmpval));
    if (ret) {
      printk( "error : alps_ioctl(cmd = ALPSIO_SET_DELAY)\n" );
      return -EFAULT;
    }
#ifdef ALPS_DEBUG
    printk("alps_ioctl(cmd = ALPSIO_SET_DELAY)\n");
#endif
    if      (tmpval <=  10) tmpval =  10;
    else if (tmpval <=  20) tmpval =  20;
    else if (tmpval <=  60) tmpval =  50;
    else if (tmpval <= 100) tmpval = 100;
    else if (tmpval <= 200) tmpval = 200;
    else                    tmpval =1000;
    mutex_lock(&alps_lock);
    delay = tmpval;
    poll_stop_cnt = POLL_STOP_TIME / tmpval;
    hscd_activate(1, flgM, delay);
    accsns_set_delay(delay);
    mutex_unlock(&alps_lock);
#ifdef ALPS_DEBUG
    printk("     delay = %d\n", delay);
#endif
    break;
    
  case ALPSIO_MAG_I2C_READ:
    {
      IoctlMagI2cRead stMagI2cRead;
      ret = copy_from_user(&stMagI2cRead, argp, sizeof(stMagI2cRead));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_MAG_I2C_READ)\n" );
        return -EFAULT;
      }
      stMagI2cRead.m_cReadData = (char)(stMagI2cRead.m_nAdrs & BYTE_MASK);
      
      mutex_lock(&alps_lock);
      ret = hscd_i2c_readm(&stMagI2cRead.m_cReadData,REGISTER_LENGTH);
      mutex_unlock(&alps_lock);
      
      if(ret) {
        printk( "error(hscd_i2c_readm) : alps_ioctl(cmd = ALPSIO_MAG_I2C_READ)\n" );
        return -EFAULT;
      }
      ret = copy_to_user(argp,&stMagI2cRead,  sizeof(stMagI2cRead));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_MAG_I2C_READ)\n" );
        return -EFAULT;
      }
    }
    break;
    
  case ALPSIO_MAG_I2C_WRITE:
    {
      IoctlMagI2cWrite stMagI2cWrite = {0};
      char caWriteBuffer[I2C_WRITE_SIZE_MIN] = {0};
      
      ret = copy_from_user(&stMagI2cWrite, argp, sizeof(stMagI2cWrite));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_MAG_I2C_WRITE)\n" );
        return -EFAULT;
      }
      caWriteBuffer[INDEX_REG] = (char)(stMagI2cWrite.m_nAdrs & BYTE_MASK);
      caWriteBuffer[INDEX_VAL] = stMagI2cWrite.m_cWriteData;
      
      mutex_lock(&alps_lock);
      ret = hscd_i2c_writem(caWriteBuffer,I2C_WRITE_SIZE_MIN);
      mutex_unlock(&alps_lock);
      
      if(ret) {
        printk( "error(hscd_i2c_writem) : alps_ioctl(cmd = ALPSIO_MAG_I2C_WRITE)\n" );
        return -EFAULT;
      }
    }
    break;
    
  case ALPSIO_MAG_GET:
    {
      IoctlMagDataGet stMagData;
      memset(&stMagData, 0x00, sizeof(stMagData));
      
      mutex_lock(&alps_lock);
      if(flgM++ == 0) {
        hscd_activate(ACT_UPDATE_FALSE, MAG_POWER_ON, delay);
        mutex_unlock(&alps_lock);
        msleep(delay + delay);
        mutex_lock(&alps_lock);
      }
      ret = hscd_get_magnetic_field_data(&stMagData.m_naData[0]);
      if(--flgM == 0) {
        hscd_activate(ACT_UPDATE_FALSE, MAG_POWER_OFF, delay);
      }
#ifdef ALPS_DEBUG
      if(!ret)
        {
          printk("Mag_I2C, x:%d, y:%d, z:%d\n",stMagData.m_naData[INDEX_X], stMagData.m_naData[INDEX_Y], stMagData.m_naData[INDEX_Z]);
        }
#endif
      mutex_unlock(&alps_lock);
      
      if (ret) {
        printk( "error(hscd_get_magnetic_field_data) : alps_ioctl(cmd = ALPSIO_MAG_GET)\n" );
        return -EFAULT;
      }
      ret = copy_to_user(argp,&stMagData,  sizeof(stMagData));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_MAG_GET)\n" );
        return -EFAULT;
      }
    }
    break;
    
  case ALPSIO_MAG_DM_READ:
    {
      IoCtlMagDataDmGetCmd stDmReadCommand = {0};
      
      ret = copy_from_user(&stDmReadCommand, argp, sizeof(stDmReadCommand));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_MAG_DM_READ)\n" );
        return -EFAULT;
      }
#ifdef ALPS_DEBUG
      printk("[MAG]stDmReadCommand.m_nCommand=0x%x\n",stDmReadCommand.m_nCommand);
      printk("[MAG]stDmReadCommand.m_naData[0]=0x%x\n",stDmReadCommand.m_naData[0]);
#endif
      mutex_lock(&alps_lock);
      ret = hscd_dm_read(&stDmReadCommand);
      mutex_unlock(&alps_lock);
      
      if(ret) {
        printk( "error(hscd_dm_read) : alps_ioctl(cmd = ALPSIO_MAG_DM_READ)\n" );
        return -EFAULT;
      }
      ret = copy_to_user(argp,&stDmReadCommand,  sizeof(stDmReadCommand));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_MAG_DM_READ)\n" );
        return -EFAULT;
      }
    }
    break;
    
  case ALPSIO_MAG_DM_WRITE:
    {
      IoCtlMagDataDmSetCmd stDmWriteCommand = {0};
      
      ret = copy_from_user(&stDmWriteCommand, argp, sizeof(stDmWriteCommand));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_MAG_DM_WRITE)\n" );
        return -EFAULT;
      }
#ifdef ALPS_DEBUG
      printk("[MAG]stDmWriteCommand.m_nCommand=0x%x\n",stDmWriteCommand.m_nCommand);
      printk("[MAG]stDmWriteCommand.m_nData=0x%x\n",stDmWriteCommand.m_nData);
#endif
      mutex_lock(&alps_lock);
      ret = hscd_dm_write(&stDmWriteCommand);
      mutex_unlock(&alps_lock);
      
      if(ret) {
        printk( "error(hscd_dm_write) : alps_ioctl(cmd = ALPSIO_MAG_DM_WRITE)\n" );
        return -EFAULT;
      }
    }
    break;

  case ALPSIO_ACC_SPI_READ:
    {
      IoctlAccSPIRead stAccSPIRead;
        
      ret = copy_from_user(&stAccSPIRead, argp, sizeof(stAccSPIRead));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_SPI_READ)\n" );
        return -EFAULT;
      }
        
      mutex_lock(&alps_lock);
      ret = accsns_spi_read((char)(stAccSPIRead.m_nAdrs & BYTE_MASK), &stAccSPIRead.m_cReadData, REGISTER_LENGTH);
      mutex_unlock(&alps_lock);
      if(ret) {
        printk( "error(accsns_spi_read) : alps_ioctl(cmd = ALPSIO_ACC_SPI_READ)\n" );
        return -EFAULT;
      }
      
      ret = copy_to_user(argp, &stAccSPIRead, sizeof(stAccSPIRead));
      if (ret) {
        printk( "error(copy_to_user) : alps_ioctl(cmd = ALPSIO_ACC_SPI_READ)\n" );
        return -EFAULT;
      }
    }
    break;

  case ALPSIO_ACC_SPI_WRITE:
    {
      IoctlAccSPIWrite stAccSPIWrite;
      
      ret = copy_from_user(&stAccSPIWrite, argp, sizeof(stAccSPIWrite));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_SPI_WRITE)\n" );
        return -EFAULT;
      }
      
      mutex_lock(&alps_lock);
      ret = accsns_spi_write((char)(stAccSPIWrite.m_nAdrs & BYTE_MASK), &stAccSPIWrite.m_cWriteData, REGISTER_LENGTH);
      mutex_unlock(&alps_lock);
      if(ret) {
        printk( "error(accsns_spi_write) : alps_ioctl(cmd = ALPSIO_ACC_SPI_WRITE)\n" );
        return -EFAULT;
      }
    }
    break;

  case ALPSIO_ACC_GET:
    {
      IoctlAccDataGet stAccData;
      memset(&stAccData, 0x00, sizeof(stAccData));
      
      mutex_lock(&alps_lock);
      
      if(flgA++ == 0) {
        accsns_activate(ACT_UPDATE_FALSE, ACC_POWER_ON);
      }
      ret = accsns_get_acceleration_data(&stAccData.m_naData[0]);
#ifdef ALPS_DEBUG
      if(!ret)
        {
          printk("Acc_I2C, x:%d, y:%d, z:%d\n", 
                 *(stAccData.m_naData), *(stAccData.m_naData + INDEX_Y), *(stAccData.m_naData + INDEX_Z));
        }
#endif      
      if(--flgA == 0) {
        accsns_activate(ACT_UPDATE_FALSE, ACC_POWER_OFF);
      }
      mutex_unlock(&alps_lock);
      
      if (ret) {
        printk( "error(accns_get_acceleration_data) : alps_ioctl(cmd = ALPSIO_ACC_GET)\n" );
        return accsns_err_check();
      }
      ret = copy_to_user(argp,&stAccData,  sizeof(stAccData));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_GET)\n" );
        return -EFAULT;
      }
    }
    break;
  case ALPSIO_ACC_DM_READ:
    {
      IoCtlAccDataDmGetCmd stDmReadCommand = {0};
      
      ret = copy_from_user(&stDmReadCommand, argp, sizeof(stDmReadCommand));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_DM_READ)\n" );
        return -EFAULT;
      }
#ifdef ALPS_DEBUG
      printk("[ACC]stDmReadCommand.m_nCommand=0x%x\n",stDmReadCommand.m_nCommand);
      printk("[ACC]stDmReadCommand.m_naData[0]=0x%x\n",stDmReadCommand.m_naData[0]);
#endif
      mutex_lock(&alps_lock);
      ret = accsns_dm_read(&stDmReadCommand);
      mutex_unlock(&alps_lock);
      
      if(ret) {
        printk( "error(accsns_dm_read) : alps_ioctl(cmd = ALPSIO_ACC_DM_READ)\n" );
        return accsns_err_check();
      }
      ret = copy_to_user(argp,&stDmReadCommand,  sizeof(stDmReadCommand));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_DM_READ)\n" );
        return -EFAULT;
      }
    }
    break;
  case ALPSIO_ACC_DM_WRITE:
    {
      IoCtlAccDataDmSetCmd stDmWriteCommand = {0};
      
      ret = copy_from_user(&stDmWriteCommand, argp, sizeof(stDmWriteCommand));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_DM_WRITE)\n" );
        return -EFAULT;
      }
#ifdef ALPS_DEBUG
      printk("[ACC]stDmWriteCommand.m_nCommand=0x%x\n",stDmWriteCommand.m_nCommand);
      printk("[ACC]stDmWriteCommand.m_nData=0x%x\n",stDmWriteCommand.m_nData);
#endif
      mutex_lock(&alps_lock);
      ret = accsns_dm_write(&stDmWriteCommand);
      mutex_unlock(&alps_lock);
      
      if(ret) {
        printk( "error(accsns_dm_write) : alps_ioctl(cmd = ALPSIO_ACC_DM_WRITE)\n" );
        return -EFAULT;
      }
    }
    break;
  case ALPSIO_ACC_SET_FREQ:
    {
      int32_t rcv = 0;
      uint8_t freq = 0;

      ret = copy_from_user(&rcv,argp,sizeof(rcv));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_SET_FREQ)\n" );
        return -EFAULT;
      }
      
      if (rcv > BYTE_MASK) {
        printk( "range over frequency %d\n", rcv);
        return -EFAULT;
      } else {
        freq = (uint8_t)(rcv & BYTE_MASK);
        accsns_set_freq(freq);
      }
    }
    break;
  case ALPSIO_ACC_SET_OFFSET:
    {
      IoCtlAccSetOffset ioc;
      memset(&ioc , 0x00, sizeof(ioc));
      ret = copy_from_user(&ioc,argp,sizeof(ioc));
      
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_SET_OFFSET)\n" );
        return -EFAULT;
      }
      accsns_set_offset(ioc.m_naOffset);
    }
    break;

  case ALPSIO_ACC_SET_NVPARAM:
    {
      IoCtlAccSetAccsnsNVParams ioc;
      
      ret = copy_from_user(&ioc, argp, sizeof(ioc));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_SET_NVPARAM)\n" );
        return -EFAULT;
      }

      accsns_set_nv_params(&ioc);
    }
    break;

  case ALPSIO_ACC_SET_PEDOM_PARAM:
    {
      IoCtlAccSetPedomParam ioc;
      int32_t SensorActive = 0;
      
      ret = copy_from_user(&ioc,argp,sizeof(ioc));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_SET_PEDOM_PARAM)\n" );
        return -EFAULT;
      }

      ret = accsns_pedom_set_info(ioc.m_nWeight, ioc.m_nStepWide, ioc.m_iDelay);
      if (ret == 0) {
        SensorActive = accsns_get_current_active();
        if (SensorActive & ACCSNS_ACTIVE_PEDOM){
          ret = accsns_pedom_start_timer(1);
        }
      }
      
      if (ret) {
        return accsns_err_check();
      }
    }
    break;
    
  case ALPSIO_ACC_FW_UPDATE_SEQ:
    {
      IoCtlAccSetFirmwareData ioc;
      uint8_t *fw_data = NULL;
      
      ret = copy_from_user(&ioc,argp,sizeof(ioc));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_FW_UPDATE_SEQ)\n" );
        return -EFAULT;
      }
      
      if((ioc.m_cData == NULL) || (ioc.m_nLen == 0)){
        printk( "error(Input param) : alps_ioctl(cmd = ALPSIO_ACC_FW_UPDATE_SEQ)\n" );
        return -EFAULT;
      }
      
      if((ioc.m_nLen % 8) != 0){
        printk( "error(Firmdata size error) : alps_ioctl(cmd = ALPSIO_ACC_FW_UPDATE_SEQ)\n" );
        return -EFAULT;
      }
      
      fw_data = (uint8_t *)kmalloc( ioc.m_nLen, GFP_KERNEL );
      if(fw_data == NULL)
      {
        printk( "error(kmalloc) : alps_ioctl(cmd = ALPSIO_ACC_FW_UPDATE_SEQ)\n" );
        return -ENOMEM;
      }
      
      ret = copy_from_user( fw_data, ioc.m_cData, ioc.m_nLen );
      if( ret != 0 )
      {
        printk( "error(copy_from_user(data)) : alps_ioctl(cmd = ALPSIO_ACC_FW_UPDATE_SEQ)\n" );
        kfree( fw_data );
        return -EFAULT;
      }
      
      ret = accsns_update_fw_seq(fw_data, ioc.m_nLen);
      kfree( fw_data );
        
      if(ret != 0){
          printk( "error(accsns_update_fw_seq) : alps_ioctl(cmd = ALPSIO_ACC_FW_UPDATE_SEQ\n");
          return -EFAULT;
      }
    }
    break;
    
  case ALPSIO_ACC_FW_UPDATE:
    {
      IoCtlAccSetFirmwareData ioc;
      uint8_t *fw_data = NULL;
      
      ret = copy_from_user(&ioc,argp,sizeof(ioc));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_FW_UPDATE)\n" );
        return -EFAULT;
      }
      
      if((ioc.m_cData == NULL) || (ioc.m_nLen == 0)){
        printk( "error(Input param) : alps_ioctl(cmd = ALPSIO_ACC_FW_UPDATE)\n" );
        return -EFAULT;
      }
      
      if((ioc.m_nLen % 8) != 0){
        printk( "error(Firmdata size error) : alps_ioctl(cmd = ALPSIO_ACC_FW_UPDATE)\n" );
        return -EFAULT;
      }
      
      fw_data = (uint8_t *)kmalloc( ioc.m_nLen, GFP_KERNEL );
      if(fw_data == NULL)
      {
        printk( "error(kmalloc) : alps_ioctl(cmd = ALPSIO_ACC_FW_UPDATE)\n" );
        return -ENOMEM;
      }
      
      ret = copy_from_user( fw_data, ioc.m_cData, ioc.m_nLen );
      if( ret != 0 )
      {
        printk( "error(copy_from_user(data)) : alps_ioctl(cmd = ALPSIO_ACC_FW_UPDATE)\n" );
        kfree( fw_data );
        return -EFAULT;
      }
      
      if(ioc.m_bSync == false){
        ret = accsns_update_fw(false, fw_data, ioc.m_nLen);
        kfree( fw_data );
        
      }else{
        ret = accsns_update_fw_async(fw_data, ioc.m_nLen);
        if(ret != 0){
          kfree( fw_data );
        }
      }
      
      if (ret) {
        printk( "error(accsns_update_fw) : alps_ioctl(cmd = ALPSIO_ACC_FW_UPDATE[%d]\n", ioc.m_bSync);
        return -EFAULT;
      }
    }
    break;

  case ALPSIO_ACC_FW_GET_VERSION:
    {
      IoCtlAccSetFirmwareVersion ioc;
      
      ret = accsns_get_fw_version(ioc.m_cData);
      
      if(ret) {
        printk( "error(accsns_dm_read) : alps_ioctl(cmd = ALPSIO_ACC_FW_GET_VERSION)\n" );
        return -EFAULT;
      }
      ret = copy_to_user(argp, &ioc, sizeof(IoCtlAccSetFirmwareVersion));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_FW_GET_VERSION)\n" );
        return -EFAULT;
      }
    }
    break;

    case ALPSIO_ACC_FW_GET_ASYNC_STATUS:
    {
      ret = accsns_update_fw_async_chk();
      if (ret) {
        printk( "error(accsns_update_fw_async_chk) : alps_ioctl(cmd = ALPSIO_ACC_FW_GET_ASYNC_STATUS) ret=%d\n", ret );
        return ret;
      }
    }
    break;

  case ALPSIO_ACC_SUS_RES:
  {
    int32_t nArg = 0;
    ret = copy_from_user(&nArg,argp,sizeof(nArg));
    if (ret) {
      printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_SET_OFFSET)\n" );
      return -EFAULT;
    }
    
    if (nArg == 0) {
      pm_message_t mesg = {0};
      mesg.event = 0;
      accsns_suspend(NULL,mesg);
    } else if (nArg == 1) {
      accsns_resume(NULL);
    } else {
    }
    break;
  }
  case ALPSIO_MAG_SUS_RES:
  {
    int32_t nArg = 0;
    ret = copy_from_user(&nArg,argp,sizeof(nArg));
    if (ret) {
      printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_SET_OFFSET)\n" );
      return -EFAULT;
    }
    
    if (nArg == 0) {
      pm_message_t mesg = {0};
      mesg.event = 0;
      hscd_suspend(NULL,mesg);
    } else if (nArg == 1) {
      hscd_resume(NULL);
    } else {
    }
    break;
  }

  case ALPSIO_ACC_SET_EXT_ACTIVATE:
  {
    IoCtlAccSetExtActivate  act_param;

    memset(&act_param , 0x00, sizeof(act_param));
    ret = copy_from_user(&act_param,argp,sizeof(act_param));
    if (ret) {
      printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_SET_EXT_ACTIVATE)\n" );
      return -EFAULT;
    }
    if((act_param.m_iType & SENSOR_EXT_FUNC_PEDOMETER) == SENSOR_EXT_FUNC_PEDOMETER)
    {
      if(act_param.m_iEnable == true)
      {
        ret = accsns_activate_pedom(ACT_UPDATE_TRUE, ACC_POWER_ON);
        if (!ret)
        {
          ret = accsns_pedom_start_timer(1);
        }
      }
      else
      {
        ret = accsns_pedom_start_timer(0);
        if (!ret)
        {
          ret = accsns_activate_pedom(ACT_UPDATE_TRUE, ACC_POWER_OFF);
        }
      }
    }
    if((act_param.m_iType & SENSOR_EXT_FUNC_VEHICLEMETER) == SENSOR_EXT_FUNC_VEHICLEMETER)
    {
      if(act_param.m_iEnable == true)
      {
        ret = accsns_activate_vehicle(ACT_UPDATE_TRUE, ACC_POWER_ON);
      }
      else
      {
        ret = accsns_activate_vehicle(ACT_UPDATE_TRUE, ACC_POWER_OFF);
      }
    }
    if (ret) {
      printk( "error : alps_ioctl(cmd = ALPSIO_ACC_SET_EXT_ACTIVATE)\n" );
      return accsns_err_check();
    }
    break;
  }
  case ALPSIO_ACC_GET_PEDOM_INFO:
  {
    int32_t param[8];
    IoCtlAccGetPedomInfo stParam;
    ret = accsns_get_pedometer_data(param);
    if(ret) {
      printk( "error(accsns_dm_read) : alps_ioctl(cmd = ALPSIO_ACC_GET_PEDOM_INFO)\n" );
      return accsns_err_check();
    }
    stParam.m_iStep       = param[0];
    stParam.m_iTime       = param[1];
    stParam.m_iCalorie    = param[2];
    stParam.m_iFat        = param[3];
    stParam.m_iSpeedmeter = param[4];
    stParam.m_iMets       = param[5];
    stParam.m_iCycle_cal  = param[6];
    stParam.m_iCycle_time = param[7];
    ret = copy_to_user(argp, &stParam, sizeof(IoCtlAccGetPedomInfo));
    if (ret) {
      printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_GET_PEDOM_INFO)\n" );
      return -EFAULT;
    }
    break;
  }
  case ALPSIO_ACC_GET_VEHICLE_INFO:
  {
    int32_t param[2];
    IoCtlAccGetVehicleInfo stParam;
    ret = accsns_get_vehicle_data(param);
    if(ret) {
      printk( "error(accsns_dm_read) : alps_ioctl(cmd = ALPSIO_ACC_GET_VEHICLE_INFO)\n" );
      return accsns_err_check();
    }
    stParam.m_iStatus = param[0];
    stParam.m_iKind   = param[1];
    ret = copy_to_user(argp, &stParam, sizeof(IoCtlAccGetVehicleInfo));
    if (ret) {
      printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_GET_VEHICLE_INFO)\n" );
      return -EFAULT;
    }
    break;
  }
  case ALPSIO_ACC_SET_PEDOM_CLEAR:
  {
    ret = accsns_pedom_clear();
    if (ret)
    {
      printk( "error(accsns_pedom_clear) : alps_ioctl(cmd = ALPSIO_ACC_SET_PEDOM_CLEAR)\n" );
      return accsns_err_check();
    }
    break;
  }
  case ALPSIO_ACC_SET_VEHICLE_CLEAR:
  {
    ret = accsns_vehicle_clear();
    if (ret)
    {
      printk( "error(accsns_vehicle_clear) : alps_ioctl(cmd = ALPSIO_ACC_SET_VEHICLE_CLEAR)\n" );
      return accsns_err_check();
    }
    break;
  }
        
  case ALPSIO_ACC_RECOVERY_INIT:
  {
    IoCtlAccRecoveryInfo info;
    
    ret = copy_from_user(&info,argp,sizeof(IoCtlAccRecoveryInfo));
    if (ret) {
      printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_RECOVERY_INIT)\n" );
      return -EFAULT;
    }
    
    ret = accsns_recovery_proc(info.kind, info.backup_data);
    if (ret)
    {
      printk( "error(accsns_recovery_proc) : alps_ioctl(cmd = ALPSIO_ACC_RECOVERY_INIT)\n" );
      return accsns_err_check();
    }
    
    if(info.kind == 0x00){
      ret = copy_to_user(argp, &info, sizeof(IoCtlAccRecoveryInfo));
      if (ret) {
        printk( "error(copy_to_user) : alps_ioctl(cmd = ALPSIO_ACC_RECOVERY_INIT)\n" );
      }
    }
    
    ret |= accsns_initialize();
    if(ret) {
      printk( "error(accsns_initialize) : alps_ioctl(cmd = ALPSIO_ACC_RECOVERY_INIT)\n" );
      return -ENODEV;
    }

    ret = accsns_check_accsensor();
    if(ret) {
      printk( "error(accsns_check_accsensor) : alps_ioctl(cmd = ALPSIO_ACC_RECOVERY_INIT)\n" );
    }
    
    break;
  }
        
  case ALPSIO_ACC_SET_DEBUG_LEVEL:
  {
    accsns_debug_level_chg((int32_t)argp);
    break;
  }

  default:
    return -ENOTTY;
  }
  return 0;
}

static int32_t 
alps_io_open( struct inode* inode, struct file* filp )
{
  return 0;
}

static int32_t 
alps_io_release( struct inode* inode, struct file* filp )
{
  return 0;
}

static unsigned int alps_io_poll_pedom(struct file *fp, poll_table *wait)
{
    unsigned int wake_func = 0;
    unsigned int ret = 0;

    wake_func = accsns_io_poll_pedom(fp, wait);

    if ((wake_func & ACCSNS_ACTIVE_PEDOM_ERROR) == ACCSNS_ACTIVE_PEDOM_ERROR) {
        printk( "error(accsns_io_poll_pedom) : alps_io_poll_pedom \n" );
        accsns_err_check();
        ret = POLLERR;
    } else if ((wake_func & ACCSNS_ACTIVE_PEDOM) == ACCSNS_ACTIVE_PEDOM) {
        ret = POLLIN | POLLPRI;
    }
    
    return ret;
}

static unsigned int alps_io_poll_vehicle(struct file *fp, poll_table *wait)
{
    unsigned int wake_func = 0;
    unsigned int ret = 0;

    wake_func = accsns_io_poll_vehicle(fp, wait);

    if ((wake_func & ACCSNS_ACTIVE_VEHICLE_ERROR) == ACCSNS_ACTIVE_VEHICLE_ERROR) {
        printk( "error(accsns_io_poll_vehicle) : alps_io_poll_vehicle \n" );
        accsns_err_check();
        ret = POLLERR;
    } else if ((wake_func & ACCSNS_ACTIVE_VEHICLE) == ACCSNS_ACTIVE_VEHICLE) {
        ret = POLLIN | POLLPRI;
    }
    
    return ret;
}

static ssize_t accsns_position_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
  int32_t x = 0,y = 0,z = 0;
  int32_t xyz[INDEX_SUM] = {0};
  
  if(accsns_get_acceleration_data(xyz) == 0) {
    x = xyz[INDEX_X];
    y = xyz[INDEX_Y];
    z = xyz[INDEX_Z];
  } else {
    x = 0;
    y = 0;
    z = 0;
  }
  return snprintf(buf, PAGE_SIZE, "(%d %d %d)\n",x,y,z);
}

static ssize_t hscd_position_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
  int32_t x = 0,y = 0,z = 0;
  int32_t xyz[INDEX_SUM] = {0};
  
  if(hscd_get_magnetic_field_data(xyz) == 0) {
    x = xyz[INDEX_X];
    y = xyz[INDEX_Y];
    z = xyz[INDEX_Z];
  } else {
    x = 0;
    y = 0;
    z = 0;
  }
  return snprintf(buf, PAGE_SIZE, "(%d %d %d)\n",x,y,z);
}

static ssize_t alps_position_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
  size_t cnt = 0;
  mutex_lock(&alps_lock);
  cnt += accsns_position_show(dev,attr,buf);
  cnt += hscd_position_show(dev,attr,buf);
  mutex_unlock(&alps_lock);
  return cnt;
}
static DEVICE_ATTR(position, 0444, alps_position_show, NULL);

static struct file_operations alps_fops = {
  .owner   = THIS_MODULE,
  .open    = alps_io_open,
  .release = alps_io_release,
  .unlocked_ioctl = alps_ioctl,
#ifdef CONFIG_COMPAT
  .compat_ioctl   = alps_ioctl,
#endif
};

static struct file_operations alps_fops_pedom = {
  .owner   = THIS_MODULE,
  .open    = alps_io_open,
  .release = alps_io_release,
  .poll    = alps_io_poll_pedom,
};

static struct file_operations alps_fops_vehicle = {
  .owner   = THIS_MODULE,
  .open    = alps_io_open,
  .release = alps_io_release,
  .poll    = alps_io_poll_vehicle,
};

static struct miscdevice alps_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "alps_io",
  .fops  = &alps_fops,
};

static struct miscdevice alps_device_pedom = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "alps_io_pedom",
  .fops  = &alps_fops_pedom,
};
static struct miscdevice alps_device_vehicle = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "alps_io_vehicle",
  .fops  = &alps_fops_vehicle,
};

static struct attribute *alps_attributes[] = {
  &dev_attr_position.attr,
  NULL,
};

static struct attribute_group alps_attribute_group = {
  .attrs = alps_attributes,
};

static struct early_suspend alps_early_suspend_handler = {
  .level	= EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
  .suspend	= alps_early_suspend,
  .resume 	= alps_late_resume,
}; 

static int32_t alps_probe(struct platform_device *dev)
{
  printk(KERN_INFO "alps: alps_probe\n");
  return 0;
}

static int32_t alps_remove(struct platform_device *dev)
{
  printk(KERN_INFO "alps: alps_remove\n");
  return 0;
}

static void alps_early_suspend(struct early_suspend *h)
{
  printk("alps: alps_early_suspend\n");
  mutex_lock(&alps_lock);
  poll_stop_sleep = true;
  mutex_unlock(&alps_lock);
}

static void alps_late_resume(struct early_suspend *h)
{
  printk("alps: alps_late_resume\n");
  mutex_lock(&alps_lock);
  poll_stop_sleep = false;
  mutex_unlock(&alps_lock);
}

static void accsns_poll(struct input_dev *idev)
{
  int32_t xyz[ACC_DATA_NUM] = {0};
  int32_t SensorActive = 0;

  SensorActive = accsns_get_current_active();

  if(SensorActive & ACCSNS_ACTIVE_ACC){
    if(accsns_get_acceleration_data(xyz) == 0) {
      if(g_bIsAlreadyExistAccImitationXData) {
        xyz[INDEX_X] = g_naAccImitationData[INDEX_X];
      }
      if(g_bIsAlreadyExistAccImitationYData) {
        xyz[INDEX_Y] = g_naAccImitationData[INDEX_Y];
      }
      if(g_bIsAlreadyExistAccImitationZData) {
        xyz[INDEX_Z] = g_naAccImitationData[INDEX_Z];
      }
      input_report_abs(idev, EVENT_TYPE_ACCEL_X, xyz[INDEX_X]);
      input_report_abs(idev, EVENT_TYPE_ACCEL_Y, xyz[INDEX_Y]);
      input_report_abs(idev, EVENT_TYPE_ACCEL_Z, xyz[INDEX_Z]);
      idev->sync = 0;
      input_event(idev, EV_SYN, SYN_REPORT, EVENT_KIND_ACC);
    }
    else
    {
      printk( "error(accsns_get_acceleration_data) : accsns_poll \n" );
      accsns_err_check();
    }
  }
}

static void hscd_poll(struct input_dev *idev)
{
  int32_t xyz[MAG_DATA_NUM] = {0};
  
  if(hscd_get_magnetic_field_data(xyz) == 0) {

    if(g_bIsAlreadyExistMagImitationXData) {
      xyz[INDEX_X] = g_naMagImitationData[INDEX_X];
    }
    if(g_bIsAlreadyExistMagImitationYData) {
      xyz[INDEX_Y] = g_naMagImitationData[INDEX_Y];
    }
    if(g_bIsAlreadyExistMagImitationZData) {
      xyz[INDEX_Z] = g_naMagImitationData[INDEX_Z];
    }
    input_report_abs(idev, EVENT_TYPE_MAGV_X, xyz[INDEX_X]);
    input_report_abs(idev, EVENT_TYPE_MAGV_Y, xyz[INDEX_Y]);
    input_report_abs(idev, EVENT_TYPE_MAGV_Z, xyz[INDEX_Z]);
    idev->sync = 0;
    input_event(idev, EV_SYN, SYN_REPORT, EVENT_KIND_MAG);
  }
}

static void alps_poll(struct input_polled_dev *dev)
{
  struct input_dev *idev = dev->input;
  
  mutex_lock(&alps_lock);
  dev->poll_interval = delay;
  if (poll_stop_cnt-- < 0 && !poll_stop_sleep) {
    poll_stop_cnt = -1;
    if (flgM) hscd_poll(idev);
    if (flgA) accsns_poll(idev);
  }
  mutex_unlock(&alps_lock);
}

int accsns_ddi_set_pause(int type, int pause_timer)
{
    int ret = 0;

    accsns_ddi_set_pause_proc(type, pause_timer);

    return ret;
}


static struct platform_driver alps_driver = {
  .driver = {
    .name = "alps-input",
    .owner = THIS_MODULE,
  },
  .probe = alps_probe,
  .remove = alps_remove,
};

static int32_t __init alps_init(void)
{
  struct input_dev *idev = NULL;
  int32_t ret = 0;
  
  ret = platform_driver_register(&alps_driver);
  if (ret)
    goto out_region;
  printk(KERN_INFO "alps-init: platform_driver_register\n");
  
  pdev = platform_device_register_simple("alps", -1, NULL, 0);
  if (IS_ERR(pdev)) {
    ret = PTR_ERR(pdev);
    goto out_driver;
  }
  printk(KERN_INFO "alps-init: platform_device_register_simple\n");
  
  ret = sysfs_create_group(&pdev->dev.kobj, &alps_attribute_group);
  if (ret)
    goto out_device;
  printk(KERN_INFO "alps-init: sysfs_create_group\n");
  
  alps_idev = input_allocate_polled_device();
  if (!alps_idev) {
    ret = -ENOMEM;
    goto out_group;
  }
  printk(KERN_INFO "alps-init: input_allocate_polled_device\n");
  
  alps_idev->poll = alps_poll;
  alps_idev->poll_interval = ALPS_POLL_INTERVAL;
  
  idev = alps_idev->input;
  idev->name = "alps";
  idev->phys = "alps/input0";
  idev->id.bustype = BUS_HOST;
  idev->dev.parent = &pdev->dev;
  idev->evbit[0] = BIT_MASK(EV_ABS);
  
  input_set_abs_params(idev, EVENT_TYPE_ACCEL_X,
                       (-1 * EVENT_ACC_DATA_MAX), EVENT_ACC_DATA_MAX, 
                       ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
  input_set_abs_params(idev, EVENT_TYPE_ACCEL_Y,
                       (-1 * EVENT_ACC_DATA_MAX), EVENT_ACC_DATA_MAX, 
                       ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
  input_set_abs_params(idev, EVENT_TYPE_ACCEL_Z,
                       (-1 * EVENT_ACC_DATA_MAX), EVENT_ACC_DATA_MAX, 
                       ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
  
  input_set_abs_params(idev, EVENT_TYPE_MAGV_X,
                       (-1 * EVENT_MAG_DATA_MAX), EVENT_MAG_DATA_MAX, 
                       ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
  input_set_abs_params(idev, EVENT_TYPE_MAGV_Y,
                       (-1 * EVENT_MAG_DATA_MAX), EVENT_MAG_DATA_MAX,
                        ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
  input_set_abs_params(idev, EVENT_TYPE_MAGV_Z,
                       (-1 * EVENT_MAG_DATA_MAX), EVENT_MAG_DATA_MAX,
                       ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
  
  ret = input_register_polled_device(alps_idev);
  if (ret)
    goto out_idev;
  printk(KERN_INFO "alps-init: input_register_polled_device\n");
  
  ret = misc_register(&alps_device_pedom);
  if (ret) {
    printk("alps-init: alps_io_device(pedom) register failed\n");
    goto exit_misc_device_register_failed;
  }

  ret = misc_register(&alps_device_vehicle);
  if (ret) {
    printk("alps-init: alps_io_device(vehicle) register failed\n");
    goto exit_misc_device_register_failed;
  }

  ret = misc_register(&alps_device);
  if (ret) {
    printk("alps-init: alps_io_device register failed\n");
    goto exit_misc_device_register_failed;
  }
  register_early_suspend(&alps_early_suspend_handler);

  registerCallbackFromAcc(accsns_ddi_set_pause);
  
  return 0;
  
 exit_misc_device_register_failed:
 out_idev:
  input_free_polled_device(alps_idev);
  printk(KERN_INFO "alps-init: input_free_polled_device\n");
 out_group:
  sysfs_remove_group(&pdev->dev.kobj, &alps_attribute_group);
  printk(KERN_INFO "alps-init: sysfs_remove_group\n");
 out_device:
  platform_device_unregister(pdev);
  printk(KERN_INFO "alps-init: platform_device_unregister\n");
 out_driver:
  platform_driver_unregister(&alps_driver);
  printk(KERN_INFO "alps-init: platform_driver_unregister\n");
 out_region:
  return ret;
}

static void __exit alps_exit(void)
{
  unregister_early_suspend(&alps_early_suspend_handler);
  misc_deregister(&alps_device);
  printk(KERN_INFO "alps-exit: misc_deregister\n");
  input_unregister_polled_device(alps_idev);
  printk(KERN_INFO "alps-exit: input_unregister_polled_device\n");
  input_free_polled_device(alps_idev);
  printk(KERN_INFO "alps-exit: input_free_polled_device\n");
  sysfs_remove_group(&pdev->dev.kobj, &alps_attribute_group);
  printk(KERN_INFO "alps-exit: sysfs_remove_group\n");
  platform_device_unregister(pdev);
  printk(KERN_INFO "alps-exit: platform_device_unregister\n");
  platform_driver_unregister(&alps_driver);
  printk(KERN_INFO "alps-exit: platform_driver_unregister\n");
}



module_init(alps_init);
module_exit(alps_exit);

MODULE_DESCRIPTION("Alps Input Device");
MODULE_AUTHOR("ALPS");
MODULE_LICENSE("GPL v2");

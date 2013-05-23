/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 */
/*
 *  alps_io.h - Linux kernel modules for interfase of acceleration and magnetic field sensor
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


#ifndef ___ALPS_IO_H_INCLUDED
#define ___ALPS_IO_H_INCLUDED

#include <linux/ioctl.h>

#define ALPSIO   0xAF

#define ALPSIO_SET_MAGACTIVATE   _IOW(ALPSIO, 0, int)
#define ALPSIO_SET_ACCACTIVATE   _IOW(ALPSIO, 1, int)
#define ALPSIO_SET_DELAY         _IOW(ALPSIO, 2, int)

#define ALPSIO_MAG_I2C_READ      _IOR(ALPSIO, 3, int)
typedef struct {
  int32_t  m_nAdrs;
  char m_cReadData;
}IoctlMagI2cRead;

#define ALPSIO_MAG_I2C_WRITE     _IOW(ALPSIO, 4, int)
typedef struct {
  int32_t  m_nAdrs;
  char m_cWriteData;
}IoctlMagI2cWrite;

#define ALPSIO_MAG_GET           _IOR(ALPSIO, 5, int)
typedef struct {
  int32_t  m_naData[3];
}IoctlMagDataGet;

#define ALPSIO_MAG_DM_READ       _IOR(ALPSIO, 6, int)

enum {
    MAG_DM_R_RAW_CMND              = 0x00,
    MAG_DM_R_RAW_OFST_CMND         ,
    MAG_DM_R_RAW_AZIMUTH_CMND      ,
    MAG_DM_R_RAW_INCRINATION_CMND  ,
    MAG_DM_R_EXECUTING_STATUS      = 0xF0,
    MAG_DM_R_EOE
};

typedef struct {
  int32_t m_nCommand;
  int32_t m_naData[3];
}IoCtlMagDataDmGetCmd;

#define ALPSIO_MAG_DM_WRITE      _IOW(ALPSIO, 7, int)

#define MAG_DM_W_X_CMND                0x10
#define MAG_DM_W_Y_CMND                0x11
#define MAG_DM_W_Z_CMND                0x12
#define MAG_DM_W_X_OFST_CMND           0x13
#define MAG_DM_W_Y_OFST_CMND           0x14
#define MAG_DM_W_Z_OFST_CMND           0x15

#define MAG_DM_W_REL_X_CMND            0x20
#define MAG_DM_W_REL_Y_CMND            0x21
#define MAG_DM_W_REL_Z_CMND            0x22
#define MAG_DM_W_REL_OFST_X_CMND       0x23
#define MAG_DM_W_REL_OFST_Y_CMND       0x24
#define MAG_DM_W_REL_OFST_Z_CMND       0x25
#define MAG_DM_W_EXECUTING_STATUS      0xf0

#define MAG_DM_DISABLE_TEMP_CORRECT_CMND        0x19
typedef struct {
  int32_t m_nCommand;
  int32_t m_nData;
}IoCtlMagDataDmSetCmd;

#define ALPSIO_ACC_SPI_READ      _IOR(ALPSIO, 8, int)
typedef struct {
  int32_t  m_nAdrs;
  char m_cReadData;
}IoctlAccSPIRead;

#define ALPSIO_ACC_SPI_WRITE     _IOW(ALPSIO, 9, int)
typedef struct {
  int32_t  m_nAdrs;
  char m_cWriteData;
}IoctlAccSPIWrite;

#define ALPSIO_ACC_GET           _IOR(ALPSIO,10, int)
typedef struct {
  int32_t  m_naData[4];
}IoctlAccDataGet;

#define ALPSIO_ACC_DM_READ       _IOR(ALPSIO,11, int)

enum {
    ACC_DM_R_RAW_CMND                    = 0x00,
    ACC_DM_R_RAW_PEDOM_CMND,
    ACC_DM_R_RAW_VEHICLE_CMND,
    ACC_DM_R_RAW_MOVE_CMND,
    ACC_DM_R_CALIB_MODE_CMND             = 0x10,
    ACC_DM_R_CALIB_STRT_CMND             ,
    ACC_DM_R_CALIB_STTS_CMND             ,
    ACC_DM_R_INIT_SEQ_CMND               = 0x20,
    ACC_DM_R_POLL_ON_CMND                ,
    ACC_DM_R_POLL_OFF_CMND               ,
    ACC_DM_R_PEDOMETER_ON_CMND           ,
    ACC_DM_R_PEDOMETER_OFF_CMND          ,
    ACC_DM_R_PEDOMETER_AND_POLL_ON_CMND  ,
    ACC_DM_R_PEDOMETER_INFO_CLEAR_CMND   ,
    ACC_DM_R_VEHICLE_ON_CMND             ,
    ACC_DM_R_VEHICLE_OFF_CMND            ,
    ACC_DM_R_VEHICLE_AND_POLL_ON_CMND    ,
    ACC_DM_R_VEHICLE_INFO_CLEAR_CMND     ,
    ACC_DM_R_CORRECT_VALUE_SET_CMND      ,
    ACC_DM_R_MOVE_ON_CMND                = 0x40,
    ACC_DM_R_MOVE_OFF_CMND               ,
    ACC_DM_R_PEDOM_TIMER_ON_CMND         ,
    ACC_DM_R_PEDOM_TIMER_OFF_CMND        ,
    
    ACC_DM_R_EOE
};

typedef struct {
  int32_t m_nCommand;
  int32_t m_naData[8];
}IoCtlAccDataDmGetCmd;

#define ALPSIO_ACC_DM_WRITE      _IOW(ALPSIO,12, int)

#define ALPSIO_ACC_SET_FREQ      _IOW(ALPSIO,13, int)

#define ALPSIO_ACC_SET_OFFSET  _IOW(ALPSIO,14, int)
typedef struct {
  int32_t m_naOffset[6];
}IoCtlAccSetOffset;

#define ALPSIO_ACC_SUS_RES  _IOW(ALPSIO,15, int)
#define ALPSIO_MAG_SUS_RES  _IOW(ALPSIO,16, int)

#define ALPSIO_INVALID           _IOW(ALPSIO,17, int) 

#define ALPSIO_ACC_SET_PEDOM_PARAM  _IOW(ALPSIO,18, int)
typedef struct {
  uint32_t m_nStepWide;
  uint32_t m_nWeight;
  uint32_t m_iDelay;
}IoCtlAccSetPedomParam;

#define ALPSIO_ACC_FW_UPDATE        _IOW(ALPSIO,19, int)
typedef struct {
  uint8_t *m_cData;
  uint32_t m_nLen;
  bool     m_bSync;
}IoCtlAccSetFirmwareData;

#define ALPSIO_ACC_FW_GET_VERSION   _IOW(ALPSIO,20, int)
typedef struct {
  uint8_t m_cData[4];
}IoCtlAccSetFirmwareVersion;

#define ALPSIO_ACC_SET_NVPARAM   _IOW(ALPSIO,21, int)
typedef struct tACC_Set_Pedo_Params_NV {
    uint8_t param_on;
    uint8_t on;
    uint8_t stepwide;
    uint8_t weight;
    uint8_t notify;
    uint8_t speed_ave_time;
    uint8_t mets_stop_time;
    uint8_t bodyfat_on;
    uint8_t bodyfat_cal;
} ACC_SET_PEDO_PARAMS_NV;
    
typedef struct tACC_Set_Dist_Stop_Params_NV {
    uint8_t  stop_notify_on;
    uint16_t stop_notify_time;
} ACC_SET_DIST_STOP_PARAMS_NV;
    
typedef struct tACC_Set_Walk_Run_Params_NV {
    uint8_t  walk_judge_on;
    uint8_t  consecutive_num;
    uint16_t speed_th;
} ACC_SET_WALK_RUN_PARAMS_NV;

typedef struct tACC_Set_Trans_Params_NV {
    uint8_t  on;
    uint8_t  judge_step;
    uint16_t judge_time;
    uint8_t  calc_time;
    uint8_t  consecutive_num;
} ACC_SET_TRANS_PARAMS_NV;
    
typedef struct tACC_Set_Trans_Byc_Params_NV {
    uint8_t consecutive_num;
    uint8_t calory_mets;
} ACC_SET_TRANS_BYC_PARAMS_NV;

typedef struct tACC_Set_Timer_Params_NV {
    uint8_t  on;
    uint32_t num;
} ACC_SET_TIMER_PARAMS_NV;

typedef struct tACC_Set_MoveDetect_Params_NV {
    uint8_t  on;
    uint8_t  axis;
    uint16_t acc_th;
    uint8_t  judge_num;
    uint8_t  judge_th;
    uint8_t  judge;
} ACC_SET_MOVE_DETECT_PARAMS_NV;
    
typedef struct {
    ACC_SET_PEDO_PARAMS_NV        pedo_p;
    ACC_SET_DIST_STOP_PARAMS_NV   dist_stop_p;
    ACC_SET_WALK_RUN_PARAMS_NV    walk_run_p;
    ACC_SET_TRANS_PARAMS_NV       trans_p;
    ACC_SET_TRANS_BYC_PARAMS_NV   trans_byc_p;
    ACC_SET_TIMER_PARAMS_NV       timer_p;
    ACC_SET_MOVE_DETECT_PARAMS_NV move_p;
}IoCtlAccSetAccsnsNVParams;

#define SENSOR_EXT_FUNC_PEDOMETER       0x0001
#define SENSOR_EXT_FUNC_VEHICLEMETER    0x0002
#define SENSOR_EXT_ERR_HWRESET          -2
#define SENSOR_EXT_ERR_OTHER            -1

#define ALPSIO_ACC_SET_EXT_ACTIVATE     _IOW(ALPSIO,22, int)
typedef struct {
  uint32_t m_iType;
  uint32_t m_iEnable;
}IoCtlAccSetExtActivate;

#define ALPSIO_ACC_GET_PEDOM_INFO       _IOW(ALPSIO,23, int)
typedef struct {
  uint32_t m_iStep;
  uint32_t m_iTime;
  uint32_t m_iCalorie;
  uint32_t m_iFat;
  uint32_t m_iSpeedmeter;
  uint32_t m_iMets;
  uint32_t m_iCycle_cal;
  uint32_t m_iCycle_time;
}IoCtlAccGetPedomInfo;

#define ALPSIO_ACC_GET_VEHICLE_INFO     _IOW(ALPSIO,24, int)
typedef struct {
  uint32_t m_iStatus;
  uint32_t m_iKind;
  uint32_t m_iRideTime;
}IoCtlAccGetVehicleInfo;

#define ALPSIO_ACC_SET_PEDOM_CLEAR      _IOW(ALPSIO,25, int)

#define ALPSIO_ACC_SET_VEHICLE_CLEAR    _IOW(ALPSIO,26, int)

#define ALPSIO_ACC_FW_GET_ASYNC_STATUS  _IOW(ALPSIO,27, int)

#define ALPSIO_ACC_FW_UPDATE_SEQ        _IOW(ALPSIO,28, int)

#define ALPSIO_ACC_RECOVERY_INIT        _IOW(ALPSIO,29, int)
typedef struct {
  uint8_t kind;
  uint8_t backup_data[0x1B];
}IoCtlAccRecoveryInfo;

#define ALPSIO_ACC_SET_DEBUG_LEVEL      _IOW(ALPSIO,999, int)

#define ACC_DM_W_X_CMND                0x10
#define ACC_DM_W_Y_CMND                0x11
#define ACC_DM_W_Z_CMND                0x12
#define ACC_DM_W_PITCH_CMND            0x13
#define ACC_DM_W_ROLL_CMND             0x14

#define ACC_DM_W_REL_X_CMND        0x20
#define ACC_DM_W_REL_Y_CMND        0x21
#define ACC_DM_W_REL_Z_CMND        0x22
#define ACC_DM_W_REL_PITCH_CMND    0x23
#define ACC_DM_W_REL_ROLL_CMND     0x24

typedef struct {
  int32_t m_nCommand;
  int32_t m_nData;
}IoCtlAccDataDmSetCmd;
#endif

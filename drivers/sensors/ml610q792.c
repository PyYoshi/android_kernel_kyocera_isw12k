/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 */
/*
 *  ml610q792.c - Linux kernel modules for acceleration sensor
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


#define CONFIG_ML610Q792_DEBUG

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/poll.h>
#include <asm/gpio.h>
#include <linux/types.h>
#include <linux/sensors/ml610q792.h>
#include <linux/sched.h>
#include <linux/earlysuspend.h>
#include <mach/kc_board.h>


#ifdef CONFIG_ML610Q792_DEBUG
#define DBG_LV_IO      0x01
#define DBG_LV_INT     0x02
#define DBG_LV_SPI     0x04
#define DBG_LV_INFO    0x10
#define DBG_LV_DATA    0x20
#define DBG_LV_ERROR   0x40
#define DBG_LV_LOW     0x80
#define DBG_LV_ALL     (DBG_LV_ERROR)
int32_t dbg_level = DBG_LV_ALL;
#define DBG(lv, msg, ...) {                       \
    if( lv & dbg_level ){                         \
        printk(KERN_ERR  msg, ##__VA_ARGS__);     \
    }                                             \
}
#define DBG_PRINT_IO(io,ret) {                                      \
    if( io == 0 ){                                                  \
        DBG(DBG_LV_IO, "[ACC]<%s>:Start\n",__FUNCTION__)            \
    }else{                                                          \
        DBG(DBG_LV_IO, "[ACC]<%s>:End(%x) ret=%x\n",__FUNCTION__ ,io, ret)    \
    }                                                               \
}
#else
#define DBG(lv, msg, ...)
#define DBG_PRINT_IO(io, ret)
#endif

#define ACC_FW_VERSION_NONE          (0x00000000)
#define ACC_FW_VERSION_CYCLE         (0x03000000)
#define ACC_FW_VERSION_RECOVER_1     (0x07000090)
#define ACC_FW_VERSION_RECOVER_2     (0x08000000)

#define ACC_FW_VERSION_DATA          (0x08000090)

#define HOST_VER_PROG_MASK           (0x00000000)

#define CFG                          (0x00u)
#define INTMASK0                     (0x01u)
#define INTMASK1                     (0x02u)
#define STATUS                       (0x08u)
#define INTREQ0                      (0x09u)
#define ERROR0                       (0x0bu)
#define CMD0                         (0x10u)
#define PRM0                         (0x12u)
#define RSLT00                       (0x20u)
#define RSLT0E                       (0x2eu)

#define INTREQ_NONE                  (0x0000u)
#define INTREQ_HOST_CMD              (0x0001u)
#define INTREQ_ACC                   (0x0002u)
#define INTREQ_ERROR                 (0x8000u)

#define ID_NONE                      (0x00u)
#define ID_ACC_PEDO_CNT              (0x02u)
#define ID_ACC_PEDO_MOVE             (0x04u)
#define ID_ACC_PEDO_STOP             (0x08u)
#define ID_ACC_PEDO_WALK_RUN         (0x10u)
#define ID_ACC_PEDO_PEDO_TRANS       (0x20u)
#define ID_ACC_MOVE_DETECT           (0x10u)
#define ID_ACC_XY_DETECT             (0x40u)
#define ID_ACC_PEDO_TIMER            (0x40u)

#define HC_MCU_GET_VERSION           (0x0001u)
#define HC_MCU_SOFT_RESET            (0x0002u)
#define HC_MCU_GET_EX_SENSOR         (0x0003u)
#define HC_MCU_SET_PCON              (0x0004u)
#define HC_MCU_GET_PCON              (0x0005u)
#define HC_MCU_GET_INT_DETAIL        (0x0006u)
#define HC_MCU_SET_PORT_OUT          (0x0007u)
#define HC_MCU_GET_PORT_OUT          (0x0008u)
#define HC_MCU_SELF_CHK_FW           (0x000Au)
#define HC_MCU_I2C_IO                (0x000Cu)
#define HC_MCU_FUP_START             (0x0101u)
#define HC_MCU_FUP_ERASE             (0x0102u)
#define HC_MCU_FUP_WRITE             (0x0103u)
#define HC_MCU_FUP_END               (0x0104u)
#define HC_ACC_MEASURE               (0x1001u)
#define HC_ACC_SET_AUTO_MEASURE      (0x1005u)
#define HC_ACC_GET_AUTO_MEASURE      (0x1006u)
#define HC_ACC_SET_CALIB             (0x1009u)
#define HC_ACC_GET_CALIB             (0x100au)
#define HC_ACC_SET_MOVE_DETECT       (0x1010u)
#define HC_ACC_SET_PEDO              (0x101bu)
#define HC_ACC_GET_PEDO              (0x101cu)
#define HC_ACC_PEDO_CNT              (0x101fu)
#define HC_ACC_SET_DIST_STOP         (0x1020u)
#define HC_ACC_SET_WALK_RUN          (0x1025u)
#define HC_ACC_SET_TRANS             (0x1028u)
#define HC_ACC_GET_TRANS             (0x1029u)
#define HC_ACC_TRANS_INFO            (0x102cu)
#define HC_ACC_PEDO_CLEAR            (0x1049u)
#define HC_ACC_TRANS_CLEAR           (0x104cu)
#define HC_ACC_SET_TRANS_BYC         (0x1050u)
#define HC_ACC_SET_PEDO_TIMER        (0x1056u)
#define HC_MUL_MEASURE               (0xf001u)
#define HC_MUL_SET_ANDROID           (0xf00bu)
#define HC_MUL_GET_ANDROID           (0xf00cu)

#define MT_AUTO_START                (1u)
#define MT_AUTO_STOP                 (2u)
#define MT_ANDROID_START             (2u)
#define MT_ANDROID_STOP              (3u)

#define RT_NORMAL                    (0u)
#define RT_ANDROID                   (1u)

#define HC_INVALID                   (0u)
#define HC_VALID                     (1u)

#define HC_ACC_PEDO_STABLE           (0u)
#define HC_ACC_PEDO_ADD              (2u)
#define HC_ACC_PEDO_CYCLE            (5u)

#define EXE_HOST_WAIT                (1)
#define EXE_HOST_RES                 (2)
#define EXE_HOST_ERR                 (4)
#define EXE_HOST_ALL                 (EXE_HOST_WAIT|EXE_HOST_RES|EXE_HOST_ERR)
#define EXE_HOST_EX_NO_RECOVER       (16)

#define SSIO_MASK_WRITE              (0x7f)
#define SSIO_MASK_READ               (0x80)

#define ERROR_FUP_MAXSIZE            (0x0011u)
#define ERROR_FUP_VERIFY             (0x0012u)
#define ERROR_FUP_CERTIFICATION      (0x0013u)

#define ACC_DRIVER_NAME              "accsns"
#define ACC_GPIO_INT_NAME            "accsns_irq"
#define ACC_GPIO_RESET_NAME          "accsns_reset"
#define ACC_GPIO_TEST0_NAME          "accsns_test0"

#define ACCSNS_GPIO_INT              (18)
#define ACCSNS_GPIO_TEST0            (25)
#define ACCSNS_GPIO_RST              (120)

#define ONESEC_MS                    (1000)
#define SETTING_0G                   (0)
#define WEIGHT_1G                    (1024)

#define ACCSNS_RC_OK                 (0)
#define ACCSNS_RC_OK_NACK            (1)
#define ACCSNS_RC_ERR                (-1)
#define ACCSNS_RC_ERR_TIMEOUT        (-2)

#define WAITEVENT_TIMEOUT            (3000)

#define DEFAULT_FREQ                 (50)
#define OFFSET_SUMPLE_NUM            (100)
#define OFFSET_AVE_NUM               (1)
#define DEFAULT_WEIGHT               (60)
#define DEFAULT_PEDOMETER            (40)
#define DEFAULT_ACC_DELAY            (20)
#define PEDO_TIMER_COUNT_COEFF       (30)
#define DEFAULT_PEDO_TIMER           (120000 / PEDO_TIMER_COUNT_COEFF)

#define DEFAULT_PAUSE_TIMER          (100)

#define    POWER_DISABLE       false
#define    POWER_ENABLE        true

#define    ACTIVE_OFF          0x00
#define    ACTIVE_ON           0x01

#define ACC_SPI_RETRY_NUM    5

#define ACC_WORK_QUEUE_NUM   5

enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_XYZ_MAX
} AXIS;

enum {
    MODE_0 = 0,
    MODE_1,
    MODE_2,
    MODE_3,
    MODE_4,
    MODE_MAX
}MODE;

enum {
    DETECT_NONE = 0,
    DETECT_STOP,
    DETECT_MOVE,
    DETECT_MAX,
}MOVE_DETECT;

typedef union {
    uint16_t   udata16;
    int16_t    sdata16;
    uint8_t    udata8[2];
    int8_t     sdata8[2];
} Word;

typedef union {
    uint8_t    ub_prm[13];
    int8_t     sb_prm[13];
    uint16_t   uw_prm[6];
    int16_t    sw_prm[6];
    uint32_t   ud_prm[3];
    int32_t    sd_prm[3];
} HCParam;

typedef union {
    uint8_t    ub_res[12];
    int8_t     sb_res[12];
    uint16_t   uw_res[6];
    int16_t    sw_res[6];
    uint32_t   ud_res[3];
    int32_t    sd_res[3];
} HCRes;

typedef struct {
    Word       cmd;
    HCParam    prm;
} HostCmd;

typedef struct {
    HCRes      res;
    Word       err;
} HostCmdRes;

struct acceleration {
    int32_t nX;
    int32_t nY;
    int32_t nZ;
};

struct pedometer {
    int32_t usStepCnt;
    int32_t usWalkTime;
    int32_t usCal;
    int32_t usBodyFat;
    int32_t usSpeed;
    int32_t usMets;
    int32_t usCycle_cal;
    int32_t usCycle_time;
};

struct vehicle {
    int32_t usVehiStatus;
    int32_t usVehiKind;
};

struct move {
    int32_t usMoveStatus;
};

typedef struct tMovFilterWork {
    int32_t* m_pSamplWork[AXIS_XYZ_MAX];
    int32_t  m_unSum[AXIS_XYZ_MAX];
    int32_t  m_unCnt[AXIS_XYZ_MAX];
    uint8_t  m_ucAveN;
} MovFilterWork;

typedef struct tCalibrationCtrl {
    bool     m_bFilterEnable;
    bool     m_bWaitSetting;
    bool     m_bComplete;
    uint16_t m_unSmpN;
    MovFilterWork m_tFilterWork;
    int32_t  m_nCalX;
    int32_t  m_nCalY;
    int32_t  m_nCalZ;
    int32_t  m_nCurrentSampleNum;
    int32_t  m_nSummationX;
    int32_t  m_nSummationY;
    int32_t  m_nSummationZ;
    int32_t  m_nMode;
    struct mutex m_tCalibMutex;
} CalibrationCtrl;

typedef struct tAsyncFWUpdateCtrl {
  uint8_t *m_cData;
  uint32_t m_nLen;
  uint32_t m_nResult;
  bool     m_bComplete;
  bool     m_bRun;
} AsyncFWUpdateCtrl;
AsyncFWUpdateCtrl    s_tAsyncFWUpdateData;

typedef struct t_ACC_WorkQueue {
    struct work_struct  work;
    bool                status;
} ACC_WorkQueue;
static ACC_WorkQueue  s_tAccWork[ACC_WORK_QUEUE_NUM];

struct ped_work_struct
{
    struct work_struct work_pause;
    bool status;
    int stop_timer;
};
static struct ped_work_struct s_twork_pause[ACC_WORK_QUEUE_NUM];

struct semaphore s_tAccSnsrSema;
static int32_t g_nAccFWVersion;
static int32_t g_nIntIrqFlg;
static int32_t g_nIntIrqNo;
static int32_t s_nAccWorkCnt;
static wait_queue_head_t s_tWaitInt;
static wait_queue_head_t s_tPollWaitPedom;
static wait_queue_head_t s_tPollWaitVehicle;
static CalibrationCtrl s_tCalibCtrl;
static IoCtlAccSetAccsnsNVParams g_acc_nv_param;
static bool g_bSpiError = false;
static uint32_t g_bPedoTimerStatus = false;
static int work_ped_pos = 0;
static bool dummy_on_flg = false;
static struct spi_driver accsns_driver;
static struct spi_device *client_accsns;
static struct workqueue_struct *accsns_wq_int;
static struct workqueue_struct *accsns_wq;
static struct workqueue_struct *fw_update_wq;
static struct work_struct  s_tWork_FW_Update;
static struct workqueue_struct *accsns_wq_pause;
static struct delayed_work s_tDelayWork_Acc;
static struct mutex s_tDataMutex;
static struct acceleration s_tLatestAccData;
static struct pedometer    s_tLatestPedoData;
static struct vehicle      s_tLatestVehiData;
static struct move         s_tLatestMoveData;
static struct hrtimer ped_timer;
static DEFINE_SPINLOCK(acc_lock);

static atomic_t g_CurrentSensorEnable;
static atomic_t g_nMoveDetectStatus;
static atomic_t g_bIsIntIrqEnable;
static atomic_t g_flgEna;
static atomic_t s_nDelayTime;
static atomic_t g_nCalX;
static atomic_t g_nCalY;
static atomic_t g_nCalZ;
static atomic_t g_nWeight;
static atomic_t g_nStepWide;
static atomic_t g_nPedoTimerCount;
static atomic_t g_WakeupSensor;
static atomic_t g_FWUpdateStatus;
static atomic_t g_pause_sens;


static int32_t accsns_mov_acc_avg (MovFilterWork* pCtrl, int32_t sample,int32_t axis);
static void accsns_calibration_periodic(const struct acceleration* accData);
static int32_t accsns_activateEx(int32_t arg_iUpdate, int32_t arg_iSensType, int32_t arg_iEnable);
static int32_t accsns_power_onoff(bool arg_iEnable);
static int32_t accsns_acc_activate(bool arg_iEnable);
static int32_t accsns_acc_getdata(struct acceleration *arg_Acc);
static int32_t accsns_acc_getdata(struct acceleration *arg_Acc);
static int32_t accsns_acc_setcalib(void);
static void accsns_acc_work_func(struct work_struct *work);
static int32_t accsns_pedom_activate(bool arg_iEnable);
static int32_t accsns_pedom_getdata(struct pedometer *arg_Pedo);
static int32_t accsns_vehicle_activate(bool arg_iEnable);
static int32_t accsns_vehicle_getdata(struct vehicle *arg_Vehi);
static int32_t accsns_move_activate(bool arg_iEnable);
static int32_t accsns_move_detect(bool arg_iEnable);
static int32_t accsns_stop_detect(bool arg_iEnable);
static int32_t accsns_measure(int32_t arg_iSensType);
int32_t accsns_check_accsensor(void);
static int32_t accsns_chip_read(uint8_t slave, uint8_t adr, uint8_t *data, uint16_t size);
static int32_t accsns_chip_write(uint8_t slave, uint8_t adr, const uint8_t *data, uint8_t size);
static int32_t CalcCRC8(uint8_t *p_MemAddr, uint8_t BlockSize);
static int32_t accsns_nvm_restore(uint8_t *backup_data);
static int32_t accsns_nvm_backup(uint8_t *backup_data);
static int32_t accsns_hostcmd(const HostCmd *prm, HostCmdRes *res, uint8_t mode);
static int32_t accsns_waitcmd(uint16_t intBit);
static irqreturn_t accsns_irq_handler(int32_t irq, void *dev_id);
static void accsns_int_work_func(struct work_struct *work);
static void accsns_int_acc_work_func(struct work_struct *work);
static void accsns_FWUpdate_work_func(struct work_struct *work);
static int32_t accsns_gpio_init(void);
static void accsns_shutdown( struct spi_device *client );
static int32_t accsns_remove( struct spi_device *client );
static int32_t accsns_probe( struct spi_device *client );
static int32_t accsns_pedom_set_timer(uint32_t arg_iEnable, uint32_t arg_iCount);
static int32_t accsns_spi_read_proc(uint8_t adr, uint8_t *data, uint16_t size);
static int32_t accsns_spi_write_proc(uint8_t adr, const uint8_t *data, uint8_t size);
static int32_t accsns_workqueue_create( struct workqueue_struct *queue, void (*func)(struct work_struct *) );
static void accsns_timeout_dump(uint8_t *reg);
static void accsns_workqueue_init(void);
static int32_t accsns_workqueue_create( struct workqueue_struct *queue, void (*func)(struct work_struct *) );
static void accsns_workqueue_delete(struct work_struct *work);
static void hrtimer_cancel_timer_func(struct hrtimer *timer);

#define WAIT_FOR_EVENT_MS(time){                                             \
    int32_t  flg = 0;                                                        \
    long timeout;                                                            \
    timeout = msecs_to_jiffies(time);                                        \
    while(1){                                                                \
        if( wait_event_interruptible_timeout(s_tWaitInt, flg, timeout) == 0){\
            break;                                                           \
        }                                                                    \
    }                                                                        \
}

#define ENABLE_IRQ {                                                         \
    if((g_nIntIrqNo != -1) && (atomic_read(&g_bIsIntIrqEnable) == false)){   \
        atomic_set(&g_bIsIntIrqEnable,true);                                 \
        enable_irq(g_nIntIrqNo);                                             \
    }                                                                        \
}
#define DISABLE_IRQ {                                                        \
    if((g_nIntIrqNo != -1) && (atomic_read(&g_bIsIntIrqEnable) == true)){    \
        disable_irq_nosync(g_nIntIrqNo);                                     \
        atomic_set(&g_bIsIntIrqEnable,false);                                \
    }                                                                        \
}

#define ERR_WAKEUP {                                                                \
            int32_t iWakeupSensor;                                                  \
            int32_t iCurrentSensorEnable;                                           \
            iWakeupSensor = atomic_read(&g_WakeupSensor);                           \
            iCurrentSensorEnable = atomic_read(&g_CurrentSensorEnable);             \
            atomic_set(&g_WakeupSensor, iWakeupSensor | ((iCurrentSensorEnable >> 4) & ACCSNS_ACTIVE_ERROR));  \
            wake_up_interruptible(&s_tPollWaitPedom);                               \
            wake_up_interruptible(&s_tPollWaitVehicle);                             \
}

#define ACC_FW_VERSION_GET_32BIT(data)         ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]))

#define ACCDATA_SIGN_COMVERT_12_32BIT(data)    ((data) & 0x00000800 ? (((~(data) & 0x00000FFF) + 1) * (-1)): (data))

int32_t accsns_dm_seqctrl(int32_t cmd)
{
    int32_t ret = ACCSNS_RC_ERR;
DBG_PRINT_IO(0, 0);
    
    switch(cmd){
        case ACC_DM_R_INIT_SEQ_CMND:
            ret = accsns_initialize();
            ret |= accsns_check_accsensor();
            break;
        
        case ACC_DM_R_POLL_ON_CMND:
            ret = accsns_activateEx(0, ACCSNS_ACTIVE_ACC, POWER_ENABLE);
            break;
        
        case ACC_DM_R_POLL_OFF_CMND:
            ret = accsns_activateEx(0, ACCSNS_ACTIVE_ACC, POWER_DISABLE);
            break;
        
        case ACC_DM_R_PEDOMETER_ON_CMND:
            ret = accsns_activateEx(0, ACCSNS_ACTIVE_PEDOM, POWER_ENABLE);
            break;
        
        case ACC_DM_R_PEDOMETER_OFF_CMND:
            ret = accsns_activateEx(0, ACCSNS_ACTIVE_PEDOM, POWER_DISABLE);
            break;
        
        case ACC_DM_R_PEDOMETER_AND_POLL_ON_CMND:
            ret = accsns_activateEx(0, ACCSNS_ACTIVE_ACC, POWER_ENABLE);
            ret = accsns_activateEx(0, ACCSNS_ACTIVE_PEDOM, POWER_ENABLE);
            break;
        
        case ACC_DM_R_PEDOMETER_INFO_CLEAR_CMND:
            ret = accsns_pedom_clear();
            break;

        case ACC_DM_R_VEHICLE_ON_CMND:
            ret = accsns_activateEx(0, ACCSNS_ACTIVE_VEHICLE, POWER_ENABLE);
            break;

        case ACC_DM_R_VEHICLE_OFF_CMND:
            ret = accsns_activateEx(0, ACCSNS_ACTIVE_VEHICLE, POWER_DISABLE);
            break;

        case ACC_DM_R_VEHICLE_AND_POLL_ON_CMND:
            ret = accsns_activateEx(0, ACCSNS_ACTIVE_ACC, POWER_ENABLE);
            ret = accsns_activateEx(0, ACCSNS_ACTIVE_VEHICLE, POWER_ENABLE);
            break;

        case ACC_DM_R_VEHICLE_INFO_CLEAR_CMND:
            ret = accsns_vehicle_clear();
            break;

        case ACC_DM_R_CORRECT_VALUE_SET_CMND:
            ret = accsns_acc_setcalib();
            break;
        
        case ACC_DM_R_MOVE_ON_CMND:
            ret = accsns_activateEx(0, ACCSNS_ACTIVE_MOVE, POWER_ENABLE);
            break;

        case ACC_DM_R_MOVE_OFF_CMND:
            ret = accsns_activateEx(0, ACCSNS_ACTIVE_MOVE, POWER_DISABLE);
            break;
        
        case ACC_DM_R_PEDOM_TIMER_ON_CMND:
            ret = accsns_pedom_set_timer(1, g_acc_nv_param.timer_p.num);
            break;

        case ACC_DM_R_PEDOM_TIMER_OFF_CMND:
            ret = accsns_pedom_set_timer(0, 0);
            break;
                
        default:
            ret = ACCSNS_RC_ERR;
            break;
    }
    
    if(ret != 0){
        DBG(DBG_LV_ERROR, "DM command error %x\n",ret);
    }
    
DBG(DBG_LV_INFO, "DM Command %x, ret %x\n",cmd, ret);
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

int32_t accsns_get_current_active(void)
{
    int32_t iCurrentEnable;
    
    iCurrentEnable = (atomic_read(&g_CurrentSensorEnable) | atomic_read(&g_pause_sens));
    
    return (iCurrentEnable & ACTIVE_FUNC_MASK);
}

int32_t accsns_get_acceleration_data( int32_t* arg_ipXYZ )
{
    int32_t ret = ACCSNS_RC_OK;
    int32_t nCalX,nCalY,nCalZ;
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        memset(arg_ipXYZ, 0x00, (sizeof(int32_t)*3));
        return 0;
    }

    nCalX = atomic_read(&g_nCalX);
    nCalY = atomic_read(&g_nCalY);
    nCalZ = atomic_read(&g_nCalZ);

    if(delayed_work_pending(&s_tDelayWork_Acc)) {
DBG(DBG_LV_LOW, "pending...\n");
        
    } else {
        ret = accsns_measure(ACCSNS_ACTIVE_ACC);
    }
    
    if(ret == ACCSNS_RC_OK){
        mutex_lock(&s_tDataMutex);
        arg_ipXYZ[0] = s_tLatestAccData.nX - (int16_t)nCalX;
        arg_ipXYZ[1] = s_tLatestAccData.nY - (int16_t)nCalY;
        arg_ipXYZ[2] = s_tLatestAccData.nZ - (int16_t)nCalZ;
        mutex_unlock(&s_tDataMutex);
    }
  
DBG(DBG_LV_LOW, "Acc, s_tLatestAccData x:%d, y:%d, z:%d\n", s_tLatestAccData.nX, s_tLatestAccData.nY, s_tLatestAccData.nZ );
DBG(DBG_LV_LOW, "Acc, nCal             x:%d, y:%d, z:%d\n", nCalX, nCalY, nCalZ );
DBG(DBG_LV_LOW, "Acc, ret=%02d         x:%d, y:%d, z:%d\n", ret, arg_ipXYZ[0], arg_ipXYZ[1], arg_ipXYZ[2] );
    return ret;
}

int32_t accsns_get_pedometer_data( int32_t* arg_ipPedom )
{
    int32_t ret = 0;
DBG_PRINT_IO(0, 0);
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        memset(arg_ipPedom, 0x00, (sizeof(int32_t)*8));
        return 0;
    }

    ret = accsns_measure(ACCSNS_ACTIVE_PEDOM);
    
    if(ret == ACCSNS_RC_OK){
        mutex_lock(&s_tDataMutex);
        arg_ipPedom[0] = s_tLatestPedoData.usStepCnt;
        arg_ipPedom[1] = s_tLatestPedoData.usWalkTime;
        arg_ipPedom[2] = s_tLatestPedoData.usCal;
        arg_ipPedom[3] = s_tLatestPedoData.usBodyFat;
        arg_ipPedom[4] = s_tLatestPedoData.usSpeed;
        arg_ipPedom[5] = s_tLatestPedoData.usMets;
        arg_ipPedom[6] = s_tLatestPedoData.usCycle_cal;
        arg_ipPedom[7] = s_tLatestPedoData.usCycle_time;
        mutex_unlock(&s_tDataMutex);
    }
  
DBG(DBG_LV_DATA, "Pedom, ret=%d usStepCnt:%d, usWalkTime:%d, usCal:%d, usBodyFat:%d, usSpeed:%d, usMets:%d, usCycle_cal:%d usCycle_time:%d\n",
         ret, arg_ipPedom[0], arg_ipPedom[1], arg_ipPedom[2], arg_ipPedom[3], arg_ipPedom[4], arg_ipPedom[5], arg_ipPedom[6], arg_ipPedom[7] );
  
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

int32_t accsns_get_vehicle_data( int32_t* arg_ipVehicle )
{
    int32_t ret = 0;
DBG_PRINT_IO(0, 0);
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        arg_ipVehicle[0] = 0;
        arg_ipVehicle[1] = 0;
        return 0;
    }

    ret = accsns_measure(ACCSNS_ACTIVE_VEHICLE);
    
    if(ret == ACCSNS_RC_OK){
        mutex_lock(&s_tDataMutex);
        arg_ipVehicle[0] = s_tLatestVehiData.usVehiStatus;
        arg_ipVehicle[1] = s_tLatestVehiData.usVehiKind;
        mutex_unlock(&s_tDataMutex);
    }
  
DBG(DBG_LV_DATA, "Vehicle, ret=%d Status:%02d, %02d\n", ret, arg_ipVehicle[0], arg_ipVehicle[1]);
  
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

int32_t accsns_pedom_set_info(uint32_t arg_iWeight, uint32_t arg_iStepWide, uint32_t arg_iDelay)
{
    uint32_t iPedoTimerCount;
DBG_PRINT_IO(0, 0);

DBG(DBG_LV_INFO, "accsns_pedom_set_info\n");
    if(arg_iWeight > 0xFF){
        arg_iWeight = DEFAULT_WEIGHT;
    }
    atomic_set(&g_nWeight, arg_iWeight);
    
    if(arg_iStepWide > 0xFF){
        arg_iStepWide = DEFAULT_PEDOMETER;
    }
    atomic_set(&g_nStepWide, arg_iStepWide);
    
    iPedoTimerCount = arg_iDelay / PEDO_TIMER_COUNT_COEFF;
    atomic_set(&g_nPedoTimerCount, iPedoTimerCount);

DBG(DBG_LV_INFO, "arg_iWeight=%d, arg_iStepWide=%d arg_iDelay=%d\n", arg_iWeight, arg_iStepWide, arg_iDelay);
DBG_PRINT_IO(0xFF, 0);
    return 0;
}

int32_t accsns_get_move_data( int32_t* arg_ipMove )
{
    int32_t ret = 0;
DBG_PRINT_IO(0, 0);
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        arg_ipMove[0] = 0;
        return 0;
    }

    ret = accsns_measure(ACCSNS_ACTIVE_MOVE);
    
    if(ret == ACCSNS_RC_OK){
        mutex_lock(&s_tDataMutex);
        arg_ipMove[0] = s_tLatestMoveData.usMoveStatus;
        mutex_unlock(&s_tDataMutex);
    }
  
DBG(DBG_LV_DATA, "Moving, ret=%d Status:%d\n", ret, arg_ipMove[0]);
  
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

int32_t accsns_activate(int32_t arg_iUpdate, int32_t arg_iEnable)
{
DBG(DBG_LV_INFO, "accsns_activate \n");
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return 0;
    }

    return accsns_activateEx(arg_iUpdate, ACCSNS_ACTIVE_ACC, arg_iEnable);
}

int32_t accsns_activate_pedom(int32_t arg_iUpdate, int32_t arg_iEnable)
{
DBG(DBG_LV_INFO, "accsns_activate_pedom \n");

    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return 0;
    }

    return accsns_activateEx(arg_iUpdate, ACCSNS_ACTIVE_PEDOM, arg_iEnable);
}

int32_t accsns_activate_vehicle(int32_t arg_iUpdate, int32_t arg_iEnable)
{
DBG(DBG_LV_INFO, "accsns_activate_vehicle \n");

    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_ERROR, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return 0;
    }

    return accsns_activateEx(arg_iUpdate, ACCSNS_ACTIVE_VEHICLE, arg_iEnable);
}

int32_t accsns_calibration_mode(void)
{
    int32_t i = 0;

DBG_PRINT_IO(0, 0);
    
    mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));

    s_tCalibCtrl.m_bFilterEnable        = false;
    s_tCalibCtrl.m_bWaitSetting         = true;
    s_tCalibCtrl.m_bComplete            = false;

    s_tCalibCtrl.m_unSmpN               = OFFSET_SUMPLE_NUM; 
    s_tCalibCtrl.m_tFilterWork.m_ucAveN = OFFSET_AVE_NUM;   
    s_tCalibCtrl.m_nCalX                = 0;
    s_tCalibCtrl.m_nCalY                = 0;
    s_tCalibCtrl.m_nCalZ                = 0;

    s_tCalibCtrl.m_nCurrentSampleNum    = 0;
    s_tCalibCtrl.m_nSummationX          = 0;
    s_tCalibCtrl.m_nSummationY          = 0;
    s_tCalibCtrl.m_nSummationZ          = 0;
    s_tCalibCtrl.m_nMode                = -1; 
    
    for (i = 0; i < AXIS_XYZ_MAX; i++) {
        if(s_tCalibCtrl.m_tFilterWork.m_pSamplWork[i] != NULL) {
DBG(DBG_LV_ERROR, "ACC Calib:err occurd \n");
            
        } else {
            s_tCalibCtrl.m_tFilterWork.m_pSamplWork[i] = NULL;
        }
    }
    memset(s_tCalibCtrl.m_tFilterWork.m_unSum, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);
    memset(s_tCalibCtrl.m_tFilterWork.m_unCnt, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);

DBG(DBG_LV_INFO, "Enter AccCalibration Mode\n");
    mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

int32_t accsns_calibration_start(int32_t argMode)
{
    int32_t delay = 0;
DBG_PRINT_IO(0, 0);
    
    mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
    s_tCalibCtrl.m_bWaitSetting = false;
    s_tCalibCtrl.m_nMode = argMode;

    s_tCalibCtrl.m_nCurrentSampleNum    = 0;
    s_tCalibCtrl.m_nSummationX          = 0;
    s_tCalibCtrl.m_nSummationY          = 0;
    s_tCalibCtrl.m_nSummationZ          = 0;

    memset(s_tCalibCtrl.m_tFilterWork.m_unSum, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);
    memset(s_tCalibCtrl.m_tFilterWork.m_unCnt, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);

DBG(DBG_LV_INFO, "Start AccCalibration Mode %d[%d]\n", s_tCalibCtrl.m_nMode,delay);
    mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));

    delay = atomic_read(&s_nDelayTime);
    schedule_delayed_work(&s_tDelayWork_Acc,msecs_to_jiffies(delay));

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

int32_t accsns_calibration_is_wait(void)
{
    int32_t wait;
DBG_PRINT_IO(0, 0);
    
    mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
    wait = s_tCalibCtrl.m_bWaitSetting;
    mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));

DBG(DBG_LV_INFO, "calib Wait %d\n", wait);
DBG_PRINT_IO(0xFF, wait);
    return wait;
}

int32_t accsns_calibration_is_comp(int32_t* argCal)
{
    int32_t comp;
DBG_PRINT_IO(0, 0);
    
    mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
    if(s_tCalibCtrl.m_bComplete == true) {
        argCal[AXIS_X] = s_tCalibCtrl.m_nCalX;
        argCal[AXIS_Y] = s_tCalibCtrl.m_nCalY;
        argCal[AXIS_Z] = s_tCalibCtrl.m_nCalZ;
    }
    
    comp = s_tCalibCtrl.m_bComplete;
    mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));

DBG(DBG_LV_INFO, "calib Wait %d\n", comp);
DBG_PRINT_IO(0xFF, comp);
    return comp;
}

void accsns_set_freq(uint8_t freq)
{
    uint8_t temp;
    
DBG_PRINT_IO(0,0);
    
    temp = freq;
    if(temp == 0) {
        temp = DEFAULT_FREQ;
    }

    atomic_set(&s_nDelayTime, ONESEC_MS/temp);

DBG(DBG_LV_INFO, "accsns freq = %d (%d)\n", temp, atomic_read(&s_nDelayTime));
DBG_PRINT_IO(0xFF, 0);
}

void accsns_set_offset(int32_t* offsets)
{
    int32_t temp;
DBG_PRINT_IO(0, 0);
    
    temp = ACCDATA_SIGN_COMVERT_12_32BIT(offsets[AXIS_X]);
    atomic_set(&g_nCalX, temp);
    
    temp = ACCDATA_SIGN_COMVERT_12_32BIT(offsets[AXIS_Y]);
    atomic_set(&g_nCalY, temp);
    
    temp = ACCDATA_SIGN_COMVERT_12_32BIT(offsets[AXIS_Z]);
    atomic_set(&g_nCalZ, temp);

DBG(DBG_LV_INFO, "set offset X %x -> %d\n", offsets[AXIS_X], temp);
DBG(DBG_LV_INFO, "set offset Y %x -> %d\n", offsets[AXIS_Y], temp);
DBG(DBG_LV_INFO, "set offset Z %x -> %d\n", offsets[AXIS_Z], temp);

DBG_PRINT_IO(0xFF, 0);
}

void accsns_set_nv_params(IoCtlAccSetAccsnsNVParams* argParam)
{
DBG_PRINT_IO(0, 0);
    
    memcpy(&g_acc_nv_param, argParam, sizeof(g_acc_nv_param));
    
    atomic_set(&g_nWeight,   g_acc_nv_param.pedo_p.weight);
    atomic_set(&g_nStepWide, g_acc_nv_param.pedo_p.stepwide);
    atomic_set(&g_nPedoTimerCount, g_acc_nv_param.timer_p.num);
    
DBG(DBG_LV_INFO, "set param.pedo_p.param_on             :%x \n", (int)g_acc_nv_param.pedo_p.param_on             );
DBG(DBG_LV_INFO, "set param.pedo_p.on                   :%x \n", (int)g_acc_nv_param.pedo_p.on                   );
DBG(DBG_LV_INFO, "set param.pedo_p.stepwide             :%x \n", (int)g_acc_nv_param.pedo_p.stepwide             );
DBG(DBG_LV_INFO, "set param.pedo_p.weight               :%x \n", (int)g_acc_nv_param.pedo_p.weight               );
DBG(DBG_LV_INFO, "set param.pedo_p.notify               :%x \n", (int)g_acc_nv_param.pedo_p.notify               );
DBG(DBG_LV_INFO, "set param.pedo_p.speed_ave_time       :%x \n", (int)g_acc_nv_param.pedo_p.speed_ave_time       );
DBG(DBG_LV_INFO, "set param.pedo_p.mets_stop_time       :%x \n", (int)g_acc_nv_param.pedo_p.mets_stop_time       );
DBG(DBG_LV_INFO, "set param.pedo_p.bodyfat_on           :%x \n", (int)g_acc_nv_param.pedo_p.bodyfat_on           );
DBG(DBG_LV_INFO, "set param.pedo_p.bodyfat_cal          :%x \n", (int)g_acc_nv_param.pedo_p.bodyfat_cal          );
DBG(DBG_LV_INFO, "set param.dist_stop_p.stop_notify_on  :%x \n", (int)g_acc_nv_param.dist_stop_p.stop_notify_on  );
DBG(DBG_LV_INFO, "set param.dist_stop_p.stop_notify_time:%x \n", (int)g_acc_nv_param.dist_stop_p.stop_notify_time);
DBG(DBG_LV_INFO, "set param.walk_run_p.walk_judge_on    :%x \n", (int)g_acc_nv_param.walk_run_p.walk_judge_on    );
DBG(DBG_LV_INFO, "set param.walk_run_p.consecutive_num  :%x \n", (int)g_acc_nv_param.walk_run_p.consecutive_num  );
DBG(DBG_LV_INFO, "set param.walk_run_p.speed_th         :%x \n", (int)g_acc_nv_param.walk_run_p.speed_th         );
DBG(DBG_LV_INFO, "set param.trans_p.on                  :%x \n", (int)g_acc_nv_param.trans_p.on                  );
DBG(DBG_LV_INFO, "set param.trans_p.judge_step          :%x \n", (int)g_acc_nv_param.trans_p.judge_step          );
DBG(DBG_LV_INFO, "set param.trans_p.judge_time          :%x \n", (int)g_acc_nv_param.trans_p.judge_time          );
DBG(DBG_LV_INFO, "set param.trans_p.calc_time           :%x \n", (int)g_acc_nv_param.trans_p.calc_time           );
DBG(DBG_LV_INFO, "set param.trans_p.consecutive_num     :%x \n", (int)g_acc_nv_param.trans_p.consecutive_num     );
DBG(DBG_LV_INFO, "set param.trans_byc_p.consecutive_num :%x \n", (int)g_acc_nv_param.trans_byc_p.consecutive_num );
DBG(DBG_LV_INFO, "set param.trans_byc_p.calory_mets     :%x \n", (int)g_acc_nv_param.trans_byc_p.calory_mets     );
DBG(DBG_LV_INFO, "set param.timer_p.on                  :%x \n", (int)g_acc_nv_param.timer_p.on                  );
DBG(DBG_LV_INFO, "set param.timer_p.num                 :%x \n", (int)g_acc_nv_param.timer_p.num                 );
DBG(DBG_LV_INFO, "set param.move_p.on                   :%x \n", (int)g_acc_nv_param.move_p.on                   );
DBG(DBG_LV_INFO, "set param.move_p.axis                 :%x \n", (int)g_acc_nv_param.move_p.axis                 );
DBG(DBG_LV_INFO, "set param.move_p.acc_th               :%x \n", (int)g_acc_nv_param.move_p.acc_th               );
DBG(DBG_LV_INFO, "set param.move_p.judge_num            :%x \n", (int)g_acc_nv_param.move_p.judge_num            );
DBG(DBG_LV_INFO, "set param.move_p.judge_th             :%x \n", (int)g_acc_nv_param.move_p.judge_th             );
DBG(DBG_LV_INFO, "set param.move_p.judge                :%x \n", (int)g_acc_nv_param.move_p.judge                );

DBG_PRINT_IO(0xFF, 0);
}

void accsns_set_delay(int32_t delay){};

static int32_t accsns_spi_write_proc(uint8_t adr, const uint8_t *data, uint8_t size)
{
    int32_t ret = 0;
    struct spi_message  message;
    struct spi_transfer transfer;
    uint8_t send_data[100];
DBG(DBG_LV_SPI, "[ACC]<%s>:Start adr=%x size=%x\n",__FUNCTION__, adr, size);

    if((data == NULL) || (size == 0)){
DBG(DBG_LV_ERROR, "SPI write input error(data %p, size %x)\n", data, size);
        return ACCSNS_RC_ERR;
    }
    
    send_data[0] = adr;
    memcpy(&send_data[1], data, size);
    memset(&transfer, 0, sizeof(transfer));

    adr &= SSIO_MASK_WRITE;
    
    ret = spi_setup(client_accsns);
    if(ret < 0) {
DBG(DBG_LV_ERROR, "init SPI failed. ret=%x\n", ret);
        return ret;
    }
    spi_message_init(&message);
    
    transfer.tx_buf = send_data;
    transfer.rx_buf = NULL;
    transfer.len    = 1 + size;
    spi_message_add_tail(&transfer, &message);
    
    ret = spi_sync(client_accsns, &message);
    if(ret < 0){
DBG(DBG_LV_ERROR, "SPI write error(ret %x, adr %x, size %x)", ret, adr, size);
    }   
    
DBG(DBG_LV_SPI, "[ACC]<%s>:End ret=%x\n",__FUNCTION__, ret);
    return ret;
}

static int32_t accsns_spi_read_proc(uint8_t adr, uint8_t *data, uint16_t size)
{
    int32_t ret = 0;
    struct spi_message  message;
    struct spi_transfer transfer[2];
    
DBG(DBG_LV_SPI, "[ACC]<%s>:Start adr=%x size=%x\n",__FUNCTION__, adr, size);

    
    if( (data == NULL) || (size == 0)){
        DBG(DBG_LV_ERROR, "SPI read input error(data %p, size %x)", data, size);
        return ACCSNS_RC_ERR;
    }
    
    memset(&transfer, 0, sizeof(transfer));
    
    adr |= SSIO_MASK_READ;
    
    ret = spi_setup(client_accsns);
    if(ret < 0){
DBG(DBG_LV_ERROR, "init SPI failed. ret=%x\n", ret);
        return ret;
    }
    spi_message_init(&message);

    transfer[0].tx_buf = &adr;
    transfer[0].rx_buf = NULL;
    transfer[0].len    = 1;
    spi_message_add_tail(&transfer[0], &message);

    transfer[1].tx_buf = NULL;
    transfer[1].rx_buf = (void *)data;
    transfer[1].len    = size;
    spi_message_add_tail(&transfer[1], &message);

    ret = spi_sync(client_accsns, &message);
    if(ret < 0){
DBG(DBG_LV_ERROR, "SPI read error(ret %x, adr %x, size %x)", ret, adr, size);
    }

DBG(DBG_LV_SPI, "[ACC]<%s>:End ret=%x\n",__FUNCTION__, ret);
    return ret;
}

int32_t accsns_spi_write(uint8_t adr, const uint8_t *data, uint8_t size)
{
    int32_t i;
    int32_t ret;
    
    for(i=0; i<ACC_SPI_RETRY_NUM; i++)
    {
        ret = accsns_spi_write_proc(adr, data, size);
        if(ret == 0){
            return 0;
            
        }else if(ret == -EBUSY){
DBG(DBG_LV_ERROR, "SPI write EBUSY error(Retry:%d)\n", i);
            msleep(100);
            
        }else{
            g_bSpiError = true;
DBG(DBG_LV_ERROR, "SPI write Other error (H/W Reset ON) \n");
            break;
        }
    }
    
    return ret;
}
int32_t accsns_spi_read(uint8_t adr, uint8_t *data, uint16_t size)
{
    int32_t i;
    int32_t ret;
    
    for(i=0; i<ACC_SPI_RETRY_NUM; i++)
    {
        ret = accsns_spi_read_proc(adr, data, size);
        if(ret == 0){
            return 0;
            
        }else if(ret == -EBUSY){
DBG(DBG_LV_ERROR, "SPI read EBUSY error(Retry:%d)\n", i);
            msleep(100);
            
        }else{
            g_bSpiError = true;
DBG(DBG_LV_ERROR, "SPI read Other error (H/W Reset ON) \n");
            break;
        }
    }
    
    return ret;
}


bool accsns_spi_error_check(void)
{
DBG(DBG_LV_SPI, "accsns_spi_error_check\n");
    return g_bSpiError;
}

int32_t accsns_io_poll_pedom(struct file *fp, poll_table *wait)
{
    int32_t iWakeupSensor;
    int32_t ret = 0;

    poll_wait(fp, &s_tPollWaitPedom, wait);

    iWakeupSensor = atomic_read(&g_WakeupSensor);
    if ((iWakeupSensor & ACCSNS_ACTIVE_PEDOM_ERROR) == ACCSNS_ACTIVE_PEDOM_ERROR) {
        atomic_set(&g_WakeupSensor, (iWakeupSensor & ~ACCSNS_ACTIVE_PEDOM_ERROR));
        ret |= ACCSNS_ACTIVE_PEDOM_ERROR;
    } else if ((iWakeupSensor & ACCSNS_ACTIVE_PEDOM) == ACCSNS_ACTIVE_PEDOM) {
        atomic_set(&g_WakeupSensor, (iWakeupSensor & ~ACCSNS_ACTIVE_PEDOM));
        ret |= ACCSNS_ACTIVE_PEDOM;
    }
    
    return ret;
}

int32_t accsns_io_poll_vehicle(struct file *fp, poll_table *wait)
{
    int32_t iWakeupSensor;
    int32_t ret = 0;

    poll_wait(fp, &s_tPollWaitVehicle, wait);

    iWakeupSensor = atomic_read(&g_WakeupSensor);
    if ((iWakeupSensor & ACCSNS_ACTIVE_VEHICLE_ERROR) == ACCSNS_ACTIVE_VEHICLE_ERROR) {
        atomic_set(&g_WakeupSensor, (iWakeupSensor & ~ACCSNS_ACTIVE_VEHICLE_ERROR));
        ret |= ACCSNS_ACTIVE_VEHICLE_ERROR;
    } else if ((iWakeupSensor & ACCSNS_ACTIVE_VEHICLE) == ACCSNS_ACTIVE_VEHICLE) {
        atomic_set(&g_WakeupSensor, (iWakeupSensor & ~ACCSNS_ACTIVE_VEHICLE));
        ret |= ACCSNS_ACTIVE_VEHICLE;
    }
    
    return ret;
}


static int32_t accsns_mov_acc_avg (MovFilterWork* pCtrl, int32_t sample,int32_t axis)
{
DBG_PRINT_IO(0, 0);
    
    if(pCtrl->m_pSamplWork[axis] == NULL) {
        pCtrl->m_pSamplWork[axis] = kzalloc(sizeof(int32_t) * pCtrl->m_ucAveN, GFP_KERNEL);
        
        if(pCtrl->m_pSamplWork[axis] == NULL) {
DBG(DBG_LV_ERROR, "ACC-Calib:Memory crisis\n");
            return ACCSNS_RC_ERR;
        }
        memset(pCtrl->m_pSamplWork[axis], 0x00, sizeof(int32_t) * pCtrl->m_ucAveN);
    }

    if(pCtrl->m_unCnt[axis]++ < pCtrl->m_ucAveN) {
        pCtrl->m_pSamplWork[axis][(pCtrl->m_unCnt[axis]-1) % pCtrl->m_ucAveN] = sample;
        pCtrl->m_unSum[axis] += sample;
        
#ifdef CONFIG_ML610Q792_DEBUG
{
    int32_t i = 0;
    if( DBG_LV_DATA & dbg_level ){
        printk("ACC-Calib:List %d:",pCtrl->m_unCnt[axis]);
        for (i = 0; i < pCtrl->m_ucAveN; i++) {
            printk("%d,",pCtrl->m_pSamplWork[axis][i]);
        }
        printk("\n");
    }
}
#endif
        return ACCSNS_RC_ERR;

    } else {
        pCtrl->m_unSum[axis] -= pCtrl->m_pSamplWork[axis][(pCtrl->m_unCnt[axis]-1) % pCtrl->m_ucAveN];
        pCtrl->m_unSum[axis] += sample;
        pCtrl->m_pSamplWork[axis][(pCtrl->m_unCnt[axis]-1) % pCtrl->m_ucAveN] = sample;
    }

#ifdef CONFIG_ML610Q792_DEBUG
{
    int32_t i = 0;
    printk("List %d:",pCtrl->m_unCnt[axis]);
    for (i = 0; i < pCtrl->m_ucAveN; i++) {
        printk("%d,",pCtrl->m_pSamplWork[axis][i]);
    }
    printk("\n");
}
#endif
  
DBG_PRINT_IO(0xFF, (pCtrl->m_unSum[axis] / pCtrl->m_ucAveN));
    return pCtrl->m_unSum[axis] / pCtrl->m_ucAveN;
}

static void accsns_calibration_periodic(const struct acceleration* accData)
{
    int32_t ret = 0;
DBG_PRINT_IO(0, 0);
    
    mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
DBG(DBG_LV_INFO, "calibPeriodic:%d\n",s_tCalibCtrl.m_nCurrentSampleNum);
    
    if(s_tCalibCtrl.m_nCurrentSampleNum == s_tCalibCtrl.m_unSmpN) {
        s_tCalibCtrl.m_bComplete    = true;
        s_tCalibCtrl.m_bWaitSetting = true;
        
    } else {
        if(s_tCalibCtrl.m_bFilterEnable == true) {
            ret = accsns_mov_acc_avg(&(s_tCalibCtrl.m_tFilterWork), accData->nX, AXIS_X);
            if(ret != ACCSNS_RC_ERR) {
                s_tCalibCtrl.m_nCurrentSampleNum++;
                s_tCalibCtrl.m_nSummationX += ret;
            }
            
            ret = accsns_mov_acc_avg(&(s_tCalibCtrl.m_tFilterWork), accData->nY, AXIS_Y);
            if(ret != ACCSNS_RC_ERR) {
                s_tCalibCtrl.m_nSummationY += ret;
            }
            
            ret = accsns_mov_acc_avg(&(s_tCalibCtrl.m_tFilterWork), accData->nZ, AXIS_Z);
            if(ret != ACCSNS_RC_ERR) {
                s_tCalibCtrl.m_nSummationZ += ret;
            }
            
        } else {
            s_tCalibCtrl.m_nSummationX += accData->nX;
            s_tCalibCtrl.m_nSummationY += accData->nY;
            s_tCalibCtrl.m_nSummationZ += accData->nZ;
            
            s_tCalibCtrl.m_nCurrentSampleNum++;
        }
        
        if(s_tCalibCtrl.m_nCurrentSampleNum == s_tCalibCtrl.m_unSmpN) {
            switch (s_tCalibCtrl.m_nMode) {
                case MODE_0:
                    s_tCalibCtrl.m_nCalX = (s_tCalibCtrl.m_nSummationX / s_tCalibCtrl.m_unSmpN) - SETTING_0G;
                    s_tCalibCtrl.m_nCalY = (s_tCalibCtrl.m_nSummationY / s_tCalibCtrl.m_unSmpN) - SETTING_0G;
                    s_tCalibCtrl.m_nCalZ = 0;
                    break;
                
                case MODE_1:
                    s_tCalibCtrl.m_nCalX = (s_tCalibCtrl.m_nSummationX / s_tCalibCtrl.m_unSmpN) - SETTING_0G;
                    s_tCalibCtrl.m_nCalY = (s_tCalibCtrl.m_nSummationY / s_tCalibCtrl.m_unSmpN) - SETTING_0G;
                    s_tCalibCtrl.m_nCalZ = (s_tCalibCtrl.m_nSummationZ / s_tCalibCtrl.m_unSmpN) - WEIGHT_1G;
                    break;
                
                case MODE_2:
                    s_tCalibCtrl.m_nCalX = (s_tCalibCtrl.m_nSummationX / s_tCalibCtrl.m_unSmpN) - SETTING_0G;
                    s_tCalibCtrl.m_nCalY = (s_tCalibCtrl.m_nSummationY / s_tCalibCtrl.m_unSmpN) - SETTING_0G;
                    s_tCalibCtrl.m_nCalZ = (s_tCalibCtrl.m_nSummationZ / s_tCalibCtrl.m_unSmpN) + WEIGHT_1G;
                    break;

                case MODE_3:
                    s_tCalibCtrl.m_nCalX = (s_tCalibCtrl.m_nSummationX / s_tCalibCtrl.m_unSmpN);
                    s_tCalibCtrl.m_nCalY = (s_tCalibCtrl.m_nSummationY / s_tCalibCtrl.m_unSmpN);
                    s_tCalibCtrl.m_nCalZ = (s_tCalibCtrl.m_nSummationZ / s_tCalibCtrl.m_unSmpN);
                    break;
                
                case MODE_4:
                    s_tCalibCtrl.m_nCalX = (s_tCalibCtrl.m_nCalX + (s_tCalibCtrl.m_nSummationX / s_tCalibCtrl.m_unSmpN)) / 2;
                    s_tCalibCtrl.m_nCalY = (s_tCalibCtrl.m_nCalY + (s_tCalibCtrl.m_nSummationY / s_tCalibCtrl.m_unSmpN)) / 2;
                    s_tCalibCtrl.m_nCalZ = (s_tCalibCtrl.m_nCalZ + (s_tCalibCtrl.m_nSummationZ / s_tCalibCtrl.m_unSmpN)) / 2; 
                    break;
                
                default:
                    ret = ACCSNS_RC_ERR;
DBG(DBG_LV_ERROR, "ACC-Calib: Mode Err!!\n");
                    break;
            }

            if(ret != ACCSNS_RC_ERR) {
                atomic_set(&g_nCalX, s_tCalibCtrl.m_nCalX);
                atomic_set(&g_nCalY, s_tCalibCtrl.m_nCalY);
                atomic_set(&g_nCalZ, s_tCalibCtrl.m_nCalZ);
DBG(DBG_LV_DATA, "AccCalib:mode[%d] complete calX = %d sumX = %u smp=%u\n",s_tCalibCtrl.m_nMode,  s_tCalibCtrl.m_nCalX, s_tCalibCtrl.m_nSummationX, s_tCalibCtrl.m_unSmpN);
DBG(DBG_LV_DATA, "AccCalib:mode[%d] complete calY = %d sumY = %u smp=%u\n",s_tCalibCtrl.m_nMode, s_tCalibCtrl.m_nCalY, s_tCalibCtrl.m_nSummationY, s_tCalibCtrl.m_unSmpN);
DBG(DBG_LV_DATA, "AccCalib:mode[%d] complete calZ = %d sumZ = %u smp=%u\n",s_tCalibCtrl.m_nMode, s_tCalibCtrl.m_nCalZ, s_tCalibCtrl.m_nSummationZ, s_tCalibCtrl.m_unSmpN);
            }

            kfree(s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_X]);
            kfree(s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_Y]);
            kfree(s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_Z]);
            s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_X] = NULL;
            s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_Y] = NULL;
            s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_Z] = NULL;
        }
    }
    mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));
    
DBG_PRINT_IO(0xFF, 0);
}

static int32_t accsns_activateEx(int32_t arg_iUpdate, int32_t arg_iSensType, int32_t arg_iEnable)
{
    bool bIsChanged = false;
    int32_t ret  = ACCSNS_RC_OK; 
    int32_t iCurrentEnable;
    int32_t iflgEna;
    int32_t pause_sens = atomic_read(&g_pause_sens);
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iUpdate %x, arg_iSensType %x, arg_iEnable %x\n",arg_iUpdate, arg_iSensType, arg_iEnable);

    if(arg_iSensType == ACTIVE_OFF){
        return ret;
    }
    
    if(arg_iEnable != 0) {
        arg_iEnable = POWER_ENABLE;
    }
    
    iCurrentEnable = atomic_read(&g_CurrentSensorEnable);
DBG(DBG_LV_INFO, "CurrentEnable check.(%x) <pause_sens=%x>\n",iCurrentEnable, pause_sens);
    iflgEna = iCurrentEnable;
    
    if(arg_iEnable) {
        if(!(iCurrentEnable & arg_iSensType)){
            if(!(iCurrentEnable & ACTIVE_ON)){
                ret = accsns_power_onoff(POWER_ENABLE);
                if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "Error ret %x\n", ret);
                    return ret;
                }
                iCurrentEnable |= ACTIVE_ON;
            }
            
            if(arg_iSensType & ACCSNS_ACTIVE_ACC){
                ret |= accsns_acc_activate(POWER_ENABLE);
            }
            if(arg_iSensType & ACCSNS_ACTIVE_PEDOM){
                if(!pause_sens) {
                    ret |= accsns_pedom_activate(POWER_ENABLE);
                }
            }
            if(arg_iSensType & ACCSNS_ACTIVE_VEHICLE){
                if(!pause_sens) {
                    ret |= accsns_vehicle_activate(POWER_ENABLE);
                }
            }

            if(arg_iSensType & ACCSNS_ACTIVE_MOVE){
                ret |= accsns_move_activate(POWER_ENABLE);
            }

            iCurrentEnable |= arg_iSensType;
            iflgEna         = (iCurrentEnable | pause_sens);
            
            bIsChanged = true;
DBG(DBG_LV_INFO, "[Enable] All OK[%d]\n", ret);
        }
        
    }else{
        if((iCurrentEnable | pause_sens) & arg_iSensType){

            if(arg_iSensType & ACCSNS_ACTIVE_ACC){
                ret |= accsns_acc_activate(POWER_DISABLE);
            }
            if((arg_iSensType & iCurrentEnable) & ACCSNS_ACTIVE_PEDOM){
                ret |= accsns_pedom_activate(POWER_DISABLE);
            }
            if((arg_iSensType & iCurrentEnable) & ACCSNS_ACTIVE_VEHICLE){
                ret |= accsns_vehicle_activate(POWER_DISABLE);
            }
            if(arg_iSensType & ACCSNS_ACTIVE_MOVE){
                ret |= accsns_move_activate(POWER_DISABLE);
            }
            
            iCurrentEnable &= ~arg_iSensType;
            
            if(dummy_on_flg == false) {
                pause_sens &= ~arg_iSensType;
                atomic_set(&g_pause_sens, pause_sens);
DBG(DBG_LV_INFO, "PauseFlg update.(%x)\n",pause_sens);

                if( (iCurrentEnable & ACTIVE_ON) && (!((iCurrentEnable | pause_sens) & ACTIVE_FUNC_MASK)) ){
                    ret |= accsns_power_onoff(POWER_DISABLE);
                    
                    iCurrentEnable = ACTIVE_OFF;
                    
                    hrtimer_cancel_timer_func(&ped_timer);
                }
            }
            
            iflgEna = (iCurrentEnable | pause_sens);

            bIsChanged = true;
DBG(DBG_LV_INFO, "[Disable] All OK[%d]\n", ret);
        }
    }
    
    if(bIsChanged) {
        atomic_set(&g_CurrentSensorEnable,iCurrentEnable);
DBG(DBG_LV_INFO, "CurrentEnable update.(%x)\n",iCurrentEnable);
    }
    
    if(arg_iUpdate) {
        atomic_set(&g_flgEna, iflgEna);
DBG(DBG_LV_INFO, "FlgEnable update.(%x)\n",iflgEna);
    }
    
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

static int32_t accsns_power_onoff(bool arg_iEnable)
{
    int32_t   ret;
    HostCmd cmd;
    HostCmdRes res;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iEnable %x\n", arg_iEnable);
    
    if(arg_iEnable == true){
        cmd.cmd.udata16 = HC_ACC_SET_AUTO_MEASURE;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x00;
        cmd.prm.ub_prm[2] = 0x1E;
        cmd.prm.ub_prm[3] = 0x00;
        cmd.prm.ub_prm[4] = 0x00;
        cmd.prm.ub_prm[5] = 0x04;
        cmd.prm.ub_prm[6] = 0x00;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
        if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power On Error HC_ACC_SET_AUTO_MEASURE(-) err %x\n", res.err.udata16);
            return ACCSNS_RC_ERR;
        }
        
        cmd.cmd.udata16 = HC_MUL_SET_ANDROID;
        cmd.prm.uw_prm[0] = HC_VALID;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
        if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power On Error HC_MUL_SET_ANDROID(HC_VALID) err %x\n", res.err.udata16);
            return ACCSNS_RC_ERR;
        }
        
        cmd.cmd.udata16 = HC_MUL_MEASURE;
        cmd.prm.ub_prm[0] = MT_ANDROID_START;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
        if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power On Error HC_MUL_MEASURE(MT_ANDROID_START) err %x\n", res.err.udata16);
            return ACCSNS_RC_ERR;
        }
    
    }else{
        cmd.cmd.udata16 = HC_MUL_MEASURE;
        cmd.prm.ub_prm[0] = MT_ANDROID_STOP;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
        if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Power Off Error HC_MUL_MEASURE(MT_ANDROID_STOP) err %x\n", res.err.udata16);
            return ACCSNS_RC_ERR;
        }
    }
    
DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t accsns_acc_activate(bool arg_iEnable)
{
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iEnable %x\n", arg_iEnable);

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
  return ACCSNS_RC_OK;
}

static int32_t accsns_acc_getdata(struct acceleration *arg_Acc)
{
    uint8_t ucBuff[AXIS_XYZ_MAX * 2];
    int32_t ret;
    int32_t temp;
    
    ret = accsns_spi_read(RSLT0E, ucBuff, sizeof(ucBuff));
    if(ACCSNS_RC_OK == ret){
    
        temp = (int32_t)ucBuff[0] | (((int32_t)ucBuff[1] & 0x0F) << 8);
        arg_Acc->nX = ACCDATA_SIGN_COMVERT_12_32BIT(temp);
        
        temp = (int32_t)ucBuff[2] | (((int32_t)ucBuff[3] & 0x0F) << 8);
        arg_Acc->nY = ACCDATA_SIGN_COMVERT_12_32BIT(temp);
        
        temp = (int32_t)ucBuff[4] | (((int32_t)ucBuff[5] & 0x0F) << 8);
        arg_Acc->nZ = ACCDATA_SIGN_COMVERT_12_32BIT(temp);

DBG(DBG_LV_LOW, "reg     x:%02x %02x, y:%02x %02x, z:%02x %02x\n",ucBuff[1],ucBuff[0],ucBuff[3],ucBuff[2],ucBuff[5],ucBuff[4]);
DBG(DBG_LV_LOW, "arg_Acc x:%04x(%d), y:%04x(%d), z:%04x(%d)\n", arg_Acc->nX, arg_Acc->nX, arg_Acc->nY, arg_Acc->nY, arg_Acc->nZ, arg_Acc->nZ);
    }
    
    return ret;
}

static int32_t accsns_acc_setcalib(void)
{
DBG_PRINT_IO(0, 0);

    return 0;
}

static void accsns_acc_work_func(struct work_struct *work)
{
    int32_t ret = ACCSNS_RC_ERR;
    int32_t delay = 0;
    bool bCalibIdle = false;
    bool bCalibComp = false;
DBG_PRINT_IO(0, 0);
    
    ret = accsns_measure(ACCSNS_ACTIVE_ACC);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "accsns_measure Error err %x\n", ret);
        return;
    }
    
    mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
    bCalibIdle = s_tCalibCtrl.m_bWaitSetting;
    bCalibComp = s_tCalibCtrl.m_bComplete;
    mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));
    
    if((bCalibIdle == false) && (bCalibComp == false)){
    
        mutex_lock(&s_tDataMutex);
        accsns_calibration_periodic(&s_tLatestAccData);
        mutex_unlock(&s_tDataMutex);
        
        mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
        bCalibComp = s_tCalibCtrl.m_bComplete;
        mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));
        if(bCalibComp == false){
            delay = atomic_read(&s_nDelayTime);
            schedule_delayed_work(&s_tDelayWork_Acc,msecs_to_jiffies(delay));
        }
    }
    
DBG(DBG_LV_INFO, "accsns_acc_work_func:Compleate ret=%x, bCalibIdle=%x, delay=%x\n", ret, bCalibIdle, delay);
DBG_PRINT_IO(0xFF, 0);
}

static int32_t accsns_pedom_activate(bool arg_iEnable)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_OK;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iEnable %x\n", arg_iEnable);
    
    cmd.cmd.udata16 = HC_ACC_SET_PEDO;
    
    if(arg_iEnable == true){
        cmd.prm.ub_prm[0] = HC_VALID;

    }else{
        cmd.prm.ub_prm[0] = HC_INVALID;
    }
    
    cmd.prm.ub_prm[1] = atomic_read(&g_nStepWide);
    cmd.prm.ub_prm[2] = atomic_read(&g_nWeight);
    cmd.prm.ub_prm[3] = g_acc_nv_param.pedo_p.notify;
    cmd.prm.ub_prm[4] = g_acc_nv_param.pedo_p.speed_ave_time;
    cmd.prm.ub_prm[5] = g_acc_nv_param.pedo_p.mets_stop_time;
    cmd.prm.ub_prm[6] = g_acc_nv_param.pedo_p.bodyfat_on;
    cmd.prm.ub_prm[7] = g_acc_nv_param.pedo_p.bodyfat_cal;
    
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Pedometer Activate Ctrl Error HC_ACC_SET_PEDO(%d) err %x\n",cmd.prm.ub_prm[0], res.err.udata16);
        return ACCSNS_RC_ERR;
    }

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t accsns_pedom_getdata(struct pedometer *arg_Pedo)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_OK;
DBG_PRINT_IO(0, 0);
    
    cmd.cmd.udata16   = HC_ACC_PEDO_CNT;
    cmd.prm.ub_prm[0] = HC_ACC_PEDO_STABLE;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usStepCnt  = res.res.ud_res[0];
        arg_Pedo->usWalkTime = res.res.ud_res[1];
        arg_Pedo->usCal      = res.res.uw_res[4];

    }else{
DBG(DBG_LV_ERROR, "Pedometer getdata Error HC_ACC_PEDO_CNT(HC_ACC_PEDO_STABLE) err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    
    cmd.prm.ub_prm[0] = HC_ACC_PEDO_ADD;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usBodyFat  = res.res.ud_res[0] / 10;
        arg_Pedo->usSpeed    = res.res.uw_res[2];
        arg_Pedo->usMets     = res.res.ub_res[6];

    }else{
DBG(DBG_LV_ERROR, "Pedometer getdata Error HC_ACC_PEDO_CNT(HC_ACC_PEDO_ADD) err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    if(g_nAccFWVersion >= ACC_FW_VERSION_CYCLE){
        cmd.prm.ub_prm[0] = HC_ACC_PEDO_CYCLE;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
        if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
            arg_Pedo->usCycle_cal  = res.res.uw_res[0];
            arg_Pedo->usCycle_time = ((res.res.uw_res[2] << 16) | res.res.uw_res[1]);
        }else{
DBG(DBG_LV_ERROR, "Pedometer getdata Error HC_ACC_PEDO_CNT(HC_ACC_PEDO_CYCLE) err %x\n", res.err.udata16);
            return ACCSNS_RC_ERR;
        }
    }else{
        arg_Pedo->usCycle_cal  = 0;
        arg_Pedo->usCycle_time = 0;
DBG(DBG_LV_INFO, "Pedometer getdata g_nAccFWVersion %x\n", g_nAccFWVersion);
    }

DBG(DBG_LV_INFO, "[Stability Step information]/[Additive information] ret %x\n", ret);
DBG(DBG_LV_DATA, " Step Count         : %u\n", arg_Pedo->usStepCnt );
DBG(DBG_LV_DATA, " Walking Time(s)    : %u\n", arg_Pedo->usWalkTime );
DBG(DBG_LV_DATA, " Calorie(kcal)      : %u\n", arg_Pedo->usCal );
DBG(DBG_LV_DATA, " Body Fat(g)        : %u\n", arg_Pedo->usBodyFat );
DBG(DBG_LV_DATA, " Speedometer(m/min) : %u\n", arg_Pedo->usSpeed );
DBG(DBG_LV_DATA, " Mets               : %d\n", arg_Pedo->usMets );
DBG(DBG_LV_DATA, " Cycle_Calorie(kcal): %u\n", arg_Pedo->usCycle_cal );
DBG(DBG_LV_DATA, " Cycling Time(s)    : %u\n", arg_Pedo->usCycle_time );
    
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

int32_t accsns_pedom_clear(void)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_OK;
DBG_PRINT_IO(0, 0);
    
    cmd.cmd.udata16 = HC_ACC_PEDO_CLEAR;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Pedometer Data Clear Errorerr %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

static int32_t accsns_pedom_set_timer(uint32_t arg_iEnable, uint32_t arg_iCount)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t iCurrentEnable;
    int32_t ret = ACCSNS_RC_ERR;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iTimer %x, arg_iCount %x\n", arg_iEnable, arg_iCount);
    
    iCurrentEnable = atomic_read(&g_CurrentSensorEnable);
    if( !(iCurrentEnable & ACCSNS_ACTIVE_PEDOM) ){
DBG(DBG_LV_INFO, "accsns_pedom_set_timer:Status Error[%x]\n", iCurrentEnable);
        return ACCSNS_RC_OK;
    }
    
    cmd.cmd.udata16   = HC_ACC_SET_PEDO_TIMER;
    if(arg_iEnable == 1)
    {
        cmd.prm.ub_prm[0] = HC_VALID;
    }else{
        cmd.prm.ub_prm[0] = HC_INVALID;
    }
    cmd.prm.ub_prm[1] = (uint8_t)((arg_iCount >>  0) & 0xFF);
    cmd.prm.ub_prm[2] = (uint8_t)((arg_iCount >>  8) & 0xFF);
    cmd.prm.ub_prm[3] = (uint8_t)((arg_iCount >> 16) & 0xFF);
    cmd.prm.ub_prm[4] = (uint8_t)((arg_iCount >> 24) & 0xFF);
    
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "accsns_initialize:CMD Error <HC_MCU_SET_PCON> err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }

DBG_PRINT_IO(0xFF, 0);
    return ACCSNS_RC_OK;
}

int32_t accsns_pedom_start_timer(uint32_t arg_iEnable)
{
    int32_t ret;
    uint32_t time_cnt;
    uint32_t iEnable = arg_iEnable;
    
DBG(DBG_LV_INFO, "accsns_pedom_start_timer\n");
    time_cnt = atomic_read(&g_nPedoTimerCount);
    if (time_cnt == 0)
    {
        iEnable = 0;
    }

    if ((g_bPedoTimerStatus == true) && (iEnable == true)) {
        ret = accsns_pedom_set_timer(0, 0);
        if (ret) {
DBG(DBG_LV_ERROR, "accsns_pedom_start_timer:accsns_pedom_set_timer(stop) Error %x\n", ret);
            return ret;
        }
        g_bPedoTimerStatus = false;
    }
    
    if (iEnable != g_bPedoTimerStatus) {
        ret = accsns_pedom_set_timer(iEnable, time_cnt);
        if (ret) {
DBG(DBG_LV_ERROR, "accsns_pedom_start_timer:accsns_pedom_set_timer Error %x\n", ret);
            return ret;
        }
    }

    g_bPedoTimerStatus = iEnable;

    return ACCSNS_RC_OK;
}

static int32_t accsns_vehicle_activate(bool arg_iEnable)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_ERR;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iEnable %x\n", arg_iEnable);
    
    cmd.cmd.udata16 = HC_ACC_SET_TRANS;
    
    if(arg_iEnable == true){
        cmd.prm.ub_prm[0] = HC_VALID;
    }else{
        cmd.prm.ub_prm[0] = HC_INVALID;
    }
    
    cmd.prm.ub_prm[1] = 20;
    cmd.prm.ub_prm[2] = (g_acc_nv_param.trans_p.judge_time & 0xFF);
    cmd.prm.ub_prm[3] = ((g_acc_nv_param.trans_p.judge_time >> 8) & 0xFF);
    cmd.prm.ub_prm[4] = g_acc_nv_param.trans_p.calc_time;
    cmd.prm.ub_prm[5] = g_acc_nv_param.trans_p.consecutive_num;
    cmd.prm.ub_prm[6] = 0x00;
    cmd.prm.ub_prm[7] = 0x00;
    
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "vehicle Activate Error HC_ACC_SET_PEDO(-) err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }

    cmd.cmd.udata16 = 0x1054;

    cmd.prm.ub_prm[0] = 0xB8;
    cmd.prm.ub_prm[1] = 0x88;
    cmd.prm.ub_prm[2] = 0xAC;
    cmd.prm.ub_prm[3] = 0x0D;
    cmd.prm.ub_prm[4] = 0x19;
    cmd.prm.ub_prm[5] = 0x0E;
    cmd.prm.ub_prm[6] = 0x1E;
    cmd.prm.ub_prm[7] = 0x00;
    cmd.prm.ub_prm[8] = 0xF6;
    cmd.prm.ub_prm[9] = 0xFF;
    cmd.prm.ub_prm[0x0a] = 0x0F;
    cmd.prm.ub_prm[0x0b] = 0x04;

    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "vehicle Activate Ctrl Error CMD:0x1054(%d) err %x\n",cmd.prm.ub_prm[0], res.err.udata16);
        return ACCSNS_RC_ERR;
    }

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t accsns_vehicle_getdata(struct vehicle *arg_Vehi)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_OK;
DBG_PRINT_IO(0, 0);
    
    cmd.cmd.udata16 = HC_ACC_TRANS_INFO;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK == ret) && (0 == res.err.udata16)) {
#if 1
        if(0 == res.res.ub_res[0]){
            arg_Vehi->usVehiStatus = 0;

        }else if( (1 == res.res.ub_res[0]) || (10 == res.res.ub_res[0]) ){
            arg_Vehi->usVehiStatus = 1;
#endif

        }else{
DBG(DBG_LV_ERROR, "Pedometer vehicle Status[%d]\r\n",res.res.ub_res[0]);
        }
        if((0 == res.res.ub_res[1]) || (1 == res.res.ub_res[1])) {
            arg_Vehi->usVehiKind = res.res.ub_res[1];
            
        }else{
DBG(DBG_LV_ERROR, "Pedometer vehicle Kind[%d]\r\n",res.res.ub_res[1]);
        }
DBG(DBG_LV_INFO, "vehicle getdata H/W : %x, %x \n", res.res.ub_res[0], res.res.ub_res[1]);            

    }else{
DBG(DBG_LV_ERROR, "vehicle getdata Error HC_ACC_TRANS_INFO(-) err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }

    
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

int32_t accsns_vehicle_clear(void)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_OK;
DBG_PRINT_IO(0, 0);
    
    cmd.cmd.udata16 = HC_ACC_TRANS_CLEAR;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "vehicle Data Clear Errorerr %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

static int32_t accsns_move_activate(bool arg_iEnable)
{
    int32_t ret = ACCSNS_RC_ERR;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iEnable %x\n", arg_iEnable);
    
    if(arg_iEnable == true){
        ret = accsns_move_detect(HC_VALID);
        if(ACCSNS_RC_OK != ret) {
DBG(DBG_LV_ERROR, "moving detect Activate Error err\n");
            return ACCSNS_RC_ERR;
        }
        atomic_set(&g_nMoveDetectStatus, DETECT_STOP);

    }else{
        accsns_move_detect(HC_INVALID);
        accsns_stop_detect(HC_INVALID);
        
        atomic_set(&g_nMoveDetectStatus, DETECT_NONE);
    }
    
DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t accsns_move_getdata(struct move *arg_Move)
{
DBG_PRINT_IO(0, 0);
    
    arg_Move->usMoveStatus = atomic_read(&g_nMoveDetectStatus);

DBG(DBG_LV_DATA, "arg_Move :%x\n", arg_Move->usMoveStatus);
    
DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t accsns_move_detect(bool arg_iEnable)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_ERR;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iEnable %x\n", arg_iEnable);
    
    cmd.cmd.udata16 = HC_ACC_SET_MOVE_DETECT;
    
    if(arg_iEnable == HC_VALID){
        cmd.prm.ub_prm[0] = HC_VALID;

    }else{
        cmd.prm.ub_prm[0] = HC_INVALID;
    }
    
    cmd.prm.ub_prm[1] = g_acc_nv_param.move_p.axis;
    cmd.prm.ub_prm[2] = (g_acc_nv_param.move_p.acc_th & 0xFF);
    cmd.prm.ub_prm[3] = ((g_acc_nv_param.move_p.acc_th >> 8) & 0xFF);
    cmd.prm.ub_prm[4] = g_acc_nv_param.move_p.judge_num;
    cmd.prm.ub_prm[5] = g_acc_nv_param.move_p.judge_th;
    cmd.prm.ub_prm[6] = g_acc_nv_param.move_p.judge;
    
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "moving detect Activate Error HC_ACC_SET_MOVE_DETECT(-) err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }

    DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t accsns_stop_detect(bool arg_iEnable)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_ERR;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "arg_iEnable %x\n", arg_iEnable);
    
    cmd.cmd.udata16 = HC_ACC_SET_DIST_STOP;
    
    if(arg_iEnable == HC_VALID){
        cmd.prm.ub_prm[0] = HC_INVALID;
        cmd.prm.ub_prm[2] = HC_VALID;

    }else{
        cmd.prm.ub_prm[0] = HC_INVALID;
        cmd.prm.ub_prm[2] = HC_INVALID;
    }
    
    cmd.prm.ub_prm[1] = 40;
    cmd.prm.ub_prm[3] = (g_acc_nv_param.dist_stop_p.stop_notify_time & 0xFF);
    cmd.prm.ub_prm[4] = ((g_acc_nv_param.dist_stop_p.stop_notify_time >> 8) & 0xFF);
    
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "moving detect Activate Error HC_ACC_SET_DIST_STOP(-) err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t accsns_measure(int32_t arg_iSensType)
{
    int32_t  ret = ACCSNS_RC_OK;
    struct pedometer     arg_Pedo;
    struct acceleration  arg_Acc;
    struct vehicle       arg_Vehi;
    struct move          arg_Move;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_LOW, "     Enable = %04x, IrqFlg = %04x GPIO_INT = %04x \n", (int)atomic_read(&g_bIsIntIrqEnable), g_nIntIrqFlg, gpio_get_value(ACCSNS_GPIO_INT));
DBG(DBG_LV_INFO, "arg_iSensType %x\n", arg_iSensType);
    
    memset(&arg_Acc,  0x00, sizeof(arg_Acc));
    memset(&arg_Pedo, 0x00, sizeof(arg_Pedo));
    memset(&arg_Vehi, 0x00, sizeof(arg_Vehi));
    memset(&arg_Move, 0x00, sizeof(arg_Move));
    
    if(arg_iSensType & ACCSNS_ACTIVE_ACC){

        ret = accsns_acc_getdata(&arg_Acc);
        mutex_lock(&s_tDataMutex);
        if(ret == ACCSNS_RC_OK){
            s_tLatestAccData.nX = arg_Acc.nX;
            s_tLatestAccData.nY = arg_Acc.nY;
            s_tLatestAccData.nZ = arg_Acc.nZ;
DBG(DBG_LV_DATA, "Acc Data get . X %x, Y %x, Z %x\n",s_tLatestAccData.nX, s_tLatestAccData.nY, s_tLatestAccData.nZ);
        }else{
            memset(&s_tLatestAccData, 0x00, sizeof(s_tLatestAccData));
        }
        mutex_unlock(&s_tDataMutex);
    }

    if(arg_iSensType & ACCSNS_ACTIVE_PEDOM){
    
        ret = accsns_pedom_getdata(&arg_Pedo);
        mutex_lock(&s_tDataMutex);
        if(ret == ACCSNS_RC_OK){
            s_tLatestPedoData.usStepCnt  = arg_Pedo.usStepCnt;
            s_tLatestPedoData.usWalkTime = arg_Pedo.usWalkTime;
            s_tLatestPedoData.usCal      = arg_Pedo.usCal;
            s_tLatestPedoData.usBodyFat  = arg_Pedo.usBodyFat;
            s_tLatestPedoData.usSpeed    = arg_Pedo.usSpeed;
            s_tLatestPedoData.usMets     = arg_Pedo.usMets;
            s_tLatestPedoData.usCycle_cal  = arg_Pedo.usCycle_cal;
            s_tLatestPedoData.usCycle_time = arg_Pedo.usCycle_time;

DBG(DBG_LV_DATA, "Pedo Data get . usStepCnt %x, usWalkTime %x, usCal %x, usBodyFat %x, usSpeed %x, usMets %x, usCycle_cal %x, usCycle_time %x\n"
                ,s_tLatestPedoData.usStepCnt, s_tLatestPedoData.usWalkTime, s_tLatestPedoData.usCal, s_tLatestPedoData.usBodyFat
                ,s_tLatestPedoData.usSpeed, s_tLatestPedoData.usMets, s_tLatestPedoData.usCycle_cal, s_tLatestPedoData.usCycle_time);
        }else{
            memset(&s_tLatestPedoData, 0x00, sizeof(s_tLatestPedoData));
        }
        mutex_unlock(&s_tDataMutex);
    }

    if(arg_iSensType & ACCSNS_ACTIVE_VEHICLE){
    
        ret = accsns_vehicle_getdata(&arg_Vehi);
        mutex_lock(&s_tDataMutex);
        if(ret == ACCSNS_RC_OK){
            s_tLatestVehiData.usVehiStatus = arg_Vehi.usVehiStatus;
            s_tLatestVehiData.usVehiKind   = arg_Vehi.usVehiKind;
DBG(DBG_LV_DATA, "vehicle Data get . Status %x %x\n", s_tLatestVehiData.usVehiStatus, s_tLatestVehiData.usVehiKind);
        }else{
            memset(&s_tLatestVehiData, 0x00, sizeof(s_tLatestVehiData));
        }
        mutex_unlock(&s_tDataMutex);
    }

    if(arg_iSensType & ACCSNS_ACTIVE_MOVE){

        ret = accsns_move_getdata(&arg_Move);
        mutex_lock(&s_tDataMutex);
        if(ret == ACCSNS_RC_OK){
            s_tLatestMoveData.usMoveStatus = arg_Move.usMoveStatus;
DBG(DBG_LV_DATA, "moving Data get . Status %x\n", arg_Move.usMoveStatus);
        }else{
            memset(&s_tLatestMoveData, 0x00, sizeof(s_tLatestMoveData));
        }
        mutex_unlock(&s_tDataMutex);
    }
  
DBG_PRINT_IO(0xFF, ret);
DBG(DBG_LV_LOW, "     Enable = %04x, IrqFlg = %04x GPIO_INT = %04x \n", (int)atomic_read(&g_bIsIntIrqEnable), g_nIntIrqFlg, gpio_get_value(ACCSNS_GPIO_INT));
  return ret;
}

int32_t accsns_initialize( void )
{
    uint8_t fw_ver[4];
    uint8_t reg = 0xFF;
    int32_t cnt;
    int32_t ret = ACCSNS_RC_OK;
    int32_t iWakeupSensor;
    HostCmd cmd;
    HostCmdRes res;
    Word sreg;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "accsns_initialize:register_init\n");

    DISABLE_IRQ;
    
    accsns_workqueue_init();

    atomic_set(&g_flgEna, ACTIVE_OFF);
    atomic_set(&g_CurrentSensorEnable,ACTIVE_OFF);
    atomic_set(&g_pause_sens, 0);
    iWakeupSensor = atomic_read(&g_WakeupSensor);
    atomic_set(&g_WakeupSensor,iWakeupSensor & ~ACTIVE_FUNC_MASK);
    g_bSpiError = false;
    g_bPedoTimerStatus = false;
    mutex_lock(&s_tDataMutex);
    
    memset(&s_tLatestAccData, 0x00, sizeof(s_tLatestAccData));
    memset(&s_tLatestPedoData, 0x00, sizeof(s_tLatestPedoData));
    memset(&s_tLatestVehiData, 0x00, sizeof(s_tLatestVehiData));
    memset(&s_tLatestMoveData, 0x00, sizeof(s_tLatestMoveData));
    mutex_unlock(&s_tDataMutex);
    
    gpio_set_value(ACCSNS_GPIO_RST, 0);
    udelay(300);
    gpio_set_value(ACCSNS_GPIO_RST, 1);

    msleep(1000);
    
    cnt = 0;
    while(1) {
        accsns_spi_read(STATUS, &reg, sizeof(reg));
        if(reg == 0x00) {
DBG(DBG_LV_INFO, "STATUS OK!!\n");
            break;
        }
        
        msleep(10);
        ++cnt;
        if(cnt > 100) {
DBG(DBG_LV_ERROR, "[ACC] accsns_initialize:STATUS read TimeOut. reg=%x \n", reg);
            return ACCSNS_RC_ERR_TIMEOUT;
        }
    }
    
    reg = 0x04;
    accsns_spi_write(CFG, &reg, sizeof(reg));

    accsns_spi_read(INTREQ0, sreg.udata8, 2);

    reg = 0x00;
    accsns_spi_write(INTMASK0, &reg, sizeof(reg));
    accsns_spi_write(INTMASK1, &reg, sizeof(reg));
    
    ENABLE_IRQ;
    
    cmd.cmd.udata16 = HC_MCU_SET_PCON;
    cmd.prm.ub_prm[0] = 0x00;
    cmd.prm.ub_prm[1] = 0x00;
    cmd.prm.ub_prm[2] = 0x00;
    cmd.prm.ub_prm[3] = 0x00;
    cmd.prm.ub_prm[4] = 0x01;
    cmd.prm.ub_prm[5] = 0x01;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "accsns_initialize:CMD Error <HC_MCU_SET_PCON> err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    
    ret = accsns_get_fw_version(fw_ver);
    g_nAccFWVersion = ACC_FW_VERSION_GET_32BIT(fw_ver);
    if(ret != ACCSNS_RC_OK ){
        g_nAccFWVersion = ACC_FW_VERSION_NONE;
DBG(DBG_LV_ERROR, "[ACC] accsns_get_fw_version:Version not get.\n");
    }
DBG(ACCSNS_RC_ERR, "[ACC] Sensor FW Version.%08x \n", g_nAccFWVersion);

DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t accsns_hostcmd(const HostCmd *prm, HostCmdRes *res, uint8_t mode)
{
    int32_t ret;
    uint8_t reg[16];
    
DBG(DBG_LV_LOW, "AccSem Get\n");
    down(&s_tAccSnsrSema);
    
    reg[0]  = prm->cmd.udata8[0];
    reg[1]  = prm->cmd.udata8[1];
    memcpy(&reg[2], &prm->prm.ub_prm[0], sizeof(prm->prm.ub_prm));
    reg[15] = 1;

DBG(DBG_LV_INFO, "[ACC]<%s>:Start HostCmd :reg[%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x] mode %x\n",__FUNCTION__
    ,reg[0] ,reg[1] ,reg[2] ,reg[3] ,reg[4] ,reg[5] ,reg[6] ,reg[7] ,reg[8] ,reg[9] ,reg[10] 
    ,reg[11] ,reg[12] ,reg[13] ,reg[14] ,reg[15] 
    , mode);
    
    mutex_lock(&s_tDataMutex);
    g_nIntIrqFlg = 0;
    mutex_unlock(&s_tDataMutex);

    ret = accsns_spi_write(CMD0, reg, sizeof(reg));
    if(ret == ACCSNS_RC_OK){
        
        if((mode & EXE_HOST_WAIT) == EXE_HOST_WAIT){
            ret |= accsns_waitcmd(INTREQ_HOST_CMD);
            if(ret != ACCSNS_RC_OK) {
                if( ((mode & EXE_HOST_EX_NO_RECOVER) == EXE_HOST_EX_NO_RECOVER) && 
                    (ret == ACCSNS_RC_ERR_TIMEOUT) ){
DBG(DBG_LV_ERROR, "SPI HostCmd error(accsns_waitcmd):F/W Self Checking...(No Recovery)\n");
                    up(&s_tAccSnsrSema);
DBG(DBG_LV_LOW, "AccSem Post\n");
                    return ACCSNS_RC_ERR_TIMEOUT;
                }
                accsns_timeout_dump(reg);
               
                up(&s_tAccSnsrSema);
DBG(DBG_LV_LOW, "DBG_LV_INFO Post\n");
DBG(DBG_LV_ERROR, "SPI HostCmd error(accsns_waitcmd)\n");
                g_bSpiError = true;
                ERR_WAKEUP;
                
                return ret;
            }
        }

        if((mode & EXE_HOST_RES) == EXE_HOST_RES){
            ret |= accsns_spi_read(RSLT00, res->res.ub_res, 12);
            if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "SPI HostCmd error(RSLT00)\n");
            }
        }

        if((mode & EXE_HOST_ERR) == EXE_HOST_ERR){
            ret |= accsns_spi_read(ERROR0, res->err.udata8, 2);
            if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "SPI HostCmd error(ERROR0)\n");
            }
        }
    }else{
DBG(DBG_LV_ERROR, "SPI HostCmd error(ret %x)", ret);
        g_bSpiError = true;
    }
    
DBG(DBG_LV_SPI, "[ACC]<%s>:End ret=%x\n",__FUNCTION__, ret);
    
    up(&s_tAccSnsrSema);
DBG(DBG_LV_LOW, "AccSem Post\n");
    
    return ret;
}

static int32_t accsns_waitcmd(uint16_t intBit)
{
    int32_t ret = 0;
    long timeout;
DBG_PRINT_IO(0, 0);
    
    timeout = msecs_to_jiffies(WAITEVENT_TIMEOUT);

    while(1){
    
        ret = wait_event_interruptible_timeout(s_tWaitInt, (g_nIntIrqFlg & INTREQ_HOST_CMD), timeout);
        
        if( g_nIntIrqFlg & INTREQ_ERROR ){
            mutex_lock(&s_tDataMutex);
            g_nIntIrqFlg &= ~INTREQ_ERROR;
            mutex_unlock(&s_tDataMutex);
            
DBG(DBG_LV_INT, "INTREQ0/1 -Error- \n");
            ret = ACCSNS_RC_ERR;
            break;

        }else if( g_nIntIrqFlg & INTREQ_HOST_CMD ){
            mutex_lock(&s_tDataMutex);
            g_nIntIrqFlg &= ~INTREQ_HOST_CMD;
            mutex_unlock(&s_tDataMutex);
            
            ret = ACCSNS_RC_OK;
DBG(DBG_LV_INFO, "Wakeup Event... \n");
            break;
        }

        if( ret == 0 ){
            ret = ACCSNS_RC_ERR_TIMEOUT;
DBG(DBG_LV_ERROR, "wait event timeout... %x \n", g_nIntIrqFlg);
            break;
        }
    }

DBG(DBG_LV_SPI, "[ACC]<%s>:End ret=%x\n",__FUNCTION__, ret);
    return ret;
}

static irqreturn_t accsns_irq_handler(int32_t irq, void *dev_id)
{
DBG(DBG_LV_LOW, "### accsns_irq_handler In \n");

    if( irq != g_nIntIrqNo ){
        return IRQ_NONE;
    }

    DISABLE_IRQ;
    if( accsns_workqueue_create(accsns_wq_int, accsns_int_work_func) != ACCSNS_RC_OK){
        ENABLE_IRQ;
        
    }else{
DBG(DBG_LV_INT, "### --> s_tWork_Int \n");
    }

    return IRQ_HANDLED;
}

static void accsns_int_work_func(struct work_struct *work)
{
    Word sreg;
    int32_t iCurrentEnable;
    
    memset( &sreg, 0x00, sizeof(sreg));
    
    iCurrentEnable = (atomic_read(&g_CurrentSensorEnable) | atomic_read(&g_pause_sens));

    accsns_spi_read(INTREQ0, sreg.udata8, 2);

    if(sreg.udata16 == 0){
        accsns_workqueue_delete(work) ;
        ENABLE_IRQ;
        return;
    }
DBG(DBG_LV_INT, "### INTREQ0/1=%x iCurrentEnable=%x \n", sreg.udata16, iCurrentEnable);
    
    if(sreg.udata16 & INTREQ_ERROR){
DBG(DBG_LV_ERROR, "### accsns_int_work_func Error %x\n", sreg.udata16);
        mutex_lock(&s_tDataMutex);
        g_nIntIrqFlg |= INTREQ_ERROR;
        mutex_unlock(&s_tDataMutex);
        
        wake_up_interruptible(&s_tWaitInt);
        
        accsns_workqueue_delete(work) ;
        ENABLE_IRQ;
        return;
    }
    
    if(sreg.udata16 & INTREQ_HOST_CMD){
        if(!(g_nIntIrqFlg & INTREQ_HOST_CMD)){
            mutex_lock(&s_tDataMutex);
            g_nIntIrqFlg |= INTREQ_HOST_CMD;
            mutex_unlock(&s_tDataMutex);
            wake_up_interruptible(&s_tWaitInt);
        }
DBG(DBG_LV_INT, "### accsns_int_work_func INTREQ_HOST_CMD g_nIntIrqFlg:%x \n", g_nIntIrqFlg);
    }
    
    if(sreg.udata16 & INTREQ_ACC){
DBG(DBG_LV_INT, "### accsns_int_work_func INTREQ_ACC iCurrentEnable:%x \n", iCurrentEnable);

        if( (iCurrentEnable & ACCSNS_ACTIVE_VEHICLE) ||
            (iCurrentEnable & ACCSNS_ACTIVE_PEDOM)   ||
            (iCurrentEnable & ACCSNS_ACTIVE_MOVE) ){
            accsns_workqueue_create(accsns_wq, accsns_int_acc_work_func);
        }
    }
    
    accsns_workqueue_delete(work) ;
    ENABLE_IRQ;
    return;
}

static void accsns_int_acc_work_func(struct work_struct *work)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t iCurrentEnable;
    int32_t iWakeupSensor;
    int32_t ret = ACCSNS_RC_OK;
    
DBG(DBG_LV_INT, "### accsns_int_acc_work_func In \n");
    
    cmd.cmd.udata16 = HC_MCU_GET_INT_DETAIL;
    cmd.prm.uw_prm[0] = INTREQ_ACC;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "### accsns_int_acc_work_func Error INTREQ_ACC\n");
        accsns_workqueue_delete(work) ;
        return;
    }

    iCurrentEnable = (atomic_read(&g_CurrentSensorEnable) | atomic_read(&g_pause_sens));
    
    if(res.res.ub_res[0] & ID_ACC_PEDO_PEDO_TRANS){
        if(iCurrentEnable & ACCSNS_ACTIVE_VEHICLE){
DBG(DBG_LV_INT, "### *** Vehicle Detected !! \n");
            iWakeupSensor = atomic_read(&g_WakeupSensor);
            atomic_set(&g_WakeupSensor,iWakeupSensor | ACCSNS_ACTIVE_VEHICLE);
            wake_up_interruptible(&s_tPollWaitVehicle);

        }
    }

    if(res.res.ub_res[1] & ID_ACC_MOVE_DETECT){
DBG(DBG_LV_INT, "### *** Moving Detected !! \n");
        accsns_measure(ACCSNS_ACTIVE_MOVE);
            
        ret = accsns_move_detect(HC_INVALID);
        if((iCurrentEnable & ACCSNS_ACTIVE_MOVE) && (ret == ACCSNS_RC_OK)) {
            ret = accsns_stop_detect(HC_VALID);
            if(ACCSNS_RC_OK == ret){
                atomic_set(&g_nMoveDetectStatus, DETECT_MOVE);
DBG(DBG_LV_INT, "Moving Detected (Stop -> Move)\n");
            }
        }else{
            ret = ACCSNS_RC_ERR;
        }
        if(ACCSNS_RC_OK != ret){
            atomic_set(&g_nMoveDetectStatus, DETECT_NONE);
DBG(DBG_LV_ERROR, "Moving Detected Error (Stop -> Move)\n");
        }
        
    }
    if(res.res.ub_res[0] & ID_ACC_PEDO_STOP){
DBG(DBG_LV_INT, "### *** Stoping Detected !! \n");
            
        accsns_measure(ACCSNS_ACTIVE_MOVE);

        ret = accsns_stop_detect(HC_INVALID);
        if((iCurrentEnable & ACCSNS_ACTIVE_MOVE) && (ret == ACCSNS_RC_OK)) {
            ret = accsns_move_detect(HC_VALID);
            if(ACCSNS_RC_OK == ret){
                atomic_set(&g_nMoveDetectStatus, DETECT_STOP);
DBG(DBG_LV_INT, "Moving Detected (Move -> Stop)\n");
            }
        }else{
            ret = ACCSNS_RC_ERR;
        }
        if(ACCSNS_RC_OK != ret){
            atomic_set(&g_nMoveDetectStatus, DETECT_NONE);
DBG(DBG_LV_ERROR, "Moving Detected Error (Move -> Stop)\n");
        }
    }

    if(res.res.ub_res[2] & ID_ACC_PEDO_TIMER){
        if(iCurrentEnable & ACCSNS_ACTIVE_PEDOM){
DBG(DBG_LV_INT, "### *** Pedom Timer Detected !! \n");
            iWakeupSensor = atomic_read(&g_WakeupSensor);
            atomic_set(&g_WakeupSensor,iWakeupSensor | ACCSNS_ACTIVE_PEDOM);
            wake_up_interruptible(&s_tPollWaitPedom);
        }
    }
        
DBG(DBG_LV_INT, "### accsns_int_acc_work_func INTREQ_ACC iCurrentEnable:%x ret:%x res[0]:%x res[1]:%x res[2]:%x \n"
    ,iCurrentEnable, ret, res.res.ub_res[0], res.res.ub_res[1], res.res.ub_res[2]);
    
    accsns_workqueue_delete(work) ;
    return;
}
    
static void accsns_FWUpdate_work_func(struct work_struct *work)
{
    int32_t ret = 0;
DBG_PRINT_IO(0, 0);
    
    ret = accsns_update_fw(false, s_tAsyncFWUpdateData.m_cData, s_tAsyncFWUpdateData.m_nLen);
    
    kfree(s_tAsyncFWUpdateData.m_cData);

    s_tAsyncFWUpdateData.m_nLen      = 0;
    s_tAsyncFWUpdateData.m_cData     = 0;
    s_tAsyncFWUpdateData.m_nResult   = ret;
    s_tAsyncFWUpdateData.m_bComplete = true;
    s_tAsyncFWUpdateData.m_bRun      = false;
    
DBG_PRINT_IO(0xFF, ret);
}

static int32_t accsns_gpio_init(void)
{
    int32_t ret;
DBG_PRINT_IO(0, 0);

    ret = gpio_request(ACCSNS_GPIO_RST, ACC_GPIO_RESET_NAME);
    if (ret < 0){
DBG(DBG_LV_ERROR, "failed to gpio_request ret=%d\n",ret);
        return ret;
    }
    
    ret = gpio_direction_output(ACCSNS_GPIO_RST, 1);
    if (ret < 0){
DBG(DBG_LV_ERROR, "failed to gpio_request ret=%d\n",ret);
        goto ERROR;
    }
    
    ret = gpio_request(ACCSNS_GPIO_TEST0, ACC_GPIO_TEST0_NAME);
    if (ret < 0){
DBG(DBG_LV_ERROR, "failed to gpio_request ret=%d\n",ret);
        goto ERROR;
    }
    
    ret = gpio_direction_output(ACCSNS_GPIO_TEST0, 0);
    if (ret < 0){
DBG(DBG_LV_ERROR, "failed to gpio_request ret=%d\n",ret);
        goto ERROR;
    }
    
    g_nIntIrqNo = MSM_GPIO_TO_INT(ACCSNS_GPIO_INT);
    atomic_set(&g_bIsIntIrqEnable, true);
    ret = gpio_request(ACCSNS_GPIO_INT, ACC_GPIO_INT_NAME);
    if (ret < 0){
DBG(DBG_LV_ERROR, "failed to gpio_request ret=%d\n",ret);
        goto ERROR;
    }
    
    ret = gpio_direction_input(ACCSNS_GPIO_INT);
    if (ret < 0){
DBG(DBG_LV_ERROR, "failed to gpio_request1 ret=%d\n",ret);
        goto ERROR;
    }

    ret = request_any_context_irq(g_nIntIrqNo, accsns_irq_handler, IRQF_TRIGGER_LOW, ACC_GPIO_INT_NAME, NULL);
    if(ret < 0) {
DBG(DBG_LV_ERROR, "Failed gpio_request. ret=%x\n", ret);
        goto ERROR;
    }
DBG(DBG_LV_INT, "g_nIntIrqNo=%d\n",g_nIntIrqNo);
    
DBG_PRINT_IO(0xFF, ret);
    return ACCSNS_RC_OK;
    
ERROR:
    gpio_free(ACCSNS_GPIO_INT);
    gpio_free(ACCSNS_GPIO_RST);
    gpio_free(ACCSNS_GPIO_TEST0);
    return -ENODEV;
}
    
int32_t accsns_check_accsensor(void)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_OK;
DBG_PRINT_IO(0, 0);
    
    cmd.cmd.udata16 = HC_MCU_GET_EX_SENSOR;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "[ACC] accsns_initialize:CMD Error <HC_MCU_GET_EX_SENSOR> err %x\n", res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    if( !(res.res.uw_res[0] & 0x01) ){
DBG(DBG_LV_ERROR, "[ACC] accsns_initialize:Acc Sensor Not found[%x].\n", res.res.uw_res[0]);
        return ACCSNS_RC_ERR;
    }
DBG(DBG_LV_INFO, "Sensor Existed.%x \n", res.res.uw_res[0]);
    
DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}

static int32_t accsns_chip_read(uint8_t slave, uint8_t adr, uint8_t *data, uint16_t size)
{
    HostCmd    cmd;
    HostCmdRes res;
    int32_t    ret = ACCSNS_RC_ERR;
    
    if((data == NULL) || (size == 0) || (size > 8)){
DBG(DBG_LV_ERROR, "accsns_chip_read Error input param(data=%d or size=%d)\n",(int)data, size);
        return ret;
    }
    
    cmd.cmd.udata16 = HC_MCU_I2C_IO;
    cmd.prm.ub_prm[0] = slave;
    cmd.prm.ub_prm[1] = 0x00;
    cmd.prm.ub_prm[2] = adr;
    cmd.prm.ub_prm[3] = size;
    
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Chip I2C Read Error HC_MCU_I2C_IO err %x\n",res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    
    if(res.res.ub_res[0] == 0x01) {
DBG(DBG_LV_ERROR, "Chip I2C Read HC_MCU_I2C_IO slave=%x addr=%x size=%x <NACK>\n",slave, adr, size);
        return ACCSNS_RC_OK_NACK;
    }
    
    memcpy(data, &res.res.ub_res[1], size);
    
DBG(DBG_LV_SPI, "Chip Read:slave=%x adr=%x size=%x data(%x %x %x %x %x %x %x %x )\n"
        , slave, adr, size, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    
    return ACCSNS_RC_OK;
}

static int32_t accsns_chip_write(uint8_t slave, uint8_t adr, const uint8_t *data, uint8_t size)
{
    HostCmd    cmd;
    HostCmdRes res;
    int32_t    ret = ACCSNS_RC_ERR;
    
    if((data == NULL) || (size == 0) || (size > 8)){
DBG(DBG_LV_ERROR, "accsns_chip_write Error input param(data=%d or size=%d)\n",(int)data, size);
        return ret;
    }
    
    cmd.cmd.udata16 = HC_MCU_I2C_IO;
    cmd.prm.ub_prm[0] = slave;
    cmd.prm.ub_prm[1] = 0x01;
    cmd.prm.ub_prm[2] = adr;
    cmd.prm.ub_prm[3] = size;
    memcpy(&cmd.prm.ub_prm[4], data, size);

    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "Chip I2C Write Error HC_MCU_I2C_IO err %x\n",res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    
    if(res.res.ub_res[0] == 0x01) {
DBG(DBG_LV_ERROR, "Chip I2C Write HC_MCU_I2C_IO slave=%x addr=%x size=%x <NACK>\n",slave, adr, size);
        return ACCSNS_RC_OK_NACK;
    }
    
DBG(DBG_LV_SPI, "Chip Write:slave=%x adr=%x size=%x data(%x %x %x %x %x %x %x %x )\n"
        , slave, adr, size, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    
    return ACCSNS_RC_OK;
}
    
static int32_t CalcCRC8(uint8_t *p_MemAddr, uint8_t BlockSize) 
{
    uint8_t CrcReg = 0xFF;
    uint8_t MemByte; 
    uint8_t BitNo; 
    while( BlockSize ) 
    { 
        MemByte = *p_MemAddr; 
        for( BitNo = 0; BitNo < 8; BitNo++ ) 
        { 
            if( (CrcReg ^ MemByte) & 0x80 ) 
            { 
                CrcReg = (CrcReg << 1) ^ 0x11D;
            } 
            else 
            { 
                CrcReg <<= 1; 
            } 
            MemByte <<= 1; 
        } 
        BlockSize--; 
        p_MemAddr++; 
    } 
    CrcReg = ~CrcReg; 
    return (CrcReg); 
}
    
static int32_t accsns_nvm_restore(uint8_t *backup_data)
{
    int32_t    i;
    int32_t    ret;
    int32_t    crc;
    uint8_t    data[8];
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "    Restoredata[ 0- 7]:%02x %02x %02x %02x %02x %02x %02x %02x\n"
        , backup_data[0], backup_data[1], backup_data[2], backup_data[3], backup_data[4], backup_data[5], backup_data[6], backup_data[7]);
DBG(DBG_LV_INFO, "    Restoredata[ 8-15]:%02x %02x %02x %02x %02x %02x %02x %02x\n"
        , backup_data[8], backup_data[9], backup_data[10], backup_data[11], backup_data[12], backup_data[13], backup_data[14], backup_data[15]);
DBG(DBG_LV_INFO, "    Restoredata[16-23]:%02x %02x %02x %02x %02x %02x %02x %02x\n"
        , backup_data[16], backup_data[17], backup_data[18], backup_data[19], backup_data[20], backup_data[21], backup_data[22], backup_data[23]);
DBG(DBG_LV_INFO, "    Restoredata[24-26]:%02x %02x %02x\n"
        , backup_data[24], backup_data[25], backup_data[26]);
    
    crc = CalcCRC8(backup_data, 0x1A);
    if(crc != backup_data[0x1A]){
DBG(DBG_LV_ERROR, "[ACC] CRC Check...:NG %x <-> %x\n",crc ,backup_data[0x1A]);
        return ACCSNS_RC_ERR;
    }
DBG(DBG_LV_ERROR, "[ACC] CRC Check...:OK %x <-> %x\n",crc ,backup_data[0x1A]);

    for(i=0; i<3; i++){
        memcpy(data, &backup_data[i*8], 8);
        ret = accsns_chip_write(0x18, (i * 8), data, 8);
        if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "accsns_nvm_restore I2C(addr=0x%x) err %x\n",(i * 8), ret);
            return ret;
        }
    }
    memcpy(data, &backup_data[0x18], 3);
    ret = accsns_chip_write(0x18, 0x18, data, 3);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "accsns_nvm_restore I2C(addr=0x18) err %x\n", ret);
        return ret;
    }
    
    data[0] = 0x0A;
    ret = accsns_chip_write(0x18, 0x35, data, 0x01);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "accsns_nvm_restore I2C(addr=0x35) err %x\n",ret);
        return ret;
    }
    
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

static int32_t accsns_nvm_backup(uint8_t *backup_data)
{
    int32_t    i;
    int32_t    ret;
    int32_t    crc;
    uint8_t    data[8];
DBG_PRINT_IO(0, 0);
    
    for(i=0; i<3; i++){
        ret = accsns_chip_read(0x18, (i * 8), data, 8);
        if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "accsns_nvm_backup I2C(addr=0x%x) err %x\n",(i * 8), ret);
            return ret;
        }
        memcpy(&backup_data[i * 8], data, 8);
    }
    ret = accsns_chip_read(0x18, 0x18, data, 3);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "accsns_nvm_backup I2C(addr=0x18) err %x\n", ret);
        return ret;
    }
    memcpy(&backup_data[0x18], data, 3);

DBG(DBG_LV_INFO, "    Backupdata[ 0- 7]:%02x %02x %02x %02x %02x %02x %02x %02x\n"
        , backup_data[0], backup_data[1], backup_data[2], backup_data[3], backup_data[4], backup_data[5], backup_data[6], backup_data[7]);
DBG(DBG_LV_INFO, "    Backupdata[ 8-15]:%02x %02x %02x %02x %02x %02x %02x %02x\n"
        , backup_data[8], backup_data[9], backup_data[10], backup_data[11], backup_data[12], backup_data[13], backup_data[14], backup_data[15]);
DBG(DBG_LV_INFO, "    Backupdata[16-23]:%02x %02x %02x %02x %02x %02x %02x %02x\n"
        , backup_data[16], backup_data[17], backup_data[18], backup_data[19], backup_data[20], backup_data[21], backup_data[22], backup_data[23]);
DBG(DBG_LV_INFO, "    Backupdata[24-26]:%02x %02x %02x\n"
        , backup_data[24], backup_data[25], backup_data[26]);
    
    data[0] = 0x0A;
    ret = accsns_chip_write(0x18, 0x35, data, 0x01);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "accsns_nvm_backup I2C(addr=0x35) err %x\n",ret);
        return ret;
    }
    
    crc = CalcCRC8(backup_data, 0x1A);
    if(crc != backup_data[0x1A]){
DBG(DBG_LV_ERROR, "[ACC] CRC Check...:NG %x <-> %x\n",crc ,backup_data[0x1A]);
        return ACCSNS_RC_ERR;
    }
DBG(DBG_LV_ERROR, "[ACC] CRC Check...:OK %x <-> %x\n",crc ,backup_data[0x1A]);
    
DBG_PRINT_IO(0xFF, ret);
    return ret;
}
    
int32_t accsns_recovery_proc(int32_t kind, uint8_t *backup_data)
{
    int32_t    i;
    int32_t    ret;
    uint8_t    data[8];
    uint8_t    slave_base = 0x00;
    uint8_t    reg_val;
    uint8_t    slave_tbl[] = { 0x18, 0x10, 0x12, 0x14, 0x16, 0x1A, 0x1C, 0x1E };
DBG_PRINT_IO(0, 0);
DBG(ACCSNS_RC_ERR, "[ACC] Snsor Backup or Restor Start:kind=%x \n",kind);
    
    atomic_set(&g_FWUpdateStatus,true);
    
    if( (g_nAccFWVersion < ACC_FW_VERSION_RECOVER_2) &&
        (g_nAccFWVersion != ACC_FW_VERSION_RECOVER_1) ){
DBG(ACCSNS_RC_ERR, "[ACC] FWVersion(%x) is Not Supported recovery function.\n", g_nAccFWVersion);
        ret =  ACCSNS_RC_OK;
        goto ERROR;
    }
    
    for(i=0; i<sizeof(slave_tbl); i++){
        slave_base = slave_tbl[i];
        ret = accsns_chip_read(slave_base, 0x00, data, 0x01 );
        if(ret < 0){
DBG(DBG_LV_ERROR, "accsns_recovery_proc I2C(addr=0x00) err %x\n",ret);
            ret = ACCSNS_RC_ERR;
            goto ERROR;
        }
DBG(DBG_LV_INFO, "Slave Check...(slave=%x ret=%x data=%x) \n",slave_base, ret, data[0]);
        if(ret != ACCSNS_RC_OK_NACK){
DBG(ACCSNS_RC_ERR, "[ACC] Sensor I2C Slave Find!! (slave=%x) \n",slave_base);
            break;
        }
    }
    if(i == sizeof(slave_tbl)){
DBG(DBG_LV_INFO, "Slave All Error \n");
        ret = ACCSNS_RC_ERR;
        goto ERROR;
    }
    
    data[0] = 0x00;
    ret = accsns_chip_write(slave_base, 0x11, data, 0x01);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "Chip Write I2C(addr=0x11, data=%x) err %x\n",data[0], ret);
        goto ERROR;
    }
    
    mdelay(2);
    
    data[0] = 0xAA;
    ret = accsns_chip_write(slave_base, 0x35, data, 0x01);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "Chip Write I2C(addr=0x35, data=%x) err %x\n",data[0], ret);
        goto ERROR;
    }
    ret = accsns_chip_write(slave_base, 0x35, data, 0x01);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "Chip Write I2C(addr=0x35, data=%x) err %x\n",data[0], ret);
        goto ERROR;
    }
    
    ret = accsns_chip_read(slave_base, 0x05, data, 0x01 );
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "Chip Read I2C(addr=0x05) err %x\n",ret);
        goto ERROR;
    }
    reg_val = data[0];
        
DBG(DBG_LV_INFO, "reg_val=%x \n",reg_val);
    
    data[0] = (reg_val & 0x1F) | 0x80;
    ret = accsns_chip_write(slave_base, 0x05, data, 0x01);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "Chip Write I2C(addr=0x35, data=%x) err %x\n",data[0], ret);
        goto ERROR;
    }
    
    if (kind == 0x00){
DBG(ACCSNS_RC_ERR, "[ACC] Snsor Backup Start. \n");
        ret = accsns_nvm_backup(backup_data);
        
    }else if(kind == 0x01){
DBG(ACCSNS_RC_ERR, "[ACC] Snsor Restore Start. \n");
        ret = accsns_nvm_restore(backup_data);
    }else{
DBG(DBG_LV_ERROR, "Param err %x\n",ret);
        goto ERROR;
    }

    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "Restore or Backup err %x\n",ret);
        goto ERROR;
    }
    
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    ret = accsns_chip_write(0x18, 0x38, data, 0x03 );
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "Chip Write I2C(addr=%x, data=%x %x %x) err %x\n",0x38, data[0], data[1], data[2], ret);
        goto ERROR;
    }

    atomic_set(&g_FWUpdateStatus,false);
    
DBG(ACCSNS_RC_ERR, "[ACC] Snsor Backup or Restor Complete[%d]!! \n",ret);
    DBG_PRINT_IO(0xFF, ret);
    return ret;
    
ERROR:
    data[0] = 0x0A;
    accsns_chip_write(0x18, 0x35, data, 0x01);

    atomic_set(&g_FWUpdateStatus,false);

DBG_PRINT_IO(0xFF, ACCSNS_RC_ERR);
    return ACCSNS_RC_ERR;
}

int32_t accsns_get_fw_version(uint8_t *arg_iData)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ACCSNS_RC_OK;
DBG_PRINT_IO(0, 0);
    
    if(s_tAsyncFWUpdateData.m_bRun){
DBG(DBG_LV_ERROR, "FW Async Update Executing now!!");
        return ACCSNS_RC_ERR;
    }
    
    cmd.cmd.udata16 = HC_MCU_GET_VERSION;

    ret = accsns_hostcmd(&cmd, &res, (EXE_HOST_ALL | EXE_HOST_EX_NO_RECOVER));
    if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "FW Version Get Error HC_MCU_GET_VERSION(%d) err %x\n",cmd.prm.ub_prm[0], res.err.udata16);
        return ACCSNS_RC_ERR;
    }
    
    arg_iData[0] = res.res.ub_res[0];
    arg_iData[1] = res.res.ub_res[1];
    arg_iData[2] = res.res.ub_res[2];
    arg_iData[3] = res.res.ub_res[3];
    
DBG(DBG_LV_INFO, "FW Version=%02x %02x %02x %02x\n", arg_iData[0], arg_iData[1], arg_iData[2], arg_iData[3]);
DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return ACCSNS_RC_OK;
}
    
static void accsns_FW_TEST0_ctrl(void)
{
DBG(DBG_LV_ERROR, "accsns_update_fw_seq: TEST0 Ctro Start... \n");

    gpio_set_value(ACCSNS_GPIO_RST, 0);
    
    gpio_set_value(ACCSNS_GPIO_TEST0, 1);
    
    udelay(55);
    
    gpio_set_value(ACCSNS_GPIO_RST, 1);
    
    msleep(560);
    
    gpio_set_value(ACCSNS_GPIO_TEST0, 0);
}

int32_t accsns_update_fw_seq(uint8_t *arg_iData, uint32_t arg_iLen)
{
    int32_t    ret = ACCSNS_RC_OK;
    uint8_t    fw_ver[4];
    uint32_t   bb_ver;
    HostCmd    cmd;
    HostCmdRes res;
DBG_PRINT_IO(0, 0);
    
    atomic_set(&g_FWUpdateStatus,true);
    
    ret = accsns_get_fw_version(fw_ver);
    g_nAccFWVersion = ACC_FW_VERSION_GET_32BIT(fw_ver);
    if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "accsns_update_fw_seq:Get Version Error!!\n");
        g_nAccFWVersion = ACC_FW_VERSION_NONE;
    }
    
DBG(DBG_LV_ERROR, "accsns_update_fw_seq:Now=%x, Base:%x \n",g_nAccFWVersion, ACC_FW_VERSION_DATA);
    
    if(g_nAccFWVersion != ACC_FW_VERSION_DATA){
DBG(DBG_LV_ERROR, "[ACC] accsns_update_fw_seq:Need to update F/W Version\n");
        
        cmd.cmd.udata16 = HC_MCU_SELF_CHK_FW;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL | EXE_HOST_EX_NO_RECOVER);
DBG(DBG_LV_ERROR, "accsns_update_fw_seq: HC_MCU_SELF_CHK_FW(%d) err res=%x err=%x\n", ret, res.res.ub_res[0], res.err.udata16);
          if(1) {
            
            bb_ver = OEM_get_board();
DBG(DBG_LV_ERROR, "accsns_update_fw_seq: BB Check... (Ver:%d)\n", bb_ver);
            if(bb_ver >= OEM_BOARD_PP2_TYPE){
                
DBG(DBG_LV_ERROR, "[ACC] accsns_update_fw_seq: F/W Update Start... \n");
                DISABLE_IRQ;
                accsns_FW_TEST0_ctrl();
                ret = accsns_update_fw(true, arg_iData, arg_iLen);
            }

        }else if(0x00 == res.res.ub_res[0]){
DBG(DBG_LV_ERROR, "accsns_update_fw_seq: HC_MCU_SELF_CHK_FW(-) OK\n");
        }
        
    }else{
DBG(DBG_LV_INFO, "accsns_update_fw_seq:None update F/W Version\n");
    }
DBG(DBG_LV_INFO, "accsns_update_fw_seq: F/W Update Check Completed... \n");
    
DBG(DBG_LV_INFO, "accsns_update_fw_seq: F/W Initialize Start... \n");
    ret |= accsns_initialize();
        
    atomic_set(&g_FWUpdateStatus,false);
    
DBG_PRINT_IO(0xFF, ret);
    return ret;
}

int32_t accsns_update_fw(bool boot, uint8_t *arg_iData, uint32_t arg_iLen)
{
    uint8_t reg = 0xFF;
    Word    sreg;
    int32_t i;
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;
    
DBG_PRINT_IO(0, 0);
    

    memset(&res, 0x00, sizeof(HostCmdRes));

DBG(DBG_LV_INFO, "### Start Firmware Update ### boot=%d, Data=%x Len=%d \r\n", boot, (int)arg_iData, arg_iLen);
    
    if((arg_iData == NULL) || (arg_iLen == 0)){
        DISABLE_IRQ;
        return ACCSNS_RC_ERR;
    }
    
    if(!boot){
        ENABLE_IRQ;
        
        cmd.cmd.udata16 = HC_MCU_FUP_START;
        cmd.prm.ub_prm[0] = 0x55;
        cmd.prm.ub_prm[1] = 0xAA;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_WAIT|EXE_HOST_ERR);
        if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "Communication Error!\r\n");
            DISABLE_IRQ;
            return ACCSNS_RC_ERR;
        }
        if(res.err.udata16 == ERROR_FUP_CERTIFICATION) {
DBG(DBG_LV_ERROR, "Certification Error!\r\n");
            DISABLE_IRQ;
            return ACCSNS_RC_ERR;
        }

    }else{
        reg = 0x04;
        accsns_spi_write(CFG, &reg, sizeof(reg));

        accsns_spi_read(INTREQ0, sreg.udata8, 2);

        reg = 0x00;
        accsns_spi_write(INTMASK0, &reg, sizeof(reg));
        accsns_spi_write(INTMASK1, &reg, sizeof(reg));
        
        ENABLE_IRQ;
        
        cmd.cmd.udata16 = HC_MCU_SET_PCON;
        cmd.prm.ub_prm[0] = 0x00;
        cmd.prm.ub_prm[1] = 0x00;
        cmd.prm.ub_prm[2] = 0x00;
        cmd.prm.ub_prm[3] = 0x00;
        cmd.prm.ub_prm[4] = 0x01;
        cmd.prm.ub_prm[5] = 0x01;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
        if((ACCSNS_RC_OK != ret) || (0 != res.err.udata16)) {
DBG(DBG_LV_ERROR, "[ACC] PortSettint err %x\n", res.err.udata16);
            DISABLE_IRQ;
            return ACCSNS_RC_ERR;
        }
    }

DBG(DBG_LV_INFO, "Check Firmware Mode.\r\n");
    cmd.cmd.udata16 = HC_MCU_GET_VERSION;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "Communication Error!\r\n");
        DISABLE_IRQ;
        return ACCSNS_RC_ERR;
    }
    if(res.res.ub_res[2] != 0x01){
DBG(DBG_LV_ERROR, "Version check Error!\r\n");
        DISABLE_IRQ;
        return ACCSNS_RC_ERR;
    }

DBG(DBG_LV_INFO, "Flash Clear.\r\n");
    cmd.cmd.udata16 = HC_MCU_FUP_ERASE;
    cmd.prm.ub_prm[0] = 0xAA;
    cmd.prm.ub_prm[1] = 0x55;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_WAIT|EXE_HOST_ERR);
    if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "Communication Error!\r\n");
        DISABLE_IRQ;
        return ACCSNS_RC_ERR;
    }
    if(res.err.udata16 == ERROR_FUP_CERTIFICATION) {
DBG(DBG_LV_ERROR, "Certification Error!\r\n");
        DISABLE_IRQ;
        return ACCSNS_RC_ERR;
    }

DBG(DBG_LV_INFO, "Flash Write.\r\n");
    for(i=0; i < arg_iLen; i+=8){
        cmd.prm.ub_prm[0] = 3;
        cmd.prm.ub_prm[1] = arg_iData[i + 0];
        cmd.prm.ub_prm[2] = arg_iData[i + 1];
        cmd.prm.ub_prm[3] = arg_iData[i + 2];
        cmd.prm.ub_prm[4] = arg_iData[i + 3];
        cmd.prm.ub_prm[5] = arg_iData[i + 4];
        cmd.prm.ub_prm[6] = arg_iData[i + 5];
        cmd.prm.ub_prm[7] = arg_iData[i + 6];
        cmd.prm.ub_prm[8] = arg_iData[i + 7];
        cmd.cmd.udata16 = HC_MCU_FUP_WRITE;
        ret = accsns_hostcmd(&cmd, &res, EXE_HOST_WAIT|EXE_HOST_ERR);
        if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "Communication Error!\r\n");
            DISABLE_IRQ;
            return ACCSNS_RC_ERR;
        }
        if(res.err.udata16 == ERROR_FUP_MAXSIZE) {
DBG(DBG_LV_ERROR, "Size max Error! :: (%d/%dbyte)\r\n",i,(60*1024));
            DISABLE_IRQ;
            return ACCSNS_RC_ERR;
        }else if(res.err.udata16 == ERROR_FUP_VERIFY) {
DBG(DBG_LV_ERROR, "Verify Error! :: (%d/%dbyte)\r\n",i,(60*1024));
            DISABLE_IRQ;
            return ACCSNS_RC_ERR;
        }
    }

DBG(DBG_LV_INFO, "End Firmware Update.\r\n");
    cmd.cmd.udata16 = HC_MCU_FUP_END;
    accsns_hostcmd(&cmd, &res, 0);
    
    msleep(300);

DBG(DBG_LV_INFO, "Check User program mode.\r\n");
    cmd.cmd.udata16 = HC_MCU_GET_VERSION;
    ret = accsns_hostcmd(&cmd, &res, EXE_HOST_ALL);
    if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "Communication Error!\r\n");
        DISABLE_IRQ;
        return ACCSNS_RC_ERR;
    }
    if(res.res.ub_res[2] != 0x00){
DBG(DBG_LV_ERROR, "Version check Error!");
        DISABLE_IRQ;
        return ACCSNS_RC_ERR;
    }
    
    DISABLE_IRQ;
DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);

    return ACCSNS_RC_OK;
}
    
int32_t accsns_update_fw_async(uint8_t *arg_iData, uint32_t arg_iLen)
{
    int32_t iCurrentEnable;
    
DBG_PRINT_IO(0, 0);
    
    iCurrentEnable = atomic_read(&g_CurrentSensorEnable);
    if(iCurrentEnable & ACTIVE_FUNC_MASK){
DBG(DBG_LV_ERROR, "FW Async Update Error Status:%x", iCurrentEnable);
        return -1;
    }
    
    if(!s_tAsyncFWUpdateData.m_bRun){
        s_tAsyncFWUpdateData.m_cData      = arg_iData;
        s_tAsyncFWUpdateData.m_nLen       = arg_iLen;
        s_tAsyncFWUpdateData.m_bRun       = true;
        s_tAsyncFWUpdateData.m_bComplete  = false;
        s_tAsyncFWUpdateData.m_nResult    = 0;
        
DBG(DBG_LV_INFO, "FW Async Update Len=%x.\n", s_tAsyncFWUpdateData.m_nLen);
        queue_work(fw_update_wq, &s_tWork_FW_Update);

    }else{
DBG(DBG_LV_ERROR, "FW Async Update Thread pending now!!\n");
        return -1;
    }
    
DBG_PRINT_IO(0xFF, ACCSNS_RC_OK);
    return 0;
}
    
int32_t accsns_update_fw_async_chk(void)
{
    int32_t result;
DBG_PRINT_IO(0, 0);
    
    if(s_tAsyncFWUpdateData.m_nResult != 0){
        result = -EFAULT;
    
    }else if(s_tAsyncFWUpdateData.m_bRun){
        result = -EBUSY;
        
    }else if(s_tAsyncFWUpdateData.m_bComplete == true){
        result = 0;
        
    }else{
        result = -EIO;
    }
    
DBG_PRINT_IO(0xFF, result);
    return result;
}
    
static void accsns_workqueue_init(void)
{
    int32_t i;
DBG_PRINT_IO(0, 0);
    
    for(i=0; i<ACC_WORK_QUEUE_NUM; i++){
        cancel_work_sync(&s_tAccWork[i].work);
        s_tAccWork[i].status = false;
    }
    s_nAccWorkCnt = 0;
}
    
static int32_t accsns_workqueue_create( struct workqueue_struct *queue, void (*func)(struct work_struct *) )
{
    int32_t ret = ACCSNS_RC_ERR;
    unsigned long flags;
    
    if((queue == NULL) || (func == NULL)){
        return ACCSNS_RC_ERR;
    }
    
    spin_lock_irqsave(&acc_lock, flags);
    
DBG(DBG_LV_LOW, "  [%d]  status:0x%x, \n",s_nAccWorkCnt, s_tAccWork[s_nAccWorkCnt].status);
        
    if(s_tAccWork[s_nAccWorkCnt].status == false){
        
        INIT_WORK( &s_tAccWork[s_nAccWorkCnt].work, func );
        
        ret = queue_work( queue, &s_tAccWork[s_nAccWorkCnt].work );
        
        if (ret == 1) {
            s_tAccWork[s_nAccWorkCnt].status = true;
            
            if(++s_nAccWorkCnt >= ACC_WORK_QUEUE_NUM){
                s_nAccWorkCnt = 0;
            }
            ret = ACCSNS_RC_OK;
            
        }else{
DBG(DBG_LV_ERROR, "ACC %s[%d] queue_work Non Create(%d) \n",__FUNCTION__, __LINE__, ret);
        }
        
    }else{
DBG(DBG_LV_ERROR, "ACC queue_work[%d] used!! \n", s_nAccWorkCnt);
    }
    
    spin_unlock_irqrestore(&acc_lock, flags);

    return ret;
}
    
static void accsns_workqueue_delete(struct work_struct *work)
{
    int32_t i;
    unsigned long flags;
    
    spin_lock_irqsave(&acc_lock, flags);
    
    for(i=0; i<ACC_WORK_QUEUE_NUM; i++){
        
        if(&s_tAccWork[i].work == work){
            s_tAccWork[i].status = false;
            
DBG(DBG_LV_LOW, "  hit delete queue[%d]! work:0x%x 0x%x \n",i, (int)&s_tAccWork[i].work, (int)work);
            break;
        }
    }
    
    spin_unlock_irqrestore(&acc_lock, flags);
    return ;
}

static void accsns_timeout_dump(uint8_t *reg)
{
    unsigned char data[0x40];
    int i;
    
    DBG(DBG_LV_ERROR, "##### ACC TimeOut Error Log #####\n");
    DBG(DBG_LV_ERROR, "  Gloval Value :\n");
    DBG(DBG_LV_ERROR, "     Send Cmd :%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x \n"
                             ,reg[0] ,reg[1] ,reg[2] ,reg[3] ,reg[4] ,reg[5] ,reg[6] ,reg[7] ,reg[8] ,reg[9] ,reg[10] 
                             ,reg[11] ,reg[12] ,reg[13] ,reg[14] ,reg[15] );
    DBG(DBG_LV_ERROR, "     g_bIsIntIrqEnable = %04x, g_nIntIrqFlg = %04x, ACCSNS_GPIO_INT= %04x \n"
                        , (int)atomic_read(&g_bIsIntIrqEnable)
                        , g_nIntIrqFlg
                        , gpio_get_value(ACCSNS_GPIO_INT));
    
    accsns_spi_read(0x00, data, sizeof(data));
    
    DBG(DBG_LV_ERROR, "  H/W Register Dump :");
    for(i=0; i<sizeof(data); i++){
        if((i % 16) == 0){
            DBG(DBG_LV_ERROR, "\n     %02XH : ", i);
        }
        DBG(DBG_LV_ERROR, "0x%02X ",data[i]);
    }
    DBG(DBG_LV_ERROR, "\n");
    
    dbg_level = 0;
}

static void accsns_shutdown( struct spi_device *client )
{
DBG(DBG_LV_IO, "shutdown\n");
    accsns_activateEx(0, atomic_read(&g_flgEna), POWER_DISABLE);
}

int32_t accsns_suspend( struct spi_device *client, pm_message_t mesg )
{
DBG(DBG_LV_IO, "ACC suspend\n");

    if (atomic_read(&g_flgEna) & ACCSNS_ACTIVE_PEDOM) {
        enable_irq_wake(g_nIntIrqNo);
    }else{
        DBG(DBG_LV_INFO, "%s: SUSPEND.\n",__FUNCTION__);
        accsns_activateEx(0, atomic_read(&g_flgEna) & ACTIVE_FUNC_MASK, POWER_DISABLE);
    }
    
    return ACCSNS_RC_OK;
}

int32_t accsns_resume( struct spi_device *client )
{
DBG(DBG_LV_IO, "ACC resume\n");

    if (atomic_read(&g_flgEna) & ACCSNS_ACTIVE_PEDOM) {
        disable_irq_wake(g_nIntIrqNo);
    } else {
DBG(DBG_LV_INFO, "%s: RESUME.\n",__FUNCTION__);
        accsns_activateEx(0, atomic_read(&g_flgEna) & ACTIVE_FUNC_MASK, POWER_ENABLE);
    }
    
    return ACCSNS_RC_OK;
}

static int32_t accsns_remove( struct spi_device *client )
{
DBG(DBG_LV_IO, "remove\n");
    accsns_activateEx(0, atomic_read(&g_flgEna), POWER_DISABLE);
    
    return ACCSNS_RC_OK;
}

static void accsns_early_suspend(struct early_suspend *h)
{
DBG(DBG_LV_IO, "ACC early_suspend\n");

    if (atomic_read(&g_flgEna) & ACCSNS_ACTIVE_PEDOM) {
DBG(DBG_LV_INFO, "%s: TIMER STOP only.\n",__FUNCTION__);
        accsns_pedom_start_timer(0);
    }
}

static void accsns_late_resume(struct early_suspend *h)
{
DBG(DBG_LV_IO, "ACC late_resume");

    if (atomic_read(&g_flgEna) & ACCSNS_ACTIVE_PEDOM) {
DBG(DBG_LV_INFO, "%s: TIMER START only.\n",__FUNCTION__);
        accsns_pedom_start_timer(1);
    }
}

static void accsns_queue_work(int pause_timer, bool on)
{
    DBG_PRINT_IO(0, 0);
    s_twork_pause[work_ped_pos].stop_timer = pause_timer;
    s_twork_pause[work_ped_pos].status = on;

	if (queue_work(accsns_wq_pause,&(s_twork_pause[work_ped_pos].work_pause)) != 0) {
	    work_ped_pos++;
	    if (work_ped_pos >= ACC_WORK_QUEUE_NUM) {
		    work_ped_pos = 0;
	    }
	}
    DBG_PRINT_IO(0xFF, 0);
}

void accsns_ddi_set_pause_proc(int type, int pause_timer)
{
    DBG_PRINT_IO(0, 0);
    
    if(atomic_read(&g_FWUpdateStatus)){
DBG(DBG_LV_INFO, "[ACC] FW Update or Recovery Now:%s\n", __FUNCTION__);
        return;
    }

    if (atomic_read(&g_flgEna) & (ACCSNS_ACTIVE_PEDOM | ACCSNS_ACTIVE_VEHICLE)){
        accsns_queue_work(pause_timer,0);
    }

    DBG_PRINT_IO(0xFF, 0);
}

static void accsns_pause_work_func(struct work_struct *work)
{
    int ret = 0;
    int pause_timer = 0;
    int iflgEna = 0;
    struct ped_work_struct *work_tmp = (struct ped_work_struct*)work;

DBG_PRINT_IO(0, 0);

    if (work == NULL){
        DBG(DBG_LV_ERROR, "[ACC]<%s> work Null pointer \n",__FUNCTION__)
        return;
    }

    if (work_tmp->stop_timer <= DEFAULT_PAUSE_TIMER ){
        pause_timer = DEFAULT_PAUSE_TIMER;
    }else{
        pause_timer = work_tmp->stop_timer;
    }

    if (hrtimer_active(&ped_timer)){
            hrtimer_cancel(&ped_timer);
		    hrtimer_start(&ped_timer,
               ktime_set(pause_timer / 1000, 
			   (pause_timer % 1000) * 1000000),
			   HRTIMER_MODE_REL);
    }else{
        iflgEna = (atomic_read(&g_flgEna) & (ACCSNS_ACTIVE_PEDOM | ACCSNS_ACTIVE_VEHICLE));
DBG(DBG_LV_INFO, "FlgEnable check...(%x) <pause_sens=%x, status=%x>\n",iflgEna, atomic_read(&g_pause_sens), work_tmp->status);
        if (iflgEna) {
            atomic_set(&g_pause_sens, 0);

            if (work_tmp->status == 0){
                dummy_on_flg = true;

                if(iflgEna & ACCSNS_ACTIVE_PEDOM) {
                    accsns_pedom_start_timer(0);
                }
            }
            
            ret = accsns_activateEx(0, iflgEna, work_tmp->status);

            if (work_tmp->status == 0){
                dummy_on_flg = false;

                atomic_set(&g_pause_sens, iflgEna);
                
                hrtimer_start(&ped_timer,
                   ktime_set(pause_timer / 1000, 
                   (pause_timer % 1000) * 1000000),
                   HRTIMER_MODE_REL);

            }else{
                if(iflgEna & ACCSNS_ACTIVE_PEDOM) {
                    accsns_pedom_start_timer(1);
                }
            }
            
        }else{
            atomic_set(&g_pause_sens, 0);
        }
    }
DBG_PRINT_IO(0xFF, 0);

}

static enum hrtimer_restart accsns_timer_func(struct hrtimer *timer)
{
    DBG_PRINT_IO(0, 0);

    accsns_queue_work(0, 1);

    DBG_PRINT_IO(0xFF, 0);

	return HRTIMER_NORESTART;
}


static void hrtimer_cancel_timer_func(struct hrtimer *timer)
{
DBG_PRINT_IO(0, 0);

    if (hrtimer_active(timer)){
        hrtimer_cancel(timer);
    }

DBG_PRINT_IO(0xFF, 0);
}

static int32_t accsns_probe( struct spi_device *client )
{
    int32_t i;
    int32_t ret;
DBG_PRINT_IO(0, 0);
DBG(DBG_LV_INFO, "[ACC] accsns_probe\n");

    g_acc_nv_param.pedo_p.param_on             = 0;
    g_acc_nv_param.pedo_p.on                   = 0;
    g_acc_nv_param.pedo_p.stepwide             = 40;
    g_acc_nv_param.pedo_p.weight               = 60;
    g_acc_nv_param.pedo_p.notify               = 0;
    g_acc_nv_param.pedo_p.speed_ave_time       = 3;
    g_acc_nv_param.pedo_p.mets_stop_time       = 10;
    g_acc_nv_param.pedo_p.bodyfat_on           = 0;
    g_acc_nv_param.pedo_p.bodyfat_cal          = 72;
    g_acc_nv_param.dist_stop_p.stop_notify_on  = 0;
    g_acc_nv_param.dist_stop_p.stop_notify_time= 60;
    g_acc_nv_param.walk_run_p.walk_judge_on    = 0;
    g_acc_nv_param.walk_run_p.consecutive_num  = 1;
    g_acc_nv_param.walk_run_p.speed_th         = 167;
    g_acc_nv_param.trans_p.on                  = 1;
    g_acc_nv_param.trans_p.judge_step          = 50;
    g_acc_nv_param.trans_p.judge_time          = 300;
    g_acc_nv_param.trans_p.calc_time           = 2;
    g_acc_nv_param.trans_p.consecutive_num     = 30;
    g_acc_nv_param.trans_byc_p.consecutive_num = 3;
    g_acc_nv_param.trans_byc_p.calory_mets     = 50;
    g_acc_nv_param.timer_p.on                  = 1;
    g_acc_nv_param.timer_p.num                 = 120000;
    g_acc_nv_param.move_p.on                   = 1;
    g_acc_nv_param.move_p.axis                 = 0x07;
    g_acc_nv_param.move_p.acc_th               = 154;
    g_acc_nv_param.move_p.judge_num            = 5;
    g_acc_nv_param.move_p.judge_th             = 3;
    g_acc_nv_param.move_p.judge                = 1;

    client_accsns = client;
    client_accsns->bits_per_word = 8;
    
    g_nIntIrqNo = -1;
    g_nIntIrqFlg = 0;
    atomic_set(&g_flgEna, ACTIVE_OFF);
    atomic_set(&g_CurrentSensorEnable,ACTIVE_OFF);
    atomic_set(&g_WakeupSensor,ACTIVE_OFF);
    atomic_set(&g_FWUpdateStatus,false);
    g_bSpiError = false;
    g_bPedoTimerStatus = false;
    work_ped_pos = 0;
    dummy_on_flg = false;
    atomic_set(&g_pause_sens, 0);
    
    atomic_set(&g_nCalX,0);
    atomic_set(&g_nCalY,0);
    atomic_set(&g_nCalZ,0);
    
    atomic_set(&s_nDelayTime, DEFAULT_ACC_DELAY);
    atomic_set(&g_nWeight, DEFAULT_WEIGHT);
    atomic_set(&g_nStepWide, DEFAULT_PEDOMETER);
    atomic_set(&g_nPedoTimerCount, DEFAULT_PEDO_TIMER);
    atomic_set(&g_nMoveDetectStatus, DETECT_NONE);

    memset(&s_tLatestAccData, 0x00, sizeof(s_tLatestAccData));
    memset(&s_tLatestPedoData, 0x00, sizeof(s_tLatestPedoData));
    memset(&s_tLatestVehiData, 0x00, sizeof(s_tLatestVehiData));
    memset(&s_tLatestMoveData, 0x00, sizeof(s_tLatestMoveData));
    memset(&s_tAsyncFWUpdateData, 0x00, sizeof(s_tAsyncFWUpdateData));

    init_waitqueue_head(&s_tWaitInt);
    INIT_WORK(&s_tWork_FW_Update, accsns_FWUpdate_work_func);
    INIT_DELAYED_WORK(&s_tDelayWork_Acc, accsns_acc_work_func);
    for (i = 0; i < ACC_WORK_QUEUE_NUM; i++) {
        INIT_WORK(&(s_twork_pause[i].work_pause), accsns_pause_work_func);
        s_twork_pause[i].status = 0;
        s_twork_pause[i].stop_timer = 0;
    }
    accsns_workqueue_init();
    
    mutex_init(&s_tDataMutex);
    mutex_init(&(s_tCalibCtrl.m_tCalibMutex));
    mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
    
    s_tCalibCtrl.m_bFilterEnable        = false;
    s_tCalibCtrl.m_bWaitSetting         = true;
    s_tCalibCtrl.m_bComplete            = true;
    s_tCalibCtrl.m_unSmpN               = OFFSET_SUMPLE_NUM; 

    s_tCalibCtrl.m_tFilterWork.m_ucAveN = OFFSET_AVE_NUM;   
    s_tCalibCtrl.m_nCalX                = 0;
    s_tCalibCtrl.m_nCalY                = 0;
    s_tCalibCtrl.m_nCalZ                = 0;
    
    s_tCalibCtrl.m_nCurrentSampleNum    = 0;
    s_tCalibCtrl.m_nSummationX          = 0;
    s_tCalibCtrl.m_nSummationY          = 0;
    s_tCalibCtrl.m_nSummationZ          = 0;
    s_tCalibCtrl.m_nMode                = -1; 
    for (i = 0; i < AXIS_XYZ_MAX; i++) {
        if(s_tCalibCtrl.m_tFilterWork.m_pSamplWork[i] != NULL) {
DBG(DBG_LV_INFO, "accsns_probe:Calib err occurd \n");
        } else {
            s_tCalibCtrl.m_tFilterWork.m_pSamplWork[i] = NULL;
        }
    }
    memset(s_tCalibCtrl.m_tFilterWork.m_unSum, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);
    memset(s_tCalibCtrl.m_tFilterWork.m_unCnt, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);
    mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));
    
    if(g_nIntIrqNo == -1){
        ret = accsns_gpio_init();
        if(ret != ACCSNS_RC_OK){
DBG(DBG_LV_ERROR, "Failed accsns_gpio_init. ret=%x\n", ret);
            return ret;
        }
        DISABLE_IRQ;
    }
    
    ret = accsns_initialize();
    if(ret != ACCSNS_RC_OK) {
DBG(DBG_LV_ERROR, "Failed accsns_initialize. ret=%x\n", ret);
        return -ENODEV;
    }
    
DBG(DBG_LV_INFO, "Init Complete!!!\n");
DBG_PRINT_IO(0xFF, 0);
    return 0;
}

static struct spi_driver accsns_driver = {
    .probe       = accsns_probe,
    .driver = {
        .name    = ACC_DRIVER_NAME,
        .bus     = &spi_bus_type,
        .owner   = THIS_MODULE,
    },
    .remove      = accsns_remove,
    .suspend     = accsns_suspend,
    .resume      = accsns_resume,
    .shutdown    = accsns_shutdown,
};
    
static struct early_suspend accsns_early_suspend_handler = {
  .level	= EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
  .suspend	= accsns_early_suspend,
  .resume 	= accsns_late_resume,
}; 

static int32_t __init accsns_init( void )
{
    int32_t ret;
DBG_PRINT_IO(0, 0);
    
    sema_init(&s_tAccSnsrSema, 1);

    hrtimer_init(&ped_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    ped_timer.function = accsns_timer_func;

    accsns_wq_int = create_singlethread_workqueue("accsns_wq_int");
    if(!accsns_wq_int)
    {
DBG(DBG_LV_ERROR, "can't create interrupt queue : accsns_wq_int \n");
        return -ENODEV;
    }
    accsns_wq = create_singlethread_workqueue("accsns_wq");
    if(!accsns_wq)
    {
DBG(DBG_LV_ERROR, "can't create interrupt queue : accsns_wq \n");
        goto REGIST_ERR;
    }
    fw_update_wq = create_singlethread_workqueue("fw_update_wq");
    if(!fw_update_wq)
    {
DBG(DBG_LV_ERROR, "can't create interrupt queue : fw_update_wq \n");
        goto REGIST_ERR;
    }
    
    accsns_wq_pause = create_singlethread_workqueue("accsns_wq_pause");
    if(!accsns_wq_pause)
    {
DBG(DBG_LV_ERROR, "can't create interrupt queue : accsns_wq_pause \n");
        return -ENODEV;
    }

    ret = spi_register_driver(&accsns_driver);
    if(ret != 0){
DBG(DBG_LV_ERROR, "can't regist spi driver \n");
        goto REGIST_ERR;
    }
    
    init_waitqueue_head(&s_tPollWaitPedom);
    init_waitqueue_head(&s_tPollWaitVehicle);
    
    register_early_suspend(&accsns_early_suspend_handler);

    return 0;
    
REGIST_ERR:
    if(accsns_wq_int != NULL){
        flush_workqueue(accsns_wq_int);
        destroy_workqueue(accsns_wq_int);
        accsns_wq_int = NULL;
    }
    if(accsns_wq != NULL){
        flush_workqueue(accsns_wq);
        destroy_workqueue(accsns_wq);
        accsns_wq = NULL;
    }

    if(fw_update_wq != NULL){
        flush_workqueue(fw_update_wq);
        destroy_workqueue(fw_update_wq);
        fw_update_wq = NULL;
    }

    if(accsns_wq_pause != NULL){
        flush_workqueue(accsns_wq_pause);
        destroy_workqueue(accsns_wq_pause);
        accsns_wq_pause = NULL;
    }

    return -ENODEV;
}

static void __exit accsns_exit( void )
{
    unregister_early_suspend(&accsns_early_suspend_handler);
    
    DISABLE_IRQ;
    accsns_workqueue_init();
    cancel_work_sync(&s_tWork_FW_Update);
    
    gpio_free(ACCSNS_GPIO_INT);
    gpio_free(ACCSNS_GPIO_RST);

    if(accsns_wq_int != NULL){
        flush_workqueue(accsns_wq_int);
        destroy_workqueue(accsns_wq_int);
        accsns_wq_int = NULL;
    }
    if(accsns_wq != NULL){
        flush_workqueue(accsns_wq);
        destroy_workqueue(accsns_wq);
        accsns_wq = NULL;
    }
    if(fw_update_wq != NULL){
        flush_workqueue(fw_update_wq);
        destroy_workqueue(fw_update_wq);
        fw_update_wq = NULL;
    }

    if(accsns_wq_pause != NULL){
        flush_workqueue(accsns_wq_pause);
        destroy_workqueue(accsns_wq_pause);
        accsns_wq_pause = NULL;
    }

    spi_unregister_driver(&accsns_driver);
    
    client_accsns = NULL;
}
    
    
void accsns_debug_level_chg(int32_t lv)
{
#ifdef CONFIG_ML610Q792_DEBUG
    dbg_level = lv;
    printk("accsns_debug_level_chg level:%x\n", lv);
#endif
}

module_init(accsns_init);
module_exit(accsns_exit);

EXPORT_SYMBOL(accsns_dm_seqctrl);
EXPORT_SYMBOL(accsns_get_current_active);
EXPORT_SYMBOL(accsns_get_acceleration_data);
EXPORT_SYMBOL(accsns_get_pedometer_data);
EXPORT_SYMBOL(accsns_get_vehicle_data);
EXPORT_SYMBOL(accsns_get_move_data);
EXPORT_SYMBOL(accsns_activate);
EXPORT_SYMBOL(accsns_activate_pedom);
EXPORT_SYMBOL(accsns_activate_vehicle);
EXPORT_SYMBOL(accsns_calibration_mode);
EXPORT_SYMBOL(accsns_calibration_start);
EXPORT_SYMBOL(accsns_calibration_is_wait);
EXPORT_SYMBOL(accsns_calibration_is_comp);
EXPORT_SYMBOL(accsns_set_freq);
EXPORT_SYMBOL(accsns_set_offset);
EXPORT_SYMBOL(accsns_set_nv_params);
EXPORT_SYMBOL(accsns_set_delay);
EXPORT_SYMBOL(accsns_spi_read);
EXPORT_SYMBOL(accsns_spi_write);
EXPORT_SYMBOL(accsns_spi_error_check);
EXPORT_SYMBOL(accsns_update_fw_seq);
EXPORT_SYMBOL(accsns_update_fw);
EXPORT_SYMBOL(accsns_update_fw_async);
EXPORT_SYMBOL(accsns_update_fw_async_chk);
EXPORT_SYMBOL(accsns_get_fw_version);
EXPORT_SYMBOL(accsns_recovery_proc);
EXPORT_SYMBOL(accsns_check_accsensor);
EXPORT_SYMBOL(accsns_pedom_set_info);
EXPORT_SYMBOL(accsns_pedom_start_timer);
EXPORT_SYMBOL(accsns_pedom_clear);
EXPORT_SYMBOL(accsns_vehicle_clear);
EXPORT_SYMBOL(accsns_debug_level_chg);
EXPORT_SYMBOL(accsns_initialize);
EXPORT_SYMBOL(accsns_suspend);
EXPORT_SYMBOL(accsns_resume);
EXPORT_SYMBOL(accsns_io_poll_pedom);
EXPORT_SYMBOL(accsns_io_poll_vehicle);
EXPORT_SYMBOL(accsns_ddi_set_pause_proc);

MODULE_AUTHOR("KYOCERA");
MODULE_DESCRIPTION("AccSensor Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ML610Q792.c");


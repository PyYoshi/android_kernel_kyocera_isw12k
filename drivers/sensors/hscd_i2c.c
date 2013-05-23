/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 */
/*
 *  hscd_i2c.c - Linux kernel modules for magnetic field sensor
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
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/wait.h>
/* #define ALPS_DEBUG */

#define I2C_RETRIES  5

#define I2C_HSCD_ADDR (0x0c)
#define I2C_BUS_NUMBER 0

#define HSCD_DRIVER_NAME "hscd_i2c"

#define HSCD_XOUT  0x10
#define HSCD_YOUT  0x12
#define HSCD_ZOUT  0x14
#define HSCD_XOUT_H  0x11
#define HSCD_XOUT_L  0x10
#define HSCD_YOUT_H  0x13
#define HSCD_YOUT_L  0x12
#define HSCD_ZOUT_H  0x15
#define HSCD_ZOUT_L  0x14

#define HSCD_STATUS  0x18
#define HSCD_STATUS_POR (1 << 3)
#define HSCD_CTRL1  0x1b
#define HSCD_CTRL2  0x1c
#define HSCD_CTRL2_TDEN (0 << 7)
#define HSCD_CTRL2_XDEN (1 << 6)
#define HSCD_CTRL2_YDEN (1 << 5)
#define HSCD_CTRL2_ZDEN (1 << 4)
#define HSCD_CTRL2_DREN (1 << 3)
#define HSCD_CTRL2_DRP  (1 << 2)
#define HSCD_CTRL2_DRHD (1 << 1)
#define HSCD_CTRL2_DRTS (1 << 0)

#define HSCD_CTRL3  0x1d

#define HSCD_INL    0x1a
#define HSCD_INC1   0x1f
#define HSCD_ITHRL  0x26
#define HSCD_ITHRH  0x27
#define HSCD_OFFX               0x20

#ifdef HSCDTD002A
#define HSCD_CTRL4 0x28
#endif

#define CONFIG_DATA_BYTE_LENGTH 2
#define CONFIG_X_DATA_INDEX_NO  0
#define CONFIG_Y_DATA_INDEX_NO  1
#define CONFIG_Z_DATA_INDEX_NO  2
#define RESET_ASSERT_WAIT_MSEC 1
#define RESET_NEGATE_WAIT_MSEC 3
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define HSCD_DRDY_PM8058_GPIO  (PM8058_GPIO_PM_TO_SYS(25 - 1))
#define HSCD_RESET_PM8058_GPIO (PM8058_GPIO_PM_TO_SYS( 6 - 1))
#define CONFIG_I2C_ERROR_RESTART_MAX_COUNT 3
#define I2C_MSG_READ_NUM 2
#define I2C_MSG_WRITE_NUM 1
#define LITTLE_WAIT 1
#define I2C_REGIST_LENGTH 2
#define HSCD_DELAY_DEF 200
#define HSCD_DELAY_TIME 50
static void restart_device(void);
static void reset_device(void);
static struct i2c_driver hscd_driver;
static struct i2c_client *client_hscd = NULL;

static atomic_t flgEna;
static atomic_t g_CurrentSensorEnable;
static atomic_t delay;
static atomic_t tempDisable;

static struct work_struct g_stMag_work;

static int32_t g_nMagX = 0;
static int32_t g_nMagY = 0;
static int32_t g_nMagZ = 0;

static int32_t g_nIntIrqNo;
static atomic_t g_bIsIntIrqEnable;

static int32_t g_nErrCnt = 0;

static void enable_int_irq
(
 void
 )
{
  atomic_set(&g_bIsIntIrqEnable,true);
  enable_irq(g_nIntIrqNo);
}

static void disable_int_irq
(
 void
 )
{
  disable_irq(g_nIntIrqNo);
  atomic_set(&g_bIsIntIrqEnable,false);
}

static irqreturn_t int_irq_handler
(
 int32_t arg_nIrq,
 void * arg_vpData
 )
{
  if(arg_nIrq != g_nIntIrqNo) {
    return IRQ_NONE;
  } else {
    if(atomic_read(&g_bIsIntIrqEnable) == false) {
      disable_int_irq();
    } else {
      schedule_work(&g_stMag_work);
    }
    return IRQ_HANDLED;
  }
}

static void hscd_irq_init
(
 void
 )
{
  int32_t nRet;
  nRet = gpio_request(HSCD_DRDY_PM8058_GPIO,
                      HSCD_DRIVER_NAME);
  if (nRet < 0) {
    printk("[%s] failed to request GPIO=%d, ret=%d\n",
           __FUNCTION__,
           HSCD_DRDY_PM8058_GPIO,
           nRet);
  }
  nRet = gpio_direction_input(HSCD_DRDY_PM8058_GPIO);
  if (nRet < 0) {
    printk("[%s] failed to configure direction for GPIO=%d, ret=%d\n",
           __FUNCTION__,
           HSCD_DRDY_PM8058_GPIO,
           nRet);
  }
  g_nIntIrqNo = gpio_to_irq(HSCD_DRDY_PM8058_GPIO);
#ifdef NOTDEF
  printk("%s g_nIntIrqNo=%d\n",__FUNCTION__,g_nIntIrqNo);
#endif
  atomic_set(&g_bIsIntIrqEnable,false);

  nRet = request_any_context_irq(g_nIntIrqNo,
                                 int_irq_handler,
                                 IRQF_TRIGGER_RISING,
                                 HSCD_DRIVER_NAME,
                                 NULL);
  if(nRet < 0) {
    printk("[%s] failed to request GPIO=%d, ret=%d\n",
           __FUNCTION__,
           HSCD_DRDY_PM8058_GPIO,
           nRet);
  } else {
    disable_int_irq();
  }
}

static void hscd_gpio_init(void)
{
    int nRet;
    
    nRet = gpio_request(HSCD_RESET_PM8058_GPIO, HSCD_DRIVER_NAME);
    if (nRet < 0){
        printk("[HSCD] GPIO_8 Req Error=%d\n",nRet);
    }
    
    nRet = gpio_direction_output(HSCD_RESET_PM8058_GPIO, 1);
    if(nRet<0){
        printk("[HSCD] GPIO_8 Error\n");
    }
}

int32_t hscd_i2c_readm(char *rxData, int32_t length)
{
  int32_t err;
  int32_t tries = 0;
#ifdef ALPS_DEBUG
  int32_t iRegAdrs = (int)(*rxData);
#endif
  
  struct i2c_msg msgs[] = {
    {
      .addr = client_hscd->addr,
      .flags = 0,
      .len = 1,
      .buf = rxData,
    },
    {
      .addr = client_hscd->addr,
      .flags = I2C_M_RD,
      .len = length,
      .buf = rxData,
    },
  };
  
  do {
    err = i2c_transfer(client_hscd->adapter, msgs, I2C_MSG_READ_NUM);
    if(err != I2C_MSG_READ_NUM) {
        msleep(LITTLE_WAIT);
    }
  } while ((err != I2C_MSG_READ_NUM) && (++tries < I2C_RETRIES));
  
  if (err != I2C_MSG_READ_NUM) {
    dev_err(&client_hscd->adapter->dev, "read transfer error\n");
    err = -EIO;
  } else {
    err = 0;
#ifdef ALPS_DEBUG
    {
      int32_t i;
      printk("[HSCD] i2c_readm\n");
      for (i=0; i<length;i++) printk("          R[%02X] %02X\n", i+iRegAdrs,rxData[i] & 0xff);
      printk("\n");
    }
#endif
  }
  
  if(err) {
    if(++g_nErrCnt <= CONFIG_I2C_ERROR_RESTART_MAX_COUNT) {
      restart_device();
    } 
  } else {
    g_nErrCnt = 0;
  }
  return err;
}

int32_t hscd_i2c_writem(char *txData, int32_t length)
{
  int32_t err;
  int32_t tries = 0;
#ifdef ALPS_DEBUG
  int32_t i;
#endif
  
  struct i2c_msg msg[] = {
    {
      .addr = client_hscd->addr,
      .flags = 0,
      .len = length,
      .buf = txData,
    },
  };
  
#ifdef ALPS_DEBUG
  printk("[HSCD] i2c_writem\n");
  for (i=0; i<length;i++) printk("W %02X, \n",txData[i] & 0xff);
  printk("\n");
#endif
  
  do {
    err = i2c_transfer(client_hscd->adapter, msg, I2C_MSG_WRITE_NUM);
    if(err != I2C_MSG_WRITE_NUM) {
      msleep(LITTLE_WAIT);
    }
  } while ((err != I2C_MSG_WRITE_NUM) && (++tries < I2C_RETRIES));
  
  if (err != I2C_MSG_WRITE_NUM) {
    dev_err(&client_hscd->adapter->dev, "write transfer error\n");
    err = -EIO;
  } else {
    err = 0;
  }
  
  if(err) {
    if(++g_nErrCnt <= CONFIG_I2C_ERROR_RESTART_MAX_COUNT) {
      restart_device();
    }
  } else {
    g_nErrCnt = 0;      
  }
  return err;
}

int32_t hscd_set_magnetic_field_offset_Xdata
(
 int32_t arg_nOffsetData
 )
{
  int32_t nRet;
  u8 ucaBuffer[CONFIG_DATA_BYTE_LENGTH + 1];
  
  ucaBuffer[0] = HSCD_OFFX;
  memcpy(&ucaBuffer[1],&arg_nOffsetData,CONFIG_DATA_BYTE_LENGTH);
  nRet = hscd_i2c_writem(&ucaBuffer[0],CONFIG_DATA_BYTE_LENGTH + 1);
  return nRet;
}

int32_t hscd_set_magnetic_field_offset_Ydata
(
 int32_t arg_nOffsetData
 )
{
  int32_t nRet;
  u8 ucaBuffer[CONFIG_DATA_BYTE_LENGTH + 1];
  
  ucaBuffer[0] = HSCD_OFFX + (CONFIG_DATA_BYTE_LENGTH * CONFIG_Y_DATA_INDEX_NO);
  memcpy(&ucaBuffer[1],&arg_nOffsetData,CONFIG_DATA_BYTE_LENGTH);
  nRet = hscd_i2c_writem(&ucaBuffer[0],CONFIG_DATA_BYTE_LENGTH + 1);
  return nRet;
}

int32_t hscd_set_magnetic_field_offset_Zdata
(
 int32_t arg_nOffsetData
 )
{
  int32_t nRet;
  u8 ucaBuffer[CONFIG_DATA_BYTE_LENGTH + 1];
  
  ucaBuffer[0] = HSCD_OFFX + (CONFIG_DATA_BYTE_LENGTH * CONFIG_Z_DATA_INDEX_NO);
  memcpy(&ucaBuffer[1],&arg_nOffsetData,CONFIG_DATA_BYTE_LENGTH);
  nRet = hscd_i2c_writem(&ucaBuffer[0],CONFIG_DATA_BYTE_LENGTH + 1);
  return nRet;
}

int32_t hscd_set_temp_correct
(
 void
 )
{
  atomic_set(&tempDisable, 1);
  return 0;
}

int32_t hscd_get_magnetic_field_offset_data
(
 int32_t* arg_npOffsetData
 )
{
  int32_t nRet,i;
  u8 ucaBuffer[CONFIG_DATA_BYTE_LENGTH * (CONFIG_Z_DATA_INDEX_NO + 1)];
  
  ucaBuffer[0] = HSCD_OFFX;
  nRet = hscd_i2c_readm(ucaBuffer, CONFIG_DATA_BYTE_LENGTH * (CONFIG_Z_DATA_INDEX_NO + 1));
  for (i=0; i<(CONFIG_Z_DATA_INDEX_NO + 1); i++) {
    *(arg_npOffsetData + i) = (int) ((short)((ucaBuffer[CONFIG_DATA_BYTE_LENGTH*i+1] << 8) | (ucaBuffer[CONFIG_DATA_BYTE_LENGTH*i])));
  }
  
#ifdef ALPS_DEBUG
  printk("Mag_I2C, ox:%d, oy:%d, oz:%d\n",
         *(arg_npOffsetData),
         *(arg_npOffsetData + 1),
         *(arg_npOffsetData + 2));
#endif
  return nRet;
}

int32_t hscd_get_magnetic_field_data
(
 int32_t *arg_nXYZ
 )
{
  *(arg_nXYZ + CONFIG_X_DATA_INDEX_NO) = g_nMagX;
  *(arg_nXYZ + CONFIG_Y_DATA_INDEX_NO) = g_nMagY;
  *(arg_nXYZ + CONFIG_Z_DATA_INDEX_NO) = g_nMagZ;
  return 0;
}

int32_t hscd_get_magnetic_field_data_from_reg
(
 int32_t *arg_nXYZ
 )
{
  int32_t nErr = -1;
  int32_t i;
  u8 sx[CONFIG_DATA_BYTE_LENGTH * (CONFIG_Z_DATA_INDEX_NO + 1)];
  
  sx[0] = HSCD_XOUT;
  nErr = hscd_i2c_readm(sx, CONFIG_DATA_BYTE_LENGTH * (CONFIG_Z_DATA_INDEX_NO + 1));
  if (nErr < 0) return nErr;
  for (i=0; i<(CONFIG_Z_DATA_INDEX_NO + 1); i++) {
    arg_nXYZ[i] = (int) ((short)((sx[CONFIG_DATA_BYTE_LENGTH*i+1] << 8) | (sx[CONFIG_DATA_BYTE_LENGTH*i])));
  }
  
#ifdef ALPS_DEBUG
  printk("Mag_I2C, x:%d, y:%d, z:%d\n",arg_nXYZ[0], arg_nXYZ[1], arg_nXYZ[2]);
#endif
  return nErr;
}

void hscd_activate
(
 int32_t flgatm, 
 int32_t flg, 
 int32_t dtime
)
{
  u8 buf[I2C_REGIST_LENGTH];
  int32_t nFormerSensorEnable;

#ifdef ALPS_DEBUG
    printk("[HSCD] hscd_activate start\n");
#endif

  dtime = HSCD_DELAY_TIME;

  if (flg != 0) flg = 1;
  
  if      (dtime <=  10) buf[1] = (0x60 | 3<<2);
  else if (dtime <=  20) buf[1] = (0x60 | 2<<2);
  else if (dtime <=  60) buf[1] = (0x60 | 1<<2);
  else                   buf[1] = (0x60 | 0<<2);
  buf[0]  = HSCD_CTRL1;
  buf[1] |= (flg<<7);
  hscd_i2c_writem(buf, 2);
  msleep(1);
  
  if ((flg) && (!(atomic_read(&tempDisable))))  {
    buf[0] = HSCD_CTRL3;
    buf[1] = 0x02;
    hscd_i2c_writem(buf, I2C_REGIST_LENGTH);
  }
  
  if (flgatm) {
    atomic_set(&flgEna, flg);
    atomic_set(&delay, dtime);
  }
  nFormerSensorEnable = atomic_read(&g_CurrentSensorEnable);
  atomic_set(&g_CurrentSensorEnable,flg);
  if((nFormerSensorEnable != 1) &&
     (flg == 1)) {
    enable_int_irq();
  } else if((nFormerSensorEnable != 0) &&
            (flg == 0)) {
    disable_int_irq();
    cancel_work_sync(&g_stMag_work);
  }
}

static void hscd_register_init
(
 void
 )
{
#ifdef ALPS_DEBUG
  printk("[HSCD] register_init\n");
#endif
  {
    u8 buf[I2C_REGIST_LENGTH];
    
    int32_t nErr = 0;
    int32_t nErrCnt = 0;
    do {
      memset(buf,0,sizeof(buf));
      buf[0] = HSCD_STATUS;
      hscd_i2c_readm(buf, 1);
      nErr = buf[0] & HSCD_STATUS_POR;
      if(!nErr) {
        printk("[HSCD] reset_device STATUS:%X\n",buf[0]);
        reset_device();
      }
    } while(!nErr && (++nErrCnt <= CONFIG_I2C_ERROR_RESTART_MAX_COUNT));

    if(!nErr && (nErrCnt > CONFIG_I2C_ERROR_RESTART_MAX_COUNT)) {
      printk("[HSCD] register_init device err STATUS=%X\n",buf[0]);
        return ;
    }

    buf[0] = HSCD_CTRL2;
    buf[1] = HSCD_CTRL2_TDEN +
      HSCD_CTRL2_XDEN +
      HSCD_CTRL2_YDEN +
      HSCD_CTRL2_ZDEN +
      HSCD_CTRL2_DREN +
      HSCD_CTRL2_DRP;

    hscd_i2c_writem(buf, I2C_REGIST_LENGTH);
  }
}

static int32_t hscd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  int32_t d[CONFIG_Z_DATA_INDEX_NO+1];
  
  printk("[HSCD] probe\n");
  if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
    dev_err(&client->adapter->dev, "client not i2c capable\n");
    return -ENOMEM;
  }
  
  i2c_set_clientdata(client, client_hscd);
  
  client_hscd = client;
 
  hscd_gpio_init();

  hscd_register_init();
  hscd_irq_init();
  
  dev_info(&client->adapter->dev, "detected HSCD magnetic field sensor\n");
  
  hscd_activate(0, 1, atomic_read(&delay));
  hscd_get_magnetic_field_data_from_reg(d);
  printk("[HSCD] x:%d y:%d z:%d\n",d[0],d[1],d[2]);
  hscd_activate(0, 0, atomic_read(&delay));
  
  return 0;
}

static int32_t __devexit hscd_remove(struct i2c_client *client)
{
#ifdef ALPS_DEBUG
  printk("[HSCD] remove\n");
#endif
  return 0;
}

#ifdef CONFIG_PM
static int hscd_suspend(struct device *dev)
{
#ifdef ALPS_DEBUG
  printk("[HSCD] suspend\n");
#endif
  if(atomic_read(&g_CurrentSensorEnable) == 1) {
    hscd_activate(0, 0, atomic_read(&delay));
  }
  return 0;
}

static int hscd_resume(struct device *dev)
{
  int32_t nEnable;
#ifdef ALPS_DEBUG
  printk("[HSCD] resume\n");
#endif
  nEnable = atomic_read(&flgEna);
  if(nEnable == 1) {
    hscd_activate(0, nEnable, atomic_read(&delay));
  }
  return 0;
}
#endif

static const struct i2c_device_id ALPS_id[] = {
  { HSCD_DRIVER_NAME, 0 },
  { }
};

#ifdef CONFIG_PM_RUNTIME
static int hscd_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: idle...\n");
	return 0;
}

static int hscd_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int hscd_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}
#endif

static const struct dev_pm_ops hscd_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(
		hscd_suspend,
		hscd_resume
	)
	SET_RUNTIME_PM_OPS(
		hscd_runtime_suspend,
		hscd_runtime_resume,
		hscd_runtime_idle
	)
};

static struct i2c_driver hscd_driver = {
  .probe    = hscd_probe,
  .remove   = hscd_remove,
  .id_table = ALPS_id,
  .driver   = {
    .name = HSCD_DRIVER_NAME,
    .pm = &hscd_dev_pm_ops,
  },
};

static void waitMsec
(
 unsigned long arg_ulMsec
 )
{
  msleep(arg_ulMsec);
}

static void reset_device
(
 void
 )
{
  gpio_set_value_cansleep(HSCD_RESET_PM8058_GPIO, 0);
  waitMsec(RESET_NEGATE_WAIT_MSEC);

  gpio_set_value_cansleep(HSCD_RESET_PM8058_GPIO, 1);
  waitMsec(RESET_ASSERT_WAIT_MSEC);
  
  gpio_set_value_cansleep(HSCD_RESET_PM8058_GPIO, 0);
  waitMsec(RESET_NEGATE_WAIT_MSEC);
}

static void restart_device
(
 void
 )
{
  reset_device();
  hscd_register_init();
  hscd_activate(0, atomic_read(&flgEna), atomic_read(&delay));
}

static void do_mag_work
(
 struct work_struct *work
 )
{
  int32_t nXYZ[CONFIG_Z_DATA_INDEX_NO+1];

  if(atomic_read(&g_CurrentSensorEnable) == 1) {
    if(hscd_get_magnetic_field_data_from_reg(nXYZ) == 0) {
      g_nMagX = nXYZ[CONFIG_X_DATA_INDEX_NO];
      g_nMagY = nXYZ[CONFIG_Y_DATA_INDEX_NO];
      g_nMagZ = nXYZ[CONFIG_Z_DATA_INDEX_NO];
    }
  }
}

static int32_t __init mag_work_init
(
 void
 )
{
  INIT_WORK(&g_stMag_work, do_mag_work);
  return 0;
}

static void __exit mag_work_exit
(
 void
 )
{
}

static int32_t __init hscd_init
(
 void
 )
{
  struct i2c_board_info i2c_info;
  struct i2c_adapter *adapter;
  int32_t rc;
  
#ifdef ALPS_DEBUG
  printk("[HSCD] init\n");
#endif
  reset_device();
  
  atomic_set(&flgEna, 0);
  atomic_set(&g_CurrentSensorEnable,-1);
  atomic_set(&delay, HSCD_DELAY_DEF);
  atomic_set(&tempDisable, 0);
  
  rc = i2c_add_driver(&hscd_driver);
  if (rc != 0) {
    printk("can't add i2c driver\n");
    rc = -ENOTSUPP;
    return rc;
  }
  
  memset(&i2c_info, 0, sizeof(struct i2c_board_info));
  i2c_info.addr = I2C_HSCD_ADDR;
  strlcpy(i2c_info.type, HSCD_DRIVER_NAME , I2C_NAME_SIZE);
  
  adapter = i2c_get_adapter(I2C_BUS_NUMBER);
  if (!adapter) {
    printk("can't get i2c adapter %d\n", I2C_BUS_NUMBER);
    rc = -ENOTSUPP;
    goto probe_done;
  }
  client_hscd = i2c_new_device(adapter, &i2c_info);
  client_hscd->adapter->timeout = 0;
  client_hscd->adapter->retries = 0;
  
  i2c_put_adapter(adapter);
  if (!client_hscd) {
    printk("can't add i2c device at 0x%x\n",(unsigned int)i2c_info.addr);
    rc = -ENOTSUPP;  
  }
 probe_done: 
  mag_work_init();
#ifdef ALPS_DEBUG
  printk("hscd_open Init end!!!!\n");
#endif
  return rc;
}

static void __exit hscd_exit(void)
{
#ifdef ALPS_DEBUG
  printk("[HSCD] exit\n");
#endif
  mag_work_exit();
  i2c_del_driver(&hscd_driver);

  i2c_unregister_device(client_hscd);
  client_hscd = NULL;
}

module_init(hscd_init);
module_exit(hscd_exit);

EXPORT_SYMBOL(hscd_get_magnetic_field_data);
EXPORT_SYMBOL(hscd_get_magnetic_field_offset_data);
EXPORT_SYMBOL(hscd_set_magnetic_field_offset_Xdata);
EXPORT_SYMBOL(hscd_set_magnetic_field_offset_Ydata);
EXPORT_SYMBOL(hscd_set_magnetic_field_offset_Zdata);
EXPORT_SYMBOL(hscd_activate);
EXPORT_SYMBOL(hscd_i2c_readm);
EXPORT_SYMBOL(hscd_i2c_writem);
EXPORT_SYMBOL(hscd_suspend);
EXPORT_SYMBOL(hscd_resume);
EXPORT_SYMBOL(hscd_set_temp_correct);

MODULE_DESCRIPTION("Alps hscd Device");
MODULE_AUTHOR("ALPS");
MODULE_LICENSE("GPL v2");

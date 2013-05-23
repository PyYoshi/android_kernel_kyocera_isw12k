/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 */
/*
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


#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <mach/vreg.h>
#include <linux/regulator/consumer.h>
#include "rj6cba200.h"

#define  RJ6CBA200_LOG_ON
#define  RJ6CBA200_CALL_LOG_ON
//#define  RJ6CBA200_DUMP_I2C_MESSAGE

/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define PMIC_OFF    0
#define PMIC_ON     1

/*============================================================================
							DATA DECLARATIONS
============================================================================*/
struct rj6cba200_work_t {
	struct work_struct work;
};
static struct  rj6cba200_work_t *rj6cba200_sensorw;
static struct  i2c_client *rj6cba200_client;
struct rj6cba200_ctrl_t {
	const struct  msm_camera_sensor_info_rj6cba200 *sensordata;
};
static struct rj6cba200_ctrl_t *rj6cba200_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(rj6cba200_wait_queue);
DEFINE_MUTEX(rj6cba200_mut);
static struct regulator *vreg_cfg_vcamd;
static struct regulator *vreg_cfg_vcama;


/*============================================================================
							Functions
============================================================================*/
static int rj6cba200_i2c_rxdata(unsigned short saddr, unsigned char *rxdata, int length);
static int32_t rj6cba200_i2c_txdata(unsigned short saddr, unsigned char *txdata, int length);
static int32_t rj6cba200_i2c_read(uint8_t raddr, uint8_t *rdata, int rlen);
static int32_t rj6cba200_i2c_write_b_sensor(uint8_t waddr, uint8_t bdata);
static int32_t rj6cba200_power_down(void);
static int rj6cba200_probe_init_done(const struct msm_camera_sensor_info *data);
static int rj6cba200_probe_init_sensor(const struct msm_camera_sensor_info *data);
int rj6cba200_sensor_init(const struct msm_camera_sensor_info *data);
static int rj6cba200_init_client(struct i2c_client *client);
static int rj6cba200_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __exit rj6cba200_i2c_remove(struct i2c_client *client);
int rj6cba200_sensor_config(void __user *argp);
static int rj6cba200_sensor_set_pmic( struct regulator *vreg_cfg, int min_uV, int max_uV);
static int rj6cba200_sensor_i2c_write(struct sensor_i2c_wr_cfg *i2c_wr_cfg);
static long rj6cba200_sensor_i2c_read(struct sensor_i2c_rd_cfg *i2c_rd_cfg);
static int rj6cba200_sensor_release(void);
static int rj6cba200_sensor_probe(const struct msm_camera_sensor_info *info, struct msm_sensor_ctrl *s);
static int __rj6cba200_probe(struct platform_device *pdev);
static int __init rj6cba200_init(void);

/*=============================================================*/

static int rj6cba200_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;

	RJ6CBA200_ENTER;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		RJ6CBA200_LOG_ERR("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	rj6cba200_sensorw = kzalloc(sizeof(struct rj6cba200_work_t), GFP_KERNEL);
	if (!rj6cba200_sensorw) {
		RJ6CBA200_LOG_ERR("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, rj6cba200_sensorw);
	rj6cba200_init_client(client);
	rj6cba200_client = client;

	mdelay(50);

	RJ6CBA200_LOG_DBG("rj6cba200_probe successed! rc = %d\n", rc);

	RJ6CBA200_RETURN_N(0);

probe_failure:
	RJ6CBA200_LOG_ERR("rj6cba200_probe failed! rc = %d\n", rc);

	RJ6CBA200_RETURN_N(rc);
}

static int __exit rj6cba200_i2c_remove(struct i2c_client *client)
{
	struct rj6cba200_work_t_t *sensorw;
	
	RJ6CBA200_ENTER;

	sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	rj6cba200_client = NULL;
	kfree(sensorw);

	RJ6CBA200_RETURN_N(0);
}

static int rj6cba200_init_client(struct i2c_client *client)
{
	RJ6CBA200_ENTER;

	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&rj6cba200_wait_queue);

	RJ6CBA200_RETURN_N(0);
}

static const struct i2c_device_id rj6cba200_i2c_id[] = {
	{"rj6cba200", 0},
	{ }
};

static int rj6cba200_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
#ifdef RJ6CBA200_DUMP_I2C_MESSAGE
    int i;
#endif

	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 1,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = rxdata,
		},
	};

	RJ6CBA200_ENTER;

#ifdef RJ6CBA200_DUMP_I2C_MESSAGE
    RJ6CBA200_LOG_DBG("slave_addr[0x%04X], len[0x%04X]", saddr, length);
    RJ6CBA200_LOG_DBG("reg_addr[0x%04X]", *rxdata);
#endif

	if (i2c_transfer(rj6cba200_client->adapter, msgs, 2) < 0) {
		RJ6CBA200_LOG_ERR("rj6cba200_i2c_rxdata failed!\n");
		RJ6CBA200_RETURN_N(-EIO);
	}

#ifdef RJ6CBA200_DUMP_I2C_MESSAGE
    for(i=0; i<length; i++)
    {
        RJ6CBA200_LOG_DBG("[0x%02X]", *(rxdata+i));
    }
#endif
	
	RJ6CBA200_RETURN_N(0);
}
static int32_t rj6cba200_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{

	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = 2,
			.buf = txdata,
		},
	};

	RJ6CBA200_ENTER;

	if (i2c_transfer(rj6cba200_client->adapter, msg, 1) < 0) {
		RJ6CBA200_LOG_ERR("rj6cba200_i2c_txdata faild 0x%x\n", rj6cba200_client->addr);
		RJ6CBA200_RETURN_N(-EIO);
	}

	RJ6CBA200_RETURN_N(0);
}

static int32_t rj6cba200_i2c_read(uint8_t raddr,
	uint8_t *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[1];

	RJ6CBA200_ENTER;

	if (!rdata)
		RJ6CBA200_RETURN_N(-EIO);
	memset(buf, 0, sizeof(buf));
	buf[0] = raddr;
	rc = rj6cba200_i2c_rxdata(rj6cba200_client->addr, buf, rlen);
	if (rc < 0) {
		RJ6CBA200_LOG_ERR("rj6cba200_i2c_read 0x%x failed!\n", raddr);
		RJ6CBA200_RETURN_N(rc);
	}
	*rdata = buf[0];
	
	RJ6CBA200_RETURN_N(rc);
}
static int32_t rj6cba200_i2c_write_b_sensor(uint8_t waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[2];

	RJ6CBA200_ENTER;

	memset(buf, 0, sizeof(buf));
	buf[0] = waddr;
	buf[1] = bdata;
	RJ6CBA200_LOG_DBG("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, bdata);
	rc = rj6cba200_i2c_txdata(rj6cba200_client->addr, buf, 2);
	if (rc < 0)
		RJ6CBA200_LOG_ERR("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);
	
	RJ6CBA200_RETURN_N(rc);
}

static int32_t rj6cba200_power_down(void)
{
	RJ6CBA200_ENTER;
	RJ6CBA200_RETURN_N(0);
}
static int rj6cba200_probe_init_done(const struct msm_camera_sensor_info *data)
{
	RJ6CBA200_ENTER;
	RJ6CBA200_RETURN_N(0);
}

static int rj6cba200_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info_rj6cba200 *sensordata;
	struct sensor_cfg_data cfg_data;
	struct clk *clk = NULL;
	spinlock_t rj6cba200_spinlock;
	unsigned long flags;
	
	RJ6CBA200_ENTER;

	sensordata = (struct msm_camera_sensor_info_rj6cba200 *)data;

	spin_lock_init(&rj6cba200_spinlock);

	rc = gpio_request(sensordata->sensor_info.sensor_reset, "rj6cba200");
	if (!rc) {
		gpio_direction_output(sensordata->sensor_info.sensor_reset, 0);
		gpio_set_value(sensordata->sensor_info.sensor_reset, 1);
	} else {
		RJ6CBA200_LOG_ERR("gpio reset fail");
	}

	spin_lock_irqsave(&rj6cba200_spinlock, flags);

	RJ6CBA200_LOG_DBG("gpio reset success");
	/* Wait:1ms */
	mdelay(1);

	/* VCAMD provide start */
	cfg_data.cfg.pmic_cfg.id = rj6cba200_ctrl->sensordata->vcmd_pwd;
	cfg_data.cfg.pmic_cfg.ctl =  PMIC_ON;
        vreg_cfg_vcamd = regulator_get(NULL, cfg_data.cfg.pmic_cfg.id);
	rc = rj6cba200_sensor_set_pmic( vreg_cfg_vcamd, 1800000, 1800000 );
	if(rc != 0){
		RJ6CBA200_RETURN_N(rc);
	}
	RJ6CBA200_LOG_DBG("VCAMD provide");

        /* Wait:100us */
	udelay(100);
	
	/* VCAMA provide start */
	cfg_data.cfg.pmic_cfg.id = rj6cba200_ctrl->sensordata->vcma_pwd;
	cfg_data.cfg.pmic_cfg.ctl =  PMIC_ON;
        vreg_cfg_vcama = regulator_get(NULL, cfg_data.cfg.pmic_cfg.id);
	rc = rj6cba200_sensor_set_pmic( vreg_cfg_vcama, 2800000, 2800000 );
	if(rc != 0){
		RJ6CBA200_RETURN_N(rc);
	}

	spin_unlock_irqrestore(&rj6cba200_spinlock, flags);

	RJ6CBA200_LOG_DBG("VCAMA provide");
	mdelay(5);

	gpio_tlmm_config(GPIO_CFG(16, 3, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), GPIO_CFG_ENABLE);

	/* enable mclk */
	clk = clk_get(NULL, "core_clk");
	msm_camio_clk_rate_set_2(clk, 24000000);
	if(!IS_ERR(clk))
		clk_enable(clk);

        /* Wait:5ms */
        msleep(5);

        /* Reset CAMIF PAD REG */
        msm_camio_camif_pad_reg_reset();

        /* Wait:5ms */
        msleep(5);

	RJ6CBA200_LOG_DBG("sensor_reset = %d\n", sensordata->sensor_info.sensor_reset);
	gpio_set_value(sensordata->sensor_info.sensor_reset, 0);

	/* Wait:5ms */
	mdelay(5);

	RJ6CBA200_RETURN_N(rc);
}

int rj6cba200_sensor_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	RJ6CBA200_ENTER;
	RJ6CBA200_LOG_DBG("pdata: %p\n", data);

	rj6cba200_ctrl = kzalloc(sizeof(struct rj6cba200_ctrl_t), GFP_KERNEL);
	if (!rj6cba200_ctrl) {
		RJ6CBA200_LOG_ERR("rj6cba200_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		rj6cba200_ctrl->sensordata = (struct msm_camera_sensor_info_rj6cba200 *)data;

	rc = rj6cba200_probe_init_sensor(data);
	if (rc < 0) {
		RJ6CBA200_LOG_ERR("Calling rj6cba200_sensor_init fail\n");
		goto init_fail;
	} else
		goto init_done;
init_fail:
	RJ6CBA200_LOG_ERR(" rj6cba200_sensor_init fail\n");
	rj6cba200_probe_init_done(data);
	kfree(rj6cba200_ctrl);
init_done:

	RJ6CBA200_RETURN_N(rc);
}

static int rj6cba200_sensor_release(void)
{
	int rc = -EBADF;
	struct sensor_cfg_data cfg_data;

	RJ6CBA200_ENTER;

	mutex_lock(&rj6cba200_mut);
	rj6cba200_power_down();
	gpio_free(rj6cba200_ctrl->sensordata->sensor_info.sensor_reset);

	/* disable mclk */
	clk_disable(clk_get(NULL, "core_clk"));

	/* Wait:5ms */
	mdelay(5);

	/* VCAMA provide stop */
	cfg_data.cfg.pmic_cfg.id = rj6cba200_ctrl->sensordata->vcma_pwd;
	cfg_data.cfg.pmic_cfg.ctl =  PMIC_OFF;
	rc = rj6cba200_sensor_set_pmic( vreg_cfg_vcama, -1, -1 );
	if(rc != 0){
		RJ6CBA200_RETURN_N(rc);
	}

	/* Wait:5ms */
	mdelay(5);

	/* VCAMD provide stop */
	cfg_data.cfg.pmic_cfg.id = rj6cba200_ctrl->sensordata->vcmd_pwd;
	cfg_data.cfg.pmic_cfg.ctl =  PMIC_OFF;
	rc = rj6cba200_sensor_set_pmic( vreg_cfg_vcamd, -1, -1 );
	if(rc != 0){
		RJ6CBA200_RETURN_N(rc);
	}
	
	kfree(rj6cba200_ctrl);
	rj6cba200_ctrl = NULL;

	mutex_unlock(&rj6cba200_mut);

	/* Wait:100ms */
	mdelay(100);

	RJ6CBA200_RETURN_N(rc);
}

static struct i2c_driver rj6cba200_i2c_driver = {
	.id_table = rj6cba200_i2c_id,
	.probe  = rj6cba200_i2c_probe,
	.remove = __exit_p(rj6cba200_i2c_remove),
	.driver = {
		.name = "rj6cba200",
	},
};

int rj6cba200_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;

	RJ6CBA200_ENTER;

	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		RJ6CBA200_RETURN_N(-EFAULT);
	mutex_lock(&rj6cba200_mut);
	
	RJ6CBA200_LOG_DBG("rj6cba200_sensor_config: cfgtype = %d\n",cdata.cfgtype);
	
	switch (cdata.cfgtype) {
	case CFG_I2C_WRITE:
		rc = rj6cba200_sensor_i2c_write( &cdata.cfg.i2c_wr_cfg );
		break;
	case CFG_I2C_READ:
		rc = rj6cba200_sensor_i2c_read( &cdata.cfg.i2c_rd_cfg );
		break;
	default:
		rc = -ENOTTY;
		break;
	}
	mutex_unlock(&rj6cba200_mut);

	RJ6CBA200_RETURN_N(rc);
}

/*
* rj6cba200_sensor_set_pmic
*/
static int rj6cba200_sensor_set_pmic(
     struct regulator *vreg_cfg, int min_uV, int max_uV)
{
    int ret = 0;

    RJ6CBA200_ENTER;

    if (IS_ERR_OR_NULL(vreg_cfg)) {
        RJ6CBA200_LOG_DBG("regulator_get error!");
        vreg_cfg = NULL;
        RJ6CBA200_RETURN_N(-1);
    }

    if ((min_uV < 0) || (max_uV < 0)) {
        ret = regulator_disable(vreg_cfg);
        if (ret) {
            RJ6CBA200_LOG_DBG("regulator disable error ret[%d]", ret);
        }
        goto regulator_free;
    } else {
        ret = regulator_set_voltage(vreg_cfg, min_uV, max_uV);
        if (ret) {
            RJ6CBA200_LOG_DBG("regulator set voltage error ret[%d]", ret);
            goto regulator_free;
        }

        ret = regulator_enable(vreg_cfg);
        if (ret) {
            RJ6CBA200_LOG_DBG("regulator_enable error ret[%d]", ret);
            goto regulator_free;
        }
    }

    RJ6CBA200_RETURN_N(ret);
regulator_free:
    regulator_put(vreg_cfg);
    vreg_cfg=NULL;
    RJ6CBA200_RETURN_N(ret);
}

/*
* rj6cba200_sensor_i2c_write
*/
static int rj6cba200_sensor_i2c_write(struct sensor_i2c_wr_cfg *i2c_wr_cfg)
{
	int rc = 0;
	unsigned char write_buf;

	RJ6CBA200_ENTER;

	if( !(rc < 0) ){
		if ( copy_from_user( &write_buf,
							 (void *)(i2c_wr_cfg->write_data_ptr),
							 1 ) ){
			rc = -EFAULT;
		}
	}

	if( !(rc < 0) ){
		rc = rj6cba200_i2c_write_b_sensor( i2c_wr_cfg->reg_addr,
											   write_buf);
	}

	RJ6CBA200_RETURN_N(rc);
}
/*
* rj6cba200_sensor_i2c_read
*/
static long rj6cba200_sensor_i2c_read(struct sensor_i2c_rd_cfg *i2c_rd_cfg)
{
	int   rc = 0;
	unsigned char *read_buf = NULL;

	RJ6CBA200_ENTER;

	read_buf = (unsigned char *)kzalloc((size_t)i2c_rd_cfg->len, GFP_KERNEL);
	if( !read_buf ){
		rc = -EFAULT;
	}

	if( !(rc < 0) ) {
		rc = rj6cba200_i2c_read( i2c_rd_cfg->reg_addr,
											   read_buf,
											   i2c_rd_cfg->len );
	}

	if( !(rc < 0) ) {
		if (copy_to_user((void *)(i2c_rd_cfg->read_data_ptr),
						  &read_buf[0],
						  i2c_rd_cfg->len) ){
			rc = -EFAULT;
		}
	}

	if( read_buf )
		kfree(read_buf);

	RJ6CBA200_RETURN_N(rc);
}

static int rj6cba200_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;

	RJ6CBA200_ENTER;

	rc = i2c_add_driver(&rj6cba200_i2c_driver);
	if (rc < 0 || rj6cba200_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_fail;
	}
	
	s->s_init = rj6cba200_sensor_init;
	s->s_release = rj6cba200_sensor_release;
	s->s_config  = rj6cba200_sensor_config;
	s->s_camera_type = FRONT_CAMERA_2D;
	s->s_mount_angle = 270;
	rj6cba200_probe_init_done(info);

	RJ6CBA200_RETURN_N(rc);

probe_fail:
	RJ6CBA200_LOG_ERR("rj6cba200_sensor_probe: SENSOR PROBE FAILS!\n");
	i2c_del_driver(&rj6cba200_i2c_driver);

	RJ6CBA200_RETURN_N(rc);
}

static int __rj6cba200_probe(struct platform_device *pdev)
{
	int rc;
	
	RJ6CBA200_ENTER;

	rc = msm_camera_drv_start(pdev, rj6cba200_sensor_probe);

	RJ6CBA200_RETURN_N(rc);
}

static struct platform_driver msm_camera_driver = {
	.probe = __rj6cba200_probe,
	.driver = {
		.name = "msm_camera_rj6cba200",
		.owner = THIS_MODULE,
	},
};

static int __init rj6cba200_init(void)
{
	int rc;
	
	RJ6CBA200_ENTER;

	rc = platform_driver_register(&msm_camera_driver);
	
	RJ6CBA200_RETURN_N(rc);
}

module_init(rj6cba200_init);

MODULE_DESCRIPTION("RJ6CBA200 VGA YUV sensor driver");
MODULE_LICENSE("GPL v2");


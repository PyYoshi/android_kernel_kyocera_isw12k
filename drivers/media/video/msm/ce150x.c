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
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <mach/camera.h>
#include <media/msm_camera.h>
#include <linux/interrupt.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <linux/wait.h>
#include <linux/param.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include "ce150x.h"

#define  CE150X_I2C_WR_PKT_HEADER  0
#define  CE150X_INTERRUPT_PORT  33
#define  CE150X_PMIC_PORT  6-1

#define  CE150X_LOG_ON
#define  CE150X_CALL_LOG_ON
//#define  CE150X_DUMP_I2C_MESSAGE

//#define  CE150X_PARTIAL_BLOCK_TRANCE
#define  CE150X_BLOCK_SIZE 10

#define  GPIO_LOW    0
#define  GPIO_HIGH   1
#define  PMIC_OFF    0
#define  PMIC_ON     1
#define  MCLK_OFF    0
#define  MCLK_ON     1

static struct i2c_client *ce150x_client;
static struct spi_device *ce150x_client_spi;
static struct regulator *vreg_cfg_vcaml2;
static struct regulator *vreg_cfg_vcamsd;
static struct regulator *vreg_cfg_vcamd;
static struct regulator *vreg_cfg_vcama;
static struct regulator *vreg_cfg_vcamaf;
static struct clk *camio_cam_m_clk;

static DECLARE_WAIT_QUEUE_HEAD(ce150x_wait_queue);
DEFINE_MUTEX(ce150x_mut);

static const struct i2c_device_id ce150x_i2c_id[] = {
    { "ce150x", 2},
    { },
};

static int ce150x_wait_flag = 0;

static struct msm_camera_sensor_info_ce150x *senser_info_ce150x;

/*
* Functions
*/
static int ce150x_i2c_probe(
    struct i2c_client *client,
    const struct i2c_device_id *id
);
static int __exit ce150x_i2c_remove(struct i2c_client *client);
static void ce150x_tasklet_proc( unsigned long data );
static irqreturn_t ce150x_interrupt(int irq, void *handle);
static int ce150x_sensor_init(
    const struct msm_camera_sensor_info* sensor_info
);
static int ce150x_sensor_release(void);
static int ce150x_sensor_i2c_write_transfer(
    unsigned short slave_addr,
    unsigned char *trans_buf,
    int len
);
static int ce150x_sensor_i2c_read_transfer(
    unsigned short slave_addr,
    unsigned char *read_buf,
    int len
);
static int ce150x_sensor_i2c_write(struct sensor_i2c_wr_cfg *i2c_wr_cfg);
static long ce150x_sensor_i2c_read(struct sensor_i2c_rd_cfg *i2c_rd_cfg);
static int ce150x_sensor_set_mclk( struct sensor_cfg_data* cfg_data );
static int ce150x_sensor_set_pmic( struct regulator *vreg_cfg, int min_uV, int max_uV);
static int ce150x_sensor_set_gpio( struct sensor_cfg_data* cfg_data );
static int ce150x_sensor_wait_interrput( struct sensor_cfg_data* cfg_data );

static int ce150x_spi_probe(struct spi_device *client);
static int ce150x_sensor_spi_write(struct sensor_spi_wr_cfg *spi_wr_cfg);
static int ce150x_sensor_spi_write_transfer(unsigned char *trans_buf, unsigned int len);
static int32_t ce150x_csi_setting(void);
static int ce150x_sensor_reset(void);

/*
* ce150x_i2c_probe
*/
static int ce150x_i2c_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    int rc = 0;

    CE150X_ENTER;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        rc = -ENOTSUPP;
    }

    ce150x_client = client;

    CE150X_RETURN_N(rc);
}

/*
* ce150x_i2c_remove
*/
static int __exit ce150x_i2c_remove(struct i2c_client *client)
{
    CE150X_ENTER;

    ce150x_client = NULL;

    CE150X_RETURN_N(0);
}

/*
* ce150x_i2c_driver
*/
static struct i2c_driver ce150x_i2c_driver = {
    .id_table = ce150x_i2c_id,
    .probe  = ce150x_i2c_probe,
    .remove = __exit_p(ce150x_i2c_remove),
    .driver = {
        .name = "ce150x",
    },
};

/*
* ce150x_tasklet_proc
*/
static void ce150x_tasklet_proc( unsigned long data )
{
    CE150X_ENTER;

    if (!gpio_get_value(CE150X_INTERRUPT_PORT)) {
        CE150X_RETURN;
    }

    ce150x_wait_flag = 1;
    wake_up_interruptible(&ce150x_wait_queue);

    CE150X_RETURN;
}

/*
* DECLARE_TASKLET
*/
DECLARE_TASKLET( ce150x_tasklet, ce150x_tasklet_proc, ( unsigned long )NULL );

/*
* ce150x_interrupt
*/
static irqreturn_t ce150x_interrupt(int irq, void *handle)
{
    CE150X_ENTER;

    tasklet_schedule( &ce150x_tasklet );

    CE150X_RETURN_N(IRQ_HANDLED);
}

/*
* ce150x_sensor_init
*  - It is called from OPEN.
*/
static int ce150x_sensor_init(
    const struct msm_camera_sensor_info* sensor_info
)
{
    long rc = 0;
    struct sensor_cfg_data cfg_data;

    CE150X_ENTER;

    senser_info_ce150x = (struct msm_camera_sensor_info_ce150x *)sensor_info;
    ce150x_wait_flag = 0;

    gpio_request(senser_info_ce150x->sensor_info.sensor_pwd, "PWD");
    gpio_request(senser_info_ce150x->sensor_info.sensor_reset, "RST");
    gpio_request(senser_info_ce150x->standby, "STB");

    CE150X_LOG_DBG("VCAML START");
    /* VCAML provide start */
    cfg_data.cfg.gpio_cfg.port = senser_info_ce150x->sensor_info.sensor_pwd;
    cfg_data.cfg.gpio_cfg.ctl = GPIO_HIGH;
    rc = ce150x_sensor_set_gpio( &cfg_data );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    /* Wait:50ms */
    msleep(50);

    CE150X_LOG_DBG("VCAML2 START");
    /* VCAML2 provide start */
    cfg_data.cfg.pmic_cfg.id = senser_info_ce150x->vcml2_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_ON;
    vreg_cfg_vcaml2 = regulator_get(NULL, cfg_data.cfg.pmic_cfg.id);
    rc = ce150x_sensor_set_pmic( vreg_cfg_vcaml2, 1200000, 1200000 );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    /* Wait:1ms */
    msleep(1);

    CE150X_LOG_DBG("VCAMSD START");
    /* VCAMSD provide start */
    cfg_data.cfg.pmic_cfg.id = senser_info_ce150x->vcmsd_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_ON;
    vreg_cfg_vcamsd = regulator_get(NULL, cfg_data.cfg.pmic_cfg.id);
    rc = ce150x_sensor_set_pmic( vreg_cfg_vcamsd, 1800000, 1800000 );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    /* Wait:1ms */
    msleep(1);

    CE150X_LOG_DBG("VCAMD START");
    /* VCAMD provide start */
    cfg_data.cfg.pmic_cfg.id = senser_info_ce150x->vcmd_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_ON;
    vreg_cfg_vcamd = regulator_get(NULL, cfg_data.cfg.pmic_cfg.id);
    rc = ce150x_sensor_set_pmic( vreg_cfg_vcamd, 1800000, 1800000 );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    /* Wait:1ms */
    msleep(1);
    
    CE150X_LOG_DBG("VCAMA START");
    /* VCAMA provide start */
    cfg_data.cfg.pmic_cfg.id = senser_info_ce150x->vcma_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_ON;
    vreg_cfg_vcama = regulator_get(NULL, cfg_data.cfg.pmic_cfg.id);
    rc = ce150x_sensor_set_pmic( vreg_cfg_vcama, 2800000, 2800000 );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    /* Wait:1ms */
    msleep(1);
    
    CE150X_LOG_DBG("VCAMAF START");
    /* VCAMAF provide start */
    cfg_data.cfg.pmic_cfg.id = senser_info_ce150x->vcmaf_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_ON;
    vreg_cfg_vcamaf = regulator_get(NULL, cfg_data.cfg.pmic_cfg.id);
    rc = ce150x_sensor_set_pmic( vreg_cfg_vcamaf, 2850000, 2850000 );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    
    /* Wait:1ms */
    msleep(1);
    
    CE150X_LOG_DBG("CLK START");
    /* CAM_CLK provide start */
    cfg_data.cfg.mclk_ctl = MCLK_ON;
    rc = ce150x_sensor_set_mclk( &cfg_data );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }

    /* Wait:1ms */
    msleep(1);

    /* standby cancel */
    cfg_data.cfg.gpio_cfg.port = senser_info_ce150x->standby;
    cfg_data.cfg.gpio_cfg.ctl = GPIO_HIGH;
    rc = ce150x_sensor_set_gpio( &cfg_data );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    
    /* Wait:1ms */
    msleep(1);

    /* reset cancel */
    cfg_data.cfg.gpio_cfg.port = senser_info_ce150x->sensor_info.sensor_reset;
    cfg_data.cfg.gpio_cfg.ctl = GPIO_HIGH;
    rc = ce150x_sensor_set_gpio( &cfg_data );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    
    /* Wait:10ms */
    msleep(15);

    rc = request_irq( gpio_to_irq(CE150X_INTERRUPT_PORT),
                      ce150x_interrupt,
                      IRQF_TRIGGER_RISING,
                      "cam_trig_h",
                      0 );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    
    CE150X_RETURN_N(rc);
}

/*
* ce150x_sensor_release
*  - It is called from CLOSE.
*/
static int ce150x_sensor_release(void)
{
    int rc = 0;
    struct sensor_cfg_data cfg_data;

    CE150X_ENTER;

    mutex_lock(&ce150x_mut);

    /* reset setting */
    cfg_data.cfg.gpio_cfg.port = senser_info_ce150x->sensor_info.sensor_reset;
    cfg_data.cfg.gpio_cfg.ctl = GPIO_LOW;
    rc = ce150x_sensor_set_gpio( &cfg_data );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }

    /* Wait:1ms */
    udelay(1);

    /* standby setting */
    cfg_data.cfg.gpio_cfg.port = senser_info_ce150x->standby;
    cfg_data.cfg.gpio_cfg.ctl = GPIO_LOW;
    rc = ce150x_sensor_set_gpio( &cfg_data );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }

    /* Wait:1ms */
    msleep(1);

    /* CAM_CLK provide stop */
    cfg_data.cfg.mclk_ctl = MCLK_OFF;
    rc = ce150x_sensor_set_mclk( &cfg_data );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }

    /* VCAMAF provide stop */
    cfg_data.cfg.pmic_cfg.id = senser_info_ce150x->vcmaf_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_OFF;
    rc = ce150x_sensor_set_pmic( vreg_cfg_vcamaf, -1, -1);
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    /* Wait:1ms */
    msleep(1);

    /* VCAMA provide stop */
    cfg_data.cfg.pmic_cfg.id = senser_info_ce150x->vcma_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_OFF;
    rc = ce150x_sensor_set_pmic( vreg_cfg_vcama, -1, -1);
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    /* Wait:1ms */
    msleep(1);
    
    /* VCAMD provide stop */
    cfg_data.cfg.pmic_cfg.id = senser_info_ce150x->vcmd_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_OFF;
    rc = ce150x_sensor_set_pmic( vreg_cfg_vcamd, -1, -1);
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    /* Wait:1ms */
    msleep(1);
    
    /* VCAMSD provide stop */
    cfg_data.cfg.pmic_cfg.id = senser_info_ce150x->vcmsd_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_OFF;
    rc = ce150x_sensor_set_pmic( vreg_cfg_vcamsd, -1, -1);
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    /* Wait:50ms */
    msleep(50);
    
    /* VCAML2 provide stop */
    cfg_data.cfg.pmic_cfg.id = senser_info_ce150x->vcml2_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_OFF;
    rc = ce150x_sensor_set_pmic( vreg_cfg_vcaml2, -1, -1);
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }

    /* VCAML provide stop */
    cfg_data.cfg.gpio_cfg.port = senser_info_ce150x->sensor_info.sensor_pwd;
    cfg_data.cfg.gpio_cfg.ctl = GPIO_LOW;
    rc = ce150x_sensor_set_gpio( &cfg_data );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    
    free_irq( gpio_to_irq(CE150X_INTERRUPT_PORT), 0 );
    tasklet_kill( &ce150x_tasklet );

    ce150x_wait_flag = 0;

    mutex_unlock(&ce150x_mut);

    CE150X_RETURN_N(rc);
}

/*
* ce150x_sensor_i2c_transfer
*/
static int ce150x_sensor_i2c_write_transfer(
    unsigned short slave_addr,
    unsigned char *trans_buf,
    int len
)
{
    int rc = 0;
#ifdef  CE150X_DUMP_I2C_MESSAGE
    int i;
#endif

    struct i2c_msg msg[] = {
        {
            .addr = slave_addr,
            .flags = 0,
            .len = len,
            .buf = trans_buf,
        },
    };

    CE150X_ENTER;

    if (i2c_transfer(ce150x_client->adapter, msg, 1) < 0) {
        rc = -EIO;
    }

#ifdef  CE150X_DUMP_I2C_MESSAGE
    CE150X_LOG_DBG("slave_addr[0x%04x], len[0x%04x]", slave_addr, len);

    for(i=0; i<len; i++)
    {
        CE150X_LOG_DBG("[0x%02x]", *(trans_buf+i));
    }
#endif

    CE150X_RETURN_N(rc);
}

/*
* ce150x_sensor_i2c_transfer
*/
static int ce150x_sensor_i2c_read_transfer(
    unsigned short slave_addr,
    unsigned char *read_buf,
    int len
)
{
    int rc = 0;
#ifdef  CE150X_DUMP_I2C_MESSAGE
    int i;
#endif

    struct i2c_msg msg[] = {
        {
            .addr = slave_addr,
            .flags = I2C_M_RD,
            .len = len,
            .buf = read_buf,
        },
    };

    CE150X_ENTER;

    if (i2c_transfer(ce150x_client->adapter, msg, 1) < 0) {
        rc = -EIO;
    }

#ifdef  CE150X_DUMP_I2C_MESSAGE
    CE150X_LOG_DBG("slave_addr[0x%04x], len[0x%04x]", slave_addr, len);

    for(i=0; i<len; i++)
    {
        CE150X_LOG_DBG("[0x%02x]", *(read_buf+i));
    }
#endif

    CE150X_RETURN_N(rc);
}

/*
* ce150x_sensor_i2c_write
*/
static int ce150x_sensor_i2c_write(struct sensor_i2c_wr_cfg *i2c_wr_cfg)
{
    int rc = 0;
    unsigned char *write_buf;

    CE150X_ENTER;

    write_buf = (unsigned char *)kzalloc((size_t)i2c_wr_cfg->len,GFP_KERNEL);
    if( !write_buf ){
        rc = -EFAULT;
    }

    if( !(rc < 0) ){

        if ( copy_from_user( &write_buf[0],
                             (void *)(i2c_wr_cfg->write_data_ptr),
                             (i2c_wr_cfg->len - CE150X_I2C_WR_PKT_HEADER) ) ){
            rc = -EFAULT;
        }
    }

    if( !(rc < 0) ){
        rc = ce150x_sensor_i2c_write_transfer( ce150x_client->addr,
                                                write_buf,
                                                i2c_wr_cfg->len );
    }

    if( write_buf )
        kfree(write_buf);

    CE150X_RETURN_N(rc);
}
/*
* ce150x_sensor_i2c_read
*/
static long ce150x_sensor_i2c_read(struct sensor_i2c_rd_cfg *i2c_rd_cfg)
{
    int   rc = 0;
    unsigned char *read_buf = NULL;

    CE150X_ENTER;

    read_buf = (unsigned char *)kzalloc((size_t)i2c_rd_cfg->len, GFP_KERNEL);
    if( !read_buf ){
        rc = -EFAULT;
    }

    if( !(rc < 0) ) {
        rc = ce150x_sensor_i2c_read_transfer( ce150x_client->addr,
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

    CE150X_RETURN_N(rc);
}



/*
* ce150x_spi_probe
*/
static int ce150x_spi_probe(struct spi_device *client)
{
    int rc = 0;

    CE150X_ENTER;

    client->bits_per_word = 16;
    client->mode = SPI_MODE_3;
    client->chip_select = 1;
    
    if(spi_setup(client) < 0)
    {
        rc = -ENOTSUPP;
    }
    ce150x_client_spi = client;
    
    CE150X_RETURN_N(rc);
}

/*
* ce150x_spi_remove
*/
static int ce150x_spi_remove(struct spi_device *client)
{
    CE150X_ENTER;

    ce150x_client_spi = NULL;
    
    CE150X_RETURN_N(0);
}

/*
* ce150x_spi_driver
*/
static struct spi_driver ce150x_spi_driver = {
    .probe       = ce150x_spi_probe,
    .remove      = __devexit_p(ce150x_spi_remove),
    .driver = {
      .name   = "ce150x_spi",
      .bus    = &spi_bus_type,
      .owner  = THIS_MODULE,
    },
};

/*
* ce150x_sensor_spi_write_transfer
*/
static int ce150x_sensor_spi_write_transfer(unsigned char *trans_buf, unsigned int len)
{
    int   rc = 0;
    struct spi_message  message;
    struct spi_transfer transfer;

    CE150X_ENTER;

    memset(&transfer, 0, sizeof(struct spi_transfer));
    /* MSB must be '1' to indicate write */
    transfer.tx_buf = trans_buf;
    transfer.rx_buf = NULL;
    transfer.len    = len;

    spi_setup(ce150x_client_spi);
    spi_message_init(&message);

    spi_message_add_tail(&transfer, &message);
    rc = spi_sync(ce150x_client_spi, &message);
    if(rc < 0)
    {
        CE150X_LOG_ERR();
    }

    CE150X_RETURN_N(rc);
}

/*
* ce150x_sensor_spi_write
*/
static int ce150x_sensor_spi_write(struct sensor_spi_wr_cfg *spi_wr_cfg)
{
    int rc = 0;
    unsigned char *write_buf;
#ifdef  CE150X_DUMP_SPI_MESSAGE
    int i;
#endif
#ifdef  CE150X_PARTIAL_BLOCK_TRANCE
    int j;
    int rem_size;
    int loop_cnt;
#endif

    CE150X_ENTER;

    write_buf = (unsigned char *)kzalloc((size_t)spi_wr_cfg->len, GFP_KERNEL);
    if( !write_buf ){
        rc = -EFAULT;
    }
    if( !(rc < 0) ){

        if ( copy_from_user( &write_buf[0],
                             (void *)(spi_wr_cfg->write_data_ptr),
                             spi_wr_cfg->len ) ){
            rc = -EFAULT;
        }
    }
    if( !(rc < 0) ){
#ifdef  CE150X_PARTIAL_BLOCK_TRANCE
        loop_cnt = spi_wr_cfg->len / CE150X_BLOCK_SIZE;
        rem_size = spi_wr_cfg->len % CE150X_BLOCK_SIZE;
        for(j=0;j<loop_cnt;j++)
        {
            rc = ce150x_sensor_spi_write_transfer( write_buf+(j*CE150X_BLOCK_SIZE), CE150X_BLOCK_SIZE );
        }
        if(rem_size != 0)
        {
            rc = ce150x_sensor_spi_write_transfer( write_buf+(j*CE150X_BLOCK_SIZE), rem_size );
        }
#else
        rc = ce150x_sensor_spi_write_transfer( write_buf, spi_wr_cfg->len );
#endif
    }

    if( write_buf ){
#ifdef  CE150X_DUMP_SPI_MESSAGE
        CE150X_LOG_DBG("gark_log kfree wrbuf len:%d", spi_wr_cfg->len);
        for(i=0; i<spi_wr_cfg->len; i++)
        {
            CE150X_LOG_DBG("[0x%02x]", *(write_buf+i));
        }
#endif
        kfree(write_buf);
    }

    CE150X_RETURN_N(rc);
}

static int ce150x_sensor_reset(void)
{
    int rc = 0;
    struct sensor_cfg_data cfg_data;
    
    /* reset setting */
    cfg_data.cfg.gpio_cfg.port = senser_info_ce150x->sensor_info.sensor_reset;
    cfg_data.cfg.gpio_cfg.ctl = GPIO_LOW;
    rc = ce150x_sensor_set_gpio( &cfg_data );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }

    /* Wait:1ms */
    udelay(1);

    /* standby setting */
    cfg_data.cfg.gpio_cfg.port = senser_info_ce150x->standby;
    cfg_data.cfg.gpio_cfg.ctl = GPIO_LOW;
    rc = ce150x_sensor_set_gpio( &cfg_data );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }

    /* Wait:1ms */
    msleep(1);

    /* standby cancel */
    cfg_data.cfg.gpio_cfg.port = senser_info_ce150x->standby;
    cfg_data.cfg.gpio_cfg.ctl = GPIO_HIGH;
    rc = ce150x_sensor_set_gpio( &cfg_data );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    
    /* Wait:1ms */
    msleep(1);

    /* reset cancel */
    cfg_data.cfg.gpio_cfg.port = senser_info_ce150x->sensor_info.sensor_reset;
    cfg_data.cfg.gpio_cfg.ctl = GPIO_HIGH;
    rc = ce150x_sensor_set_gpio( &cfg_data );
    if(rc != 0){
        CE150X_RETURN_N(rc);
    }
    
    /* Wait:15ms */
    msleep(15);

    CE150X_RETURN_N(rc);
}

/*
* ce150x_sensor_set_mclk
*/
static int ce150x_sensor_set_mclk( struct sensor_cfg_data* cfg_data )
{
    int rc = 0;

    CE150X_ENTER;

    if(cfg_data->cfg.mclk_ctl){
        gpio_tlmm_config(GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), GPIO_CFG_ENABLE);

        camio_cam_m_clk = clk_get(NULL, "cam_m_clk");
        msm_camio_clk_rate_set_2(camio_cam_m_clk, 24000000);
        clk_enable(camio_cam_m_clk);
    } else {
        if (!IS_ERR(camio_cam_m_clk)) {
            clk_disable(camio_cam_m_clk);
            clk_put(camio_cam_m_clk);
            camio_cam_m_clk = NULL;
            CE150X_LOG_DBG("mclk disable ok");
        } else {
            CE150X_LOG_DBG("mclk disable error !");
            rc = -1;
        } 
    }

    CE150X_RETURN_N(rc);
}

/*
* ce150x_sensor_set_pmic
*/
static int ce150x_sensor_set_pmic(
     struct regulator *vreg_cfg, int min_uV, int max_uV)
{
    int ret = 0;

    CE150X_ENTER;

    if (IS_ERR_OR_NULL(vreg_cfg)) {
        CE150X_LOG_DBG("regulator_get error!");
        vreg_cfg = NULL;
        CE150X_RETURN_N(-1);
    }

    if ((min_uV < 0) || (max_uV < 0)) {
        ret = regulator_disable(vreg_cfg);
        if (ret) {
            CE150X_LOG_DBG("regulator disable error ret[%d]", ret);
        }
        goto regulator_free;
    } else {
        ret = regulator_set_voltage(vreg_cfg, min_uV, max_uV);
        if (ret) {
            CE150X_LOG_DBG("regulator set voltage error ret[%d]", ret);
            goto regulator_free;
        }

        ret = regulator_enable(vreg_cfg);
        if (ret) {
            CE150X_LOG_DBG("regulator_enable error ret[%d]", ret);
            goto regulator_free;
        }
    }

    CE150X_RETURN_N(ret);
regulator_free:
    regulator_put(vreg_cfg);
    vreg_cfg=NULL;
    CE150X_RETURN_N(ret);
}

/*
* ce150x_sensor_set_gpio
*/
static int ce150x_sensor_set_gpio( struct sensor_cfg_data* cfg_data )
{
    int rc = 0;

    CE150X_ENTER;

    if(cfg_data->cfg.gpio_cfg.ctl){
        gpio_direction_output( cfg_data->cfg.gpio_cfg.port, 1 );
    }
    else{
        gpio_direction_output( cfg_data->cfg.gpio_cfg.port, 0 );
    }

    CE150X_RETURN_N(rc);
}

/*
* ce150x_sensor_wait_interrput
*/
static int ce150x_sensor_wait_interrput( struct sensor_cfg_data* cfg_data )
{
    int rc = 0;
    unsigned long timeout = 0;
/*    float sys_time_cycle_ms = 0;*/
    unsigned int sys_time_cycle_ms = 0;

    CE150X_ENTER;

    if( HZ != 0 ){
       sys_time_cycle_ms = 1000/HZ;
       if( (cfg_data->cfg.wait_timeout_ms) > sys_time_cycle_ms ){
            timeout = (cfg_data->cfg.wait_timeout_ms)/sys_time_cycle_ms;
       }
    }

    if( timeout != 0 ){
        rc = wait_event_interruptible_timeout(ce150x_wait_queue,
                                              ce150x_wait_flag != 0,
                                              timeout);
        ce150x_wait_flag = 0;

        if(rc < 0){
            CE150X_RETURN_N(-1);
        } else if(rc == 0){
            CE150X_RETURN_N(-2);
        } else {
            rc = 0;
        }
    }

    CE150X_RETURN_N(rc);
}

/*
 * ce150x_csi_setting
 */
static int32_t ce150x_csi_setting()
{
	int32_t rc = 0;
 	struct msm_camera_csi_params ce150x_csi_params;
 
 	CE150X_ENTER;
 
 	ce150x_csi_params.lane_cnt = 2;
 	ce150x_csi_params.data_format = CSI_8BIT;
 	ce150x_csi_params.lane_assign = 0xe4;
 	ce150x_csi_params.dpcm_scheme = 0;
 	ce150x_csi_params.settle_cnt = 0x18;
 
 	rc = msm_camio_csi_config(&ce150x_csi_params);
 	msleep(10);
 
 	CE150X_RETURN_N(rc);
}

/*
* ce150x_sensor_config
*  - It is called from IOCTL.
*/
static int ce150x_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cfg_data;
    int   rc = 0;

    CE150X_ENTER;

    if ( copy_from_user( &cfg_data,
                         (void *)argp,
                         sizeof(struct sensor_cfg_data) ) )
        CE150X_RETURN_N(-EFAULT);

    mutex_lock(&ce150x_mut);

    switch (cfg_data.cfgtype) {
    case CFG_START:
 	rc = ce150x_csi_setting();
 	break;
    case CFG_I2C_WRITE:
        rc = ce150x_sensor_i2c_write( &cfg_data.cfg.i2c_wr_cfg );
        break;
    case CFG_I2C_READ:
        rc = ce150x_sensor_i2c_read( &cfg_data.cfg.i2c_rd_cfg );
        break;
    case CFG_WAIT_INTERRPUT:
        rc = ce150x_sensor_wait_interrput( &cfg_data );
        break;
    case CFG_SPI_WRITE:
        rc = ce150x_sensor_spi_write( &cfg_data.cfg.spi_wr_cfg );
        break;
    case CFG_SPI_READ:
        break;
    case CFG_RESET:
        rc = ce150x_sensor_reset();
        break;
    default:
        rc = -ENOTTY;
        break;
    }

    mutex_unlock(&ce150x_mut);

    CE150X_RETURN_N(rc);
}

/*
* ce150x_sensor_probe
*
*/
static int ce150x_sensor_probe(const struct msm_camera_sensor_info *info,
                struct msm_sensor_ctrl *s)
{
    int   rc = 0;

    CE150X_ENTER;

    rc = i2c_add_driver(&ce150x_i2c_driver);
    if (rc < 0) {
        rc = -ENOTSUPP;
        goto probe_fail;
    }

    rc = spi_register_driver(&ce150x_spi_driver);
    if (rc < 0) {
        rc = -ENOTSUPP;
        goto probe_fail;
    }

    s->s_init = ce150x_sensor_init;
    s->s_release = ce150x_sensor_release;
    s->s_config  = ce150x_sensor_config;
    s->s_camera_type = BACK_CAMERA_2D;
    s->s_mount_angle = 0;

probe_fail:
    CE150X_RETURN_N(0);
}

/*
* __ce150x_probe
*/
static int __ce150x_probe(struct platform_device *pdev)
{
    int rc;

    CE150X_ENTER;
    rc = msm_camera_drv_start(pdev, ce150x_sensor_probe);
    CE150X_RETURN_N( rc );
}

/*
* msm_camera_driver
*/
static struct platform_driver msm_camera_driver = {
    .probe = __ce150x_probe,
    .driver = {
        .name = "msm_camera_ce150x",
        .owner = THIS_MODULE,
    },
};

/*
* ce150x_init
*/
static int __init ce150x_init(void)
{
    int rc;

    CE150X_ENTER;
    rc = platform_driver_register(&msm_camera_driver);
    CE150X_RETURN_N( rc );
}

/*
* MODULE_INIT
*/
module_init(ce150x_init);

MODULE_DESCRIPTION("CE150X sensor driver");
MODULE_LICENSE("GPL v2");

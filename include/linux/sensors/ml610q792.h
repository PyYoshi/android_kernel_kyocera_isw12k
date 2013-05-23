/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 */
/*
 *  ml610q792.h - Linux kernel modules for acceleration sensor
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


#ifndef _ML610D792_H_
#define _ML610D792_H_

#include <linux/spi/spi.h>
#include <linux/sensors/alps_io.h>

#define    ACCSNS_ACTIVE_ACC          0x10
#define    ACCSNS_ACTIVE_PEDOM        0x20
#define    ACCSNS_ACTIVE_VEHICLE      0x40
#define    ACCSNS_ACTIVE_MOVE         0x80
#define    ACCSNS_ACTIVE_PEDOM_ERROR    (ACCSNS_ACTIVE_PEDOM >> 4)
#define    ACCSNS_ACTIVE_VEHICLE_ERROR  (ACCSNS_ACTIVE_VEHICLE >> 4)
#define    ACTIVE_FUNC_MASK    (ACCSNS_ACTIVE_ACC | ACCSNS_ACTIVE_PEDOM | ACCSNS_ACTIVE_VEHICLE | ACCSNS_ACTIVE_MOVE)
#define    ACCSNS_ACTIVE_ERROR (ACCSNS_ACTIVE_PEDOM_ERROR | ACCSNS_ACTIVE_VEHICLE_ERROR)

int32_t accsns_dm_seqctrl(int32_t cmd);
int32_t accsns_get_current_active(void);
int32_t accsns_get_acceleration_data( int32_t* arg_ipXYZ );
int32_t accsns_get_pedometer_data( int32_t* arg_ipPedom );
int32_t accsns_get_vehicle_data( int32_t* arg_ipVehicle );
int32_t accsns_get_move_data(int32_t *arg_ipMove);
int32_t accsns_activate(int32_t arg_iUpdate, int32_t arg_iEnable);
int32_t accsns_activate_pedom(int32_t arg_iUpdate, int32_t arg_iEnable);
int32_t accsns_activate_vehicle(int32_t arg_iUpdate, int32_t arg_iEnable);
int32_t accsns_calibration_mode(void);
int32_t accsns_calibration_start(int32_t argMode);
int32_t accsns_calibration_is_wait(void);
int32_t accsns_calibration_is_comp(int32_t* argCal);
void accsns_set_freq(uint8_t freq);
void accsns_set_offset(int32_t* offsets);
void accsns_set_nv_params(IoCtlAccSetAccsnsNVParams* argParam);
void accsns_set_delay(int32_t delay);
int32_t accsns_spi_write(uint8_t adr, const uint8_t *data, uint8_t size);
int32_t accsns_spi_read(uint8_t adr, uint8_t *data, uint16_t size);
bool accsns_spi_error_check(void);
int32_t accsns_recovery_proc(int32_t kind, uint8_t *backup_data);
int32_t accsns_check_accsensor(void);
int32_t accsns_update_fw(bool boot, uint8_t *arg_iData, uint32_t arg_iLen);
int32_t accsns_update_fw_async(uint8_t *arg_iData, uint32_t arg_iLen);
int32_t accsns_update_fw_async_chk(void);
int32_t accsns_update_fw_seq(uint8_t *arg_iData, uint32_t arg_iLen);
int32_t accsns_get_fw_version(uint8_t *arg_iData);
int32_t accsns_pedom_set_info(uint32_t arg_iWeight, uint32_t arg_iStepWide, uint32_t arg_iDelay);
int32_t accsns_pedom_start_timer(uint32_t arg_iEnable);
int32_t accsns_pedom_clear(void);
int32_t accsns_vehicle_clear(void);
void accsns_debug_level_chg(int32_t lv);
int32_t accsns_initialize( void );
int32_t accsns_suspend( struct spi_device *client, pm_message_t mesg );
int32_t accsns_resume( struct spi_device *client );
int32_t accsns_io_poll_pedom(struct file *fp, poll_table *wait);
int32_t accsns_io_poll_vehicle(struct file *fp, poll_table *wait);
void accsns_ddi_set_pause_proc(int type, int pause_timer);

#endif

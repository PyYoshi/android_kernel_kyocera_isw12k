/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 */
/* 
 *
 *  Copyright (C) 2010 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2010 Avago Technologies
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

#ifndef APDS990X_H
#define APDS990X_H

#include <linux/ioctl.h>

#define D_NV_PROXIMITY_SENSOR_NEAR_SIZE         (0x06)
#define D_NV_PROXIMITY_SENSOR_FAR_SIZE          (0x06)
#define D_NV_PHOTO_SENSOR_BEAMISH_SIZE          (0x06)
#define D_NV_PHOTO_SENSOR_DARK_SIZE             (0x06)
#define D_NV_PHOTO_SENSOR_B_SIZE                (0x02)
#define D_NV_PHOTO_SENSOR_C_SIZE                (0x02)
#define D_NV_PHOTO_SENSOR_D_SIZE                (0x02)
#define D_NV_PHOTO_SENSOR_GA_SIZE               (0x02)
#define D_NV_DATA_MAX                           (0x08)

typedef struct _t_apds990x_ioctl_ps_detection
{
    unsigned long ulps_detection;
}T_APDS990X_IOCTL_PS_DETECTION;

typedef struct _t_apds990x_ioctl_als_mean_times
{
    unsigned long ulals_mean_times;
}T_APDS990X_IOCTL_ALS_MEAN_TIMES;

typedef struct _t_apds990x_ioctl_als_lux_ave
{
    unsigned long ulals_lux_ave;
    long lcdata;
    long lirdata;
}T_APDS990X_IOCTL_ALS_LUX_AVE;

typedef struct _t_apds990x_ioctl_nv
{
    unsigned long ulLength;
    unsigned char ucData[D_NV_DATA_MAX];
    unsigned long ulItem;
}T_APDS990X_IOCTL_NV;

#define APDS990X_IO             'A'

#define IOCTL_PS_DETECTION_GET        _IOWR(APDS990X_IO, 0x01, T_APDS990X_IOCTL_PS_DETECTION)
#define IOCTL_ALS_MEAN_TIMES_SET      _IOWR(APDS990X_IO, 0x02, T_APDS990X_IOCTL_ALS_MEAN_TIMES)
#define IOCTL_ALS_LUX_AVE_GET         _IOWR(APDS990X_IO, 0x03, T_APDS990X_IOCTL_ALS_LUX_AVE)
#define IOCTL_APDS990X_NV_DATA_SET    _IOWR(APDS990X_IO, 0x04, T_APDS990X_IOCTL_NV)
#define IOCTL_APDS990X_NV_DATA_GET    _IOWR(APDS990X_IO, 0x05, T_APDS990X_IOCTL_NV)

enum {
    en_NV_PROXIMITY_SENSOR_NEAR_I = 0,
    en_NV_PROXIMITY_SENSOR_FAR_I,
    en_NV_PHOTO_SENSOR_BEAMISH_I,
    en_NV_PHOTO_SENSOR_DARK_I,
    en_NV_PHOTO_SENSOR_B_I,
    en_NV_PHOTO_SENSOR_C_I,
    en_NV_PHOTO_SENSOR_D_I,
    en_NV_PHOTO_SENSOR_GA_I,
};

#endif /* APDS990X_H */

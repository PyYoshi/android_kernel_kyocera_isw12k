/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                               <sdgpiodrv.h>
DESCRIPTION
  Driver for GPIO wrapper of SD card control.

EXTERNALIZED FUNCTIONS
  Operete gpio.

This software is contributed or developed by KYOCERA Corporation.
(C) 2011 KYOCERA Corporation
(C) 2012 KYOCERA Corporation
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __SD_GPIO_DRV__
#define __SD_GPIO_DRV__

#define SDGDRV_DEVICE_NAME "kc_sdgdrv"
#define SDG_GET_VALUE   1 
#define SDG_SET_VALUE   2 
#define SDG_TO_IRQ      3 

typedef struct sdgdrv_if
{
    int gpiofuncid;
    unsigned gpio;
    int value;
} SDGDRV_IF;

#endif


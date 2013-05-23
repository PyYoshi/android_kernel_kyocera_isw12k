#ifndef KC_FOTA_DRIVER_H
#define KC_FOTA_DRIVER_H
#include <linux/ioctl.h>
#include <linux/types.h>
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 */
/*
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
struct emmc_fota_ioctl_info {
    unsigned int  addr;
    char          *buff;
    unsigned int  size;
    int           result;
};

#define IOC_MAGIC 0xF6
#define IOCTL_EMMC_FOTA_READ_CMD   _IOR(IOC_MAGIC, 1, struct emmc_fota_ioctl_info)
#define IOCTL_EMMC_FOTA_WRITE_CMD  _IOW(IOC_MAGIC, 2, struct emmc_fota_ioctl_info)


#define BOARD_EMMC_BLOCK_SIZE ( 0x20000 )

#define KC_FOTA_UI_NOERROR            (0)
#define KC_FOTA_UI_ERROR              (-1)

#endif

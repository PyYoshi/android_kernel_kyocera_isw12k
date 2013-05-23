#ifndef DNAND_DRV_H
#define DNAND_DRV_H
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
#ifdef IMAGE_MODEM_PROC
    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>
    #include "comdef.h"
#else
    #include <linux/types.h>
    #include <linux/sched.h>
    #include <linux/fs.h>
    #include <linux/bio.h>
    #include <linux/genhd.h>
    #include <linux/gfp.h>
    #include <linux/mm.h>
    #include <asm/page.h>
#endif

#define DNAND_DRV_SECTOR_BLK_SIZE            (512)

#define DNAND_MMC_BLOCK_DEV_NAME_MIBIB       ("/dev/block/mmcblk0")
#define DNAND_MMC_BLOCK_DEV_NAME             ("/dev/block/mmcblk0p")

#define DNAND_TRUE                           (10)
#define DNAND_FALSE                          (0)
#define DNAND_INITIALIZED                    (10)

#define DNAND_DRV_BOOT_REC_SIG               (0xAA55)
#define DNAND_DRV_MBR_ENTRIES                (4)
#define DNAND_DRV_EBR_ENTRIES                (2)
#define DNAND_DRV_EXTENDED_PARTITION_TYPE    (5)
#define DNAND_DRV_DNAND_PARTITION_TYPE       (0x9B)
#define DNAND_DRV_ERR_PARTITION_NUM          (0xFF)

typedef enum
{
    DNAND_DEV_READ    = 0,
    DNAND_DEV_WRITE,
    DNAND_DEV_MAX
}dnand_dev_rw_type;

#ifdef IMAGE_MODEM_PROC
    typedef PACKED struct dnand_drv_part_entry_st
    {
        uint8                   status;
        uint8                   rsvd0[3];
        uint8                   type;
        uint8                   rsvd1[3];
        uint32                  start_sector;
        uint32                  partition_size;
    }dnand_drv_part_entry;

    typedef PACKED struct dnand_drv_boot_rec_st
    {
        uint8                   rsvd0[446];
        dnand_drv_part_entry    part_entry[DNAND_DRV_MBR_ENTRIES];
        uint16                  sig;
    } dnand_drv_boot_rec;
#else
    typedef struct dnand_bdev_status_st
    {
        struct completion       event;
        int                     error;
    }dnand_bdev_status;

    struct dnand_drv_part_entry
    {
        uint8_t                 status;
        uint8_t                 rsvd0[3];
        uint8_t                 type;
        uint8_t                 rsvd1[3];
        uint32_t                start_sector;
        uint32_t                partition_size;
    } __attribute__((__packed__));

    struct dnand_drv_boot_rec
    {
        uint8_t                        rsvd0[446];
        struct dnand_drv_part_entry    part_entry[DNAND_DRV_MBR_ENTRIES];
        uint16_t                       sig;
    } __attribute__((__packed__));

#endif

#ifdef IMAGE_MODEM_PROC
    int32   dnand_drv_read( uint32 sector, uint32 num_sector, uint8 *pbuf );
#else
    int32_t dnand_drv_read( uint32_t sector, uint32_t num_sector, uint8_t *pbuf );
#endif

#ifdef IMAGE_MODEM_PROC
    int32   dnand_drv_write( uint32 sector, uint32 num_sector, uint8 *pbuf );
#else
    int32_t dnand_drv_write( uint32_t sector, uint32_t num_sector, uint8_t *pbuf );
#endif

#ifdef IMAGE_MODEM_PROC
    void dnand_sdcc_access_enable(void);
#endif

#ifdef IMAGE_MODEM_PROC
    void dnand_sdcc_access_disable( void );
#endif

#endif

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
    #include "sdcc_api.h"
#else
    #include <linux/types.h>
    #include <linux/sched.h>
    #include <linux/file.h>
    #include <linux/fs.h>
    #include <linux/bio.h>
    #include <linux/genhd.h>
    #include <linux/gfp.h>
    #include <linux/mm.h>
#endif

#include "dnand_fs.h"
#include "dnand_drv.h"

#ifdef IMAGE_MODEM_PROC
static int8      local_mmc_dev_name[32];
static uint32    init_dev_name_flg           = 0;
static uint32    dnand_part_sector_offset    = 0xFFFFFFFF;

#define DNAND_MSECT_ACCESS

static struct sdcc_device  *p_dnand_sdc_handle = NULL;

#else
static int8_t    local_mmc_dev_name[32];
static uint32_t  init_dev_name_flg           = 0;
static uint32_t  dnand_part_sector_offset    = 0xFFFFFFFF;
#endif

#ifdef IMAGE_MODEM_PROC
void dnand_sdcc_access_enable(void)
{
    if( p_dnand_sdc_handle == NULL )
    {
        p_dnand_sdc_handle = sdcc_handle_open( SDCC_DRIVENO_1, 0 );
    }
    return;
}
#endif

#ifdef IMAGE_MODEM_PROC
void dnand_sdcc_access_disable( void )
{
    if( p_dnand_sdc_handle != NULL )
    {
        sdcc_handle_close( p_dnand_sdc_handle );
        p_dnand_sdc_handle = NULL;
    }
    return;
}
#endif

#ifdef IMAGE_MODEM_PROC
static struct sdcc_device* inter_sdcc_open(void)
{
    if( p_dnand_sdc_handle == NULL )
    {
        dnand_sdcc_access_enable();
    }
    return(p_dnand_sdc_handle);
}
#endif

#ifdef IMAGE_MODEM_PROC
static void inter_sdcc_close( struct sdcc_device *handle )
{
    return;
}
#endif

#ifdef IMAGE_MODEM_PROC
static int32   inter_blk_dev_rw(
            int8                 *pname,
            uint32               sector,
            uint32               num_sector,
            uint8                *pbuf,
            dnand_dev_rw_type    rwtype )
#else
static int32_t inter_blk_dev_rw(
            int8_t               *pname,
            uint32_t             sector,
            uint32_t             num_sector,
            uint8_t              *pbuf,
            dnand_dev_rw_type    rwtype )
#endif
{
#ifdef IMAGE_MODEM_PROC
    struct sdcc_device     *handle;
    SDCC_STATUS            sdcc_rtn;
#ifdef DNAND_MSECT_ACCESS
#else
    int32                  i;
#endif

    if( pbuf == NULL )
    {
        return(DNAND_INTERNAL_ERROR);
    }
    
    handle = inter_sdcc_open();
    if ( handle == NULL )
    {
        return(DNAND_DEV_ERROR);
    }
#ifdef DNAND_MSECT_ACCESS
#else
    sdcc_rtn = SDCC_NO_ERROR;
#endif

    if( rwtype == DNAND_DEV_READ )
    {
#ifdef DNAND_MSECT_ACCESS
        sdcc_rtn    = sdcc_handle_read(
                           handle, sector, pbuf, (uint16)num_sector );
#else
        for( i=0; i<num_sector; i++ )
        {
            sdcc_rtn    = sdcc_handle_read(
                           handle, sector, pbuf, 1 );
            if( sdcc_rtn != SDCC_NO_ERROR )
            {
                break;
            }
            sector++;
            pbuf        += DNAND_DRV_SECTOR_BLK_SIZE;
        }
#endif
    }
    else
    {
#ifdef DNAND_MSECT_ACCESS
        sdcc_rtn    = sdcc_handle_write(
                           handle, sector, pbuf, (uint16)num_sector );
#else
        for( i=0; i<num_sector; i++ )
        {
            sdcc_rtn    = sdcc_handle_write(
                           handle, sector, pbuf, 1 );
            if( sdcc_rtn != SDCC_NO_ERROR )
            {
                break;
            }
            sector++;
            pbuf        += DNAND_DRV_SECTOR_BLK_SIZE;
        }
#endif
    }
    inter_sdcc_close( handle );
    if( sdcc_rtn == SDCC_NO_ERROR )
    {
        return(DNAND_NO_ERROR);
    }
    else
    {
        return(DNAND_DEV_ERROR);
    }
#else
    struct file            *dnand_file;
    int32_t                flags;
    uint32_t               size,srtn;
    mm_segment_t           _segfs;

    if( (pname == NULL)||(pbuf == NULL) )
    {
        return(DNAND_DEV_ERROR);
    }

    if( rwtype == DNAND_DEV_READ )
    {
        flags    = O_RDONLY|O_SYNC|O_LARGEFILE;
    }
    else
    {
        flags    = O_RDWR|O_SYNC|O_LARGEFILE;
    }

    dnand_file  = filp_open(pname, flags, 0);
    if( dnand_file == NULL )
    {
        return(DNAND_DEV_ERROR);
    }

    _segfs         = get_fs();
    set_fs(get_ds());

    size            = DNAND_DRV_SECTOR_BLK_SIZE*sector;
    srtn            = dnand_file->f_op->llseek(dnand_file, size, SEEK_SET);
    if( srtn != size )
    {
        set_fs(_segfs);
        filp_close(dnand_file,NULL);
        return(DNAND_DEV_ERROR);
    }
    
    size            = DNAND_DRV_SECTOR_BLK_SIZE*num_sector;

    if( rwtype == DNAND_DEV_READ )
    {
        srtn        = dnand_file->f_op->read(dnand_file, pbuf, size, &dnand_file->f_pos);
    }
    else
    {
        srtn        = dnand_file->f_op->write(dnand_file, pbuf, size, &dnand_file->f_pos);
    }
    
    set_fs(_segfs);
    filp_close(dnand_file,NULL);
    if( srtn == size )
    {
        return(DNAND_NO_ERROR);
    }
    else
    {
        return(DNAND_DEV_ERROR);
    }
#endif
}

#ifdef IMAGE_MODEM_PROC
static int32   inter_read_record(
        uint32        sector,
        uint8         *pbuf )
#else
static int32_t inter_read_record(
        uint32_t      sector,
        uint8_t       *pbuf )
#endif
{
#ifdef IMAGE_MODEM_PROC
    int32                        rtn, num;
    dnand_drv_boot_rec           *pmbr;
#else
    int32_t                      rtn, num;
    struct dnand_drv_boot_rec    *pmbr;
#endif
    
    if( pbuf == NULL )
    {
        return(DNAND_INTERNAL_ERROR);
    }

    num    = 1;
    rtn    = inter_blk_dev_rw(
        DNAND_MMC_BLOCK_DEV_NAME_MIBIB, sector, num, pbuf, DNAND_DEV_READ );
    if( rtn != DNAND_NO_ERROR )
    {
        return(rtn);
    }

#ifdef IMAGE_MODEM_PROC
    pmbr   = (dnand_drv_boot_rec*)pbuf;
#else
    pmbr   = (struct dnand_drv_boot_rec*)pbuf;
#endif
    if( pmbr->sig != DNAND_DRV_BOOT_REC_SIG )
    {
        rtn    = DNAND_DEV_ERROR;
    }
    return(rtn);
}

#ifdef IMAGE_MODEM_PROC
static uint32   search_dnand_partition(void)
#else
static uint32_t search_dnand_partition(void)
#endif
{
#ifdef IMAGE_MODEM_PROC
    dnand_drv_boot_rec           mbr;
    uint32                       i, j, rsector;
    uint32                       part_num, ebr_offset, local_ebr_offset ,part_type;
    uint32                       dnand_part_num;
    int32                        rtn;
#else
    struct dnand_drv_boot_rec    mbr;
    uint32_t                     i, j, rsector;
    uint32_t                     part_num, ebr_offset, local_ebr_offset ,part_type;
    uint32_t                     dnand_part_num;
    int32_t                      rtn;
#endif

    dnand_part_sector_offset    = 0xFFFFFFFF;
    rsector    = 0;
#ifdef IMAGE_MODEM_PROC
    rtn        = inter_read_record(rsector, (uint8*)&mbr);
#else
    rtn        = inter_read_record(rsector, (uint8_t*)&mbr);
#endif
    if( rtn != DNAND_NO_ERROR )
    {
        return(DNAND_DRV_ERR_PARTITION_NUM);
    }

    part_num   = 0;
    ebr_offset = 0;
    for( i=0; i< DNAND_DRV_MBR_ENTRIES; i++)
    {
        if(mbr.part_entry[i].type == DNAND_DRV_EXTENDED_PARTITION_TYPE)
        {
            part_num    = i + 1;
            ebr_offset  = mbr.part_entry[i].start_sector;
            break;
        }
    }

    if( i==DNAND_DRV_MBR_ENTRIES)
    {
        return(DNAND_DRV_ERR_PARTITION_NUM);
    }

    local_ebr_offset    = 0;
    dnand_part_num      = 0;
    for( i=0; i<DNAND_DRV_ERR_PARTITION_NUM; i++ )
    {
        rsector    = ebr_offset + local_ebr_offset;
#ifdef IMAGE_MODEM_PROC
        rtn        = inter_read_record(rsector, (uint8*)&mbr);
#else
        rtn        = inter_read_record(rsector, (uint8_t*)&mbr);
#endif
        if( rtn != DNAND_NO_ERROR )
        {
            dnand_part_num    = DNAND_DRV_ERR_PARTITION_NUM;
            break;
        }

        for( j=0; j<DNAND_DRV_EBR_ENTRIES; j++)
        {
            part_type    = mbr.part_entry[j].type;
            if(part_type == 0)
            {
                dnand_part_num    = DNAND_DRV_ERR_PARTITION_NUM;
                break;
            }
        
            if( part_type == DNAND_DRV_EXTENDED_PARTITION_TYPE )
            {
                local_ebr_offset = mbr.part_entry[j].start_sector;
                break;
            }
            part_num++;
            
            if( part_type == DNAND_DRV_DNAND_PARTITION_TYPE )
            {
                dnand_part_num              = part_num;
                dnand_part_sector_offset    = rsector + mbr.part_entry[j].start_sector;
                break;
            }
         }

         if( dnand_part_num > 0 )
         {
             break;
         }
    }
    
    if( dnand_part_num == 0 )
    {
        dnand_part_num    = DNAND_DRV_ERR_PARTITION_NUM;
    }
    return(dnand_part_num);
}

#ifdef IMAGE_MODEM_PROC
static int32   inter_make_dev_name(void)
#else
static int32_t inter_make_dev_name(void)
#endif
{
#ifdef IMAGE_MODEM_PROC
    uint32    dnand_part_num;
#else
    uint32_t  dnand_part_num;
#endif
    
    if( init_dev_name_flg == DNAND_INITIALIZED )
    {
        return(DNAND_NO_ERROR);
    }

#ifdef _WIN32
    dnand_part_num       = 1;
#else
    dnand_part_num       = search_dnand_partition();
#endif
    if( (dnand_part_num == 0)||(dnand_part_num >= DNAND_DRV_ERR_PARTITION_NUM) )
    {
        return(DNAND_DEV_ERROR);
    }
    sprintf( (char*)&local_mmc_dev_name[0], "%s%d", DNAND_MMC_BLOCK_DEV_NAME, dnand_part_num );
    init_dev_name_flg    = DNAND_INITIALIZED;
    return(DNAND_NO_ERROR);
}

#ifdef IMAGE_MODEM_PROC
static int8*   inter_get_dev_name(void)
#else
static int8_t* inter_get_dev_name(void)
#endif
{
#ifdef IMAGE_MODEM_PROC
    int32     rtn;
    uint32    len;
#else
    int32_t   rtn;
    uint32_t  len;
#endif
    
    rtn    = inter_make_dev_name();
    if( rtn != DNAND_NO_ERROR )
    {
        return(NULL);
    }
    
    len = strlen(DNAND_MMC_BLOCK_DEV_NAME);
    rtn = memcmp(DNAND_MMC_BLOCK_DEV_NAME, local_mmc_dev_name, len);
    if( rtn == 0 )
    {
        return( &local_mmc_dev_name[0] );
    }
    else
    {
        return(NULL);
    }
}

#ifdef IMAGE_MODEM_PROC
static uint32   inter_get_dnand_offset(void)
{
#ifdef _WIN32
    return(0);
#else
    return(dnand_part_sector_offset);
#endif
}
#endif

#ifdef IMAGE_MODEM_PROC
int32        dnand_drv_read(
                uint32        sector,
                uint32        num_sector,
                uint8         *pbuf )
#else
int32_t      dnand_drv_read(
                uint32_t      sector,
                uint32_t      num_sector,
                uint8_t       *pbuf )
#endif
{
#ifdef IMAGE_MODEM_PROC
    int32            rtn;
    int8             *pname;
#else
    int32_t          rtn;
    int8_t           *pname;
#endif

    pname  = inter_get_dev_name();
    if( pname == NULL )
    {
        return(DNAND_DEV_ERROR);
    }
#ifdef IMAGE_MODEM_PROC
    sector += inter_get_dnand_offset();
#endif
    rtn    = inter_blk_dev_rw(pname, sector, num_sector, pbuf, DNAND_DEV_READ );
    return(rtn);
}

#ifdef IMAGE_MODEM_PROC
int32        dnand_drv_write(
                uint32        sector,
                uint32        num_sector,
                uint8         *pbuf )
#else
int32_t      dnand_drv_write(
                uint32_t      sector,
                uint32_t      num_sector,
                uint8_t       *pbuf )
#endif
{
#ifdef IMAGE_MODEM_PROC
    int32            rtn;
    int8             *pname;
#else
    int32_t          rtn;
    int8_t           *pname;
#endif

    pname  = inter_get_dev_name();
    if( pname == NULL )
    {
        return(DNAND_DEV_ERROR);
    }
#ifdef IMAGE_MODEM_PROC
    sector += inter_get_dnand_offset();
#endif
    rtn    = inter_blk_dev_rw(pname, sector, num_sector, pbuf, DNAND_DEV_WRITE );
    return(rtn);
}

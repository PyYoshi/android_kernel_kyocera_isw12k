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
    #include <linux/module.h>
    #include <linux/init.h>
    #include <linux/fs.h>
    #include <linux/miscdevice.h>
    #include <linux/sched.h>
    #include <linux/wait.h>
    #include <linux/uaccess.h>
    #include <linux/slab.h>

    #include <linux/kernel.h>
    #include <linux/list.h>
    #include <linux/module.h>
#endif

#include "dnand_fs.h"
#include "dnand_fs_mng.h"
#include "dnand_drv.h"

#ifdef IMAGE_MODEM_PROC
    static uint32               dnand_fs_init = 0;
    static dnand_fs_memng       dnand_fsinfo[DNAND_FS_SYSTEM_NUM];
    static uint32               dnand_alloc_start = DNAND_FS_BLK_DATA_ST;
#else
    static uint32_t             dnand_fs_init = 0;
    static dnand_fs_memng       dnand_fsinfo[DNAND_FS_SYSTEM_NUM];
    static uint32_t             dnand_alloc_start = DNAND_FS_BLK_DATA_ST;
#endif

#ifdef IMAGE_MODEM_PROC
    static uint8                dnand_fs_rw_sectorbuf[DNAND_DRV_SECTOR_BLK_SIZE];
    static uint8                dnand_fs_rw_blockbuf[DNAND_FS_BLK_SIZE];
    static dnand_fs_flmng       dnand_fs_flmngbuf0;
    static dnand_fs_flmng       dnand_fs_flmngbuf1;
#endif

static void inter_init_fsinfo(void)
{
    memset( (void*)&dnand_fsinfo[0], 0xff, sizeof(dnand_fsinfo) );
    return;
}

static void inter_cpy_info( dnand_fs_flmng *pbuf )
{
    if( pbuf == NULL )
    {
        return;
    }
    
    memcpy( &dnand_fsinfo[0].clid[0], &pbuf->clid[0], sizeof(dnand_fsinfo[0].clid) );
    memcpy( &dnand_fsinfo[0].fat[0], &pbuf->fat[0], sizeof(dnand_fsinfo[0].fat) );
    memcpy( &dnand_fsinfo[1], &dnand_fsinfo[0], sizeof(dnand_fsinfo[1]) );
    return;
}

#ifdef IMAGE_MODEM_PROC
static int32   dnand_blk_read(
            uint32            blk_no,
            uint32            offset,
            uint8             *pbuf,
            uint32            size )
#else
static int32_t dnand_blk_read(
            uint32_t          blk_no,
            uint32_t          offset,
            uint8_t           *pbuf,
            uint32_t          size )
#endif
{
#ifdef IMAGE_MODEM_PROC
    int32         rtn;
    uint32        s_sect, e_sect, n_sect, offset_sect, rest_sect;
    uint32        r_sect, c_size;
    uint8         *p_kbuf;
#else
    int32_t       rtn;
    uint32_t      s_sect, e_sect, n_sect, offset_sect, rest_sect;
    uint32_t      r_sect, c_size;
    uint8_t       *p_kbuf;
#endif

    s_sect        = (blk_no*DNAND_FS_BLK_SECT_NUM) + (offset/DNAND_DRV_SECTOR_BLK_SIZE);
    e_sect        = (offset+size+DNAND_DRV_SECTOR_BLK_SIZE-1)/DNAND_DRV_SECTOR_BLK_SIZE;
    e_sect        += (blk_no*DNAND_FS_BLK_SECT_NUM);
    n_sect        = e_sect-s_sect;

    offset_sect   = offset%DNAND_DRV_SECTOR_BLK_SIZE;
    rest_sect     = (offset+size)%DNAND_DRV_SECTOR_BLK_SIZE;

    p_kbuf        = NULL;
#ifdef IMAGE_MODEM_PROC
    p_kbuf       = (uint8*)&dnand_fs_rw_sectorbuf[0];
#else
    p_kbuf       = (uint8_t*)kmalloc(DNAND_DRV_SECTOR_BLK_SIZE, GFP_KERNEL);
#endif
    if( p_kbuf == NULL )
    {
        return(DNAND_NOMEM_ERROR);
    }

    if( offset_sect > 0 )
    {
        r_sect       = 1;
        rtn          = dnand_drv_read(s_sect, r_sect, p_kbuf);
        if( rtn != DNAND_NO_ERROR )
        {
#ifdef IMAGE_MODEM_PROC
#else
            kfree(p_kbuf);
#endif
            return(rtn);
        }
        if( (offset_sect+size) > DNAND_DRV_SECTOR_BLK_SIZE )
        {
            c_size    = DNAND_DRV_SECTOR_BLK_SIZE-offset_sect;
        }
        else
        {
            c_size    = size;
        }
        memcpy( pbuf, (p_kbuf+offset_sect), c_size );
        pbuf          += c_size;
        size          -= c_size;
        s_sect        += r_sect;
        n_sect        -= r_sect;
    }

    if( n_sect == 0 )
    {
#ifdef IMAGE_MODEM_PROC
#else
        kfree(p_kbuf);
#endif
        return(DNAND_NO_ERROR);
    }

    if( rest_sect > 0 )
    {
        r_sect       = n_sect -1;
    }
    else
    {
        r_sect       = n_sect;
    }
    
    if( r_sect > 0 )
    {
        rtn          = dnand_drv_read(s_sect, r_sect, pbuf);
        if( rtn != DNAND_NO_ERROR )
        {
#ifdef IMAGE_MODEM_PROC
#else
            kfree(p_kbuf);
#endif
            return(rtn);
        }
        c_size        = r_sect*DNAND_DRV_SECTOR_BLK_SIZE;
        pbuf          += c_size;
        size          -= c_size;
        s_sect        += r_sect;
        n_sect        -= r_sect;
    }

    if( n_sect > 0 )
    {
        r_sect       = 1;
        rtn          = dnand_drv_read(s_sect, r_sect, p_kbuf);
        if( rtn != DNAND_NO_ERROR )
        {
#ifdef IMAGE_MODEM_PROC
#else
            kfree(p_kbuf);
#endif
            return(rtn);
        }
        memcpy( pbuf, p_kbuf, size );
    }
#ifdef IMAGE_MODEM_PROC
#else
    kfree(p_kbuf);
#endif
    return(DNAND_NO_ERROR);
}

#ifdef IMAGE_MODEM_PROC
static int32   dnand_blk_write(
            uint32            blk_no,
            uint32            offset,
            uint8             *pbuf,
            uint32            size )
#else
static int32_t dnand_blk_write(
            uint32_t          blk_no,
            uint32_t          offset,
            uint8_t           *pbuf,
            uint32_t          size )
#endif
{
#ifdef IMAGE_MODEM_PROC
    int32         rtn;
    uint32        s_sect, e_sect, n_sect, offset_sect, rest_sect;
    uint32        rw_sect, c_size;
    uint8         *p_kbuf;
#else
    int32_t       rtn;
    uint32_t      s_sect, e_sect, n_sect, offset_sect, rest_sect;
    uint32_t      rw_sect, c_size;
    uint8_t       *p_kbuf;
#endif

    s_sect        = (blk_no*DNAND_FS_BLK_SECT_NUM) + (offset/DNAND_DRV_SECTOR_BLK_SIZE);
    e_sect        = (offset+size+DNAND_DRV_SECTOR_BLK_SIZE-1)/DNAND_DRV_SECTOR_BLK_SIZE;
    e_sect        += (blk_no*DNAND_FS_BLK_SECT_NUM);
    n_sect        = e_sect-s_sect;

    offset_sect   = offset%DNAND_DRV_SECTOR_BLK_SIZE;
    rest_sect     = (offset+size)%DNAND_DRV_SECTOR_BLK_SIZE;

#ifdef IMAGE_MODEM_PROC
    p_kbuf       = (uint8*)&dnand_fs_rw_sectorbuf[0];
#else
    p_kbuf       = (uint8_t*)kmalloc(DNAND_DRV_SECTOR_BLK_SIZE, GFP_KERNEL);
#endif
    if( p_kbuf == NULL )
    {
        return(DNAND_NOMEM_ERROR);
    }

    if( offset_sect > 0 )
    {
        rw_sect       = 1;
        rtn          = dnand_drv_read(s_sect, rw_sect, p_kbuf);
        if( rtn != DNAND_NO_ERROR )
        {
#ifdef IMAGE_MODEM_PROC
#else
            kfree(p_kbuf);
#endif
            return(rtn);
        }
        if( (offset_sect+size) > DNAND_DRV_SECTOR_BLK_SIZE )
        {
            c_size    = DNAND_DRV_SECTOR_BLK_SIZE-offset_sect;
        }
        else
        {
            c_size    = size;
        }
        memcpy( (p_kbuf+offset_sect), pbuf, c_size );
        rtn          = dnand_drv_write(s_sect, rw_sect, p_kbuf);
        if( rtn != DNAND_NO_ERROR )
        {
#ifdef IMAGE_MODEM_PROC
#else
            kfree(p_kbuf);
#endif
            return(rtn);
        }

        pbuf          += c_size;
        size          -= c_size;
        s_sect        += rw_sect;
        n_sect        -= rw_sect;
    }

    if( n_sect == 0 )
    {
#ifdef IMAGE_MODEM_PROC
#else
        kfree(p_kbuf);
#endif
        return(DNAND_NO_ERROR);
    }

    if( rest_sect > 0 )
    {
        rw_sect       = n_sect -1;
    }
    else
    {
        rw_sect       = n_sect;
    }
    
    if( rw_sect > 0 )
    {
        rtn          = dnand_drv_write(s_sect, rw_sect, pbuf);
        if( rtn != DNAND_NO_ERROR )
        {
#ifdef IMAGE_MODEM_PROC
#else
            kfree(p_kbuf);
#endif
            return(rtn);
        }
        c_size        = rw_sect*DNAND_DRV_SECTOR_BLK_SIZE;
        pbuf          += c_size;
        size          -= c_size;
        s_sect        += rw_sect;
        n_sect        -= rw_sect;
    }

    if( n_sect > 0 )
    {
        rw_sect       = 1;
        rtn          = dnand_drv_read(s_sect, rw_sect, p_kbuf);
        if( rtn != DNAND_NO_ERROR )
        {
#ifdef IMAGE_MODEM_PROC
#else
            kfree(p_kbuf);
#endif
            return(rtn);
        }
        c_size    = size;
        memcpy( p_kbuf, pbuf, size );
        rtn          = dnand_drv_write(s_sect, rw_sect, p_kbuf);
        if( rtn != DNAND_NO_ERROR )
        {
#ifdef IMAGE_MODEM_PROC
#else
            kfree(p_kbuf);
#endif
            return(rtn);
        }
    }
#ifdef IMAGE_MODEM_PROC
#else
    kfree(p_kbuf);
#endif
    return(DNAND_NO_ERROR);
}

#ifdef IMAGE_MODEM_PROC
static int32   update_flmng(
            uint32                sysno,
            dnand_fs_flmng        *p_flmng )
#else
static int32_t update_flmng(
            uint32_t              sysno,
            dnand_fs_flmng        *p_flmng )
#endif
{
#ifdef IMAGE_MODEM_PROC
    uint32        blk_no;
    int32         rtn;
#else
    uint32_t      blk_no;
    int32_t       rtn;
#endif

    if( p_flmng == NULL )
    {
        return(DNAND_INTERNAL_ERROR);
    }
    if( sysno == 0 )
    {
        blk_no    = 0;
    }
    else
    {
        blk_no    = DNAND_FS_BLK_SYS_NUM;
    }
    
    memset( &p_flmng->ckcd[0], 0xFF, sizeof(p_flmng->ckcd) );
    memset( &p_flmng->dmya[0], 0xFF, sizeof(p_flmng->dmya) );
#ifdef IMAGE_MODEM_PROC
    rtn        = dnand_blk_write( blk_no, 0, (uint8*)p_flmng, DNAND_FS_BLK_SIZE );
#else
    rtn        = dnand_blk_write( blk_no, 0, (uint8_t*)p_flmng, DNAND_FS_BLK_SIZE );
#endif
    if( rtn != DNAND_NO_ERROR )
    {
        return(rtn);
    }

    memset( &p_flmng->clid[0], 0xFF, sizeof(p_flmng->clid) );
    memcpy( &p_flmng->clid[0], &dnand_fsinfo[1].clid[0], sizeof(dnand_fsinfo[1].clid) );
    memset( &p_flmng->fat[0], 0xFF, sizeof(p_flmng->fat) );
    memcpy( &p_flmng->fat[0], &dnand_fsinfo[1].fat[0], sizeof(dnand_fsinfo[1].fat) );
#ifdef IMAGE_MODEM_PROC
    rtn        = dnand_blk_write( blk_no, 0, (uint8*)p_flmng, sizeof(dnand_fs_flmng) );
#else
    rtn        = dnand_blk_write( blk_no, 0, (uint8_t*)p_flmng, sizeof(dnand_fs_flmng) );
#endif
    if( rtn != DNAND_NO_ERROR )
    {
        return(rtn);
    }

    strcpy( (char*)&p_flmng->ckcd[0], DNAND_FS_CKCD );
#ifdef IMAGE_MODEM_PROC
    rtn        = dnand_blk_write( blk_no, 0, (uint8*)p_flmng, DNAND_FS_BLK_SIZE );
#else
    rtn        = dnand_blk_write( blk_no, 0, (uint8_t*)p_flmng, DNAND_FS_BLK_SIZE );
#endif
    return(rtn);
}

#ifdef IMAGE_MODEM_PROC
static void inter_memng_clr( uint32    cid )
#else
static void inter_memng_clr( uint32_t  cid )
#endif
{
#ifdef IMAGE_MODEM_PROC
    uint32        i;
    uint16        *pfat0, *pfat1;
    uint16        blk0, blk1, nxtblk0, nxtblk1;
#else
    uint32_t      i;
    uint16_t      *pfat0, *pfat1;
    uint16_t      blk0, blk1, nxtblk0, nxtblk1;
#endif

    if( cid >= DNAND_FS_CLID_NUM )
    {
        return;
    }

#ifdef IMAGE_MODEM_PROC
    blk0     = (uint16)dnand_fsinfo[0].clid[cid];
    blk1     = (uint16)dnand_fsinfo[1].clid[cid];
#else
    blk0     = (uint16_t)dnand_fsinfo[0].clid[cid];
    blk1     = (uint16_t)dnand_fsinfo[1].clid[cid];
#endif
    pfat0    = &dnand_fsinfo[0].fat[0];
    pfat1    = &dnand_fsinfo[1].fat[0];
    nxtblk0  = DNAND_FS_BLK_UNUSE;
    nxtblk1  = DNAND_FS_BLK_UNUSE;

    dnand_fsinfo[1].clid[cid]    = dnand_fsinfo[0].clid[cid];

    for( i=0; i<DNAND_FS_BLK_NUM; i++ )
    {
        if( (blk0 >= DNAND_FS_BLK_NUM)&&(blk1 >= DNAND_FS_BLK_NUM) )
        {
            break;
        }

        if( blk0 < DNAND_FS_BLK_NUM )
        {
            nxtblk0    = pfat0[blk0];
        }
        if( blk1 < DNAND_FS_BLK_NUM )
        {
            nxtblk1    = pfat1[blk1];
        }
        if( blk0 != blk1 )
        {
            if( blk1 < DNAND_FS_BLK_NUM )
            {
                pfat0[blk1]    = DNAND_FS_BLK_UNUSE;
                pfat1[blk1]    = DNAND_FS_BLK_UNUSE;
            }
            if( blk0 < DNAND_FS_BLK_NUM )
            {
                pfat1[blk0]    = nxtblk0;
            }
        }
        else
        {
            if( blk0 < DNAND_FS_BLK_NUM )
            {
                pfat1[blk0]    = nxtblk0;
            }
        }

        if( nxtblk0 < DNAND_FS_BLK_NUM )
        {
            blk0               = nxtblk0;
        }
        else
        {
            blk0               = DNAND_FS_BLK_UNUSE;
        }
        if( nxtblk1 < DNAND_FS_BLK_NUM )
        {
            blk1               = nxtblk1;
        }
        else
        {
            blk1               = DNAND_FS_BLK_UNUSE;
        }
    }
    return;
}

#ifdef IMAGE_MODEM_PROC
static int32   update_memng( uint32    cid )
#else
static int32_t update_memng( uint32_t  cid )
#endif
{
#ifdef IMAGE_MODEM_PROC
    int32              rtn;
    uint8              *p_kbuf;
#else
    int32_t            rtn;
    uint8_t            *p_kbuf;
#endif

    if( cid >= DNAND_FS_CLID_NUM )
    {
        return(DNAND_INTERNAL_ERROR);
    }
    inter_memng_clr( cid );

#ifdef IMAGE_MODEM_PROC
    p_kbuf    = (uint8*)&dnand_fs_flmngbuf0;
#else
    p_kbuf    = (uint8_t*)kmalloc(sizeof(dnand_fs_flmng), GFP_KERNEL);
#endif
    if( p_kbuf == NULL )
    {
        return(DNAND_NOMEM_ERROR);
    }
    rtn       = update_flmng( 1, (dnand_fs_flmng*)p_kbuf );
    if( rtn != DNAND_NO_ERROR )
    {
#ifdef IMAGE_MODEM_PROC
#else
        kfree(p_kbuf);
#endif
        return(rtn);
    }
    rtn       = update_flmng( 0, (dnand_fs_flmng*)p_kbuf );
#ifdef IMAGE_MODEM_PROC
#else
    kfree(p_kbuf);
#endif
    return(rtn);
}

#ifdef IMAGE_MODEM_PROC
static int32   pre_init(void)
#else
static int32_t pre_init(void)
#endif
{
#ifdef IMAGE_MODEM_PROC
    int32              rtn, cmprtn;
    int32              flg_valid[DNAND_FS_SYSTEM_NUM];
    uint8              *p_kbuf0;
    uint8              *p_kbuf1;
    dnand_fs_flmng     *p_flmng0;
    dnand_fs_flmng     *p_flmng1;
#else
    int32_t            rtn, cmprtn;
    int32_t            flg_valid[DNAND_FS_SYSTEM_NUM];
    uint8_t            *p_kbuf0;
    uint8_t            *p_kbuf1;
    dnand_fs_flmng     *p_flmng0;
    dnand_fs_flmng     *p_flmng1;
#endif

    if( dnand_fs_init == DNAND_INITIALIZED )
    {
        return(DNAND_NO_ERROR);
    }

    dnand_alloc_start = DNAND_FS_BLK_DATA_ST;

#ifdef IMAGE_MODEM_PROC
    p_kbuf0    = (uint8*)&dnand_fs_flmngbuf0;
#else
    p_kbuf0    = (uint8_t*)kmalloc(sizeof(dnand_fs_flmng), GFP_KERNEL);
#endif
    if( p_kbuf0 == NULL )
    {
        return(DNAND_NOMEM_ERROR);
    }
    rtn        = dnand_blk_read( 0, 0, p_kbuf0, sizeof(dnand_fs_flmng) );
    if( rtn != DNAND_NO_ERROR )
    {
#ifdef IMAGE_MODEM_PROC
#else
        kfree(p_kbuf0);
#endif
        return(rtn);
    }

#ifdef IMAGE_MODEM_PROC
    p_kbuf1    = (uint8*)&dnand_fs_flmngbuf1;
#else
    p_kbuf1    = (uint8_t*)kmalloc(sizeof(dnand_fs_flmng), GFP_KERNEL);
#endif
    if( p_kbuf1 == NULL )
    {
#ifdef IMAGE_MODEM_PROC
#else
        kfree(p_kbuf0);
#endif
        return(DNAND_NOMEM_ERROR);
    }
    rtn        = dnand_blk_read( DNAND_FS_BLK_SYS_NUM, 0, p_kbuf1, sizeof(dnand_fs_flmng) );
    if( rtn != DNAND_NO_ERROR )
    {
#ifdef IMAGE_MODEM_PROC
#else
        kfree(p_kbuf0);
        kfree(p_kbuf1);
#endif
        return(rtn);
    }
    
    p_flmng0        = (dnand_fs_flmng*)p_kbuf0;
    p_flmng1        = (dnand_fs_flmng*)p_kbuf1;
    flg_valid[0]    = DNAND_FALSE;
    flg_valid[1]    = DNAND_FALSE;
    rtn             = DNAND_NO_ERROR;
    
    cmprtn          = strcmp( (char*)p_flmng0->ckcd, DNAND_FS_CKCD );
    if( cmprtn == 0 )
    {
        flg_valid[0]    = DNAND_TRUE;
    }
    cmprtn          = strcmp( (char*)p_flmng1->ckcd, DNAND_FS_CKCD );
    if( cmprtn == 0 )
    {
        flg_valid[1]    = DNAND_TRUE;
    }
    
    if( (flg_valid[0] != DNAND_FALSE)&&(flg_valid[0] == flg_valid[1]) )
    {
       inter_cpy_info( p_flmng0 );
       cmprtn       = memcmp( p_flmng0, p_flmng1, sizeof(dnand_fs_flmng) );
       if( cmprtn != 0 )
       {
           rtn    = update_flmng( 1, p_flmng0 );
       }
    }
    else if( flg_valid[0] != DNAND_FALSE )
    {
        inter_cpy_info( p_flmng0 );
        rtn       = update_flmng( 1, p_flmng0 );
    }
    else if( flg_valid[1] != DNAND_FALSE )
    {
        inter_cpy_info( p_flmng1 );
        rtn       = update_flmng( 0, p_flmng1 );
    }
    else
    {
        inter_init_fsinfo();
        rtn       = update_flmng( 0, p_flmng0 );
        if( rtn == DNAND_NO_ERROR )
        {
            rtn       = update_flmng( 1, p_flmng0 );
        }
    }

#ifdef IMAGE_MODEM_PROC
#else
    kfree(p_kbuf0);
    kfree(p_kbuf1);
#endif
    if( rtn == DNAND_NO_ERROR )
    {
        dnand_fs_init = DNAND_INITIALIZED;
    }
    return(rtn);
}

#ifdef IMAGE_MODEM_PROC
static uint16   get_clid( uint32    cid )
#else
static uint16_t get_clid( uint32_t  cid )
#endif
{
#ifdef IMAGE_MODEM_PROC
    uint16        blk;
#else
    uint16_t      blk;
#endif

    if( cid >= DNAND_FS_CLID_NUM )
    {
        return(DNAND_FS_BLK_UNUSE);
    }
#ifdef IMAGE_MODEM_PROC
    blk    = (uint16)dnand_fsinfo[0].clid[cid];
#else
    blk    = (uint16_t)dnand_fsinfo[0].clid[cid];
#endif
    if( (blk < DNAND_FS_BLK_DATA_ST)||(blk >= DNAND_FS_BLK_NUM) )
    {
        blk    = DNAND_FS_BLK_UNUSE;
    }
    return(blk);
}

#ifdef IMAGE_MODEM_PROC
static void set_clid( uint32    cid, uint16    pos )
#else
static void set_clid( uint32_t  cid, uint16_t  pos )
#endif
{
#ifdef IMAGE_MODEM_PROC
    uint32        blk;
#else
    uint32_t      blk;
#endif

    blk        = pos;
    if( cid >= DNAND_FS_CLID_NUM )
    {
        return;
    }
    if( (blk < DNAND_FS_BLK_DATA_ST)||(blk >= DNAND_FS_BLK_NUM) )
    {
        blk    = DNAND_FS_BLK_UNUSE;
    }

    dnand_fsinfo[0].clid[cid]    = blk;
    return;
}

#ifdef IMAGE_MODEM_PROC
static uint16   get_fat( uint16    blk )
#else
static uint16_t get_fat( uint16_t  blk )
#endif
{
#ifdef IMAGE_MODEM_PROC
    uint16        nxtblk;
#else
    uint16_t      nxtblk;
#endif

    if( (blk < DNAND_FS_BLK_DATA_ST)||(blk >= DNAND_FS_BLK_NUM) )
    {
        nxtblk    = DNAND_FS_BLK_UNUSE;
    }
    else
    {
#ifdef IMAGE_MODEM_PROC
        nxtblk    = (uint16)dnand_fsinfo[0].fat[blk];
#else
        nxtblk    = (uint16_t)dnand_fsinfo[0].fat[blk];
#endif
    }
    return(nxtblk);
}

#ifdef IMAGE_MODEM_PROC
static void set_fat( uint16    pos, uint16    blk )
#else
static void set_fat( uint16_t  pos, uint16_t  blk )
#endif
{
    if( (pos < DNAND_FS_BLK_DATA_ST)||(pos >= DNAND_FS_BLK_NUM) )
    {
        return;
    }
    dnand_fsinfo[0].fat[pos]    = blk;
    return;
}

#ifdef IMAGE_MODEM_PROC
static uint16   alloc_fat( void )
#else
static uint16_t alloc_fat( void )
#endif
{
#ifdef IMAGE_MODEM_PROC
    uint32        i, blk;
    uint16        rtnblk, tmpblk;
#else
    uint32_t      i, blk;
    uint16_t      rtnblk, tmpblk;
#endif

    if( (dnand_alloc_start<DNAND_FS_BLK_DATA_ST) ||
        (dnand_alloc_start>=DNAND_FS_BLK_NUM) )
    {
        dnand_alloc_start = DNAND_FS_BLK_DATA_ST;
    }

    rtnblk    = DNAND_FS_BLK_UNUSE;
    for( i=DNAND_FS_BLK_DATA_ST; i<DNAND_FS_BLK_NUM; i++ )
    {
        blk    = dnand_fsinfo[0].fat[dnand_alloc_start];
        tmpblk = dnand_alloc_start;

        dnand_alloc_start++;
        if( dnand_alloc_start >= DNAND_FS_BLK_NUM )
        {
            dnand_alloc_start = DNAND_FS_BLK_DATA_ST;
        }

        if( blk == DNAND_FS_BLK_UNUSE )
        {
            rtnblk    = tmpblk;
            break;
        }
    }
    if( (rtnblk < DNAND_FS_BLK_DATA_ST)||(rtnblk >= DNAND_FS_BLK_NUM) )
    {
        rtnblk    = DNAND_FS_BLK_UNUSE;
    }
    return(rtnblk);
}

#ifdef IMAGE_MODEM_PROC
static int32   inter_fs_read(
            uint16              blk,
            uint32              offset,
            uint8               *pbuf,
            uint32              size )
#else
static int32_t inter_fs_read(
            uint16_t            blk,
            uint32_t            offset,
            uint8_t             *pbuf,
            uint32_t            size )
#endif
{
#ifdef IMAGE_MODEM_PROC
    int32         rtn;
    uint32        i, cnt, c_size;
#else
    int32_t       rtn;
    uint32_t      i, cnt, c_size;
#endif

    rtn       = DNAND_NO_ERROR;
    if( size == 0 )
    {
        return(rtn);
    }
    
    cnt       = (offset+size+DNAND_FS_BLK_SIZE-1)/DNAND_FS_BLK_SIZE;
    for( i=0; i<cnt; i++ )
    {
        if( offset >= DNAND_FS_BLK_SIZE )
        {
            offset    -= DNAND_FS_BLK_SIZE;
            blk       = get_fat( blk );
            continue;
        }
        if( (offset+size) >= DNAND_FS_BLK_SIZE )
        {
            c_size    = DNAND_FS_BLK_SIZE-offset;
        }
        else
        {
            c_size    = size;
        }
        if( (blk < DNAND_FS_BLK_DATA_ST)||(blk >= DNAND_FS_BLK_NUM) )
        {
            rtn       = DNAND_EOF_ERROR;
            break;
        }
        else
        {
            rtn    = dnand_blk_read( blk, offset, pbuf, c_size );
            if( rtn != DNAND_NO_ERROR )
            {
                break;
            }
        }
        offset    = 0;
        size      -= c_size;
        pbuf      += c_size;
        
        blk       = get_fat( blk );
    }
    return(rtn);
}

#ifdef IMAGE_MODEM_PROC
static int32   inter_fs_write(
            uint32              cid,
            uint16              blk,
            uint32              offset,
            uint8               *pbuf,
            uint32              size )
#else
static int32_t inter_fs_write(
            uint32_t            cid,
            uint16_t            blk,
            uint32_t            offset,
            uint8_t             *pbuf,
            uint32_t            size )
#endif
{
#ifdef IMAGE_MODEM_PROC
    int32         rtn;
    uint32        i, cnt, c_size;
    uint16        a_blk, prevblk, nextblk;
    uint8         *p_kbuf;
#else
    int32_t       rtn;
    uint32_t      i, cnt, c_size;
    uint16_t      a_blk, prevblk, nextblk;
    uint8_t       *p_kbuf;
#endif

    if( size == 0 )
    {
        return(DNAND_NO_ERROR);
    }
    if( cid >= DNAND_FS_CLID_NUM )
    {
        return(DNAND_PARAM_ERROR);
    }
    if( ( (blk<DNAND_FS_BLK_DATA_ST)||(blk>=DNAND_FS_BLK_NUM) )&&
        (blk != DNAND_FS_BLK_UNUSE) )
    {
        return(DNAND_MNG_ERROR);
    }
    
    rtn       = DNAND_NO_ERROR;
    cnt       = (offset+size+DNAND_FS_BLK_SIZE-1)/DNAND_FS_BLK_SIZE;
    a_blk  = alloc_fat();
    if( a_blk >= DNAND_FS_BLK_NUM )
    {
        return(DNAND_NOSPC_ERROR);
    }

    prevblk   = DNAND_FS_BLK_UNUSE;
#ifdef IMAGE_MODEM_PROC
    p_kbuf    = (uint8*)&dnand_fs_rw_blockbuf[0];
#else
    p_kbuf    = (uint8_t*)kmalloc(DNAND_FS_BLK_SIZE, GFP_KERNEL);
#endif
    if( p_kbuf == NULL )
    {
        return(DNAND_NOMEM_ERROR);
    }

    for( i=0; i<cnt; i++ )
    {
        if( offset >= DNAND_FS_BLK_SIZE )
        {
            if( (blk>=DNAND_FS_BLK_DATA_ST)&&(blk<DNAND_FS_BLK_NUM) )
            {
                offset    -= DNAND_FS_BLK_SIZE;
                prevblk   = blk;
                blk       = get_fat( prevblk );
                continue;
            }
            memset(p_kbuf, 0x00, DNAND_FS_BLK_SIZE);
            rtn           = dnand_blk_write( a_blk, 0, p_kbuf, DNAND_FS_BLK_SIZE );
            offset        -= DNAND_FS_BLK_SIZE;
        }
        else
        {
            if( (offset+size) >= DNAND_FS_BLK_SIZE )
            {
                c_size    = DNAND_FS_BLK_SIZE-offset;
            }
            else
            {
                c_size    = size;
            }
            
            if( c_size < DNAND_FS_BLK_SIZE )
            {
                if( (blk>=DNAND_FS_BLK_DATA_ST)&&(blk<DNAND_FS_BLK_NUM) )
                {
                    rtn    = dnand_blk_read( blk, 0, p_kbuf, DNAND_FS_BLK_SIZE );
                    if( rtn == DNAND_NO_ERROR )
                    {
                        memcpy( p_kbuf+offset, pbuf, c_size );
                        rtn    = dnand_blk_write( a_blk, 0, p_kbuf, DNAND_FS_BLK_SIZE );
                    }
                }
                else
                {
                    memset( p_kbuf, 0x00, DNAND_FS_BLK_SIZE );
                    memcpy( p_kbuf+offset, pbuf, c_size );
                    rtn    = dnand_blk_write( a_blk, 0, p_kbuf, DNAND_FS_BLK_SIZE );
                }
            }
            else
            {
                rtn    = dnand_blk_write( a_blk, 0, pbuf, c_size );
            }
            offset = 0;
            size   -= c_size;
            pbuf   += c_size;
        }
        if( rtn != DNAND_NO_ERROR )
        {
            break;
        }
        
        nextblk   = get_fat( blk );
        blk       = nextblk;
        if(  ((nextblk >= DNAND_FS_BLK_DATA_ST)&&(nextblk < DNAND_FS_BLK_NUM)) ||
             (nextblk == DNAND_FS_BLK_EOF) )
        {
            set_fat( a_blk, nextblk );
        }
        else
        {
            set_fat( a_blk, DNAND_FS_BLK_EOF );
        }
        
        if( (prevblk >= DNAND_FS_BLK_DATA_ST)&&(prevblk < DNAND_FS_BLK_NUM) )
        {
            set_fat( prevblk, a_blk );
        }
        if( i == 0 )
        {
            set_clid( cid, a_blk );
        }
        prevblk   = a_blk;

        if( (i+1) < cnt )
        {
            a_blk     = alloc_fat();
            if( a_blk >= DNAND_FS_BLK_NUM )
            {
                rtn    = DNAND_NOSPC_ERROR;
                break;
            }
        }
    }

#ifdef IMAGE_MODEM_PROC
#else
    kfree(p_kbuf);
#endif
    return(rtn);
}

#ifdef IMAGE_MODEM_PROC
int32          dnand_fs_read(
                uint32              cid,
                uint32              offset,
                uint8               *pbuf,
                uint32              size )
#else
int32_t        dnand_fs_read(
                uint32_t            cid,
                uint32_t            offset,
                uint8_t             *pbuf,
                uint32_t            size )
#endif
{
#ifdef IMAGE_MODEM_PROC
    int32        rtn;
    uint16       blk;
#else
    int32_t      rtn;
    uint16_t     blk;
#endif

    if( pbuf == NULL )
    {
        return(DNAND_PARAM_ERROR);
    }
    
    if( cid >= DNAND_ID_ENUM_MAX )
    {
        return(DNAND_PARAM_ERROR);
    }
    if( cid >= DNAND_FS_CLID_NUM )
    {
        return(DNAND_PARAM_ERROR);
    }
    
    memset( pbuf, 0x00, size );
    
#ifdef IMAGE_MODEM_PROC
    dnand_sdcc_access_enable();
#endif
    
    rtn    = pre_init();
    if( rtn != DNAND_NO_ERROR )
    {
#ifdef IMAGE_MODEM_PROC
        dnand_sdcc_access_disable();
#endif
        return(rtn);
    }

    blk    = get_clid( cid );
    if( blk < DNAND_FS_BLK_DATA_ST )
    {
#ifdef IMAGE_MODEM_PROC
        dnand_sdcc_access_disable();
#endif
        return(DNAND_MNG_ERROR);
    }
    if( blk >= DNAND_FS_BLK_NUM )
    {
#ifdef IMAGE_MODEM_PROC
        dnand_sdcc_access_disable();
#endif
        return(DNAND_NOEXISTS_ERROR);
    }
    rtn    = inter_fs_read( blk, offset, pbuf, size );

#ifdef IMAGE_MODEM_PROC
        dnand_sdcc_access_disable();
#endif
    return(rtn);
}

#ifdef IMAGE_MODEM_PROC
int32          dnand_fs_write(
                uint32              cid,
                uint32              offset,
                uint8               *pbuf,
                uint32              size )
#else
int32_t        dnand_fs_write(
                uint32_t            cid,
                uint32_t            offset,
                uint8_t             *pbuf,
                uint32_t            size )
#endif
{
#ifdef IMAGE_MODEM_PROC
    int32        rtn;
    uint16       blk;
#else
    int32_t      rtn;
    uint16_t     blk;
#endif

    if( pbuf == NULL )
    {
        return(DNAND_PARAM_ERROR);
    }
    if( cid >= DNAND_ID_ENUM_MAX )
    {
        return(DNAND_PARAM_ERROR);
    }
    if( cid >= DNAND_FS_CLID_NUM )
    {
        return(DNAND_PARAM_ERROR);
    }
    
    rtn    = pre_init();
    if( rtn != DNAND_NO_ERROR )
    {
        return(rtn);
    }
    
    blk    = get_clid( cid );
    rtn    = inter_fs_write( cid, blk, offset, pbuf, size );
    if( rtn == DNAND_NO_ERROR )
    {
        rtn    = update_memng( cid );
    }
    return(rtn);
}

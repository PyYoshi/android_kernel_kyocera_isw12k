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
    #include <linux/dnand_cdev_driver.h>
#endif

#ifdef IMAGE_MODEM_PROC
    #include "dnand_clid.h"
#else
    #include <linux/dnand_clid.h>
#endif
#include "dnand_fs.h"
#include "dnand_fs_mng.h"


#ifdef IMAGE_MODEM_PROC
#else
    static DEFINE_MUTEX(dnand_lock);
#endif

#ifdef IMAGE_MODEM_PROC
static int32   inter_param_check(
            uint32              cid,
            uint32              offset,
            uint8               *pbuf,
            uint32              size )
#else
static int32_t inter_param_check(
            uint32_t            cid,
            uint32_t            offset,
            uint8_t             *pbuf,
            uint32_t            size )
#endif
{
#ifdef IMAGE_MODEM_PROC
    int32    rtn;
#else
    int32_t  rtn;
#endif
    
    rtn    = DNAND_NO_ERROR;
    if( cid >= DNAND_ID_ENUM_MAX )
    {
        rtn    = DNAND_PARAM_ERROR;
    }
    if( pbuf == NULL )
    {
        rtn    = DNAND_PARAM_ERROR;
    }
    if( size == 0 )
    {
        rtn    = DNAND_PARAM_ERROR;
    }
    return(rtn);
}

#ifdef IMAGE_MODEM_PROC
int32          kdnand_id_read(
                uint32              cid,
                uint32              offset,
                uint8               *pbuf,
                uint32              size )
#else
int32_t        kdnand_id_read(
                uint32_t            cid,
                uint32_t            offset,
                uint8_t             *pbuf,
                uint32_t            size )
#endif
{
#ifdef IMAGE_MODEM_PROC
    int32        rtn;
#else
    int32_t      rtn;
#endif

    rtn    = inter_param_check( cid, offset, pbuf, size );
    if( rtn != DNAND_NO_ERROR )
    {
        if( size == 0 )
        {
            rtn    = DNAND_NO_ERROR;
        }
        return(rtn);
    }

#ifdef IMAGE_MODEM_PROC
#else
    mutex_lock(&dnand_lock);
#endif
    rtn    = dnand_fs_read(cid, offset, pbuf, size);
#ifdef IMAGE_MODEM_PROC
#else
    mutex_unlock(&dnand_lock);
#endif
    return(rtn);
}

#ifdef IMAGE_MODEM_PROC
int32          kdnand_id_write(
                uint32              cid,
                uint32              offset,
                uint8               *pbuf,
                uint32              size )
#else
int32_t        kdnand_id_write(
                uint32_t            cid,
                uint32_t            offset,
                uint8_t             *pbuf,
                uint32_t            size )
#endif
{
#ifdef IMAGE_MODEM_PROC
    int32        rtn;
#else
    int32_t      rtn;
#endif

    rtn    = inter_param_check( cid, offset, pbuf, size );
    if( rtn != DNAND_NO_ERROR )
    {
        if( size == 0 )
        {
            rtn    = DNAND_NO_ERROR;
        }
        return(rtn);
    }

#ifdef IMAGE_MODEM_PROC
#else
    mutex_lock(&dnand_lock);
#endif
    rtn    = dnand_fs_write(cid, offset, pbuf, size);
#ifdef IMAGE_MODEM_PROC
#else
    mutex_unlock(&dnand_lock);
#endif
    return(rtn);
}

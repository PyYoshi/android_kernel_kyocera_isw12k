#ifndef DNAND_FS_H
#define DNAND_FS_H
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
    #include "dnand_clid.h"
    #include "dnand_status.h"
#else
    #include <linux/types.h>
    #include <linux/dnand_clid.h>
    #include <linux/dnand_status.h>
#endif

#ifdef IMAGE_MODEM_PROC
    int32   dnand_fs_read( uint32 cid, uint32 offset, uint8 *pbuf, uint32 size );
#else
    int32_t dnand_fs_read( uint32_t cid, uint32_t offset, uint8_t *pbuf, uint32_t size );
#endif

#ifdef IMAGE_MODEM_PROC
    int32   dnand_fs_write( uint32 cid, uint32 offset, uint8 *pbuf, uint32 size );
#else
    int32_t dnand_fs_write( uint32_t cid, uint32_t offset, uint8_t *pbuf, uint32_t size );
#endif

#endif

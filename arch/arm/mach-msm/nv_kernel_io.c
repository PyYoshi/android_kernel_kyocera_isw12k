/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <NV_KERNEL_IO.C>

This software is contributed or developed by KYOCERA Corporation.
(C) 2011 KYOCERA Corporation
(C) 2012 KYOCERA Corporation

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 and
only version 2 as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
02110-1301, USA.

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
#include <mach/sdio_al.h>
#include <linux/semaphore.h>
#include "proc_comm.h"
#include "smd_private.h"
#include <mach/msm_smsm.h>
#include <mach/nv_kernel_io.h>

#define NV_SMEM_SIZE        NV_SMEM_HEADER_SIZE + NV_SMEM_DATA_SIZE
#define NV_SMEM_HEADER_SIZE 1
#define NV_SMEM_DATA_SIZE   32768

#define NV_SEM_VAL          1
static struct semaphore     nv_sem;

void nv_kernel_init(void)
{
    sema_init( &nv_sem, NV_SEM_VAL );
    return;
}

int nv_cmd_kernel(enum nv_func_enum_type cmd, unsigned item, unsigned size, void* data_ptr)
{
    unsigned func;
    void     *smem_ptr;
    int      result = 0;

    down(&nv_sem);

    if(size > NV_SMEM_DATA_SIZE || data_ptr == NULL){
        printk(KERN_ERR "nv_cmd_kernel : Bad Param");
        result = -1;
    }
    
    if(result == 0){
        switch(cmd)
        {
            case NV_WRITE_F:
                func = PCOM_NV_WRITE;
            break;
            
            case NV_READ_F:
                func = PCOM_NV_READ;
            break;
            
            default:
                printk(KERN_ERR "nv_cmd_kernel : Bad Cmd");
                result = -1;
        }
    }

    if(result == 0){
        smem_ptr = kc_smem_alloc(SMEM_NV_DATA, NV_SMEM_SIZE);
        
        if(smem_ptr == NULL){
            printk(KERN_ERR "nv kc_smem_alloc: failed");
            result = -1;
        }
    }

    if(result == 0){
        *(char*)smem_ptr = 1;
        memcpy(((char*)smem_ptr + NV_SMEM_HEADER_SIZE), data_ptr, size);
        
        result = msm_proc_comm(func, &item, &size);

        if(result == 0 && cmd == NV_READ_F)
        {
            memcpy(data_ptr, ((char*)smem_ptr + NV_SMEM_HEADER_SIZE), size);
        }
        *(char*)smem_ptr = 0;
    }
    
    up(&nv_sem);

    if(result == 0){
        return 0;
    }
    else{
        return -1;
    }
}
EXPORT_SYMBOL(nv_cmd_kernel);

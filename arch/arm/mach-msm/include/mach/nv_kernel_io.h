#ifndef NV_KERNEL_IO_H
#define NV_KERNEL_IO_H
/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <nv_kernel_io.h>

This software is contributed or developed by KYOCERA Corporation.
(C) 2011 KYOCERA Corporation

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

enum nv_func_enum_type {
  NV_READ_F,
  NV_WRITE_F,
};

extern void nv_kernel_init(void);

extern int nv_cmd_kernel(unsigned cmd, unsigned item, unsigned size, void* data_ptr);

#endif /* NV_KERNEL_IO_H */

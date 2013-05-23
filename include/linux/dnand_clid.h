#ifndef DNAND_CLID_H
#define DNAND_CLID_H
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
#else
    #include <linux/types.h>
#endif

typedef enum
{
  DNAND_ID_KERNEL_0 = 0,
  DNAND_ID_KERNEL_1,
  DNAND_ID_KERNEL_2,
  DNAND_ID_KERNEL_3,
  DNAND_ID_KERNEL_4,
  DNAND_ID_KERNEL_5,
  DNAND_ID_KERNEL_6,
  DNAND_ID_KERNEL_7,
  DNAND_ID_KERNEL_8,
  DNAND_ID_KERNEL_9,
  DNAND_ID_KERNEL_10,
  DNAND_ID_KERNEL_11,
  DNAND_ID_KERNEL_12,
  DNAND_ID_KERNEL_13,
  DNAND_ID_KERNEL_14,
  DNAND_ID_KERNEL_15,
  DNAND_ID_KERNEL_16,

  DNAND_ID_ENUM_MAX
} dnand_id_enum_type;

#endif

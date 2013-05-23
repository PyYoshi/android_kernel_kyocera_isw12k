/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 */
/*
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

#ifndef CE150X_H
#define CE150X_H

//#define CE150X_CALL_LOG_ON
//#define CE150X_LOG_ON

#include <mach/board.h>
#include <linux/kernel.h>

#ifdef CE150X_CALL_LOG_ON
 #define CE150X_ENTER   printk(KERN_INFO "CE150X:%s enter\n", __func__)
 #define CE150X_RETURN  { \
  printk(KERN_INFO "CE150X:%s(%d) exit\n", __func__, __LINE__); \
  return; \
  }
 #define CE150X_RETURN_N(x) { \
  printk(KERN_INFO "CE150X:%s(%d) exit(%ld)\n", \
  __func__, __LINE__, (unsigned long)x); \
  return x; \
 }
#else
 #define CE150X_ENTER
 #define CE150X_RETURN             return
 #define CE150X_RETURN_N(x)        return x
#endif /* CE150X_CALL_LOG_ON */


#ifdef CE150X_LOG_ON
#define CE150X_LOG_OUT(label, fmt, ...) \
  printk(KERN_INFO "CE150X:%s +%-4d %-28s "fmt"%s\n", \
  label, __LINE__, __func__, __VA_ARGS__)

#define CE150X_LOG_ERR(...)  CE150X_LOG_OUT("ERR", __VA_ARGS__, "")
#define CE150X_LOG_DBG(...)  CE150X_LOG_OUT("DBG", __VA_ARGS__, "")
#else
#define CE150X_LOG_ERR(...)
#define CE150X_LOG_DBG(...)
#endif /* CE150X_LOG_ON */

#endif /* CE150X */


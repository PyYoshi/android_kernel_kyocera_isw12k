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

#ifndef RJ6CBA200_H
#define RJ6CBA200_H

#define RJ6CBA200_CALL_LOG_ON
#define RJ6CBA200_LOG_ON

#include <mach/board.h>
#include <linux/kernel.h>

#ifdef RJ6CBA200_CALL_LOG_ON
 #define RJ6CBA200_ENTER   printk(KERN_INFO "RJ6CBA200:%s enter\n", __func__)
 #define RJ6CBA200_RETURN  { \
  printk(KERN_INFO "RJ6CBA200:%s(%d) exit\n", __func__, __LINE__); \
  return; \
  }
 #define RJ6CBA200_RETURN_N(x) { \
  printk(KERN_INFO "RJ6CBA200:%s(%d) exit(%ld)\n", \
  __func__, __LINE__, (unsigned long)x); \
  return x; \
 }
#else
 #define RJ6CBA200_ENTER
 #define RJ6CBA200_RETURN             return
 #define RJ6CBA200_RETURN_N(x)        return x
#endif /* RJ6CBA200_CALL_LOG_ON */


#ifdef RJ6CBA200_LOG_ON
#define RJ6CBA200_LOG_OUT(label, fmt, ...) \
  printk(KERN_INFO "RJ6CBA200:%s:%s +%-4d %-28s "fmt"%s\n", \
  label, __FILE__, __LINE__, __func__, __VA_ARGS__)

#define RJ6CBA200_LOG_ERR(...)  RJ6CBA200_LOG_OUT("ERR", __VA_ARGS__, "")
#define RJ6CBA200_LOG_DBG(...)  RJ6CBA200_LOG_OUT("DBG", __VA_ARGS__, "")
#else
#define RJ6CBA200_LOG_ERR(...)
#define RJ6CBA200_LOG_DBG(...)
#endif /* RJ6CBA200_LOG_ON */

enum rj6cba200_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};

enum rj6cba200_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

#endif /* RJ6CBA200 */


/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ARCH_ARM_MACH_MSM_BOARD_7X30_WIFI_H__
#define __ARCH_ARM_MACH_MSM_BOARD_7X30_WIFI_H__

extern int msm7x30_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id);

extern unsigned int msm7x30_wifi_status(struct device *dev);

#ifndef CONFIG_BCMDHD_MODULE
extern int msm7x30_wifi_init(void);
#endif

#endif

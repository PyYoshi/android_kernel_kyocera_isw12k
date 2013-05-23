/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 */
/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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
 */

#ifndef __ASM_ARCH_MSM_RPC_SERVER_HANDSET_H
#define __ASM_ARCH_MSM_RPC_SERVER_HANDSET_H

struct msm_handset_platform_data {
	const char *hs_name;
	uint32_t pwr_key_delay_ms; /* default 500ms */
};

void report_headset_status(bool connected);

#ifdef CONFIG_FEATURE_KCC_00
extern void kc_hs_switch_change_state(bool switch_state);
extern void kc_hs_jack_change_state(bool jack_state);
#endif
#endif

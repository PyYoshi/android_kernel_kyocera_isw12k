/* security/kyocera/kclsm.h
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 */
#ifndef __KCLSM_H__
#define __KCLSM_H__

#ifdef KCLSM_DEBUG
#define KCLSM_RETURN(ret) (0)
#else
#define KCLSM_RETURN(ret) (ret)
#endif

#define KCLSM_SYSTEM_MOUNT_POINT "/system"
#define KCLSM_DEVICE_DIR "/dev"


#ifdef CONFIG_SECURITY_KCLSM_INSMOD
static const char *kclsm_module_checklist[] = {
	"alps_input",
	"ansi_cprng",
	"bcmwimax",
	"bcmsdio",
	"cls_flow",
	"cpaccess",
	"dal_remotetest",
	"dma_test",
	"evbug",
	"felica",
	"gp2ap003a10f",
	"gspca_main",
	"hscd_i2c",
	"kc_sdcarddrv",
	"kwimax_gpio_wrapper",
	"kwimax_onoff",
	"leds_ledlight",
	"libra_ftm",
	"librasdioif",
	"mtd_erasepart",
	"mtd_nandecctest",
	"mtd_oobtest",
	"mtd_pagetest",
	"mtd_readtest",
	"mtd_speedtest",
	"mtd_stresstest",
	"mtd_subpagetest",
	"mtd_torturetest",
	"oprofile",
	"qce",
	"qcedev",
	"qcrypto",
	"reset_modem",
	"sch_dsmark",
	"scsi_wait_scan",
	"WCN1314_rf_ftm",
	"wlan",
	"kc_sdgdrv",
	"ml610q792",
	"bcmdhd",
};
#endif
#ifdef CONFIG_SECURITY_KCLSM_MOUNT
static const char *kclsm_mount_checklist[] = {
	"/dev/block/mmcblk0p12",
	NULL
};
#endif
#ifdef CONFIG_SECURITY_KCLSM_DEVICEPERMISSION
static const char *kclsm_device_checklist[] = {
	NULL
};
#endif

#endif

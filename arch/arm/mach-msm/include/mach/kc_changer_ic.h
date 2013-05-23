/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
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

#ifndef KC_CHANGER_IC_H
#define KC_CHANGER_IC_H

typedef enum {
    KC_CHANGER_UNINITIALIZE        = 0x00,
    KC_CHANGER_NO_ACCESSORY        = 0xF8,
    KC_CHANGER_AC_ADAPTER          = 0xFE,
    KC_CHANGER_USB_MODE            = 0xFC,
    KC_CHANGER_AUDIO_MIC_MONO      = 0xF0,
    KC_CHANGER_AUDIO_CHG_MIC_MONO  = 0xE4,
    KC_CHANGER_FAST_CHARGER        = 0xDE,
    KC_CHANGER_CARKIT_TYPE2        = 0xDC,
    KC_CHANGER_VIDEO_AUDIO         = 0xD0,
    KC_CHANGER_VIDEO_AUDIO_CHG     = 0xD4,
    KC_CHANGER_AUDIO_MIC_STEREO    = 0xC0,
    KC_CHANGER_AUDIO_CHG_STEREO    = 0xC4,
    KC_CHANGER_STD_CHARGER         = 0xBE,
    KC_CHANGER_CARKIT_TYPE1        = 0xBC,
    KC_CHANGER_UART                = 0xB0,
    KC_CHANGER_RID_A               = 0xAC,
    KC_CHANGER_AUDIO_STEREO        = 0xA0,
    KC_CHANGER_RID_B               = 0x9C,
    KC_CHANGER_RID_C               = 0x8C,
    KC_CHANGER_MIC_STEREO          = 0xC8,
} kc_changer_ic_accessory_enum;

typedef enum {
    KC_CHANGER_OVP_NOT_DETECTION   = 0x00,
    KC_CHANGER_OVP_DETECTION,
} kc_changer_ic_ovp_detection_enum;

typedef void (*kc_changer_ic_event_func)(kc_changer_ic_accessory_enum accessoryinfo);
typedef void (*kc_changer_ic_ovp_det_cb)(kc_changer_ic_ovp_detection_enum ovp_state);

struct kc_changer_ic_event_callback {
    kc_changer_ic_event_func fn;
};

extern int32_t kc_changer_ic_reg_cbfunc(struct kc_changer_ic_event_callback* cb);
extern int32_t kc_changer_ic_unreg_cbfunc(struct kc_changer_ic_event_callback* cb);
extern kc_changer_ic_accessory_enum kc_changer_ic_get_accessory(void);

extern int32_t kc_changer_ic_ovp_det_reg_cbfunc(kc_changer_ic_ovp_det_cb cb);
extern int32_t kc_changer_ic_ovp_det_unreg_cbfunc(kc_changer_ic_ovp_det_cb cb);
extern kc_changer_ic_ovp_detection_enum kc_changer_ic_get_ovp_state(void);

static inline bool kc_changer_ic_is_audio(kc_changer_ic_accessory_enum value)
{
    bool ret = false;
    switch (value) {
    case KC_CHANGER_AUDIO_MIC_MONO :    
    case KC_CHANGER_AUDIO_CHG_MIC_MONO :
    case KC_CHANGER_AUDIO_MIC_STEREO :  
    case KC_CHANGER_AUDIO_CHG_STEREO :  
    case KC_CHANGER_AUDIO_STEREO : 
        ret = true;
        break;
    default : 
        break;
    }
    return ret;
}
static inline bool kc_changer_ic_is_fix_accessory(u8 value)
{
    bool ret = false;
    switch (value) {
    case KC_CHANGER_AC_ADAPTER :        
    case KC_CHANGER_USB_MODE :          
    case KC_CHANGER_AUDIO_MIC_MONO :    
    case KC_CHANGER_AUDIO_CHG_MIC_MONO :
    case KC_CHANGER_FAST_CHARGER :      
    case KC_CHANGER_CARKIT_TYPE2 :      
    case KC_CHANGER_VIDEO_AUDIO :       
    case KC_CHANGER_VIDEO_AUDIO_CHG :   
    case KC_CHANGER_AUDIO_MIC_STEREO :  
    case KC_CHANGER_AUDIO_CHG_STEREO :  
    case KC_CHANGER_STD_CHARGER :       
    case KC_CHANGER_CARKIT_TYPE1 :      
    case KC_CHANGER_UART :              
    case KC_CHANGER_RID_A :             
    case KC_CHANGER_AUDIO_STEREO :      
    case KC_CHANGER_RID_B :             
    case KC_CHANGER_RID_C :             
    case KC_CHANGER_MIC_STEREO :        
    case KC_CHANGER_NO_ACCESSORY :
        ret = true;
        break;
    case KC_CHANGER_UNINITIALIZE :      
    default : 
        break;
    }
    return ret;
}

#endif

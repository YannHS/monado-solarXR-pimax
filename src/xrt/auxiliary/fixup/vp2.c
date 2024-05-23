
#include "vp2.h"

#include "string.h"

#include "math/m_api.h"
#include "util/u_debug.h"

const uint8_t set_mode_str1[] = "wireless,0";

DEBUG_GET_ONCE_NUM_OPTION(vp2_mode_index, "VP2_MODE_INDEX", 0)

void vp2_set_feature(hid_device* hid, uint8_t report, uint16_t sub_id, const uint8_t* data, size_t len){
    uint8_t buf[64];
    buf[0] = report;
    *(uint16_t*)(buf+1) = sub_id;   // This only works on little_endian machines!
    buf[3] = len;
    memcpy(buf+4, data, MIN(len, ARRAY_SIZE(buf) - 4));
    hid_send_feature_report(hid, buf, MIN(sizeof(buf), len+4));
}

void vp2_set_mode(hid_device* hid, uint8_t mode){

    // valid mode indicies are 0 to 5
    if(mode > 5){
        mode = 5;
    }

    vp2_set_feature(hid, 0x04, 0x2970, set_mode_str1, ARRAY_SIZE(set_mode_str1)-1);
    
    uint8_t buf[] = "dtd,0";
    buf[4] = '0' + mode;   // select the mode
    vp2_set_feature(hid, 0x04, 0x2970, buf, ARRAY_SIZE(buf)-1);
}


long init_vivepro2(struct fixup_context* ctx, struct fixup_func_list* funcs, struct hid_device_info* devinfo){

    hid_device* hid;
    hid = hid_open(VP2_VID, VP2_PID, NULL);
    if(hid){
        U_LOG_E("Failed to open VP2 HID device");
		return 0;
    }
    uint8_t mode = debug_get_num_option_vp2_mode_index();
    vp2_set_mode(hid, mode);
    hid_close(hid);
    return 3000;
}
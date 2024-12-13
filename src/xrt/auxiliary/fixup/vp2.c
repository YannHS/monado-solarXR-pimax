// Copyright 2024, Coreforge
// SPDX-License-Identifier: BSL-1.0

// the initialization packets were derived from https://github.com/CertainLach/VivePro2-Linux-Driver

#include "vp2.h"

#include "string.h"

#include "util/u_debug.h"

DEBUG_GET_ONCE_NUM_OPTION(vp2_mode_index, "VP2_MODE_INDEX", 0)

long init_vivepro2(struct fixup_context* ctx, struct fixup_func_list* funcs, struct hid_device_info* devinfo){

    hid_device* hid;
    hid = hid_open(VP2_VID, VP2_PID, NULL);
    if(hid){
        U_LOG_E("Failed to open VP2 HID device");
		return 0;
    }

    uint8_t cmd1[64] = "\x04\x70\x29\x0b""wireless, 0";
    hid_send_feature_report(hid, cmd1, sizeof(cmd1));
    uint8_t cmd2[64] = "\x04\x70\x29\x05""dtd,0";
    uint8_t mode = debug_get_num_option_vp2_mode_index();
    cmd2[8] += mode;
    hid_send_feature_report(hid, cmd1, sizeof(cmd1));
    hid_close(hid);
    return 3000;
}
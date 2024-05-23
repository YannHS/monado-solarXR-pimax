#include "fixup.h"

#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_time.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "os/os_time.h"

#include "math/m_vec2.h"

#include "pimax.h"

extern struct fixup_definition fixups[];


void fixup_patch_devices(struct fixup_context* ctx, struct xrt_system_devices *xsysd){
    for(size_t i = 0; i < ctx->num_devices; i++){
        if(ctx->devices[i]->fixup_funcs->patch){
            ctx->devices[i]->fixup_funcs->patch(ctx->devices[i], ctx, xsysd);
        }
    }

}

int max_i(int a, int b){
    return a>b?a:b;
}

struct fixup_context* fixup_init_devices(){
    U_LOG_D("Quirk: checking for HID devices to init\n");

    struct fixup_context* ctx = U_TYPED_CALLOC(struct fixup_context);
    if(!ctx){
        U_LOG_E("Error allocating memory for fixup context\n");
        return NULL;
    }
    ctx->num_devices = 0;
    long delay = 0;
    struct fixup_definition* def = fixups;
    for(;def->filter.vid != 0;def++){

        if(!def->funcs.init){
            continue;
        }

        struct hid_device_info* devinfo = hid_enumerate(def->filter.vid, def->filter.pid);
        if(!devinfo){
            continue;
        }
        struct hid_device_info* current = devinfo;
        while(current){
            delay = max_i(def->funcs.init(ctx, &def->funcs, current), delay);
            current = current->next;
        }
    }
    
    U_LOG_D("Waiting %ldms for devices to finish starting\n", delay);
    os_nanosleep(delay * U_TIME_1MS_IN_NS);
    return ctx;
}
#pragma once
#ifdef __cplusplus
extern "C"{
#endif
#include <hidapi/hidapi.h>

#include "xrt/xrt_system.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#define FIXUP_MAX_DEVICES 8     // realistically, one would be enough (for now at least)


struct fixup_context;
struct fixup_func_list;
struct fixup_device;

/*
 *  Does any additional initialization the device may need (like waking up/
 *  configuring displays). Also has to create and add the fixup_device if 
 *  further steps are required (e.g. HMDs with custom distortion parameters)
 *
 */
// return: delay in ms to add to give devices time to init/register
// the maximum of all init functions will be waited after all init functions have returned
typedef long (*quirk_init_func_t)(struct fixup_context*, struct fixup_func_list* funcs, struct hid_device_info* devinfo);

// return: delay in ms to add to give devices time to init/register
// the maximum of all init functions will be waited after all init functions have returned
typedef void (*quirk_patch_func_t)(struct fixup_device* dev, struct fixup_context* ctx, struct xrt_system_devices *xsysd);

struct device_usb_filter{
    uint16_t vid;
    uint16_t pid;
};

struct fixup_func_list{
    quirk_init_func_t init;
    quirk_patch_func_t patch;
};

struct fixup_definition{
    struct device_usb_filter filter;
    struct fixup_func_list funcs;
};

struct fixup_device{
    struct xrt_device base;
    struct fixup_func_list* fixup_funcs;
};

struct fixup_context{
    struct fixup_device* devices[FIXUP_MAX_DEVICES];
    size_t num_devices;
};

struct fixup_context* fixup_init_devices();
void fixup_patch_devices(struct fixup_context* ctx, struct xrt_system_devices *xsysd);
#ifdef __cplusplus
}
#endif

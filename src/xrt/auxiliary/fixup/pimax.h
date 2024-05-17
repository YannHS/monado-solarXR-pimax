#pragma once
#include "fixup.h"

#include "os/os_threading.h"

#define PIMAX_VID 0x0483
#define PIMAX_8KX_PID 0x0101

#define PIMAX_POLL_TIMEOUT 5
#define PIMAX_POLL_WAIT 10
#define PIMAX_POLL_KEEPALIVE_WAIT_COUNT 50  // every x polls, send keepalive packets

#define PIMAX_MODEL_NAME_LENGTH 32

long init_pimax8kx(struct fixup_context* ctx, struct fixup_func_list* funcs, struct hid_device_info* devinfo);
void patch_pimax8kx(struct fixup_device* dev, struct fixup_context* ctx, struct xrt_system_devices *xsysd);

struct pimax_device;

struct pimax_display_properties{
    struct xrt_vec2 size_in_meters;
    uint32_t pixels_width;
    uint32_t pixels_height;
    float nominal_frame_interval_ns;
};

typedef void(*pimax_display_size_func_t)(struct pimax_device* dev, uint32_t* out_width, uint32_t* out_height);
typedef void(*pimax_display_get_props_func_t)(struct pimax_device* dev, struct pimax_display_properties* out_props);


struct pimax_model_funcs{
    //pimax_display_size_func_t get_display_size;
    pimax_display_get_props_func_t get_display_properties;
};

struct pimax_model_config{
    const wchar_t product_name[PIMAX_MODEL_NAME_LENGTH];    // the name in the USB descriptor
    const char display_name[PIMAX_MODEL_NAME_LENGTH];   // the name for the xdev
    struct pimax_model_funcs funcs;
};

struct pimax_device{

    struct fixup_device base;   // must be the first member

    struct pimax_model_funcs* model_funcs;

    hid_device* hid_dev;
    struct os_mutex hid_mutex;
    struct pimax_device_config{
        float ipd;
        float separation;
        bool upscaling;
    } device_config;

    struct os_thread poll_thread;
    bool should_poll;
    uint32_t polls_since_last_keepalive;
};
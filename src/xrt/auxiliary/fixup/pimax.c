
#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_time.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_var.h"
#include "util/u_file.h"

#include "../../drivers/multi_wrapper/multi.h"

#include "math/m_vec2.h"
#include "math/m_api.h"

#include <cjson/cJSON.h>

#include "pimax.h"

#define OFFSETS_MIN -0.15
#define OFFSETS_MAX 0.15
#define OFFSETS_STEP 0.00005


DEBUG_GET_ONCE_FLOAT_OPTION(pimax_ipd_offs_v0, "PIMAX_IPD_V0", 0.0)
DEBUG_GET_ONCE_FLOAT_OPTION(pimax_ipd_offs_v1, "PIMAX_IPD_V1", 0.0)
DEBUG_GET_ONCE_FLOAT_OPTION(pimax_ipd_offs_h0, "PIMAX_IPD_H0", 0.0)
DEBUG_GET_ONCE_FLOAT_OPTION(pimax_ipd_offs_h1, "PIMAX_IPD_H1", 0.0)
// required, as the resolution needs to be known
DEBUG_GET_ONCE_NUM_OPTION(pimax_desired_mode, "XRT_COMPOSITOR_DESIRED_MODE", -1)

// ugly workaround for the 5K XR until monado has proper subpixel shading (it's not as noticeable on other headsets)
DEBUG_GET_ONCE_FLOAT_OPTION(pimax_img_offs_x_r, "PIMAX_OFFS_X_R", 0.0)
DEBUG_GET_ONCE_FLOAT_OPTION(pimax_img_offs_x_g, "PIMAX_OFFS_X_G", 0.0)
DEBUG_GET_ONCE_FLOAT_OPTION(pimax_img_offs_x_b, "PIMAX_OFFS_X_B", 0.0)


// for loading meshes from json files instead of having them compiled in
#define PIMAX_MESHES_DEFAULT_PATH ".config/pimax/meshes"
#define PIMAX_MESHES_PATH_ENV_VAR "PIMAX_MESHES_PATH"   // folder to look in for meshes
#define PIMAX_MESH_NAME_ENV_VAR "PIMAX_MESH_NAME"       // file name of the mesh json

// forward declarations, these aren't needed anywhere else, so no need to put them into the header
void pimax_8kx_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props);
void pimax_5ks_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props);
void pimax_p2d_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props);
void pimax_p2c_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props);
void pimax_p2b_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props);
void pimax_p2ea_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props);
void pimax_p2z_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props);

struct pimax_model_config model_configs[] = {
    {L"Pimax P2EA", "Pimax 8K Plus", "p2ea.json", {pimax_p2ea_get_display_props}},
    {L"Pimax P2A", "Pimax 5K Super", "p2a.json", {pimax_5ks_get_display_props}},
    {L"Pimax P2C", "Pimax 5K Super", "p2c.json", {pimax_p2c_get_display_props}},
    {L"Pimax P2N", "Pimax 8KX", "p2n.json", {pimax_8kx_get_display_props}},
    {L"Pimax P2D", "Pimax 5K+", "p2d.json", {pimax_p2d_get_display_props}},
    {L"Pimax P2B", "Pimax 5K XR", "p2b.json", {pimax_p2b_get_display_props}},
    {L"Pimax P2Z", "Pimax 5K Super", "p2z.json", {pimax_p2z_get_display_props}},
};


uint8_t pimax_packet_parallel_projections_off[64] = {
    0xF0, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

uint8_t pimax_init2[64] = {
    0xF0, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xf6, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

uint8_t pimax_keepalive[64] = {
    0xF0, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

uint8_t pimax_hmd_power[64] = {
    0xF0, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

uint8_t pimax_poll_freq[6] = {0x11, 0x00, 0x00, 0x0b, 0x10, 0x27};

bool pimax_compute_distortion(
	    struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *out_result);

bool pimax_compute_distortion2(
	    struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *out_result);


static void
pimax_get_view_poses(struct xrt_device *xdev,
                           const struct xrt_vec3 *default_eye_relation,
                           int64_t at_timestamp_ns,
                           uint32_t view_count,
                           struct xrt_space_relation *out_head_relation,
                           struct xrt_fov *out_fovs,
                           struct xrt_pose *out_poses);

void pimax_8kx_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props){
    out_props->pixels_width = dev->device_config.upscaling ? 2160 : 3168;
    out_props->pixels_height = dev->device_config.upscaling ? 1440 : 2160;

    out_props->nominal_frame_interval_ns = 1000.*1000.*1000./(dev->device_config.upscaling ? 110. : 90.);
    out_props->refresh_rate = dev->device_config.upscaling ? 110 : 90;
}

void pimax_5ks_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props){
    out_props->pixels_width = 2160;
    out_props->pixels_height = 1440;

    out_props->nominal_frame_interval_ns = 1000.*1000.*1000./110.;
    out_props->refresh_rate = 110;
}

void pimax_p2d_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props){
    out_props->pixels_width = 2038;
    out_props->pixels_height = 1440;
    out_props->nominal_frame_interval_ns = 1000.*1000.*1000./90.;
    out_props->refresh_rate = 90;

    switch(debug_get_num_option_pimax_desired_mode()){
        case 0:
            out_props->nominal_frame_interval_ns = 1000.*1000.*1000./65.;
            out_props->refresh_rate = 65;
            break;
        case 1:
            // this is the one monado should already choose by default
            break;
        case 2:
            out_props->pixels_width = 1600;
            out_props->nominal_frame_interval_ns = 1000.*1000.*1000./120.;
            out_props->refresh_rate = 120;
            break;
        case 3:
            out_props->pixels_width = 1600;
            out_props->nominal_frame_interval_ns = 1000.*1000.*1000./144.;
            out_props->refresh_rate = 144;
            break;
        default:
            break;
    }

}

void pimax_p2c_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props){
    out_props->pixels_width = 1560;
    out_props->pixels_height = 1440;
    out_props->nominal_frame_interval_ns = 1000.*1000.*1000./90.;
    out_props->refresh_rate = 90;

    switch(debug_get_num_option_pimax_desired_mode()){
        case 1:
            out_props->nominal_frame_interval_ns = 1000.*1000.*1000./72.;
            out_props->refresh_rate = 72;
            break;
        case 2:
            // this is the one monado should already choose by default
            break;
        case 3:
            out_props->pixels_width = 1600;
            out_props->nominal_frame_interval_ns = 1000.*1000.*1000./120.;
            out_props->refresh_rate = 120;
            break;
        default:
            break;
    }

}

void pimax_p2ea_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props){
    out_props->pixels_width = 2560;
    out_props->pixels_height = 1440;
    out_props->nominal_frame_interval_ns = 1000.*1000.*1000./90.;
    out_props->refresh_rate = 90;
    switch(debug_get_num_option_pimax_desired_mode()){
        case 0:
            out_props->nominal_frame_interval_ns = 1000.*1000.*1000./72.;
            out_props->refresh_rate = 72;
            break;
        case 1:
            // this is the one monado should already choose by default
            break;
        case 2:
            out_props->pixels_width = 2160;
            out_props->nominal_frame_interval_ns = 1000.*1000.*1000./110.;
            out_props->refresh_rate = 110;
            break;
        default:
            break;
    }
}

void pimax_p2b_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props){
    out_props->pixels_width = 2560;
    out_props->pixels_height = 1440;
    out_props->nominal_frame_interval_ns = 1000.*1000.*1000./82.;
    out_props->refresh_rate = 82;
    switch(debug_get_num_option_pimax_desired_mode()){
        case 0:
            out_props->nominal_frame_interval_ns = 1000.*1000.*1000./64.;
            out_props->refresh_rate = 64;
            break;
        case 1:
            out_props->nominal_frame_interval_ns = 1000.*1000.*1000./72.;
            out_props->refresh_rate = 72;
            break;
        case 2:
            // this is the one monado should already choose by default
            break;
        default:
            break;
    }
}

void pimax_p2z_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props){
    out_props->pixels_width = 2464;
    out_props->pixels_height = 1440;
    out_props->nominal_frame_interval_ns = 1000.*1000.*1000./90.;
    out_props->refresh_rate = 90;

    switch(debug_get_num_option_pimax_desired_mode()){
        case 1:
            // this is the one monado should already choose by default
            break;
        case 2:
            out_props->pixels_width = 1700;
            out_props->nominal_frame_interval_ns = 1000.*1000.*1000./120.;
            out_props->refresh_rate = 120;
            break;
        case 3:
            out_props->pixels_width = 1600;
            out_props->nominal_frame_interval_ns = 1000.*1000.*1000./144.;
            out_props->refresh_rate = 144;
            break;
        case 4:
            out_props->pixels_width = 1436;
            out_props->nominal_frame_interval_ns = 1000.*1000.*1000./160.;
            out_props->refresh_rate = 160;
            break;
        case 5:
            out_props->pixels_width = 1328;
            out_props->nominal_frame_interval_ns = 1000.*1000.*1000./180.;
            out_props->refresh_rate = 180;
            break;
        default:
            break;
    }

}

// the results don't exactly line up with what I get from the pimax software

float pimax_8kx_lens_separation_from_raw(int16_t raw){
    //U_LOG_D("Raw: %d", raw);
    return (raw < 0 ? (((raw & 0x7fff) * -0.168f) + 4960) : raw) / 50000.f ;
}

float pimax_8kx_ipd_from_raw(uint16_t raw){
    return pimax_8kx_lens_separation_from_raw(raw) - 0.026f;
}


void pimax_8kx_read_config(struct pimax_device* dev){
    if(!dev->hid_dev){
        U_LOG_E("Pimax HID device not available");
        return;
    }
    uint8_t buf[64];
    buf[0] = 240;
    if(hid_get_feature_report(dev->hid_dev, buf, 64) == -1){
        U_LOG_E("Failed to read feature report: %ls", hid_error(dev->hid_dev));
        return;
    } 
    dev->device_config.upscaling = buf[35] & 1;
    dev->device_config.ipd = pimax_8kx_ipd_from_raw(*(uint16_t*)(&buf[36]));
    dev->device_config.separation = pimax_8kx_lens_separation_from_raw(*(uint16_t*)(&buf[36]));
    U_LOG_D("IPD Set to %f", dev->device_config.ipd);
    U_LOG_D("Separation Set to %f", dev->device_config.separation);
}

struct xrt_uv_triplet uv_triplet_lerp(struct xrt_uv_triplet from, struct xrt_uv_triplet to, float amount){
    struct xrt_uv_triplet out;
    out.r = m_vec2_lerp(from.r, to.r, amount);
    out.g = m_vec2_lerp(from.g, to.g, amount);
    out.b = m_vec2_lerp(from.b, to.b, amount);
    return out;
}

void pimax_update_fovs_from_mesh(struct pimax_device* dev){
    for(int i = 0; i < 2; i++){
        if(!dev->mesh_set){
            // there's nothing really useful that can be done here
            return;
        }
        struct xrt_fov fov_lower, fov_upper;
        float seperation = dev->device_config.ipd;
        if(i == 0){
            seperation += dev->device_config.offset_h_0.val*2;
        } else {
            seperation += dev->device_config.offset_h_1.val*2;
        }
        
        int lower_mesh_idx = -1;
        int upper_mesh_idx = -1;
        // this assumes the entries are sorted with ascending lens separation
        for(int i = dev->mesh_set->mesh_count - 1; i >= 0; i--){
            if(dev->mesh_set->meshes[i].ipd < seperation){
                upper_mesh_idx = i;
                if(i == 0){
                    if(dev->mesh_set->mesh_count > 1)
                        upper_mesh_idx = 1;
                    lower_mesh_idx = 0;
                } else {
                    lower_mesh_idx = i-1;
                }
                break;
            }
        }
        if(lower_mesh_idx == -1 || upper_mesh_idx == -1){
            lower_mesh_idx = upper_mesh_idx = dev->mesh_set->mesh_count - 1;
            if(dev->mesh_set->mesh_count > 1)
                lower_mesh_idx--;
        }    
        float range = dev->mesh_set->meshes[upper_mesh_idx].ipd - dev->mesh_set->meshes[lower_mesh_idx].ipd;
        float amount = (seperation - dev->mesh_set->meshes[upper_mesh_idx].ipd) / range;

        fov_lower = dev->mesh_set->meshes[lower_mesh_idx].views[i].fov;
        fov_upper = dev->mesh_set->meshes[upper_mesh_idx].views[i].fov;
        dev->base.base.hmd->distortion.fov[i].angle_up = math_lerp(fov_lower.angle_up, fov_upper.angle_up, amount);
        dev->base.base.hmd->distortion.fov[i].angle_down = math_lerp(fov_lower.angle_down, fov_upper.angle_down, amount);
        dev->base.base.hmd->distortion.fov[i].angle_left = math_lerp(fov_lower.angle_left, fov_upper.angle_left, amount);
        dev->base.base.hmd->distortion.fov[i].angle_right = math_lerp(fov_lower.angle_right, fov_upper.angle_right, amount);
    }
}

bool pimax_compute_distortion_from_mesh(
	    struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *out_result){

    struct pimax_device* dev = (struct pimax_device*)xdev;

    // if no mesh got loaded, do no distortion
    if(!dev->mesh_set){
        out_result->r.x = u;
        out_result->r.y = v;
        out_result->g.x = u;
        out_result->g.y = v;
        out_result->b.x = u;
        out_result->b.y = v;
        return true;
    }

    float seperation = dev->device_config.ipd;
    if(view == 0){
        seperation += dev->device_config.offset_h_0.val*2;
    } else {
        seperation += dev->device_config.offset_h_1.val*2;
    }

    // first, find the two closest meshes, could also be done in the poll function (probably better)
    int lower_mesh_idx = -1;
    int upper_mesh_idx = -1;
    // this assumes the entries are sorted with ascending lens separation
    for(int i = dev->mesh_set->mesh_count - 1; i >= 0; i--){
        //U_LOG_D("Mesh %d: %f (%f)", i, dev->mesh_set->meshes[i].ipd, seperation);
        if(dev->mesh_set->meshes[i].ipd < seperation){
            lower_mesh_idx = i;
            if(i == (int)dev->mesh_set->mesh_count-1){
                if(dev->mesh_set->mesh_count > 1)
                    upper_mesh_idx = dev->mesh_set->mesh_count-1;
                lower_mesh_idx = i-1;
            } else {
                upper_mesh_idx = i+1;
            }
            break;
        }
    }
    if(lower_mesh_idx == -1 || upper_mesh_idx == -1){
        //U_LOG_E("Could not find a distortion mesh for lens separation %fmm\n", dev->device_config.separation);
        //return false;
        lower_mesh_idx = upper_mesh_idx = 0;
        if(dev->mesh_set->mesh_count > 1)
            upper_mesh_idx = 1;
    }
    /*U_LOG_D("Using meshes with IPDs %f and %f for IPD %f", dev->mesh_set->meshes[lower_mesh_idx].ipd, 
        dev->mesh_set->meshes[upper_mesh_idx].ipd, 
        seperation);*/
    float range = dev->mesh_set->meshes[upper_mesh_idx].ipd - dev->mesh_set->meshes[lower_mesh_idx].ipd;
    struct xrt_vec2 pos = {u, v};

    int mesh_steps_u = 65;
    int mesh_steps_v = 65;
    int stride_in_floats = 8;

    int u_lower = floor(u*(mesh_steps_u-1));
    int u_upper = ceil(u*(mesh_steps_u-1));
    int v_lower = floor(v*(mesh_steps_v-1));
    int v_upper = ceil(v*(mesh_steps_v-1));

    int idx_ulvl = v_lower*mesh_steps_u + u_lower;
    int idx_uuvl = v_lower*mesh_steps_u + u_upper;
    int idx_ulvu = v_upper*mesh_steps_u + u_lower;
    int idx_uuvu = v_upper*mesh_steps_u + u_upper;
    struct xrt_uv_triplet triplets[2];
    for(int i = 0; i < 2; i++){

        int mesh_idx = i ? upper_mesh_idx : lower_mesh_idx;

        struct xrt_vec2 vec_ulvl = *(struct xrt_vec2*)(&dev->mesh_set->meshes[mesh_idx].views[view].data[idx_ulvl * stride_in_floats]);
        struct xrt_vec2 vec_uuvl = *(struct xrt_vec2*)(&dev->mesh_set->meshes[mesh_idx].views[view].data[idx_uuvl * stride_in_floats]);
        //struct xrt_vec2 vec_ulvu = *(struct xrt_vec2*)(&dev->mesh_set->meshes[mesh_idx].views[view].data[idx_ulvu * stride_in_floats]);
        struct xrt_vec2 vec_uuvu = *(struct xrt_vec2*)(&dev->mesh_set->meshes[mesh_idx].views[view].data[idx_uuvu * stride_in_floats]);

        struct xrt_uv_triplet triplet_uvl;
        struct xrt_uv_triplet triplet_uvu;
        if(u_lower == u_upper){
            triplet_uvl = *(struct xrt_uv_triplet*)(&dev->mesh_set->meshes[mesh_idx].views[view].data[idx_uuvl * stride_in_floats + 2]);
            triplet_uvu = *(struct xrt_uv_triplet*)(&dev->mesh_set->meshes[mesh_idx].views[view].data[idx_uuvu * stride_in_floats + 2]);
        } else {

            struct xrt_uv_triplet triplet_ulvl = *(struct xrt_uv_triplet*)(&dev->mesh_set->meshes[mesh_idx].views[view].data[idx_ulvl * stride_in_floats + 2]);
            struct xrt_uv_triplet triplet_uuvl = *(struct xrt_uv_triplet*)(&dev->mesh_set->meshes[mesh_idx].views[view].data[idx_uuvl * stride_in_floats + 2]);
            struct xrt_uv_triplet triplet_ulvu = *(struct xrt_uv_triplet*)(&dev->mesh_set->meshes[mesh_idx].views[view].data[idx_ulvu * stride_in_floats + 2]);
            struct xrt_uv_triplet triplet_uuvu = *(struct xrt_uv_triplet*)(&dev->mesh_set->meshes[mesh_idx].views[view].data[idx_uuvu * stride_in_floats + 2]);

            // interpolate u first
            float amount = (pos.x - vec_ulvl.x) / (vec_uuvl.x - vec_ulvl.x);
            triplet_uvl = uv_triplet_lerp(triplet_ulvl, triplet_uuvl, amount);
            triplet_uvu = uv_triplet_lerp(triplet_ulvu, triplet_uuvu, amount);
        }
        if(v_upper == v_lower){
            triplets[i] = triplet_uvl;
        } else {
            // now v
            float amount = (pos.y - vec_ulvl.y) / (vec_uuvu.y - vec_ulvl.y);
            triplets[i] = uv_triplet_lerp(triplet_uvl, triplet_uvu, amount);
        }

    }
    if(upper_mesh_idx == lower_mesh_idx){
        *out_result = triplets[0];    
    } else {
        float amount = (seperation - dev->mesh_set->meshes[upper_mesh_idx].ipd) / range;
        *out_result = uv_triplet_lerp(triplets[0], triplets[1], amount);
    }
    float dir = view ? -1.f : 1.f;
    struct pimax_display_properties props;
    dev->model_funcs->get_display_properties(dev, &props);
    out_result->r.x += (debug_get_float_option_pimax_img_offs_x_r() / (float)props.pixels_width) * dir;
    out_result->g.x += (debug_get_float_option_pimax_img_offs_x_g() / (float)props.pixels_width) * dir;
    out_result->b.x += (debug_get_float_option_pimax_img_offs_x_b() / (float)props.pixels_width) * dir;

    return true;
}

void pimax_8kx_poll(struct pimax_device* dev){
    //U_LOG_D("Pimax poll");
    bool update_distortion = dev->always_update_distortion;
    os_mutex_lock(&dev->hid_mutex);
    if(!dev->hid_dev){
        U_LOG_E("Pimax HID device not available");
        os_mutex_unlock(&dev->hid_mutex);
        return;
    }
    if(hid_send_feature_report(dev->hid_dev, pimax_poll_freq, sizeof(pimax_poll_freq)) == -1){
        U_LOG_E("Failed to send polling frequency report: %ls", hid_error(dev->hid_dev));
        os_mutex_unlock(&dev->hid_mutex);
        return; // without this report sent, no data can be polled
    }
    uint8_t buf[64];
    if(hid_read_timeout(dev->hid_dev, buf, 64, PIMAX_POLL_TIMEOUT) == -1){
        U_LOG_E("Failed to read input report: %ls", hid_error(dev->hid_dev));
        os_mutex_unlock(&dev->hid_mutex);
        return;
    }

    uint8_t report_type = buf[0];

    if(report_type == 238){
        // IPD Changed
        dev->device_config.ipd = pimax_8kx_ipd_from_raw(*(uint16_t*)(&buf[4]));
        dev->device_config.separation = pimax_8kx_lens_separation_from_raw(*(uint16_t*)(&buf[4]));
        U_LOG_D("IPD Set to %f", dev->device_config.ipd);
        U_LOG_D("Separation Set to %f", dev->device_config.separation);
        update_distortion = true;
    }
    if(update_distortion){
        if(dev->base.base.hmd->dist_update){
            if(dev->base.base.hmd->distortion.mesh.vertices){
                free(dev->base.base.hmd->distortion.mesh.vertices);
                dev->base.base.hmd->distortion.mesh.vertices = 0;
            }
            if(dev->base.base.hmd->distortion.mesh.indices){
                free(dev->base.base.hmd->distortion.mesh.indices);
                dev->base.base.hmd->distortion.mesh.indices = 0;
            }
            u_distortion_mesh_fill_in_compute(&dev->base.base);
            dev->base.base.hmd->dist_update(&dev->base.base);
            pimax_update_fovs_from_mesh(dev);
        } else {
            U_LOG_D("No distortion update function!");
        }
    }

    if(dev->polls_since_last_keepalive > PIMAX_POLL_KEEPALIVE_WAIT_COUNT){
        // send keepalive
        dev->polls_since_last_keepalive = 0;
        hid_send_feature_report(dev->hid_dev, pimax_keepalive, sizeof(pimax_keepalive));
    }

    dev->polls_since_last_keepalive++;

    os_mutex_unlock(&dev->hid_mutex);
}

void pimax_update_inputs(struct xrt_device* dev){
    //U_LOG_D("Pimax update inputs");

    // update_inputs doesn't get called reliably
    //pimax_8kx_poll((struct pimax_device*) dev);
}

void* pimax_poll_thread_func(void* ptr){
    struct pimax_device* dev = (struct pimax_device*)ptr;
    while(dev->should_poll){
        pimax_8kx_poll((struct pimax_device*)ptr);
        os_nanosleep(PIMAX_POLL_WAIT * 1000 * OS_NS_PER_USEC);
    }
    return NULL;
}

void pimax_delete_mesh_set(struct pimax_device* dev){
    for(size_t mesh = 0; mesh < dev->mesh_set->mesh_count; mesh++){
        free(dev->mesh_set->meshes[mesh].views[0].data);
        free(dev->mesh_set->meshes[mesh].views[1].data);
    }
    free(dev->mesh_set->meshes);
    free(dev->mesh_set);
    dev->mesh_set = NULL;
}

void pimax_destroy(struct xrt_device* xrtdev){
	struct pimax_device* dev = (struct pimax_device*)xrtdev;
	U_LOG_D("Pimax destroy\n");
    dev->should_poll = false;
    os_thread_join(&dev->poll_thread);
    os_mutex_lock(&dev->hid_mutex);
    hid_close(dev->hid_dev);
    os_mutex_unlock(&dev->hid_mutex);
    os_mutex_destroy(&dev->hid_mutex);

    // delete mesh
    if(dev->mesh_set){
        pimax_delete_mesh_set(dev);
    }

    free(dev);
}

size_t pimax_get_mesh_path(struct pimax_device* dev, size_t max_len, char* out_path){
    size_t outlen = 0;
    // directory
    if(getenv(PIMAX_MESHES_PATH_ENV_VAR) != NULL){
        strncpy(out_path, getenv(PIMAX_MESHES_PATH_ENV_VAR), max_len);
        outlen += strlen(getenv(PIMAX_MESHES_PATH_ENV_VAR));
        if(max_len > outlen){
            out_path[outlen] = '/';
        }
        outlen++;
    } else {
        strncpy(out_path, getenv("HOME"), max_len);
        outlen += strlen(getenv("HOME"));
        if(max_len > outlen){
            out_path[outlen] = '/';
        }
        outlen++;
        if(max_len > outlen){
            strncpy(out_path + outlen, PIMAX_MESHES_DEFAULT_PATH, (strlen(PIMAX_MESHES_DEFAULT_PATH) > (max_len - outlen)) ? 0 : (max_len - outlen));
        }
        outlen += strlen(PIMAX_MESHES_DEFAULT_PATH);
        if(max_len > outlen){
            out_path[outlen] = '/';
        }
        outlen++;
    }
    
    // file
    if(getenv(PIMAX_MESH_NAME_ENV_VAR)){
        if(max_len > outlen){
            size_t remaining = (strlen(getenv(PIMAX_MESH_NAME_ENV_VAR)) > (max_len - outlen)) ? 0 : (max_len - outlen);
            strncpy(out_path + outlen, getenv(PIMAX_MESH_NAME_ENV_VAR), remaining);
        }
        outlen += strlen(getenv(PIMAX_MESH_NAME_ENV_VAR));
    } else {
        if(max_len > outlen){
            size_t remaining = (strlen(dev->default_mesh_name) > (max_len - outlen)) ? 0 : (max_len - outlen);
            strncpy(out_path + outlen, dev->default_mesh_name, remaining);
        }
        outlen += strlen(dev->default_mesh_name);
    }

    if(max_len > outlen){
        out_path[outlen] = 0;
    }
    outlen++;
    if(max_len > 0){
        out_path[max_len-1] = 0;
    }
    return outlen;
}

void pimax_load_meshes_from_file(struct pimax_device* dev, const char* path){
    dev->mesh_set = NULL;
    char* data = u_file_read_content_from_path(path);
    if(!data){
        U_LOG_E("Error reading mesh file %s! Check if the path is correct and if you have permissions to read the file!", path);
        return;
    }
    cJSON* json = cJSON_Parse(data);
    if(!json){
        U_LOG_E("Failed to parse json from %s!", path);
        return;
    }

    struct pimax_display_properties display_props;
    dev->model_funcs->get_display_properties(dev, &display_props);
    if(!cJSON_IsArray(json)){
        U_LOG_E("Mesh json has wrong structure!");
        return;
    }
    cJSON* currentMeshSet;
    bool mesh_matches = false;
    cJSON_ArrayForEach(currentMeshSet,json){
        // check resolutions first
        cJSON* currentResolution;
        bool res_match = false;
        bool refrate_match = false;
        cJSON_ArrayForEach(currentResolution, cJSON_GetObjectItem(currentMeshSet, "resolutions")){
            uint32_t width = cJSON_GetNumberValue(cJSON_GetObjectItem(currentResolution, "width"));
            uint32_t height = cJSON_GetNumberValue(cJSON_GetObjectItem(currentResolution, "height"));
            if(width == display_props.pixels_width && height == display_props.pixels_height){
                res_match = true;
                break;
            }
        }
        // also check the refresh rates at some point
        cJSON_ArrayForEach(currentResolution, cJSON_GetObjectItem(currentMeshSet, "refresh_rates")){
            uint32_t refrate = cJSON_GetNumberValue(currentResolution);
            if(refrate == display_props.refresh_rate){
                refrate_match = true;
                break;
            }
        }

        if(res_match && refrate_match){
            mesh_matches = true;
            break;
        }
    }
    if(!mesh_matches){
        U_LOG_E("Could not find a matching mesh for the current display mode!");
        return;
    }
    U_LOG_D("Found mesh!");

    // now create the mesh structures in the pimax_device
    cJSON* meshes = cJSON_GetObjectItem(currentMeshSet, "meshes");
    int mesh_count = cJSON_GetArraySize(meshes);
    dev->mesh_set = U_TYPED_CALLOC(struct pimax_mesh_set);
    dev->mesh_set->mesh_count = mesh_count;
    dev->mesh_set->meshes = U_TYPED_ARRAY_CALLOC(struct pimax_mesh, mesh_count);

    cJSON* currentMesh;
    size_t meshIdx = 0;
    cJSON_ArrayForEach(currentMesh, meshes){
        struct pimax_mesh* currentPMesh = dev->mesh_set->meshes + meshIdx;
        currentPMesh->ipd = atof(currentMesh->string);
        for(size_t view = 0; view < 2; view++){
            cJSON* view_json = cJSON_GetArrayItem(currentMesh, view);
            currentPMesh->views[view].fov = (struct xrt_fov){
                cJSON_GetNumberValue(cJSON_GetArrayItem(cJSON_GetObjectItem(view_json, "fov"), 0)),
                cJSON_GetNumberValue(cJSON_GetArrayItem(cJSON_GetObjectItem(view_json, "fov"), 1)),
                cJSON_GetNumberValue(cJSON_GetArrayItem(cJSON_GetObjectItem(view_json, "fov"), 2)),
                cJSON_GetNumberValue(cJSON_GetArrayItem(cJSON_GetObjectItem(view_json, "fov"), 3)),
            };
            cJSON* points = cJSON_GetObjectItem(view_json, "points");
            currentPMesh->views[view].len = cJSON_GetArraySize(points);
            currentPMesh->views[view].data = U_TYPED_ARRAY_CALLOC(float, currentPMesh->views[view].len);
            size_t idx = 0;
            cJSON* pval;
            cJSON_ArrayForEach(pval, points){
                currentPMesh->views[view].data[idx] = cJSON_GetNumberValue(pval);
                idx++;
            }
        }
        meshIdx++;
    }
}

long init_pimax8kx(struct fixup_context* ctx, struct fixup_func_list* funcs, struct hid_device_info* devinfo){
    if(devinfo->interface_number) return 0;

	if(ctx->num_devices >= FIXUP_MAX_DEVICES){
		U_LOG_E("Too many devices in fixup_devices");
		return 0;
	}
	struct pimax_device* dev = U_TYPED_CALLOC(struct pimax_device);
	dev->base.fixup_funcs = funcs;
    os_mutex_init(&dev->hid_mutex);

    U_LOG_D("Pimax 8KX init\n");
    os_mutex_lock(&dev->hid_mutex);
	dev->hid_dev = NULL;
	dev->hid_dev = hid_open(PIMAX_VID, PIMAX_8KX_PID, NULL);
	if(!dev->hid_dev){
        os_mutex_unlock(&dev->hid_mutex);
        os_mutex_destroy(&dev->hid_mutex);
		free(dev);
		U_LOG_E("Failed to open Pimax 8KX HID device");
		return 0;
	}

    wchar_t buf[PIMAX_MODEL_NAME_LENGTH];
    if(hid_get_product_string(dev->hid_dev, buf, ARRAY_SIZE(buf)) == -1){
        U_LOG_E("Failed to retrieve HID model string!");
        os_mutex_unlock(&dev->hid_mutex);
        os_mutex_destroy(&dev->hid_mutex);
        hid_close(dev->hid_dev);
		free(dev);
        return 0;
    }
    bool found_match = false;
    // retrieve the model specific functions
    for(size_t i = 0; i < ARRAY_SIZE(model_configs); i++){
        if(wcsncmp(buf, model_configs[i].product_name, PIMAX_MODEL_NAME_LENGTH) == 0){
            dev->model_funcs = &model_configs[i].funcs;
            strncpy(dev->base.base.str, model_configs[i].display_name, 
                (PIMAX_MODEL_NAME_LENGTH<XRT_DEVICE_NAME_LEN?PIMAX_MODEL_NAME_LENGTH:XRT_DEVICE_NAME_LEN)-1);
                found_match = true;
            dev->default_mesh_name = model_configs[i].default_mesh_name;
            break;
        }
    }
    if(!found_match){
        U_LOG_W("Could not find a matching model specific configuration for \"%ls\". Falling back to the first one in the list (please report this)", buf);
    }


    hid_send_feature_report(dev->hid_dev, pimax_init2, sizeof(pimax_init2));
	hid_send_feature_report(dev->hid_dev, pimax_packet_parallel_projections_off, sizeof(pimax_packet_parallel_projections_off));
	hid_send_feature_report(dev->hid_dev, pimax_hmd_power, sizeof(pimax_hmd_power));
    hid_send_feature_report(dev->hid_dev, pimax_keepalive, sizeof(pimax_keepalive));
    pimax_8kx_read_config(dev);
    os_mutex_unlock(&dev->hid_mutex);


	struct xrt_device* xrtdev = &dev->base.base;
	xrtdev->destroy = pimax_destroy;
	xrtdev->device_type = XRT_DEVICE_TYPE_HMD;
	xrtdev->name = XRT_DEVICE_GENERIC_HMD;
	xrtdev->update_inputs = u_device_noop_update_inputs;
	xrtdev->hmd = U_TYPED_CALLOC(struct xrt_hmd_parts);
	xrtdev->hmd->view_count = 2;
	xrtdev->hmd->distortion.models =  XRT_DISTORTION_MODEL_COMPUTE;
	xrtdev->hmd->distortion.preferred =  XRT_DISTORTION_MODEL_COMPUTE;
    xrtdev->compute_distortion = pimax_compute_distortion_from_mesh;

    xrtdev->hmd->distortion.fov[0] = (struct xrt_fov){-1.224257374, 0.931880974, 0.9037, -0.9037};
    xrtdev->hmd->distortion.fov[1] = (struct xrt_fov){-0.931880974, 1.224257374, 0.9037, -0.9037};

	xrtdev->get_view_poses = pimax_get_view_poses;
	xrtdev->hmd->blend_modes[0] = XRT_BLEND_MODE_OPAQUE;
	xrtdev->hmd->blend_mode_count = 1;
    xrtdev->stage_supported = false;

    /*
     * the hmd doesn't actually have any inputs, but the IPC client doesn't like that
     */
    xrtdev->inputs = U_TYPED_CALLOC(struct xrt_input);
    xrtdev->inputs[0].name = XRT_INPUT_SIMPLE_SELECT_CLICK;
    xrtdev->inputs[0].active = true;
    xrtdev->input_count = 1;

	ctx->devices[ctx->num_devices] = (struct fixup_device*)dev;
	ctx->num_devices++;


    /*
     * Distortion mesh stuff
     */

    size_t mesh_path_len = pimax_get_mesh_path(dev, 0, NULL);
    char* meshpath = U_TYPED_ARRAY_CALLOC(char, mesh_path_len);
    pimax_get_mesh_path(dev, mesh_path_len, meshpath);
    U_LOG_I("Using distortion meshes from: %s", meshpath);
    pimax_load_meshes_from_file(dev, meshpath);
    free(meshpath);

    pimax_update_fovs_from_mesh(dev);

    /*
     * Start the polling thread. Currently, this only handles IPD adjustments
     */
    dev->should_poll = true;
    dev->polls_since_last_keepalive = 0;
    os_thread_init(&dev->poll_thread);
    os_thread_start(&dev->poll_thread, pimax_poll_thread_func, dev);


    dev->device_config.offset_h_0.min = OFFSETS_MIN;
    dev->device_config.offset_h_0.max = OFFSETS_MAX;
    dev->device_config.offset_h_0.step = OFFSETS_STEP;
    dev->device_config.offset_h_0.val = debug_get_float_option_pimax_ipd_offs_h0();

    dev->device_config.offset_h_1.min = OFFSETS_MIN;
    dev->device_config.offset_h_1.max = OFFSETS_MAX;
    dev->device_config.offset_h_1.step = OFFSETS_STEP;
    dev->device_config.offset_h_1.val = debug_get_float_option_pimax_ipd_offs_h1();

    dev->device_config.offset_v_0.min = OFFSETS_MIN;
    dev->device_config.offset_v_0.max = OFFSETS_MAX;
    dev->device_config.offset_v_0.step = OFFSETS_STEP;
    dev->device_config.offset_v_0.val = debug_get_float_option_pimax_ipd_offs_v0();

    dev->device_config.offset_v_1.min = OFFSETS_MIN;
    dev->device_config.offset_v_1.max = OFFSETS_MAX;
    dev->device_config.offset_v_1.step = OFFSETS_STEP;
    dev->device_config.offset_v_1.val = debug_get_float_option_pimax_ipd_offs_v1();

    return 3000;
}


// mostly copied from the vive driver
static void
pimax_get_view_poses(struct xrt_device *xdev,
                           const struct xrt_vec3 *default_eye_relation,
                           int64_t at_timestamp_ns,
                           uint32_t view_count,
                           struct xrt_space_relation *out_head_relation,
                           struct xrt_fov *out_fovs,
                           struct xrt_pose *out_poses)
{
    struct pimax_device* dev = (struct pimax_device*)xdev; 

    struct  xrt_vec3 newEyeRelation = *default_eye_relation;
    newEyeRelation.x = dev->device_config.ipd;

    for (uint32_t i = 0; i < view_count && i < ARRAY_SIZE(xdev->hmd->views); i++) {
		out_fovs[i] = xdev->hmd->distortion.fov[i];
	}

    for(uint32_t view_index = 0; view_index < view_count; view_index++){
        struct xrt_pose pose = XRT_POSE_IDENTITY;
	    bool adjust = view_index == 0;

        float v_adjust = 0.0;
        float h_adjust = 0.0;
        if(view_index == 0){
            h_adjust = dev->device_config.offset_h_0.val;
            v_adjust = dev->device_config.offset_v_0.val;
        } else {
            h_adjust = dev->device_config.offset_h_1.val;
            v_adjust = dev->device_config.offset_v_1.val;
        }

	    pose.position.x = (newEyeRelation.x / 2.0f) + h_adjust;
	    pose.position.y = (newEyeRelation.y / 2.0f) + v_adjust;
	    pose.position.z = newEyeRelation.z / 2.0f;

	    // Adjust for left/right while also making sure there aren't any -0.f.
	    if (pose.position.x != 0.0f && adjust) {
	    	pose.position.x = -pose.position.x;
	    }
	    if (pose.position.y != 0.0f && adjust) {
	    	pose.position.y = -pose.position.y;
	    }
	    if (pose.position.z != 0.0f && adjust) {
	    	pose.position.z = -pose.position.z;
	    }

	    out_poses[view_index] = pose;
    }

    //U_LOG_D("get view poses\n");
    // canted displays, based on PiTools output 
    // ONLY WORKS IF LIGHTHOUSES ARE ON!!!
    
    out_poses[0].orientation = (struct xrt_quat){0,0.0871557, 0, 0.996195};
    out_poses[1].orientation = (struct xrt_quat){0,-0.0871557, 0, 0.996195};
}

// width and height here refer to how the displays are located in the hmd
void pimax_fill_display(struct pimax_device* dev,struct pimax_display_properties* display_props){
    struct xrt_hmd_parts* hmd;
    hmd = dev->base.base.hmd;
    hmd->screens[0].w_pixels = 2*display_props->pixels_height;
    hmd->screens[0].h_pixels = display_props->pixels_width;
    hmd->screens[0].nominal_frame_interval_ns = display_props->nominal_frame_interval_ns;
    for(int i = 0; i < 2; i++){
        hmd->views[i].viewport.w_pixels = display_props->pixels_height;
        hmd->views[i].viewport.h_pixels = display_props->pixels_width;
        hmd->views[i].viewport.x_pixels = i * display_props->pixels_height;
        hmd->views[i].viewport.y_pixels = 0;
        hmd->views[i].display.w_pixels = display_props->pixels_width;
        hmd->views[i].display.h_pixels = display_props->pixels_height;
        hmd->views[i].rot.v[0] = 0;
        hmd->views[i].rot.v[1] = i ? 1 : -1;
        hmd->views[i].rot.v[2] = i ? -1 : 1;
        hmd->views[i].rot.v[3] = 0;
    }
}

void patch_pimax8kx(struct fixup_device* fdev, struct fixup_context* ctx, struct xrt_system_devices *xsysd){
    //if(devinfo->interface_number) return;
    U_LOG_D("Pimax 8KX patch\n");
    if(!xsysd->static_roles.head){
        U_LOG_W("Pimax 8KX HID device detected, but no HMD available\n");
        return;
    }
	struct pimax_device* dev = (struct pimax_device*) fdev;
    os_mutex_lock(&dev->hid_mutex);
    pimax_8kx_read_config(dev);
    os_mutex_unlock(&dev->hid_mutex);

    struct pimax_display_properties display_props;
    dev->model_funcs->get_display_properties(dev, &display_props);
    pimax_fill_display(dev, &display_props);
    //pimax_8kx_set_display_info(dev);

    // probably do some more checks to make sure this is actually the right HMD

	struct xrt_pose ident = XRT_POSE_IDENTITY;

	struct xrt_device* multidev = multi_create_tracking_override(XRT_TRACKING_OVERRIDE_DIRECT,
		&dev->base.base, xsysd->static_roles.head, XRT_INPUT_GENERIC_HEAD_POSE, &ident);
    xsysd->static_roles.head->device_type = XRT_DEVICE_TYPE_UNKNOWN;    // make u_device_assign_xdev_roles work correctly
	xsysd->static_roles.head = multidev;
    xsysd->xdevs[xsysd->xdev_count++] = multidev;


    // setup debug GUI
    u_var_add_root(dev, "Pimax HMD", true);
    u_var_add_ro_f32(dev, &dev->device_config.ipd, "Current IPD");
    u_var_add_bool(dev, &dev->always_update_distortion, "Constantly update distortion mesh (performance heavy)");
    u_var_add_gui_header(dev, NULL, "Horizontal offset: ");
    u_var_add_draggable_f32(dev, &dev->device_config.offset_h_0, "Left H");
    u_var_add_draggable_f32(dev, &dev->device_config.offset_h_1, "Right H");
    
    u_var_add_gui_header(dev, NULL, "Vertical offset: ");
    u_var_add_draggable_f32(dev, &dev->device_config.offset_v_0, "Left V");
    u_var_add_draggable_f32(dev, &dev->device_config.offset_v_1, "Right V");
    
}

#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_time.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"

#include "../../drivers/multi_wrapper/multi.h"

#include "math/m_vec2.h"
#include "math/m_api.h"

#include "pimax.h"
#include "pimax_projection.h"


// forward declarations, these aren't needed anywhere else, so no need to put them into the header
void pimax_8kx_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props);
void pimax_5ks_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props);


struct pimax_model_config model_configs[] = {
    {L"Pimax P2EA", "Pimax 8K Plus", {pimax_5ks_get_display_props}},
    {L"Pimax P2A", "Pimax 5K Super", {pimax_5ks_get_display_props}},
    {L"Pimax P2C", "Pimax 5K Super", {pimax_5ks_get_display_props}},
    {L"Pimax P2N", "Pimax 8KX", {pimax_8kx_get_display_props}}
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
                           uint64_t at_timestamp_ns,
                           uint32_t view_count,
                           struct xrt_space_relation *out_head_relation,
                           struct xrt_fov *out_fovs,
                           struct xrt_pose *out_poses);

void pimax_8kx_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props){
    out_props->pixels_width = dev->device_config.upscaling ? 2160 : 3168;
    out_props->pixels_height = dev->device_config.upscaling ? 1440 : 2160;

    out_props->size_in_meters.x = dev->device_config.upscaling ? 0.10206f : 0.099792f;
    out_props->size_in_meters.y = 0.13608f/2.f;
    out_props->nominal_frame_interval_ns = 1000.*1000.*1000./(dev->device_config.upscaling ? 110. : 90.);
}

void pimax_5ks_get_display_props(struct pimax_device* dev, struct pimax_display_properties* out_props){
    out_props->pixels_width = 2160;
    out_props->pixels_height = 1440;

    out_props->size_in_meters.x = 0.10206f;
    out_props->size_in_meters.y = 0.13608f/2.f;
    out_props->nominal_frame_interval_ns = 1000.*1000.*1000./110.;
}


// the results don't exactly line up with what I get from the pimax software
#define PIMAX_IPD_RAW_MAX 36648
#define PIMAX_IPD_RAW_MIN 34557
#define PIMAX_IPD_MIN 0.06043485552072525
#define PIMAX_IPD_MAX 0.0670139342546463
#define PIMAX_SEPARATION_MIN 0.08623039722442627
#define PIMAX_SEPARATION_MAX 0.09301088005304337

float pimax_8kx_ipd_from_raw(uint16_t raw){
    return PIMAX_IPD_MAX - (raw - PIMAX_IPD_RAW_MIN) * (PIMAX_IPD_MAX-PIMAX_IPD_MIN)/(PIMAX_IPD_RAW_MAX-PIMAX_IPD_RAW_MIN);
}

float pimax_8kx_lens_separation_from_raw(uint16_t raw){
    return PIMAX_SEPARATION_MAX - (raw - PIMAX_IPD_RAW_MIN) * (PIMAX_SEPARATION_MAX-PIMAX_SEPARATION_MIN)/(PIMAX_IPD_RAW_MAX-PIMAX_IPD_RAW_MIN);
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
}


void pimax_8kx_poll(struct pimax_device* dev){
    //U_LOG_D("Pimax poll");
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

void pimax_destroy(struct xrt_device* xrtdev){
	struct pimax_device* dev = (struct pimax_device*)xrtdev;
	U_LOG_D("Pimax destroy\n");
    dev->should_poll = false;
    os_thread_join(&dev->poll_thread);
    os_mutex_lock(&dev->hid_mutex);
    hid_close(dev->hid_dev);
    os_mutex_unlock(&dev->hid_mutex);
    os_mutex_destroy(&dev->hid_mutex);
    free(dev);
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
    os_mutex_unlock(&dev->hid_mutex);


	struct xrt_device* xrtdev = &dev->base.base;
	xrtdev->destroy = pimax_destroy;
	xrtdev->device_type = XRT_DEVICE_TYPE_HMD;
	xrtdev->name = XRT_DEVICE_GENERIC_HMD;
	//strncpy(xrtdev->str, "Pimax 8KX", XRT_DEVICE_NAME_LEN);
	xrtdev->update_inputs = pimax_update_inputs;
	xrtdev->hmd = U_TYPED_CALLOC(struct xrt_hmd_parts);
	xrtdev->hmd->view_count = 2;
	xrtdev->hmd->distortion.models = XRT_DISTORTION_MODEL_COMPUTE;
	xrtdev->hmd->distortion.preferred = XRT_DISTORTION_MODEL_COMPUTE;
	xrtdev->compute_distortion = pimax_compute_distortion2;
    // pure guesses, likely wrong
    /*xrtdev->hmd->distortion.fov[0] = (struct xrt_fov){-1., 1., 0.8927, -0.8927};
    xrtdev->hmd->distortion.fov[1] = (struct xrt_fov){-1., 1., 0.8927, -0.8927};*/
    xrtdev->hmd->distortion.fov[0] = (struct xrt_fov){-1.0611, 1.0611, 0.9037, -0.9037};
    xrtdev->hmd->distortion.fov[1] = (struct xrt_fov){-1.0611, 1.0611, 0.9037, -0.9037};

	xrtdev->get_view_poses = pimax_get_view_poses;
	xrtdev->hmd->blend_modes[0] = XRT_BLEND_MODE_OPAQUE;
	xrtdev->hmd->blend_mode_count = 1;	// need to check this again
    xrtdev->stage_supported = true;

    /*
     * the hmd doesn't actually have any inputs, but the IPC client doesn't like that
     */
    xrtdev->inputs = U_TYPED_CALLOC(struct xrt_input);
    xrtdev->inputs[0].name = XRT_INPUT_SIMPLE_SELECT_CLICK;
    xrtdev->input_count = 1;

	ctx->devices[ctx->num_devices] = (struct fixup_device*)dev;
	ctx->num_devices++;

    /*
     * Start the polling thread. Currently, this only handles IPD adjustments
     */
    dev->should_poll = true;
    dev->polls_since_last_keepalive = 0;
    os_thread_init(&dev->poll_thread);
    os_thread_start(&dev->poll_thread, pimax_poll_thread_func, dev);

    return 3000;
}


// mostly copied from the vive driver
static void
pimax_get_view_poses(struct xrt_device *xdev,
                           const struct xrt_vec3 *default_eye_relation,
                           uint64_t at_timestamp_ns,
                           uint32_t view_count,
                           struct xrt_space_relation *out_head_relation,
                           struct xrt_fov *out_fovs,
                           struct xrt_pose *out_poses)
{
    struct pimax_device* dev = (struct pimax_device*)xdev; 

    struct  xrt_vec3 newEyeRelation = *default_eye_relation;
    newEyeRelation.x = dev->device_config.separation;

    for (uint32_t i = 0; i < view_count && i < ARRAY_SIZE(xdev->hmd->views); i++) {
		out_fovs[i] = xdev->hmd->distortion.fov[i];
	}

    for(uint32_t view_index = 0; view_index < view_count; view_index++){
        struct xrt_pose pose = XRT_POSE_IDENTITY;
	    bool adjust = view_index == 0;

	    pose.position.x = newEyeRelation.x / 2.0f;
	    pose.position.y = newEyeRelation.y / 2.0f;
	    pose.position.z = newEyeRelation.z / 2.0f;

	    // Adjust for left/right while also making sure there aren't any -0.f.
	    if (pose.position.x > 0.0f && adjust) {
	    	pose.position.x = -pose.position.x;
	    }
	    if (pose.position.y > 0.0f && adjust) {
	    	pose.position.y = -pose.position.y;
	    }
	    if (pose.position.z > 0.0f && adjust) {
	    	pose.position.z = -pose.position.z;
	    }

	    out_poses[view_index] = pose;
    }

    //U_LOG_D("get view poses\n");
    // canted displays, based on hmdgdb
    // ONLY WORKS IF LIGHTHOUSES ARE ON!!!
    
    out_poses[0].orientation = (struct xrt_quat){0,0.173648, 0, 0.984808};
    out_poses[1].orientation = (struct xrt_quat){0,-0.173648, 0, 0.984808};
}


// probably best to ignore this, it doesn't work very well at all
bool
u_compute_distortion_ndvive(struct u_vive_values *values, float u, float v, float uoffs, float p1, float p2, struct xrt_uv_triplet *result)
{
	// Reading the whole struct like this gives the compiler more opportunity to optimize.
	const struct u_vive_values val = *values;

	const float common_factor_value = 0.5f / (1.0f + val.grow_for_undistort);
	const struct xrt_vec2 factor = {
	    common_factor_value,
	    common_factor_value * val.aspect_x_over_y,
	};

	// Results r/g/b.
	struct xrt_vec2 tc[3] = {{0, 0}, {0, 0}, {0, 0}};

	// Dear compiler, please vectorize.
	for (int i = 0; i < 3; i++) {
		struct xrt_vec2 texCoord = {
		    2.f * u - 1.f,
		    2.f * v - 1.f,
		};

		texCoord.y /= val.aspect_x_over_y;
		texCoord.x -= val.center[i].x;
		texCoord.y -= val.center[i].y;


        struct xrt_vec2 distTexCoord = {
		    2.f * (u+uoffs) - 1.f,
		    2.f * v - 1.f,
		};

		distTexCoord.y /= val.aspect_x_over_y;
		distTexCoord.x -= val.center[i].x;
		distTexCoord.y -= val.center[i].y;

		float r2 = m_vec2_dot(distTexCoord, distTexCoord);
		float k1 = val.coefficients[i][0];
		float k2 = val.coefficients[i][1];
		float k3 = val.coefficients[i][2];
		float k4 = val.coefficients[i][3];

		/*
		 *                     1.0
		 * d = -------------------------------------- + k4
		 *      1.0 + r^2 * k1 + r^4 * k2 + r^6 * k3
		 *
		 * The variable k4 is the scaled part of DISTORT_DPOLY3_SCALED.
		 *
		 * Optimization to reduce the number of multiplications.
		 *    1.0 + r^2 * k1 + r^4 * k2 + r^6 * k3
		 *    1.0 + r^2 * ((k1 + r^2 * k2) + r^2 * k3)
		 */

		float top = 1.f;
		float bottom = 1.f + r2 * (k1 + r2 * (k2 + r2 * k3));
		float d = (bottom) + k4;

        // tangential distortion
        float x = texCoord.x;
        float y = texCoord.y;
        float xn = x;// + (2*p1*x*y + p2*(r2 + 2 * x*x));
        float yn = y;// + (p1*(r2 + 2 * y*y) + 2*p2*x*y);
		struct xrt_vec2 offset = {0.5f, 0.5f};

		tc[i].x = offset.x + (xn * d + val.center[i].x) * factor.x * 1.2;
		tc[i].y = offset.y + (yn * d + val.center[i].y) * factor.y * 1.3;
	}

	result->r = tc[0];
	result->g = tc[1];
	result->b = tc[2];

	return true;
}

bool pimax_compute_distortion(
	    struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *out_result){

    //U_LOG_D("Distortion %f:%f is now %f:%f\n", u, v, out_result->r.x, out_result->r.y);
    // correct for the canted displays
    float xdir = view ? -1.f : 1.f;
    u -= 0.5f;
    v -= 0.5f;
    v *= 1 + ((view ? -u : u) * (0.173228346f));
    v += 0.5f;
    u *= (608.f/508.f);
    u += 0.5f;
    u -= xdir* 0.07f;  // adjust for the lenses moving over the displays. this value works decently for 0.067m IPD
    //u*= 0.5;
    //v*= 0.5;

    // no distortion for now
    out_result->r.x = u;
    out_result->r.y = v;
    out_result->g.x = u;
    out_result->g.y = v;
    out_result->b.x = u;
    out_result->b.y = v;
    
    struct u_vive_values pimaxLeft = {1.65, 0.6, 1.5,
        //{(struct xrt_vec2){xdir*0.2,0}, (struct xrt_vec2){xdir*0.2,0}, (struct xrt_vec2){xdir*0.2,0}},
        {(struct xrt_vec2){xdir*0.,0}, (struct xrt_vec2){xdir*0.,0}, (struct xrt_vec2){xdir*0.,0}},
        //{{0.74398162, -0.20681542, 0.89263725, 0.0}, {0.74398162, -0.20681542, 0.89263725, 0.0}, {0.74398162, -0.20681542, 0.89263725, 0.0}}};
        //{{0.20168104, -0.00836374, 0.08422303, 0.0}, {0.20168104, -0.00836374, 0.08422303, 0.0}, {0.20168104, -0.00836374, 0.08422303, 0.0}}};
        {{0.60168104, -0.00836374, 0.08422303, 0.0}, {0.60168104, -0.00836374, 0.08422303, 0.0}, {0.60168104, -0.00836374, 0.08422303, 0.0}}};
        //{{0.00784313772, 0.00392156886, 0, 0.0}, {0.00784313772, 0.00392156886, 0, 0.0}, {0.00784313772, 0.00392156886, 0, 0.0}}};
    u_compute_distortion_ndvive(&pimaxLeft, u, v, 0.0, -0.02637853, 0.03973991, out_result);
    //U_LOG_D("Distortion %f:%f is now %f:%f\n", u, v, out_result->r.x, out_result->r.y);
    return true;
}



// width and height here refer to how the displays are located in the hmd
void pimax_fill_display(struct pimax_device* dev,struct pimax_display_properties* display_props){
    struct xrt_hmd_parts* hmd;
    hmd = dev->base.base.hmd;
    hmd->screens[0].w_pixels = 2*display_props->pixels_height;
    hmd->screens[0].h_pixels = display_props->pixels_width;
    hmd->screens[0].nominal_frame_interval_ns = display_props->nominal_frame_interval_ns;   // TODO: use the actual refresh rate
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
	xsysd->static_roles.head = multidev;
    xsysd->xdevs[xsysd->xdev_count++] = multidev;

}
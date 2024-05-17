#include "pimax_projection.h"
#include "pimax.h"


#include "util/u_debug.h"
#include "util/u_distortion_mesh.h"
#include "math/m_api.h"


// these are from pitool
float pimax_8kx_spline_points[] = {1.0000,1.0484,1.1042,1.1800,1.2650,1.3450,1.4200,1.4700,1.4900,1.5000,1.5100,1.5100,1.5200,1.5400,1.5670,1.5900,1.6100,1.6260};
float pimax_8kx_chromatic_aberration[] = {-0.005,-0.007,0.008,0.023};

bool _pimax_compute_distortion(struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *out_result){

    return true;
}


float catmullRom_interpolate(int count, float* points, float pos){
    int p0 = pos;
    float p[4];
    if(p0 > count-1){
        p0 = count-1;
    }
    p0 = MAX(0, p0);

    float delta[2];
    delta[0] = pos - p0;
    delta[1] = 1.f - delta[0];
    if(p0 == 0){
        p[0] = points[0];
        p[1] = points[1];
        p[2] = (points[2] - p[0]) * 0.5f;
        p[3] = p[1] - p[0];
    } else if(p0 == count-2){
        p[0] = points[count-2];
        p[1] = points[count-1];
        p[2] = p[1]-p[0];
        p[3] = p[2] * 0.5f;
    } else if(p0 == count-1){
        p[0] = points[count-1];
        p[3] = p[0] - points[count-2];
        p[1] = p[3]+p[0];
        p[2] = p[3];
    } else {
        p[0] = points[p0];
        p[1] = points[p0+1];
        p[2] = (points[p0+2] - p[0]) * 0.5f;
        p[3] = (p[1]-points[p0-1]) * 0.5f;
    }

    return ((2*delta[1] + 1.f) * p[1] - delta[1] * p[2]) * delta[0] * delta[0] +
           ((2*delta[0] + 1.f) * p[0] + delta[0] * p[3]) * delta[1] * delta[1];

}

void pimax_distort_spline(float u, float v, struct xrt_vec2 center, struct xrt_uv_triplet *result){

    const float common_factor_value = 0.5f / (1.0f + 0.3);
	const struct xrt_vec2 factor = {
	    common_factor_value,
	    common_factor_value * 1.5f,
	};

    struct xrt_vec2 texCoord = {
	    2.f * u - 1.f,
	    2.f * v - 1.f,
	};
    // no center/aspect ratio adjustment for now (will be needed though)

    texCoord.y /= 1.5f;
    texCoord.x -= center.x;
    texCoord.y -= center.y;

    struct xrt_vec2 offset = m_vec2_div_scalar(m_vec2_add_scalar(center, 1.f), 2.f);

    float max_radius = 1.4757009417836655;

    float r = m_vec2_len(texCoord);
    float r2 = m_vec2_dot(texCoord, texCoord);
    //float fact = interpolate_spline(pimax_8kx_spline_points, ARRAY_SIZE(pimax_8kx_spline_points), r);
    float fact = catmullRom_interpolate(18, pimax_8kx_spline_points, r2*17./(max_radius*max_radius)/*add divisor here*/);
    //U_LOG_D("Factor(r=%f)=%f\n", r2, fact);

    struct xrt_vec2 tc[3] = {{0, 0}, {0, 0}, {0, 0}};
    struct xrt_vec2 chroma[3] = {{pimax_8kx_chromatic_aberration[0], pimax_8kx_chromatic_aberration[1]}, {0.,0.}, {pimax_8kx_chromatic_aberration[2], pimax_8kx_chromatic_aberration[3]}};
    for(int i = 0; i < 3; i++){
        
        float x = texCoord.x;
        float y = texCoord.y;

        float chromaFact = 1.f + chroma[i].x + r2 * chroma[i].y;

        tc[i].x = offset.x + (x*fact) * factor.x * chromaFact;
        tc[i].y = offset.y + (y*fact) * factor.y * chromaFact;
    }
    //U_LOG_D("%f:%f -> %f:%f",u,v,tc[0].x, tc[0].y);
    result->r = tc[0];
    result->g = tc[1];
    result->b = tc[2];
}




extern "C"{
bool pimax_compute_distortion2(
	    struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *out_result){
    
    float xdir = view ? -1.f : 1.f;


    struct pimax_device* dev = (struct pimax_device*)xdev; 
    struct pimax_display_properties props;
    dev->model_funcs->get_display_properties(dev, &props);

    struct xrt_vec2 sizeInMeters = props.size_in_meters;
    float gap = 0.0144;


    float angle = -10.f / (180.f/M_PI) * xdir;

    float width = sizeInMeters.x * abs(cos(angle));
    float centerOffset = (dev->device_config.separation - dev->device_config.ipd)/2;    // probably not exactly, but close
    float screenCenterOffset = (centerOffset / cos(angle)) / sizeInMeters.x;
    //U_LOG_D("Sep: %f IPD: %f width: %f Offset: %f",dev->device_config.separation, dev->device_config.ipd, width,  screenCenterOffset);
    xrt_vec2 center = {screenCenterOffset*xdir, 0.f};

    float ucenter = (center.x+1.f)/2.f;
    float vcenter = (center.y+1.f)/2.f;

    float urealworld = (u-ucenter) * sizeInMeters.x - xdir*gap/2;
    float vrealworld = (v-vcenter) * sizeInMeters.y;
    
    //float d = 0.05; // distance from eye to display (when looking straight), measurement/guess
    float d = 0.071746735;  // also likely wrong
    float common_d = (d+sin(angle)*urealworld);
    float u2 = (((cos(angle)*urealworld)/common_d*d) / sizeInMeters.x) + ucenter;
    float v2 = ((vrealworld/common_d)*d / sizeInMeters.y) + vcenter;
    //U_LOG_D("U: %f U2: %f V: %f V2: %f", u, u2, v, v2);

    pimax_distort_spline(u2,v2,center, out_result);

    return true;
}
}
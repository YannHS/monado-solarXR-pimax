#pragma once
#include "math/m_vec2.h"
#include "math/m_api.h"

#ifdef __cplusplus
extern "C"{
#endif

bool pimax_compute_distortion2(
	    struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *out_result);

#ifdef __cplusplus
}
#endif
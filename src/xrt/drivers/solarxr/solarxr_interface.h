// Copyright 2025, rcelyte
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  SolarXR protocol bridge device
 * @ingroup drv_solarxr
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
struct xrt_device;
struct xrt_tracking_origin;

#ifdef __cplusplus
extern "C" {
#endif

uint32_t
solarxr_device_create_xdevs(struct xrt_tracking_origin *tracking_origin,
                            struct xrt_device *out_xdevs[],
                            uint32_t out_xdevs_cap);

static inline struct xrt_device *
solarxr_device_create(struct xrt_tracking_origin *const tracking_origin)
{
	struct xrt_device *out = NULL;
	solarxr_device_create_xdevs(tracking_origin, &out, 1);
	return out;
}

bool
solarxr_device_set_feeder_devices(struct xrt_device *solarxr, struct xrt_device *const xdevs[], uint32_t xdevs_len);


#ifdef __cplusplus
}
#endif

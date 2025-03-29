// Copyright 2022-2023, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Qwerty devices builder.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup drv_qwerty
 */

#include "xrt/xrt_config_drivers.h"

#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_builders.h"
#include "util/u_system_helpers.h"

#include "target_builder_interface.h"

#include "qwerty/qwerty_interface.h"

#ifdef XRT_BUILD_DRIVER_SOLARXR
#include "solarxr/solarxr_interface.h"
#endif

#include <assert.h>


#ifndef XRT_BUILD_DRIVER_QWERTY
#error "Must only be built with XRT_BUILD_DRIVER_QWERTY set"
#endif

// Using INFO as default to inform events real devices could report physically
DEBUG_GET_ONCE_LOG_OPTION(qwerty_log, "QWERTY_LOG", U_LOGGING_INFO)

// Driver disabled by default for being experimental
DEBUG_GET_ONCE_BOOL_OPTION(enable_qwerty, "QWERTY_ENABLE", false)


/*
 *
 * Helper functions.
 *
 */

static const char *driver_list[] = {
    "qwerty",
};


/*
 *
 * Member functions.
 *
 */

static xrt_result_t
qwerty_estimate_system(struct xrt_builder *xb,
                       cJSON *config,
                       struct xrt_prober *xp,
                       struct xrt_builder_estimate *estimate)
{
	if (!debug_get_bool_option_enable_qwerty()) {
		return XRT_SUCCESS;
	}

	estimate->certain.head = true;
	estimate->certain.left = true;
	estimate->certain.right = true;
	estimate->priority = -25;

	return XRT_SUCCESS;
}

#ifdef XRT_BUILD_DRIVER_SOLARXR
static void
destroy_system_devices_solarxr(struct xrt_system_devices *xsysd)
{
	solarxr_device_set_feeder_devices(xsysd->static_roles.body, NULL, 0);
	u_system_devices_close(xsysd);
	free(xsysd);
}
#endif

static xrt_result_t
qwerty_open_system_impl(struct xrt_builder *xb,
                        cJSON *config,
                        struct xrt_prober *xp,
                        struct xrt_tracking_origin *origin,
                        struct xrt_system_devices *xsysd,
                        struct xrt_frame_context *xfctx,
                        struct u_builder_roles_helper *ubrh)
{
	struct xrt_device *head = NULL;
	struct xrt_device *left = NULL;
	struct xrt_device *right = NULL;
	enum u_logging_level log_level = debug_get_log_option_qwerty_log();

	xrt_result_t xret = qwerty_create_devices(log_level, &head, &left, &right);
	if (xret != XRT_SUCCESS) {
		return xret;
	}

	// Add to device list.
	xsysd->xdevs[xsysd->xdev_count++] = head;
	if (left != NULL) {
		xsysd->xdevs[xsysd->xdev_count++] = left;
	}
	if (right != NULL) {
		xsysd->xdevs[xsysd->xdev_count++] = right;
	}

	// Assign to role(s).
	ubrh->head = head;
	ubrh->left = left;
	ubrh->right = right;

#ifdef XRT_BUILD_DRIVER_SOLARXR
	const uint32_t count = solarxr_device_create_xdevs(head->tracking_origin, &xsysd->xdevs[xsysd->xdev_count],
	                                                   ARRAY_SIZE(xsysd->xdevs) - xsysd->xdev_count);
	if (count != 0) {
		xsysd->static_roles.body = xsysd->xdevs[xsysd->xdev_count];
		solarxr_device_set_feeder_devices(xsysd->static_roles.body, xsysd->xdevs, xsysd->xdev_count);
		xsysd->destroy = destroy_system_devices_solarxr;
	}
	xsysd->xdev_count += count;
#endif

	return XRT_SUCCESS;
}

static void
qwerty_destroy(struct xrt_builder *xb)
{
	free(xb);
}


/*
 *
 * 'Exported' functions.
 *
 */

struct xrt_builder *
t_builder_qwerty_create(void)
{
	struct u_builder *ub = U_TYPED_CALLOC(struct u_builder);

	// xrt_builder fields.
	ub->base.estimate_system = qwerty_estimate_system;
	ub->base.open_system = u_builder_open_system_static_roles;
	ub->base.destroy = qwerty_destroy;
	ub->base.identifier = "qwerty";
	ub->base.name = "Qwerty devices builder";
	ub->base.driver_identifiers = driver_list;
	ub->base.driver_identifier_count = ARRAY_SIZE(driver_list);
	ub->base.exclude_from_automatic_discovery = !debug_get_bool_option_enable_qwerty();

	// u_builder fields.
	ub->open_system_static_roles = qwerty_open_system_impl;

	return &ub->base;
}
